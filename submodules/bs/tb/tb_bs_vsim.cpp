// =============================================================================
// easy_simt · bs 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/bs/rtl/bs.sv 编译为 C++ 模型（Vbs）；
// 本 harness 驱动时钟/复位/消费者决策，参考侧直接链接 top/cmodel
// （bs_step），记分板按协议语义逐笔比对（顺序 + 载荷位精确）。
//
// 波形：Verilator 原生 VCD（<mod>.vcd），gtkwave 查看。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vbs.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链） ----------------
static sim_t ref;

static unsigned int seed_g;
static int rnd(void)
{
    seed_g = seed_g * 1103515245u + 12345u;
    return (seed_g >> 16) & 32767;
}

// ---------------- 记分板 ----------------
struct LdT { unsigned idx, n, shb; };
static LdT q_ld[4096]; static int q_ld_h, q_ld_t;

static int errors, total_errors, cyc;
static vluint64_t gtime;   // VCD 单调时基
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- 响应器/镜像状态 ----------------
static int ld_mode, ld_pct, ld_on, ld_off;
static int bd_lo, bd_hi;
static int ld_pat_cnt;
static int ref_inflight, ref_done;
static int dut_launch_cnt, dut_done, dut_done_cyc, ref_done_cyc;
static int bd_armed, bd_cnt, bd_active, bd_fire_last;
static unsigned cur_block_idx;

static int rdy_dec(int mode, int pct, int on, int off, int *pat)
{
    switch (mode) {
    case 0: return 1;
    case 1: return (rnd() % 100) < pct;
    case 2: {
        int r = *pat < on;
        *pat = *pat + 1;
        if (*pat == on + off) *pat = 0;
        return r;
    }
    default: return 1;
    }
}

// ---------------- 参考推进一拍 ----------------
static void ref_cycle(int ld_rdy, int bd_vld, unsigned bd_idx,
                      int *r_ld, unsigned *li, unsigned *ln, unsigned *ls,
                      int *r_bd)
{
    if (bd_vld && ref.bs.inflight && !ref.ws_bs_bdone.vld) {
        ref.ws.active = 0;    /* tb 扮 ws：全 warp 已完成，前端侧先行交回
                                 （与 cmodel 一致：active 在最后一个 RET 当拍
                                 清零，早于 bdone；bs_step 借此不再重置
                                 inflight） */
        ref.ws_bs_bdone.vld = 1;
        ref.ws_bs_bdone.p.block_idx = bd_idx;
    }
    int had = ref.ws_bs_bdone.vld;
    bs_step(&ref);
    *r_bd = had && !ref.ws_bs_bdone.vld;
    if (*r_bd)
        ref.ws.launched = 0;  /* tb 扮 ws：调度侧在 bdone 消费后交回 */

    *r_ld = 0;
    if (ref.bs_ws_launch.vld && ld_rdy) {
        *r_ld = 1;
        *li = ref.bs_ws_launch.p.block_idx;
        *ln = ref.bs_ws_launch.p.n;
        *ls = ref.bs_ws_launch.p.shbase;
        ref.bs_ws_launch.vld = 0;
        ref.ws.launched = 1;    /* tb 扮 ws：接受块启动 */
        ref.ws.active = 1;
    }
    ref_inflight = ref.bs.inflight;
    if (ref.bs.done) ref_done = 1;
}

// ---------------- 单个用例 ----------------
static void run_test(Vbs *top, VerilatedVcdC *tfp, const char *name, int n,
                     int ldm, int ldp, int ldo, int ldf,
                     int blo, int bhi, unsigned seed0)
{
    ld_mode = ldm; ld_pct = ldp; ld_on = ldo; ld_off = ldf;
    bd_lo = blo; bd_hi = bhi; seed_g = seed0;
    errors = 0;
    ld_pat_cnt = 0;
    q_ld_h = q_ld_t = 0;
    ref_inflight = ref_done = 0;
    dut_launch_cnt = dut_done = dut_done_cyc = ref_done_cyc = 0;
    bd_armed = bd_cnt = bd_active = bd_fire_last = 0;
    cur_block_idx = 0; cyc = 0;

    memset(&ref, 0, sizeof ref);
    ref.n = n;
    ref.grid = (n + 31) / 32;
    ref.bs.started = 1;

    top->rst_n = 0;
    top->bs_cfg_n = n;
    top->ws_bs_launch_rdy = 0;
    top->ws_bs_bdone_vld = 0;
    top->ws_bs_bdone_block_idx = 0;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 5; i++) {
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    }
    top->rst_n = 1;

    int cap = 400 * ((n + 31) / 32) + 2000;
    int finished = 0;
    while (!finished && cyc < cap) {
        // -- 响应器决策 --
        int ld_rdy = rdy_dec(ld_mode, ld_pct, ld_on, ld_off, &ld_pat_cnt);

        // -- block_done 注入门控 --
        if (bd_fire_last) { bd_active = 0; bd_fire_last = 0; }
        if (!bd_active && !bd_armed && dut_launch_cnt == 1 && ref_inflight == 1) {
            bd_armed = 1;
            bd_cnt = bd_lo + (bd_hi > bd_lo ? rnd() % (bd_hi - bd_lo + 1) : 0);
        }
        if (bd_armed) {
            if (bd_cnt == 0) { bd_active = 1; bd_armed = 0; }
            else bd_cnt--;
        }
        int bd_vld = bd_active;

        // -- 驱动 DUT 输入 --
        top->ws_bs_launch_rdy = ld_rdy;
        top->ws_bs_bdone_vld = bd_vld;
        top->ws_bs_bdone_block_idx = cur_block_idx;
        top->eval();

        // -- 本拍末沿将发生的 DUT 发射（vld 为寄存器输出，边沿前稳定） --
        int d_ld = top->bs_ws_launch_vld && ld_rdy;
        unsigned d_li = top->bs_ws_launch_block_idx;
        unsigned d_ln = top->bs_ws_launch_n;
        unsigned d_ls = top->bs_ws_launch_shbase;
        int d_bd = top->ws_bs_bdone_vld && top->bs_ws_bdone_rdy;
        if (top->ws_bs_bdone_vld && !top->bs_ws_bdone_rdy)
            err("bs_ws_bdone_rdy not asserted while block_done pending");

        // -- 参考同拍推进 --
        int r_ld, r_bd;
        unsigned r_li, r_ln, r_ls;
        ref_cycle(ld_rdy, bd_vld, cur_block_idx,
                  &r_ld, &r_li, &r_ln, &r_ls, &r_bd);

        // -- 逐笔比对 --
        if (r_ld) { q_ld[q_ld_t % 4096] = (LdT){r_li, r_ln, r_ls}; q_ld_t++; }
        if (d_ld) {
            if (q_ld_h >= q_ld_t) err("DUT ws launch without ref transaction");
            else {
                LdT e = q_ld[q_ld_h % 4096]; q_ld_h++;
                if (e.idx != d_li || e.n != d_ln || e.shb != d_ls)
                    err("bs_ws_launch payload mismatch");
                cur_block_idx = d_li;
                dut_launch_cnt++;
            }
        }
        if (d_bd) {
            if (top->ws_bs_bdone_block_idx != cur_block_idx)
                err("ws_bs_bdone payload mismatch");
            if (dut_launch_cnt != 1)
                err("block_done consumed without launch fired");
            dut_launch_cnt = 0;
            bd_fire_last = 1;
        }
        if (r_bd && ref_done && ref_done_cyc == 0) ref_done_cyc = cyc;
        if (top->bs_top_done && !dut_done) { dut_done = 1; dut_done_cyc = cyc; }
        if (top->bs_top_done && !ref_done)
            err("bs_top_done asserted before ref done");

        // -- 时钟上升沿 --
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (ref_done && dut_done && q_ld_h == q_ld_t)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (!ref_done) err("ref did not reach done");
    if (!dut_done) err("DUT did not reach bs_top_done");
    if (q_ld_h != q_ld_t) err("launch scoreboard not drained");

    int exp_blocks = (n + 31) / 32;
    total_errors += errors;
    printf("[vsim] %-10s n=%-5d grid=%-4d cyc=%-7d -> %s\n",
           name, n, exp_blocks, cyc, errors ? "FAIL" : "PASS");
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vbs *top = new Vbs;
    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("bs.vcd");

    run_test(top, tfp, "golden", 1000, 0,100,0,0, 0, 3, 1);
    run_test(top, tfp, "single", 1, 0,100,0,0, 0, 0, 2);
    run_test(top, tfp, "exact2", 64, 0,100,0,0, 0, 2, 3);
    run_test(top, tfp, "b2b", 3200, 0,100,0,0, 0, 0, 4);
    run_test(top, tfp, "rand50", 1000, 1, 50,0,0, 0, 8, 5);
    run_test(top, tfp, "rand20", 1000, 1, 20,0,0, 2,20, 6);
    run_test(top, tfp, "pat", 1000, 2,100,3,11, 0, 5, 7);
    run_test(top, tfp, "ld_slow", 1000, 2,100,1,15, 0, 4, 8);
    run_test(top, tfp, "bd_slow", 500, 0,100,0,0, 30,60, 10);
    run_test(top, tfp, "soak", 2000, 1, 60,0,0, 1,12, 11);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
