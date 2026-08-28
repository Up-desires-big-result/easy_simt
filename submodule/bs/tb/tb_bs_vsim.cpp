// =============================================================================
// easy_simt · bs 的 Verilator harness（开源仿真路线，独立于 VCS）
//
// 结构：Verilator 把 submodule/bs/rtl/bs.sv 编译为 C++ 模型（Vbs）；
// 本 harness 驱动时钟/复位/消费者决策，参考侧直接链接 top/cmodel
// （bs_step，不经 DPI），记分板逻辑与 SV testbench（tb_bs.sv）同构：
// 同一组决策同时驱动 DUT 与参考，事务逐笔比对（顺序 + 载荷位精确）。
//
// 波形：Verilator 原生 VCD（<mod>.vcd，落 tmp/vsim/<mod>/），gtkwave 查看。
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
struct SfT { unsigned idx, n, shb; };
struct WsT { unsigned idx; };
static SfT q_sf[4096]; static int q_sf_h, q_sf_t;
static WsT q_ws[4096]; static int q_ws_h, q_ws_t;

static int errors, total_errors, cyc;
static vluint64_t gtime;   // VCD 单调时基
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- 响应器/镜像状态（与 tb_bs.sv 同构） ----------------
static int sf_mode, sf_pct, sf_on, sf_off;
static int ws_mode, ws_pct, ws_on, ws_off;
static int bd_lo, bd_hi;
static int sf_pat_cnt, ws_pat_cnt;
static int sf_active_m, ws_launched_m, ref_inflight, ref_done;
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

// ---------------- 参考推进一拍（镜像 dpi_ref.c 语义） ----------------
static void ref_cycle(int sf_rdy, int ws_rdy, int bd_vld, unsigned bd_idx,
                      int *r_sf, unsigned *si, unsigned *sn, unsigned *ss,
                      int *r_ws, unsigned *wi, int *r_bd)
{
    if (bd_vld && ref.bs.inflight && !ref.ws_bs_bdone.vld) {
        ref.sf.active = 0;
        ref.ws_bs_bdone.vld = 1;
        ref.ws_bs_bdone.p.block_idx = bd_idx;
    }
    int had = ref.ws_bs_bdone.vld;
    bs_step(&ref);
    *r_bd = had && !ref.ws_bs_bdone.vld;
    if (*r_bd)
        ref.ws.launched = 0;

    *r_sf = *r_ws = 0;
    if (ref.bs_sf_launch.vld && sf_rdy) {
        *r_sf = 1;
        *si = ref.bs_sf_launch.p.block_idx;
        *sn = ref.bs_sf_launch.p.n;
        *ss = ref.bs_sf_launch.p.shbase;
        ref.bs_sf_launch.vld = 0;
        ref.sf.active = 1;
    }
    if (ref.bs_ws_launch.vld && ws_rdy) {
        *r_ws = 1;
        *wi = ref.bs_ws_launch.p.block_idx;
        ref.bs_ws_launch.vld = 0;
        ref.ws.launched = 1;
    }
    ref_inflight = ref.bs.inflight;
    if (ref.bs.done) ref_done = 1;
}

// ---------------- 单个用例 ----------------
static void run_test(Vbs *top, VerilatedVcdC *tfp, const char *name, int n,
                     int sfm, int sfp, int sfo, int sff,
                     int wsm, int wsp, int wso, int wsf,
                     int blo, int bhi, unsigned seed0)
{
    sf_mode = sfm; sf_pct = sfp; sf_on = sfo; sf_off = sff;
    ws_mode = wsm; ws_pct = wsp; ws_on = wso; ws_off = wsf;
    bd_lo = blo; bd_hi = bhi; seed_g = seed0;
    errors = 0;
    sf_pat_cnt = ws_pat_cnt = 0;
    q_sf_h = q_sf_t = q_ws_h = q_ws_t = 0;
    sf_active_m = ws_launched_m = ref_inflight = ref_done = 0;
    dut_launch_cnt = dut_done = dut_done_cyc = ref_done_cyc = 0;
    bd_armed = bd_cnt = bd_active = bd_fire_last = 0;
    cur_block_idx = 0; cyc = 0;

    memset(&ref, 0, sizeof ref);
    ref.n = n;
    ref.grid = (n + 31) / 32;
    ref.bs.started = 1;

    top->rst_n = 0;
    top->bs_cfg_n = n;
    top->sf_bs_launch_rdy = 0;
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
        int sf_rdy = rdy_dec(sf_mode, sf_pct, sf_on, sf_off, &sf_pat_cnt);
        int ws_rdy = rdy_dec(ws_mode, ws_pct, ws_on, ws_off, &ws_pat_cnt);

        // -- block_done 注入门控 --
        if (bd_fire_last) { bd_active = 0; bd_fire_last = 0; }
        if (!bd_active && !bd_armed && dut_launch_cnt == 2 && ref_inflight == 1) {
            bd_armed = 1;
            bd_cnt = bd_lo + (bd_hi > bd_lo ? rnd() % (bd_hi - bd_lo + 1) : 0);
        }
        if (bd_armed) {
            if (bd_cnt == 0) { bd_active = 1; bd_armed = 0; }
            else bd_cnt--;
        }
        int bd_vld = bd_active;

        // -- 驱动 DUT 输入 --
        top->sf_bs_launch_rdy = sf_rdy;
        top->ws_bs_launch_rdy = ws_rdy;
        top->ws_bs_bdone_vld = bd_vld;
        top->ws_bs_bdone_block_idx = cur_block_idx;
        top->eval();

        // -- 本拍末沿将发生的 DUT 发射（vld 为寄存器输出，边沿前稳定） --
        int d_sf = top->bs_sf_launch_vld && sf_rdy;
        unsigned d_si = top->bs_sf_launch_block_idx;
        unsigned d_sn = top->bs_sf_launch_n;
        unsigned d_ss = top->bs_sf_launch_shbase;
        int d_ws = top->bs_ws_launch_vld && ws_rdy;
        unsigned d_wi = top->bs_ws_launch_block_idx;
        int d_bd = top->ws_bs_bdone_vld && top->bs_ws_bdone_rdy;
        if (top->ws_bs_bdone_vld && !top->bs_ws_bdone_rdy)
            err("bs_ws_bdone_rdy not asserted while block_done pending");

        // -- 参考同拍推进 --
        int r_sf, r_ws, r_bd;
        unsigned r_si, r_sn, r_ss, r_wi;
        ref_cycle(sf_rdy, ws_rdy, bd_vld, cur_block_idx,
                  &r_sf, &r_si, &r_sn, &r_ss, &r_ws, &r_wi, &r_bd);

        // -- 逐笔比对 --
        if (r_sf) { q_sf[q_sf_t % 4096] = (SfT){r_si, r_sn, r_ss}; q_sf_t++; }
        if (r_ws) { q_ws[q_ws_t % 4096] = (WsT){r_wi}; q_ws_t++; }
        if (d_sf) {
            if (q_sf_h >= q_sf_t) err("DUT sf launch without ref transaction");
            else {
                SfT e = q_sf[q_sf_h % 4096]; q_sf_h++;
                if (e.idx != d_si || e.n != d_sn || e.shb != d_ss)
                    err("bs_sf_launch payload mismatch");
                dut_launch_cnt++;
            }
        }
        if (d_ws) {
            if (q_ws_h >= q_ws_t) err("DUT ws launch without ref transaction");
            else {
                WsT e = q_ws[q_ws_h % 4096]; q_ws_h++;
                if (e.idx != d_wi) err("bs_ws_launch payload mismatch");
                cur_block_idx = d_wi;
                dut_launch_cnt++;
            }
        }
        if (d_bd) {
            if (top->ws_bs_bdone_block_idx != cur_block_idx)
                err("ws_bs_bdone payload mismatch");
            if (dut_launch_cnt != 2)
                err("block_done consumed without both launches fired");
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

        if (ref_done && dut_done && q_sf_h == q_sf_t && q_ws_h == q_ws_t)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (!ref_done) err("ref did not reach done");
    if (!dut_done) err("DUT did not reach bs_top_done");
    if (q_sf_h != q_sf_t) err("sf scoreboard not drained");
    if (q_ws_h != q_ws_t) err("ws scoreboard not drained");

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

    run_test(top, tfp, "golden", 1000, 0,100,0,0, 0,100,0,0, 0, 3, 1);
    run_test(top, tfp, "single", 1, 0,100,0,0, 0,100,0,0, 0, 0, 2);
    run_test(top, tfp, "exact2", 64, 0,100,0,0, 0,100,0,0, 0, 2, 3);
    run_test(top, tfp, "b2b", 3200, 0,100,0,0, 0,100,0,0, 0, 0, 4);
    run_test(top, tfp, "rand50", 1000, 1, 50,0,0, 1, 50,0,0, 0, 8, 5);
    run_test(top, tfp, "rand20", 1000, 1, 20,0,0, 1, 70,0,0, 2,20, 6);
    run_test(top, tfp, "pat", 1000, 2,100,3,11, 2,100,5,7, 0, 5, 7);
    run_test(top, tfp, "sf_slow", 1000, 2,100,1,15, 0,100,0,0, 0, 4, 8);
    run_test(top, tfp, "ws_slow", 1000, 0,100,0,0, 2,100,1,15, 1, 6, 9);
    run_test(top, tfp, "bd_slow", 500, 0,100,0,0, 0,100,0,0, 30,60, 10);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
