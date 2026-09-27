// =============================================================================
// easy_simt · ws 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/ws/rtl/ws.sv 编译为 C++ 模型（Vws）；
// 本 harness 驱动时钟/复位/周边行为，参考侧直接链接 top/cmodel
// （ws_step），事务级逐笔比对（边界事务序列一致：cmodel 逐步推进、
// RTL 每事件一拍，周期不比对）。
//
// tb 扮全部周边（ws_spec §10）：
//   icache —— 指令存储（程序镜像由激励序列隐式给出：pc→inst 映射），
//             取指请求受理后按 IFETCH_LAT 倒计时回送；
//   rf     —— 寄存器堆行为模型（warp,reg,lane 寻址），读请求受理后按
//             RD_LAT 倒计时回送双字；
//   ialu   —— SETP 谓词求值 + BR 决议（按 tb 预置的谓词真值表产生
//             taken 向量，目标自发射载荷 imm 解出）+ wbdone；
//   falu   —— issue 消费 + wbdone（随机延迟）；
//   lsu    —— issue 消费 + wbdone（随机延迟）+ 停顿上报（LMISS 随发射
//             握手当拍、R_NONE 于空闲拍，镜像 cmodel lsu 口径）；
//   bs     —— launch 序列（块启动）+ bdone 消费。
//
// 程序镜像：激励构造器按 warp 无关的 pc→32b 指令字映射生成（各 warp
// 执行同一段程序的不同谓词路径）；BRT 表由 tb 经 ws_cfg_brt_* 装载。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <map>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vws.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链） ----------------
static sim_t ref;

// ---------------- 随机源 ----------------
static unsigned int seed_g;
static int rnd(void)
{
    seed_g = seed_g * 1103515245u + 12345u;
    return (seed_g >> 16) & 32767;
}
static uint32_t rnd32(void)
{
    return ((uint32_t)rnd() << 17) ^ ((uint32_t)rnd() << 6) ^ (uint32_t)rnd();
}

// ---------------- 参数（与 cmodel 一致） ----------------
#define NBLK_MAX 4

// ---------------- 指令编码辅助（isa_spec §1.8） ----------------
static uint32_t enc_r3(int op, int rd, int ra, int rb, int rc)
{
    return ((uint32_t)op << 26) | ((uint32_t)rd << 21) | ((uint32_t)ra << 16)
         | ((uint32_t)rb << 11) | ((uint32_t)rc << 6);
}
static uint32_t enc_rr(int op, int rd, int ra, int rb)
{
    return ((uint32_t)op << 26) | ((uint32_t)rd << 21) | ((uint32_t)ra << 16)
         | ((uint32_t)rb << 11);
}
static uint32_t enc_imm(int op, int rd, int ra, uint32_t imm16)
{
    return ((uint32_t)op << 26) | ((uint32_t)rd << 21) | ((uint32_t)ra << 16)
         | (imm16 & 0xFFFFu);
}
static uint32_t enc_lui(int rd, uint32_t hi20)
{
    return (0x06u << 26) | ((uint32_t)rd << 21) | ((hi20 & 0xFFFFFu) << 1);
}
static uint32_t enc_setp(int pd, int ra, int rb, int fmt, int cond)
{
    return (0x07u << 26) | ((uint32_t)pd << 24) | ((uint32_t)ra << 18)
         | ((uint32_t)rb << 13) | ((uint32_t)fmt << 12)
         | ((uint32_t)cond << 9);
}
static uint32_t enc_br(int psel, int u, int neg, int32_t off)
{
    return (0x11u << 26) | ((uint32_t)psel << 24) | ((uint32_t)u << 23)
         | ((uint32_t)neg << 22) | ((uint32_t)off & 0x3FFFFFu);
}
static uint32_t enc_join(int32_t off)
{
    return (0x12u << 26) | ((uint32_t)off & 0x3FFFFFu);
}
static uint32_t enc_bar(void) { return 0x13u << 26; }
static uint32_t enc_ret(void) { return 0x14u << 26; }

// ---------------- 程序镜像（pc → 指令字）与 BRT ----------------
static std::map<uint32_t, uint32_t> prog;
struct BrtEntry { uint32_t pc, rpc; int valid; };
static BrtEntry brt[8];
static int brt_n;

static void prog_clear(void)
{
    prog.clear();
    for (int i = 0; i < 8; i++) brt[i].valid = 0;
    brt_n = 0;
}
static void put(uint32_t pc, uint32_t inst) { prog[pc] = inst; }
static void brt_put(uint32_t pc, uint32_t rpc)
{
    for (int i = 0; i < 8; i++)
        if (!brt[i].valid) {
            brt[i].pc = pc; brt[i].rpc = rpc; brt[i].valid = 1;
            brt_n++;
            return;
        }
}

// ---------------- ialu 谓词真值表（BR 决议用：pc → taken 向量） ----------------
// 由激励构造器预置：分化分支给 mixed 向量，均匀分支给 0x00/0xFF
static std::map<uint32_t, uint8_t> br_taken;
static void brput(uint32_t pc, uint8_t t) { br_taken[pc] = t; }

// ---------------- 记分板：期望事务队列 ----------------
struct EvT {
    int type;          // 1=ireq 2=irsp 3=rrd 4=rrsp 5=issA 6=issF 7=issL
                       // 8=br 9=wbd 10=stall 11=bdone
    uint32_t a, b;     // 载荷（pc/inst/rs/imm/warp/rd/...）
    uint8_t vec[8];    // mask / taken
    uint32_t opa[8], opb[8], opc[8];
};
static EvT eq[65536];
static int eq_h, eq_t;

static int errors, total_errors, cyc;
static vluint64_t gtime;
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- tb 周边状态 ----------------
// icache
static int ifetch_lat;              // 取指回送延迟（倒计时口径）
static int ic_occupied, ic_cnt;
static uint32_t ic_pc;
// rf
static int rd_lat;
static int rf_occupied, rf_cnt;
static uint32_t rf_r1, rf_r2;
static int rf_r1v, rf_r2v;
// 执行单元 wbdone 延迟（issue 消费后倒计数）
static int eu_lat;
static int eu_busy, eu_cnt;         // 执行单元 wbdone/BR 决议倒计时
static int eu_done;                  // eu 呈现中（DUT 消费才释放）
static int eu_which;                // 1=falu 2=lsu
// lsu 停顿上报镜像（cmodel 口径：LMISS 随发射握手当拍、R_NONE 空闲拍）
static int lmiss_pend;              // 待置位（issue 握手后）
static uint32_t lmiss_warp;
static int rnone_pend;              // 待补发 R_NONE
static uint32_t rnone_warp;
// 背压模式
static int rdy_mode, rdy_pct, rdy_on, rdy_off, rdy_pat;

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

// ---------------- 期望事务压队（镜像 cmodel 各事件） ----------------
static void ref_pump_events(void)
{
    /* 每拍将参考模型推进到静默：消费其全部输出事务并压队 */
    for (int it = 0; it < 256; it++) {
        int acted = 0;
        /* bs：launch 序列由主循环驱动（参考侧同 DUT 侧输入） */
        /* 周边从设备：受理参考输出请求 */
        if (ref.ws_icache_req.vld) {
            EvT e; memset(&e, 0, sizeof e);
            e.type = 1; e.a = ref.ws_icache_req.p.pc;
            eq[eq_t++ & 65535] = e;
            ref.ws_icache_req.vld = 0;
            acted = 1;
        }
        if (ref.ws_rf_rd.vld) {
            EvT e; memset(&e, 0, sizeof e);
            e.type = 3; e.a = ref.ws_rf_rd.p.rs1; e.b = ref.ws_rf_rd.p.rs2;
            e.vec[0] = (uint8_t)ref.ws_rf_rd.p.warp_id;
            eq[eq_t++ & 65535] = e;
            ref.ws_rf_rd.vld = 0;
            acted = 1;
        }
        if (ref.ws_ialu_issue.vld) {
            EvT e; memset(&e, 0, sizeof e);
            e.type = 5; e.a = (uint32_t)ref.ws_ialu_issue.p.opcode;
            e.b = (uint32_t)ref.ws_ialu_issue.p.rd;
            e.vec[0] = (uint8_t)ref.ws_ialu_issue.p.warp_id;
            e.vec[1] = ref.ws_ialu_issue.p.lane_mask;
            eq[eq_t++ & 65535] = e;
            ref.ws_ialu_issue.vld = 0;
            acted = 1;
        }
        if (ref.ws_falu_issue.vld) {
            EvT e; memset(&e, 0, sizeof e);
            e.type = 6; e.a = (uint32_t)ref.ws_falu_issue.p.opcode;
            e.b = (uint32_t)ref.ws_falu_issue.p.rd;
            e.vec[0] = (uint8_t)ref.ws_falu_issue.p.warp_id;
            e.vec[1] = ref.ws_falu_issue.p.lane_mask;
            eq[eq_t++ & 65535] = e;
            ref.ws_falu_issue.vld = 0;
            acted = 1;
        }
        if (ref.ws_lsu_issue.vld) {
            EvT e; memset(&e, 0, sizeof e);
            e.type = 7; e.a = (uint32_t)ref.ws_lsu_issue.p.opcode;
            e.b = (uint32_t)ref.ws_lsu_issue.p.rd;
            e.vec[0] = (uint8_t)ref.ws_lsu_issue.p.warp_id;
            e.vec[1] = ref.ws_lsu_issue.p.lane_mask;
            eq[eq_t++ & 65535] = e;
            ref.ws_lsu_issue.vld = 0;
            acted = 1;
        }
        if (ref.ws_bs_bdone.vld) {
            EvT e; memset(&e, 0, sizeof e);
            e.type = 11; e.a = ref.ws_bs_bdone.p.block_idx;
            eq[eq_t++ & 65535] = e;
            ref.ws_bs_bdone.vld = 0;
            acted = 1;
        }
        if (ref.lsu_ws_stall.vld) {
            ref.lsu_ws_stall.vld = 0;   // ws 恒收（提示性）
            acted = 1;
        }
        /* 周边主动呈现（零延迟泵）：icache 回送 / rf 回送 / BR 决议 /
         * wbdone——先于 ws_step 让 ws 能当拍消费（镜像 cmodel 顺序：
         * 输入事务在 step 开头处理）。注入内容由主循环在 DUT 侧受理
         * 事件时排入 pending 队列（pump_in_*），保证两侧同序。 */
        if (lmiss_pend && !ref.lsu_ws_stall.vld) {
            ref.lsu_ws_stall.p.warp_id = lmiss_warp;
            ref.lsu_ws_stall.p.reason = R_LMISS;
            ref.lsu_ws_stall.vld = 1;
            lmiss_pend = 0;
            acted = 1;
        }
        if (rnone_pend && !ref.lsu_ws_stall.vld) {
            ref.lsu_ws_stall.p.warp_id = rnone_warp;
            ref.lsu_ws_stall.p.reason = R_NONE;
            ref.lsu_ws_stall.vld = 1;
            rnone_pend = 0;
            acted = 1;
        }
        /* ws_step 推进一步（其内部消费输入并产生输出） */
        if (ws_step(&ref))
            acted = 1;
        if (!acted)
            break;
    }
}

// ---------------- 参考侧输入注入 pending（主循环 DUT 受理事件时排入） ----------------
static int    pin_irsp;              // icache 回送待注入
static uint32_t pin_irsp_inst;
static int    pin_rrsp;              // rf 回送待注入
static uint32_t pin_rrsp_a[8], pin_rrsp_b[8];
static int    pin_br;                // BR 决议待注入
static uint32_t pin_br_w, pin_br_target;
static uint8_t  pin_br_taken;
static int    pin_wbd;               // wbdone 待注入（eu_which 定源）
static uint32_t pin_wbd_w, pin_wbd_rd;

// ---------------- rf 行为模型（warp,reg,lane 寻址） ----------------
static uint32_t rfmem[NWARPS][32][NLANES];
static uint32_t rf_read(uint32_t w, uint32_t r, uint32_t l)
{
    return r ? rfmem[w][r][l] : 0;
}

// ---------------- DUT 输出比对 ----------------
static void cmp_pop(int type, const char *what)
{
    if (eq_h >= eq_t) {
        err("DUT event without ref transaction");
        return;
    }
    EvT e = eq[eq_h++ & 65535];
    if (e.type != type) {
        err("event order mismatch");
        return;
    }
    (void)what;
}
static void cmp_pop_ireq(uint32_t pc)
{
    if (eq_h >= eq_t) { err("DUT ireq without ref"); return; }
    EvT e = eq[eq_h++ & 65535];
    if (e.type != 1 || e.a != pc) err("ireq pc mismatch");
}
static void cmp_pop_rrd(uint32_t w, uint32_t s1, uint32_t s2)
{
    if (eq_h >= eq_t) { err("DUT rrd without ref"); return; }
    EvT e = eq[eq_h++ & 65535];
    if (e.type != 3 || e.vec[0] != (uint8_t)w
        || e.a != s1 || e.b != s2) err("rrd mismatch");
}
static void cmp_pop_iss(int type, uint32_t op, uint32_t rd, uint32_t w,
                        uint8_t m)
{
    if (eq_h >= eq_t) { err("DUT issue without ref"); return; }
    EvT e = eq[eq_h++ & 65535];
    if (e.type != type || e.a != op || e.b != rd
        || e.vec[0] != (uint8_t)w || e.vec[1] != m) err("issue mismatch");
}
static void cmp_pop_bdone(uint32_t idx)
{
    if (eq_h >= eq_t) { err("DUT bdone without ref"); return; }
    EvT e = eq[eq_h++ & 65535];
    if (e.type != 11 || e.a != idx) err("bdone mismatch");
}

// ---------------- 单个用例 ----------------
static int cur_blk, nblks;           // bs launch 序列
static int dut_bdone_cnt;

static void run_test(Vws *top, VerilatedVcdC *tfp, const char *name,
                     int rm, int rp, int ro, int rf_,
                     int ilat, int rlat, int elat, unsigned seed0)
{
    rdy_mode = rm; rdy_pct = rp; rdy_on = ro; rdy_off = rf_;
    ifetch_lat = ilat; rd_lat = rlat; eu_lat = elat;
    seed_g = seed0;
    errors = 0;
    eq_h = eq_t = 0;
    ic_occupied = ic_cnt = 0;
    rf_occupied = rf_cnt = 0;
    rf_r1v = rf_r2v = 0;
    eu_busy = eu_cnt = 0;
    eu_done = 0;
    lmiss_pend = rnone_pend = 0;
    pin_irsp = pin_rrsp = pin_br = pin_wbd = 0;
    cur_blk = 0; nblks = 1; dut_bdone_cnt = 0;
    cyc = 0;

    memset(&ref, 0, sizeof ref);
    ref.params[0] = 0x00100000;
    ref.params[1] = 0x00200000;
    ref.params[2] = 1000;
    for (int w = 0; w < NWARPS; w++)
        for (int r = 0; r < 32; r++)
            for (int l = 0; l < NLANES; l++)
                rfmem[w][r][l] = rnd32();
    /* 参考侧 ws 初始态（镜像 cmodel sim_init） */
    ref.ws.fetch_warp = -1;
    ref.ws.rd_warp = -1;

    // ---- 复位（含 BRT 装载，isa_spec §1.11：启动前完成） ----
    top->rst_n = 0;
    top->ws_cfg_param0 = ref.params[0];
    top->ws_cfg_param1 = ref.params[1];
    top->ws_cfg_param2 = ref.params[2];
    top->ws_cfg_brt_we = 0;
    top->bs_ws_launch_vld = 0;
    top->bs_ws_launch_block_idx = 0;
    top->bs_ws_launch_n = 1000;
    top->bs_ws_launch_shbase = 0;
    top->bs_ws_bdone_rdy = 0;
    top->lsu_ws_stall_vld = 0;
    top->icache_ws_req_rdy = 0;
    top->icache_ws_rsp_vld = 0;
    top->icache_ws_rsp_inst = 0;
    top->rf_ws_rd_rdy = 0;
    top->rf_ws_rddata_vld = 0;
    for (int l = 0; l < NLANES; l++) {
        top->rf_ws_rddata_a[l] = 0;
        top->rf_ws_rddata_b[l] = 0;
    }
    top->ialu_ws_issue_rdy = 0;
    top->falu_ws_issue_rdy = 0;
    top->lsu_ws_issue_rdy = 0;
    top->ialu_ws_br_vld = 0;
    top->ialu_ws_br_warp_id = 0;
    top->ialu_ws_br_taken = 0;
    top->ialu_ws_br_target = 0;
    top->ialu_ws_br_brt_idx = 0;
    top->ialu_ws_wbdone_vld = 0;
    top->falu_ws_wbdone_vld = 0;
    top->lsu_ws_wbdone_vld = 0;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 5; i++) {
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    }
    /* BRT 装载（复位释放后、launch 前，isa_spec §1.11：启动前完成） */
    top->rst_n = 1;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 8; i++)
        if (brt[i].valid) {
            top->ws_cfg_brt_we = 1;
            top->ws_cfg_brt_idx = i;
            top->ws_cfg_brt_wpc = brt[i].pc;
            top->ws_cfg_brt_wrpc = brt[i].rpc;
            top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
            top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        }
    top->ws_cfg_brt_we = 0;
    /* 参考侧 BRT */
    for (int i = 0; i < 8; i++)
        if (brt[i].valid) {
            ref.brt_valid[brt[i].pc] = 1;
            ref.brt_rpc[brt[i].pc] = brt[i].rpc;
        }

    int cap = 60000;
    int finished = 0;
    while (!finished && cyc < cap) {
        // -- 周边决策（背压：取指请求 / rf 读 / 三路 issue） --
        int r_i = rdy_dec(rdy_mode, rdy_pct, rdy_on, rdy_off, &rdy_pat);
        int r_r = rdy_dec(rdy_mode, rdy_pct, rdy_on, rdy_off, &rdy_pat);
        int r_a = rdy_dec(rdy_mode, rdy_pct, rdy_on, rdy_off, &rdy_pat);
        int r_f = rdy_dec(rdy_mode, rdy_pct, rdy_on, rdy_off, &rdy_pat);
        int r_l = rdy_dec(rdy_mode, rdy_pct, rdy_on, rdy_off, &rdy_pat);

        // -- bs：launch 序列（块间推进） --
        int do_launch = 0;
        if (cur_blk < nblks && !top->ws_bs_bdone_vld)
            do_launch = 1;

        // -- DUT 侧输入呈现 --
        top->bs_ws_launch_vld = do_launch;
        top->bs_ws_launch_block_idx = cur_blk;
        top->bs_ws_bdone_rdy = 1;
        top->icache_ws_req_rdy = !ic_occupied && r_i;
        top->rf_ws_rd_rdy = !rf_occupied && r_r;
        top->ialu_ws_issue_rdy = r_a;
        top->falu_ws_issue_rdy = r_f;
        top->lsu_ws_issue_rdy = r_l;
        // icache 回送呈现（倒计时到 0）
        top->icache_ws_rsp_vld = ic_occupied && ic_cnt == 0;
        if (top->icache_ws_rsp_vld)
            top->icache_ws_rsp_inst =
                prog.count(ic_pc) ? prog[ic_pc] : 0;
        // rf 回送呈现
        top->rf_ws_rddata_vld = rf_occupied && rf_cnt == 0;
        if (top->rf_ws_rddata_vld) {
            uint32_t ww = ref.ws_rf_rd.vld ? ref.ws_rf_rd.p.warp_id
                         : 0;  // 仅供调试；实际 warp 由 DUT 载荷自存
            (void)ww;
            for (int l = 0; l < NLANES; l++) {
                top->rf_ws_rddata_a[l] = rf_r1v ? rf_r1 : 0;
                top->rf_ws_rddata_b[l] = rf_r2v ? rf_r2 : 0;
            }
        }
        // BR 决议呈现（ialu：发射消费后按 eu 延迟或立即）
        top->ialu_ws_br_vld = pin_br;
        if (pin_br) {
            top->ialu_ws_br_warp_id = pin_br_w;
            top->ialu_ws_br_taken = pin_br_taken;
            top->ialu_ws_br_target = pin_br_target;
        }
        // wbdone 呈现（eu 倒计时到 0：eu_which 定源，rdy 恒 1）
        top->ialu_ws_wbdone_vld = eu_busy && eu_done && eu_which == 0;
        top->falu_ws_wbdone_vld = eu_busy && eu_done && eu_which == 1;
        top->lsu_ws_wbdone_vld  = eu_busy && eu_done && eu_which == 2;
        if (eu_busy && eu_cnt == 0) {
            top->ialu_ws_wbdone_warp_id = pin_wbd_w;
            top->ialu_ws_wbdone_rd = pin_wbd_rd;
            top->falu_ws_wbdone_warp_id = pin_wbd_w;
            top->falu_ws_wbdone_rd = pin_wbd_rd;
            top->lsu_ws_wbdone_warp_id = pin_wbd_w;
            top->lsu_ws_wbdone_rd = pin_wbd_rd;
        }
        top->eval();

        // ---- 本拍末沿将发生的 DUT 事件（vld 为寄存器输出） ----
        int d_launch = do_launch && top->ws_bs_launch_rdy;
        int d_ireq = top->ws_icache_req_vld && top->icache_ws_req_rdy;
        int d_irsp = top->icache_ws_rsp_vld;        // rdy 恒 1
        int d_rrd = top->ws_rf_rd_vld && top->rf_ws_rd_rdy;
        int d_rrsp = top->rf_ws_rddata_vld;         // rdy 恒 1
        int d_issA = top->ws_ialu_issue_vld && top->ialu_ws_issue_rdy;
        int d_issF = top->ws_falu_issue_vld && top->falu_ws_issue_rdy;
        int d_issL = top->ws_lsu_issue_vld && top->lsu_ws_issue_rdy;
        int d_br = top->ialu_ws_br_vld;             // rdy 恒 1
        int d_wbdA = top->ialu_ws_wbdone_vld;       // rdy 恒 1
        int d_wbdF = top->falu_ws_wbdone_vld;       // rdy 恒 1
        int d_wbdL = top->lsu_ws_wbdone_vld;        // rdy 恒 1
        int d_bdone = top->ws_bs_bdone_vld;         // rdy 恒 1

        // ---- 参考侧同步：同输入驱动（launch / 背压下的受理序列一致） ----
        if (d_launch) {
            ref.bs_ws_launch.p.block_idx = cur_blk;
            ref.bs_ws_launch.p.n = 1000;
            ref.bs_ws_launch.p.shbase = 0;
            ref.bs_ws_launch.vld = 1;
        }

        // ---- DUT 事件比对 + 周边状态机推进 ----
        if (d_ireq) {
            cmp_pop_ireq(top->ws_icache_req_pc);
            ic_occupied = 1;
            ic_cnt = ifetch_lat;
            ic_pc = top->ws_icache_req_pc;
        }
        if (d_irsp) {
            // DUT 侧指令回送被消费：注入参考侧
            pin_irsp = 1;
            pin_irsp_inst = prog.count(ic_pc) ? prog[ic_pc] : 0;
            ic_occupied = 0;
        }
        if (d_rrd) {
            uint32_t w = top->ws_rf_rd_warp_id;
            cmp_pop_rrd(w, top->ws_rf_rd_rs1, top->ws_rf_rd_rs2);
            rf_occupied = 1;
            rf_cnt = rd_lat;
            rf_r1 = rf_read(w, top->ws_rf_rd_rs1, 0);  // 均匀值口径（
            rf_r2 = rf_read(w, top->ws_rf_rd_rs2, 0);  //  lane 0 代表）
            rf_r1v = top->ws_rf_rd_rs1 != 0;
            rf_r2v = top->ws_rf_rd_rs2 != 0;
        }
        if (d_rrsp) {
            pin_rrsp = 1;
            for (int l = 0; l < NLANES; l++) {
                pin_rrsp_a[l] = rf_r1v ? rf_r1 : 0;
                pin_rrsp_b[l] = rf_r2v ? rf_r2 : 0;
            }
            rf_occupied = 0;
        }
        if (d_issA) {
            cmp_pop_iss(5, top->ws_ialu_issue_opcode, top->ws_ialu_issue_rd,
                        top->ws_ialu_issue_warp_id,
                        top->ws_ialu_issue_lane_mask);
            if (top->ws_ialu_issue_opcode == OP_BR) {
                // BR：按谓词真值表产生决议（延迟 eu_lat）
                eu_busy = 1; eu_cnt = eu_lat; eu_which = 3; eu_done = 0;
                pin_br_w = top->ws_ialu_issue_warp_id;
                pin_br_taken = br_taken.count(top->ws_ialu_issue_pc)
                               ? br_taken[top->ws_ialu_issue_pc] : 0;
                pin_br_target = top->ws_ialu_issue_imm & 0x3FFFFFFFu;
            } else {
                eu_busy = 1; eu_cnt = eu_lat; eu_which = 0; eu_done = 0;
                pin_wbd_w = top->ws_ialu_issue_warp_id;
                pin_wbd_rd = top->ws_ialu_issue_rd;
            }
        }
        if (d_issF) {
            cmp_pop_iss(6, top->ws_falu_issue_opcode, top->ws_falu_issue_rd,
                        top->ws_falu_issue_warp_id,
                        top->ws_falu_issue_lane_mask);
            eu_busy = 1; eu_cnt = eu_lat; eu_which = 1; eu_done = 0;
            pin_wbd_w = top->ws_falu_issue_warp_id;
            pin_wbd_rd = top->ws_falu_issue_rd;
        }
        if (d_issL) {
            cmp_pop_iss(7, top->ws_lsu_issue_opcode, top->ws_lsu_issue_rd,
                        top->ws_lsu_issue_warp_id,
                        top->ws_lsu_issue_lane_mask);
            eu_busy = 1; eu_cnt = eu_lat; eu_which = 2; eu_done = 0;
            pin_wbd_w = top->ws_lsu_issue_warp_id;
            pin_wbd_rd = top->ws_lsu_issue_rd;
            // LMISS 随发射握手当拍（cmodel lsu 口径）
            if (!lmiss_pend) {
                lmiss_pend = 1;
                lmiss_warp = top->ws_lsu_issue_warp_id;
            }
        }
        if (d_br) {
            pin_br = 0;
        }
        if (d_wbdA || d_wbdF || d_wbdL) {
            eu_busy = 0;
            // R_NONE 空闲拍补发（cmodel lsu 口径：wbdone 后空闲拍）
            if (d_wbdL && !rnone_pend) {
                rnone_pend = 1;
                rnone_warp = pin_wbd_w;
            }
        }
        if (d_bdone) {
            cmp_pop_bdone(cur_blk);
            dut_bdone_cnt++;
            cur_blk++;
        }

        // ---- 参考泵（推进至静默；注入 pin_* 输入） ----
        if (pin_irsp && !ref.icache_ws_rsp.vld) {
            ref.icache_ws_rsp.p.inst = pin_irsp_inst;
            ref.icache_ws_rsp.vld = 1;
            pin_irsp = 0;
        }
        if (pin_rrsp && !ref.rf_ws_rddata.vld) {
            for (int l = 0; l < NLANES; l++) {
                ref.rf_ws_rddata.p.a[l] = pin_rrsp_a[l];
                ref.rf_ws_rddata.p.b[l] = pin_rrsp_b[l];
            }
            ref.rf_ws_rddata.vld = 1;
            pin_rrsp = 0;
        }
        /* eu 槽（elat 倒计时到 0）：呈现 wbdone/决议至参考侧（DUT 侧
         * 呈现由输入呈现段按 eu 状态给出，持续至 DUT 消费才释放） */
        if (eu_busy && eu_cnt == 0) {
            if (!eu_done) {
                eu_done = 1;
                if (eu_which == 3) {
                    ref.ialu_ws_br.p.warp_id = pin_br_w;
                    ref.ialu_ws_br.p.taken = pin_br_taken;
                    ref.ialu_ws_br.p.target = pin_br_target;
                    ref.ialu_ws_br.p.brt_idx = 0;
                    ref.ialu_ws_br.vld = 1;
                    pin_br = 1;      // DUT 侧呈现（下一拍输入段）
                } else if (eu_which == 0) {
                    ref.ialu_ws_wbdone.p.warp_id = pin_wbd_w;
                    ref.ialu_ws_wbdone.p.rd = pin_wbd_rd;
                    ref.ialu_ws_wbdone.vld = 1;
                } else if (eu_which == 1) {
                    ref.falu_ws_wbdone.p.warp_id = pin_wbd_w;
                    ref.falu_ws_wbdone.p.rd = pin_wbd_rd;
                    ref.falu_ws_wbdone.vld = 1;
                } else {
                    ref.lsu_ws_wbdone.p.warp_id = pin_wbd_w;
                    ref.lsu_ws_wbdone.p.rd = pin_wbd_rd;
                    ref.lsu_ws_wbdone.vld = 1;
                }
            }
            /* 释放由 d_wbdA/d_wbdF/d_wbdL/d_br 检测段完成（DUT 消费）；
             * 参考侧消费由泵完成。此处不撤呈现 */
        }
        if (eu_busy && eu_cnt > 0)
            eu_cnt--;
        if (ic_occupied && ic_cnt > 0)
            ic_cnt--;
        if (rf_occupied && rf_cnt > 0)
            rf_cnt--;

        ref_pump_events();

        // ---- 时钟上升沿 ----
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (cur_blk >= nblks && eq_h == eq_t
            && !top->ws_bs_bdone_vld && !top->ws_icache_req_vld
            && !top->ws_rf_rd_vld && !top->ws_ialu_issue_vld
            && !top->ws_falu_issue_vld && !top->ws_lsu_issue_vld
            && !eu_busy && !ic_occupied && !rf_occupied
            && !pin_irsp && !pin_rrsp && !pin_br && !pin_wbd
            && !lmiss_pend && !rnone_pend
            && !ref.ws.launched && !ref.ws.active
            && !ref.icache_ws_rsp.vld && !ref.rf_ws_rddata.vld
            && !ref.ialu_ws_br.vld && !ref.ialu_ws_wbdone.vld
            && !ref.falu_ws_wbdone.vld && !ref.lsu_ws_wbdone.vld
            && !ref.ws_icache_req.vld && !ref.ws_rf_rd.vld
            && !ref.ws_ialu_issue.vld && !ref.ws_falu_issue.vld
            && !ref.ws_lsu_issue.vld && !ref.ws_bs_bdone.vld
            && ref.ws.fetch_warp < 0 && !ref.ws.fetch_pend
            && ref.ws.rd_warp < 0)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (cur_blk < nblks) err("blocks not all done");
    if (eq_h != eq_t) err("event queue not drained");
    if (ref.err || ref.ws.err)
        err("ref reported error");

    total_errors += errors;
    printf("[vsim] %-10s cyc=%-7d blks=%d -> %s\n",
           name, cyc, dut_bdone_cnt, errors ? "FAIL" : "PASS");
}

// ============================================================================
//  用例集
// ============================================================================
static void build_seq(void)
{
    /* 直行：LUI/ORI/IMAD/IADD + RET（每 warp 同程序） */
    put(0,  enc_lui(1, 0x12345));
    put(1,  enc_imm(OP_ORI, 1, 1, 0x678));
    put(2,  enc_r3(OP_IMAD, 2, 1, 1, 1));
    put(3,  enc_rr(OP_IADD, 3, 2, 1));
    put(4,  enc_ret());
}

static void build_hazard(void)
{
    /* 记分板互锁：IADD 紧跟 IMAD 写同一寄存器序列 */
    put(0,  enc_r3(OP_IMAD, 4, 1, 2, 3));
    put(1,  enc_rr(OP_IADD, 5, 4, 4));      // 读 4（上一条目的）→ 互锁
    put(2,  enc_rr(OP_IADD, 4, 5, 1));      // 写 4 再读 4
    put(3,  enc_rr(OP_XOR, 6, 4, 5));
    put(4,  enc_ret());
}

static void build_diverge(void)
{
    /* 单侧跳过型（target==R，黄金 pc9→13 模式）：BR 处置 mask←nt 顺序流，
     * taken lane 被 mask 屏蔽跳过 pc2 区域；R 处 JOIN 弹栈恢复全 mask */
    put(0,  enc_setp(0, 1, 2, 0, 1));       // P0 = (R1 > R2)
    put(1,  enc_br(0, 0, 0, 2));            // pc1: 目标 pc3 == R（单侧型）
    brput(1, 0x0F);                          // 前 4 lane taken（分化）
    brt_put(1, 3);                           // 重聚点 R=pc3
    put(2,  enc_rr(OP_IADD, 5, 5, 1));      // 顺序侧区域（nt lane）
    put(3,  enc_join(2));                    // R：JOIN 弹栈恢复全 mask，跳 pc5
    put(5,  enc_bar());                      // mask 已恢复全 1
    put(6,  enc_ret());
    /* 双侧型（黄金 pc21 模式）：BR pc11 目标=pc13（taken 侧）、R=pc16。
     * 压栈 (16,m)+(12,nt)，先走 taken：pc13 XOR → pc14 JOIN 弹顶
     * (12,nt) → mask=nt、pc=12 → always-BR 跳 pc15（顺序侧 XOR）→
     * pc15 后顺序到 pc16 JOIN 弹底 (16,m) → mask 全量、pc=17 RET */
    put(10, enc_setp(1, 1, 2, 0, 1));
    put(11, enc_br(1, 0, 0, 2));            // pc11: 目标 pc13 ≠ R（双侧型）
    brput(11, 0x3C);                          // 分化
    brt_put(11, 16);                          // R=pc16
    put(12, enc_br(2, 1, 0, 3));             // pc12: always 跳 pc15
    put(13, enc_rr(OP_XOR, 6, 6, 6));        // taken 侧
    put(14, enc_join(-2));                   // taken 出口：弹顶 (12,nt) 跳 12
    put(15, enc_rr(OP_XOR, 7, 7, 7));        // 顺序侧（nt）
    put(16, enc_join(1));                    // 汇合点：弹底 (16,m) 跳 17
    put(17, enc_ret());
    nblks = 1;
}

static void build_barrier(void)
{
    /* 屏障：三 warp 独立程序 + BAR + RET（pc0 起） */
    put(0,  enc_lui(1, 0x11111));
    put(1,  enc_bar());
    put(2,  enc_rr(OP_IADD, 2, 1, 1));
    put(3,  enc_ret());
}

static void build_ldp_csrr(void)
{
    /* LDP/CSRR 直通 + LDS/STS（lsu 路径 + LMISS 上报） */
    put(0,  enc_imm(OP_LDP, 4, 0, 0));      // R4 = param0
    put(1,  enc_rr(OP_CSRR, 5, 0, 0));      // R5 = lane id
    put(2,  enc_rr(OP_CSRR, 6, 1, 0));      // R6 = 线程总数
    put(3,  enc_rr(OP_CSRR, 7, 2, 0));      // R7 = blockIdx
    put(4,  enc_rr(OP_LDS, 8, 0, 4));       // LDS rd=8 rb=4
    put(5,  enc_ret());
}

static void build_err_bar(void)
{
    /* 错误注入：非法操作码（0x00）→ ws_top_err（DUT 侧检查） */
    put(0,  0x00000000u);                    // 非法 op
    put(1,  enc_ret());
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vws *top = new Vws;
    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("ws.vcd");

    prog_clear(); build_seq();
    run_test(top, tfp, "seq", 0,100,0,0, 0, 0, 0, 1);

    prog_clear(); build_hazard();
    run_test(top, tfp, "hazard", 0,100,0,0, 0, 0, 0, 2);

    prog_clear(); build_diverge();
    run_test(top, tfp, "diverge", 0,100,0,0, 1, 1, 1, 3);

    prog_clear(); build_barrier();
    run_test(top, tfp, "barrier", 0,100,0,0, 2, 0, 2, 4);

    prog_clear(); build_ldp_csrr();
    run_test(top, tfp, "ldp_csrr", 0,100,0,0, 0, 0, 1, 5);

    prog_clear(); build_seq();
    run_test(top, tfp, "bp50", 1, 50,0,0, 1, 1, 2, 6);

    prog_clear(); build_diverge();
    run_test(top, tfp, "bp20", 1, 20,0,0, 2, 2, 3, 7);

    prog_clear(); build_barrier();
    run_test(top, tfp, "pat", 2,100,3,11, 1, 2, 1, 8);

    prog_clear(); build_hazard();
    run_test(top, tfp, "lat4", 0,100,0,0, 4, 4, 4, 9);

    prog_clear(); build_seq();
    run_test(top, tfp, "soak", 1, 60,0,0, 0, 1, 0, 10);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
