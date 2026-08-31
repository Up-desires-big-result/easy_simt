// =============================================================================
// easy_simt · falu 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/falu/rtl/falu.sv 编译为 C++ 模型（Vfalu）；
// 本 harness 驱动时钟/复位/issue 激励与两路消费者背压，参考侧直接链接
// top/cmodel（falu_step，其数值通路为 softfloat.c 的 IEEE-754 binary32 RN），
// 记分板按协议语义逐笔比对（顺序 + 载荷位精确）。
//
// 波形：Verilator 原生 VCD（falu.vcd），gtkwave 查看。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vfalu.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链，仅用 falu 部分） ----------------
static sim_t ref;

// ---------------- 随机源（与 ialu harness 同式） ----------------
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

// ---------------- 激励队列（sf 侧发射序列，vld 保持至握手） ----------------
static falu_issue_t txq[65536];
static int txq_h, txq_t;
static void tx_push(const falu_issue_t *t) { txq[txq_t++ & 65535] = *t; }

// ---------------- 记分板：参考输出事务（类型 + 顺序 + 载荷） ----------------
enum { T_WB = 1, T_WD = 2 };
struct EvT {
    int type;
    int warp, rd, mask;
    uint32_t wdata[NLANES];
};
static EvT q[65536];
static int q_h, q_t;
static void ref_push(const EvT *e) { q[q_t++ & 65535] = *e; }

static int errors, total_errors, cyc;
static vluint64_t gtime;   // VCD 单调时基
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- 消费者背压决策（与 ialu harness 同式） ----------------
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
static int wb_mode, wb_pct, wb_on, wb_off, wb_pat;
static int wd_mode, wd_pct, wd_on, wd_off, wd_pat;

// ---------------- 在途镜像与协议保持影子 ----------------
static int mirror_busy, cur_op;
static int p_wb_v, p_wb_rdy, p_wb_warp, p_wb_rd, p_wb_mask;
static uint32_t p_wb_wdata[NLANES];
static int p_wd_v, p_wd_rdy, p_wd_warp, p_wd_rd;

// ---------------- DUT 发射比对 ----------------
static void dut_fire_wb(int warp, int rd, int mask, const uint32_t *wdata)
{
    if (q_h >= q_t) { err("DUT wb fire without ref transaction"); return; }
    EvT e = q[q_h++ & 65535];
    if (e.type != T_WB) { err("fire order mismatch at wb"); return; }
    if (e.warp != warp) err("wb warp_id mismatch");
    if (e.rd != rd)     err("wb rd mismatch");
    if (e.mask != mask) err("wb lane_mask mismatch");
    for (int l = 0; l < NLANES; l++)
        if (e.wdata[l] != wdata[l]) { err("wb wdata mismatch"); break; }
}
static void dut_fire_wd(int warp, int rd)
{
    if (q_h >= q_t) { err("DUT wbdone fire without ref transaction"); return; }
    EvT e = q[q_h++ & 65535];
    if (e.type != T_WD) { err("fire order mismatch at wbdone"); return; }
    if (e.warp != warp) err("wbdone warp_id mismatch");
    if (e.rd != rd)     err("wbdone rd mismatch");
}

// ---------------- 参考推进一拍：同激励下调用 falu_step，按 rdy 消费输出 ----------------
static int ref_cycle(int offer, const falu_issue_t *op, int r_wb, int r_wd)
{
    if (offer) {
        ref.sf_falu_issue.p = *op;
        ref.sf_falu_issue.vld = 1;
    }
    int had = ref.sf_falu_issue.vld;
    falu_step(&ref);
    int acc = had && !ref.sf_falu_issue.vld;

    if (ref.falu_rf_wb.vld && r_wb) {
        EvT e; memset(&e, 0, sizeof e);
        e.type = T_WB;
        e.warp = ref.falu_rf_wb.p.warp_id;
        e.rd = ref.falu_rf_wb.p.rd;
        e.mask = ref.falu_rf_wb.p.lane_mask;
        for (int l = 0; l < NLANES; l++) e.wdata[l] = ref.falu_rf_wb.p.wdata[l];
        ref_push(&e);
        ref.falu_rf_wb.vld = 0;
    }
    if (ref.falu_sf_wbdone.vld && r_wd) {
        EvT e; memset(&e, 0, sizeof e);
        e.type = T_WD;
        e.warp = ref.falu_sf_wbdone.p.warp_id;
        e.rd = ref.falu_sf_wbdone.p.rd;
        ref_push(&e);
        ref.falu_sf_wbdone.vld = 0;
    }
    return acc;
}

// ---------------- 单个用例 ----------------
static void run_test(Vfalu *top, VerilatedVcdC *tfp, const char *name,
                     int wbm, int wbp, int wbo, int wbf,
                     int wdm, int wdp, int wdo, int wdf)
{
    wb_mode = wbm; wb_pct = wbp; wb_on = wbo; wb_off = wbf;
    wd_mode = wdm; wd_pct = wdp; wd_on = wdo; wd_off = wdf;
    wb_pat = wd_pat = 0;
    errors = 0;
    q_h = q_t = 0;
    mirror_busy = 0; cur_op = -1;
    p_wb_v = p_wd_v = 0;
    cyc = 0;

    memset(&ref, 0, sizeof ref);

    // ---- 复位 ----
    top->rst_n = 0;
    top->sf_falu_issue_vld = 0;
    top->sf_falu_issue_opcode = 0;
    top->sf_falu_issue_rd = 0;
    top->sf_falu_issue_warp_id = 0;
    top->sf_falu_issue_lane_mask = 0;
    for (int l = 0; l < NLANES; l++) {
        top->sf_falu_issue_opa[l] = 0;
        top->sf_falu_issue_opb[l] = 0;
    }
    top->rf_falu_wb_rdy = 0;
    top->sf_falu_wbdone_rdy = 0;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 5; i++) {
        if (top->falu_rf_wb_vld || top->falu_sf_wbdone_vld)
            err("vld not 0 during reset");
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    }
    top->rst_n = 1;

    int ntx = txq_t - txq_h;
    int cap = ntx * 40 + 2000;
    int finished = 0;
    while (!finished && cyc < cap) {
        // -- 消费者决策 --
        int r_wb = rdy_dec(wb_mode, wb_pct, wb_on, wb_off, &wb_pat);
        int r_wd = rdy_dec(wd_mode, wd_pct, wd_on, wd_off, &wd_pat);

        // -- 激励：队列头持续给出直至握手（vld 保持，协议 §9 条 1） --
        int offer = (txq_h < txq_t);
        const falu_issue_t *tx = offer ? &txq[txq_h & 65535] : 0;

        top->sf_falu_issue_vld = offer;
        if (offer) {
            top->sf_falu_issue_opcode = tx->opcode;
            top->sf_falu_issue_rd = tx->rd;
            top->sf_falu_issue_warp_id = tx->warp_id;
            top->sf_falu_issue_lane_mask = tx->lane_mask;
            for (int l = 0; l < NLANES; l++) {
                top->sf_falu_issue_opa[l] = tx->opa[l];
                top->sf_falu_issue_opb[l] = tx->opb[l];
            }
        }
        top->rf_falu_wb_rdy = r_wb;
        top->sf_falu_wbdone_rdy = r_wd;
        top->eval();

        // -- 协议保持检查：上一拍 vld && !rdy，则本拍 vld 不撤、载荷不变 --
        if (p_wb_v && !p_wb_rdy) {
            if (!top->falu_rf_wb_vld)
                err("wb vld dropped under backpressure");
            else if ((int)top->falu_rf_wb_warp_id != p_wb_warp ||
                     (int)top->falu_rf_wb_rd != p_wb_rd ||
                     (int)top->falu_rf_wb_lane_mask != p_wb_mask)
                err("wb payload header changed under backpressure");
            else
                for (int l = 0; l < NLANES; l++)
                    if (top->falu_rf_wb_wdata[l] != p_wb_wdata[l]) {
                        err("wb wdata changed under backpressure");
                        break;
                    }
        }
        if (p_wd_v && !p_wd_rdy) {
            if (!top->falu_sf_wbdone_vld)
                err("wbdone vld dropped under backpressure");
            else if ((int)top->falu_sf_wbdone_warp_id != p_wd_warp ||
                     (int)top->falu_sf_wbdone_rd != p_wd_rd)
                err("wbdone payload changed under backpressure");
        }

        // -- 本拍末沿将发生的发射（输出为寄存器值，边沿前稳定） --
        int d_acc = offer && top->falu_sf_issue_rdy;
        int d_wb = top->falu_rf_wb_vld && r_wb;
        int d_wd = top->falu_sf_wbdone_vld && r_wd;
        if (d_wb + d_wd > 1)
            err("multiple output channels fire in one cycle");
        if (top->falu_sf_issue_rdy != !mirror_busy)
            err("issue_rdy inconsistent with in-flight state");

        // -- 参考同拍推进（同激励、同背压） --
        int r_acc = ref_cycle(offer, tx, r_wb, r_wd);
        if (r_acc != d_acc)
            err("issue handshake divergence between DUT and ref");

        if (d_acc) {
            mirror_busy = 1;
            cur_op = tx->opcode;
            txq_h++;
        }
        if (d_wb) {
            dut_fire_wb(top->falu_rf_wb_warp_id, top->falu_rf_wb_rd,
                        top->falu_rf_wb_lane_mask, &top->falu_rf_wb_wdata[0]);
            if (!mirror_busy) err("wb fire without in-flight op");
        }
        if (d_wd) {
            dut_fire_wd(top->falu_sf_wbdone_warp_id, top->falu_sf_wbdone_rd);
            if (cur_op != OP_FMUL && cur_op != OP_FADD && cur_op != OP_FNEG)
                err("wbdone fire from unexpected opcode");
            mirror_busy = 0; cur_op = -1;
        }

        // -- 影子更新 --
        p_wb_v = top->falu_rf_wb_vld; p_wb_rdy = r_wb;
        p_wb_warp = top->falu_rf_wb_warp_id;
        p_wb_rd = top->falu_rf_wb_rd;
        p_wb_mask = top->falu_rf_wb_lane_mask;
        for (int l = 0; l < NLANES; l++) p_wb_wdata[l] = top->falu_rf_wb_wdata[l];
        p_wd_v = top->falu_sf_wbdone_vld; p_wd_rdy = r_wd;
        p_wd_warp = top->falu_sf_wbdone_warp_id;
        p_wd_rd = top->falu_sf_wbdone_rd;

        // -- 时钟上升沿 --
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (txq_h == txq_t && q_h == q_t && !mirror_busy &&
            !top->falu_rf_wb_vld && !top->falu_sf_wbdone_vld &&
            !ref.falu_rf_wb.vld && !ref.falu_sf_wbdone.vld &&
            !ref.falu.has_issue)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (txq_h != txq_t) err("stimulus not drained");
    if (q_h != q_t) err("scoreboard not drained");
    if (ref.err) err("ref reported error");
    if (ref.falu.has_issue) err("ref still busy at end");

    total_errors += errors;
    printf("[vsim] %-10s n=%-6d cyc=%-8d -> %s\n",
           name, ntx, cyc, errors ? "FAIL" : "PASS");
}

// ============================================================================
//  激励生成
// ============================================================================
// 浮点数值类别池（±0/±inf/NaN/非规格化/规格化边界/常规值）
static const uint32_t pool[17] = {
    0x00000000u,   /* +0 */
    0x80000000u,   /* -0 */
    0x00000001u,   /* 最小非规格化 */
    0x007FFFFFu,   /* 最大非规格化 */
    0x00800000u,   /* 最小规格化 */
    0x00800001u,   /* 最小规格化 + 1ulp */
    0x3F800000u,   /* 1.0 */
    0xBF800000u,   /* -1.0 */
    0x3FC00000u,   /* 1.5 */
    0x7F7FFFFFu,   /* 最大有限数 */
    0x7F000000u,   /* 2^127 */
    0x00400000u,   /* 2^-127 */
    0x7F800000u,   /* +inf */
    0xFF800000u,   /* -inf */
    0x7FC00000u,   /* 静默 NaN */
    0x7F800001u,   /* NaN（另一编码） */
    0xFFC00001u    /* 负 NaN */
};
#define POOL_N 17

static void fill(uint32_t *v, uint32_t x)
{
    for (int l = 0; l < NLANES; l++) v[l] = x;
}

static void emit_fp(int op, int rd, int w, int mask,
                    const uint32_t *a, const uint32_t *b)
{
    falu_issue_t t; memset(&t, 0, sizeof t);
    t.opcode = op; t.rd = rd; t.warp_id = w;
    t.lane_mask = (uint8_t)mask;
    for (int l = 0; l < NLANES; l++) {
        t.opa[l] = a ? a[l] : 0;
        t.opb[l] = b ? b[l] : 0;
    }
    tx_push(&t);
}

// ---- 用例 1：FMUL 边界数据（全掩码、无背压） ----
static void build_op_fmul(void)
{
    uint32_t A[8], B[8];

    /* 符号与恒等 */
    fill(A, 0x3F800000u); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 1, 0, 0xFF, A, B);
    fill(A, 0xBF800000u); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 2, 0, 0xFF, A, B);
    fill(A, 0xBF800000u); fill(B, 0xBF800000u); emit_fp(OP_FMUL, 3, 0, 0xFF, A, B);

    /* 上溢 -> ±inf */
    fill(A, 0x7F7FFFFFu); fill(B, 0x40000000u); emit_fp(OP_FMUL, 4, 0, 0xFF, A, B);
    fill(A, 0x7F000000u); fill(B, 0x40800000u); emit_fp(OP_FMUL, 5, 0, 0xFF, A, B);
    fill(A, 0xFF7FFFFFu); fill(B, 0x40000000u); emit_fp(OP_FMUL, 6, 0, 0xFF, A, B);
    fill(A, 0x7F7FFFFFu); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 7, 0, 0xFF, A, B);

    /* 下溢 / FTZ */
    fill(A, 0x00800000u); fill(B, 0x00800000u); emit_fp(OP_FMUL, 8, 0, 0xFF, A, B);
    fill(A, 0x00400000u); fill(B, 0x00000001u); emit_fp(OP_FMUL, 9, 0, 0xFF, A, B);
    fill(A, 0x00000001u); fill(B, 0x00000001u); emit_fp(OP_FMUL, 10, 0, 0xFF, A, B);

    /* 非规格化输入先规格化 */
    fill(A, 0x007FFFFFu); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 11, 0, 0xFF, A, B);
    fill(A, 0x00000001u); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 12, 0, 0xFF, A, B);
    fill(A, 0x00000001u); fill(B, 0x40000000u); emit_fp(OP_FMUL, 13, 0, 0xFF, A, B);

    /* inf / NaN / 零类 */
    fill(A, 0x7F800000u); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 14, 0, 0xFF, A, B);
    fill(A, 0xFF800000u); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 15, 0, 0xFF, A, B);
    fill(A, 0x7F800000u); fill(B, 0x00000000u); emit_fp(OP_FMUL, 16, 0, 0xFF, A, B);
    fill(A, 0xFF800000u); fill(B, 0x80000000u); emit_fp(OP_FMUL, 17, 0, 0xFF, A, B);
    fill(A, 0x7F800000u); fill(B, 0xFF800000u); emit_fp(OP_FMUL, 18, 0, 0xFF, A, B);
    fill(A, 0x7FC00000u); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 19, 0, 0xFF, A, B);
    fill(A, 0x7F800001u); fill(B, 0x7F800000u); emit_fp(OP_FMUL, 20, 0, 0xFF, A, B);
    fill(A, 0x00000000u); fill(B, 0x00000000u); emit_fp(OP_FMUL, 21, 0, 0xFF, A, B);
    fill(A, 0x80000000u); fill(B, 0x3F800000u); emit_fp(OP_FMUL, 22, 0, 0xFF, A, B);
    fill(A, 0x80000000u); fill(B, 0xBF800000u); emit_fp(OP_FMUL, 23, 0, 0xFF, A, B);

    /* 舍入 */
    fill(A, 0x3F800001u); fill(B, 0x3F800001u); emit_fp(OP_FMUL, 24, 0, 0xFF, A, B);
    fill(A, 0x3F800001u); fill(B, 0x3F000000u); emit_fp(OP_FMUL, 25, 0, 0xFF, A, B);
    fill(A, 0x3FC00000u); fill(B, 0x3FC00000u); emit_fp(OP_FMUL, 26, 0, 0xFF, A, B);
    fill(A, 0x3FFFFFFFu); fill(B, 0x3FFFFFFFu); emit_fp(OP_FMUL, 27, 0, 0xFF, A, B);

    /* 逐 lane 混排 */
    for (int l = 0; l < NLANES; l++) { A[l] = pool[l]; B[l] = pool[(l + 5) % POOL_N]; }
    emit_fp(OP_FMUL, 28, 1, 0xFF, A, B);
    for (int l = 0; l < NLANES; l++) { A[l] = pool[(l * 3) % POOL_N]; B[l] = pool[(l * 7 + 2) % POOL_N]; }
    emit_fp(OP_FMUL, 29, 2, 0xFF, A, B);

    /* rd = R0 */
    fill(A, 0x3F800000u); fill(B, 0x40000000u); emit_fp(OP_FMUL, 0, 0, 0xFF, A, B);
}

// ---- 用例 2：FADD 边界数据（全掩码、无背压） ----
static void build_op_fadd(void)
{
    uint32_t A[8], B[8];

    /* 零类与恒等 */
    fill(A, 0x3F800000u); fill(B, 0x00000000u); emit_fp(OP_FADD, 1, 0, 0xFF, A, B);
    fill(A, 0x00000001u); fill(B, 0x00000000u); emit_fp(OP_FADD, 2, 0, 0xFF, A, B);
    fill(A, 0x007FFFFFu); fill(B, 0x80000000u); emit_fp(OP_FADD, 3, 0, 0xFF, A, B);
    fill(A, 0x00000000u); fill(B, 0x00000000u); emit_fp(OP_FADD, 4, 0, 0xFF, A, B);
    fill(A, 0x80000000u); fill(B, 0x80000000u); emit_fp(OP_FADD, 5, 0, 0xFF, A, B);
    fill(A, 0x00000000u); fill(B, 0x80000000u); emit_fp(OP_FADD, 6, 0, 0xFF, A, B);

    /* inf 类 */
    fill(A, 0x7F800000u); fill(B, 0x3F800000u); emit_fp(OP_FADD, 7, 0, 0xFF, A, B);
    fill(A, 0xFF800000u); fill(B, 0xBF800000u); emit_fp(OP_FADD, 8, 0, 0xFF, A, B);
    fill(A, 0x7F800000u); fill(B, 0xFF800000u); emit_fp(OP_FADD, 9, 0, 0xFF, A, B);
    fill(A, 0x7F800000u); fill(B, 0x7F800000u); emit_fp(OP_FADD, 10, 0, 0xFF, A, B);
    fill(A, 0xFF800000u); fill(B, 0x00000001u); emit_fp(OP_FADD, 11, 0, 0xFF, A, B);

    /* NaN 类 */
    fill(A, 0x7FC00000u); fill(B, 0x3F800000u); emit_fp(OP_FADD, 12, 0, 0xFF, A, B);
    fill(A, 0x3F800000u); fill(B, 0xFFC00001u); emit_fp(OP_FADD, 13, 0, 0xFF, A, B);
    fill(A, 0x7F800001u); fill(B, 0x7FC00000u); emit_fp(OP_FADD, 14, 0, 0xFF, A, B);

    /* 精确抵消 -> +0 */
    fill(A, 0x3FC00000u); fill(B, 0xBFC00000u); emit_fp(OP_FADD, 15, 0, 0xFF, A, B);
    fill(A, 0x00000001u); fill(B, 0x80000001u); emit_fp(OP_FADD, 16, 0, 0xFF, A, B);
    fill(A, 0x7F7FFFFFu); fill(B, 0xFF7FFFFFu); emit_fp(OP_FADD, 17, 0, 0xFF, A, B);

    /* 近抵消与大指数差对齐 */
    fill(A, 0x3F800000u); fill(B, 0xBF7FFFFFu); emit_fp(OP_FADD, 18, 0, 0xFF, A, B);
    fill(A, 0x7F000000u); fill(B, 0x00000001u); emit_fp(OP_FADD, 19, 0, 0xFF, A, B);
    fill(A, 0x7F7FFFFFu); fill(B, 0x00800000u); emit_fp(OP_FADD, 20, 0, 0xFF, A, B);
    fill(A, 0x00400000u); fill(B, 0x3F800000u); emit_fp(OP_FADD, 21, 0, 0xFF, A, B);

    /* FTZ 边界 */
    fill(A, 0x00800000u); fill(B, 0x807FFFFFu); emit_fp(OP_FADD, 22, 0, 0xFF, A, B);
    fill(A, 0x007FFFFFu); fill(B, 0x00000001u); emit_fp(OP_FADD, 23, 0, 0xFF, A, B);
    fill(A, 0x00000001u); fill(B, 0x00000001u); emit_fp(OP_FADD, 24, 0, 0xFF, A, B);

    /* 上溢 -> ±inf */
    fill(A, 0x7F7FFFFFu); fill(B, 0x7F7FFFFFu); emit_fp(OP_FADD, 25, 0, 0xFF, A, B);
    fill(A, 0xFF7FFFFFu); fill(B, 0xFF7FFFFFu); emit_fp(OP_FADD, 26, 0, 0xFF, A, B);
    fill(A, 0x7F000000u); fill(B, 0x7F000000u); emit_fp(OP_FADD, 27, 0, 0xFF, A, B);

    /* 舍入临界（1.0 附近半 ulp 对齐） */
    fill(A, 0x3F800000u); fill(B, 0x33800000u); emit_fp(OP_FADD, 28, 0, 0xFF, A, B);
    fill(A, 0x3F800000u); fill(B, 0x33800001u); emit_fp(OP_FADD, 29, 0, 0xFF, A, B);
    fill(A, 0x3F800000u); fill(B, 0x337FFFFFu); emit_fp(OP_FADD, 30, 0, 0xFF, A, B);
    fill(A, 0x3F800001u); fill(B, 0x33800000u); emit_fp(OP_FADD, 31, 0, 0xFF, A, B);

    /* 逐 lane 混排 */
    for (int l = 0; l < NLANES; l++) { A[l] = pool[l]; B[l] = pool[(l + 9) % POOL_N]; }
    emit_fp(OP_FADD, 32, 1, 0xFF, A, B);
    for (int l = 0; l < NLANES; l++) { A[l] = pool[(l * 5 + 1) % POOL_N]; B[l] = pool[(l * 3 + 4) % POOL_N]; }
    emit_fp(OP_FADD, 33, 2, 0xFF, A, B);

    /* rd = R0 */
    fill(A, 0x3F800000u); fill(B, 0x3F800000u); emit_fp(OP_FADD, 0, 0, 0xFF, A, B);
}

// ---- 用例 3：FNEG 全类别 ----
static void build_fneg(void)
{
    uint32_t A[8];
    for (int i = 0; i < POOL_N; i++) {
        fill(A, pool[i]);
        emit_fp(OP_FNEG, (i & 31), i & 3, 0xFF, A, 0);
    }
    for (int l = 0; l < NLANES; l++) A[l] = pool[l];
    emit_fp(OP_FNEG, 1, 0, 0xFF, A, 0);
    for (int l = 0; l < NLANES; l++) A[l] = pool[POOL_N - 1 - l];
    emit_fp(OP_FNEG, 2, 1, 0xFF, A, 0);
    for (int l = 0; l < NLANES; l++) A[l] = rnd32();
    emit_fp(OP_FNEG, 3, 2, 0xFF, A, 0);
    emit_fp(OP_FNEG, 0, 0, 0xFF, A, 0);   /* rd = R0 */
}

// ---- 用例 4：掩码门控（非活动 lane 结果为 0） ----
static void build_masking(void)
{
    static const int masks[8] = { 0x00, 0x01, 0x80, 0xA5, 0x5A, 0x0F, 0xF0, 0x7E };
    uint32_t A[8], B[8];
    for (int i = 0; i < 8; i++) {
        for (int l = 0; l < NLANES; l++) {
            A[l] = pool[rnd() % POOL_N];
            B[l] = pool[rnd() % POOL_N];
            if (rnd() & 1) A[l] = rnd32();
            if (rnd() & 1) B[l] = rnd32();
        }
        emit_fp(OP_FMUL, 1, 0, masks[i], A, B);
        emit_fp(OP_FADD, 2, 1, masks[i], A, B);
        emit_fp(OP_FNEG, 3, 2, masks[i], A, 0);
        emit_fp(OP_FMUL, 4, 3, masks[i], A, B);
    }
}

// ---- 用例 5：特殊值两两组合全交叉（pool × pool × {FMUL,FADD}） ----
static void build_edge_mix(void)
{
    uint32_t A[8], B[8];
    for (int i = 0; i < POOL_N; i++)
        for (int j = 0; j < POOL_N; j++) {
            fill(A, pool[i]); fill(B, pool[j]);
            emit_fp(OP_FMUL, 1, 0, 0xFF, A, B);
            emit_fp(OP_FADD, 2, 1, 0xFF, A, B);
        }
    /* 逐 lane 异值交叉 */
    for (int k = 0; k < 4; k++) {
        for (int l = 0; l < NLANES; l++) {
            A[l] = pool[(l + k * 4) % POOL_N];
            B[l] = pool[(l * 2 + k * 3 + 1) % POOL_N];
        }
        emit_fp(OP_FMUL, 3, k, 0xFF, A, B);
        emit_fp(OP_FADD, 4, k, 0xFF, A, B);
    }
}

// ---- 随机浮点操作数（与同 lane 前一操作数关联，覆盖对齐/抵消/舍入） ----
static uint32_t fp_rnd(uint32_t sibling)
{
    switch (rnd() % 10) {
    case 0: return pool[rnd() % POOL_N];              /* 特殊类别 */
    case 1: return rnd32();                           /* 任意位型 */
    case 2: return sibling;                           /* 等值（x+x / x*x） */
    case 3: return sibling ^ 0x80000000u;             /* 变号（精确抵消） */
    case 4: return sibling + (uint32_t)((rnd() & 3) - 1);   /* ulp 邻域 */
    case 5: return (sibling & 0xFF800000u) | (rnd32() & 0x007FFFFFu); /* 同指数 */
    case 6: {                                          /* 邻指数同尾数 */
        uint32_t e = (sibling >> 23) & 0xFF;
        int d = (rnd() % 3) - 1;
        int en = (int)e + d;
        if (en < 0) en = 0; if (en > 255) en = 255;
        return (sibling & 0x807FFFFFu) | ((uint32_t)en << 23);
    }
    case 7: {                                          /* 偏向指数构造 */
        static const int exps[10] = { 0, 1, 2, 125, 126, 127, 128, 129, 254, 255 };
        uint32_t sg = (uint32_t)(rnd() & 1) << 31;
        uint32_t e = (uint32_t)exps[rnd() % 10];
        return sg | (e << 23) | (rnd32() & 0x007FFFFFu);
    }
    case 8: return (rnd32() & 0x007FFFFFu) | ((uint32_t)(rnd() & 1) << 31); /* 非规格化偏向 */
    default: {                                         /* 全范围规格化 */
        uint32_t sg = (uint32_t)(rnd() & 1) << 31;
        uint32_t e = 1 + (uint32_t)(rnd() % 254);
        return sg | (e << 23) | (rnd32() & 0x007FFFFFu);
    }
    }
}

// ---- 随机混合 ----
static void gen_random(int n)
{
    uint32_t A[8], B[8];
    for (int i = 0; i < n; i++) {
        int op = OP_FMUL + (rnd() % 3);
        int w = rnd() % NWARPS;
        int mask = rnd() & 0xFF;
        int rd = rnd() % 32;
        for (int l = 0; l < NLANES; l++) {
            A[l] = fp_rnd(0x3F800000u);
            B[l] = fp_rnd(A[l]);
        }
        if (op == OP_FNEG)
            emit_fp(op, rd, w, mask, A, 0);
        else
            emit_fp(op, rd, w, mask, A, B);
    }
}

static void begin_stim(void) { txq_h = txq_t = 0; }

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vfalu *top = new Vfalu;
    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("falu.vcd");

    seed_g = 1; begin_stim(); build_op_fmul();
    run_test(top, tfp, "op_fmul", 0,100,0,0, 0,100,0,0);

    seed_g = 2; begin_stim(); build_op_fadd();
    run_test(top, tfp, "op_fadd", 0,100,0,0, 0,100,0,0);

    seed_g = 3; begin_stim(); build_fneg();
    run_test(top, tfp, "fneg",    0,100,0,0, 0,100,0,0);

    seed_g = 4; begin_stim(); build_masking();
    run_test(top, tfp, "masking", 0,100,0,0, 0,100,0,0);

    seed_g = 5; begin_stim(); build_edge_mix();
    run_test(top, tfp, "edge",    2,100,3,5, 2,100,5,7);

    seed_g = 6; begin_stim(); gen_random(2000);
    run_test(top, tfp, "b2b",     0,100,0,0, 0,100,0,0);

    seed_g = 7; begin_stim(); gen_random(4000);
    run_test(top, tfp, "rand50",  1, 50,0,0, 1, 50,0,0);

    seed_g = 8; begin_stim(); gen_random(4000);
    run_test(top, tfp, "rand20",  1, 20,0,0, 1, 30,0,0);

    seed_g = 9; begin_stim(); gen_random(3000);
    run_test(top, tfp, "pat",     2,100,3,11, 2,100,1,15);

    seed_g = 10; begin_stim(); gen_random(10000);
    run_test(top, tfp, "soak",    1, 60,0,0, 2,100,7,3);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
