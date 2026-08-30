// =============================================================================
// easy_simt · ialu 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/ialu/rtl/ialu.sv 编译为 C++ 模型（Vialu）；
// 本 harness 驱动时钟/复位/issue 激励与三路消费者背压，参考侧直接链接
// top/cmodel（ialu_step），记分板按协议语义逐笔比对（顺序 + 载荷位精确）。
//
// 波形：Verilator 原生 VCD（ialu.vcd），gtkwave 查看。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vialu.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链，仅用 ialu 部分） ----------------
static sim_t ref;

// ---------------- 随机源（与 bs harness 同式） ----------------
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
static ialu_issue_t txq[65536];
static int txq_h, txq_t;
static uint32_t pc_seq;
static void tx_push(const ialu_issue_t *t) { txq[txq_t++ & 65535] = *t; }

// ---------------- 记分板：参考输出事务（类型 + 顺序 + 载荷） ----------------
enum { T_BR = 1, T_WB = 2, T_WD = 3 };
struct EvT {
    int type;
    int warp, rd, mask;
    uint32_t taken, target, brt;
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

// ---------------- 消费者背压决策（与 bs harness 同式） ----------------
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
static int br_mode, br_pct, br_on, br_off, br_pat;
static int wb_mode, wb_pct, wb_on, wb_off, wb_pat;
static int wd_mode, wd_pct, wd_on, wd_off, wd_pat;

// ---------------- 在途镜像与协议保持影子 ----------------
static int mirror_busy, cur_op;
static int p_br_v, p_br_rdy, p_br_warp; static uint32_t p_br_taken, p_br_target;
static int p_wb_v, p_wb_rdy, p_wb_warp, p_wb_rd, p_wb_mask;
static uint32_t p_wb_wdata[NLANES];
static int p_wd_v, p_wd_rdy, p_wd_warp, p_wd_rd;

// ---------------- DUT 发射比对 ----------------
static void dut_fire_br(int warp, uint32_t taken, uint32_t target, uint32_t brt)
{
    if (q_h >= q_t) { err("DUT br fire without ref transaction"); return; }
    EvT e = q[q_h++ & 65535];
    if (e.type != T_BR) { err("fire order mismatch at br"); return; }
    if (e.warp != warp)        err("br warp_id mismatch");
    if (e.taken != taken)      err("br taken mismatch");
    if (e.target != target)    err("br target mismatch");
    if (brt != 0 || e.brt != 0) err("br brt_idx not 0");
}
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

// ---------------- 参考推进一拍：同激励下调用 ialu_step，按 rdy 消费输出 ----------------
static int ref_cycle(int offer, const ialu_issue_t *op,
                     int r_br, int r_wb, int r_wd)
{
    if (offer) {
        ref.sf_ialu_issue.p = *op;
        ref.sf_ialu_issue.vld = 1;
    }
    int had = ref.sf_ialu_issue.vld;
    ialu_step(&ref);
    int acc = had && !ref.sf_ialu_issue.vld;

    if (ref.ialu_sf_br.vld && r_br) {
        EvT e; memset(&e, 0, sizeof e);
        e.type = T_BR;
        e.warp = ref.ialu_sf_br.p.warp_id;
        e.taken = ref.ialu_sf_br.p.taken;
        e.target = ref.ialu_sf_br.p.target;
        e.brt = ref.ialu_sf_br.p.brt_idx;
        ref_push(&e);
        ref.ialu_sf_br.vld = 0;
    }
    if (ref.ialu_rf_wb.vld && r_wb) {
        EvT e; memset(&e, 0, sizeof e);
        e.type = T_WB;
        e.warp = ref.ialu_rf_wb.p.warp_id;
        e.rd = ref.ialu_rf_wb.p.rd;
        e.mask = ref.ialu_rf_wb.p.lane_mask;
        for (int l = 0; l < NLANES; l++) e.wdata[l] = ref.ialu_rf_wb.p.wdata[l];
        ref_push(&e);
        ref.ialu_rf_wb.vld = 0;
    }
    if (ref.ialu_sf_wbdone.vld && r_wd) {
        EvT e; memset(&e, 0, sizeof e);
        e.type = T_WD;
        e.warp = ref.ialu_sf_wbdone.p.warp_id;
        e.rd = ref.ialu_sf_wbdone.p.rd;
        ref_push(&e);
        ref.ialu_sf_wbdone.vld = 0;
    }
    return acc;
}

// ---------------- 单个用例 ----------------
static void run_test(Vialu *top, VerilatedVcdC *tfp, const char *name,
                     int brm, int brp, int bro, int brf,
                     int wbm, int wbp, int wbo, int wbf,
                     int wdm, int wdp, int wdo, int wdf)
{
    br_mode = brm; br_pct = brp; br_on = bro; br_off = brf;
    wb_mode = wbm; wb_pct = wbp; wb_on = wbo; wb_off = wbf;
    wd_mode = wdm; wd_pct = wdp; wd_on = wdo; wd_off = wdf;
    br_pat = wb_pat = wd_pat = 0;
    errors = 0;
    q_h = q_t = 0;
    mirror_busy = 0; cur_op = -1;
    p_br_v = p_wb_v = p_wd_v = 0;
    cyc = 0;

    memset(&ref, 0, sizeof ref);

    // ---- 复位 ----
    top->rst_n = 0;
    top->sf_ialu_issue_vld = 0;
    top->sf_ialu_issue_opcode = 0;
    top->sf_ialu_issue_rd = 0;
    top->sf_ialu_issue_warp_id = 0;
    top->sf_ialu_issue_lane_mask = 0;
    top->sf_ialu_issue_pc = 0;
    top->sf_ialu_issue_imm = 0;
    for (int l = 0; l < NLANES; l++) {
        top->sf_ialu_issue_opa[l] = 0;
        top->sf_ialu_issue_opb[l] = 0;
        top->sf_ialu_issue_opc[l] = 0;
    }
    top->sf_ialu_br_rdy = 0;
    top->rf_ialu_wb_rdy = 0;
    top->sf_ialu_wbdone_rdy = 0;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 5; i++) {
        if (top->ialu_sf_br_vld || top->ialu_rf_wb_vld || top->ialu_sf_wbdone_vld)
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
        int r_br = rdy_dec(br_mode, br_pct, br_on, br_off, &br_pat);
        int r_wb = rdy_dec(wb_mode, wb_pct, wb_on, wb_off, &wb_pat);
        int r_wd = rdy_dec(wd_mode, wd_pct, wd_on, wd_off, &wd_pat);

        // -- 激励：队列头持续给出直至握手（vld 保持，协议 §9 条 1） --
        int offer = (txq_h < txq_t);
        const ialu_issue_t *tx = offer ? &txq[txq_h & 65535] : 0;

        top->sf_ialu_issue_vld = offer;
        if (offer) {
            top->sf_ialu_issue_opcode = tx->opcode;
            top->sf_ialu_issue_rd = tx->rd;
            top->sf_ialu_issue_warp_id = tx->warp_id;
            top->sf_ialu_issue_lane_mask = tx->lane_mask;
            top->sf_ialu_issue_pc = tx->pc;
            top->sf_ialu_issue_imm = tx->imm;
            for (int l = 0; l < NLANES; l++) {
                top->sf_ialu_issue_opa[l] = tx->opa[l];
                top->sf_ialu_issue_opb[l] = tx->opb[l];
                top->sf_ialu_issue_opc[l] = tx->opc[l];
            }
        }
        top->sf_ialu_br_rdy = r_br;
        top->rf_ialu_wb_rdy = r_wb;
        top->sf_ialu_wbdone_rdy = r_wd;
        top->eval();

        // -- 协议保持检查：上一拍 vld && !rdy，则本拍 vld 不撤、载荷不变 --
        if (p_br_v && !p_br_rdy) {
            if (!top->ialu_sf_br_vld)
                err("br vld dropped under backpressure");
            else if ((int)top->ialu_sf_br_warp_id != p_br_warp ||
                     top->ialu_sf_br_taken != p_br_taken ||
                     top->ialu_sf_br_target != p_br_target)
                err("br payload changed under backpressure");
        }
        if (p_wb_v && !p_wb_rdy) {
            if (!top->ialu_rf_wb_vld)
                err("wb vld dropped under backpressure");
            else if ((int)top->ialu_rf_wb_warp_id != p_wb_warp ||
                     (int)top->ialu_rf_wb_rd != p_wb_rd ||
                     (int)top->ialu_rf_wb_lane_mask != p_wb_mask)
                err("wb payload header changed under backpressure");
            else
                for (int l = 0; l < NLANES; l++)
                    if (top->ialu_rf_wb_wdata[l] != p_wb_wdata[l]) {
                        err("wb wdata changed under backpressure");
                        break;
                    }
        }
        if (p_wd_v && !p_wd_rdy) {
            if (!top->ialu_sf_wbdone_vld)
                err("wbdone vld dropped under backpressure");
            else if ((int)top->ialu_sf_wbdone_warp_id != p_wd_warp ||
                     (int)top->ialu_sf_wbdone_rd != p_wd_rd)
                err("wbdone payload changed under backpressure");
        }

        // -- 本拍末沿将发生的发射（输出为寄存器值，边沿前稳定） --
        int d_acc = offer && top->ialu_sf_issue_rdy;
        int d_br = top->ialu_sf_br_vld && r_br;
        int d_wb = top->ialu_rf_wb_vld && r_wb;
        int d_wd = top->ialu_sf_wbdone_vld && r_wd;
        if (d_br + d_wb + d_wd > 1)
            err("multiple output channels fire in one cycle");
        if (top->ialu_sf_issue_rdy != !mirror_busy)
            err("issue_rdy inconsistent with in-flight state");

        // -- 参考同拍推进（同激励、同背压） --
        int r_acc = ref_cycle(offer, tx, r_br, r_wb, r_wd);
        if (r_acc != d_acc)
            err("issue handshake divergence between DUT and ref");

        if (d_acc) {
            mirror_busy = 1;
            cur_op = tx->opcode;
            txq_h++;
        }
        if (d_br) {
            dut_fire_br(top->ialu_sf_br_warp_id, top->ialu_sf_br_taken,
                        top->ialu_sf_br_target, top->ialu_sf_br_brt_idx);
            if (cur_op != OP_BR) err("br fire while in-flight op is not BR");
            mirror_busy = 0; cur_op = -1;
        }
        if (d_wb) {
            dut_fire_wb(top->ialu_rf_wb_warp_id, top->ialu_rf_wb_rd,
                        top->ialu_rf_wb_lane_mask, &top->ialu_rf_wb_wdata[0]);
            if (cur_op == OP_BR || cur_op == OP_SETP)
                err("wb fire from non-wb opcode");
        }
        if (d_wd) {
            dut_fire_wd(top->ialu_sf_wbdone_warp_id, top->ialu_sf_wbdone_rd);
            if (cur_op == OP_BR) err("wbdone fire from BR");
            mirror_busy = 0; cur_op = -1;
        }

        // -- 影子更新 --
        p_br_v = top->ialu_sf_br_vld; p_br_rdy = r_br;
        p_br_warp = top->ialu_sf_br_warp_id;
        p_br_taken = top->ialu_sf_br_taken;
        p_br_target = top->ialu_sf_br_target;
        p_wb_v = top->ialu_rf_wb_vld; p_wb_rdy = r_wb;
        p_wb_warp = top->ialu_rf_wb_warp_id;
        p_wb_rd = top->ialu_rf_wb_rd;
        p_wb_mask = top->ialu_rf_wb_lane_mask;
        for (int l = 0; l < NLANES; l++) p_wb_wdata[l] = top->ialu_rf_wb_wdata[l];
        p_wd_v = top->ialu_sf_wbdone_vld; p_wd_rdy = r_wd;
        p_wd_warp = top->ialu_sf_wbdone_warp_id;
        p_wd_rd = top->ialu_sf_wbdone_rd;

        // -- 时钟上升沿 --
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (txq_h == txq_t && q_h == q_t && !mirror_busy &&
            !top->ialu_sf_br_vld && !top->ialu_rf_wb_vld &&
            !top->ialu_sf_wbdone_vld &&
            !ref.ialu_sf_br.vld && !ref.ialu_rf_wb.vld &&
            !ref.ialu_sf_wbdone.vld && !ref.ialu.has_issue)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (txq_h != txq_t) err("stimulus not drained");
    if (q_h != q_t) err("scoreboard not drained");
    if (ref.err) err("ref reported error");
    if (ref.ialu.has_issue) err("ref still busy at end");

    total_errors += errors;
    printf("[vsim] %-10s n=%-6d cyc=%-8d -> %s\n",
           name, ntx, cyc, errors ? "FAIL" : "PASS");
}

// ============================================================================
//  激励生成
// ============================================================================
static void fill(uint32_t *v, uint32_t x)
{
    for (int l = 0; l < NLANES; l++) v[l] = x;
}

static void emit_alu(int op, int rd, int w, int mask,
                     const uint32_t *a, const uint32_t *b, const uint32_t *c)
{
    ialu_issue_t t; memset(&t, 0, sizeof t);
    t.opcode = op; t.rd = rd; t.warp_id = w;
    t.lane_mask = (uint8_t)mask;
    t.pc = pc_seq++;
    for (int l = 0; l < NLANES; l++) {
        t.opa[l] = a ? a[l] : 0;
        t.opb[l] = b ? b[l] : 0;
        t.opc[l] = c ? c[l] : 0;
    }
    tx_push(&t);
}

static void emit_setp(int pd, int w, int mask, int fmt, int cond,
                      const uint32_t *a, const uint32_t *b)
{
    ialu_issue_t t; memset(&t, 0, sizeof t);
    t.opcode = OP_SETP; t.rd = pd; t.warp_id = w;
    t.lane_mask = (uint8_t)mask;
    t.pc = pc_seq++;
    t.imm = (uint32_t)((fmt << 3) | cond);
    for (int l = 0; l < NLANES; l++) { t.opa[l] = a[l]; t.opb[l] = b[l]; }
    tx_push(&t);
}

static void emit_br(int psel, int w, int mask, int u, int neg, uint32_t target)
{
    ialu_issue_t t; memset(&t, 0, sizeof t);
    t.opcode = OP_BR; t.rd = psel; t.warp_id = w;
    t.lane_mask = (uint8_t)mask;
    t.pc = pc_seq++;
    t.imm = ((uint32_t)u << 31) | ((uint32_t)neg << 30) | (target & 0x3FFFFFFFu);
    tx_push(&t);
}

// ---- 用例 1：ALU 类指令 × 边界数据（全掩码、无背压） ----
static void build_op_alu(void)
{
    uint32_t A[8], B[8], C[8];

    fill(A, 0xFFFFFFFFu); fill(B, 1);            emit_alu(OP_IADD, 1, 0, 0xFF, A, B, 0);
    fill(A, 0x7FFFFFFFu); fill(B, 1);            emit_alu(OP_IADD, 2, 0, 0xFF, A, B, 0);
    fill(A, 0x80000000u); fill(B, 0x80000000u);  emit_alu(OP_IADD, 3, 0, 0xFF, A, B, 0);
    for (int l = 0; l < NLANES; l++) { A[l] = (uint32_t)l * 0x11111111u; B[l] = (uint32_t)l; }
    emit_alu(OP_IADD, 4, 1, 0xFF, A, B, 0);

    for (int l = 0; l < NLANES; l++) { fill(A, 0xDEADBEEFu); B[l] = 0; }
    B[0] = 0; B[1] = 1; B[2] = 15; B[3] = 31; B[4] = 35; B[5] = 32; B[6] = 63; B[7] = 16;
    fill(A, 0xDEADBEEFu);
    emit_alu(OP_SHL, 5, 0, 0xFF, A, B, 0);
    fill(B, 0);  emit_alu(OP_SHL, 6, 2, 0xFF, A, B, 0);
    fill(B, 31); emit_alu(OP_SHL, 7, 3, 0xFF, A, B, 0);

    fill(A, 0x12345678u); fill(B, 0x00000000u);  emit_alu(OP_XOR, 8, 0, 0xFF, A, B, 0);
    fill(B, 0xFFFFFFFFu);                        emit_alu(OP_XOR, 9, 0, 0xFF, A, B, 0);
    fill(B, 0x12345678u);                        emit_alu(OP_XOR, 10, 0, 0xFF, A, B, 0);
    for (int l = 0; l < NLANES; l++) { A[l] = rnd32(); B[l] = rnd32(); }
    emit_alu(OP_XOR, 11, 1, 0xFF, A, B, 0);

    fill(A, 0xFFFF0000u); fill(B, 0x0000ABCDu);  emit_alu(OP_ORI, 12, 0, 0xFF, A, B, 0);
    fill(A, 0x0000FFFFu); fill(B, 0xFFFF0000u);  emit_alu(OP_ORI, 13, 0, 0xFF, A, B, 0);

    fill(A, 0x3F800000u); emit_alu(OP_LUI, 14, 0, 0xFF, A, 0, 0);
    fill(A, 0x00000000u); emit_alu(OP_LUI, 15, 0, 0xFF, A, 0, 0);
    fill(A, 1000);        emit_alu(OP_LDP, 16, 0, 0xFF, A, 0, 0);
    for (int l = 0; l < NLANES; l++) A[l] = (uint32_t)l;   /* CSRR TID 形态：逐 lane */
    emit_alu(OP_CSRR, 17, 2, 0xFF, A, 0, 0);

    fill(A, 0x7FFFFFFFu); fill(B, 2); fill(C, 1);          emit_alu(OP_IMAD, 18, 0, 0xFF, A, B, C);
    fill(A, 0xFFFFFFFFu); fill(B, 0xFFFFFFFFu); fill(C, 0); emit_alu(OP_IMAD, 19, 0, 0xFF, A, B, C);
    fill(A, 0x80000000u); fill(B, 2); fill(C, 0x80000000u); emit_alu(OP_IMAD, 20, 0, 0xFF, A, B, C);
    for (int l = 0; l < NLANES; l++) {
        A[l] = (uint32_t)((int)l - 4) * 12345u;
        B[l] = (uint32_t)((int)l - 2) * 6789u;
        C[l] = 0xFFFFFFF6u;
    }
    emit_alu(OP_IMAD, 21, 1, 0xFF, A, B, C);
    fill(A, 0xFFFFFFFFu); fill(B, 0x7FFFFFFFu); fill(C, 0x7FFFFFFFu);
    emit_alu(OP_IMAD, 22, 2, 0xFF, A, B, C);

    fill(A, 7); fill(B, 9); emit_alu(OP_IADD, 0, 0, 0xFF, A, B, 0);   /* rd = R0 */
}

// ---- 用例 2：掩码门控（非活动 lane 结果为 0） ----
static void build_masking(void)
{
    static const int masks[8] = { 0x00, 0x01, 0x80, 0xA5, 0x5A, 0x0F, 0xF0, 0x7E };
    uint32_t A[8], B[8], C[8];
    for (int i = 0; i < 8; i++) {
        for (int l = 0; l < NLANES; l++) { A[l] = rnd32(); B[l] = rnd32(); C[l] = rnd32(); }
        emit_alu(OP_IADD, 1, 0, masks[i], A, B, 0);
        emit_alu(OP_XOR,  2, 1, masks[i], A, B, 0);
        emit_alu(OP_IMAD, 3, 2, masks[i], A, B, C);
        emit_alu(OP_LUI,  4, 3, masks[i], A, 0, 0);
        emit_setp(i & 3, i & 3, masks[i], 0, i % 6, A, B);
        emit_br(i & 3, i & 3, 0xFF, 0, 0, 0x1000u + i);
    }
}

// ---- 用例 3：SETP 整数比较全 cond × 关系，后置 BR 读出谓词 ----
static void build_setp_int(void)
{
    static const uint32_t va[6] = { 0xFFFFFFFBu, 5, 3, 0xFFFFFFFFu, 0x7FFFFFFFu, 0 };
    static const uint32_t vb[6] = { 5, 0xFFFFFFFBu, 3, 0, 0x80000000u, 0 };
    uint32_t A[8], B[8];
    for (int pd = 0; pd < 4; pd++)
        for (int cond = 0; cond < 6; cond++)
            for (int k = 0; k < 6; k++) {
                fill(A, va[k]); fill(B, vb[k]);
                emit_setp(pd, pd, 0xFF, 0, cond, A, B);
                emit_br(pd, pd, 0xFF, 0, 0, 0x2000u + (pd << 6) + (cond << 3) + k);
            }
    /* 逐 lane 异值 */
    for (int l = 0; l < NLANES; l++) {
        A[l] = (uint32_t)((int)l - 3);
        B[l] = 0;
    }
    for (int cond = 0; cond < 6; cond++) {
        emit_setp(0, 1, 0xFF, 0, cond, A, B);
        emit_br(0, 1, 0xFF, 0, 1, 0x3000u + cond);
    }
}

// ---- 用例 4：SETP 浮点比较全 cond × 浮点类别（含 NaN/±∞/±0/非规格化） ----
static void build_setp_flt(void)
{
    static const uint32_t pool[8] = {
        0x00000000u, 0x80000000u,             /* +0 / -0 */
        0x7F800000u, 0xFF800000u,             /* +inf / -inf */
        0x7FC00000u, 0xFFC00001u,             /* NaN（两种编码） */
        0x3F800000u, 0x00000001u              /* 1.0 / 最小非规格化 */
    };
    uint32_t A[8], B[8];
    for (int cond = 0; cond < 6; cond++) {
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++) {
                fill(A, pool[i]); fill(B, pool[j]);
                emit_setp(cond & 3, 0, 0xFF, 1, cond, A, B);
                emit_br(cond & 3, 0, 0xFF, 0, 0, 0x4000u + (cond << 6) + (i << 3) + j);
            }
        /* 逐 lane 混排 */
        for (int l = 0; l < NLANES; l++) { A[l] = pool[l]; B[l] = pool[7 - l]; }
        emit_setp(cond & 3, 1, 0xFF, 1, cond, A, B);
        emit_br(cond & 3, 1, 0xFF, 0, 1, 0x5000u + cond);
        /* ±1.0 / ±1.5 规整数对 */
        fill(A, 0x3F800000u); fill(B, 0xBF800000u);
        emit_setp(cond & 3, 2, 0xFF, 1, cond, A, B);
        emit_br(cond & 3, 2, 0xFF, 0, 0, 0x5100u + cond);
        fill(A, 0x3FC00000u); fill(B, 0x3F800000u);
        emit_setp(cond & 3, 2, 0xFF, 1, cond, A, B);
        emit_br(cond & 3, 2, 0xFF, 0, 0, 0x5200u + cond);
        fill(A, 0x00800000u); fill(B, 0x007FFFFFu);   /* 最小规格化 vs 最大非规格化 */
        emit_setp(cond & 3, 2, 0xFF, 1, cond, A, B);
        emit_br(cond & 3, 2, 0xFF, 0, 0, 0x5300u + cond);
    }
}

// ---- 用例 5：SETP→BR 链、谓词组合与跨 warp 独立性 ----
static void build_br_chain(void)
{
    uint32_t A[8], B[8];

    /* warp0：部分掩码置谓词后，u=0 / neg / u=1 各形态读取 */
    for (int l = 0; l < NLANES; l++) { A[l] = (uint32_t)l; B[l] = 4; }
    emit_setp(0, 0, 0x0F, 0, 0, A, B);          /* pred0 = (l<4) & 0x0F = 0x07 */
    emit_br(0, 0, 0x0F, 0, 0, 0x6000u);         /* taken = 0x0F & 0x07 = 0x07 */
    emit_br(0, 0, 0x0F, 0, 1, 0x6001u);         /* taken = 0x0F & ~0x07 = 0x08 */
    emit_br(0, 0, 0xFF, 0, 0, 0x6002u);         /* 更大掩码读同一谓词 */
    emit_br(0, 0, 0xFF, 1, 0, 0x6003u);         /* u=1：taken = 掩码 */
    emit_br(0, 0, 0x00, 1, 0, 0x6004u);         /* 空掩码 */

    /* warp1：谓词未经写入（复位值 0），直接读 */
    emit_br(0, 1, 0xFF, 0, 0, 0x6100u);         /* taken = 0 */
    emit_br(0, 1, 0xFF, 0, 1, 0x6101u);         /* neg：taken = 0xFF */
    emit_br(2, 1, 0xA5, 0, 0, 0x6102u);         /* 另一 psel，仍为复位值 */

    /* warp2：掩码 0 的 SETP 不改谓词 */
    fill(A, 1); fill(B, 2);
    emit_setp(1, 2, 0x00, 0, 2, A, B);
    emit_br(1, 2, 0xFF, 0, 0, 0x6200u);         /* taken = 0（复位值） */
    emit_br(1, 2, 0xFF, 0, 1, 0x6201u);

    /* warp3：全掩码置谓词后以子掩码读（分化子集形态） */
    for (int l = 0; l < NLANES; l++) { A[l] = (uint32_t)(l * 3); B[l] = 10; }
    emit_setp(2, 3, 0xFF, 0, 5, A, B);          /* pred2 = (3l > 10) = 0xE0 */
    emit_br(2, 3, 0x33, 0, 0, 0x6300u);         /* taken = 0x33 & 0xE0 = 0x20 */
    emit_br(2, 3, 0xCC, 0, 1, 0x6301u);         /* taken = 0xCC & ~0xE0 = 0x0C */

    /* 同一谓词被多次覆写后读取 */
    fill(A, 1); fill(B, 1);
    emit_setp(3, 0, 0xFF, 0, 2, A, B);          /* pred3 = 0xFF */
    emit_setp(3, 0, 0x0F, 0, 3, A, B);          /* 低 4 位清 0：0xF0 */
    emit_br(3, 0, 0xFF, 0, 0, 0x6400u);         /* taken = 0xF0 */
}

// ---- 随机混合 ----
static void gen_random(int n)
{
    static const int ops[10] = {
        OP_IMAD, OP_IADD, OP_SHL, OP_XOR, OP_ORI,
        OP_LUI, OP_SETP, OP_BR, OP_LDP, OP_CSRR
    };
    static const uint32_t edge[12] = {
        0, 1, 0xFFFFFFFFu, 0x80000000u, 0x7FFFFFFFu, 0x0000FFFFu,
        0xFFFF0000u, 0x7F800000u, 0xFF800000u, 0x7FC00000u, 0x00000001u, 0x3F800000u
    };
    uint32_t A[8], B[8], C[8];
    for (int i = 0; i < n; i++) {
        int op = ops[rnd() % 10];
        int w = rnd() % NWARPS;
        int mask = rnd() & 0xFF;
        int rd = rnd() % 32;
        for (int l = 0; l < NLANES; l++) {
            A[l] = rnd32(); B[l] = rnd32(); C[l] = rnd32();
            if ((rnd() & 7) == 0) A[l] = edge[rnd() % 12];
            if ((rnd() & 7) == 0) B[l] = edge[rnd() % 12];
        }
        switch (op) {
        case OP_SETP:
            emit_setp(rnd() % 4, w, mask, rnd() % 2, rnd() % 6, A, B);
            if ((rnd() & 3) == 0)   /* 适时读出谓词 */
                emit_br(rnd() % 4, w, rnd() & 0xFF, rnd() % 2, rnd() % 2, rnd32() & 0x3FFFFFFFu);
            break;
        case OP_BR:
            emit_br(rnd() % 4, w, mask, rnd() % 2, rnd() % 2, rnd32() & 0x3FFFFFFFu);
            break;
        case OP_IMAD:
            emit_alu(op, rd, w, mask, A, B, C);
            break;
        case OP_LUI: case OP_LDP: case OP_CSRR:
            emit_alu(op, rd, w, mask, A, 0, 0);
            break;
        default:
            emit_alu(op, rd, w, mask, A, B, 0);
            break;
        }
    }
}

static void begin_stim(void) { txq_h = txq_t = 0; pc_seq = 0; }

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vialu *top = new Vialu;
    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("ialu.vcd");

    seed_g = 1; begin_stim(); build_op_alu();
    run_test(top, tfp, "op_alu",  0,100,0,0, 0,100,0,0, 0,100,0,0);

    seed_g = 2; begin_stim(); build_masking();
    run_test(top, tfp, "masking", 0,100,0,0, 0,100,0,0, 0,100,0,0);

    seed_g = 3; begin_stim(); build_setp_int();
    run_test(top, tfp, "setp_i",  0,100,0,0, 0,100,0,0, 0,100,0,0);

    seed_g = 4; begin_stim(); build_setp_flt();
    run_test(top, tfp, "setp_f",  2,100,3,5, 0,100,0,0, 2,100,5,7);

    seed_g = 5; begin_stim(); build_br_chain();
    run_test(top, tfp, "br_chain",1, 60,0,0, 1, 70,0,0, 1, 50,0,0);

    seed_g = 6; begin_stim(); gen_random(2000);
    run_test(top, tfp, "b2b",     0,100,0,0, 0,100,0,0, 0,100,0,0);

    seed_g = 7; begin_stim(); gen_random(4000);
    run_test(top, tfp, "rand50",  1, 50,0,0, 1, 50,0,0, 1, 50,0,0);

    seed_g = 8; begin_stim(); gen_random(4000);
    run_test(top, tfp, "rand20",  1, 20,0,0, 1, 30,0,0, 1, 25,0,0);

    seed_g = 9; begin_stim(); gen_random(3000);
    run_test(top, tfp, "pat",     2,100,3,11, 2,100,5,7, 2,100,1,15);

    seed_g = 10; begin_stim(); gen_random(10000);
    run_test(top, tfp, "soak",    1, 60,0,0, 2,100,7,3, 1, 55,0,0);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
