// =============================================================================
// easy_simt · lsu 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/lsu/rtl/lsu.sv 编译为 C++ 模型（Vlsu）；
// 本 harness 驱动时钟/复位/issue 激励，扮演 l1sm（消费请求、按随机延迟
// 呈现响应）、rf / sf / ws 三个消费者（随机背压），参考侧直接链接
// top/cmodel（lsu_step），按协议语义逐拍锁步比对五条通道的握手发生拍与
// 载荷：lsu_l1sm_req（rw/sm/mask/addr/wdata）、l1sm_lsu_rsp 消费拍、
// lsu_rf_wb（warp_id/rd/lane_mask/wdata）、lsu_sf_wbdone、
// lsu_ws_stall（warp_id/reason，含 LMISS 同拍置位 / R_NONE 空闲拍 /
// 通道在途丢弃 / R_NONE 跨指令滞留）。
//
// 波形：Verilator 原生 VCD（lsu.vcd），gtkwave 查看。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vlsu.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链，仅用 lsu 部分） ----------------
static sim_t ref;

// ---------------- 随机源（与 ialu/falu/rf harness 同式） ----------------
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
static lsu_issue_t txq[65536];
static int txq_h, txq_t;
static void tx_push(const lsu_issue_t *t) { txq[txq_t++ & 65535] = *t; }

static int errors, total_errors, cyc;
static vluint64_t gtime;   // VCD 单调时基
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- 消费者背压决策（与 ialu/falu/rf harness 同式） ----------------
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
struct Cfg { int m, p, on, off, pat; };
static Cfg cfg_req, cfg_wb, cfg_wd, cfg_st;
static int cfg_rsp_span;                    // 响应延迟 = 1 + rnd() % span
static int rdy_of(Cfg *c) { return rdy_dec(c->m, c->p, c->on, c->off, &c->pat); }

// ---------------- l1sm 响应源模型 ----------------
// 请求握手后经 1..span 拍延迟呈现响应（vld 保持至 DUT 消费）；
// 响应数据随机生成，装载时全 8 lane 拷入 wb（与 C 模型一致）
static int rsp_pend, rsp_cnt;
static uint32_t rsp_data[NLANES];

// ---------------- 在途镜像与协议保持影子 ----------------
static int mirror_busy;                     // issue_rdy 一致性检查
static int p_req_v, p_req_r, p_req_rw, p_req_sm, p_req_mask;
static uint32_t p_req_addr[NLANES], p_req_wdata[NLANES];
static int p_wb_v, p_wb_r, p_wb_warp, p_wb_rd, p_wb_mask;
static uint32_t p_wb_wdata[NLANES];
static int p_wd_v, p_wd_r, p_wd_warp, p_wd_rd;
static int p_st_v, p_st_r, p_st_warp, p_st_reason;

// ---------------- 事务计数（汇报用） ----------------
static uint64_t cnt_req, cnt_rsp, cnt_wb, cnt_wd, cnt_st, cnt_lmiss, cnt_rnone;

// ---------------- 单个用例 ----------------
static void run_test(Vlsu *top, VerilatedVcdC *tfp, const char *name)
{
    errors = 0;
    cyc = 0;
    mirror_busy = 0;
    rsp_pend = rsp_cnt = 0;
    p_req_v = p_wb_v = p_wd_v = p_st_v = 0;
    cnt_req = cnt_rsp = cnt_wb = cnt_wd = cnt_st = cnt_lmiss = cnt_rnone = 0;

    memset(&ref, 0, sizeof ref);

    // ---- 复位 ----
    top->rst_n = 0;
    top->sf_lsu_issue_vld = 0;
    top->sf_lsu_issue_opcode = 0;
    top->sf_lsu_issue_rd = 0;
    top->sf_lsu_issue_warp_id = 0;
    top->sf_lsu_issue_lane_mask = 0;
    for (int l = 0; l < NLANES; l++) {
        top->sf_lsu_issue_opa[l] = 0;
        top->sf_lsu_issue_opb[l] = 0;
    }
    top->sf_lsu_issue_imm = 0;
    top->sf_lsu_issue_shbase = 0;
    top->l1sm_lsu_rsp_vld = 0;
    for (int l = 0; l < NLANES; l++) top->l1sm_lsu_rsp_rdata[l] = 0;
    top->l1sm_lsu_req_rdy = 0;
    top->rf_lsu_wb_rdy = 0;
    top->sf_lsu_wbdone_rdy = 0;
    top->ws_lsu_stall_rdy = 0;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 5; i++) {
        if (top->lsu_l1sm_req_vld || top->lsu_rf_wb_vld ||
            top->lsu_sf_wbdone_vld || top->lsu_ws_stall_vld)
            err("vld not 0 during reset");
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    }
    top->rst_n = 1;

    int ntx = txq_t - txq_h;
    int cap = ntx * 60 + 4000;
    int finished = 0;
    while (!finished && cyc < cap) {
        // -- 消费者决策（req/wb/wbdone/stall 四路独立） --
        int dr_req = rdy_of(&cfg_req);
        int dr_wb  = rdy_of(&cfg_wb);
        int dr_wd  = rdy_of(&cfg_wd);
        int dr_st  = rdy_of(&cfg_st);

        // -- l1sm 响应源：倒计时到达则呈现（保持至 DUT 消费） --
        if (rsp_pend && rsp_cnt > 0) rsp_cnt--;
        int rsp_offer = rsp_pend && (rsp_cnt == 0);

        // -- 激励：队列头持续给出直至握手（vld 保持，协议 §9 条 1） --
        int offer = (txq_h < txq_t);
        const lsu_issue_t *tx = offer ? &txq[txq_h & 65535] : 0;

        top->sf_lsu_issue_vld = offer;
        if (offer) {
            top->sf_lsu_issue_opcode = tx->opcode;
            top->sf_lsu_issue_rd = tx->rd;
            top->sf_lsu_issue_warp_id = tx->warp_id;
            top->sf_lsu_issue_lane_mask = tx->lane_mask;
            for (int l = 0; l < NLANES; l++) {
                top->sf_lsu_issue_opa[l] = tx->opa[l];
                top->sf_lsu_issue_opb[l] = tx->opb[l];
            }
            top->sf_lsu_issue_imm = tx->imm;
            top->sf_lsu_issue_shbase = tx->shbase;
        }
        top->l1sm_lsu_rsp_vld = rsp_offer;
        if (rsp_offer)
            for (int l = 0; l < NLANES; l++) top->l1sm_lsu_rsp_rdata[l] = rsp_data[l];
        top->l1sm_lsu_req_rdy = dr_req;
        top->rf_lsu_wb_rdy = dr_wb;
        top->sf_lsu_wbdone_rdy = dr_wd;
        top->ws_lsu_stall_rdy = dr_st;
        top->eval();

        // -- 协议保持检查：上一拍 vld && !rdy，则本拍 vld 不撤、载荷不变 --
        if (p_req_v && !p_req_r) {
            if (!top->lsu_l1sm_req_vld)
                err("req vld dropped under backpressure");
            else if ((int)top->lsu_l1sm_req_rw != p_req_rw ||
                     (int)top->lsu_l1sm_req_sm != p_req_sm ||
                     (int)top->lsu_l1sm_req_mask != p_req_mask)
                err("req payload header changed under backpressure");
            else
                for (int l = 0; l < NLANES; l++)
                    if (top->lsu_l1sm_req_addr[l] != p_req_addr[l] ||
                        top->lsu_l1sm_req_wdata[l] != p_req_wdata[l]) {
                        err("req payload changed under backpressure");
                        break;
                    }
        }
        if (p_wb_v && !p_wb_r) {
            if (!top->lsu_rf_wb_vld)
                err("wb vld dropped under backpressure");
            else if ((int)top->lsu_rf_wb_warp_id != p_wb_warp ||
                     (int)top->lsu_rf_wb_rd != p_wb_rd ||
                     (int)top->lsu_rf_wb_lane_mask != p_wb_mask)
                err("wb payload header changed under backpressure");
            else
                for (int l = 0; l < NLANES; l++)
                    if (top->lsu_rf_wb_wdata[l] != p_wb_wdata[l]) {
                        err("wb wdata changed under backpressure");
                        break;
                    }
        }
        if (p_wd_v && !p_wd_r) {
            if (!top->lsu_sf_wbdone_vld)
                err("wbdone vld dropped under backpressure");
            else if ((int)top->lsu_sf_wbdone_warp_id != p_wd_warp ||
                     (int)top->lsu_sf_wbdone_rd != p_wd_rd)
                err("wbdone payload changed under backpressure");
        }
        if (p_st_v && !p_st_r) {
            if (!top->lsu_ws_stall_vld)
                err("stall vld dropped under backpressure");
            else if ((int)top->lsu_ws_stall_warp_id != p_st_warp ||
                     (int)top->lsu_ws_stall_reason != p_st_reason)
                err("stall payload changed under backpressure");
        }

        // -- 本拍末沿将发生的发射（输出为寄存器值或组合置位项，边沿前稳定） --
        int d_acc = offer && top->lsu_sf_issue_rdy;
        int d_req = top->lsu_l1sm_req_vld && dr_req;
        int d_rsp = top->l1sm_lsu_rsp_vld && top->lsu_l1sm_rsp_rdy;
        int d_wb  = top->lsu_rf_wb_vld && dr_wb;
        int d_wd  = top->lsu_sf_wbdone_vld && dr_wd;
        int d_st  = top->lsu_ws_stall_vld && dr_st;
        if (d_req + d_wb + d_wd > 1)
            err("multiple state-decoded channels fire in one cycle");
        if (top->lsu_sf_issue_rdy != !mirror_busy)
            err("issue_rdy inconsistent with in-flight state");

        // -- 参考同拍推进（同激励、同背压；先置激励，后 lsu_step，再按 rdy 消费） --
        if (offer && !ref.sf_lsu_issue.vld) {
            ref.sf_lsu_issue.p = *tx;
            ref.sf_lsu_issue.vld = 1;
        }
        if (rsp_offer && !ref.l1sm_lsu_rsp.vld) {
            for (int l = 0; l < NLANES; l++)
                ref.l1sm_lsu_rsp.p.rdata[l] = rsp_data[l];
            ref.l1sm_lsu_rsp.vld = 1;
        }
        int had_rsp = ref.l1sm_lsu_rsp.vld;
        int had_iss = ref.sf_lsu_issue.vld;
        lsu_step(&ref);
        int r_acc = had_iss && !ref.sf_lsu_issue.vld;
        int r_rsp = had_rsp && !ref.l1sm_lsu_rsp.vld;

        // -- 参考输出按同拍 rdy 消费，载荷位精确比对 --
        int r_req = 0;
        if (ref.lsu_l1sm_req.vld && dr_req) {
            if ((int)top->lsu_l1sm_req_rw != ref.lsu_l1sm_req.p.rw)
                err("req rw mismatch");
            if ((int)top->lsu_l1sm_req_sm != ref.lsu_l1sm_req.p.sm)
                err("req sm mismatch");
            if ((int)top->lsu_l1sm_req_mask != ref.lsu_l1sm_req.p.mask)
                err("req mask mismatch");
            for (int l = 0; l < NLANES; l++) {
                if (top->lsu_l1sm_req_addr[l] != ref.lsu_l1sm_req.p.addr[l]) {
                    err("req addr mismatch"); break;
                }
            }
            for (int l = 0; l < NLANES; l++) {
                if (top->lsu_l1sm_req_wdata[l] != ref.lsu_l1sm_req.p.wdata[l]) {
                    err("req wdata mismatch"); break;
                }
            }
            ref.lsu_l1sm_req.vld = 0;
            r_req = 1;
        }
        int r_wb = 0;
        if (ref.lsu_rf_wb.vld && dr_wb) {
            if ((int)top->lsu_rf_wb_warp_id != ref.lsu_rf_wb.p.warp_id)
                err("wb warp_id mismatch");
            if ((int)top->lsu_rf_wb_rd != ref.lsu_rf_wb.p.rd)
                err("wb rd mismatch");
            if ((int)top->lsu_rf_wb_lane_mask != ref.lsu_rf_wb.p.lane_mask)
                err("wb lane_mask mismatch");
            for (int l = 0; l < NLANES; l++) {
                if (top->lsu_rf_wb_wdata[l] != ref.lsu_rf_wb.p.wdata[l]) {
                    err("wb wdata mismatch"); break;
                }
            }
            ref.lsu_rf_wb.vld = 0;
            r_wb = 1;
        }
        int r_wd = 0;
        if (ref.lsu_sf_wbdone.vld && dr_wd) {
            if ((int)top->lsu_sf_wbdone_warp_id != ref.lsu_sf_wbdone.p.warp_id)
                err("wbdone warp_id mismatch");
            if ((int)top->lsu_sf_wbdone_rd != ref.lsu_sf_wbdone.p.rd)
                err("wbdone rd mismatch");
            ref.lsu_sf_wbdone.vld = 0;
            r_wd = 1;
        }
        int r_st = 0;
        if (ref.lsu_ws_stall.vld && dr_st) {
            if ((int)top->lsu_ws_stall_warp_id != ref.lsu_ws_stall.p.warp_id)
                err("stall warp_id mismatch");
            if ((int)top->lsu_ws_stall_reason != ref.lsu_ws_stall.p.reason)
                err("stall reason mismatch");
            if (ref.lsu_ws_stall.p.reason == R_LMISS) cnt_lmiss++;
            else if (ref.lsu_ws_stall.p.reason == R_NONE) cnt_rnone++;
            ref.lsu_ws_stall.vld = 0;
            r_st = 1;
        }

        // -- 逐拍握手一致性 --
        if (r_acc != d_acc) err("issue handshake divergence between DUT and ref");
        if (r_req != d_req) err("req handshake divergence between DUT and ref");
        if (r_rsp != d_rsp) err("rsp consumption divergence between DUT and ref");
        if (r_wb  != d_wb)  err("wb handshake divergence between DUT and ref");
        if (r_wd  != d_wd)  err("wbdone handshake divergence between DUT and ref");
        if (r_st  != d_st)  err("stall handshake divergence between DUT and ref");

        // -- 记账：响应调度 / 队列推进 --
        if (d_req) {
            rsp_pend = 1;
            rsp_cnt = 1 + rnd() % cfg_rsp_span;
            for (int l = 0; l < NLANES; l++) rsp_data[l] = rnd32();
            cnt_req++;
        }
        if (d_rsp) { rsp_pend = 0; cnt_rsp++; }
        if (d_acc) { mirror_busy = 1; txq_h++; }
        if (d_wb)  cnt_wb++;
        if (d_wd)  { mirror_busy = 0; cnt_wd++; }
        if (d_st)  cnt_st++;

        // -- 影子更新 --
        p_req_v = top->lsu_l1sm_req_vld; p_req_r = dr_req;
        p_req_rw = top->lsu_l1sm_req_rw;
        p_req_sm = top->lsu_l1sm_req_sm;
        p_req_mask = top->lsu_l1sm_req_mask;
        for (int l = 0; l < NLANES; l++) {
            p_req_addr[l] = top->lsu_l1sm_req_addr[l];
            p_req_wdata[l] = top->lsu_l1sm_req_wdata[l];
        }
        p_wb_v = top->lsu_rf_wb_vld; p_wb_r = dr_wb;
        p_wb_warp = top->lsu_rf_wb_warp_id;
        p_wb_rd = top->lsu_rf_wb_rd;
        p_wb_mask = top->lsu_rf_wb_lane_mask;
        for (int l = 0; l < NLANES; l++) p_wb_wdata[l] = top->lsu_rf_wb_wdata[l];
        p_wd_v = top->lsu_sf_wbdone_vld; p_wd_r = dr_wd;
        p_wd_warp = top->lsu_sf_wbdone_warp_id;
        p_wd_rd = top->lsu_sf_wbdone_rd;
        p_st_v = top->lsu_ws_stall_vld; p_st_r = dr_st;
        p_st_warp = top->lsu_ws_stall_warp_id;
        p_st_reason = top->lsu_ws_stall_reason;

        // -- 时钟上升沿 --
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (txq_h == txq_t && !mirror_busy && !rsp_pend &&
            !top->lsu_l1sm_req_vld && !top->lsu_rf_wb_vld &&
            !top->lsu_sf_wbdone_vld && !top->lsu_ws_stall_vld &&
            !ref.sf_lsu_issue.vld && !ref.lsu_l1sm_req.vld &&
            !ref.l1sm_lsu_rsp.vld && !ref.lsu_rf_wb.vld &&
            !ref.lsu_sf_wbdone.vld && !ref.lsu_ws_stall.vld &&
            !ref.lsu.busy && !ref.lsu.stall_sent)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (txq_h != txq_t) err("stimulus not drained");
    if (rsp_pend) err("rsp not consumed at end");
    if (ref.lsu.busy || ref.lsu.req_stage) err("ref still busy at end");
    if (ref.lsu.stall_sent) err("ref stall clear pending at end");
    if (ref.err) err("ref reported error");

    total_errors += errors;
    printf("[vsim] %-10s n=%-6d req=%-6llu rsp=%-6llu wb=%-6llu wd=%-6llu "
           "st=%-6llu(lmiss=%llu,rnone=%llu) cyc=%-8d -> %s\n",
           name, ntx,
           (unsigned long long)cnt_req, (unsigned long long)cnt_rsp,
           (unsigned long long)cnt_wb, (unsigned long long)cnt_wd,
           (unsigned long long)cnt_st,
           (unsigned long long)cnt_lmiss, (unsigned long long)cnt_rnone,
           cyc, errors ? "FAIL" : "PASS");
}

// ============================================================================
//  激励生成
// ============================================================================
static void begin_stim(void) { txq_h = txq_t = 0; }

static void emit(int op, int rd, int w, int mask, uint32_t imm, uint32_t shbase,
                 const uint32_t *opa, const uint32_t *opb)
{
    lsu_issue_t t;
    memset(&t, 0, sizeof t);
    t.opcode = op; t.rd = rd; t.warp_id = w; t.lane_mask = (uint8_t)mask;
    t.imm = imm; t.shbase = shbase;
    for (int l = 0; l < NLANES; l++) {
        t.opa[l] = opa ? opa[l] : 0;
        t.opb[l] = opb ? opb[l] : 0;
    }
    tx_push(&t);
}

// ---- 用例 1：LDG 顺序（全掩码、无背压） ----
static void build_ldg(void)
{
    uint32_t a[NLANES], b[NLANES];
    for (int i = 0; i < 64; i++) {
        for (int l = 0; l < NLANES; l++) {
            a[l] = ((uint32_t)l * 16u + (uint32_t)i * 64u) & ~3u;
            b[l] = rnd32() & ~3u;              /* LDG 不读取 opb */
        }
        emit(OP_LDG, 1 + (i % 31), i & 3, 0xFF, (uint32_t)i * 4u, 0, a, b);
    }
}

// ---- 用例 2：STG 顺序（全掩码、无背压） ----
static void build_stg(void)
{
    uint32_t a[NLANES], b[NLANES];
    for (int i = 0; i < 64; i++) {
        for (int l = 0; l < NLANES; l++) {
            a[l] = 0xD00D0000u ^ ((uint32_t)i << 8) ^ (uint32_t)l;  /* 数据任意 */
            b[l] = ((uint32_t)l * 32u + (uint32_t)i * 128u) & ~3u;  /* 偏移对齐 */
        }
        emit(OP_STG, (i % 16 == 0) ? 0 : 1 + (i % 31), i & 3, 0xFF,
             (uint32_t)i * 4u, 0, a, b);
    }
}

// ---- 用例 3：LDS/STS 混合（shbase 变化、无背压） ----
static void build_shmem(void)
{
    uint32_t a[NLANES], b[NLANES];
    for (int i = 0; i < 128; i++) {
        uint32_t shb = ((uint32_t)(i % 7) * 256u) & ~3u;
        int lds = (i % 2) == 0;
        for (int l = 0; l < NLANES; l++) {
            b[l] = ((uint32_t)l * 8u + (uint32_t)i * 24u) & ~3u;    /* 偏移 */
            a[l] = lds ? (((uint32_t)l * 12u + (uint32_t)i * 48u) & ~3u)
                       : (0xBEEF0000u ^ (uint32_t)i ^ (uint32_t)l); /* STS 数据 */
        }
        emit(lds ? OP_LDS : OP_STS, 1 + (i % 31), i & 3, 0xFF,
             0, shb, a, b);
    }
}

// ---- 用例 4：lane 掩码（含全 0/单 bit/交替/全 1） ----
static void build_masking(void)
{
    static const int masks[12] = {
        0x00, 0x01, 0x02, 0x80, 0x55, 0xAA, 0x0F, 0xF0, 0x7E, 0x81, 0xC3, 0xFF
    };
    uint32_t a[NLANES], b[NLANES];
    for (int i = 0; i < 12; i++) {
        int op = OP_LDG + (i % 4);
        uint32_t shb = ((uint32_t)i * 512u) & ~3u;
        for (int l = 0; l < NLANES; l++) {
            a[l] = ((uint32_t)l * 4u + (uint32_t)i * 32u) & ~3u;
            b[l] = ((uint32_t)l * 4u + 7u + (uint32_t)i * 32u) & ~3u;
        }
        emit(op, 1 + (i % 31), i & 3, masks[i],
             (op == OP_LDS || op == OP_STS) ? 0 : (uint32_t)i * 4u, shb, a, b);
        /* 掩码全 0：请求仍发出（addr/wdata 全 0） */
        if (i == 0)
            emit(OP_LDS, 5, 1, 0x00, 0, shb, a, b);
    }
}

// ---- 用例 5：边界值（0 地址 / 最大对齐值 / 32 位回绕 / rd=0） ----
static void build_edge(void)
{
    uint32_t z[NLANES], mx[NLANES], wp[NLANES], dd[NLANES];
    for (int l = 0; l < NLANES; l++) {
        z[l] = 0;
        mx[l] = 0xFFFFFFFCu;
        wp[l] = 0xFFFFFFF8u;                   /* +8 回绕到 4 */
        dd[l] = (l & 1) ? 0xFFFFFFFFu : 0x80000000u;   /* 数据极值 */
    }
    emit(OP_LDG, 1, 0, 0xFF, 0, 0, z, z);           /* 全 0 地址 */
    emit(OP_LDG, 2, 0, 0xFF, 0xFFFFFFFCu, 0, mx, z);/* 最大对齐基址+偏移 */
    emit(OP_STG, 3, 0, 0xFF, 8u, 0, dd, wp);        /* 偏移回绕 */
    emit(OP_LDS, 0, 0, 0xFF, 0, 0xFFFFFFFCu, mx, z);/* rd=0 + 最大 shbase */
    emit(OP_STS, 0, 0, 0xFF, 0, 0, dd, z);          /* rd=0 + shbase=0 */
    emit(OP_LDG, 31, 3, 0x81, 0, 0, z, z);          /* 掩码 0x81 + 全 0 */
    emit(OP_STG, 30, 3, 0x7E, 4u, 0, dd, wp);
    emit(OP_LDS, 29, 1, 0xFF, 0, 4u, mx, z);        /* shbase=4 + 0xFFFFFFFC */
    emit(OP_STS, 28, 2, 0xFF, 0, 0xFFFFFFFCu, dd, mx);
    for (int i = 0; i < 60; i++) {
        uint32_t a[NLANES], b[NLANES];
        for (int l = 0; l < NLANES; l++) {
            a[l] = rnd32() & ~3u;
            b[l] = rnd32() & ~3u;
        }
        emit(OP_LDG + (rnd() % 4), (rnd() % 40 < 32) ? 1 + rnd() % 31 : 0,
             rnd() % NWARPS, rnd() & 0xFF, rnd32() & ~3u, rnd32() & ~3u, a, b);
    }
}

// ---- 随机混合 ----
static void gen_random(int n)
{
    for (int i = 0; i < n; i++) {
        int op = OP_LDG + (rnd() % 4);
        int w = rnd() % NWARPS;
        int rd = (rnd() % 40 < 32) ? (rnd() % 32) : 0;   /* 少量 R0 */
        if (rd == 0 && (rnd() % 8)) rd = 1 + rnd() % 31;
        int mask = rnd() & 0xFF;
        uint32_t imm = rnd32() & ~3u;
        uint32_t shb = rnd32() & ~3u;
        uint32_t a[NLANES], b[NLANES];
        for (int l = 0; l < NLANES; l++) {
            /* opa：存储为数据（任意）、装载为偏移（对齐）；
               opb：存储为偏移（对齐）、装载不被读取（任意） */
            a[l] = (op == OP_STG || op == OP_STS) ? rnd32()
                                                  : (rnd32() & ~3u);
            b[l] = (op == OP_STG || op == OP_STS) ? (rnd32() & ~3u)
                                                  : rnd32();
        }
        emit(op, rd, w, mask, imm, shb, a, b);
    }
}

// ---- 用例配置辅助 ----
static void cfg_all(int qm,int qp, int wm,int wp, int dm,int dp,
                    int sm,int sp, int span)
{
    cfg_req.m = qm; cfg_req.p = qp; cfg_req.on = cfg_req.off = 0; cfg_req.pat = 0;
    cfg_wb.m  = wm; cfg_wb.p  = wp; cfg_wb.on  = cfg_wb.off  = 0; cfg_wb.pat  = 0;
    cfg_wd.m  = dm; cfg_wd.p  = dp; cfg_wd.on  = cfg_wd.off  = 0; cfg_wd.pat  = 0;
    cfg_st.m  = sm; cfg_st.p  = sp; cfg_st.on  = cfg_st.off  = 0; cfg_st.pat  = 0;
    cfg_rsp_span = span;
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vlsu *top = new Vlsu;
    VerilatedVcdC *tfp = new VerilatedVcdC;
    Verilated::traceEverOn(true);
    top->trace(tfp, 99);
    tfp->open("lsu.vcd");

    seed_g = 1; begin_stim(); build_ldg();
    cfg_all(0,100, 0,100, 0,100, 0,100, 1);
    run_test(top, tfp, "ldg");

    seed_g = 2; begin_stim(); build_stg();
    cfg_all(0,100, 0,100, 0,100, 0,100, 1);
    run_test(top, tfp, "stg");

    seed_g = 3; begin_stim(); build_shmem();
    cfg_all(0,100, 0,100, 0,100, 0,100, 2);
    run_test(top, tfp, "shmem");

    seed_g = 4; begin_stim(); build_masking();
    cfg_all(0,100, 0,100, 0,100, 0,100, 2);
    run_test(top, tfp, "masking");

    seed_g = 5; begin_stim(); build_edge();
    cfg_all(0,100, 0,100, 0,100, 0,100, 2);
    run_test(top, tfp, "edge");

    seed_g = 6; begin_stim(); gen_random(2000);
    cfg_all(0,100, 0,100, 0,100, 1, 70, 2);
    run_test(top, tfp, "b2b");

    seed_g = 7; begin_stim(); gen_random(4000);
    cfg_all(1, 50, 1, 50, 1, 50, 1, 50, 4);
    run_test(top, tfp, "rand50");

    seed_g = 8; begin_stim(); gen_random(4000);
    cfg_all(1, 20, 1, 20, 1, 20, 1, 30, 8);
    run_test(top, tfp, "rand20");

    // 停顿通道重背压：其余全通，制造 R_NONE 置位与下次发射同拍碰撞 /
    // LMISS 丢弃 / R_NONE 跨指令滞留
    seed_g = 9; begin_stim(); gen_random(2000);
    cfg_all(0,100, 0,100, 0,100, 1, 15, 2);
    run_test(top, tfp, "staldo");

    seed_g = 10; begin_stim(); gen_random(10000);
    cfg_all(1, 60, 1, 60, 1, 60, 1, 60, 16);
    run_test(top, tfp, "soak");

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
