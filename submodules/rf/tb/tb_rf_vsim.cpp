// =============================================================================
// easy_simt · rf 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/rf/rtl/rf.sv 编译为 C++ 模型（Vrf）；
// 本 harness 驱动时钟/复位、三写口激励与读请求队列、应答消费者背压，
// 参考侧直接链接 top/cmodel（rf_step），按协议语义逐拍锁步比对：
// 三源写仲裁（每拍至多一笔、lsu > ialu > falu）、读应答顺序与载荷位精确、
// 同拍写读（含同地址旁路）与参考写先读后一致。
//
// 波形：Verilator 原生 VCD（rf.vcd），gtkwave 查看。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vrf.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链，仅用 rf 部分） ----------------
static sim_t ref;

// ---------------- 随机源（与 falu harness 同式） ----------------
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

// ---------------- 激励队列（写口三源 + 读请求，vld 保持至握手） ----------------
// 写源编号：0 = lsu，1 = ialu，2 = falu
static rf_rd_t rdq[65536];
static int rdq_h, rdq_t;
static wb_t wbq[3][65536];
static int wbq_h[3], wbq_t[3];

static void emit_rd(int w, int rs1, int rs2)
{
    rf_rd_t *t = &rdq[rdq_t++ & 65535];
    t->warp_id = w; t->rs1 = rs1; t->rs2 = rs2;
}
static void emit_wb(int src, int w, int rd, int mask, const uint32_t *d)
{
    wb_t *t = &wbq[src][wbq_t[src]++ & 65535];
    t->warp_id = w; t->rd = rd; t->lane_mask = (uint8_t)mask;
    for (int l = 0; l < NLANES; l++) t->wdata[l] = d ? d[l] : 0;
}

static int errors, total_errors, cyc;
static vluint64_t gtime;   // VCD 单调时基
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- 事务计数（汇报用） ----------------
static uint64_t cnt_rd, cnt_rsp, cnt_wb[3];

// ---------------- 应答消费者背压决策（与 falu harness 同式） ----------------
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
static int rsp_mode, rsp_pct, rsp_on, rsp_off, rsp_pat;

// ---------------- 应答保持影子（协议检查用） ----------------
static int p_rsp_v, p_rsp_r;
static uint32_t p_rsp_a[NLANES], p_rsp_b[NLANES];

// ---------------- DUT 写口驱动/读取辅助 ----------------
static void drv_wb(Vrf *top, int s, int vld, const wb_t *t)
{
    if (s == 0) {
        top->lsu_rf_wb_vld = vld;
        if (vld) {
            top->lsu_rf_wb_warp_id = t->warp_id;
            top->lsu_rf_wb_rd = t->rd;
            top->lsu_rf_wb_lane_mask = t->lane_mask;
            for (int l = 0; l < NLANES; l++) top->lsu_rf_wb_wdata[l] = t->wdata[l];
        } else {
            top->lsu_rf_wb_warp_id = 0; top->lsu_rf_wb_rd = 0;
            top->lsu_rf_wb_lane_mask = 0;
            for (int l = 0; l < NLANES; l++) top->lsu_rf_wb_wdata[l] = 0;
        }
    } else if (s == 1) {
        top->ialu_rf_wb_vld = vld;
        if (vld) {
            top->ialu_rf_wb_warp_id = t->warp_id;
            top->ialu_rf_wb_rd = t->rd;
            top->ialu_rf_wb_lane_mask = t->lane_mask;
            for (int l = 0; l < NLANES; l++) top->ialu_rf_wb_wdata[l] = t->wdata[l];
        } else {
            top->ialu_rf_wb_warp_id = 0; top->ialu_rf_wb_rd = 0;
            top->ialu_rf_wb_lane_mask = 0;
            for (int l = 0; l < NLANES; l++) top->ialu_rf_wb_wdata[l] = 0;
        }
    } else {
        top->falu_rf_wb_vld = vld;
        if (vld) {
            top->falu_rf_wb_warp_id = t->warp_id;
            top->falu_rf_wb_rd = t->rd;
            top->falu_rf_wb_lane_mask = t->lane_mask;
            for (int l = 0; l < NLANES; l++) top->falu_rf_wb_wdata[l] = t->wdata[l];
        } else {
            top->falu_rf_wb_warp_id = 0; top->falu_rf_wb_rd = 0;
            top->falu_rf_wb_lane_mask = 0;
            for (int l = 0; l < NLANES; l++) top->falu_rf_wb_wdata[l] = 0;
        }
    }
}
static int dut_wb_rdy(Vrf *top, int s)
{
    if (s == 0) return top->rf_lsu_wb_rdy;
    if (s == 1) return top->rf_ialu_wb_rdy;
    return top->rf_falu_wb_rdy;
}
static int dut_wb_vld(Vrf *top, int s)
{
    if (s == 0) return top->lsu_rf_wb_vld;
    if (s == 1) return top->ialu_rf_wb_vld;
    return top->falu_rf_wb_vld;
}
static chan_wb_t *ref_wbchan(int s)
{
    if (s == 0) return &ref.lsu_rf_wb;
    if (s == 1) return &ref.ialu_rf_wb;
    return &ref.falu_rf_wb;
}

// ---------------- 单个用例 ----------------
static void run_test(Vrf *top, VerilatedVcdC *tfp, const char *name,
                     int rspm, int rspp, int rspo, int rspf)
{
    rsp_mode = rspm; rsp_pct = rspp; rsp_on = rspo; rsp_off = rspf;
    rsp_pat = 0;
    errors = 0;
    cnt_rd = cnt_rsp = 0;
    cnt_wb[0] = cnt_wb[1] = cnt_wb[2] = 0;
    p_rsp_v = p_rsp_r = 0;
    cyc = 0;

    memset(&ref, 0, sizeof ref);

    // ---- 复位 ----
    top->rst_n = 0;
    top->sf_rf_rd_vld = 0;
    top->sf_rf_rd_warp_id = 0;
    top->sf_rf_rd_rs1 = 0;
    top->sf_rf_rd_rs2 = 0;
    top->sf_rf_rddata_rdy = 0;
    for (int s = 0; s < 3; s++) drv_wb(top, s, 0, 0);
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    // 复位保持 70 拍：SRAM 宏版在复位期间做宏清零扫描（两口并行，共 64 拍），
    // 保持 ≥64 拍方能扫全阵（寄存器版不受影响）
    for (int i = 0; i < 70; i++) {
        if (top->rf_sf_rddata_vld)
            err("rddata vld not 0 during reset");
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    }
    top->rst_n = 1;

    int nops = (rdq_t - rdq_h)
             + (wbq_t[0] - wbq_h[0]) + (wbq_t[1] - wbq_h[1])
             + (wbq_t[2] - wbq_h[2]);
    int cap = nops * 40 + 2000;
    int finished = 0;
    while (!finished && cyc < cap) {
        // -- 消费者决策 --
        int r = rdy_dec(rsp_mode, rsp_pct, rsp_on, rsp_off, &rsp_pat);

        // -- 激励：各队列头持续给出直至握手（vld 保持，协议 §9 条 1/2） --
        int offer_rd = (rdq_h < rdq_t);
        const rf_rd_t *txr = offer_rd ? &rdq[rdq_h & 65535] : 0;
        int offer_wb[3];
        for (int s = 0; s < 3; s++) {
            offer_wb[s] = (wbq_h[s] < wbq_t[s]);
            drv_wb(top, s, offer_wb[s],
                   offer_wb[s] ? &wbq[s][wbq_h[s] & 65535] : 0);
        }
        top->sf_rf_rd_vld = offer_rd;
        if (offer_rd) {
            top->sf_rf_rd_warp_id = txr->warp_id;
            top->sf_rf_rd_rs1 = txr->rs1;
            top->sf_rf_rd_rs2 = txr->rs2;
        } else {
            top->sf_rf_rd_warp_id = 0;
            top->sf_rf_rd_rs1 = 0;
            top->sf_rf_rd_rs2 = 0;
        }
        top->sf_rf_rddata_rdy = r;
        top->eval();

        // -- 锁步不变量：参考应答在途 == DUT 应答在途 --
        if (ref.rf_sf_rddata.vld != (int)top->rf_sf_rddata_vld)
            err("rddata vld divergence between DUT and ref");

        // -- 协议保持检查：上一拍 vld && !rdy，则本拍 vld 不撤、载荷不变 --
        if (p_rsp_v && !p_rsp_r) {
            if (!top->rf_sf_rddata_vld)
                err("rddata vld dropped under backpressure");
            else
                for (int l = 0; l < NLANES; l++)
                    if (top->rf_sf_rddata_a[l] != p_rsp_a[l] ||
                        top->rf_sf_rddata_b[l] != p_rsp_b[l]) {
                        err("rddata payload changed under backpressure");
                        break;
                    }
        }

        // -- rdy 组合函数检查（§2 模块补充） --
        if ((int)top->rf_sf_rd_rdy != (!top->rf_sf_rddata_vld || r))
            err("rd_rdy inconsistent with in-flight response");
        if (!top->rf_lsu_wb_rdy)
            err("lsu wb rdy not always 1");
        if ((int)top->rf_ialu_wb_rdy != !top->lsu_rf_wb_vld)
            err("ialu wb rdy violates priority");
        if ((int)top->rf_falu_wb_rdy != (!top->lsu_rf_wb_vld && !top->ialu_rf_wb_vld))
            err("falu wb rdy violates priority");

        // -- 本拍末沿将发生的发射 --
        int d_rdacc = offer_rd && top->rf_sf_rd_rdy;
        int d_wb[3], d_nwb = 0;
        for (int s = 0; s < 3; s++) {
            d_wb[s] = offer_wb[s] && dut_wb_rdy(top, s);
            d_nwb += d_wb[s];
        }
        if (d_nwb > 1)
            err("multiple write handshakes in one cycle");
        int d_rsp = top->rf_sf_rddata_vld && r;

        // -- 参考推进（先消费应答，再置激励，后 rf_step；同激励、同背压） --
        int r_rsp = 0;
        if (ref.rf_sf_rddata.vld && r) {
            r_rsp = 1;
            // 载荷位精确比对（a/b 逐 lane）
            for (int l = 0; l < NLANES; l++) {
                if (ref.rf_sf_rddata.p.a[l] != top->rf_sf_rddata_a[l]) {
                    err("rddata a mismatch"); break;
                }
            }
            for (int l = 0; l < NLANES; l++) {
                if (ref.rf_sf_rddata.p.b[l] != top->rf_sf_rddata_b[l]) {
                    err("rddata b mismatch"); break;
                }
            }
            ref.rf_sf_rddata.vld = 0;
            cnt_rsp++;
        }
        if (r_rsp != d_rsp)
            err("rddata handshake divergence between DUT and ref");

        if (offer_rd && !ref.sf_rf_rd.vld) {
            ref.sf_rf_rd.p = *txr;
            ref.sf_rf_rd.vld = 1;
        }
        int wb_before[3];
        for (int s = 0; s < 3; s++) {
            chan_wb_t *c = ref_wbchan(s);
            if (offer_wb[s] && !c->vld) {
                c->p = wbq[s][wbq_h[s] & 65535];
                c->vld = 1;
            }
            if (!offer_wb[s] && c->vld)
                err("ref wb vld set without stimulus");
            wb_before[s] = c->vld;
        }
        int rd_before = ref.sf_rf_rd.vld;

        rf_step(&ref);

        // -- 握手逐拍一致性：写口（含仲裁结果）与读请求 --
        int r_rdacc = rd_before && !ref.sf_rf_rd.vld;
        if (r_rdacc != d_rdacc)
            err("rd request handshake divergence between DUT and ref");
        if (d_rdacc) { cnt_rd++; rdq_h++; }
        for (int s = 0; s < 3; s++) {
            int r_wb = wb_before[s] && !ref_wbchan(s)->vld;
            if (r_wb != d_wb[s])
                err("wb handshake divergence between DUT and ref");
            if (d_wb[s]) { cnt_wb[s]++; wbq_h[s]++; }
        }

        // -- 影子更新 --
        p_rsp_v = top->rf_sf_rddata_vld; p_rsp_r = r;
        for (int l = 0; l < NLANES; l++) {
            p_rsp_a[l] = top->rf_sf_rddata_a[l];
            p_rsp_b[l] = top->rf_sf_rddata_b[l];
        }

        // -- 时钟上升沿 --
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (rdq_h == rdq_t &&
            wbq_h[0] == wbq_t[0] && wbq_h[1] == wbq_t[1] && wbq_h[2] == wbq_t[2] &&
            !ref.sf_rf_rd.vld &&
            !ref.lsu_rf_wb.vld && !ref.ialu_rf_wb.vld && !ref.falu_rf_wb.vld &&
            !ref.rf_sf_rddata.vld && !top->rf_sf_rddata_vld)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (rdq_h != rdq_t) err("rd stimulus not drained");
    for (int s = 0; s < 3; s++)
        if (wbq_h[s] != wbq_t[s]) err("wb stimulus not drained");
    if (ref.rf_sf_rddata.vld) err("ref rddata pending at end");
    if (top->rf_sf_rddata_vld) err("DUT rddata pending at end");
    if (ref.err) err("ref reported error");

    total_errors += errors;
    printf("[vsim] %-10s rd=%-6llu rsp=%-6llu wr=%llu/%llu/%llu cyc=%-8d -> %s\n",
           name,
           (unsigned long long)cnt_rd, (unsigned long long)cnt_rsp,
           (unsigned long long)cnt_wb[0], (unsigned long long)cnt_wb[1],
           (unsigned long long)cnt_wb[2],
           cyc, errors ? "FAIL" : "PASS");
}

// ============================================================================
//  激励生成
// ============================================================================
static void begin_stim(void)
{
    rdq_h = rdq_t = 0;
    for (int s = 0; s < 3; s++) wbq_h[s] = wbq_t[s] = 0;
}

static void fill_seq(uint32_t *d, uint32_t base)
{
    for (int l = 0; l < NLANES; l++) d[l] = base ^ ((uint32_t)(l + 1) * 0x01010101u);
}

// ---- 用例 1：三源顺序写满 + 逐笔读回（全掩码、无背压） ----
static void build_wr_seq(void)
{
    uint32_t d[NLANES];
    for (int src = 0; src < 3; src++)
        for (int w = 0; w < NWARPS; w++)
            for (int rd = 1; rd < 32; rd++) {
                fill_seq(d, ((uint32_t)src << 28) ^ ((uint32_t)w << 24)
                            ^ ((uint32_t)rd << 16) ^ 0x5A5A5A5Au);
                emit_wb(src, w, rd, 0xFF, d);
                emit_rd(w, rd, rd);
            }
}

// ---- 用例 2：R0 恒零与未写寄存器 ----
static void build_r0_zero(void)
{
    uint32_t d[NLANES];
    /* 对 R0 写任意数据，均被忽略 */
    for (int src = 0; src < 3; src++)
        for (int w = 0; w < NWARPS; w++) {
            fill_seq(d, 0xDEAD0000u ^ ((uint32_t)src << 20) ^ ((uint32_t)w << 16));
            emit_wb(src, w, 0, 0xFF, d);
        }
    for (int w = 0; w < NWARPS; w++) {
        emit_rd(w, 0, 0);
        emit_rd(w, 0, 3);
        emit_rd(w, 3, 0);
    }
    /* 从未写过的寄存器：复位值 0 */
    emit_rd(0, 13, 29); emit_rd(1, 31, 1);
    emit_rd(2, 17, 17); emit_rd(3, 5, 11);
    /* 正常写后再验 R0 仍为 0 */
    fill_seq(d, 0x12345678u);
    emit_wb(0, 0, 5, 0xFF, d);
    emit_rd(0, 5, 0);
    emit_rd(0, 0, 5);
    emit_wb(1, 1, 0, 0xA5, d);   /* R0 带掩码写，同样忽略 */
    emit_rd(1, 0, 0);
}

// ---- 用例 3：lane 掩码（未命中 lane 保持原值） ----
static void build_masking(void)
{
    static const int masks[8] = { 0x00, 0x01, 0x80, 0xA5, 0x5A, 0x0F, 0xF0, 0x7E };
    uint32_t A[NLANES], B[NLANES];
    for (int i = 0; i < 8; i++) {
        int w = i & 3, rd = 1 + (i % 30), src = i % 3;
        fill_seq(A, 0xAAAA0000u ^ ((uint32_t)i << 8));
        fill_seq(B, 0xBBBB0000u ^ ((uint32_t)i << 12));
        emit_wb(src, w, rd, 0xFF, A);       /* 预置底值 */
        emit_rd(w, rd, rd);
        emit_wb(src, w, rd, masks[i], B);   /* 部分写 */
        emit_rd(w, rd, rd);
    }
}

// ---- 用例 4：三源同拍竞争与固定优先级 ----
static void build_arb(void)
{
    uint32_t d[NLANES];
    /* 两两同拍 */
    fill_seq(d, 0x11110000u); emit_wb(0, 0, 10, 0xFF, d);
    fill_seq(d, 0x22220000u); emit_wb(1, 0, 11, 0xFF, d);
    emit_rd(0, 10, 11);
    fill_seq(d, 0x33330000u); emit_wb(0, 1, 12, 0xFF, d);
    fill_seq(d, 0x44440000u); emit_wb(2, 1, 13, 0xFF, d);
    emit_rd(1, 12, 13);
    fill_seq(d, 0x55550000u); emit_wb(1, 2, 14, 0xFF, d);
    fill_seq(d, 0x66660000u); emit_wb(2, 2, 15, 0xFF, d);
    emit_rd(2, 14, 15);
    /* 三源同拍 */
    fill_seq(d, 0x77770000u); emit_wb(0, 3, 16, 0xFF, d);
    fill_seq(d, 0x88880000u); emit_wb(1, 3, 17, 0xFF, d);
    fill_seq(d, 0x99990000u); emit_wb(2, 3, 18, 0xFF, d);
    emit_rd(3, 16, 17);
    emit_rd(3, 18, 18);
    /* 同地址三源先后：最终值为最后一笔 */
    fill_seq(d, 0xAAAA0001u); emit_wb(0, 0, 20, 0xFF, d);
    fill_seq(d, 0xAAAA0002u); emit_wb(1, 0, 20, 0xFF, d);
    fill_seq(d, 0xAAAA0003u); emit_wb(2, 0, 20, 0xFF, d);
    emit_rd(0, 20, 20);
    /* 写读交错（含部分掩码覆写） */
    fill_seq(d, 0xBBBB0001u); emit_wb(0, 1, 21, 0xFF, d);
    emit_rd(1, 21, 21);
    fill_seq(d, 0xBBBB0002u); emit_wb(1, 1, 21, 0x0F, d);
    emit_rd(1, 21, 21);
}

// ---- 用例 5：读写顺序与同拍写读（含同地址旁路） ----
static void build_wrrd_order(void)
{
    uint32_t d[NLANES];
    for (int k = 0; k < 32; k++) {
        int w = k & 3, rd = 1 + (k % 31), src = k % 3;
        fill_seq(d, 0xC0DE0000u ^ ((uint32_t)k << 8));
        emit_wb(src, w, rd, 0xFF, d);
        emit_rd(w, rd, rd);   /* 与写相邻：无在途应答时同拍写读、同地址 */
        emit_rd(w, rd, rd);   /* 后续读：阵列路径 */
    }
    /* 写后读另一寄存器 + 同拍异地址 */
    for (int k = 0; k < 16; k++) {
        int w = k & 3, rd1 = 1 + (k % 15), rd2 = 16 + (k % 15), src = k % 3;
        fill_seq(d, 0xD00D0000u ^ ((uint32_t)k << 12));
        emit_wb(src, w, rd1, 0xFF, d);
        emit_rd(w, rd2, rd1);
    }
}

// ---- 随机混合 ----
static void gen_random(int n)
{
    uint32_t d[NLANES];
    for (int i = 0; i < n; i++) {
        if (rnd() % 10 < 4) {                     /* 写 40% */
            int src = rnd() % 3;
            int w = rnd() % NWARPS;
            int rd = (rnd() % 40 < 32) ? (rnd() % 32) : 0;
            if (rd == 0 && (rnd() % 8)) rd = 1 + rnd() % 31;  /* 少量 R0 写 */
            int mask = rnd() & 0xFF;
            for (int l = 0; l < NLANES; l++) d[l] = rnd32();
            emit_wb(src, w, rd, mask, d);
        } else {                                  /* 读 60% */
            int w = rnd() % NWARPS;
            int rs1 = rnd() % 40, rs2 = rnd() % 40;
            if (rs1 >= 32) rs1 = 0;
            if (rs2 >= 32) rs2 = 0;
            emit_rd(w, rs1, rs2);
        }
    }
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vrf *top = new Vrf;
    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("rf.vcd");

    seed_g = 1; begin_stim(); build_wr_seq();
    run_test(top, tfp, "wr_seq",  0,100,0,0);

    seed_g = 2; begin_stim(); build_r0_zero();
    run_test(top, tfp, "r0_zero", 0,100,0,0);

    seed_g = 3; begin_stim(); build_masking();
    run_test(top, tfp, "masking", 0,100,0,0);

    seed_g = 4; begin_stim(); build_arb();
    run_test(top, tfp, "arb",     0,100,0,0);

    seed_g = 5; begin_stim(); build_wrrd_order();
    run_test(top, tfp, "wrrd",    0,100,0,0);

    seed_g = 6; begin_stim(); gen_random(2000);
    run_test(top, tfp, "b2b",     0,100,0,0);

    seed_g = 7; begin_stim(); gen_random(4000);
    run_test(top, tfp, "rand50",  1, 50,0,0);

    seed_g = 8; begin_stim(); gen_random(4000);
    run_test(top, tfp, "rand20",  1, 20,0,0);

    seed_g = 9; begin_stim(); gen_random(3000);
    run_test(top, tfp, "pat",     2,100,3,5);

    seed_g = 10; begin_stim(); gen_random(10000);
    run_test(top, tfp, "soak",    1, 60,0,0);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
