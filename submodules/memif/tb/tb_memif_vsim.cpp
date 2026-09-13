// =============================================================================
// easy_simt · memif 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/memif/rtl/memif.sv 编译为 C++ 模型（Vmemif）；
// 本 harness 扮演 tb 侧 AXI4 从设备（intf_spec §9：片外固定延迟 MEM_LAT
// 归从设备建模，memif 自身不加延迟）与内部侧 icache/l1sm 双请求源、
// 双响应消费者（随机背压），参考侧直接链接 top/cmodel（memif_step），
// 按协议语义逐拍锁步比对：
//   - 请求受理拍与来源（固定优先级 icache 优先、单在途）；
//   - 响应消费拍与载荷（回填整行 8 字位精确、写应答 line=0）；
//   - AXI 请求格式（arlen=0/arsize=32B/INCR/ID=0；写窄传 awsize=4B、
//     wstrb 按 awaddr[4:2] 定位、wdata 行内字定位、wlast=1）；
//   - 非 OKAY 响应注入：memif_top_err 次拍置位并保持至复位（DUT 侧单独
//     检查，注入拍数据仍正确、事务序列不受影响）。
//
// 从设备时序与 C 模型 countdown 同构（memif_spec §1.5）：请求握手拍末沿
// 装载 MEM_LAT 倒计时，计数到 0 呈现 rvalid/bvalid——C 模型在同一拍序
// 提交响应（接收拍 + MEM_LAT + 1），逐拍对齐（含 MEM_LAT=0）。
// 地址口径与 cmodel 片外后备存储一致（top.c：in_base=0x00100000、
// out_base=0x00200000；icache 按字索引 imem、越界为 0；l1sm 两段全局
// 数据区）。写按 wstrb 字节选通提交，与 C 模型 gmem_write 同序。
//
// 波形：Verilator 原生 VCD（memif.vcd），gtkwave 查看。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vmemif.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链，仅用 memif 部分） ----------------
static sim_t ref;

// ---------------- 随机源（与 ialu/falu/lsu/rf harness 同式） ----------------
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

// ---------------- 地址口径（与 cmodel 片外后备存储一致） ----------------
static const uint32_t IN_BASE  = 0x00100000u;    /* gmem_in 段基址（top.c） */
static const uint32_t OUT_BASE = 0x00200000u;    /* gmem_out 段基址（top.c） */
static const int      IMEM_N   = 200;            /* 程序长度（字）：越界读为 0 */

// ---------------- tb 侧 AXI 从设备存储（与 ref.memif 同初始化同写序） ----
static uint32_t s_imem[IMEM_WORDS];
static uint32_t s_gin[GMEM_WORDS], s_gout[GMEM_WORDS];

// ---------------- 激励队列（vld 保持至握手） ----------------
struct IcTx { uint32_t addr; };
struct L1Tx { int rw; uint32_t addr, wdata; };
static IcTx iq[65536]; static int iq_h, iq_t;
static L1Tx dq[65536]; static int dq_h, dq_t;
static void emit_ic(uint32_t addr) { iq[iq_t++ & 65535].addr = addr; }
static void emit_l1(int rw, uint32_t addr, uint32_t wd)
{
    L1Tx *t = &dq[dq_t++ & 65535];
    t->rw = rw; t->addr = addr; t->wdata = wd;
}

static int errors, total_errors, cyc;
static vluint64_t gtime;   // VCD 单调时基
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- 消费者背压决策（同式） ----------------
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
static Cfg cfg_ic, cfg_l1;
static int rdy_of(Cfg *c) { return rdy_dec(c->m, c->p, c->on, c->off, &c->pat); }

// ---------------- tb 侧 AXI4 从设备 ----------------
// 请求通道（AR/AW/W）恒 ready；响应倒计时 = MEM_LAT（请求握手拍末沿装载，
// 与 C 模型 countdown 同构）；rvalid/bvalid 保持至 DUT 接收；
// 写数据按 wstrb 字节选通于 B 握手拍末沿提交；SLVERR 注入仅改 resp，
// 数据仍正确（事务序列不受影响，供 memif_top_err 检查）。
static int SL_LAT, inject_pct;
static int sl_pend, sl_mode, sl_cnt, sl_rv, sl_bv, sl_inject;
static uint32_t sl_addr, sl_wstrb;
static uint32_t sl_wdata[ILINE_WORDS];
static uint32_t sl_rdata[ILINE_WORDS], sl_rresp, sl_bresp;

static uint32_t sl_word(uint32_t addr, int *ok)
{
    *ok = 1;
    /* 指令段：行读自段内起始时字索引可越出映射上界一行（cmodel 按
       imem_n 截 0，不触片外后备），映射留 ILINE_WORDS 字余量 */
    if (addr < 4u * (IMEM_WORDS + ILINE_WORDS)) {
        uint32_t w = addr >> 2;
        return (w < (uint32_t)IMEM_N) ? s_imem[w] : 0u;
    }
    if (addr >= IN_BASE && addr < IN_BASE + 4u * GMEM_WORDS)
        return s_gin[(addr - IN_BASE) >> 2];
    if (addr >= OUT_BASE && addr < OUT_BASE + 4u * GMEM_WORDS)
        return s_gout[(addr - OUT_BASE) >> 2];
    *ok = 0;               /* 激励生成保证不会到达（越界为架构错误口径） */
    return 0u;
}

static void sl_present_read(void)
{
    for (int i = 0; i < ILINE_WORDS; i++) {
        int ok;
        sl_rdata[i] = sl_word(sl_addr + 4u * (uint32_t)i, &ok);
        if (!ok) err("slave read out of address map");
    }
    sl_rresp = sl_inject ? 2u : 0u;
    sl_rv = 1;
}

static void sl_commit_write(void)
{
    for (int b = 0; b < 32; b++) {
        if (!((sl_wstrb >> b) & 1u))
            continue;
        uint32_t byte_addr = (sl_addr & ~31u) + (uint32_t)b;
        uint32_t byte_val = (sl_wdata[b >> 2] >> ((b & 3) * 8)) & 0xFFu;
        if (byte_addr >= IN_BASE && byte_addr < IN_BASE + 4u * GMEM_WORDS) {
            uint32_t w = (byte_addr - IN_BASE) >> 2;
            int sh = (int)((byte_addr & 3u) * 8);
            s_gin[w] = (s_gin[w] & ~(0xFFu << sh)) | (byte_val << sh);
        } else if (byte_addr >= OUT_BASE && byte_addr < OUT_BASE + 4u * GMEM_WORDS) {
            uint32_t w = (byte_addr - OUT_BASE) >> 2;
            int sh = (int)((byte_addr & 3u) * 8);
            s_gout[w] = (s_gout[w] & ~(0xFFu << sh)) | (byte_val << sh);
        } else {
            err("slave write out of address map");
        }
    }
}

// 上升沿调用：fires/载荷取本拍（边沿前）捕获的 DUT 输出副本
static void slave_edge(int ar_fire, uint32_t araddr,
                       int aw_fire, uint32_t awaddr,
                       int w_fire, const uint32_t *wdata, uint32_t wstrb,
                       int r_fire, int b_fire)
{
    if (sl_rv) {
        if (r_fire) { sl_rv = 0; sl_pend = 0; }
        return;
    }
    if (sl_bv) {
        if (b_fire) { sl_commit_write(); sl_bv = 0; sl_pend = 0; }
        return;
    }
    if (sl_pend) {
        if (sl_cnt > 1) { sl_cnt--; return; }
        if (sl_mode == 0) sl_present_read();
        else { sl_bresp = sl_inject ? 2u : 0u; sl_bv = 1; }
        return;
    }
    if (ar_fire) {
        sl_pend = 1; sl_mode = 0; sl_addr = araddr; sl_cnt = SL_LAT;
        sl_inject = inject_pct && (rnd() % 100) < inject_pct;
        if (SL_LAT == 0) sl_present_read();
    } else if (aw_fire && w_fire) {
        sl_pend = 1; sl_mode = 1;
        sl_addr = awaddr; sl_wstrb = wstrb;
        for (int w = 0; w < ILINE_WORDS; w++) sl_wdata[w] = wdata[w];
        sl_cnt = SL_LAT;
        sl_inject = inject_pct && (rnd() % 100) < inject_pct;
        if (SL_LAT == 0) { sl_bresp = sl_inject ? 2u : 0u; sl_bv = 1; }
    }
}

// ---------------- 在途镜像与协议保持影子 ----------------
static int mirror_busy, mirror_to_icache, mirror_rw;
static int p_ic_v, p_ic_r, p_l1_v, p_l1_r;
static uint32_t p_ic_d[ILINE_WORDS], p_l1_d[ILINE_WORDS];
static int exp_err;

// ---------------- 事务计数（汇报用） ----------------
static uint64_t cnt_ic, cnt_l1r, cnt_l1w;

// ---------------- 单个用例 ----------------
static void run_test(Vmemif *top, VerilatedVcdC *tfp, const char *name, int lat)
{
    SL_LAT = lat;
    errors = 0;
    cyc = 0;
    mirror_busy = mirror_to_icache = mirror_rw = 0;
    p_ic_v = p_l1_v = 0;
    exp_err = 0;
    cnt_ic = cnt_l1r = cnt_l1w = 0;
    sl_pend = sl_mode = sl_cnt = sl_rv = sl_bv = sl_inject = 0;
    sl_addr = sl_wstrb = sl_rresp = sl_bresp = 0;
    memset(sl_wdata, 0, sizeof sl_wdata);
    memset(sl_rdata, 0, sizeof sl_rdata);

    // ---- 存储初始化（从设备与 ref.memif 片外后备存储同值） ----
    for (int w = 0; w < IMEM_WORDS; w++) s_imem[w] = rnd32();
    for (int w = 0; w < GMEM_WORDS; w++) { s_gin[w] = rnd32(); s_gout[w] = rnd32(); }

    memset(&ref, 0, sizeof ref);
    ref.memif.imem_n = IMEM_N;
    memcpy(ref.memif.imem, s_imem, sizeof s_imem);
    memcpy(ref.memif.gmem_in, s_gin, sizeof s_gin);
    memcpy(ref.memif.gmem_out, s_gout, sizeof s_gout);
    ref.memif.in_base = IN_BASE;
    ref.memif.out_base = OUT_BASE;
    ref.memif.memlat = lat;

    // ---- 复位 ----
    top->rst_n = 0;
    top->icache_memif_req_vld = 0;
    top->icache_memif_req_addr = 0;
    top->l1sm_memif_req_vld = 0;
    top->l1sm_memif_req_rw = 0;
    top->l1sm_memif_req_addr = 0;
    top->l1sm_memif_req_wdata = 0;
    top->icache_memif_rsp_rdy = 0;
    top->l1sm_memif_rsp_rdy = 0;
    top->axi_arready = 1;
    top->axi_awready = 1;
    top->axi_wready = 1;
    top->axi_rvalid = 0;
    top->axi_bvalid = 0;
    for (int w = 0; w < ILINE_WORDS; w++) top->axi_rdata[w] = 0;
    top->axi_rresp = 0; top->axi_bresp = 0;
    top->axi_rid = 0; top->axi_bid = 0; top->axi_rlast = 0;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 5; i++) {
        if (top->memif_icache_rsp_vld || top->memif_l1sm_rsp_vld ||
            top->axi_arvalid || top->axi_awvalid || top->axi_wvalid)
            err("vld not 0 during reset");
        if (top->memif_top_err)
            err("memif_top_err not 0 during reset");
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    }
    top->rst_n = 1;

    int ntx = (iq_t - iq_h) + (dq_t - dq_h);
    int cap = ntx * (lat + 40) + 4000;
    int finished = 0;
    while (!finished && cyc < cap) {
        // -- 消费者决策（两路独立） --
        int r_ic = rdy_of(&cfg_ic);
        int r_l1 = rdy_of(&cfg_l1);

        // -- 激励：两队列头各自持续给出直至握手 --
        int offer_i = (iq_h < iq_t);
        int offer_d = (dq_h < dq_t);
        top->icache_memif_req_vld = offer_i;
        top->icache_memif_req_addr = offer_i ? iq[iq_h & 65535].addr : 0;
        top->l1sm_memif_req_vld = offer_d;
        if (offer_d) {
            const L1Tx *t = &dq[dq_h & 65535];
            top->l1sm_memif_req_rw = t->rw;
            top->l1sm_memif_req_addr = t->addr;
            top->l1sm_memif_req_wdata = t->wdata;
        } else {
            top->l1sm_memif_req_rw = 0;
            top->l1sm_memif_req_addr = 0;
            top->l1sm_memif_req_wdata = 0;
        }

        // -- AXI 从设备响应呈现（寄存值，保持至 DUT 接收） --
        top->axi_rvalid = sl_rv;
        for (int w = 0; w < ILINE_WORDS; w++) top->axi_rdata[w] = sl_rdata[w];
        top->axi_rresp = sl_rv ? sl_rresp : 0;
        top->axi_rlast = sl_rv ? 1 : 0;
        top->axi_rid = 0;
        top->axi_bvalid = sl_bv;
        top->axi_bresp = sl_bv ? sl_bresp : 0;
        top->axi_bid = 0;

        top->icache_memif_rsp_rdy = r_ic;
        top->l1sm_memif_rsp_rdy = r_l1;
        top->eval();

        // -- 协议保持检查：上一拍 vld && !rdy，则本拍 vld 不撤、载荷不变 --
        if (p_ic_v && !p_ic_r) {
            if (!top->memif_icache_rsp_vld)
                err("icache rsp vld dropped under backpressure");
            else
                for (int w = 0; w < ILINE_WORDS; w++)
                    if (top->memif_icache_rsp_data[w] != p_ic_d[w]) {
                        err("icache rsp data changed under backpressure");
                        break;
                    }
        }
        if (p_l1_v && !p_l1_r) {
            if (!top->memif_l1sm_rsp_vld)
                err("l1sm rsp vld dropped under backpressure");
            else
                for (int w = 0; w < ILINE_WORDS; w++)
                    if (top->memif_l1sm_rsp_data[w] != p_l1_d[w]) {
                        err("l1sm rsp data changed under backpressure");
                        break;
                    }
        }

        // -- 本拍事件 --
        int d_ic_acc = top->icache_memif_req_vld && top->memif_icache_req_rdy;
        int d_l1_acc = top->l1sm_memif_req_vld && top->memif_l1sm_req_rdy;
        if (d_ic_acc + d_l1_acc > 1)
            err("both internal requests accepted in one cycle");
        int acc_rw = d_l1_acc ? dq[dq_h & 65535].rw : 0;
        int d_ic_rsp = top->memif_icache_rsp_vld && r_ic;
        int d_l1_rsp = top->memif_l1sm_rsp_vld && r_l1;
        if (d_ic_rsp + d_l1_rsp > 1)
            err("both internal responses consumed in one cycle");

        int ar_fire = top->axi_arvalid && top->axi_arready;
        int aw_fire = top->axi_awvalid && top->axi_awready;
        int w_fire  = top->axi_wvalid  && top->axi_wready;
        int r_beat  = top->axi_rvalid  && top->axi_rready;
        int b_beat  = top->axi_bvalid  && top->axi_bready;

        // -- rdy 与在途状态一致性（tb 请求通道恒 ready） --
        if (top->memif_icache_req_rdy != !mirror_busy)
            err("icache req rdy inconsistent with in-flight state");
        if (top->memif_l1sm_req_rdy !=
            (!mirror_busy && !top->icache_memif_req_vld))
            err("l1sm req rdy inconsistent with in-flight/priority state");
        if (mirror_busy &&
            (top->axi_arvalid || top->axi_awvalid || top->axi_wvalid))
            err("AXI request valid while transaction in flight");
        if (top->memif_icache_rsp_vld && (!mirror_busy || !mirror_to_icache))
            err("icache rsp without matching in-flight txn");
        if (top->memif_l1sm_rsp_vld && (!mirror_busy || mirror_to_icache))
            err("l1sm rsp without matching in-flight txn");

        // -- AXI 请求格式与内部受理的对应（memif_spec §5.2） --
        if (ar_fire != (d_ic_acc || (d_l1_acc && !acc_rw)))
            err("AR handshake divergence with internal read accept");
        if (aw_fire != (d_l1_acc && acc_rw))
            err("AW handshake divergence with internal write accept");
        if (w_fire != aw_fire)
            err("W handshake divergence with AW (tb ready always 1)");
        if (r_beat != (d_ic_rsp || (d_l1_rsp && !mirror_rw)))
            err("R beat divergence with internal read rsp consume");
        if (b_beat != (d_l1_rsp && mirror_rw))
            err("B beat divergence with internal write rsp consume");

        if (ar_fire) {
            uint32_t exp_addr = d_ic_acc ? iq[iq_h & 65535].addr
                                         : dq[dq_h & 65535].addr;
            if (top->axi_araddr != exp_addr) err("araddr mismatch");
            if (top->axi_arlen != 0)   err("arlen != 0");
            if (top->axi_arsize != 5)  err("arsize != 32B");
            if (top->axi_arburst != 1) err("arburst != INCR");
            if (top->axi_arid != 0)    err("arid != 0");
        }
        if (aw_fire) {
            const L1Tx *t = &dq[dq_h & 65535];
            if (top->axi_awaddr != t->addr) err("awaddr mismatch");
            if (top->axi_awlen != 0)   err("awlen != 0");
            if (top->axi_awsize != 2)  err("awsize != 4B");
            if (top->axi_awburst != 1) err("awburst != INCR");
            if (top->axi_awid != 0)    err("awid != 0");
            if (!top->axi_wlast)       err("wlast != 1");
            int ws = (int)((t->addr >> 2) & 7u);
            if (top->axi_wstrb != (0xFu << (ws * 4)))
                err("wstrb placement mismatch");
            for (int l = 0; l < ILINE_WORDS; l++) {
                uint32_t expw = (l == ws) ? t->wdata : 0u;
                if (top->axi_wdata[l] != expw) {
                    err("wdata placement mismatch");
                    break;
                }
            }
        }

        // -- memif_top_err 期望（呈现拍检测、次拍置位、粘滞） --
        int err_cond = mirror_busy && sl_inject &&
                       ((sl_rv && !mirror_rw) || (sl_bv && mirror_rw));
        if (top->memif_top_err != exp_err)
            err("memif_top_err mismatch with expected sticky state");

        // -- 参考同拍推进（置激励 → memif_step → 按 rdy 消费） --
        if (offer_i && !ref.icache_memif_req.vld) {
            ref.icache_memif_req.p.addr = iq[iq_h & 65535].addr;
            ref.icache_memif_req.vld = 1;
        }
        if (offer_d && !ref.l1sm_memif_req.vld) {
            const L1Tx *t = &dq[dq_h & 65535];
            ref.l1sm_memif_req.p.rw = t->rw;
            ref.l1sm_memif_req.p.addr = t->addr;
            ref.l1sm_memif_req.p.wdata = t->wdata;
            ref.l1sm_memif_req.vld = 1;
        }
        int had_i = ref.icache_memif_req.vld;
        int had_d = ref.l1sm_memif_req.vld;
        memif_step(&ref);
        int r_ic_acc = had_i && !ref.icache_memif_req.vld;
        int r_l1_acc = had_d && !ref.l1sm_memif_req.vld;

        int r_ic_rsp = 0;
        if (ref.memif_icache_rsp.vld && r_ic) {
            for (int w = 0; w < ILINE_WORDS; w++)
                if (ref.memif_icache_rsp.p.line[w] != top->memif_icache_rsp_data[w]) {
                    err("icache rsp data mismatch");
                    break;
                }
            ref.memif_icache_rsp.vld = 0;
            r_ic_rsp = 1;
        }
        int r_l1_rsp = 0;
        if (ref.memif_l1sm_rsp.vld && r_l1) {
            for (int w = 0; w < ILINE_WORDS; w++)
                if (ref.memif_l1sm_rsp.p.line[w] != top->memif_l1sm_rsp_data[w]) {
                    err("l1sm rsp data mismatch");
                    break;
                }
            ref.memif_l1sm_rsp.vld = 0;
            r_l1_rsp = 1;
        }

        // -- 逐拍一致性 --
        if (r_ic_acc != d_ic_acc) err("icache req accept divergence");
        if (r_l1_acc != d_l1_acc) err("l1sm req accept divergence");
        if (r_ic_rsp != d_ic_rsp) err("icache rsp consume divergence");
        if (r_l1_rsp != d_l1_rsp) err("l1sm rsp consume divergence");

        // -- 记账 --
        if (d_ic_acc) { cnt_ic++; iq_h++; }
        if (d_l1_acc) { if (acc_rw) cnt_l1w++; else cnt_l1r++; dq_h++; }
        if (d_ic_acc) { mirror_busy = 1; mirror_to_icache = 1; mirror_rw = 0; }
        if (d_l1_acc) { mirror_busy = 1; mirror_to_icache = 0; mirror_rw = acc_rw; }
        if (d_ic_rsp || d_l1_rsp) mirror_busy = 0;

        // -- 影子更新 --
        p_ic_v = top->memif_icache_rsp_vld; p_ic_r = r_ic;
        for (int w = 0; w < ILINE_WORDS; w++) p_ic_d[w] = top->memif_icache_rsp_data[w];
        p_l1_v = top->memif_l1sm_rsp_vld; p_l1_r = r_l1;
        for (int w = 0; w < ILINE_WORDS; w++) p_l1_d[w] = top->memif_l1sm_rsp_data[w];

        // -- AXI 载荷边沿前捕获（供从设备采样，256b 走指针） --
        uint32_t c_araddr = top->axi_araddr;
        uint32_t c_awaddr = top->axi_awaddr;
        uint32_t c_wstrb  = top->axi_wstrb;
        uint32_t c_wdata[ILINE_WORDS];
        for (int w = 0; w < ILINE_WORDS; w++) c_wdata[w] = top->axi_wdata[w];

        // -- 时钟上升沿：DUT 寄存器与从设备同步更新 --
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        slave_edge(ar_fire, c_araddr, aw_fire, c_awaddr,
                   w_fire, c_wdata, c_wstrb, r_beat, b_beat);
        if (err_cond) exp_err = 1;
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (iq_h == iq_t && dq_h == dq_t && !mirror_busy &&
            !sl_pend && !sl_rv && !sl_bv &&
            !top->memif_icache_rsp_vld && !top->memif_l1sm_rsp_vld &&
            !ref.icache_memif_req.vld && !ref.l1sm_memif_req.vld &&
            !ref.memif_icache_rsp.vld && !ref.memif_l1sm_rsp.vld &&
            !ref.memif.busy && !ref.memif.rsp_set)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (iq_h != iq_t) err("icache stimulus not drained");
    if (dq_h != dq_t) err("l1sm stimulus not drained");
    if (sl_pend || sl_rv || sl_bv) err("slave transaction pending at end");
    if (ref.memif.busy) err("ref still busy at end");
    if (ref.memif.err || ref.err) err("ref reported error");

    total_errors += errors;
    printf("[vsim] %-8s n=%-6d ic=%-6llu l1rd=%-6llu l1wr=%-6llu lat=%-3d "
           "errf=%d cyc=%-8d -> %s\n",
           name, ntx,
           (unsigned long long)cnt_ic, (unsigned long long)cnt_l1r,
           (unsigned long long)cnt_l1w, SL_LAT, exp_err,
           cyc, errors ? "FAIL" : "PASS");
}

// ============================================================================
//  激励生成
// ============================================================================
static void begin_stim(void) { iq_h = iq_t = 0; dq_h = dq_t = 0; }

static uint32_t ic_word(int w) { return 4u * (uint32_t)w; }

// ---- 用例 1：icache 顺序回填（全部指令字两遍，含 imem_n 越界字） ----
static void build_icache(void)
{
    for (int rep = 0; rep < 2; rep++)
        for (int w = 0; w < IMEM_WORDS; w++)
            emit_ic(ic_word(w));
}

// ---- 用例 2：l1sm 顺序读（两段全局数据区，对齐/非对齐） ----
static void build_l1sm_rd(void)
{
    for (int k = 0; k < 128; k++) {
        uint32_t base = (k & 1) ? OUT_BASE : IN_BASE;
        uint32_t w = (uint32_t)(k / 2) * 64u;
        emit_l1(0, base + 4u * w, 0);
        if ((k & 3) == 1)
            emit_l1(0, base + 4u * w + 4u, 0);   /* 非对齐行读 */
    }
}

// ---- 用例 3：l1sm 写通与写后读 ----
static void build_l1sm_wr(void)
{
    for (int k = 0; k < 128; k++) {
        uint32_t base = (k & 1) ? OUT_BASE : IN_BASE;
        uint32_t w = (uint32_t)k * 31u;          /* 覆盖 wsel 0..7 */
        uint32_t addr = base + 4u * w;
        emit_l1(1, addr, 0xC0DE0000u ^ (uint32_t)k ^ (base >> 4));
        emit_l1(0, addr & ~31u, 0);              /* 写后读同行 */
    }
}

// ---- 用例 4：双源竞争（icache 优先） ----
static void build_arb(void)
{
    for (int k = 0; k < 50; k++)                  /* 仅 l1sm 先行 */
        emit_l1(k & 1, ((k & 1) ? OUT_BASE : IN_BASE) + 4u * (uint32_t)(k * 7),
                0x5A5A0000u ^ (uint32_t)k);
    for (int k = 0; k < 150; k++) {               /* 双源同时挂起 */
        emit_ic(ic_word(rnd() % IMEM_WORDS));
        emit_l1(k & 1, ((k & 1) ? OUT_BASE : IN_BASE) + 4u * (uint32_t)(k * 13 % 4096),
                rnd32());
    }
    for (int k = 0; k < 30; k++)                  /* 仅 icache 收尾 */
        emit_ic(ic_word(IMEM_WORDS - 1 - k));
}

// ---- 用例 5：边界（地址/数据极值、MEM_LAT=0） ----
static void build_edge(void)
{
    /* icache：字 0、程序长度边界（196 起跨界行）、越界全 0、最高字 */
    emit_ic(ic_word(0));
    emit_ic(ic_word(IMEM_N - 1));
    emit_ic(ic_word(IMEM_N - 4));          /* w0=196：字 196..199 有效、200..203 为 0 */
    emit_ic(ic_word(IMEM_N));              /* 全 0 行 */
    emit_ic(ic_word(IMEM_WORDS - 1));      /* w0=255：全 0 行 */
    emit_ic(ic_word(IMEM_WORDS - 8));
    /* l1sm：段首/段尾行、非对齐、最后字写 */
    emit_l1(0, IN_BASE, 0);
    emit_l1(0, IN_BASE + 4u, 0);
    emit_l1(0, IN_BASE + 4u * (GMEM_WORDS - 8), 0);
    emit_l1(0, IN_BASE + 4u * (GMEM_WORDS - 9), 0);
    emit_l1(0, OUT_BASE, 0);
    emit_l1(0, OUT_BASE + 4u * (GMEM_WORDS - 8), 0);
    /* 写数据极值与 wsel 覆盖（w mod 8 = 0..7） */
    static const uint32_t xd[5] = {
        0x00000000u, 0xFFFFFFFFu, 0x80000000u, 0x7FFFFFFFu, 0xA5A5A5A5u
    };
    for (int k = 0; k < 8; k++)
        emit_l1(1, IN_BASE + 4u * (uint32_t)k, xd[k % 5]);
    emit_l1(1, IN_BASE + 4u * (GMEM_WORDS - 1), 0xDEADBEEFu);   /* 段尾最后字 */
    emit_l1(1, OUT_BASE + 4u * (GMEM_WORDS - 1), 0x1BADD00Du);
    emit_l1(0, IN_BASE, 0);                                     /* 回读验证 */
    emit_l1(0, IN_BASE + 4u * (GMEM_WORDS - 8), 0);
    emit_l1(0, OUT_BASE + 4u * (GMEM_WORDS - 8), 0);
    /* 随机补充 */
    for (int k = 0; k < 60; k++) {
        if (rnd() % 2)
            emit_ic(ic_word(rnd() % IMEM_WORDS));
        else {
            uint32_t base = (rnd() % 2) ? OUT_BASE : IN_BASE;
            uint32_t w = rnd() % (GMEM_WORDS - 7);
            emit_l1(rnd() % 2, base + 4u * w, rnd32());
        }
    }
}

// ---- 随机混合 ----
static void gen_random(int n)
{
    for (int i = 0; i < n; i++) {
        int r = rnd() % 10;
        if (r < 4) {                              /* icache 读 40% */
            emit_ic(ic_word(rnd() % IMEM_WORDS));
        } else {
            uint32_t base = (rnd() % 2) ? OUT_BASE : IN_BASE;
            if (rnd() % 2) {                      /* l1sm 写 */
                emit_l1(1, base + 4u * (uint32_t)(rnd() % GMEM_WORDS), rnd32());
            } else {                              /* l1sm 读（8 字余量） */
                uint32_t addr = base + 4u * (uint32_t)(rnd() % (GMEM_WORDS - 7));
                if (rnd() % 2) addr &= ~31u;
                emit_l1(0, addr, 0);
            }
        }
    }
}

// ---- 用例配置 ----
static void cfg_all(int im, int ip, int lm, int lp)
{
    cfg_ic.m = im; cfg_ic.p = ip; cfg_ic.on = cfg_ic.off = 0; cfg_ic.pat = 0;
    cfg_l1.m = lm; cfg_l1.p = lp; cfg_l1.on = cfg_l1.off = 0; cfg_l1.pat = 0;
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vmemif *top = new Vmemif;
    VerilatedVcdC *tfp = new VerilatedVcdC;
    Verilated::traceEverOn(true);
    top->trace(tfp, 99);
    tfp->open("memif.vcd");

    seed_g = 1; begin_stim(); build_icache();
    inject_pct = 0; cfg_all(0,100, 0,100);
    run_test(top, tfp, "icache", 1);

    seed_g = 2; begin_stim(); build_l1sm_rd();
    inject_pct = 0; cfg_all(0,100, 0,100);
    run_test(top, tfp, "l1rd", 1);

    seed_g = 3; begin_stim(); build_l1sm_wr();
    inject_pct = 0; cfg_all(0,100, 0,100);
    run_test(top, tfp, "l1wr", 1);

    seed_g = 4; begin_stim(); build_arb();
    inject_pct = 0; cfg_all(0,100, 0,100);
    run_test(top, tfp, "arb", 2);

    seed_g = 5; begin_stim(); build_edge();
    inject_pct = 0; cfg_all(0,100, 0,100);
    run_test(top, tfp, "edge", 0);

    seed_g = 6; begin_stim(); gen_random(2000);
    inject_pct = 0; cfg_all(0,100, 0,100);
    run_test(top, tfp, "b2b", 2);

    seed_g = 7; begin_stim(); gen_random(4000);
    inject_pct = 0; cfg_all(1, 50, 1, 50);
    run_test(top, tfp, "rand50", 3);

    seed_g = 8; begin_stim(); gen_random(4000);
    inject_pct = 0; cfg_all(1, 20, 1, 20);
    run_test(top, tfp, "rand20", 5);

    // SLVERR 注入：事务序列不受影响（数据仍正确），memif_top_err 次拍置位并粘滞
    seed_g = 9; begin_stim(); gen_random(2000);
    inject_pct = 30; cfg_all(1, 60, 1, 60);
    run_test(top, tfp, "slverr", 2);

    seed_g = 10; begin_stim(); gen_random(10000);
    inject_pct = 0; cfg_all(1, 60, 1, 60);
    run_test(top, tfp, "soak", 8);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
