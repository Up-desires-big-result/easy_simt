// =============================================================================
// easy_simt · l1sm 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/l1sm/rtl/l1sm.sv 编译为 C++ 模型（Vl1sm）；
// 本 harness 驱动时钟/复位/激励与背压，参考侧直接链接 top/cmodel
// （l1sm_step），记分板按协议语义逐笔比对。
//
// 事务级比对（l1sm_spec §10）：RTL 为 8-bank 并行行组服务、cmodel 为逐
// lane 串行，两侧边界事务序列一致（请求→响应逐笔、memif 事务笔数/顺序/
// 载荷相同），周期不比对。tb 扮 lsu 侧（请求源 + 响应消费者，随机背压）
// 与 memif 侧从设备（倒计时 = MEM_LAT，与 memif_spec §1.5 同构：呈现拍 =
// 请求握手拍 + MEM_LAT + 1，含 0；两段全局数据区行为级后备，读写共用，
// 读留行余量）。参考侧从设备零延迟，每拍泵至静默（pump）。
//
// 周期级断言（仅已知单行无冲突且必命中的装载）：响应呈现拍 = 接收握手
// 拍 + 2（l1sm_spec §7.1/L3）。
//
// 波形：Verilator 原生 VCD（<mod>.vcd），gtkwave 查看。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vl1sm.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链，仅用 l1sm 部分） ----------------
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

// ---------------- 片外后备存储（DUT 从设备与参考从设备共用） ----------------
// 全局数据窗：GBASE 起共 2048 字（256 行）；SM 区不经 memif
#define GBASE   0x1000u
#define GWORDS  2048
#define NBACK   (0x400 + GWORDS + 8)
static uint32_t backing[NBACK];

// ---------------- 激励队列（lsu 侧请求序列，vld 保持至握手） ----------------
struct ReqT {
    int rw, sm, single;             // single：装载且单行组且已知命中（L3 用）
    uint32_t addr[8], wdata[8];
    uint8_t mask;
};
static ReqT stimQ[65536];
static int sq_h, sq_t;

// DUT 已接收、待喂参考模型的请求（保序）
static ReqT pendQ[65536];
static int pq_h, pq_t;

// ---------------- 记分板：参考输出事务 ----------------
struct MtxT { int rw; uint32_t addr, wdata; };
static MtxT expM[65536]; static int em_h, em_t;     // 期望 memif 事务
struct RspT { uint32_t rdata[8]; };
static RspT expR[65536]; static int er_h, er_t;     // 期望响应

static int errors, total_errors, cyc;
static vluint64_t gtime;   // VCD 单调时基
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- 消费者/从设备背压决策（与 ialu harness 同式） ----------------
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
static int rsp_mode, rsp_pct, rsp_on, rsp_off, rsp_pat;   // l1sm_lsu_rsp 消费
static int mreq_mode, mreq_pct, mreq_on, mreq_off, mreq_pat; // memif 请求受理

// ---------------- 参考侧从设备（零延迟） ----------------
static int ref_armed;
static memif_dreq_t ref_mreq;

// ---------------- DUT 侧 memif 从设备（倒计时 = MEM_LAT） ----------------
static int    sl_busy, sl_rsp_v, sl_cnt;
static int    sl_rw;
static uint32_t sl_addr, sl_wdata;
static int memlat;

// ---------------- 单拍时序断言（L3） ----------------
static int single_armed, single_cyc;

// ---------------- 协议影子（背压下载荷稳定） ----------------
static int p_rsp_v, p_rsp_rdy;
static uint32_t p_rsp_data[8];
static int p_mreq_v, p_mreq_rdy, p_mreq_rw, p_mreq_rot;
static uint32_t p_mreq_addr, p_mreq_wdata;
static int mirror_busy;

// ---------------- 参考泵：喂请求 + 从设备 + l1sm_step 至静默 ----------------
// 终止条件为完全静默（cmodel L_SVC 服务步不产生 fired，不能按动作计数退出）
static void ref_pump(void)
{
    for (int it = 0; it < 2048; it++) {
        /* 完全静默且无待喂：泵毕 */
        if (!ref.l1sm.busy && !ref.lsu_l1sm_req.vld
            && !ref.l1sm_memif_req.vld && !ref.memif_l1sm_rsp.vld
            && !ref.l1sm_lsu_rsp.vld && !ref_armed && pq_h >= pq_t)
            break;
        /* 喂待处理请求（保序；cmodel 空闲才收） */
        if (!ref.l1sm.busy && !ref.lsu_l1sm_req.vld && pq_h < pq_t) {
            ReqT *r = &pendQ[pq_h++ & 65535];
            ref.lsu_l1sm_req.p.rw = r->rw;
            ref.lsu_l1sm_req.p.sm = r->sm;
            ref.lsu_l1sm_req.p.mask = r->mask;
            for (int l = 0; l < NLANES; l++) {
                ref.lsu_l1sm_req.p.addr[l] = r->addr[l];
                ref.lsu_l1sm_req.p.wdata[l] = r->wdata[l];
            }
            ref.lsu_l1sm_req.vld = 1;
        }
        /* 从设备受理 memif 请求（压入期望事务，供 DUT 侧逐笔比对） */
        if (ref.l1sm_memif_req.vld) {
            ref_mreq = ref.l1sm_memif_req.p;
            ref.l1sm_memif_req.vld = 0;
            ref_armed = 1;
            MtxT *e = &expM[em_t++ & 65535];
            e->rw = ref_mreq.rw;
            e->addr = ref_mreq.addr;
            e->wdata = ref_mreq.wdata;
        }
        /* 从设备呈现响应（零延迟；写在呈现拍提交后备） */
        if (ref_armed && !ref.memif_l1sm_rsp.vld) {
            if (ref_mreq.rw) {
                if (((ref_mreq.addr - GBASE) >> 2) < GWORDS)
                    backing[ref_mreq.addr >> 2] = ref_mreq.wdata;
                for (int i = 0; i < ILINE_WORDS; i++)
                    ref.memif_l1sm_rsp.p.line[i] = 0;
            } else {
                uint32_t base = ref_mreq.addr >> 2;
                for (int i = 0; i < ILINE_WORDS; i++)
                    ref.memif_l1sm_rsp.p.line[i] = backing[base + i];
            }
            ref.memif_l1sm_rsp.vld = 1;
            ref_armed = 0;
        }
        /* 参考推进一步 */
        l1sm_step(&ref);
        /* 收参考响应（tb 扮 lsu，立即消费） */
        if (ref.l1sm_lsu_rsp.vld) {
            RspT *e = &expR[er_t++ & 65535];
            for (int l = 0; l < NLANES; l++)
                e->rdata[l] = ref.l1sm_lsu_rsp.p.rdata[l];
            ref.l1sm_lsu_rsp.vld = 0;
        }
    }
}

// ---------------- 单行组判定（激励构造用） ----------------
static int is_single(const ReqT *r)
{
    int seen = 0;
    uint32_t row = 0;
    int banks = 0;
    for (int l = 0; l < NLANES; l++) {
        if (!((r->mask >> l) & 1))
            continue;
        uint32_t rr = r->addr[l] >> 5;
        int b = (r->addr[l] >> 2) & 7;
        if (!seen) { seen = 1; row = rr; banks = 1 << b; }
        else if (rr != row || (banks >> b) & 1) return 0;
        else banks |= 1 << b;
    }
    return seen;
}

static void emit_req(int rw, int sm, const uint32_t *addr,
                     const uint32_t *wd, uint8_t mask, int hit_known)
{
    ReqT r;
    memset(&r, 0, sizeof r);
    r.rw = rw;
    r.sm = sm;
    r.mask = mask;
    for (int l = 0; l < NLANES; l++) {
        r.addr[l] = addr ? addr[l] : 0;
        r.wdata[l] = wd ? wd[l] : 0;
    }
    r.single = (!rw) && (sm || hit_known) && is_single(&r);
    stimQ[sq_t++ & 65535] = r;
}

static uint32_t mk_sm(int row, int word) { return (uint32_t)(row * 32 + word * 4); }
static uint32_t mk_gl(int line, int word) { return GBASE + (uint32_t)(line * 32 + word * 4); }

// ---------------- 单个用例 ----------------
static void run_test(Vl1sm *top, VerilatedVcdC *tfp, const char *name,
                     int rpm, int rpp, int rpo, int rpf,
                     int mpmm, int mpp, int mpo, int mpf,
                     int lat, unsigned seed0)
{
    rsp_mode = rpm; rsp_pct = rpp; rsp_on = rpo; rsp_off = rpf;
    mreq_mode = mpmm; mreq_pct = mpp; mreq_on = mpo; mreq_off = mpf;
    memlat = lat;
    seed_g = seed0;
    errors = 0;
    pq_h = pq_t = 0;
    em_h = em_t = 0;
    er_h = er_t = 0;
    ref_armed = 0;
    sl_busy = sl_rsp_v = sl_cnt = 0;
    single_armed = 0;
    p_rsp_v = p_mreq_v = 0;
    p_mreq_rot = 0;
    mirror_busy = 0;
    cyc = 0;

    memset(&ref, 0, sizeof ref);
    /* 参考侧 SM 复位钉扎（对应 RTL 复位钉扎，sim_block_start 等价） */
    for (int r = 0; r < SM_LINES; r++) {
        ref.l1sm.cls[r] = CLS_SM;
        ref.l1sm.valid[r] = 1;
        ref.l1sm.tag[r] = (uint32_t)r;
    }
    ref.l1sm.shbase = 0;
    for (int i = 0; i < NBACK; i++)
        backing[i] = rnd32();

    // ---- 复位 ----
    top->rst_n = 0;
    top->lsu_l1sm_req_vld = 0;
    top->lsu_l1sm_req_rw = 0;
    top->lsu_l1sm_req_sm = 0;
    top->lsu_l1sm_req_mask = 0;
    for (int l = 0; l < NLANES; l++) {
        top->lsu_l1sm_req_addr[l] = 0;
        top->lsu_l1sm_req_wdata[l] = 0;
    }
    top->lsu_l1sm_rsp_rdy = 0;
    top->memif_l1sm_req_rdy = 0;
    top->memif_l1sm_rsp_vld = 0;
    for (int i = 0; i < ILINE_WORDS; i++)
        top->memif_l1sm_rsp_data[i] = 0;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 5; i++) {
        if (top->l1sm_lsu_rsp_vld || top->l1sm_memif_req_vld)
            err("vld not 0 during reset");
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    }
    top->rst_n = 1;

    int ntx = sq_t - sq_h;
    int cap = ntx * 400 + 4000;
    int finished = 0;
    while (!finished && cyc < cap) {
        // -- 消费者/从设备决策 --
        int r_rsp = rdy_dec(rsp_mode, rsp_pct, rsp_on, rsp_off, &rsp_pat);
        int r_mreq = !sl_busy && !sl_rsp_v
                     && rdy_dec(mreq_mode, mreq_pct, mreq_on, mreq_off, &mreq_pat);

        // -- 激励：队列头持续给出直至握手（vld 保持，协议 §9 条 1） --
        int offer = (sq_h < sq_t);
        ReqT *tx = offer ? &stimQ[sq_h & 65535] : 0;

        top->lsu_l1sm_req_vld = offer;
        top->lsu_l1sm_req_rw = offer ? tx->rw : 0;
        top->lsu_l1sm_req_sm = offer ? tx->sm : 0;
        top->lsu_l1sm_req_mask = offer ? tx->mask : 0;
        for (int l = 0; l < NLANES; l++) {
            top->lsu_l1sm_req_addr[l]  = offer ? tx->addr[l] : 0;
            top->lsu_l1sm_req_wdata[l] = offer ? tx->wdata[l] : 0;
        }
        top->lsu_l1sm_rsp_rdy = r_rsp;
        top->memif_l1sm_req_rdy = r_mreq;
        top->memif_l1sm_rsp_vld = sl_rsp_v;
        if (sl_rsp_v && sl_rw == 0) {
            uint32_t base = sl_addr >> 2;
            for (int i = 0; i < ILINE_WORDS; i++)
                top->memif_l1sm_rsp_data[i] = backing[base + i];
        } else {
            for (int i = 0; i < ILINE_WORDS; i++)
                top->memif_l1sm_rsp_data[i] = 0;
        }
        top->eval();

        // -- 协议保持检查：上一拍 vld && !rdy，则本拍 vld 不撤、载荷不变 --
        if (p_rsp_v && !p_rsp_rdy) {
            if (!top->l1sm_lsu_rsp_vld)
                err("rsp vld dropped under backpressure");
            else
                for (int l = 0; l < NLANES; l++)
                    if (top->l1sm_lsu_rsp_rdata[l] != p_rsp_data[l]) {
                        err("rsp rdata changed under backpressure");
                        break;
                    }
        }
        if (p_mreq_v && !p_mreq_rdy && !p_mreq_rot) {
            /* 载荷旋转仅发生在上一笔事务握手完成之后（写通链在响应
             * 消费沿轮换到下一 lane，属新事务呈现，协议合法），故仅在
             * 无旋转且无握手间隔内要求载荷稳定 */
            if (!top->l1sm_memif_req_vld)
                err("memif req vld dropped under backpressure");
            else if ((int)top->l1sm_memif_req_rw != p_mreq_rw ||
                     top->l1sm_memif_req_addr != p_mreq_addr ||
                     top->l1sm_memif_req_wdata != p_mreq_wdata)
                err("memif req payload changed under backpressure");
        } else if (p_mreq_v && !p_mreq_rdy) {
            if (!top->l1sm_memif_req_vld)
                err("memif req vld dropped under backpressure");
        }
        if (sl_rsp_v && !top->l1sm_memif_rsp_rdy)
            err("memif rsp presented but l1sm not ready");

        // -- 本拍末沿将发生的事件（输出为寄存器/状态译码，边沿前稳定） --
        int d_acc = offer && top->l1sm_lsu_req_rdy;
        if (top->l1sm_lsu_req_rdy != !mirror_busy)
            err("req_rdy inconsistent with in-flight state");
        int d_mreq = top->l1sm_memif_req_vld && r_mreq;
        int d_mrsp = sl_rsp_v && top->l1sm_memif_rsp_rdy;
        int d_rsp = top->l1sm_lsu_rsp_vld && r_rsp;
        if (d_mreq + d_mrsp > 1)
            err("memif req/rsp both fire in one cycle");

        // -- L3 单拍时序断言：装载且单行组且已知命中 --
        if (top->l1sm_lsu_rsp_vld && single_armed) {
            if (cyc - single_cyc != 2)
                err("single-beat timing violation (rsp not 2 cycles after accept)");
            single_armed = 0;
        }

        if (d_acc) {
            pendQ[pq_t++ & 65535] = *tx;
            if (tx->single) { single_armed = 1; single_cyc = cyc; }
            sq_h++;
            mirror_busy = 1;
        }
        if (d_mreq) {
            if (em_h >= em_t) {
                err("DUT memif req without ref transaction");
            } else {
                MtxT *e = &expM[em_h++ & 65535];
                if (e->rw != (int)top->l1sm_memif_req_rw)
                    err("memif req rw mismatch");
                else if (e->addr != top->l1sm_memif_req_addr)
                    err("memif req addr mismatch");
                else if (e->wdata != top->l1sm_memif_req_wdata)
                    err("memif req wdata mismatch");
            }
            sl_busy = 1;
            sl_cnt = memlat;
            sl_rsp_v = 0;
            sl_rw = top->l1sm_memif_req_rw;
            sl_addr = top->l1sm_memif_req_addr;
            sl_wdata = top->l1sm_memif_req_wdata;
        }
        if (d_mrsp) {
            sl_rsp_v = 0;
            sl_busy = 0;
        }
        if (d_rsp) {
            if (er_h >= er_t) {
                err("DUT rsp without ref transaction");
            } else {
                RspT *e = &expR[er_h++ & 65535];
                for (int l = 0; l < NLANES; l++)
                    if (e->rdata[l] != top->l1sm_lsu_rsp_rdata[l]) {
                        err("rsp rdata mismatch");
                        break;
                    }
            }
            mirror_busy = 0;
        }

        // -- 影子更新（p_mreq_rot：本拍发生响应消费沿（载荷合法轮换点）） --
        p_rsp_v = top->l1sm_lsu_rsp_vld; p_rsp_rdy = r_rsp;
        for (int l = 0; l < NLANES; l++)
            p_rsp_data[l] = top->l1sm_lsu_rsp_rdata[l];
        p_mreq_v = top->l1sm_memif_req_vld; p_mreq_rdy = r_mreq;
        p_mreq_rw = top->l1sm_memif_req_rw;
        p_mreq_addr = top->l1sm_memif_req_addr;
        p_mreq_wdata = top->l1sm_memif_req_wdata;
        p_mreq_rot = d_mrsp;

        // -- 从设备倒计时（呈现拍 = 握手拍 + MEM_LAT + 1，含 0） --
        if (sl_busy && !sl_rsp_v) {
            if (sl_cnt == 0) {
                sl_rsp_v = 1;
                if (sl_rw) {          // 写在呈现拍提交后备
                    if (((sl_addr - GBASE) >> 2) < GWORDS)
                        backing[sl_addr >> 2] = sl_wdata;
                }
            } else {
                sl_cnt--;
            }
        }

        // -- 参考泵（喂请求 + 零延迟从设备 + l1sm_step 至静默） --
        ref_pump();

        // -- 时钟上升沿 --
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (sq_h == sq_t && pq_h == pq_t && !mirror_busy &&
            !top->l1sm_lsu_rsp_vld && !top->l1sm_memif_req_vld &&
            !sl_busy && !sl_rsp_v && !ref.l1sm.busy &&
            !ref.lsu_l1sm_req.vld && !ref.l1sm_lsu_rsp.vld &&
            !ref.l1sm_memif_req.vld && !ref.memif_l1sm_rsp.vld &&
            em_h == em_t && er_h == er_t)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (sq_h != sq_t) err("stimulus not drained");
    if (pq_h != pq_t) err("ref feed not drained");
    if (em_h != em_t) err("memif scoreboard not drained");
    if (er_h != er_t) err("rsp scoreboard not drained");
    if (ref.l1sm.busy) err("ref still busy at end");
    if (ref.err) err("ref reported error");

    total_errors += errors;
    printf("[vsim] %-10s n=%-6d cyc=%-8d -> %s\n",
           name, ntx, cyc, errors ? "FAIL" : "PASS");
}

// ============================================================================
//  激励生成
// ============================================================================
static uint32_t A8[8], B8[8];
static void fill8(uint32_t *v, uint32_t x)
{
    for (int l = 0; l < NLANES; l++) v[l] = x;
}

// ---- 用例 1：SM 顺序访问（单行 stride-1 与跨行） ----
static void build_sm_seq(void)
{
    for (int l = 0; l < NLANES; l++) A8[l] = mk_sm(0, l);
    emit_req(0, 1, A8, 0, 0xFF, 0);                       // 单行读（L3 标记）
    for (int l = 0; l < NLANES; l++) A8[l] = mk_sm(1, l);
    for (int l = 0; l < NLANES; l++) B8[l] = 0xA1000000u + (uint32_t)l;
    emit_req(1, 1, A8, B8, 0xFF, 0);                      // 单行写
    for (int l = 0; l < NLANES; l++) A8[l] = mk_sm(1, l);
    emit_req(0, 1, A8, 0, 0xFF, 0);                       // 写后读回
    for (int l = 0; l < NLANES; l++) A8[l] = mk_sm(l & 3, l >> 1);
    emit_req(0, 1, A8, 0, 0xFF, 0);                       // 跨行（4 行组分拍）
    for (int l = 0; l < NLANES; l++) B8[l] = 0xB2000000u + (uint32_t)(l * 7);
    emit_req(1, 1, A8, B8, 0xFF, 0);                      // 跨行写
    for (int l = 0; l < NLANES; l++) A8[l] = mk_sm(l & 3, l >> 1);
    emit_req(0, 1, A8, 0, 0xFF, 0);                       // 跨行写后读回
    for (int l = 0; l < NLANES; l++) A8[l] = mk_sm(3, l & 3);
    emit_req(0, 1, A8, 0, 0x0F, 0);                       // 部分掩码
    emit_req(0, 1, A8, 0, 0xA5, 0);                       // 稀疏掩码
}

// ---- 用例 2：SM 同 bank/同字冲突（行组分裂、后写为准） ----
static void build_sm_conflict(void)
{
    fill8(A8, mk_sm(2, 3));                               // 全 lane 同字
    for (int l = 0; l < NLANES; l++) B8[l] = 0xC3000000u + (uint32_t)l;
    emit_req(1, 1, A8, B8, 0xFF, 0);                      // 同字多写：后写 lane 为准
    emit_req(0, 1, A8, 0, 0xFF, 0);                       // 同字读：全 lane 同值
    for (int l = 0; l < NLANES; l++) A8[l] = mk_sm(1, l & 3);   // 同 bank 异字
    emit_req(0, 1, A8, 0, 0xFF, 0);
    for (int l = 0; l < NLANES; l++)                       // 两 bank 交替重复
        A8[l] = (l & 1) ? mk_sm(0, 5) : mk_sm(0, 6);
    for (int l = 0; l < NLANES; l++) B8[l] = 0xC4000000u + (uint32_t)l;
    emit_req(1, 1, A8, B8, 0xFF, 0);
    emit_req(0, 1, A8, 0, 0xFF, 0);
    emit_req(0, 1, 0, 0, 0x00, 0);                        // 空掩码
}

// ---- 用例 3：全局顺序装载（缺失回填→命中；写通更新） ----
static void build_glb_seq(void)
{
    for (int line = 0; line < 8; line++) {
        for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(line, l);
        emit_req(0, 0, A8, 0, 0xFF, 0);                   // 首触：缺失回填
        emit_req(0, 0, A8, 0, 0xFF, 1);                   // 重读：命中（L3 标记）
    }
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(l, 3); // 跨行命中（8 行组）
    emit_req(0, 0, A8, 0, 0xFF, 0);
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(2, l); // 命中行写 + 写通
    for (int l = 0; l < NLANES; l++) B8[l] = 0xD5000000u + (uint32_t)l;
    emit_req(1, 0, A8, B8, 0xFF, 0);
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(2, l);
    emit_req(0, 0, A8, 0, 0xFF, 1);                       // 重读：命中见新值
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(5, l); // 跨 bank 部分掩码
    emit_req(0, 0, A8, 0, 0x3C, 0);
}

// ---- 用例 4：set 冲突替换（同 set 行互相驱逐） ----
static void build_set_thrash(void)
{
    /* 行 0/60/120/180 同 set（0 mod 60）：两行互逐 */
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(0, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(60, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(0, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);                       // 再缺失（被逐）
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(120, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(60, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);                       // 再缺失
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(0, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);
    /* SM 钉扎不受替换影响 */
    fill8(A8, mk_sm(0, 0));
    emit_req(0, 1, A8, 0, 0xFF, 0);
}

// ---- 用例 5：写直通不写分配 ----
static void build_wt_noalloc(void)
{
    /* 全新行写：不分配；随后装载必须回填，且后备已含写通数据 */
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(30, l);
    for (int l = 0; l < NLANES; l++) B8[l] = 0xE6000000u + (uint32_t)(l * 3);
    emit_req(1, 0, A8, B8, 0xFF, 0);
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(30, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);                       // 回填返回写通值
    /* 已命中行写缺失侧邻居：命中更新 + 写通，非分配路径并存 */
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(31, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);                       // 装载 31（分配）
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(31, l);
    emit_req(1, 0, A8, B8, 0xFF, 0);                      // 命中写
    for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(30, l);
    emit_req(0, 0, A8, 0, 0xFF, 0);
}

// ---- 用例 6：SM 钉扎（L1 大量替换后 SM 恒正确） ----
static void build_sm_pin(void)
{
    for (int i = 0; i < 12; i++) {
        int line = (i * 17) % 40;
        for (int l = 0; l < NLANES; l++) A8[l] = mk_gl(line, l);
        emit_req(0, 0, A8, 0, 0xFF, 0);
        for (int l = 0; l < NLANES; l++) A8[l] = mk_sm(i & 3, l);
        for (int l = 0; l < NLANES; l++) B8[l] = 0xF7000000u + (uint32_t)(i * 9 + l);
        emit_req(1, 1, A8, B8, 0xFF, 0);
        emit_req(0, 1, A8, 0, 0xFF, 0);
    }
}

// ---- 随机混合（lmax 行数上界；thrash 为真时偏向同 set 行） ----
static void gen_random(int n, int lmax, int thrash)
{
    for (int i = 0; i < n; i++) {
        int rw = rnd() & 1;
        int sm = rnd() & 1;
        int mask = rnd() & 0xFF;
        if ((rnd() % 16) == 0) mask = 0;
        for (int l = 0; l < NLANES; l++) {
            if (sm) {
                A8[l] = mk_sm(rnd() % SM_LINES, rnd() % 8);
            } else {
                int line = thrash ? (rnd() % 4) * 60 + (rnd() % 8)
                                  : rnd() % lmax;
                A8[l] = mk_gl(line, rnd() % 8);
            }
            B8[l] = rnd32();
        }
        emit_req(rw, sm, A8, B8, (uint8_t)mask, 0);
    }
}

static void begin_stim(void) { sq_h = sq_t = 0; }

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vl1sm *top = new Vl1sm;
    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("l1sm.vcd");

    seed_g = 1; begin_stim(); build_sm_seq();
    run_test(top, tfp, "sm_seq",   0,100,0,0, 0,100,0,0, 4, 1);

    seed_g = 2; begin_stim(); build_sm_conflict();
    run_test(top, tfp, "sm_conf",  0,100,0,0, 0,100,0,0, 4, 2);

    seed_g = 3; begin_stim(); build_glb_seq();
    run_test(top, tfp, "glb_seq",  0,100,0,0, 0,100,0,0, 4, 3);

    seed_g = 4; begin_stim(); build_set_thrash();
    run_test(top, tfp, "thrash",   0,100,0,0, 0,100,0,0, 2, 4);

    seed_g = 5; begin_stim(); build_wt_noalloc();
    run_test(top, tfp, "wt_noalc", 0,100,0,0, 0,100,0,0, 0, 5);

    seed_g = 6; begin_stim(); build_sm_pin();
    run_test(top, tfp, "sm_pin",   0,100,0,0, 0,100,0,0, 6, 6);

    seed_g = 7; begin_stim(); gen_random(2000, 200, 0);
    run_test(top, tfp, "rand50",   1, 50,0,0, 1, 50,0,0, 3, 7);

    seed_g = 8; begin_stim(); gen_random(2000, 60, 1);
    run_test(top, tfp, "thr_rnd",  1, 20,0,0, 1, 30,0,0, 5, 8);

    seed_g = 9; begin_stim(); gen_random(3000, 200, 0);
    run_test(top, tfp, "pat",      2,100,3,11, 2,100,5,7, 8, 9);

    seed_g = 10; begin_stim(); gen_random(6000, 100, 1);
    run_test(top, tfp, "soak",     1, 60,0,0, 2,100,7,3, 8, 10);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
