// =============================================================================
// easy_simt · icache 的 Verilator harness（开源单仿真路线）
//
// 结构：Verilator 把 submodules/icache/rtl/icache.sv 编译为 C++ 模型（Vicache）；
// 本 harness 扮演 ws 侧（取指请求源 + 指令消费者）与 memif 侧从设备，
// 参考侧直接链接 top/cmodel（icache_step），按事务语义逐拍锁步比对：
//   - 取指请求受理拍（缺失/响应在途不接受新请求）；
//   - 指令消费拍与指令字（命中/回填两路径）位精确；
//   - 回填请求呈现/受理拍与地址（行对齐字节地址）位精确；
//   - 回填数据消费拍（tb 从设备倒计时同构）与落阵列后命中一致性
//     （经指令字比对覆盖：冲突替换、指令段越界字为 0）；
//   - 协议保持：vld 拉起后保持、载荷稳定至握手；rdy 与状态一致；
//     复位期间全部 vld = 0。
//
// 从设备时序与 C 模型 memif countdown 同构（memif_spec §1.5）：回填请求
// 握手拍末沿装载 MEM_LAT 倒计时，计数到 0 呈现整行（呈现拍 = 请求握手拍
// + MEM_LAT + 1，含 MEM_LAT=0）。指令段后备存储口径与 cmodel 一致
// （top.c：按字索引 imem，超出程序长度 imem_n 的字返回 0）。
// tb 参考推进顺序（icache_spec §1.5）：置激励（取指 pc / 从设备整行）→
// icache_step → 按同拍 rdy 消费；四类事件逐拍比对。
//
// 波形：Verilator 原生 VCD（icache.vcd），gtkwave 查看。
// 判据：末尾 "VSIM PASS"。
// =============================================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vicache.h"
#include "sim_common.h"

// ---------------- 参考模型（cmodel 直链，仅用 icache 部分） ----------------
static sim_t ref;

// ---------------- 随机源（与 ialu/falu/lsu/rf/memif harness 同式） --------
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

// ---------------- 指令段后备存储（口径同 cmodel top.c） ----------------
static const int IMEM_N = 200;              /* 程序长度（字）：越界读为 0 */
static uint32_t s_imem[IMEM_WORDS];

// ---------------- 激励队列（vld 保持至握手） ----------------
struct Tx { uint32_t pc; };
static Tx pq[65536];
static int pq_h, pq_t;
static void emit(uint32_t pc) { pq[pq_t++ & 65535].pc = pc; }

static int errors, total_errors, cyc;
static vluint64_t gtime;   // VCD 单调时基
static void err(const char *msg)
{
    errors++;
    if (errors <= 20)
        printf("[vsim][ERR] cyc=%d %s\n", cyc, msg);
}

// ---------------- 消费者/从设备就绪决策（同式） ----------------
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
static Cfg cfg_rsp, cfg_mreq;               // 指令消费 / 回填请求受理背压
static int rdy_of(Cfg *c) { return rdy_dec(c->m, c->p, c->on, c->off, &c->pat); }

// ---------------- tb 侧 memif 从设备 ----------------
// 回填请求通道随机就绪；响应倒计时 = MEM_LAT（请求握手拍末沿装载，与
// C 模型 countdown 同构，memif_spec §1.5）；整行保持至 DUT 接收；
// 指令段按字索引、越界（imem_n 起）为 0。
static int SL_LAT;
static int sl_pend, sl_cnt, sl_rv;
static uint32_t sl_addr;
static uint32_t sl_line[ILINE_WORDS];

static void sl_present(void)
{
    uint32_t w0 = sl_addr >> 2;
    for (int i = 0; i < ILINE_WORDS; i++)
        sl_line[i] = (w0 + (uint32_t)i < (uint32_t)IMEM_N) ? s_imem[w0 + i] : 0u;
    sl_rv = 1;
}

// 上升沿调用：fires/载荷取本拍（边沿前）捕获的 DUT 输出副本
static void slave_edge(int mreq_fire, uint32_t maddr, int mrsp_fire)
{
    if (sl_rv) {
        if (mrsp_fire) { sl_rv = 0; sl_pend = 0; }
        return;
    }
    if (sl_pend) {
        if (sl_cnt > 1) { sl_cnt--; return; }
        sl_present();
        return;
    }
    if (mreq_fire) {
        sl_pend = 1;
        sl_addr = maddr;
        sl_cnt = SL_LAT;
        if (SL_LAT == 0) sl_present();
    }
}

// ---------------- DUT 状态镜像（rdy/vld 一致性检查用） ----------------
enum { M_IDLE = 0, M_REQ = 1, M_REFILL = 2, M_RSP = 3 };
static int mirror;

// ---------------- 协议保持影子 ----------------
static int p_rsp_v, p_rsp_r, p_mreq_v, p_mq;
static uint32_t p_inst, p_maddr;

// ---------------- 事务计数（汇报用） ----------------
static uint64_t cnt_fetch, cnt_mreq;

// ---------------- 单个用例 ----------------
static void run_test(Vicache *top, VerilatedVcdC *tfp, const char *name, int lat)
{
    SL_LAT = lat;
    errors = 0;
    cyc = 0;
    mirror = M_IDLE;
    p_rsp_v = p_mreq_v = 0;
    cnt_fetch = cnt_mreq = 0;
    sl_pend = sl_cnt = sl_rv = 0;
    sl_addr = 0;
    memset(sl_line, 0, sizeof sl_line);

    // ---- 指令段初始化（从设备后备存储，越界字恒 0） ----
    for (int w = 0; w < IMEM_WORDS; w++) s_imem[w] = rnd32();

    memset(&ref, 0, sizeof ref);

    // ---- 复位 ----
    top->rst_n = 0;
    top->ws_icache_req_vld = 0;
    top->ws_icache_req_pc = 0;
    top->ws_icache_rsp_rdy = 0;
    top->memif_icache_req_rdy = 0;
    top->memif_icache_rsp_vld = 0;
    for (int w = 0; w < ILINE_WORDS; w++) top->memif_icache_rsp_data[w] = 0;
    top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    for (int i = 0; i < 5; i++) {
        if (top->icache_memif_req_vld || top->icache_ws_rsp_vld)
            err("vld not 0 during reset");
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
    }
    top->rst_n = 1;

    int ntx = pq_t - pq_h;
    int cap = ntx * (lat + 40) + 4000;
    int finished = 0;
    while (!finished && cyc < cap) {
        // -- 消费者/从设备就绪决策（两路独立） --
        int r_ws = rdy_of(&cfg_rsp);
        int r_mq = rdy_of(&cfg_mreq);

        // -- 激励：队列头持续给出直至握手 --
        int offer = (pq_h < pq_t);
        top->ws_icache_req_vld = offer;
        top->ws_icache_req_pc = offer ? pq[pq_h & 65535].pc : 0;
        top->ws_icache_rsp_rdy = r_ws;
        top->memif_icache_req_rdy = r_mq;

        // -- 从设备响应呈现（寄存值，保持至 DUT 接收） --
        top->memif_icache_rsp_vld = sl_rv;
        for (int w = 0; w < ILINE_WORDS; w++) top->memif_icache_rsp_data[w] = sl_line[w];
        top->eval();

        // -- DUT 输出捕获 --
        int dut_rsp_v = top->icache_ws_rsp_vld;
        int dut_mreq_v = top->icache_memif_req_vld;
        uint32_t c_inst = top->icache_ws_rsp_inst;
        uint32_t c_maddr = top->icache_memif_req_addr;

        // -- 本拍事件 --
        int d_req_acc = top->ws_icache_req_vld && top->icache_ws_req_rdy;
        int d_rsp_con = dut_rsp_v && r_ws;
        int d_mreq = dut_mreq_v && r_mq;
        int d_mrsp = top->memif_icache_rsp_vld && top->icache_memif_rsp_rdy;

        // -- rdy/vld 与镜像状态一致性（icache_spec §2/§6） --
        if (top->icache_ws_req_rdy != (mirror == M_IDLE))
            err("ws req rdy inconsistent with state");
        if (top->icache_memif_rsp_rdy != (mirror == M_REFILL))
            err("memif rsp rdy inconsistent with state");
        if (dut_mreq_v != (mirror == M_REQ))
            err("memif req vld inconsistent with state");
        if (dut_rsp_v != (mirror == M_RSP))
            err("ws rsp vld inconsistent with state");

        // -- 协议保持检查：上一拍 vld && !rdy，则本拍 vld 不撤、载荷不变 --
        if (p_rsp_v && !p_rsp_r) {
            if (!dut_rsp_v)
                err("ws rsp vld dropped under backpressure");
            else if (c_inst != p_inst)
                err("ws rsp inst changed under backpressure");
        }
        if (p_mreq_v && !p_mq) {
            if (!dut_mreq_v)
                err("memif req vld dropped under backpressure");
            else if (c_maddr != p_maddr)
                err("memif req addr changed under backpressure");
        }

        // -- 参考同拍推进（置激励 → icache_step → 按同拍 rdy 消费） --
        if (offer && !ref.ws_icache_req.vld) {
            ref.ws_icache_req.p.pc = pq[pq_h & 65535].pc;
            ref.ws_icache_req.vld = 1;
        }
        if (sl_rv && !ref.memif_icache_rsp.vld) {
            for (int w = 0; w < ILINE_WORDS; w++)
                ref.memif_icache_rsp.p.line[w] = sl_line[w];
            ref.memif_icache_rsp.vld = 1;
        }
        int had_req = ref.ws_icache_req.vld;
        int had_mrsp = ref.memif_icache_rsp.vld;
        icache_step(&ref);
        int r_req_acc = had_req && !ref.ws_icache_req.vld;
        int r_mrsp = had_mrsp && !ref.memif_icache_rsp.vld;

        // -- 输出通道逐拍等价（呈现拍与载荷） --
        if (ref.icache_ws_rsp.vld) {
            if (!dut_rsp_v)
                err("ref ws rsp vld but DUT not presenting");
            else if (c_inst != ref.icache_ws_rsp.p.inst)
                err("ws rsp inst mismatch");
        } else if (dut_rsp_v) {
            err("DUT ws rsp vld but ref not presenting");
        }
        if (ref.icache_memif_req.vld) {
            if (!dut_mreq_v)
                err("ref memif req vld but DUT not presenting");
            else if (c_maddr != ref.icache_memif_req.p.addr)
                err("memif req addr mismatch");
        } else if (dut_mreq_v) {
            err("DUT memif req vld but ref not presenting");
        }

        // -- tb 扮 ws：按同拍 rdy 清参考响应通道 --
        int r_rsp_con = 0;
        if (ref.icache_ws_rsp.vld && r_ws) {
            ref.icache_ws_rsp.vld = 0;
            r_rsp_con = 1;
        }
        // -- tb 从设备：按 DUT 同拍握手清参考回填请求通道 --
        if (d_mreq) ref.icache_memif_req.vld = 0;

        // -- 逐拍一致性 --
        if (d_req_acc != r_req_acc) err("fetch req accept divergence");
        if (d_rsp_con != r_rsp_con) err("ws rsp consume divergence");
        if (d_mrsp != r_mrsp) err("memif rsp consume divergence");

        // -- 镜像推进（次拍状态，事件互斥于 DUT 状态） --
        if (d_req_acc) {
            mirror = ref.icache.miss ? M_REQ : M_RSP;
            pq_h++;
            cnt_fetch++;
        } else if (d_mreq) {
            mirror = M_REFILL;
            cnt_mreq++;
        } else if (d_mrsp) {
            mirror = M_RSP;
        } else if (d_rsp_con) {
            mirror = M_IDLE;
        }

        // -- 影子更新 --
        p_rsp_v = dut_rsp_v; p_rsp_r = r_ws; p_inst = c_inst;
        p_mreq_v = dut_mreq_v; p_mq = r_mq; p_maddr = c_maddr;

        // -- 时钟上升沿：DUT 寄存器与从设备同步更新 --
        top->clk = 1; top->eval(); if (tfp) tfp->dump(gtime++);
        slave_edge(d_mreq, c_maddr, d_mrsp);
        top->clk = 0; top->eval(); if (tfp) tfp->dump(gtime++);
        cyc++;

        if (pq_h == pq_t && mirror == M_IDLE && !sl_pend && !sl_rv &&
            !ref.ws_icache_req.vld && !ref.icache_ws_rsp.vld &&
            !ref.icache_memif_req.vld && !ref.memif_icache_rsp.vld &&
            !ref.icache.miss && !ref.icache.rsp_pending &&
            !dut_rsp_v && !dut_mreq_v)
            finished = 1;
    }

    if (cyc >= cap) err("test timeout");
    if (pq_h != pq_t) err("stimulus not drained");
    if (sl_pend || sl_rv) err("slave transaction pending at end");
    if (ref.icache.miss || ref.icache.rsp_pending)
        err("ref still busy at end");

    total_errors += errors;
    printf("[vsim] %-8s n=%-6d miss=%-6llu hit=%-6llu lat=%-3d cyc=%-8d -> %s\n",
           name, ntx,
           (unsigned long long)ref.st.icache_miss,
           (unsigned long long)(cnt_fetch - ref.st.icache_miss),
           SL_LAT, cyc, errors ? "FAIL" : "PASS");
}

// ============================================================================
//  激励生成
// ============================================================================
static void begin_stim(void) { pq_h = pq_t = 0; }

// ---- 用例 1：顺序取指两遍（首遍每行 1 缺失 + 7 命中，次遍全命中） ----
static void build_seq(void)
{
    for (int rep = 0; rep < 2; rep++)
        for (int w = 0; w < IMEM_WORDS; w++)
            emit((uint32_t)w);
}

// ---- 用例 2：热区命中（预热后全命中，b2b 响应） ----
static void build_hit(void)
{
    for (int w = 0; w < 32; w++)                /* 预热 4 行 */
        emit((uint32_t)w);
    for (int k = 0; k < 500; k++)
        emit((uint32_t)(rnd() % 32));
}

// ---- 用例 3：冲突替换（同索引不同 tag 交替，t 与 t+16 同行号） ----
static void build_conf(void)
{
    for (int k = 0; k < 300; k++) {
        uint32_t half = (uint32_t)(k / 2) % 8;
        emit((k & 1) ? 128u + half : half);      /* t=16 与 t=0 同行 0 */
    }
}

// ---- 用例 4：边界（pc=0、程序长度边界行、越界全 0、最高字） ----
static void build_edge(void)
{
    emit(0u);
    emit((uint32_t)(IMEM_N - 1));               /* 字 199 */
    emit((uint32_t)(IMEM_N - 4));               /* w0=196：跨 imem_n 行 */
    emit((uint32_t)IMEM_N);                     /* 全 0 行 */
    emit((uint32_t)(IMEM_WORDS - 1));           /* w0=252：越出映射一行 */
    emit((uint32_t)(IMEM_WORDS - 8));
    emit((uint32_t)(IMEM_N + 5));               /* 越界行中段 */
    /* 重复回读（命中路径返回回填数据） */
    emit(0u); emit((uint32_t)(IMEM_N - 4)); emit((uint32_t)(IMEM_WORDS - 1));
    /* 随机补充 */
    for (int k = 0; k < 60; k++)
        emit((uint32_t)(rnd() % IMEM_WORDS));
}

// ---- 随机混合 ----
static void gen_random(int n)
{
    for (int i = 0; i < n; i++)
        emit((uint32_t)(rnd() % IMEM_WORDS));
}

// ---- 用例配置 ----
static void cfg_all(int rm, int rp, int qm, int qp)
{
    cfg_rsp.m = rm; cfg_rsp.p = rp; cfg_rsp.on = cfg_rsp.off = 0; cfg_rsp.pat = 0;
    cfg_mreq.m = qm; cfg_mreq.p = qp; cfg_mreq.on = cfg_mreq.off = 0; cfg_mreq.pat = 0;
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    Vicache *top = new Vicache;
    VerilatedVcdC *tfp = new VerilatedVcdC;
    Verilated::traceEverOn(true);
    top->trace(tfp, 99);
    tfp->open("icache.vcd");

    seed_g = 1; begin_stim(); build_seq();
    cfg_all(0,100, 0,100);
    run_test(top, tfp, "seq", 1);

    seed_g = 2; begin_stim(); build_hit();
    cfg_all(0,100, 0,100);
    run_test(top, tfp, "hit", 1);

    seed_g = 3; begin_stim(); build_conf();
    cfg_all(0,100, 0,100);
    run_test(top, tfp, "conf", 2);

    seed_g = 4; begin_stim(); build_edge();
    cfg_all(0,100, 0,100);
    run_test(top, tfp, "edge", 0);

    seed_g = 5; begin_stim(); gen_random(2000);
    cfg_all(0,100, 0,100);
    run_test(top, tfp, "b2b", 2);

    seed_g = 6; begin_stim(); gen_random(4000);
    cfg_all(1, 50, 0,100);
    run_test(top, tfp, "rand50", 3);

    seed_g = 7; begin_stim(); gen_random(4000);
    cfg_all(1, 20, 1, 20);
    run_test(top, tfp, "rand20", 5);

    // 回填请求受理背压（S_REQ 保持 vld、拍序顺延）
    seed_g = 8; begin_stim(); gen_random(2000);
    cfg_all(1, 80, 1, 30);
    run_test(top, tfp, "rreq", 2);

    seed_g = 9; begin_stim(); gen_random(2000);
    cfg_rsp.m = 2; cfg_rsp.p = 0; cfg_rsp.on = 3; cfg_rsp.off = 2; cfg_rsp.pat = 0;
    cfg_mreq.m = 0; cfg_mreq.p = 100; cfg_mreq.on = cfg_mreq.off = 0; cfg_mreq.pat = 0;
    run_test(top, tfp, "pat", 8);

    seed_g = 10; begin_stim(); gen_random(10000);
    cfg_all(1, 60, 1, 60);
    run_test(top, tfp, "soak", 8);

    tfp->close();
    delete top;
    if (total_errors == 0)
        printf("[vsim] VSIM PASS (10 tests, 0 errors)\n");
    else
        printf("[vsim] VSIM FAIL (%d errors)\n", total_errors);
    return total_errors ? 1 : 0;
}
