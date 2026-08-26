/* ==========================================================================
 * easy_simt · top/cmodel · sim_common.h
 *
 * 事务级（transaction-accurate）C 模型公共定义。
 * 依据：top/docs/ma_spec_v0.1.md、intf_spec_v0.1.md、isa_spec_v0.1.md。
 *
 * 模型约定：
 *   - 不建模时钟。通道为深度 1 的 vld/rdy 单槽握手，vld && rdy 成立即
 *     "发射"一笔事务；顶层按固定顺序轮询各模块 step()。
 *   - memif 的 MEM_LAT 建模为响应路径上的倒计时（对应 RTL 中 tb 侧
 *     AXI 从设备的固定延迟；intf_spec §10：memif 自身不加延迟）。
 *   - 全部状态收进 sim_t 上下文，无全局变量（为将来 DPI 接入预留）。
 *
 * 模型约束（已知与规范的偏差，均在此声明）：
 *   C1. intf_spec §2 的 sf_lsu_issue 载荷未含第二个逐 lane 操作数，
 *       存储类指令需要"每 lane 数据 + 每 lane 偏移"两个向量，本模型
 *       增设 opb 字段，intf_spec 需补勘误。
 *   C2. LDG/STG 的 ra（基址）须为活动 lane 间同值（黄金程序由 LDP
 *       广播保证）；sf 校验该约束，违反置错误标志。这是单 kernel
 *       原型机的已知限制。
 *   C3. LDP/CSRR 经 ialu 直通路径写回（sf 将参数/特殊寄存器值广播进
 *       opa，ialu 原样产出 wdata），以保持 rf 写口三源结构不变。
 *   C4. JOIN 由 sf 本地处置（仅涉及分化栈，无 ialu 决议需求）。
 *   C5. IMAD 为 R3 型需三个逐 lane 源，sf_ialu_issue 增设 opc 向量，
 *       intf_spec 需补勘误。
 * ========================================================================== */
#ifndef SIM_COMMON_H
#define SIM_COMMON_H

#include <stdint.h>

/* ---------------- 参数位（默认取 v1 最简值，可 -D 覆盖） ---------------- */
#ifndef NLANES
#define NLANES          8       /* lane/warp */
#endif
#ifndef NWARPS
#define NWARPS          4       /* warp/块（ma_spec §1.2） */
#endif
#define MAX_BLOCKS_INFLIGHT 1   /* v1 串行块 */
#define DIV_STACK_DEPTH 4       /* 分化栈深 */
#define BRT_ENTRIES     4       /* 重聚表表项（ma_spec §1.6） */
#define ILINE_WORDS     8       /* 32B 行 = 8 条指令 */
#ifndef ICACHE_LINES
#define ICACHE_LINES    16      /* icache 行数（512B） */
#endif
#ifndef U_LINES
#define U_LINES         64      /* 统一 SRAM 总行数（2KB） */
#endif
#ifndef SM_LINES
#define SM_LINES        4       /* SM 分区行数（128B，块启动钉扎） */
#endif
#define NBANKS          8       /* = NLANES，锁步单拍 */
#ifndef IMEM_WORDS
#define IMEM_WORDS      256     /* 程序镜像容量上限 */
#endif
#ifndef GMEM_WORDS
#define GMEM_WORDS      4096    /* 每段全局存储容量（字） */
#endif

/* ---------------- ISA 操作码（isa_spec §1.8） ---------------- */
enum {
    OP_IMAD = 0x01, OP_IADD = 0x02, OP_SHL = 0x03, OP_XOR = 0x04,
    OP_ORI  = 0x05, OP_LUI  = 0x06, OP_SETP = 0x07, OP_FMUL = 0x08,
    OP_FADD = 0x09, OP_FNEG = 0x0A, OP_LDG  = 0x0B, OP_STG  = 0x0C,
    OP_LDS  = 0x0D, OP_STS  = 0x0E, OP_LDP  = 0x0F, OP_CSRR = 0x10,
    OP_BR   = 0x11, OP_JOIN = 0x12, OP_BAR  = 0x13, OP_RET  = 0x14,
    OP_NCOUNT = 0x15
};

/* ---------------- 停顿原因（intf_spec §1.5） ---------------- */
enum {
    R_NONE = 0, R_HAZARD = 1, R_IMISS = 2, R_LMISS = 3,
    R_BARRIER = 4, R_BRSTALL = 5, R_DONE = 6
};

/* ---------------- 基本向量类型 ---------------- */
typedef uint32_t lanev_t[NLANES];   /* 8×32b 逐 lane 数据 */

/* ---------------- 通道事务载荷（与 intf_spec 端口表一一对应） ---------------- */
/* bs_sf_launch / bs_ws_launch */
typedef struct { uint32_t block_idx, n, shbase; } launch_t;
/* ws_sf_grant */
typedef struct { int warp_id; } grant_t;
/* sf_ws_stall / lsu_ws_stall */
typedef struct { int warp_id, reason; } stall_t;
/* sf_ws_bar */
typedef struct { int warp_id; uint32_t block_id; } bar_t;
/* sf_icache_req */
typedef struct { uint32_t pc; } fetch_req_t;
/* icache_sf_rsp */
typedef struct { uint32_t inst; } fetch_rsp_t;
/* sf_rf_rd */
typedef struct { int warp_id, rs1, rs2; } rf_rd_t;
/* rf_sf_rddata */
typedef struct { lanev_t a, b; } rf_rddata_t;
/* sf_ialu_issue */
typedef struct {
    int opcode, rd, warp_id;
    uint8_t lane_mask;
    uint32_t pc, imm;
    lanev_t opa, opb, opc;      /* opc：R3 型（IMAD）第三源，见偏差 C5 */
} ialu_issue_t;
/* sf_falu_issue */
typedef struct {
    int opcode, rd, warp_id;
    uint8_t lane_mask;
    lanev_t opa, opb;
} falu_issue_t;
/* sf_lsu_issue（含 C1 声明的 opb 扩展） */
typedef struct {
    int opcode, rd, warp_id;
    uint8_t lane_mask;
    uint32_t imm, shbase;
    lanev_t opa, opb;
} lsu_issue_t;
/* ialu_sf_br */
typedef struct {
    int warp_id;
    uint8_t taken;
    uint32_t target, brt_idx;
} br_t;
/* {ialu,falu,lsu}_rf_wb */
typedef struct {
    int warp_id, rd;
    uint8_t lane_mask;
    lanev_t wdata;
} wb_t;
/* {ialu,falu,lsu}_sf_wbdone */
typedef struct { int warp_id, rd; } wbdone_t;
/* lsu_l1sm_req */
typedef struct {
    int rw, sm;                 /* rw: 0读 1写；sm: 1共享 0全局 */
    lanev_t addr, wdata;
    uint8_t mask;
} l1sm_req_t;
/* l1sm_lsu_rsp */
typedef struct { lanev_t rdata; } l1sm_rsp_t;
/* icache_memif_req */
typedef struct { uint32_t addr; } memif_ireq_t;
/* memif_icache_rsp */
typedef struct { uint32_t line[ILINE_WORDS]; } memif_irsp_t;
/* l1sm_memif_req */
typedef struct { int rw; uint32_t addr, wdata; } memif_dreq_t;
/* memif_l1sm_rsp */
typedef struct { uint32_t line[ILINE_WORDS]; } memif_drsp_t;
/* ws_bs_bdone */
typedef struct { uint32_t block_idx; } bdone_t;

/* ---------------- 深度 1 的 vld/rdy 通道 ---------------- */
typedef struct { int vld; launch_t p; }      chan_launch_t;
typedef struct { int vld; bdone_t p; }       chan_bdone_t;
typedef struct { int vld; grant_t p; }       chan_grant_t;
typedef struct { int vld; stall_t p; }       chan_stall_t;
typedef struct { int vld; bar_t p; }         chan_bar_t;
typedef struct { int vld; fetch_req_t p; }   chan_fetch_req_t;
typedef struct { int vld; fetch_rsp_t p; }   chan_fetch_rsp_t;
typedef struct { int vld; rf_rd_t p; }       chan_rf_rd_t;
typedef struct { int vld; rf_rddata_t p; }   chan_rf_rddata_t;
typedef struct { int vld; ialu_issue_t p; }  chan_ialu_issue_t;
typedef struct { int vld; falu_issue_t p; }  chan_falu_issue_t;
typedef struct { int vld; lsu_issue_t p; }   chan_lsu_issue_t;
typedef struct { int vld; br_t p; }          chan_br_t;
typedef struct { int vld; wb_t p; }          chan_wb_t;
typedef struct { int vld; wbdone_t p; }      chan_wbdone_t;
typedef struct { int vld; l1sm_req_t p; }    chan_l1sm_req_t;
typedef struct { int vld; l1sm_rsp_t p; }    chan_l1sm_rsp_t;
typedef struct { int vld; memif_ireq_t p; }  chan_memif_ireq_t;
typedef struct { int vld; memif_irsp_t p; }  chan_memif_irsp_t;
typedef struct { int vld; memif_dreq_t p; }  chan_memif_dreq_t;
typedef struct { int vld; memif_drsp_t p; }  chan_memif_drsp_t;

/* ---------------- 统计 ---------------- */
typedef struct {
    uint64_t issued;            /* 发射 warp 指令数 */
    uint64_t lane_insns;        /* lane 等效指令数 */
    uint64_t op_count[OP_NCOUNT];
    uint64_t shmem_ops;         /* warp 粒度 LDS/STS */
    uint64_t ldg, stg;
    uint64_t uniform_br;
    uint64_t diverge[IMEM_WORDS];   /* 按分支 pc 计分化 warp 数 */
    uint64_t icache_miss, l1_miss, l1sm_conflict;
    uint64_t memif_rd, memif_wr, memif_icache_rd;
    uint64_t fires;             /* 全通道发射事务总数 */
} stats_t;

/* ---------------- 前向声明与模块状态 ---------------- */
typedef struct sim_s sim_t;

/* bs */
typedef struct {
    int started;                /* CTRL 启动后 */
    int inflight;               /* 有块在途 */
    int done;                   /* grid 结束（bs_top_done） */
    uint32_t block_idx;
} bs_t;

/* ws */
typedef struct {
    int launched;               /* 本块已收 launch */
    uint32_t block_id;
    int sf_reason[NWARPS];      /* 来自 sf 的停顿原因锁存 */
    int lsu_reason[NWARPS];     /* 来自 lsu 的停顿原因锁存 */
    int barrier[NWARPS];        /* 到达屏障 */
    int bar_count;
    int done[NWARPS];
    int ptr;                    /* 2 位轮转指针 */
    int grant_warp;             /* 待命授予的 warp（-1 无） */
    int bdone_sent;             /* block_done 已置位待消费 */
} ws_t;

/* sf 每 warp 状态 */
enum {
    WS_IDLE = 0, WS_FETCH, WS_HAZ, WS_RD1, WS_RD2, WS_ISSUE,
    WS_EXEC, WS_BR, WS_BAR, WS_DONE
};
typedef struct {
    uint32_t pc, mask;
    uint32_t rpc[DIV_STACK_DEPTH];  /* 分化栈：重聚 PC */
    uint8_t  dmask[DIV_STACK_DEPTH];/* 分化栈：掩码 */
    int dsp;
    int state;
    uint32_t inst;              /* 已取回指令 */
    int rd_pending;             /* 记分板待清除目的寄存器（-1 无） */
    uint32_t br_pc;             /* 在途 BR 的 pc（BRT 查表用） */
    lanev_t rd_data_a, rd_data_b; /* 第一阶段读口数据暂存 */
    lanev_t rd_data_c;          /* 第二阶段读回（存储数据等） */
    int rd_phase;               /* 0 无需/未始 1/-1 第一阶段 2/-2 第二阶段 */
} sf_warp_t;

typedef struct {
    int active;                 /* 已收 launch */
    uint32_t block_idx, n, shbase;
    sf_warp_t w[NWARPS];
    uint32_t sb_busy[NWARPS];   /* 记分板：每 warp 32 寄存器忙位 */
    int lsu_out[NWARPS];        /* 该 warp 在 lsu 的未退休访存计数 */
    int fetch_warp;             /* 在途取指 warp（-1 无） */
    int fetch_pend;             /* 取指请求待置位 */
    int rd_warp;                /* 在途 rf 读 warp（-1 无） */
    int des_reason[NWARPS];     /* 期望上报的停顿原因 */
    int cur_reason[NWARPS];     /* 已上报的停顿原因 */
    int bar_pending[NWARPS];    /* 屏障到达消息待发送 */
    int err;                    /* sf_top_err 锁存 */
} sf_t;

/* rf */
typedef struct {
    uint32_t r[NWARPS][32][NLANES];
} rf_t;

/* ialu */
typedef struct {
    uint8_t pred[NWARPS][4];    /* 谓词 P0..P3，每 warp 8 lane 位图 */
    int has_issue;              /* 在途指令占用 */
    ialu_issue_t iss;
    int wb_stage;               /* 0 无 / 1 wb 待发射 / 2 wbdone 待发射 */
    wb_t wb;
    wbdone_t wbd;
    int br_stage;               /* BR 决议待发射 */
    br_t br;
    int br_sent, wb_sent, wbd_sent;
} ialu_t;

/* falu */
typedef struct {
    int has_issue;
    falu_issue_t iss;
    int wb_stage;
    wb_t wb;
    wbdone_t wbd;
    int wb_sent, wbd_sent;
} falu_t;

/* lsu */
typedef struct {
    int busy;
    lsu_issue_t iss;
    l1sm_req_t req;
    int req_stage;              /* 0 空闲 / 1 置位 / 2 等收 / 3 等 rsp / 4 wb / 5 wbdone */
    int req_sent;               /* 当前阶段的发送跟踪 */
    wb_t wb;
    wbdone_t wbd;
    int stall_sent;             /* LMISS 停顿上报未清 */
} lsu_t;

/* icache */
typedef struct {
    uint32_t data[ICACHE_LINES][ILINE_WORDS];
    uint32_t tag[ICACHE_LINES];
    int valid[ICACHE_LINES];
    int miss;                   /* 缺失在途 */
    uint32_t miss_pc;
    int req_sent;               /* 回填请求已置位待消费 */
    int rsp_pending;            /* 命中/回填完成的指令待回送 */
    int rsp_sent;               /* 响应已置位待消费 */
    uint32_t rsp_inst;
} icache_t;

/* l1sm */
typedef struct {
    uint32_t data[U_LINES][NBANKS];
    uint8_t  cls[U_LINES];      /* 00 INV / 01 L1 / 10 SM */
    int      valid[U_LINES];
    uint32_t tag[U_LINES];
    uint32_t shbase;
    int busy;                   /* 在途请求 */
    l1sm_req_t req;
    int stage;                  /* L_* 状态 */
    int glane;                  /* 逐 lane 处理游标 */
    int req_sent;               /* memif 请求已置位待消费 */
    int rsp_sent;               /* lsu 响应已置位待消费 */
    l1sm_rsp_t rsp;
} l1sm_t;

/* memif（含 C 模型侧的片外存储与 AXI 从设备延迟） */
typedef struct {
    /* 片外存储：指令段 + 两段全局数据 */
    uint32_t imem[IMEM_WORDS];
    int      imem_n;
    uint32_t gmem_in[GMEM_WORDS], gmem_out[GMEM_WORDS];
    uint32_t in_base, out_base;
    int memlat;
    /* 在途事务 */
    int busy;
    int to_icache;              /* 1=icache 回填 / 0=l1sm */
    int rw;
    uint32_t addr, wdata;
    int countdown;
    int rsp_set;                /* 响应已置位待消费 */
    int err;                    /* memif_top_err */
} memif_t;

/* ---------------- 顶层上下文 ---------------- */
struct sim_s {
    /* 配置 */
    int n, nwarps, nlanes, grid;
    uint32_t params[3];

    /* 程序与 BRT */
    int n_instr;
    uint32_t brt_rpc[IMEM_WORDS];   /* pc -> 重聚 pc（无表项 = 0 且见 brt_valid） */
    int brt_valid[IMEM_WORDS];

    /* 通道（源_宿 命名） */
    chan_launch_t     bs_sf_launch;
    chan_launch_t     bs_ws_launch;
    chan_bdone_t      ws_bs_bdone;
    chan_grant_t      ws_sf_grant;
    chan_stall_t      sf_ws_stall;
    chan_bar_t        sf_ws_bar;
    chan_stall_t      lsu_ws_stall;
    chan_fetch_req_t  sf_icache_req;
    chan_fetch_rsp_t  icache_sf_rsp;
    chan_rf_rd_t      sf_rf_rd;
    chan_rf_rddata_t  rf_sf_rddata;
    chan_ialu_issue_t sf_ialu_issue;
    chan_falu_issue_t sf_falu_issue;
    chan_lsu_issue_t  sf_lsu_issue;
    chan_br_t         ialu_sf_br;
    chan_wb_t         ialu_rf_wb;
    chan_wb_t         falu_rf_wb;
    chan_wb_t         lsu_rf_wb;
    chan_wbdone_t     ialu_sf_wbdone;
    chan_wbdone_t     falu_sf_wbdone;
    chan_wbdone_t     lsu_sf_wbdone;
    chan_l1sm_req_t   lsu_l1sm_req;
    chan_l1sm_rsp_t   l1sm_lsu_rsp;
    chan_memif_ireq_t icache_memif_req;
    chan_memif_irsp_t memif_icache_rsp;
    chan_memif_dreq_t l1sm_memif_req;
    chan_memif_drsp_t memif_l1sm_rsp;

    /* 模块状态 */
    bs_t bs; ws_t ws; sf_t sf; rf_t rf;
    ialu_t ialu; falu_t falu; lsu_t lsu;
    icache_t icache; l1sm_t l1sm; memif_t memif;

    stats_t st;
    int done, err;              /* bs_top_done / 全局错误锁存 */
    uint64_t rounds;
};

/* ---------------- l1sm tag class（ma_spec §9） ---------------- */
enum { CLS_INV = 0, CLS_L1 = 1, CLS_SM = 2 };

/* ---------------- 模块接口 ---------------- */
void sim_init(sim_t *s);
int  sim_load_hex(sim_t *s, const char *path);
int  sim_load_brt_json(sim_t *s, const char *path);
void sim_block_start(sim_t *s);
int  sim_run(sim_t *s);

int bs_step(sim_t *s);
int ws_step(sim_t *s);
int sf_step(sim_t *s);
int rf_step(sim_t *s);
int ialu_step(sim_t *s);
int falu_step(sim_t *s);
int lsu_step(sim_t *s);
int icache_step(sim_t *s);
int l1sm_step(sim_t *s);
int memif_step(sim_t *s);

/* 软浮点（与 verify 脚本 iss.py 的 FP32 RN 语义逐位一致，含 FTZ） */
uint32_t f32_mul(uint32_t a, uint32_t b);
uint32_t f32_add(uint32_t a, uint32_t b);
uint32_t f32_neg(uint32_t a);
int      f32_gt(uint32_t a, uint32_t b);

#endif /* SIM_COMMON_H */
