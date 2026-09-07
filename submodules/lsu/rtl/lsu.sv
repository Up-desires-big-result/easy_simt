// =============================================================================
// easy_simt · lsu — Load/Store Unit（ma_spec §7）
//
// 保持薄：不含 tag 比较、不含阵列/bank。per-lane 地址生成（基址+偏移，
// 共享再叠 SHBASE）、active mask 门控、8-lane 锁步一拍发出一个 8-lane
// 请求、请求分 shmem/global 两路、装载数据引导写回、向 ws 报停顿。
//
// 完成路径（lsu_spec §1.2）：
//   LDG/LDS：issue -> req -> rsp -> wb -> wbdone
//   STG/STS：issue -> req -> rsp（写应答）-> wbdone（存储不写 rf）
//
// 状态机与 C 模型（top/cmodel/lsu.c）逐拍对应（lsu_spec §1.5）：C 模型
// 每级通道事件（置位、接收检测）各占一步，RTL 以对应状态逐拍镜像——
// S_WAIT 为请求接收检测拍、S_WBW 为写回接收检测拍、S_IDLE 兼作写回
// 完成接收检测拍与发射拍，使 testbench 逐拍锁步比对成立。
//
// 停顿上报（lsu_spec §5.4）：LMISS 随发射握手当拍组合置位、R_NONE 于
// 空闲拍组合置位（与 C 模型同拍上报口径一致），未被消费则锁存保持；
// 通道在途或清除未发时发射的指令不报 LMISS（与 C 模型一致）。
//
// 设计依据：lsu/docs/lsu_spec_v0.1.md；
//   端口命名与 intf_spec §7 一致（issue 载荷见 intf_spec §2；
//   sf_lsu_issue_opb 为偏差 C1 增设字段，与 cmodel 一致），
//   握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module lsu #(
    parameter DATA_W   = 32,              // 数据/地址/载荷位宽（intf_spec §1.4）
    parameter NWARPS   = 4,               // warp 数/块（ma_spec §1.2）
    parameter NLANES   = 8,               // lane 数/warp（ma_spec §1.2）
    parameter REG_AW   = 5,               // 寄存器地址位宽（intf_spec §1.4）
    parameter OPCODE_W = 5,               // 操作码位宽（intf_spec §1.4）
    localparam WARP_IW = (NWARPS > 1) ? $clog2(NWARPS) : 1,
    localparam VEC_W   = NLANES * DATA_W
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // sf_lsu_issue：sf 发射（载荷为 sf 译码归一化形式，lsu_spec §1.3）
    input  wire                 sf_lsu_issue_vld,
    input  wire [OPCODE_W-1:0]  sf_lsu_issue_opcode,
    input  wire [REG_AW-1:0]    sf_lsu_issue_rd,
    input  wire [WARP_IW-1:0]   sf_lsu_issue_warp_id,
    input  wire [NLANES-1:0]    sf_lsu_issue_lane_mask,
    input  wire [VEC_W-1:0]     sf_lsu_issue_opa,
    input  wire [VEC_W-1:0]     sf_lsu_issue_opb,   // 偏差 C1 增设（lsu_spec §2）
    input  wire [DATA_W-1:0]    sf_lsu_issue_imm,
    input  wire [DATA_W-1:0]    sf_lsu_issue_shbase,
    output wire                 lsu_sf_issue_rdy,

    // lsu_l1sm_req：8-lane 锁步访存请求（intf_spec §7）
    output wire                 lsu_l1sm_req_vld,
    output wire                 lsu_l1sm_req_rw,
    output wire                 lsu_l1sm_req_sm,
    output wire [VEC_W-1:0]     lsu_l1sm_req_addr,
    output wire [VEC_W-1:0]     lsu_l1sm_req_wdata,
    output wire [NLANES-1:0]    lsu_l1sm_req_mask,
    input  wire                 l1sm_lsu_req_rdy,

    // l1sm_lsu_rsp：访存响应（装载读数据 / 写应答；单在途、与请求严格顺序对应）
    input  wire                 l1sm_lsu_rsp_vld,
    input  wire [VEC_W-1:0]     l1sm_lsu_rsp_rdata,
    output wire                 lsu_l1sm_rsp_rdy,

    // lsu_ws_stall：访存停顿上报（LMISS 置位 / R_NONE 清除，lsu_spec §5.4）
    output wire                 lsu_ws_stall_vld,
    output wire [WARP_IW-1:0]   lsu_ws_stall_warp_id,
    output wire [2:0]           lsu_ws_stall_reason,
    input  wire                 ws_lsu_stall_rdy,

    // lsu_rf_wb：装载数据写回（仅装载；存储不写 rf）
    output wire                 lsu_rf_wb_vld,
    output wire [WARP_IW-1:0]   lsu_rf_wb_warp_id,
    output wire [REG_AW-1:0]    lsu_rf_wb_rd,
    output wire [NLANES-1:0]    lsu_rf_wb_lane_mask,
    output wire [VEC_W-1:0]     lsu_rf_wb_wdata,
    input  wire                 rf_lsu_wb_rdy,

    // lsu_sf_wbdone：写回完成（供 sf 清记分板 / 上报排空状态）
    output wire                 lsu_sf_wbdone_vld,
    output wire [WARP_IW-1:0]   lsu_sf_wbdone_warp_id,
    output wire [REG_AW-1:0]    lsu_sf_wbdone_rd,
    input  wire                 sf_lsu_wbdone_rdy
);

    // ---------------- 操作码（isa_spec §1.8，本模块合法子集） ----------------
    localparam [OPCODE_W-1:0] OP_LDG = 5'h0B;
    localparam [OPCODE_W-1:0] OP_STG = 5'h0C;
    localparam [OPCODE_W-1:0] OP_LDS = 5'h0D;
    localparam [OPCODE_W-1:0] OP_STS = 5'h0E;

    // ---------------- 停顿原因（intf_spec §3 stall_t） ----------------
    localparam [2:0] R_NONE  = 3'd0;
    localparam [2:0] R_LMISS = 3'd3;

    // ---------------- 状态编码（lsu_spec §6.1） ----------------
    localparam [2:0] S_IDLE = 3'd0;   // 空闲：wbdone 接收检测拍兼发射拍
    localparam [2:0] S_REQ  = 3'd1;   // 请求段：呈现 lsu_l1sm_req
    localparam [2:0] S_WAIT = 3'd2;   // 请求接收检测拍（镜像 C 模型 stage 2）
    localparam [2:0] S_RSP  = 3'd3;   // 响应段：呈现 lsu_l1sm_rsp_rdy
    localparam [2:0] S_WB   = 3'd4;   // 写回段：呈现 lsu_rf_wb（仅装载）
    localparam [2:0] S_WBW  = 3'd5;   // 写回接收检测拍（镜像 C 模型 stage 4 检测）
    localparam [2:0] S_WBD  = 3'd6;   // 写回完成段：呈现 lsu_sf_wbdone

    // ---------------- 内部状态（lsu_spec §4） ----------------
    reg  [2:0]          state;
    reg                 req_rw, req_sm;
    reg  [NLANES-1:0]   req_mask;
    reg  [VEC_W-1:0]    req_addr, req_wdata;
    reg  [WARP_IW-1:0]  wb_warp_id, wbd_warp_id;
    reg  [REG_AW-1:0]   wb_rd, wbd_rd;
    reg  [NLANES-1:0]   wb_lane_mask;
    reg  [VEC_W-1:0]    wb_wdata;
    reg                 stall_sent;            // LMISS 已上报、清除（R_NONE）未发
    reg  [WARP_IW-1:0]  stall_warp;            // 最近一次发射的 warp_id（R_NONE 载荷）
    reg                 stall_busy;            // 停顿消息在途（锁存保持）
    reg  [WARP_IW-1:0]  stall_pwarp;
    reg  [2:0]          stall_preason;

    // ---------------- 握手组合判据 ----------------
    wire issue_fire = sf_lsu_issue_vld && lsu_sf_issue_rdy;
    wire req_fire   = lsu_l1sm_req_vld  && l1sm_lsu_req_rdy;
    wire rsp_fire   = l1sm_lsu_rsp_vld  && lsu_l1sm_rsp_rdy;
    wire wb_fire    = lsu_rf_wb_vld     && rf_lsu_wb_rdy;
    wire wbd_fire   = lsu_sf_wbdone_vld && sf_lsu_wbdone_rdy;

    // ---------------- 通道 vld / rdy（寄存器输出经状态译码，lsu_spec §4） ----
    assign lsu_sf_issue_rdy  = (state == S_IDLE);
    assign lsu_l1sm_req_vld  = (state == S_REQ);
    assign lsu_l1sm_rsp_rdy  = (state == S_RSP);
    assign lsu_rf_wb_vld     = (state == S_WB);
    assign lsu_sf_wbdone_vld = (state == S_WBD);

    // ---------------- per-lane 地址生成与请求拼装（lsu_spec §5.1/§5.2） ------
    // 地址 = 基址 + 偏移：LDG opa+imm、STG opb+imm、LDS shbase+opa、STS shbase+opb；
    // mask 外 lane 地址/数据置 0（与 C 模型一致）；存储数据取 opa
    wire is_store = (sf_lsu_issue_opcode == OP_STG) ||
                    (sf_lsu_issue_opcode == OP_STS);
    wire is_sm    = (sf_lsu_issue_opcode == OP_LDS) ||
                    (sf_lsu_issue_opcode == OP_STS);

    wire [VEC_W-1:0] addr_c, wdata_c;
    genvar gi;
    generate
        for (gi = 0; gi < NLANES; gi = gi + 1) begin : g_lane
            wire [DATA_W-1:0] a = sf_lsu_issue_opa[gi*DATA_W +: DATA_W];
            wire [DATA_W-1:0] b = sf_lsu_issue_opb[gi*DATA_W +: DATA_W];
            wire              act = sf_lsu_issue_lane_mask[gi];
            wire [DATA_W-1:0] off  = is_store ? b : a;
            wire [DATA_W-1:0] base = is_sm ? sf_lsu_issue_shbase
                                           : sf_lsu_issue_imm;
            assign addr_c[gi*DATA_W +: DATA_W]  = act ? (base + off)
                                                       : {DATA_W{1'b0}};
            assign wdata_c[gi*DATA_W +: DATA_W] = (act && is_store) ? a
                                                                    : {DATA_W{1'b0}};
        end
    endgenerate

    // ---------------- 停顿上报通路（lsu_spec §5.4） --------------------------
    // LMISS：发射握手当拍组合置位（载荷取发射 warp_id）；
    // R_NONE：空闲拍（stall_sent=1 且通道空闲）组合置位（载荷取最近一次
    //         发射的 warp_id）；二者未被消费则于拍末沿锁存保持；
    // 通道在途（stall_busy）或清除未发（stall_sent）时发射的指令不报 LMISS
    wire lmiss_go = issue_fire && !stall_sent && !stall_busy;
    wire clear_go = (state == S_IDLE) && stall_sent && !stall_busy;

    assign lsu_ws_stall_vld     = stall_busy || lmiss_go || clear_go;
    assign lsu_ws_stall_warp_id = lmiss_go ? sf_lsu_issue_warp_id
                                : clear_go ? stall_warp
                                :            stall_pwarp;
    assign lsu_ws_stall_reason  = lmiss_go ? R_LMISS
                                : clear_go ? R_NONE
                                :            stall_preason;
    wire stall_fire = lsu_ws_stall_vld && ws_lsu_stall_rdy;

    // ---------------- 主状态机与载荷入级（lsu_spec §6） ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            req_rw       <= 1'b0;
            req_sm       <= 1'b0;
            req_mask     <= {NLANES{1'b0}};
            req_addr     <= {VEC_W{1'b0}};
            req_wdata    <= {VEC_W{1'b0}};
            wb_warp_id   <= {WARP_IW{1'b0}};
            wb_rd        <= {REG_AW{1'b0}};
            wb_lane_mask <= {NLANES{1'b0}};
            wb_wdata     <= {VEC_W{1'b0}};
            wbd_warp_id  <= {WARP_IW{1'b0}};
            wbd_rd       <= {REG_AW{1'b0}};
            stall_sent   <= 1'b0;
            stall_warp   <= {WARP_IW{1'b0}};
            stall_busy   <= 1'b0;
            stall_pwarp  <= {WARP_IW{1'b0}};
            stall_preason<= R_NONE;
        end else begin
            // ---- 状态迁移（§6.3） ----
            case (state)
            S_IDLE: begin
                if (issue_fire) state <= S_REQ;
            end
            S_REQ: begin
                if (req_fire) state <= S_WAIT;
            end
            S_WAIT: begin
                state <= S_RSP;              // 请求接收检测拍
            end
            S_RSP: begin
                // 装载：读数据入级进 S_WB；存储：写应答已交付，直接进 S_WBD
                if (rsp_fire) state <= req_rw ? S_WBD : S_WB;
            end
            S_WB: begin
                if (wb_fire) state <= S_WBW;
            end
            S_WBW: begin
                state <= S_WBD;              // 写回接收检测拍
            end
            S_WBD: begin
                if (wbd_fire) state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase

            // ---- issue 载荷入级（发射握手拍末沿，§5.1/§5.2） ----
            if (issue_fire) begin
                req_rw       <= is_store;
                req_sm       <= is_sm;
                req_mask     <= sf_lsu_issue_lane_mask;
                req_addr     <= addr_c;
                req_wdata    <= wdata_c;
                wb_warp_id   <= sf_lsu_issue_warp_id;
                wb_rd        <= sf_lsu_issue_rd;
                wb_lane_mask <= sf_lsu_issue_lane_mask;
                wbd_warp_id  <= sf_lsu_issue_warp_id;
                wbd_rd       <= sf_lsu_issue_rd;
                stall_warp   <= sf_lsu_issue_warp_id;
            end

            // ---- 装载数据引导（§5.3）：响应全 8 lane 拷入 wb（与 C 模型一致，
            //      含 mask 外 lane；lane_mask 随路快照门控写由 rf 解释） ----
            if (rsp_fire && !req_rw)
                wb_wdata <= l1sm_lsu_rsp_rdata;

            // ---- 停顿上报（§5.4）：置位即登记，未被消费则锁存保持 ----
            if (lmiss_go) begin
                stall_sent <= 1'b1;
                if (!stall_fire) begin
                    stall_busy    <= 1'b1;
                    stall_pwarp   <= sf_lsu_issue_warp_id;
                    stall_preason <= R_LMISS;
                end
            end else if (clear_go) begin
                stall_sent <= 1'b0;
                if (!stall_fire) begin
                    stall_busy    <= 1'b1;
                    stall_pwarp   <= stall_warp;
                    stall_preason <= R_NONE;
                end
            end else if (stall_fire) begin
                stall_busy <= 1'b0;
            end
        end
    end

    // ---------------- 载荷导出（vld 保持期内稳定，§4） -----------------------
    assign lsu_l1sm_req_rw        = req_rw;
    assign lsu_l1sm_req_sm        = req_sm;
    assign lsu_l1sm_req_addr      = req_addr;
    assign lsu_l1sm_req_wdata     = req_wdata;
    assign lsu_l1sm_req_mask      = req_mask;
    assign lsu_rf_wb_warp_id      = wb_warp_id;
    assign lsu_rf_wb_rd           = wb_rd;
    assign lsu_rf_wb_lane_mask    = wb_lane_mask;
    assign lsu_rf_wb_wdata        = wb_wdata;
    assign lsu_sf_wbdone_warp_id  = wbd_warp_id;
    assign lsu_sf_wbdone_rd       = wbd_rd;

endmodule
