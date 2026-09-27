// =============================================================================
// easy_simt · easy_simt_top — 顶层互连（ma_spec §1.2/§1.5，intf_spec §1.7）
//
// 只做互连、时钟/复位分发与配置透传，自身无逻辑（intf_spec §1.3）。
// 例化 9 个功能模块（ws bs ialu falu lsu icache l1sm memif rf），按
// intf_spec §1.7 连接矩阵连线：
//   bs →ws（launch）；ws→bs（bdone）
//   ws ↔icache（取指）；ws ↔rf（读口）；ws→{ialu,falu,lsu}（issue）
//   ialu→ws（br 决议/wbdone）；falu/lsu→ws（wbdone）；lsu→ws（stall）
//   {ialu,falu,lsu}→rf（wb）；lsu↔l1sm（req/rsp）
//   icache↔memif、l1sm↔memif（回填/写通）；memif 对外 AXI4 全总线
//
// 配置（静态，复位释放前生效）：
//   top_cfg_n            —— 每块线程总数 N（bs_cfg_n）
//   top_cfg_param0/1/2   —— ld.param k=0/1/2 值（in_base/out_base/N）
//   top_cfg_brt_*        —— BRT 装载（isa_spec §1.11，8 表项 CAM）
//
// 顶层观测（intf_spec §1.6）：bs_top_done / ws_top_err / memif_top_err。
//
// 设计依据：top/docs/ma_spec_v0.1.md §1.5、top/docs/intf_spec_v0.1.md §1.7；
//   模块端口与各模块级规范一一对应。
// =============================================================================
`timescale 1ns/1ps

module easy_simt_top #(
    parameter DATA_W      = 32,        // 数据/地址/载荷位宽（intf_spec §1.4）
    parameter NWARPS      = 4,         // warp 数/块（ma_spec §1.2）
    parameter NLANES      = 8,         // lane 数/warp（ma_spec §1.2）
    parameter REG_AW      = 5,         // 寄存器地址位宽（intf_spec §1.4）
    parameter OPCODE_W    = 5,         // 操作码位宽（intf_spec §1.4）
    parameter BRT_IW      = 2,         // BRT 表项索引位宽（intf_spec §1.4）
    parameter ICACHE_LINES = 16,       // icache 行数（ma_spec §1.6）
    parameter ILINE_WORDS = 8,         // 每行字数 32B（ma_spec §1.6）
    parameter U_LINES     = 64,        // 统一 SRAM 总行数（ma_spec §1.6）
    parameter SM_LINES    = 4,         // SM 分区行数（ma_spec §1.6）
    parameter NBANKS      = 8,         // l1sm bank 数（ma_spec §1.6）
    parameter AXI_ID_W    = 4,         // AXI ID 位宽（intf_spec §1.4）
    parameter AXI_ADDR_W  = 32,        // AXI 地址位宽（intf_spec §1.4）
    parameter AXI_DATA_W  = 256        // AXI 数据位宽 = 1 行（intf_spec §1.4）
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- 配置（静态，复位释放前生效） ----
    input  wire [DATA_W-1:0]     top_cfg_n,        // 每块线程总数 N
    input  wire [DATA_W-1:0]     top_cfg_param0,   // ld.param k=0（in_base）
    input  wire [DATA_W-1:0]     top_cfg_param1,   // ld.param k=1（out_base）
    input  wire [DATA_W-1:0]     top_cfg_param2,   // ld.param k=2（N）
    input  wire                  top_cfg_brt_we,
    input  wire [BRT_IW:0]       top_cfg_brt_idx,  // 0..7
    input  wire [DATA_W-1:0]     top_cfg_brt_wpc,  // 分支 pc（键）
    input  wire [DATA_W-1:0]     top_cfg_brt_wrpc, // 重聚 pc（值）

    // ---- 对外 AXI4 主总线（memif，intf_spec §1.6/§9） ----
    output wire [AXI_ID_W-1:0]   axi_awid,
    output wire [AXI_ADDR_W-1:0] axi_awaddr,
    output wire [7:0]            axi_awlen,
    output wire [2:0]            axi_awsize,
    output wire [1:0]            axi_awburst,
    output wire                  axi_awvalid,
    input  wire                  axi_awready,
    output wire [AXI_DATA_W-1:0] axi_wdata,
    output wire [AXI_DATA_W/8-1:0] axi_wstrb,
    output wire                  axi_wlast,
    output wire                  axi_wvalid,
    input  wire                  axi_wready,
    input  wire [AXI_ID_W-1:0]   axi_bid,
    input  wire [1:0]            axi_bresp,
    input  wire                  axi_bvalid,
    output wire                  axi_bready,
    output wire [AXI_ID_W-1:0]   axi_arid,
    output wire [AXI_ADDR_W-1:0] axi_araddr,
    output wire [7:0]            axi_arlen,
    output wire [2:0]            axi_arsize,
    output wire [1:0]            axi_arburst,
    output wire                  axi_arvalid,
    input  wire                  axi_arready,
    input  wire [AXI_ID_W-1:0]   axi_rid,
    input  wire [AXI_DATA_W-1:0] axi_rdata,
    input  wire [1:0]            axi_rresp,
    input  wire                  axi_rlast,
    input  wire                  axi_rvalid,
    output wire                  axi_rready,

    // ---- 顶层观测（intf_spec §1.6） ----
    output wire                  bs_top_done,   // grid 结束（锁存）
    output wire                  ws_top_err,    // ws 错误标志（锁存）
    output wire                  memif_top_err  // AXI 非 OKAY（锁存）
);

    // ---------------- 派生位宽 ----------------
    localparam WARP_IW = (NWARPS > 1) ? $clog2(NWARPS) : 1;
    localparam VEC_W   = NLANES * DATA_W;
    localparam LINE_W  = ILINE_WORDS * DATA_W;

    // =====================================================================
    // 通道线网（intf_spec §1.7 连接矩阵，源_宿 命名）
    // =====================================================================

    // bs ↔ ws
    wire                  bs_ws_launch_vld;
    wire [DATA_W-1:0]     bs_ws_launch_block_idx, bs_ws_launch_n, bs_ws_launch_shbase;
    wire                  ws_bs_launch_rdy;
    wire                  ws_bs_bdone_vld;
    wire [DATA_W-1:0]     ws_bs_bdone_block_idx;
    wire                  bs_ws_bdone_rdy;

    // ws ↔ icache
    wire                  ws_icache_req_vld;
    wire [DATA_W-1:0]     ws_icache_req_pc;
    wire                  icache_ws_req_rdy;
    wire                  icache_ws_rsp_vld;
    wire [DATA_W-1:0]     icache_ws_rsp_inst;
    wire                  ws_icache_rsp_rdy;

    // ws ↔ rf
    wire                  ws_rf_rd_vld;
    wire [WARP_IW-1:0]    ws_rf_rd_warp_id;
    wire [REG_AW-1:0]     ws_rf_rd_rs1, ws_rf_rd_rs2;
    wire                  rf_ws_rd_rdy;
    wire                  rf_ws_rddata_vld;
    wire [VEC_W-1:0]      rf_ws_rddata_a, rf_ws_rddata_b;
    wire                  ws_rf_rddata_rdy;

    // ws → ialu
    wire                  ws_ialu_issue_vld;
    wire [OPCODE_W-1:0]   ws_ialu_issue_opcode;
    wire [REG_AW-1:0]     ws_ialu_issue_rd;
    wire [WARP_IW-1:0]    ws_ialu_issue_warp_id;
    wire [NLANES-1:0]     ws_ialu_issue_lane_mask;
    wire [DATA_W-1:0]     ws_ialu_issue_pc, ws_ialu_issue_imm;
    wire [VEC_W-1:0]      ws_ialu_issue_opa, ws_ialu_issue_opb, ws_ialu_issue_opc;
    wire                  ialu_ws_issue_rdy;

    // ws → falu
    wire                  ws_falu_issue_vld;
    wire [OPCODE_W-1:0]   ws_falu_issue_opcode;
    wire [REG_AW-1:0]     ws_falu_issue_rd;
    wire [WARP_IW-1:0]    ws_falu_issue_warp_id;
    wire [NLANES-1:0]     ws_falu_issue_lane_mask;
    wire [VEC_W-1:0]      ws_falu_issue_opa, ws_falu_issue_opb;
    wire                  falu_ws_issue_rdy;

    // ws → lsu
    wire                  ws_lsu_issue_vld;
    wire [OPCODE_W-1:0]   ws_lsu_issue_opcode;
    wire [REG_AW-1:0]     ws_lsu_issue_rd;
    wire [WARP_IW-1:0]    ws_lsu_issue_warp_id;
    wire [NLANES-1:0]     ws_lsu_issue_lane_mask;
    wire [VEC_W-1:0]      ws_lsu_issue_opa, ws_lsu_issue_opb;
    wire [DATA_W-1:0]     ws_lsu_issue_imm, ws_lsu_issue_shbase;
    wire                  lsu_ws_issue_rdy;

    // ialu → ws（分支决议）
    wire                  ialu_ws_br_vld;
    wire [WARP_IW-1:0]    ialu_ws_br_warp_id;
    wire [NLANES-1:0]     ialu_ws_br_taken;
    wire [DATA_W-1:0]     ialu_ws_br_target;
    wire [BRT_IW-1:0]     ialu_ws_br_brt_idx;
    wire                  ws_ialu_br_rdy;

    // lsu → ws（停顿上报）
    wire                  lsu_ws_stall_vld;
    wire [WARP_IW-1:0]    lsu_ws_stall_warp_id;
    wire [2:0]            lsu_ws_stall_reason;
    wire                  ws_lsu_stall_rdy;

    // {ialu,falu,lsu} → rf（写回）
    wire                  ialu_rf_wb_vld, falu_rf_wb_vld, lsu_rf_wb_vld;
    wire [WARP_IW-1:0]    ialu_rf_wb_warp_id, falu_rf_wb_warp_id, lsu_rf_wb_warp_id;
    wire [REG_AW-1:0]     ialu_rf_wb_rd, falu_rf_wb_rd, lsu_rf_wb_rd;
    wire [NLANES-1:0]     ialu_rf_wb_lane_mask, falu_rf_wb_lane_mask, lsu_rf_wb_lane_mask;
    wire [VEC_W-1:0]      ialu_rf_wb_wdata, falu_rf_wb_wdata, lsu_rf_wb_wdata;
    wire                  rf_ialu_wb_rdy, rf_falu_wb_rdy, rf_lsu_wb_rdy;

    // {ialu,falu,lsu} → ws（写回完成）
    wire                  ialu_ws_wbdone_vld, falu_ws_wbdone_vld, lsu_ws_wbdone_vld;
    wire [WARP_IW-1:0]    ialu_ws_wbdone_warp_id, falu_ws_wbdone_warp_id, lsu_ws_wbdone_warp_id;
    wire [REG_AW-1:0]     ialu_ws_wbdone_rd, falu_ws_wbdone_rd, lsu_ws_wbdone_rd;
    wire                  ws_ialu_wbdone_rdy, ws_falu_wbdone_rdy, ws_lsu_wbdone_rdy;

    // lsu ↔ l1sm
    wire                  lsu_l1sm_req_vld;
    wire                  lsu_l1sm_req_rw, lsu_l1sm_req_sm;
    wire [VEC_W-1:0]      lsu_l1sm_req_addr, lsu_l1sm_req_wdata;
    wire [NLANES-1:0]     lsu_l1sm_req_mask;
    wire                  l1sm_lsu_req_rdy;
    wire                  l1sm_lsu_rsp_vld;
    wire [VEC_W-1:0]      l1sm_lsu_rsp_rdata;
    wire                  lsu_l1sm_rsp_rdy;

    // l1sm ↔ memif
    wire                  l1sm_memif_req_vld;
    wire                  l1sm_memif_req_rw;
    wire [DATA_W-1:0]     l1sm_memif_req_addr, l1sm_memif_req_wdata;
    wire                  memif_l1sm_req_rdy;
    wire                  memif_l1sm_rsp_vld;
    wire [LINE_W-1:0]     memif_l1sm_rsp_data;
    wire                  l1sm_memif_rsp_rdy;

    // icache ↔ memif
    wire                  icache_memif_req_vld;
    wire [DATA_W-1:0]     icache_memif_req_addr;
    wire                  memif_icache_req_rdy;
    wire                  memif_icache_rsp_vld;
    wire [LINE_W-1:0]     memif_icache_rsp_data;
    wire                  icache_memif_rsp_rdy;

    // =====================================================================
    // 模块例化（9 个，ma_spec §1.4 模块清单）
    // =====================================================================

    // ---- bs — Block Scheduler（ma_spec §3） ----
    bs #(
        .DATA_W  (DATA_W),
        .NWARPS  (NWARPS),
        .NLANES  (NLANES)
    ) u_bs (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .bs_cfg_n               (top_cfg_n),
        .bs_ws_launch_vld       (bs_ws_launch_vld),
        .bs_ws_launch_block_idx (bs_ws_launch_block_idx),
        .bs_ws_launch_n         (bs_ws_launch_n),
        .bs_ws_launch_shbase    (bs_ws_launch_shbase),
        .ws_bs_launch_rdy       (ws_bs_launch_rdy),
        .ws_bs_bdone_vld        (ws_bs_bdone_vld),
        .ws_bs_bdone_block_idx  (ws_bs_bdone_block_idx),
        .bs_ws_bdone_rdy        (bs_ws_bdone_rdy),
        .bs_top_done            (bs_top_done)
    );

    // ---- ws — Warp Scheduler（SIMT 前端与调度合一，ma_spec §2） ----
    ws #(
        .DATA_W   (DATA_W),
        .NWARPS   (NWARPS),
        .NLANES   (NLANES),
        .REG_AW   (REG_AW),
        .OPCODE_W (OPCODE_W),
        .BRT_IW   (BRT_IW)
    ) u_ws (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .ws_cfg_param0          (top_cfg_param0),
        .ws_cfg_param1          (top_cfg_param1),
        .ws_cfg_param2          (top_cfg_param2),
        .ws_cfg_brt_we          (top_cfg_brt_we),
        .ws_cfg_brt_idx         (top_cfg_brt_idx),
        .ws_cfg_brt_wpc         (top_cfg_brt_wpc),
        .ws_cfg_brt_wrpc        (top_cfg_brt_wrpc),
        .bs_ws_launch_vld       (bs_ws_launch_vld),
        .bs_ws_launch_block_idx (bs_ws_launch_block_idx),
        .bs_ws_launch_n         (bs_ws_launch_n),
        .bs_ws_launch_shbase    (bs_ws_launch_shbase),
        .ws_bs_launch_rdy       (ws_bs_launch_rdy),
        .ws_bs_bdone_vld        (ws_bs_bdone_vld),
        .ws_bs_bdone_block_idx  (ws_bs_bdone_block_idx),
        .bs_ws_bdone_rdy        (bs_ws_bdone_rdy),
        .lsu_ws_stall_vld       (lsu_ws_stall_vld),
        .lsu_ws_stall_warp_id   (lsu_ws_stall_warp_id),
        .lsu_ws_stall_reason    (lsu_ws_stall_reason),
        .ws_lsu_stall_rdy       (ws_lsu_stall_rdy),
        .ws_icache_req_vld      (ws_icache_req_vld),
        .ws_icache_req_pc       (ws_icache_req_pc),
        .icache_ws_req_rdy      (icache_ws_req_rdy),
        .icache_ws_rsp_vld      (icache_ws_rsp_vld),
        .icache_ws_rsp_inst     (icache_ws_rsp_inst),
        .ws_icache_rsp_rdy      (ws_icache_rsp_rdy),
        .ws_rf_rd_vld           (ws_rf_rd_vld),
        .ws_rf_rd_warp_id       (ws_rf_rd_warp_id),
        .ws_rf_rd_rs1           (ws_rf_rd_rs1),
        .ws_rf_rd_rs2           (ws_rf_rd_rs2),
        .rf_ws_rd_rdy           (rf_ws_rd_rdy),
        .rf_ws_rddata_vld       (rf_ws_rddata_vld),
        .rf_ws_rddata_a         (rf_ws_rddata_a),
        .rf_ws_rddata_b         (rf_ws_rddata_b),
        .ws_rf_rddata_rdy       (ws_rf_rddata_rdy),
        .ws_ialu_issue_vld      (ws_ialu_issue_vld),
        .ws_ialu_issue_opcode   (ws_ialu_issue_opcode),
        .ws_ialu_issue_rd       (ws_ialu_issue_rd),
        .ws_ialu_issue_warp_id  (ws_ialu_issue_warp_id),
        .ws_ialu_issue_lane_mask(ws_ialu_issue_lane_mask),
        .ws_ialu_issue_pc       (ws_ialu_issue_pc),
        .ws_ialu_issue_imm      (ws_ialu_issue_imm),
        .ws_ialu_issue_opa      (ws_ialu_issue_opa),
        .ws_ialu_issue_opb      (ws_ialu_issue_opb),
        .ws_ialu_issue_opc      (ws_ialu_issue_opc),
        .ialu_ws_issue_rdy      (ialu_ws_issue_rdy),
        .ws_falu_issue_vld      (ws_falu_issue_vld),
        .ws_falu_issue_opcode   (ws_falu_issue_opcode),
        .ws_falu_issue_rd       (ws_falu_issue_rd),
        .ws_falu_issue_warp_id  (ws_falu_issue_warp_id),
        .ws_falu_issue_lane_mask(ws_falu_issue_lane_mask),
        .ws_falu_issue_opa      (ws_falu_issue_opa),
        .ws_falu_issue_opb      (ws_falu_issue_opb),
        .falu_ws_issue_rdy      (falu_ws_issue_rdy),
        .ws_lsu_issue_vld       (ws_lsu_issue_vld),
        .ws_lsu_issue_opcode    (ws_lsu_issue_opcode),
        .ws_lsu_issue_rd        (ws_lsu_issue_rd),
        .ws_lsu_issue_warp_id   (ws_lsu_issue_warp_id),
        .ws_lsu_issue_lane_mask (ws_lsu_issue_lane_mask),
        .ws_lsu_issue_opa       (ws_lsu_issue_opa),
        .ws_lsu_issue_opb       (ws_lsu_issue_opb),
        .ws_lsu_issue_imm       (ws_lsu_issue_imm),
        .ws_lsu_issue_shbase    (ws_lsu_issue_shbase),
        .lsu_ws_issue_rdy       (lsu_ws_issue_rdy),
        .ialu_ws_br_vld         (ialu_ws_br_vld),
        .ialu_ws_br_warp_id     (ialu_ws_br_warp_id),
        .ialu_ws_br_taken       (ialu_ws_br_taken),
        .ialu_ws_br_target      (ialu_ws_br_target),
        .ialu_ws_br_brt_idx     (ialu_ws_br_brt_idx),
        .ws_ialu_br_rdy         (ws_ialu_br_rdy),
        .ialu_ws_wbdone_vld     (ialu_ws_wbdone_vld),
        .ialu_ws_wbdone_warp_id (ialu_ws_wbdone_warp_id),
        .ialu_ws_wbdone_rd      (ialu_ws_wbdone_rd),
        .ws_ialu_wbdone_rdy     (ws_ialu_wbdone_rdy),
        .falu_ws_wbdone_vld     (falu_ws_wbdone_vld),
        .falu_ws_wbdone_warp_id (falu_ws_wbdone_warp_id),
        .falu_ws_wbdone_rd      (falu_ws_wbdone_rd),
        .ws_falu_wbdone_rdy     (ws_falu_wbdone_rdy),
        .lsu_ws_wbdone_vld      (lsu_ws_wbdone_vld),
        .lsu_ws_wbdone_warp_id  (lsu_ws_wbdone_warp_id),
        .lsu_ws_wbdone_rd       (lsu_ws_wbdone_rd),
        .ws_lsu_wbdone_rdy      (ws_lsu_wbdone_rdy),
        .ws_top_err             (ws_top_err)
    );

    // ---- ialu — Integer ALU（ma_spec §4） ----
    ialu #(
        .DATA_W   (DATA_W),
        .NWARPS   (NWARPS),
        .NLANES   (NLANES),
        .REG_AW   (REG_AW),
        .OPCODE_W (OPCODE_W),
        .BRT_IW   (BRT_IW)
    ) u_ialu (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .ws_ialu_issue_vld      (ws_ialu_issue_vld),
        .ws_ialu_issue_opcode   (ws_ialu_issue_opcode),
        .ws_ialu_issue_rd       (ws_ialu_issue_rd),
        .ws_ialu_issue_warp_id  (ws_ialu_issue_warp_id),
        .ws_ialu_issue_lane_mask(ws_ialu_issue_lane_mask),
        .ws_ialu_issue_pc       (ws_ialu_issue_pc),
        .ws_ialu_issue_imm      (ws_ialu_issue_imm),
        .ws_ialu_issue_opa      (ws_ialu_issue_opa),
        .ws_ialu_issue_opb      (ws_ialu_issue_opb),
        .ws_ialu_issue_opc      (ws_ialu_issue_opc),
        .ialu_ws_issue_rdy      (ialu_ws_issue_rdy),
        .ialu_ws_br_vld         (ialu_ws_br_vld),
        .ialu_ws_br_warp_id     (ialu_ws_br_warp_id),
        .ialu_ws_br_taken       (ialu_ws_br_taken),
        .ialu_ws_br_target      (ialu_ws_br_target),
        .ialu_ws_br_brt_idx     (ialu_ws_br_brt_idx),
        .ws_ialu_br_rdy         (ws_ialu_br_rdy),
        .ialu_rf_wb_vld         (ialu_rf_wb_vld),
        .ialu_rf_wb_warp_id     (ialu_rf_wb_warp_id),
        .ialu_rf_wb_rd          (ialu_rf_wb_rd),
        .ialu_rf_wb_lane_mask   (ialu_rf_wb_lane_mask),
        .ialu_rf_wb_wdata       (ialu_rf_wb_wdata),
        .rf_ialu_wb_rdy         (rf_ialu_wb_rdy),
        .ialu_ws_wbdone_vld     (ialu_ws_wbdone_vld),
        .ialu_ws_wbdone_warp_id (ialu_ws_wbdone_warp_id),
        .ialu_ws_wbdone_rd      (ialu_ws_wbdone_rd),
        .ws_ialu_wbdone_rdy     (ws_ialu_wbdone_rdy)
    );

    // ---- falu — Floating-point ALU（ma_spec §5） ----
    falu #(
        .DATA_W   (DATA_W),
        .NWARPS   (NWARPS),
        .NLANES   (NLANES),
        .REG_AW   (REG_AW),
        .OPCODE_W (OPCODE_W)
    ) u_falu (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .ws_falu_issue_vld      (ws_falu_issue_vld),
        .ws_falu_issue_opcode   (ws_falu_issue_opcode),
        .ws_falu_issue_rd       (ws_falu_issue_rd),
        .ws_falu_issue_warp_id  (ws_falu_issue_warp_id),
        .ws_falu_issue_lane_mask(ws_falu_issue_lane_mask),
        .ws_falu_issue_opa      (ws_falu_issue_opa),
        .ws_falu_issue_opb      (ws_falu_issue_opb),
        .falu_ws_issue_rdy      (falu_ws_issue_rdy),
        .falu_rf_wb_vld         (falu_rf_wb_vld),
        .falu_rf_wb_warp_id     (falu_rf_wb_warp_id),
        .falu_rf_wb_rd          (falu_rf_wb_rd),
        .falu_rf_wb_lane_mask   (falu_rf_wb_lane_mask),
        .falu_rf_wb_wdata       (falu_rf_wb_wdata),
        .rf_falu_wb_rdy         (rf_falu_wb_rdy),
        .falu_ws_wbdone_vld     (falu_ws_wbdone_vld),
        .falu_ws_wbdone_warp_id (falu_ws_wbdone_warp_id),
        .falu_ws_wbdone_rd      (falu_ws_wbdone_rd),
        .ws_falu_wbdone_rdy     (ws_falu_wbdone_rdy)
    );

    // ---- lsu — Load/Store Unit（ma_spec §6） ----
    lsu #(
        .DATA_W   (DATA_W),
        .NWARPS   (NWARPS),
        .NLANES   (NLANES),
        .REG_AW   (REG_AW),
        .OPCODE_W (OPCODE_W)
    ) u_lsu (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .ws_lsu_issue_vld       (ws_lsu_issue_vld),
        .ws_lsu_issue_opcode    (ws_lsu_issue_opcode),
        .ws_lsu_issue_rd        (ws_lsu_issue_rd),
        .ws_lsu_issue_warp_id   (ws_lsu_issue_warp_id),
        .ws_lsu_issue_lane_mask (ws_lsu_issue_lane_mask),
        .ws_lsu_issue_opa       (ws_lsu_issue_opa),
        .ws_lsu_issue_opb       (ws_lsu_issue_opb),
        .ws_lsu_issue_imm       (ws_lsu_issue_imm),
        .ws_lsu_issue_shbase    (ws_lsu_issue_shbase),
        .lsu_ws_issue_rdy       (lsu_ws_issue_rdy),
        .lsu_l1sm_req_vld       (lsu_l1sm_req_vld),
        .lsu_l1sm_req_rw        (lsu_l1sm_req_rw),
        .lsu_l1sm_req_sm        (lsu_l1sm_req_sm),
        .lsu_l1sm_req_addr      (lsu_l1sm_req_addr),
        .lsu_l1sm_req_wdata     (lsu_l1sm_req_wdata),
        .lsu_l1sm_req_mask      (lsu_l1sm_req_mask),
        .l1sm_lsu_req_rdy       (l1sm_lsu_req_rdy),
        .l1sm_lsu_rsp_vld       (l1sm_lsu_rsp_vld),
        .l1sm_lsu_rsp_rdata     (l1sm_lsu_rsp_rdata),
        .lsu_l1sm_rsp_rdy       (lsu_l1sm_rsp_rdy),
        .lsu_ws_stall_vld       (lsu_ws_stall_vld),
        .lsu_ws_stall_warp_id   (lsu_ws_stall_warp_id),
        .lsu_ws_stall_reason    (lsu_ws_stall_reason),
        .ws_lsu_stall_rdy       (ws_lsu_stall_rdy),
        .lsu_rf_wb_vld          (lsu_rf_wb_vld),
        .lsu_rf_wb_warp_id      (lsu_rf_wb_warp_id),
        .lsu_rf_wb_rd           (lsu_rf_wb_rd),
        .lsu_rf_wb_lane_mask    (lsu_rf_wb_lane_mask),
        .lsu_rf_wb_wdata        (lsu_rf_wb_wdata),
        .rf_lsu_wb_rdy          (rf_lsu_wb_rdy),
        .lsu_ws_wbdone_vld      (lsu_ws_wbdone_vld),
        .lsu_ws_wbdone_warp_id  (lsu_ws_wbdone_warp_id),
        .lsu_ws_wbdone_rd       (lsu_ws_wbdone_rd),
        .ws_lsu_wbdone_rdy      (ws_lsu_wbdone_rdy)
    );

    // ---- icache — Instruction Cache（ma_spec §7） ----
    icache #(
        .ICACHE_LINES (ICACHE_LINES),
        .ILINE_WORDS  (ILINE_WORDS)
    ) u_icache (
        .clk                  (clk),
        .rst_n                (rst_n),
        .ws_icache_req_vld    (ws_icache_req_vld),
        .ws_icache_req_pc     (ws_icache_req_pc),
        .icache_ws_req_rdy    (icache_ws_req_rdy),
        .icache_ws_rsp_vld    (icache_ws_rsp_vld),
        .icache_ws_rsp_inst   (icache_ws_rsp_inst),
        .ws_icache_rsp_rdy    (ws_icache_rsp_rdy),
        .icache_memif_req_vld (icache_memif_req_vld),
        .icache_memif_req_addr(icache_memif_req_addr),
        .memif_icache_req_rdy (memif_icache_req_rdy),
        .memif_icache_rsp_vld (memif_icache_rsp_vld),
        .memif_icache_rsp_data(memif_icache_rsp_data),
        .icache_memif_rsp_rdy (icache_memif_rsp_rdy)
    );

    // ---- l1sm — L1 + Shared Memory（统一 SRAM，ma_spec §8） ----
    l1sm #(
        .DATA_W      (DATA_W),
        .NWARPS      (NWARPS),
        .NLANES      (NLANES),
        .U_LINES     (U_LINES),
        .SM_LINES    (SM_LINES),
        .NBANKS      (NBANKS),
        .ILINE_WORDS (ILINE_WORDS)
    ) u_l1sm (
        .clk                 (clk),
        .rst_n               (rst_n),
        .lsu_l1sm_req_vld    (lsu_l1sm_req_vld),
        .lsu_l1sm_req_rw     (lsu_l1sm_req_rw),
        .lsu_l1sm_req_sm     (lsu_l1sm_req_sm),
        .lsu_l1sm_req_addr   (lsu_l1sm_req_addr),
        .lsu_l1sm_req_wdata  (lsu_l1sm_req_wdata),
        .lsu_l1sm_req_mask   (lsu_l1sm_req_mask),
        .l1sm_lsu_req_rdy    (l1sm_lsu_req_rdy),
        .l1sm_lsu_rsp_vld    (l1sm_lsu_rsp_vld),
        .l1sm_lsu_rsp_rdata  (l1sm_lsu_rsp_rdata),
        .lsu_l1sm_rsp_rdy    (lsu_l1sm_rsp_rdy),
        .l1sm_memif_req_vld  (l1sm_memif_req_vld),
        .l1sm_memif_req_rw   (l1sm_memif_req_rw),
        .l1sm_memif_req_addr (l1sm_memif_req_addr),
        .l1sm_memif_req_wdata(l1sm_memif_req_wdata),
        .memif_l1sm_req_rdy  (memif_l1sm_req_rdy),
        .memif_l1sm_rsp_vld  (memif_l1sm_rsp_vld),
        .memif_l1sm_rsp_data (memif_l1sm_rsp_data),
        .l1sm_memif_rsp_rdy  (l1sm_memif_rsp_rdy)
    );

    // ---- memif — Memory Interface（ma_spec §9） ----
    memif #(
        .AXI_ID_W    (AXI_ID_W),
        .AXI_ADDR_W  (AXI_ADDR_W),
        .AXI_DATA_W  (AXI_DATA_W),
        .AXI_STRB_W  (AXI_DATA_W / 8),
        .ILINE_WORDS (ILINE_WORDS)
    ) u_memif (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .icache_memif_req_vld  (icache_memif_req_vld),
        .icache_memif_req_addr (icache_memif_req_addr),
        .memif_icache_req_rdy  (memif_icache_req_rdy),
        .memif_icache_rsp_vld  (memif_icache_rsp_vld),
        .memif_icache_rsp_data (memif_icache_rsp_data),
        .icache_memif_rsp_rdy  (icache_memif_rsp_rdy),
        .l1sm_memif_req_vld    (l1sm_memif_req_vld),
        .l1sm_memif_req_rw     (l1sm_memif_req_rw),
        .l1sm_memif_req_addr   (l1sm_memif_req_addr),
        .l1sm_memif_req_wdata  (l1sm_memif_req_wdata),
        .memif_l1sm_req_rdy    (memif_l1sm_req_rdy),
        .memif_l1sm_rsp_vld    (memif_l1sm_rsp_vld),
        .memif_l1sm_rsp_data   (memif_l1sm_rsp_data),
        .l1sm_memif_rsp_rdy    (l1sm_memif_rsp_rdy),
        .axi_awid              (axi_awid),
        .axi_awaddr            (axi_awaddr),
        .axi_awlen             (axi_awlen),
        .axi_awsize            (axi_awsize),
        .axi_awburst           (axi_awburst),
        .axi_awvalid           (axi_awvalid),
        .axi_awready           (axi_awready),
        .axi_wdata             (axi_wdata),
        .axi_wstrb             (axi_wstrb),
        .axi_wlast             (axi_wlast),
        .axi_wvalid            (axi_wvalid),
        .axi_wready            (axi_wready),
        .axi_bid               (axi_bid),
        .axi_bresp             (axi_bresp),
        .axi_bvalid            (axi_bvalid),
        .axi_bready            (axi_bready),
        .axi_arid              (axi_arid),
        .axi_araddr            (axi_araddr),
        .axi_arlen             (axi_arlen),
        .axi_arsize            (axi_arsize),
        .axi_arburst           (axi_arburst),
        .axi_arvalid           (axi_arvalid),
        .axi_arready           (axi_arready),
        .axi_rid               (axi_rid),
        .axi_rdata             (axi_rdata),
        .axi_rresp             (axi_rresp),
        .axi_rlast             (axi_rlast),
        .axi_rvalid            (axi_rvalid),
        .axi_rready            (axi_rready),
        .memif_top_err         (memif_top_err)
    );

    // ---- rf — Register File（ma_spec §10，SRAM 宏实现见 rf_spec §5.1） ----
    rf #(
        .DATA_W (DATA_W),
        .NWARPS (NWARPS),
        .NLANES (NLANES),
        .REG_AW (REG_AW)
    ) u_rf (
        .clk               (clk),
        .rst_n             (rst_n),
        .ws_rf_rd_vld      (ws_rf_rd_vld),
        .ws_rf_rd_warp_id  (ws_rf_rd_warp_id),
        .ws_rf_rd_rs1      (ws_rf_rd_rs1),
        .ws_rf_rd_rs2      (ws_rf_rd_rs2),
        .rf_ws_rd_rdy      (rf_ws_rd_rdy),
        .rf_ws_rddata_vld  (rf_ws_rddata_vld),
        .rf_ws_rddata_a    (rf_ws_rddata_a),
        .rf_ws_rddata_b    (rf_ws_rddata_b),
        .ws_rf_rddata_rdy  (ws_rf_rddata_rdy),
        .ialu_rf_wb_vld    (ialu_rf_wb_vld),
        .ialu_rf_wb_warp_id(ialu_rf_wb_warp_id),
        .ialu_rf_wb_rd     (ialu_rf_wb_rd),
        .ialu_rf_wb_lane_mask(ialu_rf_wb_lane_mask),
        .ialu_rf_wb_wdata  (ialu_rf_wb_wdata),
        .rf_ialu_wb_rdy    (rf_ialu_wb_rdy),
        .falu_rf_wb_vld    (falu_rf_wb_vld),
        .falu_rf_wb_warp_id(falu_rf_wb_warp_id),
        .falu_rf_wb_rd     (falu_rf_wb_rd),
        .falu_rf_wb_lane_mask(falu_rf_wb_lane_mask),
        .falu_rf_wb_wdata  (falu_rf_wb_wdata),
        .rf_falu_wb_rdy    (rf_falu_wb_rdy),
        .lsu_rf_wb_vld     (lsu_rf_wb_vld),
        .lsu_rf_wb_warp_id (lsu_rf_wb_warp_id),
        .lsu_rf_wb_rd      (lsu_rf_wb_rd),
        .lsu_rf_wb_lane_mask(lsu_rf_wb_lane_mask),
        .lsu_rf_wb_wdata   (lsu_rf_wb_wdata),
        .rf_lsu_wb_rdy     (rf_lsu_wb_rdy)
    );

endmodule
