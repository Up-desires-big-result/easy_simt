// =============================================================================
// easy_simt · rf — Register File（ma_spec §11）
//
// 8 lane × 32 寄存器 × 32b，每 lane 独立副本；R0 恒零（写忽略、读恒 0）。
// 读：译码双读口（rs1/rs2），应答与请求严格顺序对应（单在途，不回带地址）；
// 写：单写口三源固定优先级仲裁 lsu > ialu > falu，lane 掩码随路。
//
// 设计依据：rf/docs/rf_spec_v0.1.md；
//   端口命名与 intf_spec §11 一致，握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module rf #(
    parameter DATA_W = 32,              // 数据位宽（intf_spec §1.4）
    parameter NWARPS = 4,               // warp 数/块（ma_spec §1.2）
    parameter NLANES = 8,               // lane 数/warp（ma_spec §1.2）
    parameter REG_AW = 5,               // 寄存器地址位宽（intf_spec §1.4）
    localparam WARP_IW = (NWARPS > 1) ? $clog2(NWARPS) : 1,
    localparam VEC_W   = NLANES * DATA_W,
    localparam DEPTH   = NWARPS * (1 << REG_AW),   // 基线 128 字（rf_spec §3）
    localparam IDX_W   = WARP_IW + REG_AW
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // sf_rf_rd：译码双读口请求（intf_spec §11）
    input  wire                 sf_rf_rd_vld,
    input  wire [WARP_IW-1:0]   sf_rf_rd_warp_id,
    input  wire [REG_AW-1:0]    sf_rf_rd_rs1,
    input  wire [REG_AW-1:0]    sf_rf_rd_rs2,
    output wire                 rf_sf_rd_rdy,

    // rf_sf_rddata：读应答（单在途、与请求严格顺序对应）
    output wire                 rf_sf_rddata_vld,
    output wire [VEC_W-1:0]     rf_sf_rddata_a,
    output wire [VEC_W-1:0]     rf_sf_rddata_b,
    input  wire                 sf_rf_rddata_rdy,

    // ialu_rf_wb：写口（三源之一）
    input  wire                 ialu_rf_wb_vld,
    input  wire [WARP_IW-1:0]   ialu_rf_wb_warp_id,
    input  wire [REG_AW-1:0]    ialu_rf_wb_rd,
    input  wire [NLANES-1:0]    ialu_rf_wb_lane_mask,
    input  wire [VEC_W-1:0]     ialu_rf_wb_wdata,
    output wire                 rf_ialu_wb_rdy,

    // falu_rf_wb：写口（三源之一）
    input  wire                 falu_rf_wb_vld,
    input  wire [WARP_IW-1:0]   falu_rf_wb_warp_id,
    input  wire [REG_AW-1:0]    falu_rf_wb_rd,
    input  wire [NLANES-1:0]    falu_rf_wb_lane_mask,
    input  wire [VEC_W-1:0]     falu_rf_wb_wdata,
    output wire                 rf_falu_wb_rdy,

    // lsu_rf_wb：写口（三源之一，固定优先级最高）
    input  wire                 lsu_rf_wb_vld,
    input  wire [WARP_IW-1:0]   lsu_rf_wb_warp_id,
    input  wire [REG_AW-1:0]    lsu_rf_wb_rd,
    input  wire [NLANES-1:0]    lsu_rf_wb_lane_mask,
    input  wire [VEC_W-1:0]     lsu_rf_wb_wdata,
    output wire                 rf_lsu_wb_rdy
);

    // ---------------- 存储阵列（rf_spec §5.1） ----------------
    // 索引 {warp_id, reg}，每字 NLANES 个 DATA_W lane 字段；复位全 0（§4）
    reg [VEC_W-1:0] mem [0:DEPTH-1];

    // ---------------- 在途读应答（rf_spec §4/§6） ----------------
    reg              rsp_vld;
    reg  [VEC_W-1:0] rsp_a, rsp_b;

    // ---------------- 写口三源固定优先级仲裁（rf_spec §5.3） ----------------
    wire lsu_g  = lsu_rf_wb_vld;
    wire ialu_g = ialu_rf_wb_vld && !lsu_rf_wb_vld;
    wire falu_g = falu_rf_wb_vld && !lsu_rf_wb_vld && !ialu_rf_wb_vld;
    wire wb_fire = lsu_g | ialu_g | falu_g;

    assign rf_lsu_wb_rdy  = 1'b1;
    assign rf_ialu_wb_rdy = !lsu_rf_wb_vld;
    assign rf_falu_wb_rdy = !lsu_rf_wb_vld && !ialu_rf_wb_vld;

    // 授予源载荷选择
    wire [WARP_IW-1:0] wb_warp_id   = lsu_g ? lsu_rf_wb_warp_id
                                    : (ialu_g ? ialu_rf_wb_warp_id
                                              : falu_rf_wb_warp_id);
    wire [REG_AW-1:0]  wb_rd        = lsu_g ? lsu_rf_wb_rd
                                    : (ialu_g ? ialu_rf_wb_rd
                                              : falu_rf_wb_rd);
    wire [NLANES-1:0]  wb_lane_mask = lsu_g ? lsu_rf_wb_lane_mask
                                    : (ialu_g ? ialu_rf_wb_lane_mask
                                              : falu_rf_wb_lane_mask);
    wire [VEC_W-1:0]   wb_wdata     = lsu_g ? lsu_rf_wb_wdata
                                    : (ialu_g ? ialu_rf_wb_wdata
                                              : falu_rf_wb_wdata);

    // ---------------- 读路径（rf_spec §5.2） ----------------
    // 单在途：无在途应答时可接受请求；在途应答本拍被消费时同拍接受下一请求
    wire rsp_fire = rsp_vld && sf_rf_rddata_rdy;
    assign rf_sf_rddata_vld = rsp_vld;
    assign rf_sf_rd_rdy     = !rsp_vld || sf_rf_rddata_rdy;
    wire rd_fire = sf_rf_rd_vld && rf_sf_rd_rdy;

    wire [IDX_W-1:0] ridx_a = {sf_rf_rd_warp_id, sf_rf_rd_rs1};
    wire [IDX_W-1:0] ridx_b = {sf_rf_rd_warp_id, sf_rf_rd_rs2};

    wire [VEC_W-1:0] base_a = (sf_rf_rd_rs1 != {REG_AW{1'b0}}) ? mem[ridx_a]
                                                               : {VEC_W{1'b0}};
    wire [VEC_W-1:0] base_b = (sf_rf_rd_rs2 != {REG_AW{1'b0}}) ? mem[ridx_b]
                                                               : {VEC_W{1'b0}};

    // 同拍写旁路：写握手与读请求握手同拍且地址相同时取写后值
    //（与 rf_step 写先读后一致，rf_spec §5.2）
    wire wb_en    = wb_fire && (wb_rd != {REG_AW{1'b0}});
    wire [IDX_W-1:0] wb_idx = {wb_warp_id, wb_rd};
    wire match_a  = wb_en && (wb_idx == ridx_a);
    wire match_b  = wb_en && (wb_idx == ridx_b);

    wire [VEC_W-1:0] rd_a_c;
    wire [VEC_W-1:0] rd_b_c;

    genvar gl;
    generate
        for (gl = 0; gl < NLANES; gl = gl + 1) begin : g_lane
            assign rd_a_c[gl*DATA_W +: DATA_W] =
                (match_a && wb_lane_mask[gl]) ? wb_wdata[gl*DATA_W +: DATA_W]
                                              : base_a[gl*DATA_W +: DATA_W];
            assign rd_b_c[gl*DATA_W +: DATA_W] =
                (match_b && wb_lane_mask[gl]) ? wb_wdata[gl*DATA_W +: DATA_W]
                                              : base_b[gl*DATA_W +: DATA_W];
        end
    endgenerate

    // ---------------- 时序：阵列写入与应答寄存器（rf_spec §5.3/§6） ----------------
    integer i, l;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rsp_vld <= 1'b0;
            rsp_a   <= {VEC_W{1'b0}};
            rsp_b   <= {VEC_W{1'b0}};
            for (i = 0; i < DEPTH; i = i + 1)
                mem[i] <= {VEC_W{1'b0}};
        end else begin
            // 写：握手且 rd != 0，按 lane_mask 逐 lane 更新（R0 写忽略）
            if (wb_fire && (wb_rd != {REG_AW{1'b0}})) begin
                for (l = 0; l < NLANES; l = l + 1)
                    if (wb_lane_mask[l])
                        mem[wb_idx][l*DATA_W +: DATA_W]
                            <= wb_wdata[l*DATA_W +: DATA_W];
            end
            // 读应答：新请求入级优先于在途应答清空（零间隙周转，§6）
            if (rd_fire) begin
                rsp_vld <= 1'b1;
                rsp_a   <= rd_a_c;
                rsp_b   <= rd_b_c;
            end else if (rsp_fire) begin
                rsp_vld <= 1'b0;
            end
        end
    end

    // ---------------- 应答载荷导出（vld 保持期内稳定，§4） ----------------
    assign rf_sf_rddata_a = rsp_a;
    assign rf_sf_rddata_b = rsp_b;

endmodule
