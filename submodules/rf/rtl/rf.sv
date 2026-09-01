// =============================================================================
// easy_simt · rf — Register File（ma_spec §11）· SRAM 宏实验版
//
// 存储阵列改用 OpenRAM 生成的 128 字×32 位双 RW 口宏（freepdk45）：
//   宏深度 128 = 阵列全深度，地址即 {warp_id, reg}（7 位），无 bank 译码；
//   宏宽度 32 位 = 一个 lane。每 lane 一对互为镜像的宏：
//     M0[l] 供 rs1 读、M1[l] 供 rs2 读、写广播到两口（共 16 宏），
//   保证任意拍「2 读 + 1 写」无端口冲突，读/写握手时序与寄存器版逐拍一致
//   （锁步比对要求的零停顿）。
//
// 与寄存器版的实现差异（外部协议不变，端口仍按 intf_spec §11）：
//   1) 读数据路径为混合时序：应答首拍直通宏读出（宏地址于请求握手拍末沿
//      锁存，次拍输出有效），背压自次拍起改由应答寄存器保持（held）；
//      寄存器版为握手拍组合采样、当拍末沿整体入级。
//   2) 同拍写旁路（§5.2）的写上下文随应答锁存，于应答首拍对宏读出值做
//      逐 lane 合并，语义与寄存器版一致。
//   3) 复位全 0 改为复位期间宏清零扫描：两口并行（口 0 扫低 64 字、口 1 扫
//      高 64 字），64 拍扫全阵；要求复位保持不少于 64 拍（testbench 70 拍）。
//   4) 仅支持基线几何（128 字 × 256 位、8 lane），宏为固定尺寸例化。
//
// 宏模型：sram_2rw0r0w_32_128_freepdk45（OpenRAM 生成，make sram 产物落
//   tmp/sram/；make rtl/netlist 编译前自动检查并生成，不入库）。
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

    // ---------------- 几何守卫（宏为固定尺寸例化，仅基线口径） ----------------
    generate
        if (VEC_W != 256 || DEPTH != 128 || NLANES != 8 || IDX_W != 7) begin : g_cfg_err
            initial begin
                $display("rf(sram): 宏版仅支持基线几何（128 字×256 位、8 lane），收到 VEC_W=%0d DEPTH=%0d NLANES=%0d",
                         VEC_W, DEPTH, NLANES);
                $finish;
            end
        end
    endgenerate

    // ---------------- 写口三源固定优先级仲裁（rf_spec §5.3，与寄存器版相同） ----
    wire lsu_g  = lsu_rf_wb_vld;
    wire ialu_g = ialu_rf_wb_vld && !lsu_rf_wb_vld;
    wire falu_g = falu_rf_wb_vld && !lsu_rf_wb_vld && !ialu_rf_wb_vld;
    wire wb_fire = lsu_g | ialu_g | falu_g;

    assign rf_lsu_wb_rdy  = 1'b1;
    assign rf_ialu_wb_rdy = !lsu_rf_wb_vld;
    assign rf_falu_wb_rdy = !lsu_rf_wb_vld && !ialu_rf_wb_vld;

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

    // ---------------- 读通道握手（单在途、零间隙周转，与寄存器版相同） --------
    reg rsp_vld;
    assign rf_sf_rddata_vld = rsp_vld;
    assign rf_sf_rd_rdy     = !rsp_vld || sf_rf_rddata_rdy;
    wire rd_fire  = sf_rf_rd_vld && rf_sf_rd_rdy;
    wire rsp_fire = rsp_vld && sf_rf_rddata_rdy;

    // ---------------- 寻址（索引 {warp_id, reg}，7 位，宏全深度无译码） -------
    wire [IDX_W-1:0] ridx_a = {sf_rf_rd_warp_id, sf_rf_rd_rs1};
    wire [IDX_W-1:0] ridx_b = {sf_rf_rd_warp_id, sf_rf_rd_rs2};
    wire [IDX_W-1:0] wb_idx = {wb_warp_id, wb_rd};
    wire wb_en = rst_n && wb_fire && (wb_rd != {REG_AW{1'b0}});

    // ---------------- 同拍写旁路上下文（§5.2）：请求握手拍随应答锁存 ----------
    wire match_a_c = wb_en && (wb_idx == ridx_a);
    wire match_b_c = wb_en && (wb_idx == ridx_b);

    reg              held;                // 应答已锁存进寄存器（背压自次拍起）
    reg              a_r0_q,  b_r0_q;
    reg              byp_a_q, byp_b_q;
    reg [NLANES-1:0] byp_mask_a_q, byp_mask_b_q;
    reg [VEC_W-1:0]  byp_data_a_q, byp_data_b_q;
    reg [VEC_W-1:0]  rsp_a, rsp_b;

    // ---------------- 宏阵列：8 lane × 镜像对（M0 供 rs1、M1 供 rs2） ---------
    wire [DATA_W-1:0] dout_m0 [0:NLANES-1];
    wire [DATA_W-1:0] dout_m1 [0:NLANES-1];

    // 复位清零扫描：两口并行，口 0 扫低 64 字、口 1 扫高 64 字，64 拍扫全阵
    reg [5:0] clr_cnt;                    // 自由运行，复位保持 ≥64 拍即扫全阵
    always @(posedge clk)
        clr_cnt <= clr_cnt + 1'b1;

    wire clear_on = !rst_n;
    wire [IDX_W-1:0] clr_addr0 = {1'b0, clr_cnt};      // 0..63
    wire [IDX_W-1:0] clr_addr1 = {1'b1, clr_cnt};      // 64..127

    // 读呈现：请求呈现期间持续供址（vld 保持协议下地址稳定，重复锁存无害），
    // 握手拍末沿锁存、次拍输出有效
    wire rd_go = rst_n && sf_rf_rd_vld;

    genvar gl;
    generate
        for (gl = 0; gl < NLANES; gl = gl + 1) begin : g_lane
            // 口 0：写（lane 掩码选中本宏）/ 清零扫描低半
            wire p0_csb = !((clear_on) ||
                            (wb_en && wb_lane_mask[gl]));
            wire [IDX_W-1:0] p0_addr = clear_on ? clr_addr0 : wb_idx;
            wire [DATA_W-1:0] p0_din = clear_on ? {DATA_W{1'b0}}
                                                : wb_wdata[gl*DATA_W +: DATA_W];

            // 口 1：rs1/rs2 读 / 清零扫描高半
            wire p1_csb = !(clear_on || rd_go);
            wire [IDX_W-1:0] p1_addr_m0 = clear_on ? clr_addr1 : ridx_a;
            wire [IDX_W-1:0] p1_addr_m1 = clear_on ? clr_addr1 : ridx_b;

            sram_2rw0r0w_32_128_freepdk45 #(.VERBOSE(0)) u_m0 (
                .clk0 (clk),
                .csb0 (p0_csb),
                .web0 (1'b0),                 // 0 = 写
                .addr0(p0_addr),
                .din0 (p0_din),
                .dout0(),
                .clk1 (clk),
                .csb1 (p1_csb),
                .web1 (clear_on ? 1'b0 : 1'b1),   // 正常 1 = 读；清零 0 = 写
                .addr1(p1_addr_m0),
                .din1 ({DATA_W{1'b0}}),
                .dout1(dout_m0[gl])
            );

            sram_2rw0r0w_32_128_freepdk45 #(.VERBOSE(0)) u_m1 (
                .clk0 (clk),
                .csb0 (p0_csb),
                .web0 (1'b0),
                .addr0(p0_addr),
                .din0 (p0_din),
                .dout0(),
                .clk1 (clk),
                .csb1 (p1_csb),
                .web1 (clear_on ? 1'b0 : 1'b1),
                .addr1(p1_addr_m1),
                .din1 ({DATA_W{1'b0}}),
                .dout1(dout_m1[gl])
            );
        end
    endgenerate

    // 逐 lane 宏读出拼成整字
    wire [VEC_W-1:0] base_a, base_b;
    generate
        for (gl = 0; gl < NLANES; gl = gl + 1) begin : g_concat
            assign base_a[gl*DATA_W +: DATA_W] = dout_m0[gl];
            assign base_b[gl*DATA_W +: DATA_W] = dout_m1[gl];
        end
    endgenerate

    // ---------------- 应答载荷：首拍直通宏读出，背压自次拍起寄存器保持 --------
    wire [VEC_W-1:0] merge_a, merge_b;
    generate
        for (gl = 0; gl < NLANES; gl = gl + 1) begin : g_merge
            assign merge_a[gl*DATA_W +: DATA_W] =
                (byp_a_q && byp_mask_a_q[gl]) ? byp_data_a_q[gl*DATA_W +: DATA_W]
                                              : base_a[gl*DATA_W +: DATA_W];
            assign merge_b[gl*DATA_W +: DATA_W] =
                (byp_b_q && byp_mask_b_q[gl]) ? byp_data_b_q[gl*DATA_W +: DATA_W]
                                              : base_b[gl*DATA_W +: DATA_W];
        end
    endgenerate

    wire [VEC_W-1:0] out_a = a_r0_q ? {VEC_W{1'b0}} : merge_a;
    wire [VEC_W-1:0] out_b = b_r0_q ? {VEC_W{1'b0}} : merge_b;

    wire capture = rsp_vld && !sf_rf_rddata_rdy && !held;

    // ---------------- 时序：应答状态与上下文锁存 ------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rsp_vld      <= 1'b0;
            held         <= 1'b0;
            rsp_a        <= {VEC_W{1'b0}};
            rsp_b        <= {VEC_W{1'b0}};
            a_r0_q       <= 1'b0;
            b_r0_q       <= 1'b0;
            byp_a_q      <= 1'b0;
            byp_b_q      <= 1'b0;
            byp_mask_a_q <= {NLANES{1'b0}};
            byp_mask_b_q <= {NLANES{1'b0}};
            byp_data_a_q <= {VEC_W{1'b0}};
            byp_data_b_q <= {VEC_W{1'b0}};
        end else begin
            if (rd_fire) begin
                // 新应答入级（含零间隙周转）：宏地址于本拍末沿锁存，
                // 次拍宏输出有效；旁路上下文随应答锁存
                rsp_vld      <= 1'b1;
                held         <= 1'b0;
                a_r0_q       <= (sf_rf_rd_rs1 == {REG_AW{1'b0}});
                b_r0_q       <= (sf_rf_rd_rs2 == {REG_AW{1'b0}});
                byp_a_q      <= match_a_c;
                byp_b_q      <= match_b_c;
                byp_mask_a_q <= wb_lane_mask;
                byp_mask_b_q <= wb_lane_mask;
                byp_data_a_q <= wb_wdata;
                byp_data_b_q <= wb_wdata;
            end else if (rsp_fire) begin
                rsp_vld <= 1'b0;
                held    <= 1'b0;
            end
            // 应答首拍未被消费：锁存载荷，其后由寄存器保持（vld 保持期稳定）
            if (capture) begin
                rsp_a <= out_a;
                rsp_b <= out_b;
                held  <= 1'b1;
            end
        end
    end

    // ---------------- 应答载荷导出 --------------------------------------------
    assign rf_sf_rddata_a = held ? rsp_a : out_a;
    assign rf_sf_rddata_b = held ? rsp_b : out_b;

endmodule
