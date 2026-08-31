// =============================================================================
// easy_simt · falu — Floating-point ALU（ma_spec §6）
//
// FMUL/FADD/FNEG，IEEE-754 binary32、RN 舍入（与 top/cmodel/softfloat.c
// 位精确一致，含 FTZ）。数据通路 8 lane 并行、锁步。
// 完成路径（falu_spec §1.2）：三种操作码均为 issue -> wb -> wbdone。
//
// 设计依据：falu/docs/falu_spec_v0.1.md；
//   端口命名与 intf_spec §6 一致（issue 载荷见 intf_spec §2），
//   握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module falu #(
    parameter DATA_W   = 32,              // 数据/地址/载荷位宽（intf_spec §1.4）
    parameter NWARPS   = 4,               // warp 数/块（ma_spec §1.2）
    parameter NLANES   = 8,               // lane 数/warp（ma_spec §1.2）
    parameter REG_AW   = 5,               // 寄存器地址位宽（intf_spec §1.4）
    parameter OPCODE_W = 5,               // 操作码位宽（intf_spec §1.4）
    localparam WARP_IW = (NWARPS > 1) ? $clog2(NWARPS) : 1,
    localparam VEC_W   = NLANES * DATA_W,
    // ---- FADD 和差位宽：50 位定点场（尾数 24 位 + 小数扩展 25 位 + 余量 1 位）+ 进位 1 位 ----
    localparam MW      = 51
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // sf_falu_issue：sf 发射（载荷为 sf 译码归一化形式，falu_spec §1.3）
    input  wire                  sf_falu_issue_vld,
    input  wire [OPCODE_W-1:0]   sf_falu_issue_opcode,
    input  wire [REG_AW-1:0]     sf_falu_issue_rd,
    input  wire [WARP_IW-1:0]    sf_falu_issue_warp_id,
    input  wire [NLANES-1:0]     sf_falu_issue_lane_mask,
    input  wire [VEC_W-1:0]      sf_falu_issue_opa,
    input  wire [VEC_W-1:0]      sf_falu_issue_opb,  // FNEG 时不读取（falu_spec §1.3）
    output wire                  falu_sf_issue_rdy,

    // falu_rf_wb：结果写回
    output wire                  falu_rf_wb_vld,
    output wire [WARP_IW-1:0]    falu_rf_wb_warp_id,
    output wire [REG_AW-1:0]     falu_rf_wb_rd,
    output wire [NLANES-1:0]     falu_rf_wb_lane_mask,
    output wire [VEC_W-1:0]      falu_rf_wb_wdata,
    input  wire                  rf_falu_wb_rdy,

    // falu_sf_wbdone：写回完成（供 sf 清记分板）
    output wire                  falu_sf_wbdone_vld,
    output wire [WARP_IW-1:0]    falu_sf_wbdone_warp_id,
    output wire [REG_AW-1:0]     falu_sf_wbdone_rd,
    input  wire                  sf_falu_wbdone_rdy
);

    // ---------------- 操作码（isa_spec §1.8，本模块合法子集） ----------------
    localparam [OPCODE_W-1:0] OP_FMUL = 5'h08;
    localparam [OPCODE_W-1:0] OP_FADD = 5'h09;
    localparam [OPCODE_W-1:0] OP_FNEG = 5'h0A;

    // ---------------- 状态编码（falu_spec §6.1） ----------------
    localparam [1:0] S_IDLE = 2'd0;   // 空闲，可接受 issue
    localparam [1:0] S_WB   = 2'd1;   // 写回段
    localparam [1:0] S_WBD  = 2'd2;   // 写回完成段

    // ---------------- 内部状态（falu_spec §4） ----------------
    reg  [1:0]           state;
    reg  [WARP_IW-1:0]   wb_warp_id, wbd_warp_id;
    reg  [REG_AW-1:0]    wb_rd, wbd_rd;
    reg  [NLANES-1:0]    wb_lane_mask;
    reg  [VEC_W-1:0]     wb_wdata;

    // ---------------- issue 载荷切片 ----------------
    wire [OPCODE_W-1:0] op   = sf_falu_issue_opcode;
    wire [REG_AW-1:0]   rd_i = sf_falu_issue_rd;
    wire [WARP_IW-1:0]  w_i  = sf_falu_issue_warp_id;
    wire [NLANES-1:0]   m_i  = sf_falu_issue_lane_mask;

    // ---------------- 握手组合判据 ----------------
    wire issue_fire = sf_falu_issue_vld  && falu_sf_issue_rdy;
    wire wb_fire    = falu_rf_wb_vld     && rf_falu_wb_rdy;
    wire wbd_fire   = falu_sf_wbdone_vld && sf_falu_wbdone_rdy;

    // ---------------- 通道 vld / rdy（寄存器输出经状态译码，falu_spec §4） ----------------
    assign falu_sf_issue_rdy  = (state == S_IDLE);
    assign falu_rf_wb_vld     = (state == S_WB);
    assign falu_sf_wbdone_vld = (state == S_WBD);

    // ---------------- 解包辅助：最高置位位（x != 0 时返回值有效） ----------------
    function automatic integer hi23(input [22:0] x);
        integer i;
        reg found;
        begin
            hi23 = 0;
            found = 0;
            for (i = 22; i >= 0; i = i - 1)
                if (!found && x[i]) begin
                    hi23 = i;
                    found = 1;
                end
        end
    endfunction

    function automatic integer hiM(input [MW-1:0] x);
        integer i;
        reg found;
        begin
            hiM = 0;
            found = 0;
            for (i = MW - 1; i >= 0; i = i - 1)
                if (!found && x[i]) begin
                    hiM = i;
                    found = 1;
                end
        end
    endfunction

    // ---------------- 每 lane 数据通路（falu_spec §5） ----------------
    wire [VEC_W-1:0] wdata_c;   // 运算结果（已按 lane_mask 门控）

    genvar gi;
    generate
        for (gi = 0; gi < NLANES; gi = gi + 1) begin : g_lane
            wire [DATA_W-1:0] a = sf_falu_issue_opa[gi*DATA_W +: DATA_W];
            wire [DATA_W-1:0] b = sf_falu_issue_opb[gi*DATA_W +: DATA_W];

            // ---- 解包（§5.2） ----
            wire        a_sg = a[31],        b_sg = b[31];
            wire [7:0]  a_ex = a[30:23],     b_ex = b[30:23];
            wire [22:0] a_fr = a[22:0],      b_fr = b[22:0];

            wire a_nan  = (a_ex == 8'hFF) && (a_fr != 23'd0);
            wire a_inf  = (a_ex == 8'hFF) && (a_fr == 23'd0);
            wire a_zero = (a[30:0] == 31'd0);
            wire a_den  = (a_ex == 8'd0)  && (a_fr != 23'd0);
            wire b_nan  = (b_ex == 8'hFF) && (b_fr != 23'd0);
            wire b_inf  = (b_ex == 8'hFF) && (b_fr == 23'd0);
            wire b_zero = (b[30:0] == 31'd0);
            wire b_den  = (b_ex == 8'd0)  && (b_fr != 23'd0);

            wire [4:0] a_p = hi23(a_fr);    // denorm 尾数最高置位位
            wire [4:0] b_p = hi23(b_fr);

            // 值 = (-1)^sg * mant * 2^q；normal: mant={1,frac}, q=e-150；
            // denorm: MSB 对齐至 bit23, q = p - 172（softfloat.c funpack）
            wire [23:0] a_mant = a_den ? ({1'b0, a_fr} << (5'd23 - a_p))
                                       : {1'b1, a_fr};
            wire [23:0] b_mant = b_den ? ({1'b0, b_fr} << (5'd23 - b_p))
                                       : {1'b1, b_fr};
            wire signed [11:0] a_q = a_den ? ({7'b0, a_p} - 12'sd172)
                                           : ({4'b0, a_ex} - 12'sd150);
            wire signed [11:0] b_q = b_den ? ({7'b0, b_p} - 12'sd172)
                                           : ({4'b0, b_ex} - 12'sd150);

            // ---- FMUL（§5.3） ----
            wire        m_sg  = a_sg ^ b_sg;
            wire [47:0] prod  = a_mant * b_mant;
            wire        p47   = prod[47];   // 积最高位：1 -> 右移 24，0 -> 右移 23
            wire [23:0] m_keep  = p47 ? prod[47:24] : prod[46:23];
            wire        m_guard = p47 ? prod[23]    : prod[22];
            wire        m_stky  = p47 ? |prod[22:0] : |prod[21:0];
            wire signed [12:0] q_m0 = {a_q[11], a_q} + {b_q[11], b_q};
            wire signed [12:0] q_m1 = q_m0 + (p47 ? 13'sd24 : 13'sd23);
            wire        m_rup  = m_guard && (m_stky || m_keep[0]);
            wire [24:0] m_manr = {1'b0, m_keep} + {24'd0, m_rup};
            wire        m_cy   = m_manr[24];
            wire [23:0] m_manf = m_cy ? m_manr[24:1] : m_manr[23:0];
            wire signed [12:0] q_mf = m_cy ? q_m1 + 13'sd1 : q_m1;
            wire signed [13:0] m_e  = {q_mf[12], q_mf} + 14'sd150;
            wire [31:0] mul_norm = (m_e >= 14'sd255) ? {m_sg, 31'h7F800000}
                                 : (m_e <= 14'sd0)   ? {m_sg, 31'd0}
                                 : {m_sg, m_e[7:0], m_manf[22:0]};
            wire [31:0] mul_r = (a_nan || b_nan) ? 32'h7FC00000
                              : (a_inf || b_inf) ? ((a_zero || b_zero) ? 32'h7FC00000
                                                   : {m_sg, 31'h7F800000})
                              : (a_zero || b_zero) ? {m_sg, 31'd0}
                              : mul_norm;

            // ---- FADD（§5.4） ----
            // 特殊值：x + 0 = 规格化形式（normal 原样；denorm 按 FTZ ±0）
            wire [31:0] canon_a = a_den ? {a_sg, 31'd0} : a;
            wire [31:0] canon_b = b_den ? {b_sg, 31'd0} : b;

            // 对齐（50 位定点场，标度 2^(q_max-25)）：较大者置于 [48:25]，
            // 较小者按真实指数差右移；移出场的位全部或进对齐 sticky
            wire signed [11:0] q_max = (a_q > b_q) ? a_q : b_q;
            wire [8:0] delta = (a_q >= b_q) ? (a_q - b_q) : (b_q - a_q);
            wire        a_ge_b = (a_q > b_q) || ((a_q == b_q) && (a_mant >= b_mant));
            wire [23:0] lg_mant = a_ge_b ? a_mant : b_mant;
            wire [23:0] sm_mant = a_ge_b ? b_mant : a_mant;
            wire [MW-2:0] L50 = {1'b0, lg_mant, 25'd0};
            wire [MW-2:0] S50 = {1'b0, sm_mant, 25'd0} >> delta;

            // 对齐 sticky：被右移出场的尾数位（sm 的低 n_lost 位）
            wire [8:0] nl9 = delta - 9'd25;
            wire [4:0] n_lost = (delta <= 9'd25) ? 5'd0
                              : (delta >= 9'd49) ? 5'd24
                              : nl9[4:0];
            wire [23:0] sm_lo_mask = {24{1'b1}} >> (5'd24 - n_lost);
            wire        sticky_al  = |(sm_mant & sm_lo_mask);

            wire        same_sg = (a_sg == b_sg);
            wire [MW-1:0] M_sum = same_sg ? ({1'b0, L50} + {1'b0, S50})
                                          : ({1'b0, L50} - {1'b0, S50});
            // 异号且有对齐丢失位：真值 = M_sum - D（0<D<1 场单位），
            // 按 (M_sum-1) 的整数部分舍入，丢失量并入最终 sticky
            wire [MW-1:0] M = (~same_sg && sticky_al) ? (M_sum - {{(MW-1){1'b0}}, 1'b1})
                                                      : M_sum;
            wire add_sg = same_sg ? a_sg : (a_ge_b ? a_sg : b_sg);
            wire m_zero = (M_sum == {MW{1'b0}});   // 精确抵消 -> +0（该情形 sticky_al 必 0）

            // 单舍入（M != 0；定点标度：value = M * 2^(q_max - 25)）
            wire [5:0] lz_m    = hiM(M);
            wire       lz_ge23 = (lz_m >= 6'd23);
            wire [5:0] shift_r = lz_ge23 ? (lz_m - 6'd23) : 6'd0;
            wire [5:0] s1      = (lz_m >= 6'd24) ? (lz_m - 6'd24) : 6'd0; // max(shift-1,0)
            wire [MW-1:0] Ms  = M >> shift_r;
            wire [MW-1:0] Ms1 = M >> s1;
            wire [23:0] a_keep  = Ms[23:0];
            wire        a_guard = lz_ge23 && (shift_r != 6'd0) && Ms1[0];
            wire [MW-1:0] lo_mask = {MW{1'b1}} >> (MW - s1);  // 低 s1 位置 1
            wire        a_stky  = (|(M & lo_mask)) | sticky_al;
            wire        a_rup   = a_guard && (a_stky || a_keep[0]);
            wire [24:0] a_manr  = {1'b0, a_keep} + {24'd0, a_rup};
            wire        a_cy    = a_manr[24];
            wire [23:0] a_manhi = a_cy ? a_manr[24:1] : a_manr[23:0];

            // qq = q_max + lz - 48（lz >= 23 与 lz < 23 两支统一，含进位 +1）
            wire signed [12:0] qq0 = {q_max[11], q_max}
                                     + $signed({7'b0, lz_m}) - 13'sd48;
            wire signed [12:0] qq  = a_cy ? qq0 + 13'sd1 : qq0;
            // lz < 23：左移对齐，无舍入（深抵消仅发生在 delta <= 1，sticky_al 必 0）
            wire [5:0]  lshift  = 6'd23 - lz_m;
            wire [23:0] a_manlo = M[23:0] << lshift;

            wire [23:0] a_manf = lz_ge23 ? a_manhi : a_manlo;
            wire signed [13:0] a_e = {qq[12], qq} + 14'sd150;
            wire [31:0] add_norm = (a_e >= 14'sd255) ? {add_sg, 31'h7F800000}
                                 : (a_e <= 14'sd0)   ? {add_sg, 31'd0}
                                 : {add_sg, a_e[7:0], a_manf[22:0]};

            wire [31:0] add_r = (a_nan || b_nan) ? 32'h7FC00000
                              : (a_inf || b_inf) ? ((a_inf && b_inf && (a_sg != b_sg))
                                                    ? 32'h7FC00000
                                                    : {(a_inf ? a_sg : b_sg), 31'h7F800000})
                              : (a_zero && b_zero) ? {(a_sg & b_sg), 31'd0}
                              : a_zero ? canon_b
                              : b_zero ? canon_a
                              : (~same_sg && m_zero) ? 32'd0
                              : add_norm;

            // ---- FNEG（§5.5） ----
            wire [31:0] neg_r = {~a_sg, a[30:0]};

            // ---- 操作码选择与 lane 门控（§5.1） ----
            wire [DATA_W-1:0] lane_res = (op == OP_FADD) ? add_r
                                       : (op == OP_FNEG) ? neg_r
                                       :                   mul_r;
            assign wdata_c[gi*DATA_W +: DATA_W] =
                m_i[gi] ? lane_res : {DATA_W{1'b0}};
        end
    endgenerate

    // ---------------- 主状态机（§6） ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            wb_warp_id   <= {WARP_IW{1'b0}};
            wb_rd        <= {REG_AW{1'b0}};
            wb_lane_mask <= {NLANES{1'b0}};
            wb_wdata     <= {VEC_W{1'b0}};
            wbd_warp_id  <= {WARP_IW{1'b0}};
            wbd_rd       <= {REG_AW{1'b0}};
        end else begin
            case (state)
            // ----------------------------------------------------
            // S_IDLE：接受 issue，运算当拍完成、结果入输出级（§6.2）
            // ----------------------------------------------------
            S_IDLE: begin
                if (issue_fire) begin
                    wb_warp_id   <= w_i;
                    wb_rd        <= rd_i;
                    wb_lane_mask <= m_i;
                    wb_wdata     <= wdata_c;
                    wbd_warp_id  <= w_i;
                    wbd_rd       <= rd_i;
                    state        <= S_WB;
                end
            end
            // ----------------------------------------------------
            // S_WB / S_WBD：各自通道保持至握手（§6.2）
            // ----------------------------------------------------
            S_WB: begin
                if (wb_fire) state <= S_WBD;
            end
            S_WBD: begin
                if (wbd_fire) state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

    // ---------------- 载荷导出（vld 保持期内稳定，§4） ----------------
    assign falu_rf_wb_warp_id     = wb_warp_id;
    assign falu_rf_wb_rd          = wb_rd;
    assign falu_rf_wb_lane_mask   = wb_lane_mask;
    assign falu_rf_wb_wdata       = wb_wdata;
    assign falu_sf_wbdone_warp_id = wbd_warp_id;
    assign falu_sf_wbdone_rd      = wbd_rd;

endmodule
