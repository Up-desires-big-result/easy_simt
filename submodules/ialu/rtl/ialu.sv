// =============================================================================
// easy_simt · ialu — Integer ALU（ma_spec §5）
//
// 整数运算 + 分支解析，8 lane 并行、锁步。谓词寄存器 P0..P3 驻留本模块
// （SETP 写、BR 读）。完成路径（ialu_spec §1.2）：
//   ALU 类（IMAD/IADD/SHL/XOR/ORI/LUI/LDP/CSRR）：issue -> wb -> wbdone
//   SETP：issue -> wbdone（仅谓词写，无寄存器写回）
//   BR：  issue -> br（无写回）
//
// 设计依据：ialu/docs/ialu_spec_v0.1.md；
//   端口命名与 intf_spec §5 一致（issue 载荷含 opc，intf_spec 勘误 / 偏差 C5），
//   握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module ialu #(
    parameter DATA_W   = 32,              // 数据/地址/载荷位宽（intf_spec §1.4）
    parameter NWARPS   = 4,               // warp 数/块（ma_spec §1.2）
    parameter NLANES   = 8,               // lane 数/warp（ma_spec §1.2）
    parameter REG_AW   = 5,               // 寄存器地址位宽（intf_spec §1.4）
    parameter OPCODE_W = 5,               // 操作码位宽（intf_spec §1.4）
    parameter BRT_IW   = 2,               // BRT 表项索引位宽（intf_spec §1.4）
    localparam WARP_IW = (NWARPS > 1) ? $clog2(NWARPS) : 1,
    localparam VEC_W   = NLANES * DATA_W,
    localparam PRED_W  = NWARPS * 4 * NLANES
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // sf_ialu_issue：sf 发射（载荷为 sf 译码归一化形式，ialu_spec §1.3）
    input  wire                  sf_ialu_issue_vld,
    input  wire [OPCODE_W-1:0]   sf_ialu_issue_opcode,
    input  wire [REG_AW-1:0]     sf_ialu_issue_rd,
    input  wire [WARP_IW-1:0]    sf_ialu_issue_warp_id,
    input  wire [NLANES-1:0]     sf_ialu_issue_lane_mask,
    input  wire [DATA_W-1:0]     sf_ialu_issue_pc,   // 接收但内部不使用（ialu_spec §1.3）
    input  wire [DATA_W-1:0]     sf_ialu_issue_imm,
    input  wire [VEC_W-1:0]      sf_ialu_issue_opa,
    input  wire [VEC_W-1:0]      sf_ialu_issue_opb,
    input  wire [VEC_W-1:0]      sf_ialu_issue_opc,  // IMAD 第三源（偏差 C5）
    output wire                  ialu_sf_issue_rdy,

    // ialu_sf_br：分支决议回注 sf
    output wire                  ialu_sf_br_vld,
    output wire [WARP_IW-1:0]    ialu_sf_br_warp_id,
    output wire [NLANES-1:0]     ialu_sf_br_taken,
    output wire [DATA_W-1:0]     ialu_sf_br_target,
    output wire [BRT_IW-1:0]     ialu_sf_br_brt_idx,
    input  wire                  sf_ialu_br_rdy,

    // ialu_rf_wb：结果写回
    output wire                  ialu_rf_wb_vld,
    output wire [WARP_IW-1:0]    ialu_rf_wb_warp_id,
    output wire [REG_AW-1:0]     ialu_rf_wb_rd,
    output wire [NLANES-1:0]     ialu_rf_wb_lane_mask,
    output wire [VEC_W-1:0]      ialu_rf_wb_wdata,
    input  wire                  rf_ialu_wb_rdy,

    // ialu_sf_wbdone：写回完成（供 sf 清记分板）
    output wire                  ialu_sf_wbdone_vld,
    output wire [WARP_IW-1:0]    ialu_sf_wbdone_warp_id,
    output wire [REG_AW-1:0]     ialu_sf_wbdone_rd,
    input  wire                  sf_ialu_wbdone_rdy
);

    // ---------------- 操作码（isa_spec §1.8，本模块合法子集） ----------------
    localparam [OPCODE_W-1:0] OP_IMAD = 5'h01;
    localparam [OPCODE_W-1:0] OP_IADD = 5'h02;
    localparam [OPCODE_W-1:0] OP_SHL  = 5'h03;
    localparam [OPCODE_W-1:0] OP_XOR  = 5'h04;
    localparam [OPCODE_W-1:0] OP_ORI  = 5'h05;
    localparam [OPCODE_W-1:0] OP_LUI  = 5'h06;
    localparam [OPCODE_W-1:0] OP_SETP = 5'h07;
    localparam [OPCODE_W-1:0] OP_LDP  = 5'h0F;
    localparam [OPCODE_W-1:0] OP_CSRR = 5'h10;
    localparam [OPCODE_W-1:0] OP_BR   = 5'h11;

    // ---------------- 状态编码（ialu_spec §6.1） ----------------
    localparam [1:0] S_IDLE = 2'd0;   // 空闲，可接受 issue
    localparam [1:0] S_WB   = 2'd1;   // 写回段
    localparam [1:0] S_WBD  = 2'd2;   // 写回完成段
    localparam [1:0] S_BR   = 2'd3;   // 分支决议段

    // ---------------- 内部状态（ialu_spec §4） ----------------
    reg  [1:0]           state;
    reg  [WARP_IW-1:0]   wb_warp_id, wbd_warp_id, br_warp_id;
    reg  [REG_AW-1:0]    wb_rd, wbd_rd;
    reg  [NLANES-1:0]    wb_lane_mask;
    reg  [VEC_W-1:0]     wb_wdata;
    reg  [NLANES-1:0]    br_taken;
    reg  [DATA_W-1:0]    br_target;
    reg  [PRED_W-1:0]    pred;        // 谓词阵列：{warp, pd, lane} 位图

    // ---------------- issue 载荷切片 ----------------
    wire [OPCODE_W-1:0] op    = sf_ialu_issue_opcode;
    wire [REG_AW-1:0]   rd_i  = sf_ialu_issue_rd;
    wire [WARP_IW-1:0]  w_i   = sf_ialu_issue_warp_id;
    wire [NLANES-1:0]   m_i   = sf_ialu_issue_lane_mask;
    wire [DATA_W-1:0]   imm_i = sf_ialu_issue_imm;

    wire is_setp = (op == OP_SETP);
    wire is_br   = (op == OP_BR);

    // ---------------- 握手组合判据 ----------------
    wire issue_fire = sf_ialu_issue_vld  && ialu_sf_issue_rdy;
    wire wb_fire    = ialu_rf_wb_vld     && rf_ialu_wb_rdy;
    wire wbd_fire   = ialu_sf_wbdone_vld && sf_ialu_wbdone_rdy;
    wire br_fire    = ialu_sf_br_vld     && sf_ialu_br_rdy;

    // ---------------- 通道 vld / rdy（寄存器输出经状态译码，ialu_spec §4） ----------------
    assign ialu_sf_issue_rdy  = (state == S_IDLE);
    assign ialu_rf_wb_vld     = (state == S_WB);
    assign ialu_sf_wbdone_vld = (state == S_WBD);
    assign ialu_sf_br_vld     = (state == S_BR);

    // ---------------- 每 lane 数据通路（ialu_spec §5） ----------------
    wire [VEC_W-1:0]  wdata_c;   // 运算结果（已按 lane_mask 门控）
    wire [NLANES-1:0] cmp_ok;    // SETP 每 lane 比较结果

    genvar gi;
    generate
        for (gi = 0; gi < NLANES; gi = gi + 1) begin : g_lane
            wire [DATA_W-1:0] opa_l = sf_ialu_issue_opa[gi*DATA_W +: DATA_W];
            wire [DATA_W-1:0] opb_l = sf_ialu_issue_opb[gi*DATA_W +: DATA_W];
            wire [DATA_W-1:0] opc_l = sf_ialu_issue_opc[gi*DATA_W +: DATA_W];

            // ---- 整数运算（§5.2） ----
            wire signed [2*DATA_W-1:0] prod = $signed(opa_l) * $signed(opb_l);
            wire [DATA_W-1:0] imad_r = prod[DATA_W-1:0] + opc_l;
            wire [DATA_W-1:0] iadd_r = opa_l + opb_l;
            wire [DATA_W-1:0] shl_r  = opa_l << opb_l[4:0];
            wire [DATA_W-1:0] xor_r  = opa_l ^ opb_l;
            wire [DATA_W-1:0] ori_r  = opa_l | opb_l;

            reg [DATA_W-1:0] alu_r;
            always @* begin
                case (op)
                    OP_IMAD: alu_r = imad_r;
                    OP_IADD: alu_r = iadd_r;
                    OP_SHL : alu_r = shl_r;
                    OP_XOR : alu_r = xor_r;
                    OP_ORI : alu_r = ori_r;
                    default: alu_r = opa_l;   // LUI/LDP/CSRR 直通（偏差 C3）
                endcase
            end

            // ---- SETP 比较（§5.3） ----
            wire        fmt  = imm_i[3];
            wire [2:0]  cond = imm_i[2:0];

            wire s_lt = $signed(opa_l) < $signed(opb_l);
            wire s_eq = (opa_l == opb_l);
            wire i_ok = (cond == 3'd0) ? s_lt
                      : (cond == 3'd1) ? (s_lt || s_eq)
                      : (cond == 3'd2) ? s_eq
                      : (cond == 3'd3) ? ~s_eq
                      : (cond == 3'd4) ? ~s_lt
                      :                  (~s_lt && ~s_eq);

            // fgt(x,y)：IEEE-754 有序大于，与 cmodel f32_gt 对齐（§5.3）
            wire a_nan  = (opa_l[30:23] == 8'hFF) && (opa_l[22:0] != 0);
            wire b_nan  = (opb_l[30:23] == 8'hFF) && (opb_l[22:0] != 0);
            wire a_zero = (opa_l[DATA_W-2:0] == 0);
            wire b_zero = (opb_l[DATA_W-2:0] == 0);
            wire [DATA_W-2:0] a_mag = opa_l[DATA_W-2:0];
            wire [DATA_W-2:0] b_mag = opb_l[DATA_W-2:0];

            wire fgt_ab = (~a_nan && ~b_nan) &&
                          ( a_zero ? (~b_zero &&  opb_l[DATA_W-1])
                          : b_zero ? (~a_zero && ~opa_l[DATA_W-1])
                          : (opa_l[DATA_W-1] != opb_l[DATA_W-1]) ? ~opa_l[DATA_W-1]
                          : ~opa_l[DATA_W-1] ? (a_mag > b_mag)
                          :                    (b_mag > a_mag) );
            wire fgt_ba = (~a_nan && ~b_nan) &&
                          ( b_zero ? (~a_zero &&  opa_l[DATA_W-1])
                          : a_zero ? (~b_zero && ~opb_l[DATA_W-1])
                          : (opa_l[DATA_W-1] != opb_l[DATA_W-1]) ? ~opb_l[DATA_W-1]
                          : ~opb_l[DATA_W-1] ? (b_mag > a_mag)
                          :                    (a_mag > b_mag) );
            wire eq_bits = (opa_l == opb_l);   // 按位 eq/ne（cmodel 口径）
            wire f_ok = (cond == 3'd5) ? fgt_ab
                      : (cond == 3'd4) ? (fgt_ab || eq_bits)
                      : (cond == 3'd0) ? fgt_ba
                      : (cond == 3'd1) ? (fgt_ba || eq_bits)
                      : (cond == 3'd2) ? eq_bits
                      :                  ~eq_bits;

            assign cmp_ok[gi] = fmt ? f_ok : i_ok;

            // ---- lane 门控：非活动 lane 输出 0（§5.1） ----
            assign wdata_c[gi*DATA_W +: DATA_W] =
                m_i[gi] ? alu_r : {DATA_W{1'b0}};
        end
    endgenerate

    // ---------------- BR 决议组合逻辑（§5.4） ----------------
    wire        br_u   = imm_i[DATA_W-1];
    wire        br_neg = imm_i[DATA_W-2];
    wire [1:0]  psel   = rd_i[1:0];
    wire [WARP_IW+2-1:0] pred_row = {w_i, psel};   // warp_id*4 + psel
    wire [NLANES-1:0] pred_rd = pred[pred_row*NLANES +: NLANES];
    wire [NLANES-1:0] taken_c = br_u   ? m_i
                              : br_neg ? (m_i & ~pred_rd)
                              :          (m_i &  pred_rd);

    // ---------------- 谓词写入（SETP，§5.3/§1.4） ----------------
    wire [WARP_IW+2-1:0] pred_wr_row = {w_i, rd_i[1:0]};   // warp_id*4 + pd
    integer pl;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pred <= {PRED_W{1'b0}};
        end else if (issue_fire && is_setp) begin
            for (pl = 0; pl < NLANES; pl = pl + 1)
                if (m_i[pl])
                    pred[pred_wr_row*NLANES + pl] <= cmp_ok[pl];
        end
    end

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
            br_warp_id   <= {WARP_IW{1'b0}};
            br_taken     <= {NLANES{1'b0}};
            br_target    <= {DATA_W{1'b0}};
        end else begin
            case (state)
            // ----------------------------------------------------
            // S_IDLE：接受 issue，运算当拍完成、结果入输出级（§6.2）
            // ----------------------------------------------------
            S_IDLE: begin
                if (issue_fire) begin
                    wbd_warp_id <= w_i;
                    wbd_rd      <= rd_i;
                    if (is_setp) begin
                        state <= S_WBD;
                    end else if (is_br) begin
                        br_warp_id <= w_i;
                        br_taken   <= taken_c;
                        br_target  <= {{2{1'b0}}, imm_i[DATA_W-3:0]};
                        state      <= S_BR;
                    end else begin
                        wb_warp_id   <= w_i;
                        wb_rd        <= rd_i;
                        wb_lane_mask <= m_i;
                        wb_wdata     <= wdata_c;
                        state        <= S_WB;
                    end
                end
            end
            // ----------------------------------------------------
            // S_WB / S_WBD / S_BR：各自通道保持至握手（§6.2）
            // ----------------------------------------------------
            S_WB: begin
                if (wb_fire) state <= S_WBD;
            end
            S_WBD: begin
                if (wbd_fire) state <= S_IDLE;
            end
            S_BR: begin
                if (br_fire) state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

    // ---------------- 载荷导出（vld 保持期内稳定，§4） ----------------
    assign ialu_rf_wb_warp_id     = wb_warp_id;
    assign ialu_rf_wb_rd          = wb_rd;
    assign ialu_rf_wb_lane_mask   = wb_lane_mask;
    assign ialu_rf_wb_wdata       = wb_wdata;
    assign ialu_sf_wbdone_warp_id = wbd_warp_id;
    assign ialu_sf_wbdone_rd      = wbd_rd;
    assign ialu_sf_br_warp_id     = br_warp_id;
    assign ialu_sf_br_taken       = br_taken;
    assign ialu_sf_br_target      = br_target;
    assign ialu_sf_br_brt_idx     = {BRT_IW{1'b0}};   // sf 按分支 pc 查 BRT（§1.3）

endmodule
