// =============================================================================
// easy_simt · ws — Warp Scheduler（SIMT 前端与调度合一，ma_spec §2）
//
// 每 warp 一份 PC、active mask、分化栈（深 4）；取指发起、定长译码、立即
// 数扩展、ld.param/csr 值构造；冒险检测与互锁（记分板，互锁不转发）；
// issue 分派（ialu/falu/lsu）；SIMT 分化控制（判定在 ialu，处置在本模块）；
// RUN_TO_DONE 单 warp 独占调度（ma_spec §2）；bar.sync 到达计数与凑齐
// 统一释放；块完成判定（全 warp ret 后发 block_done）。
//
// 结构（ws_spec §4/§5）：RUN_TO_DONE 下任一时刻至多一个 warp 在途，在途
// 上下文（指令/读口阶段/分支 pc/读数据）为单一组寄存器（ifw_q 所属），
// 每 warp 独立保有 PC/mask/分化栈/记分板/访存计数。每拍至多推进一个事件
// （授予→取指呈现→取指响应/译码→读呈现→读应答→发射呈现→wbdone/决议
// →释放），事件间经寄存器传递；与 cmodel ws_step 逐步对应、每步含 1~3
// 拍流水气泡，边界事务序列与 cmodel 完全同构。
//
// 模块补充配置口（ws_spec §2，bs_cfg_n 先例）：
//   ws_cfg_param0/1/2 —— ld.param k=0/1/2 值（in_base/out_base/N）；
//   ws_cfg_brt_*      —— BRT 装载（isa_spec §1.11：主机随程序装载、启动
//                        前完成；8 表项 CAM，按分支 pc 查重聚点）。
//
// 设计依据：ws/docs/ws_spec_v0.1.md；
//   端口命名与 intf_spec §2 一致，握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module ws #(
    parameter DATA_W   = 32,             // 数据/地址/载荷位宽（intf_spec §1.4）
    parameter NWARPS   = 4,              // warp 数/块（ma_spec §1.2）
    parameter NLANES   = 8,              // lane 数/warp（ma_spec §1.2）
    parameter REG_AW   = 5,              // 寄存器地址位宽（intf_spec §1.4）
    parameter OPCODE_W = 5,              // 操作码位宽（intf_spec §1.4）
    parameter BRT_IW   = 2,              // BRT 表项索引位宽（intf_spec §1.4）
    localparam WARP_IW = (NWARPS > 1) ? $clog2(NWARPS) : 1,
    localparam VEC_W   = NLANES * DATA_W,
    localparam DSP_D   = 4,              // 分化栈深（ma_spec §1.6）
    localparam BRT_N   = 8               // BRT 表项数（isa_spec §19，深度 8）
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- 模块补充配置（静态，启动前生效；ws_spec §2/§3） ----
    input  wire [DATA_W-1:0]     ws_cfg_param0,   // ld.param k=0（in_base）
    input  wire [DATA_W-1:0]     ws_cfg_param1,   // ld.param k=1（out_base）
    input  wire [DATA_W-1:0]     ws_cfg_param2,   // ld.param k=2（N）
    input  wire                  ws_cfg_brt_we,
    input  wire [BRT_IW:0]       ws_cfg_brt_idx,  // 0..7
    input  wire [DATA_W-1:0]     ws_cfg_brt_wpc,  // 分支 pc（键）
    input  wire [DATA_W-1:0]     ws_cfg_brt_wrpc, // 重聚 pc（值）

    // ---- bs_ws_launch：块启动 ----
    input  wire                  bs_ws_launch_vld,
    input  wire [DATA_W-1:0]     bs_ws_launch_block_idx,
    input  wire [DATA_W-1:0]     bs_ws_launch_n,
    input  wire [DATA_W-1:0]     bs_ws_launch_shbase,
    output wire                  ws_bs_launch_rdy,

    // ---- ws_bs_bdone：块完成 ----
    output reg                   ws_bs_bdone_vld,
    output wire [DATA_W-1:0]     ws_bs_bdone_block_idx,
    input  wire                  bs_ws_bdone_rdy,

    // ---- lsu_ws_stall：lsu 停顿上报（提示性状态同步） ----
    input  wire                  lsu_ws_stall_vld,
    input  wire [WARP_IW-1:0]    lsu_ws_stall_warp_id,
    input  wire [2:0]            lsu_ws_stall_reason,
    output wire                  ws_lsu_stall_rdy,

    // ---- ws_icache_req / icache_ws_rsp：取指 ----
    output reg                   ws_icache_req_vld,
    output wire [DATA_W-1:0]     ws_icache_req_pc,
    input  wire                  icache_ws_req_rdy,
    input  wire                  icache_ws_rsp_vld,
    input  wire [DATA_W-1:0]     icache_ws_rsp_inst,
    output wire                  ws_icache_rsp_rdy,

    // ---- ws_rf_rd / rf_ws_rddata：读口 ----
    output reg                   ws_rf_rd_vld,
    output wire [WARP_IW-1:0]    ws_rf_rd_warp_id,
    output wire [REG_AW-1:0]     ws_rf_rd_rs1,
    output wire [REG_AW-1:0]     ws_rf_rd_rs2,
    input  wire                  rf_ws_rd_rdy,
    input  wire                  rf_ws_rddata_vld,
    input  wire [VEC_W-1:0]      rf_ws_rddata_a,
    input  wire [VEC_W-1:0]      rf_ws_rddata_b,
    output wire                  ws_rf_rddata_rdy,

    // ---- ws_ialu_issue（含 LDP/CSRR 直通与 BR） ----
    output reg                   ws_ialu_issue_vld,
    output wire [OPCODE_W-1:0]   ws_ialu_issue_opcode,
    output wire [REG_AW-1:0]     ws_ialu_issue_rd,
    output wire [WARP_IW-1:0]    ws_ialu_issue_warp_id,
    output wire [NLANES-1:0]     ws_ialu_issue_lane_mask,
    output wire [DATA_W-1:0]     ws_ialu_issue_pc,
    output wire [DATA_W-1:0]     ws_ialu_issue_imm,
    output wire [VEC_W-1:0]      ws_ialu_issue_opa,
    output wire [VEC_W-1:0]      ws_ialu_issue_opb,
    output wire [VEC_W-1:0]      ws_ialu_issue_opc,
    input  wire                  ialu_ws_issue_rdy,

    // ---- ws_falu_issue ----
    output reg                   ws_falu_issue_vld,
    output wire [OPCODE_W-1:0]   ws_falu_issue_opcode,
    output wire [REG_AW-1:0]     ws_falu_issue_rd,
    output wire [WARP_IW-1:0]    ws_falu_issue_warp_id,
    output wire [NLANES-1:0]     ws_falu_issue_lane_mask,
    output wire [VEC_W-1:0]      ws_falu_issue_opa,
    output wire [VEC_W-1:0]      ws_falu_issue_opb,
    input  wire                  falu_ws_issue_rdy,

    // ---- ws_lsu_issue（含 shbase） ----
    output reg                   ws_lsu_issue_vld,
    output wire [OPCODE_W-1:0]   ws_lsu_issue_opcode,
    output wire [REG_AW-1:0]     ws_lsu_issue_rd,
    output wire [WARP_IW-1:0]    ws_lsu_issue_warp_id,
    output wire [NLANES-1:0]     ws_lsu_issue_lane_mask,
    output wire [VEC_W-1:0]      ws_lsu_issue_opa,
    output wire [VEC_W-1:0]      ws_lsu_issue_opb,
    output wire [DATA_W-1:0]     ws_lsu_issue_imm,
    output wire [DATA_W-1:0]     ws_lsu_issue_shbase,
    input  wire                  lsu_ws_issue_rdy,

    // ---- ialu_ws_br：分支决议（判定在 ialu，处置在本模块） ----
    input  wire                  ialu_ws_br_vld,
    input  wire [WARP_IW-1:0]    ialu_ws_br_warp_id,
    input  wire [NLANES-1:0]     ialu_ws_br_taken,
    input  wire [DATA_W-1:0]     ialu_ws_br_target,
    input  wire [BRT_IW-1:0]     ialu_ws_br_brt_idx,  // 恒 0（ws 按分支 pc 查表）
    output wire                  ws_ialu_br_rdy,

    // ---- {ialu,falu,lsu}_ws_wbdone：写回完成（清记分板） ----
    input  wire                  ialu_ws_wbdone_vld,
    input  wire [WARP_IW-1:0]    ialu_ws_wbdone_warp_id,
    input  wire [REG_AW-1:0]     ialu_ws_wbdone_rd,
    output wire                  ws_ialu_wbdone_rdy,
    input  wire                  falu_ws_wbdone_vld,
    input  wire [WARP_IW-1:0]    falu_ws_wbdone_warp_id,
    input  wire [REG_AW-1:0]     falu_ws_wbdone_rd,
    output wire                  ws_falu_wbdone_rdy,
    input  wire                  lsu_ws_wbdone_vld,
    input  wire [WARP_IW-1:0]    lsu_ws_wbdone_warp_id,
    input  wire [REG_AW-1:0]     lsu_ws_wbdone_rd,
    output wire                  ws_lsu_wbdone_rdy,

    // ---- 顶层：错误标志（锁存） ----
    output wire                  ws_top_err
);

    // ---------------- 操作码（isa_spec §1.8；内部 6 位口径同 cmodel） ----------------
    localparam [5:0] OP_IMAD = 6'h01, OP_IADD = 6'h02, OP_SHL  = 6'h03;
    localparam [5:0] OP_XOR  = 6'h04, OP_ORI  = 6'h05, OP_LUI  = 6'h06;
    localparam [5:0] OP_SETP = 6'h07, OP_FMUL = 6'h08, OP_FADD = 6'h09;
    localparam [5:0] OP_FNEG = 6'h0A, OP_LDG  = 6'h0B, OP_STG  = 6'h0C;
    localparam [5:0] OP_LDS  = 6'h0D, OP_STS  = 6'h0E, OP_LDP  = 6'h0F;
    localparam [5:0] OP_CSRR = 6'h10, OP_BR   = 6'h11, OP_JOIN = 6'h12;
    localparam [5:0] OP_BAR  = 6'h13, OP_RET  = 6'h14;

    // ---------------- 在途 warp 状态编码（ws_spec §5.1） ----------------
    localparam [3:0] W_FETCH = 4'd1,  W_HAZ   = 4'd2;
    localparam [3:0] W_RD1   = 4'd3,  W_RD2   = 4'd4,  W_ISSUE = 4'd5;
    localparam [3:0] W_EXEC  = 4'd6,  W_BR    = 4'd7;

    // 读口阶段（ws_spec §4.2；R1/R2 为待呈现编码）
    localparam [2:0] RP_NONE = 3'd0,  RP_P1 = 3'd1,  RP_P2 = 3'd2;
    localparam [2:0] RP_R1   = 3'd3,  RP_R2 = 3'd4;

    localparam [NLANES-1:0] FULL_MASK = {NLANES{1'b1}};

    // ---------------- 握手判据 ----------------
    wire launch_fire = bs_ws_launch_vld && ws_bs_launch_rdy;
    wire stall_fire  = lsu_ws_stall_vld;                    // rdy 恒 1
    wire irsp_fire   = icache_ws_rsp_vld;                   // rdy 恒 1
    wire rrsp_fire   = rf_ws_rddata_vld;                    // rdy 恒 1
    wire br_fire     = ialu_ws_br_vld;                      // rdy 恒 1
    wire wbd_lsu     = lsu_ws_wbdone_vld;                   // 优先级 lsu>ialu>falu
    wire wbd_ialu    = !wbd_lsu && ialu_ws_wbdone_vld;
    wire wbd_falu    = !wbd_lsu && !wbd_ialu && falu_ws_wbdone_vld;
    wire ireq_fire   = ws_icache_req_vld && icache_ws_req_rdy;
    wire rrd_fire    = ws_rf_rd_vld && rf_ws_rd_rdy;
    wire iss_a_fire  = ws_ialu_issue_vld && ialu_ws_issue_rdy;
    wire iss_f_fire  = ws_falu_issue_vld && falu_ws_issue_rdy;
    wire iss_l_fire  = ws_lsu_issue_vld && lsu_ws_issue_rdy;
    wire bdone_fire  = ws_bs_bdone_vld && bs_ws_bdone_rdy;

    // ---------------- 块级寄存器（ws_spec §4.1） ----------------
    reg                 launched_q, active_q, bdone_sent_q, err_q;
    reg  [DATA_W-1:0]   block_idx_q, shbase_q;
    reg  [NWARPS-1:0]   bar_arr_q, done_q;
    reg  [2:0]          bar_cnt_q;
    reg  [WARP_IW-1:0]  cur_warp_q;

    // ---------------- 每 warp 上下文（ws_spec §4.2） ----------------
    reg  [DATA_W-1:0]   pc_q     [NWARPS];
    reg  [NLANES-1:0]   mask_q   [NWARPS];
    reg  [DATA_W-1:0]   rpc_q    [NWARPS][DSP_D];
    reg  [NLANES-1:0]   dmask_q  [NWARPS][DSP_D];
    reg  [2:0]          dsp_q    [NWARPS];      // 0..4
    reg  [DATA_W-1:0]   sb_busy_q[NWARPS];      // 记分板 32 位
    reg  [2:0]          lsu_out_q[NWARPS];      // 未退休访存计数
    reg  [2:0]          lsur_q   [NWARPS];      // lsu 停顿锁存（R_NONE=0）

    // ---------------- 在途上下文（单一组，ws_spec §4.3） ----------------
    reg                 ifv_q;                  // 在途 warp 有效
    reg  [WARP_IW-1:0]  ifw_q;                  // 在途 warp id
    reg  [3:0]          istate_q;               // 在途状态（W_FETCH..W_BR）
    reg  [DATA_W-1:0]   inst_q;                 // 已取回指令
    reg  [2:0]          rdp_q;                  // 读口阶段（RP_*）
    reg                 bar_pend_q;             // 屏障等 LSU 排空
    reg  [DATA_W-1:0]   br_pc_q;                // 在途 BR 的 pc
    reg  [VEC_W-1:0]    rd_a_q, rd_b_q, rd_c_q; // 读口数据暂存
    reg                 fetch_vld_q;            // 取指在途（授予→响应）
    reg                 fetch_pend_q;           // 取指请求待呈现
    reg                 rd_vld_q;               // rf 读在途

    // ---------------- 输出载荷寄存（呈现期稳定） ----------------
    reg  [DATA_W-1:0]   ireq_pc_q;
    reg  [WARP_IW-1:0]  rrd_w_q;
    reg  [REG_AW-1:0]   rrd_s1_q, rrd_s2_q;
    reg  [5:0]          ia_op_q;
    reg  [REG_AW-1:0]   ia_rd_q;
    reg  [WARP_IW-1:0]  ia_w_q;
    reg  [NLANES-1:0]   ia_m_q;
    reg  [DATA_W-1:0]   ia_pc_q, ia_imm_q;
    reg  [VEC_W-1:0]    ia_a_q, ia_b_q, ia_c_q;
    reg  [5:0]          fa_op_q;
    reg  [REG_AW-1:0]   fa_rd_q;
    reg  [WARP_IW-1:0]  fa_w_q;
    reg  [NLANES-1:0]   fa_m_q;
    reg  [VEC_W-1:0]    fa_a_q, fa_b_q;
    reg  [5:0]          la_op_q;
    reg  [REG_AW-1:0]   la_rd_q;
    reg  [WARP_IW-1:0]  la_w_q;
    reg  [NLANES-1:0]   la_m_q;
    reg  [VEC_W-1:0]    la_a_q, la_b_q;
    reg  [DATA_W-1:0]   la_imm_q;
    reg  [DATA_W-1:0]   bdone_idx_q;

    // ---------------- BRT（8 表项 CAM，isa_spec §19） ----------------
    reg                  brt_v_q   [BRT_N];
    reg  [DATA_W-1:0]    brt_pc_q  [BRT_N];
    reg  [DATA_W-1:0]    brt_rpc_q [BRT_N];

    // ---------------- 输出赋值（rdy 常量 + 载荷寄存导出） ----------------
    assign ws_bs_launch_rdy     = !launched_q && !active_q;
    assign ws_lsu_stall_rdy     = 1'b1;
    assign ws_icache_rsp_rdy    = 1'b1;
    assign ws_rf_rddata_rdy     = 1'b1;
    assign ws_ialu_br_rdy       = 1'b1;
    assign ws_ialu_wbdone_rdy   = 1'b1;
    assign ws_falu_wbdone_rdy   = 1'b1;
    assign ws_lsu_wbdone_rdy    = 1'b1;
    assign ws_top_err           = err_q;
    assign ws_icache_req_pc     = ireq_pc_q;
    assign ws_rf_rd_warp_id     = rrd_w_q;
    assign ws_rf_rd_rs1         = rrd_s1_q;
    assign ws_rf_rd_rs2         = rrd_s2_q;
    assign ws_ialu_issue_opcode = ia_op_q[OPCODE_W-1:0];
    assign ws_ialu_issue_rd     = ia_rd_q;
    assign ws_ialu_issue_warp_id = ia_w_q;
    assign ws_ialu_issue_lane_mask = ia_m_q;
    assign ws_ialu_issue_pc     = ia_pc_q;
    assign ws_ialu_issue_imm    = ia_imm_q;
    assign ws_ialu_issue_opa    = ia_a_q;
    assign ws_ialu_issue_opb    = ia_b_q;
    assign ws_ialu_issue_opc    = ia_c_q;
    assign ws_falu_issue_opcode = fa_op_q[OPCODE_W-1:0];
    assign ws_falu_issue_rd     = fa_rd_q;
    assign ws_falu_issue_warp_id = fa_w_q;
    assign ws_falu_issue_lane_mask = fa_m_q;
    assign ws_falu_issue_opa    = fa_a_q;
    assign ws_falu_issue_opb    = fa_b_q;
    assign ws_lsu_issue_opcode  = la_op_q[OPCODE_W-1:0];
    assign ws_lsu_issue_rd      = la_rd_q;
    assign ws_lsu_issue_warp_id = la_w_q;
    assign ws_lsu_issue_lane_mask = la_m_q;
    assign ws_lsu_issue_opa     = la_a_q;
    assign ws_lsu_issue_opb     = la_b_q;
    assign ws_lsu_issue_imm     = la_imm_q;
    assign ws_lsu_issue_shbase  = shbase_q;
    assign ws_bs_bdone_block_idx = bdone_idx_q;

    // ---------------- 译码组合（ws_spec §5.3，逐域镜像 cmodel decode） ----------------
    wire [31:0] dec_inst_w = irsp_fire ? icache_ws_rsp_inst : inst_q;
    wire [31:0] pc0_w      = pc_q[ifw_q];

    reg  [5:0]  d_op;
    reg  [4:0]  d_rd, d_ra, d_rb, d_rc, d_pd, d_psel;
    reg         d_u, d_neg, d_fmt;
    reg  [2:0]  d_cond;
    reg  [31:0] d_imm;      // 扩展立即数 / BR/JOIN 目标
    reg         d_ok;

    always @* begin
        d_op  = dec_inst_w[31:26];
        d_rd  = 5'd0; d_ra = 5'd0; d_rb = 5'd0; d_rc = 5'd0;
        d_pd  = 5'd0; d_psel = 5'd0;
        d_u   = 1'b0; d_neg = 1'b0; d_fmt = 1'b0; d_cond = 3'd0;
        d_imm = 32'd0; d_ok = 1'b1;
        case (d_op)
        OP_IMAD: begin
            d_rd = dec_inst_w[25:21]; d_ra = dec_inst_w[20:16];
            d_rb = dec_inst_w[15:11]; d_rc = dec_inst_w[10:6];
        end
        OP_IADD, OP_SHL: begin
            d_rd = dec_inst_w[25:21]; d_ra = dec_inst_w[20:16];
            if (dec_inst_w[15])
                d_imm = (d_op == OP_IADD)
                      ? {{17{dec_inst_w[14]}}, dec_inst_w[14:0]}
                      : {27'd0, dec_inst_w[4:0]};
            else
                d_rb = dec_inst_w[15:11];
        end
        OP_XOR, OP_FMUL, OP_FADD: begin
            d_rd = dec_inst_w[25:21]; d_ra = dec_inst_w[20:16];
            d_rb = dec_inst_w[15:11];
        end
        OP_FNEG: begin
            d_rd = dec_inst_w[25:21]; d_ra = dec_inst_w[20:16];
        end
        OP_ORI: begin
            d_rd = dec_inst_w[25:21]; d_ra = dec_inst_w[20:16];
            d_imm = {16'd0, dec_inst_w[15:0]};
        end
        OP_LUI: begin
            d_rd = dec_inst_w[25:21];
            d_imm = {dec_inst_w[20:1], 12'd0};
        end
        OP_SETP: begin
            d_pd   = {3'd0, dec_inst_w[25:24]};
            d_ra   = dec_inst_w[22:18];
            d_rb   = dec_inst_w[17:13];
            d_fmt  = dec_inst_w[12];
            d_cond = dec_inst_w[11:9];
        end
        OP_LDG, OP_STG: begin
            d_rd = dec_inst_w[25:21]; d_ra = dec_inst_w[20:16];
            d_rb = dec_inst_w[15:11];
        end
        OP_LDS, OP_STS: begin
            d_rd = dec_inst_w[25:21]; d_rb = dec_inst_w[15:11];
        end
        OP_LDP, OP_CSRR: begin
            d_rd = dec_inst_w[25:21]; d_ra = dec_inst_w[20:16];
            if (d_ra > 5'd2) d_ok = 1'b0;
        end
        OP_BR, OP_JOIN: begin
            d_psel = {3'd0, dec_inst_w[25:24]};
            d_u    = dec_inst_w[23];
            d_neg  = dec_inst_w[22];
            d_imm  = pc0_w + {{10{dec_inst_w[21]}}, dec_inst_w[21:0]};
        end
        OP_BAR, OP_RET: ;
        default: d_ok = 1'b0;
        endcase
    end

    // 操作分类
    wire is_ialu_cls_w = (d_op == OP_IMAD) || (d_op == OP_IADD) ||
                         (d_op == OP_SHL)  || (d_op == OP_XOR)  ||
                         (d_op == OP_ORI)  || (d_op == OP_LUI)  ||
                         (d_op == OP_SETP) || (d_op == OP_LDP)  ||
                         (d_op == OP_CSRR);
    wire is_falu_cls_w = (d_op == OP_FMUL) || (d_op == OP_FADD) ||
                         (d_op == OP_FNEG);
    wire is_lsu_cls_w  = (d_op == OP_LDG)  || (d_op == OP_STG)  ||
                         (d_op == OP_LDS)  || (d_op == OP_STS);
    wire is_br_w   = (d_op == OP_BR);
    wire is_join_w = (d_op == OP_JOIN);
    wire is_bar_w  = (d_op == OP_BAR);
    wire is_ret_w  = (d_op == OP_RET);

    // 冒险位集（镜像 cmodel haz_mask：源 | 目的）
    function automatic [31:0] haz_mask_f(input [5:0] op,
                                         input [4:0] rd, ra, rb, rc);
        reg [31:0] m;
        begin
            m = 32'd0;
            case (op)
            OP_IMAD: m = (32'd1 << ra) | (32'd1 << rb) | (32'd1 << rc);
            OP_IADD, OP_SHL: begin
                m = 32'd1 << ra;
                if (rb != 5'd0) m = m | (32'd1 << rb);
            end
            OP_XOR, OP_FMUL, OP_FADD, OP_SETP:
                m = (32'd1 << ra) | (32'd1 << rb);
            OP_FNEG, OP_ORI:
                m = 32'd1 << ra;
            OP_LDG, OP_STG:
                m = (32'd1 << ra) | (32'd1 << rb);
            OP_LDS, OP_STS:
                m = 32'd1 << rb;
            default: ;
            endcase
            case (op)
            OP_IMAD, OP_IADD, OP_SHL, OP_XOR, OP_ORI, OP_LUI,
            OP_FMUL, OP_FADD, OP_FNEG, OP_LDP, OP_CSRR,
            OP_LDG, OP_LDS, OP_STG, OP_STS:
                m = m | (32'd1 << rd);   // STG/STS 的 rt 亦为源，已按位计入
            default: ;
            endcase
            haz_mask_f = m;
        end
    endfunction
    wire [31:0] haz_w = haz_mask_f(d_op, d_rd, d_ra, d_rb, d_rc);

    // 读口规划（镜像 cmodel read_plan）
    reg [4:0] rp_p1_0_w, rp_p1_1_w, rp_p2_0_w, rp_p2_1_w;
    reg [2:0] rp_ph_w;
    always @* begin
        rp_p1_0_w = 5'd0; rp_p1_1_w = 5'd0;
        rp_p2_0_w = 5'd0; rp_p2_1_w = 5'd0; rp_ph_w = 3'd0;
        case (d_op)
        OP_IMAD: begin
            rp_p1_0_w = d_ra; rp_p1_1_w = d_rb; rp_p2_0_w = d_rc;
            rp_ph_w = 3'd2;
        end
        OP_IADD, OP_SHL, OP_XOR, OP_FMUL, OP_FADD, OP_SETP, OP_LDG: begin
            rp_p1_0_w = d_ra; rp_p1_1_w = d_rb;
            rp_ph_w = 3'd1;
        end
        OP_FNEG, OP_ORI: begin
            rp_p1_0_w = d_ra;
            rp_ph_w = 3'd1;
        end
        OP_LDS: begin
            rp_p1_0_w = d_rb;
            rp_ph_w = 3'd1;
        end
        OP_STG: begin
            rp_p1_0_w = d_ra; rp_p1_1_w = d_rb; rp_p2_0_w = d_rd;
            rp_ph_w = 3'd2;
        end
        OP_STS: begin
            rp_p1_0_w = d_rb; rp_p2_0_w = d_rd;
            rp_ph_w = 3'd2;
        end
        default: ;
        endcase
    end

    // 均匀基址判定（偏差 C2：LDG/STG 基址须为活动 lane 同值）
    function automatic [32:0] uni_f(input [VEC_W-1:0] v, input [NLANES-1:0] m);
        integer l;
        reg seen, conf;
        reg [31:0] x;
        begin
            seen = 1'b0; conf = 1'b0; x = 32'd0;
            for (l = 0; l < NLANES; l = l + 1)
                if (m[l]) begin
                    if (!seen) begin seen = 1'b1; x = v[l*32 +: 32]; end
                    else if (v[l*32 +: 32] != x) conf = 1'b1;
                end
            uni_f = {seen & ~conf, x};
        end
    endfunction
    wire [32:0] uni_a_w = uni_f(rd_a_q, mask_q[ifw_q]);
    wire [32:0] uni_b_w = uni_f(rd_b_q, mask_q[ifw_q]);

    // BRT 查表（按在途 BR 的 pc）
    reg         brt_hit_w;
    reg  [31:0] brt_R_w;
    integer bi;
    always @* begin
        brt_hit_w = 1'b0;
        brt_R_w   = 32'd0;
        for (bi = 0; bi < BRT_N; bi = bi + 1)
            if (brt_v_q[bi] && (brt_pc_q[bi] == br_pc_q)) begin
                brt_hit_w = 1'b1;
                brt_R_w   = brt_rpc_q[bi];
            end
    end

    // 调度推进函数（RUN_TO_DONE，镜像 cmodel；自 cur 起环扫不含自身）
    function automatic [WARP_IW-1:0] adv_done_f(input [WARP_IW-1:0] c);
        integer i;
        reg f;
        reg [WARP_IW-1:0] w2;
        begin
            f = 1'b0; adv_done_f = c;
            for (i = 1; i < NWARPS; i = i + 1) begin
                w2 = c + i;
                if (!f && !done_q[w2]) begin
                    adv_done_f = w2;
                    f = 1'b1;
                end
            end
        end
    endfunction

    function automatic [WARP_IW-1:0] adv_bar_f(input [WARP_IW-1:0] c);
        integer i;
        reg f;
        reg [WARP_IW-1:0] w2;
        begin
            f = 1'b0; adv_bar_f = c;
            for (i = 1; i < NWARPS; i = i + 1) begin
                w2 = c + i;
                if (!f && !done_q[w2] && !bar_arr_q[w2]) begin
                    adv_bar_f = w2;
                    f = 1'b1;
                end
            end
        end
    endfunction

    function automatic [WARP_IW-1:0] min_not_done_f();
        integer i;
        reg f;
        begin
            f = 1'b0; min_not_done_f = {WARP_IW{1'b0}};
            for (i = 0; i < NWARPS; i = i + 1)
                if (!f && !done_q[i]) begin
                    min_not_done_f = i;
                    f = 1'b1;
                end
        end
    endfunction

    // 授予用的停顿有效值（本拍 stall 锁存即时可见，镜像 cmodel 步序）
    wire [2:0] lsur_cur_w =
        (stall_fire && (lsu_ws_stall_warp_id == cur_warp_q))
        ? lsu_ws_stall_reason : lsur_q[cur_warp_q];

    integer k, li;

    // ---------------- 主时序（每拍至多一个事件，ws_spec §5） ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            launched_q   <= 1'b0;
            active_q     <= 1'b0;
            bdone_sent_q <= 1'b0;
            err_q        <= 1'b0;
            block_idx_q  <= {DATA_W{1'b0}};
            shbase_q     <= {DATA_W{1'b0}};
            bar_arr_q    <= {NWARPS{1'b0}};
            done_q       <= {NWARPS{1'b0}};
            bar_cnt_q    <= 3'd0;
            cur_warp_q   <= {WARP_IW{1'b0}};
            ifv_q        <= 1'b0;
            ifw_q        <= {WARP_IW{1'b0}};
            istate_q     <= W_FETCH;
            inst_q       <= {DATA_W{1'b0}};
            rdp_q        <= RP_NONE;
            bar_pend_q   <= 1'b0;
            br_pc_q      <= {DATA_W{1'b0}};
            rd_a_q       <= {VEC_W{1'b0}};
            rd_b_q       <= {VEC_W{1'b0}};
            rd_c_q       <= {VEC_W{1'b0}};
            fetch_vld_q  <= 1'b0;
            fetch_pend_q <= 1'b0;
            rd_vld_q     <= 1'b0;
            ws_icache_req_vld  <= 1'b0;
            ws_rf_rd_vld       <= 1'b0;
            ws_ialu_issue_vld  <= 1'b0;
            ws_falu_issue_vld  <= 1'b0;
            ws_lsu_issue_vld   <= 1'b0;
            ws_bs_bdone_vld    <= 1'b0;
            ireq_pc_q    <= {DATA_W{1'b0}};
            rrd_w_q      <= {WARP_IW{1'b0}};
            rrd_s1_q     <= {REG_AW{1'b0}};
            rrd_s2_q     <= {REG_AW{1'b0}};
            ia_op_q      <= 6'd0;  ia_rd_q <= 5'd0;
            ia_w_q       <= {WARP_IW{1'b0}};  ia_m_q <= {NLANES{1'b0}};
            ia_pc_q      <= {DATA_W{1'b0}};   ia_imm_q <= {DATA_W{1'b0}};
            ia_a_q       <= {VEC_W{1'b0}};  ia_b_q <= {VEC_W{1'b0}};
            ia_c_q       <= {VEC_W{1'b0}};
            fa_op_q      <= 6'd0;  fa_rd_q <= 5'd0;
            fa_w_q       <= {WARP_IW{1'b0}};  fa_m_q <= {NLANES{1'b0}};
            fa_a_q       <= {VEC_W{1'b0}};  fa_b_q <= {VEC_W{1'b0}};
            la_op_q      <= 6'd0;  la_rd_q <= 5'd0;
            la_w_q       <= {WARP_IW{1'b0}};  la_m_q <= {NLANES{1'b0}};
            la_a_q       <= {VEC_W{1'b0}};  la_b_q <= {VEC_W{1'b0}};
            la_imm_q     <= {DATA_W{1'b0}};
            bdone_idx_q  <= {DATA_W{1'b0}};
            for (k = 0; k < NWARPS; k = k + 1) begin
                pc_q[k]      <= {DATA_W{1'b0}};
                mask_q[k]    <= {NLANES{1'b0}};
                dsp_q[k]     <= 3'd0;
                sb_busy_q[k] <= {DATA_W{1'b0}};
                lsu_out_q[k] <= 3'd0;
                lsur_q[k]    <= 3'd0;
                for (li = 0; li < DSP_D; li = li + 1) begin
                    rpc_q[k][li]   <= {DATA_W{1'b0}};
                    dmask_q[k][li] <= {NLANES{1'b0}};
                end
            end
            for (k = 0; k < BRT_N; k = k + 1) begin
                brt_v_q[k]   <= 1'b0;
                brt_pc_q[k]  <= {DATA_W{1'b0}};
                brt_rpc_q[k] <= {DATA_W{1'b0}};
            end
        end else begin
            // ---- 默认：握手完成撤 vld ----
            if (ireq_fire)  ws_icache_req_vld <= 1'b0;
            if (rrd_fire)   ws_rf_rd_vld      <= 1'b0;
            if (iss_a_fire) ws_ialu_issue_vld <= 1'b0;
            if (iss_f_fire) ws_falu_issue_vld <= 1'b0;
            if (iss_l_fire) ws_lsu_issue_vld  <= 1'b0;
            if (bdone_fire) ws_bs_bdone_vld   <= 1'b0;

            // ---- BRT 配置装载（isa_spec §1.11：启动前完成） ----
            if (ws_cfg_brt_we) begin
                brt_v_q[ws_cfg_brt_idx]   <= 1'b1;
                brt_pc_q[ws_cfg_brt_idx]  <= ws_cfg_brt_wpc;
                brt_rpc_q[ws_cfg_brt_idx] <= ws_cfg_brt_wrpc;
            end

            if (launch_fire) begin
                // ================ 1. 块启动（镜像 cmodel launch 初始化） ================
                launched_q   <= 1'b1;
                active_q     <= 1'b1;
                block_idx_q  <= bs_ws_launch_block_idx;
                shbase_q     <= bs_ws_launch_shbase;
                ifv_q        <= 1'b0;
                fetch_vld_q  <= 1'b0;
                fetch_pend_q <= 1'b0;
                rd_vld_q     <= 1'b0;
                bar_pend_q   <= 1'b0;
                cur_warp_q   <= {WARP_IW{1'b0}};
                bar_arr_q    <= {NWARPS{1'b0}};
                bar_cnt_q    <= 3'd0;
                done_q       <= {NWARPS{1'b0}};
                bdone_sent_q <= 1'b0;
                for (k = 0; k < NWARPS; k = k + 1) begin
                    pc_q[k]      <= {DATA_W{1'b0}};
                    mask_q[k]    <= FULL_MASK;
                    dsp_q[k]     <= 3'd0;
                    sb_busy_q[k] <= {DATA_W{1'b0}};
                    lsu_out_q[k] <= 3'd0;
                    lsur_q[k]    <= 3'd0;
                end
            end else begin
                // ---- 2. lsu 停顿锁存（提示性状态同步） ----
                if (stall_fire)
                    lsur_q[lsu_ws_stall_warp_id] <= lsu_ws_stall_reason;

                if (irsp_fire && fetch_vld_q) begin
                    // ================ 3. 取指响应 → 译码（ws_spec §5.3） ================
                    inst_q      <= icache_ws_rsp_inst;
                    fetch_vld_q <= 1'b0;
                    rdp_q       <= RP_NONE;
                    if (!d_ok)
                        err_q <= 1'b1;
                    else if (is_ret_w) begin
                        if (dsp_q[ifw_q] != 3'd0)
                            err_q <= 1'b1;
                        else begin
                            pc_q[ifw_q]  <= pc0_w + 32'd1;
                            done_q[ifw_q] <= 1'b1;
                            ifv_q      <= 1'b0;
                            cur_warp_q <= adv_done_f(cur_warp_q);
                        end
                    end else if (is_bar_w) begin
                        if (mask_q[ifw_q] != FULL_MASK)
                            err_q <= 1'b1;
                        else begin
                            pc_q[ifw_q] <= pc0_w + 32'd1;
                            if (lsu_out_q[ifw_q] == 3'd0) begin
                                // 到达：计数 + 推进；凑齐则统一释放（ws_spec §6）
                                bar_arr_q[ifw_q] <= 1'b1;
                                cur_warp_q <= adv_bar_f(cur_warp_q);
                                if (bar_cnt_q + 3'd1 == NWARPS) begin
                                    bar_arr_q <= {NWARPS{1'b0}};
                                    bar_cnt_q <= 3'd0;
                                    cur_warp_q <= min_not_done_f();
                                end else
                                    bar_cnt_q <= bar_cnt_q + 3'd1;
                                ifv_q <= 1'b0;
                            end else begin
                                bar_pend_q <= 1'b1;
                                istate_q   <= W_HAZ;
                            end
                        end
                    end else if (is_join_w) begin
                        // JOIN 本地处置（分化栈，ws_spec §5.3）
                        if (dsp_q[ifw_q] != 3'd0) begin
                            if (rpc_q[ifw_q][dsp_q[ifw_q] - 3'd1] == d_imm) begin
                                mask_q[ifw_q] <= dmask_q[ifw_q][dsp_q[ifw_q] - 3'd1];
                                dsp_q[ifw_q]  <= dsp_q[ifw_q] - 3'd1;
                                pc_q[ifw_q]   <= d_imm;
                            end else begin
                                pc_q[ifw_q]   <= rpc_q[ifw_q][dsp_q[ifw_q] - 3'd1];
                                mask_q[ifw_q] <= dmask_q[ifw_q][dsp_q[ifw_q] - 3'd1];
                                dsp_q[ifw_q]  <= dsp_q[ifw_q] - 3'd1;
                            end
                        end else
                            pc_q[ifw_q] <= d_imm;
                        ifv_q <= 1'b0;
                    end else if (is_br_w) begin
                        istate_q <= W_ISSUE;   // 走 ialu 发射（无读口）
                    end else begin
                        // exec 类：记分板互锁（互锁不转发，ws_spec §5.3）
                        if ((sb_busy_q[ifw_q] & haz_w) != 32'd0) begin
                            istate_q <= W_HAZ;
                            rdp_q    <= RP_NONE;
                        end else if (rp_ph_w == 3'd0) begin
                            istate_q <= W_ISSUE;
                        end else begin
                            istate_q <= W_HAZ;
                            rdp_q    <= RP_R1;   // 待呈现第一阶段读
                        end
                    end
                end else if (rrsp_fire && rd_vld_q) begin
                    // ================ 4. rf 读应答（ws_spec §5.2） ================
                    if ((istate_q == W_RD1) && (rdp_q == RP_P1)) begin
                        rd_a_q   <= rf_ws_rddata_a;
                        rd_b_q   <= rf_ws_rddata_b;
                        rd_vld_q <= 1'b0;
                        if (rp_ph_w == 3'd2) begin
                            istate_q <= W_HAZ;
                            rdp_q    <= RP_R2;  // 待呈现第二阶段读
                        end else
                            istate_q <= W_ISSUE;
                    end else if ((istate_q == W_RD2) && (rdp_q == RP_P2)) begin
                        rd_c_q   <= rf_ws_rddata_a;
                        rd_vld_q <= 1'b0;
                        istate_q <= W_ISSUE;
                    end
                end else if (br_fire) begin
                    // ================ 5. 分支决议处置（ws_spec §5.5） ================
                    if ((ialu_ws_br_taken == {NLANES{1'b0}}) ||
                        ((mask_q[ialu_ws_br_warp_id]
                          & ~ialu_ws_br_taken) == {NLANES{1'b0}})) begin
                        // 均匀分支：不压栈
                        pc_q[ialu_ws_br_warp_id] <= (|ialu_ws_br_taken)
                            ? ialu_ws_br_target
                            : br_pc_q + 32'd1;
                    end else begin
                        // 分化：BRT 查表（表项缺失为架构错误）
                        if (!brt_hit_w)
                            err_q <= 1'b1;
                        else if (ialu_ws_br_target == brt_R_w) begin
                            // 单侧跳过型：压 (R, mask)，走顺序流
                            if (dsp_q[ialu_ws_br_warp_id] >= DSP_D)
                                err_q <= 1'b1;
                            else begin
                                rpc_q[ialu_ws_br_warp_id][dsp_q[ialu_ws_br_warp_id]]
                                    <= brt_R_w;
                                dmask_q[ialu_ws_br_warp_id][dsp_q[ialu_ws_br_warp_id]]
                                    <= mask_q[ialu_ws_br_warp_id];
                                dsp_q[ialu_ws_br_warp_id]
                                    <= dsp_q[ialu_ws_br_warp_id] + 3'd1;
                                mask_q[ialu_ws_br_warp_id]
                                    <= mask_q[ialu_ws_br_warp_id]
                                       & ~ialu_ws_br_taken;
                                pc_q[ialu_ws_br_warp_id] <= br_pc_q + 32'd1;
                            end
                        end else begin
                            // 双侧型：压 (R,mask)+(pc+1,nt)，先走 taken
                            if (dsp_q[ialu_ws_br_warp_id] + 3'd2 > DSP_D)
                                err_q <= 1'b1;
                            else begin
                                rpc_q[ialu_ws_br_warp_id][dsp_q[ialu_ws_br_warp_id]]
                                    <= brt_R_w;
                                dmask_q[ialu_ws_br_warp_id][dsp_q[ialu_ws_br_warp_id]]
                                    <= mask_q[ialu_ws_br_warp_id];
                                rpc_q[ialu_ws_br_warp_id][dsp_q[ialu_ws_br_warp_id] + 3'd1]
                                    <= br_pc_q + 32'd1;
                                dmask_q[ialu_ws_br_warp_id][dsp_q[ialu_ws_br_warp_id] + 3'd1]
                                    <= mask_q[ialu_ws_br_warp_id]
                                       & ~ialu_ws_br_taken;
                                dsp_q[ialu_ws_br_warp_id]
                                    <= dsp_q[ialu_ws_br_warp_id] + 3'd2;
                                mask_q[ialu_ws_br_warp_id] <= ialu_ws_br_taken;
                                pc_q[ialu_ws_br_warp_id] <= ialu_ws_br_target;
                            end
                        end
                    end
                    // 在途 BR 完成（决议即前端职责完成）
                    if (ifv_q && (ifw_q == ialu_ws_br_warp_id)
                        && (istate_q == W_BR))
                        ifv_q <= 1'b0;
                end else if (wbd_lsu || wbd_ialu || wbd_falu) begin
                    // ================ 6. 写回完成（清记分板，优先级 lsu>ialu>falu） ================
                    if (wbd_lsu) begin
                        if (lsu_ws_wbdone_rd != 5'd0)
                            sb_busy_q[lsu_ws_wbdone_warp_id]
                                <= sb_busy_q[lsu_ws_wbdone_warp_id]
                                   & ~(32'd1 << lsu_ws_wbdone_rd);
                        if (lsu_out_q[lsu_ws_wbdone_warp_id] != 3'd0)
                            lsu_out_q[lsu_ws_wbdone_warp_id]
                                <= lsu_out_q[lsu_ws_wbdone_warp_id] - 3'd1;
                        if (ifv_q && (ifw_q == lsu_ws_wbdone_warp_id)
                            && (istate_q == W_EXEC))
                            ifv_q <= 1'b0;
                    end else if (wbd_ialu) begin
                        if (ialu_ws_wbdone_rd != 5'd0)
                            sb_busy_q[ialu_ws_wbdone_warp_id]
                                <= sb_busy_q[ialu_ws_wbdone_warp_id]
                                   & ~(32'd1 << ialu_ws_wbdone_rd);
                        if (ifv_q && (ifw_q == ialu_ws_wbdone_warp_id)
                            && (istate_q == W_EXEC))
                            ifv_q <= 1'b0;
                    end else begin
                        if (falu_ws_wbdone_rd != 5'd0)
                            sb_busy_q[falu_ws_wbdone_warp_id]
                                <= sb_busy_q[falu_ws_wbdone_warp_id]
                                   & ~(32'd1 << falu_ws_wbdone_rd);
                        if (ifv_q && (ifw_q == falu_ws_wbdone_warp_id)
                            && (istate_q == W_EXEC))
                            ifv_q <= 1'b0;
                    end
                end else if (launched_q && !ifv_q && !fetch_vld_q
                             && !fetch_pend_q && !rd_vld_q
                             && !done_q[cur_warp_q]
                             && !bar_arr_q[cur_warp_q]
                             && (lsur_cur_w == 3'd0)) begin
                    // ================ 7. 调度授予（RUN_TO_DONE 单 warp 独占，ws_spec §6） ================
                    ifv_q        <= 1'b1;
                    ifw_q        <= cur_warp_q;
                    istate_q     <= W_FETCH;
                    fetch_vld_q  <= 1'b1;
                    fetch_pend_q <= 1'b1;
                end else if (ifv_q) begin
                    // ================ 8. 在途推进（呈现 / 重试，ws_spec §5.2/§5.4） ================
                    case (istate_q)
                    W_FETCH: begin
                        // 取指请求呈现（单在途；通道空闲时）
                        if (fetch_pend_q && !ws_icache_req_vld) begin
                            ws_icache_req_vld <= 1'b1;
                            ireq_pc_q         <= pc_q[ifw_q];
                            fetch_pend_q      <= 1'b0;
                        end
                    end
                    W_HAZ: begin
                        if (bar_pend_q) begin
                            // 屏障：等 LSU 排空，排空即到达（回合结束点）
                            if (lsu_out_q[ifw_q] == 3'd0) begin
                                bar_arr_q[ifw_q] <= 1'b1;
                                cur_warp_q <= adv_bar_f(cur_warp_q);
                                if (bar_cnt_q + 3'd1 == NWARPS) begin
                                    bar_arr_q <= {NWARPS{1'b0}};
                                    bar_cnt_q <= 3'd0;
                                    cur_warp_q <= min_not_done_f();
                                end else
                                    bar_cnt_q <= bar_cnt_q + 3'd1;
                                bar_pend_q <= 1'b0;
                                ifv_q      <= 1'b0;
                            end
                        end else if (rdp_q == RP_R1) begin
                            // 第一阶段读呈现
                            if (!ws_rf_rd_vld) begin
                                ws_rf_rd_vld <= 1'b1;
                                rrd_w_q  <= ifw_q;
                                rrd_s1_q <= rp_p1_0_w;
                                rrd_s2_q <= rp_p1_1_w;
                                rd_vld_q <= 1'b1;
                                rdp_q    <= RP_P1;
                                istate_q <= W_RD1;
                            end
                        end else if (rdp_q == RP_R2) begin
                            // 第二阶段读呈现
                            if (!ws_rf_rd_vld) begin
                                ws_rf_rd_vld <= 1'b1;
                                rrd_w_q  <= ifw_q;
                                rrd_s1_q <= rp_p2_0_w;
                                rrd_s2_q <= rp_p2_1_w;
                                rd_vld_q <= 1'b1;
                                rdp_q    <= RP_P2;
                                istate_q <= W_RD2;
                            end
                        end else begin
                            // RP_NONE：记分板恢复后重译码（互锁重试）
                            if (!d_ok)
                                err_q <= 1'b1;
                            else if ((sb_busy_q[ifw_q] & haz_w) == 32'd0) begin
                                if (rp_ph_w == 3'd0)
                                    istate_q <= W_ISSUE;
                                else
                                    rdp_q <= RP_R1;
                            end
                            // 冒险未清：保持 W_HAZ
                        end
                    end
                    W_ISSUE: begin
                        // 发射呈现（通道空闲时；目标在途由顶层独占保证，
                        // 通道忙为结构兜底，ws_spec §5.4）
                        if (is_br_w) begin
                            if (!ws_ialu_issue_vld) begin
                                ws_ialu_issue_vld <= 1'b1;
                                ia_op_q  <= OP_BR;
                                ia_rd_q  <= d_psel;
                                ia_w_q   <= ifw_q;
                                ia_m_q   <= mask_q[ifw_q];
                                ia_pc_q  <= pc0_w;
                                ia_imm_q <= {d_u, d_neg, d_imm[29:0]};
                                ia_a_q   <= {VEC_W{1'b0}};
                                ia_b_q   <= {VEC_W{1'b0}};
                                ia_c_q   <= {VEC_W{1'b0}};
                                br_pc_q  <= pc0_w;
                                istate_q <= W_BR;
                            end
                        end else if (is_ialu_cls_w) begin
                            if (!ws_ialu_issue_vld) begin
                                ws_ialu_issue_vld <= 1'b1;
                                ia_op_q <= d_op;
                                ia_rd_q <= (d_op == OP_SETP) ? d_pd : d_rd;
                                ia_w_q  <= ifw_q;
                                ia_m_q  <= mask_q[ifw_q];
                                ia_pc_q <= pc0_w;
                                ia_imm_q <= (d_op == OP_SETP)
                                            ? {28'd0, d_fmt, d_cond} : 32'd0;
                                // opa/opb/opc 按操作码归一化（ws_spec §5.4）
                                if (d_op == OP_IMAD) begin
                                    ia_a_q <= rd_a_q;
                                    ia_b_q <= rd_b_q;
                                    ia_c_q <= rd_c_q;
                                end else if ((d_op == OP_IADD)
                                             || (d_op == OP_SHL)
                                             || (d_op == OP_XOR)) begin
                                    ia_a_q <= rd_a_q;
                                    ia_b_q <= (d_rb != 5'd0)
                                              ? rd_b_q : {NLANES{d_imm}};
                                    ia_c_q <= {VEC_W{1'b0}};
                                end else if (d_op == OP_ORI) begin
                                    ia_a_q <= rd_a_q;
                                    ia_b_q <= {NLANES{d_imm}};
                                    ia_c_q <= {VEC_W{1'b0}};
                                end else if (d_op == OP_LUI) begin
                                    ia_a_q <= {NLANES{d_imm}};
                                    ia_b_q <= {VEC_W{1'b0}};
                                    ia_c_q <= {VEC_W{1'b0}};
                                end else if (d_op == OP_SETP) begin
                                    ia_a_q <= rd_a_q;
                                    ia_b_q <= rd_b_q;
                                    ia_c_q <= {VEC_W{1'b0}};
                                end else if (d_op == OP_LDP) begin
                                    ia_a_q <= (d_ra == 5'd0)
                                              ? {NLANES{ws_cfg_param0}}
                                              : (d_ra == 5'd1)
                                              ? {NLANES{ws_cfg_param1}}
                                              : {NLANES{ws_cfg_param2}};
                                    ia_b_q <= {VEC_W{1'b0}};
                                    ia_c_q <= {VEC_W{1'b0}};
                                end else begin
                                    // CSRR：逐 lane 构造 lane id / 线程总数 / blockIdx
                                    for (li = 0; li < NLANES; li = li + 1)
                                        ia_a_q[li*32 +: 32] <=
                                            (d_ra == 5'd0)
                                            ? (ifw_q * NLANES + li)
                                            : (d_ra == 5'd1)
                                            ? (NWARPS * NLANES)
                                            : block_idx_q;
                                    ia_b_q <= {VEC_W{1'b0}};
                                    ia_c_q <= {VEC_W{1'b0}};
                                end
                                // 记分板登记 + PC 推进（exec 类公共，BR 除外）
                                if ((d_op != OP_SETP) && (d_rd != 5'd0))
                                    sb_busy_q[ifw_q] <= sb_busy_q[ifw_q]
                                                        | (32'd1 << d_rd);
                                pc_q[ifw_q] <= pc0_w + 32'd1;
                                istate_q   <= W_EXEC;
                            end
                        end else if (is_falu_cls_w) begin
                            if (!ws_falu_issue_vld) begin
                                ws_falu_issue_vld <= 1'b1;
                                fa_op_q <= d_op;
                                fa_rd_q <= d_rd;
                                fa_w_q  <= ifw_q;
                                fa_m_q  <= mask_q[ifw_q];
                                fa_a_q  <= rd_a_q;
                                fa_b_q  <= rd_b_q;
                                if (d_rd != 5'd0)
                                    sb_busy_q[ifw_q] <= sb_busy_q[ifw_q]
                                                        | (32'd1 << d_rd);
                                pc_q[ifw_q] <= pc0_w + 32'd1;
                                istate_q   <= W_EXEC;
                            end
                        end else begin
                            // lsu 目的（含 C2 均匀基址校验，ws_spec §5.4）
                            if (d_op == OP_LDG) begin
                                if (!uni_a_w[32] && !uni_b_w[32])
                                    err_q <= 1'b1;   // 基址非均匀（偏差 C2）
                                else if (!ws_lsu_issue_vld) begin
                                    ws_lsu_issue_vld <= 1'b1;
                                    la_op_q  <= d_op;
                                    la_rd_q  <= d_rd;
                                    la_w_q   <= ifw_q;
                                    la_m_q   <= mask_q[ifw_q];
                                    la_imm_q <= uni_a_w[32] ? uni_a_w[31:0]
                                                            : uni_b_w[31:0];
                                    la_a_q   <= uni_a_w[32] ? rd_b_q : rd_a_q;
                                    la_b_q   <= {VEC_W{1'b0}};
                                    if (d_rd != 5'd0)
                                        sb_busy_q[ifw_q] <= sb_busy_q[ifw_q]
                                                            | (32'd1 << d_rd);
                                    lsu_out_q[ifw_q] <= lsu_out_q[ifw_q] + 3'd1;
                                    pc_q[ifw_q] <= pc0_w + 32'd1;
                                    istate_q   <= W_EXEC;
                                end
                            end else if (d_op == OP_STG) begin
                                if (!uni_a_w[32])
                                    err_q <= 1'b1;   // 基址非均匀（偏差 C2）
                                else if (!ws_lsu_issue_vld) begin
                                    ws_lsu_issue_vld <= 1'b1;
                                    la_op_q  <= d_op;
                                    la_rd_q  <= d_rd;
                                    la_w_q   <= ifw_q;
                                    la_m_q   <= mask_q[ifw_q];
                                    la_imm_q <= uni_a_w[31:0];
                                    la_a_q   <= rd_c_q;   // 数据 R[rt]
                                    la_b_q   <= rd_b_q;   // 偏移 R[rb]
                                    if (d_rd != 5'd0)
                                        sb_busy_q[ifw_q] <= sb_busy_q[ifw_q]
                                                            | (32'd1 << d_rd);
                                    lsu_out_q[ifw_q] <= lsu_out_q[ifw_q] + 3'd1;
                                    pc_q[ifw_q] <= pc0_w + 32'd1;
                                    istate_q   <= W_EXEC;
                                end
                            end else if (!ws_lsu_issue_vld) begin
                                // LDS / STS（无全局基址）
                                ws_lsu_issue_vld <= 1'b1;
                                la_op_q  <= d_op;
                                la_rd_q  <= d_rd;
                                la_w_q   <= ifw_q;
                                la_m_q   <= mask_q[ifw_q];
                                la_imm_q <= 32'd0;
                                if (d_op == OP_LDS) begin
                                    la_a_q <= rd_a_q;   // 逐 lane 偏移
                                    la_b_q <= {VEC_W{1'b0}};
                                end else begin
                                    la_a_q <= rd_c_q;   // 数据 R[rt]
                                    la_b_q <= rd_a_q;   // 逐 lane 偏移
                                end
                                if (d_rd != 5'd0)
                                    sb_busy_q[ifw_q] <= sb_busy_q[ifw_q]
                                                        | (32'd1 << d_rd);
                                lsu_out_q[ifw_q] <= lsu_out_q[ifw_q] + 3'd1;
                                pc_q[ifw_q] <= pc0_w + 32'd1;
                                istate_q   <= W_EXEC;
                            end
                        end
                    end
                    default: ;
                    endcase
                end
            end

            // ================ 9. 块完成 / 前端占用释放（ws_spec §6） ================
            if (launched_q && (done_q == {NWARPS{1'b1}})
                && !ws_bs_bdone_vld && !bdone_sent_q) begin
                ws_bs_bdone_vld <= 1'b1;
                bdone_idx_q     <= block_idx_q;
                bdone_sent_q    <= 1'b1;
            end
            if (bdone_sent_q && bdone_fire) begin
                bdone_sent_q <= 1'b0;
                launched_q   <= 1'b0;
            end
            if (active_q && (done_q == {NWARPS{1'b1}}))
                active_q <= 1'b0;
        end
    end

endmodule
