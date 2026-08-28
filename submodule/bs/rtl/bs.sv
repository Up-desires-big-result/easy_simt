// =============================================================================
// easy_simt · bs — Block Scheduler（ma_spec §4）
//
// 纯块派发：推进 grid、下发启动上下文 {blockIdx, N, SHBASE}、收 block_done
// 拉下一块。MAX_BLOCKS_INFLIGHT = 1（v1 串行）。
//
// 设计依据：bs/docs/bs_spec_v0.1.md；
//   端口命名与 intf_spec §4 一致，握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module bs #(
    parameter DATA_W = 32,              // 数据/地址/载荷位宽（intf_spec §1.4）
    parameter NWARPS = 4,               // warp 数/块（ma_spec §1.2）
    parameter NLANES = 8                // lane 数/warp（ma_spec §1.2）
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // 配置：N（每块线程总数），静态（bs_spec §3）
    input  wire [DATA_W-1:0]     bs_cfg_n,

    // bs_sf_launch：向 sf 发起块启动（输出载荷为寄存器/寄存值导出）
    output reg                   bs_sf_launch_vld,
    output wire [DATA_W-1:0]     bs_sf_launch_block_idx,
    output wire [DATA_W-1:0]     bs_sf_launch_n,
    output wire [DATA_W-1:0]     bs_sf_launch_shbase,
    input  wire                  sf_bs_launch_rdy,

    // bs_ws_launch：向 ws 发起块启动
    output reg                   bs_ws_launch_vld,
    output wire [DATA_W-1:0]     bs_ws_launch_block_idx,
    input  wire                  ws_bs_launch_rdy,

    // ws_bs_bdone：ws 上报块完成（载荷不参与逻辑，bs_spec §8 条 4）
    input  wire                  ws_bs_bdone_vld,
    input  wire [DATA_W-1:0]     ws_bs_bdone_block_idx,
    output wire                  bs_ws_bdone_rdy,

    // 顶层：grid 结束（锁存，intf_spec §1.6）
    output reg                   bs_top_done
);

    // ---------------- 状态编码（bs_spec §5.1） ----------------
    localparam [1:0] S_LAUNCH = 2'd0;   // 派发
    localparam [1:0] S_RUN    = 2'd1;   // 在途
    localparam [1:0] S_DONE   = 2'd2;   // 停机

    // 每块线程数 = NWARPS × NLANES（=32，2 的幂，grid 用移位实现除法）
    localparam TPB      = NWARPS * NLANES;
    localparam TPB_SH   = $clog2(TPB);

    // ---------------- 内部状态（bs_spec §4） ----------------
    reg  [1:0]         state;
    reg  [DATA_W-1:0]  block_idx;       // 当前（派发中/在途）块索引
    reg                sf_done;         // 本块 bs_sf_launch 已完成握手
    reg                ws_done;         // 本块 bs_ws_launch 已完成握手
    reg  [DATA_W-1:0]  n_r;             // bs_cfg_n 寄存值
    reg  [DATA_W-1:0]  grid_r;          // grid = ceil(N / TPB) 寄存值

    // 握手组合判据（本拍完成）
    wire sf_fire = bs_sf_launch_vld && sf_bs_launch_rdy;
    wire ws_fire = bs_ws_launch_vld && ws_bs_launch_rdy;
    wire bd_fire = ws_bs_bdone_vld  && bs_ws_bdone_rdy;

    // ---------------- 主状态机 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_LAUNCH;   // 上电即启动第 0 块（ma_spec §4）
            block_idx          <= {DATA_W{1'b0}};
            sf_done            <= 1'b0;
            ws_done            <= 1'b0;
            n_r                <= {DATA_W{1'b0}};
            grid_r             <= {DATA_W{1'b0}};
            bs_sf_launch_vld   <= 1'b0;
            bs_ws_launch_vld   <= 1'b0;
            bs_top_done        <= 1'b0;
        end else begin
            // 配置采样：静态约束下寄存值与端口值一致（bs_spec §3）
            n_r    <= bs_cfg_n;
            grid_r <= (bs_cfg_n + TPB[DATA_W-1:0] - {{(DATA_W-1){1'b0}}, 1'b1}) >> TPB_SH;

            case (state)
            // ----------------------------------------------------
            // S_LAUNCH：两条通道各自拉起并保持至各自握手（bs_spec §5.2）
            // ----------------------------------------------------
            S_LAUNCH: begin
                if (!sf_done) begin
                    bs_sf_launch_vld <= 1'b1;
                    if (sf_fire) begin
                        sf_done            <= 1'b1;
                        bs_sf_launch_vld   <= 1'b0;
                    end
                end
                if (!ws_done) begin
                    bs_ws_launch_vld <= 1'b1;
                    if (ws_fire) begin
                        ws_done            <= 1'b1;
                        bs_ws_launch_vld   <= 1'b0;
                    end
                end
                // 两侧均完成（含本拍完成）→ 块启动完成
                if ((sf_done || sf_fire) && (ws_done || ws_fire))
                    state <= S_RUN;
            end
            // ----------------------------------------------------
            // S_RUN：等待 block_done；消费后递增并判越界（bs_spec §5.2）
            // ----------------------------------------------------
            S_RUN: begin
                if (bd_fire) begin
                    if (block_idx + {{(DATA_W-1){1'b0}}, 1'b1} >= grid_r) begin
                        state       <= S_DONE;
                        bs_top_done <= 1'b1;
                    end else begin
                        block_idx <= block_idx + {{(DATA_W-1){1'b0}}, 1'b1};
                        sf_done   <= 1'b0;
                        ws_done   <= 1'b0;
                        state     <= S_LAUNCH;
                    end
                end
            end
            // ----------------------------------------------------
            // S_DONE：停机，保持（bs_spec §5.2）
            // ----------------------------------------------------
            S_DONE: begin
            end
            default: ;
            endcase
        end
    end

    // ---------------- 组合导出（载荷在 vld 保持期内稳定，bs_spec §4） ----------------
    assign bs_sf_launch_block_idx = block_idx;
    assign bs_sf_launch_n         = n_r;
    assign bs_sf_launch_shbase    = {DATA_W{1'b0}};   // v1 单块在途恒 0
    assign bs_ws_launch_block_idx = block_idx;
    assign bs_ws_bdone_rdy        = 1'b1;             // bs 始终接受 block_done

endmodule
