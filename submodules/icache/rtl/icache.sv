// =============================================================================
// easy_simt · icache — Instruction Cache（ma_spec §8）
//
// 直接映射指令缓存：32B 行 = 8 条指令，默认 16 行（512B）。缺失阻塞：
// 缺失期间取指请求挂起（sf 侧记 IMISS），经 memif 回填整行后返回指令。
// 无预取、无无效化（程序只读，上电后内容不变）。
//
// 时序结构（icache_spec §1.4/§6/§7）——四态状态机：
//   S_IDLE   受理取指请求，组合查 tag/data：命中锁存指令转 S_RSP，
//            缺失锁存 miss_pc 转 S_REQ（受理拍与查找同拍）；
//   S_REQ    呈现回填请求（行对齐字节地址），vld 保持至握手，转 S_REFILL；
//   S_REFILL 等待回填数据（rdy 恒 1，呈现即握手），握手拍整行写阵列
//            并自回填数据锁存指令，转 S_RSP；
//   S_RSP    呈现指令，vld 保持至握手，回 S_IDLE。
// 命中路径受理拍+1 呈现指令；缺失路径回填握手拍+1 呈现指令——与 C 模型
// icache_step 的步序逐拍同构（icache_spec §1.5/§7.4）。片外固定延迟
// MEM_LAT 归 tb 侧从设备建模（intf_spec §10），本模块自身不加延迟。
//
// 设计依据：icache/docs/icache_spec_v0.1.md；
//   端口命名与 intf_spec §8 一致，握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module icache #(
    parameter ICACHE_LINES = 16,              // 行数（512B，ma_spec §1.6）
    parameter ILINE_WORDS  = 8                // 每行字数（32B 行，ma_spec §1.6）
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // sf_icache_req / icache_sf_rsp（intf_spec §8）
    input  wire                  sf_icache_req_vld,
    input  wire [31:0]           sf_icache_req_pc,
    output wire                  icache_sf_req_rdy,
    output wire                  icache_sf_rsp_vld,
    output wire [31:0]           icache_sf_rsp_inst,
    input  wire                  sf_icache_rsp_rdy,

    // icache_memif_req / memif_icache_rsp（intf_spec §8）
    output wire                  icache_memif_req_vld,
    output wire [31:0]           icache_memif_req_addr,
    input  wire                  memif_icache_req_rdy,
    input  wire                  memif_icache_rsp_vld,
    input  wire [ILINE_WORDS*32-1:0] memif_icache_rsp_data,
    output wire                  icache_memif_rsp_rdy
);

    // ---------------- 几何守卫（直接映射索引按位截取） ----------------
    localparam LW    = $clog2(ILINE_WORDS);   // 行内字偏移位宽
    localparam LI    = $clog2(ICACHE_LINES);  // 行索引位宽
    localparam TAG_W = 32 - LW;               // 行 tag = pc>>LW 全宽
    generate
        if (ILINE_WORDS < 2 || (ILINE_WORDS & (ILINE_WORDS-1)) != 0 ||
            ICACHE_LINES < 2 || (ICACHE_LINES & (ICACHE_LINES-1)) != 0) begin : g_cfg_err
            initial begin
                $display("icache: ICACHE_LINES/ILINE_WORDS 须为 >=2 的 2 的幂，收到 %0d / %0d",
                         ICACHE_LINES, ILINE_WORDS);
                $finish;
            end
        end
    endgenerate

    // ---------------- 状态编码（icache_spec §6.1） ----------------
    localparam [1:0] S_IDLE   = 2'd0;   // 空闲：受理取指请求（组合查找）
    localparam [1:0] S_REQ    = 2'd1;   // 回填请求呈现（保持至握手）
    localparam [1:0] S_REFILL = 2'd2;   // 回填等待（rdy 恒 1，呈现即握手）
    localparam [1:0] S_RSP    = 2'd3;   // 指令呈现（保持至握手）

    // ---------------- 内部状态（icache_spec §4） ----------------
    reg  [1:0]                   state;
    reg  [31:0]                  miss_pc_q;    // 缺失在途的取指 pc
    reg  [31:0]                  rsp_inst_q;   // 待回送 sf 的指令字
    reg  [ILINE_WORDS*32-1:0]    data [ICACHE_LINES];
    reg  [TAG_W-1:0]             tag  [ICACHE_LINES];
    reg  [ICACHE_LINES-1:0]      valid;

    // ---------------- 取指查找（S_IDLE 组合，icache_spec §5.1） -----------
    wire [31:0]       pc     = sf_icache_req_pc;
    wire [LI-1:0]     ld_idx = pc[LW+LI-1:LW];
    wire [TAG_W-1:0]  pc_tag = pc[31:LW];
    wire [ILINE_WORDS*32-1:0] rd_line = data[ld_idx];
    wire [31:0]       rd_word = rd_line[pc[LW-1:0] * 32 +: 32];
    wire              hit = valid[ld_idx] && (tag[ld_idx] == pc_tag);

    assign icache_sf_req_rdy = (state == S_IDLE);
    wire req_fire = sf_icache_req_vld && icache_sf_req_rdy;

    // ---------------- 回填请求地址形成（组合，icache_spec §5.2） ----------
    assign icache_memif_req_vld  = (state == S_REQ);
    assign icache_memif_req_addr = (miss_pc_q >> LW) << (LW + 2);
    wire mreq_fire = icache_memif_req_vld && memif_icache_req_rdy;

    // ---------------- 回填写行与指令选取（icache_spec §5.3） --------------
    assign icache_memif_rsp_rdy = (state == S_REFILL);
    wire mrsp_fire = memif_icache_rsp_vld && icache_memif_rsp_rdy;

    wire [LI-1:0]    wr_idx  = miss_pc_q[LW+LI-1:LW];
    wire [TAG_W-1:0] wr_tag  = miss_pc_q[31:LW];
    wire [31:0]      wr_word = memif_icache_rsp_data[miss_pc_q[LW-1:0] * 32 +: 32];

    // ---------------- 指令呈现（icache_spec §6.1） ------------------------
    assign icache_sf_rsp_vld  = (state == S_RSP);
    assign icache_sf_rsp_inst = rsp_inst_q;
    wire rsp_fire = icache_sf_rsp_vld && sf_icache_rsp_rdy;

    // ---------------- 主状态机（icache_spec §6） ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            miss_pc_q  <= 32'd0;
            rsp_inst_q <= 32'd0;
            valid      <= {ICACHE_LINES{1'b0}};
        end else begin
            case (state)
            // ----------------------------------------------------
            // S_IDLE：受理取指请求，命中锁存指令 / 缺失锁存 pc（§5.1）
            // ----------------------------------------------------
            S_IDLE: begin
                if (req_fire) begin
                    if (hit) begin
                        rsp_inst_q <= rd_word;
                        state      <= S_RSP;
                    end else begin
                        miss_pc_q <= pc;
                        state     <= S_REQ;
                    end
                end
            end
            // ----------------------------------------------------
            // S_REQ：回填请求呈现，握手后转回填等待
            // ----------------------------------------------------
            S_REQ: begin
                if (mreq_fire) state <= S_REFILL;
            end
            // ----------------------------------------------------
            // S_REFILL：回填数据握手拍整行写阵列并锁存指令（§5.3）
            // ----------------------------------------------------
            S_REFILL: begin
                if (mrsp_fire) begin
                    data[wr_idx]  <= memif_icache_rsp_data;
                    tag[wr_idx]   <= wr_tag;
                    valid[wr_idx] <= 1'b1;
                    rsp_inst_q    <= wr_word;
                    state         <= S_RSP;
                end
            end
            // ----------------------------------------------------
            // S_RSP：指令呈现，握手后回空闲（次拍可受理新请求）
            // ----------------------------------------------------
            S_RSP: begin
                if (rsp_fire) state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
