// =============================================================================
// easy_simt · memif — Memory Interface（ma_spec §10）
//
// 片外唯一通道：仲裁 icache 与 l1sm 的回填/写通请求，固定优先级
// （icache 优先），单请求在途；对外为 AXI4 主设备（全五通道，ID 恒 0，
// INCR）。请求队列留参数位，v1 不实现（ma_spec §10）。
//
// 时序结构（memif_spec §1.4/§7）：
//   请求通道组合直通——arvalid/awvalid/wvalid 随内部请求组合呈现，
//   内部请求握手与 AXI 请求通道握手同拍完成（写路径 AW/W 握手不同拍时
//   经 S_WREQ 补齐，§6）；
//   响应通道组合桥接——memif_*_rsp_vld 由状态译码与 rvalid/bvalid 组合
//   导出，axi_rready/bready 由内部消费者 rdy 门控（从设备保持
//   rvalid/bvalid 至本模块接收）；
//   本模块自身不加延迟：片外固定延迟 MEM_LAT 由 tb 侧 AXI 从设备建模
//   （intf_spec §10；C 模型中为响应倒计时，memif_spec §1.5）。
//
// rresp/bresp 非 OKAY 于响应呈现拍检测，次拍起 memif_top_err=1 并保持
// 至复位（intf_spec §10）；响应照常交付。
//
// 设计依据：memif/docs/memif_spec_v0.1.md；
//   端口命名与 intf_spec §10 一致（内部通道见 §10 内部侧、AXI 参数见
//   §1.4），握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module memif #(
    parameter AXI_ID_W    = 4,               // AXI ID 位宽，v1 恒置 0（intf_spec §1.4）
    parameter AXI_ADDR_W  = 32,              // AXI 地址位宽（intf_spec §1.4）
    parameter AXI_DATA_W  = 256,             // AXI 数据位宽 = 1 行（intf_spec §1.4）
    parameter AXI_STRB_W  = AXI_DATA_W / 8,  // AXI 字节选通位宽（intf_spec §1.4）
    parameter ILINE_WORDS = 8,               // 指令行字数（ma_spec §1.6）
    localparam LINE_W     = ILINE_WORDS * 32
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // icache_memif_req / memif_icache_rsp（intf_spec §10 内部侧）
    input  wire                  icache_memif_req_vld,
    input  wire [AXI_ADDR_W-1:0] icache_memif_req_addr,
    output wire                  memif_icache_req_rdy,
    output wire                  memif_icache_rsp_vld,
    output wire [AXI_DATA_W-1:0] memif_icache_rsp_data,
    input  wire                  icache_memif_rsp_rdy,

    // l1sm_memif_req / memif_l1sm_rsp（intf_spec §10 内部侧）
    input  wire                  l1sm_memif_req_vld,
    input  wire                  l1sm_memif_req_rw,
    input  wire [AXI_ADDR_W-1:0] l1sm_memif_req_addr,
    input  wire [31:0]           l1sm_memif_req_wdata,
    output wire                  memif_l1sm_req_rdy,
    output wire                  memif_l1sm_rsp_vld,
    output wire [AXI_DATA_W-1:0] memif_l1sm_rsp_data,
    input  wire                  l1sm_memif_rsp_rdy,

    // AXI4 AW 通道
    output wire [AXI_ID_W-1:0]   axi_awid,
    output wire [AXI_ADDR_W-1:0] axi_awaddr,
    output wire [7:0]            axi_awlen,
    output wire [2:0]            axi_awsize,
    output wire [1:0]            axi_awburst,
    output wire                  axi_awvalid,
    input  wire                  axi_awready,

    // AXI4 W 通道
    output wire [AXI_DATA_W-1:0] axi_wdata,
    output wire [AXI_STRB_W-1:0] axi_wstrb,
    output wire                  axi_wlast,
    output wire                  axi_wvalid,
    input  wire                  axi_wready,

    // AXI4 B 通道
    input  wire [AXI_ID_W-1:0]   axi_bid,
    input  wire [1:0]            axi_bresp,
    input  wire                  axi_bvalid,
    output wire                  axi_bready,

    // AXI4 AR 通道
    output wire [AXI_ID_W-1:0]   axi_arid,
    output wire [AXI_ADDR_W-1:0] axi_araddr,
    output wire [7:0]            axi_arlen,
    output wire [2:0]            axi_arsize,
    output wire [1:0]            axi_arburst,
    output wire                  axi_arvalid,
    input  wire                  axi_arready,

    // AXI4 R 通道
    input  wire [AXI_ID_W-1:0]   axi_rid,
    input  wire [AXI_DATA_W-1:0] axi_rdata,
    input  wire [1:0]            axi_rresp,
    input  wire                  axi_rlast,
    input  wire                  axi_rvalid,
    output wire                  axi_rready,

    // 顶层
    output wire                  memif_top_err
);

    // ---------------- 几何守卫（回填整行口径） ----------------
    generate
        if (AXI_DATA_W != LINE_W || AXI_STRB_W != AXI_DATA_W / 8) begin : g_cfg_err
            initial begin
                $display("memif: AXI_DATA_W 须为 ILINE_WORDS*32（回填整行），收到 %0d vs %0d",
                         AXI_DATA_W, LINE_W);
                $finish;
            end
        end
    endgenerate

    // ---------------- 常量（intf_spec §10 v1 约束） ----------------
    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [2:0] ASIZE_LINE  = 3'b101;    // 读：32B/拍，单拍整行
    localparam [2:0] ASIZE_WORD  = 3'b010;    // 写：4B 窄传
    localparam [1:0] BURST_INCR  = 2'b01;

    // ---------------- 状态编码（memif_spec §6.1） ----------------
    localparam [1:0] S_IDLE = 2'd0;   // 空闲：可受理内部请求（读同拍入 AXI）
    localparam [1:0] S_WREQ = 2'd1;   // 写请求通道补齐（AW/W 握手分离时，§6）
    localparam [1:0] S_RSP  = 2'd2;   // 响应等待：桥接 AXI R/B 至内部响应

    // ---------------- 内部状态（memif_spec §4） ----------------
    reg  [1:0]         state;
    reg                to_icache_q;    // 在途响应目标：1=icache 0=l1sm
    reg                rw_q;           // 在途事务：0=读 1=写
    reg                aw_done_q;      // S_WREQ：AW 已握手
    reg                w_done_q;       // S_WREQ：W 已握手
    reg                err_q;          // 非 OKAY 粘滞（memif_top_err）

    // ---------------- 请求仲裁（组合，icache 优先，memif_spec §5.1） ------
    wire ic_sel = icache_memif_req_vld;
    wire l1_sel = !icache_memif_req_vld && l1sm_memif_req_vld;
    wire req_wr = l1_sel && l1sm_memif_req_rw;

    assign memif_icache_req_rdy = (state == S_IDLE) && axi_arready;
    assign memif_l1sm_req_rdy   = ((state == S_IDLE) && !icache_memif_req_vld &&
                                    (l1sm_memif_req_rw ? (axi_awready && axi_wready)
                                                       : axi_arready))
                                || ((state == S_WREQ) && (aw_done_q || axi_awvalid && axi_awready)
                                              && (w_done_q  || axi_wvalid  && axi_wready));

    wire ic_fire = icache_memif_req_vld && memif_icache_req_rdy;
    wire l1_fire = l1sm_memif_req_vld   && memif_l1sm_req_rdy;

    // ---------------- AXI 请求通道呈现（组合直通，memif_spec §5.2） -------
    wire [AXI_ADDR_W-1:0] req_addr = ic_sel ? icache_memif_req_addr
                                            : l1sm_memif_req_addr;
    wire [31:0]  req_wdata = l1sm_memif_req_wdata;
    wire [4:0]   wsel      = req_addr[4:2];      // 写窄传字定位（intf_spec §10）

    wire wr_req_go = (state == S_IDLE) ? req_wr : (state == S_WREQ);

    assign axi_arvalid = (state == S_IDLE) && !req_wr &&
                         (icache_memif_req_vld || l1sm_memif_req_vld);
    assign axi_araddr  = req_addr;
    assign axi_arid    = {AXI_ID_W{1'b0}};
    assign axi_arlen   = 8'd0;                   // 单拍整行
    assign axi_arsize  = ASIZE_LINE;
    assign axi_arburst = BURST_INCR;

    assign axi_awvalid = wr_req_go && !aw_done_q;
    assign axi_awaddr  = req_addr;
    assign axi_awid    = {AXI_ID_W{1'b0}};
    assign axi_awlen   = 8'd0;                   // 单拍
    assign axi_awsize  = ASIZE_WORD;             // 4B 窄传
    assign axi_awburst = BURST_INCR;

    assign axi_wvalid  = wr_req_go && !w_done_q;
    assign axi_wdata   = {{(AXI_DATA_W-32){1'b0}}, req_wdata} << (wsel * 32);
    assign axi_wstrb   = 4'hF << (wsel * 4);
    assign axi_wlast   = 1'b1;                   // 单拍

    wire ar_fire = axi_arvalid && axi_arready;
    wire aw_fire = axi_awvalid && axi_awready;
    wire w_fire  = axi_wvalid  && axi_wready;
    wire aw_ok   = aw_done_q || (axi_awvalid && axi_awready);
    wire w_ok    = w_done_q  || (axi_wvalid  && axi_wready);

    // ---------------- 响应桥接（组合，memif_spec §5.3） --------------------
    assign axi_rready = (state == S_RSP) && !rw_q &&
                        (to_icache_q ? icache_memif_rsp_rdy : l1sm_memif_rsp_rdy);
    assign axi_bready = (state == S_RSP) && rw_q && l1sm_memif_rsp_rdy;

    wire r_beat = axi_rvalid && axi_rready;
    wire b_beat = axi_bvalid && axi_bready;
    wire rsp_fire = (state == S_RSP) && (r_beat || b_beat);

    assign memif_icache_rsp_vld  = (state == S_RSP) && to_icache_q && axi_rvalid;
    assign memif_icache_rsp_data = axi_rdata;
    assign memif_l1sm_rsp_vld    = (state == S_RSP) && !to_icache_q &&
                                   (rw_q ? axi_bvalid : axi_rvalid);
    assign memif_l1sm_rsp_data   = rw_q ? {AXI_DATA_W{1'b0}} : axi_rdata;

    assign memif_top_err = err_q;

    // ---------------- 主状态机（memif_spec §6） ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            to_icache_q <= 1'b0;
            rw_q       <= 1'b0;
            aw_done_q  <= 1'b0;
            w_done_q   <= 1'b0;
            err_q      <= 1'b0;
        end else begin
            // ---- AW/W 部分完成登记（S_WREQ 补齐用，§6.2） ----
            if (aw_fire) aw_done_q <= 1'b1;
            if (w_fire)  w_done_q  <= 1'b1;

            // ---- 错误检测：响应呈现拍非 OKAY，次拍起粘滞（§5.4） ----
            if ((state == S_RSP) && !rw_q && axi_rvalid && (axi_rresp != RESP_OKAY))
                err_q <= 1'b1;
            if ((state == S_RSP) && rw_q && axi_bvalid && (axi_bresp != RESP_OKAY))
                err_q <= 1'b1;

            case (state)
            // ----------------------------------------------------
            // S_IDLE：读同拍入 AXI（AR 握手 = 内部受理）；写请求同拍
            // 完成或转 S_WREQ 补齐（§6.2）
            // ----------------------------------------------------
            S_IDLE: begin
                if (req_wr) begin
                    if (aw_ok && w_ok) begin
                        to_icache_q <= 1'b0;
                        rw_q        <= 1'b1;
                        aw_done_q   <= 1'b0;
                        w_done_q    <= 1'b0;
                        state       <= S_RSP;
                    end else begin
                        state <= S_WREQ;
                    end
                end else if (ar_fire) begin
                    to_icache_q <= icache_memif_req_vld;   // icache 优先
                    rw_q        <= 1'b0;
                    state       <= S_RSP;
                end
            end
            // ----------------------------------------------------
            // S_WREQ：写请求通道补齐（AW/W 握手分离；tb 请求通道恒
            // ready，单元验证不激发，为一般从设备预留，§6.2）
            // ----------------------------------------------------
            S_WREQ: begin
                if (aw_ok && w_ok) begin
                    to_icache_q <= 1'b0;
                    rw_q        <= 1'b1;
                    aw_done_q   <= 1'b0;
                    w_done_q    <= 1'b0;
                    state       <= S_RSP;
                end
            end
            // ----------------------------------------------------
            // S_RSP：响应桥接至内部消费（R/B 握手 = 内部响应握手，§5.3）
            // ----------------------------------------------------
            S_RSP: begin
                if (rsp_fire) state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
