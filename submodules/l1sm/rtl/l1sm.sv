// =============================================================================
// easy_simt · l1sm — L1 + Shared Memory（统一 SRAM，ma_spec §8）
//
// L1 与共享内存共用一块统一 SRAM（数据阵列 + tag 阵列 {class,valid,tag}），
// 行的概念归本模块管。SM 侧钉扎自指哨兵（复位时钉扎，恒命中、不可替换）；
// L1 侧偏移索引直接映射（set = line mod NSETS 精确模），缺失阻塞回填、
// 写直通不写分配。8-bank 锁步：单行无 bank 冲突单服务节拍完成；跨行/
// 冲突按行组逐拍串行（l1sm_spec §1.4）。
//
// 服务节拍：S_GRP 一拍完成一次行级 tag 比较与该行组内各 bank 的并行
// 访问；写通逐 lane（升序）经 S_WT 交付 memif；回填整行单沿写入。
// 完成路径：S_IDLE → S_GRP（→ S_REFILL / S_WT）*→ S_RSP。
//
// 设计依据：l1sm/docs/l1sm_spec_v0.1.md；
//   端口命名与 intf_spec §8 一致，握手协议与 intf_spec §1.2 一致。
// =============================================================================
`timescale 1ns/1ps

module l1sm #(
    parameter DATA_W      = 32,             // 数据/地址位宽（intf_spec §1.4）
    parameter NWARPS      = 4,              // warp 数/块（ma_spec §1.2）
    parameter NLANES      = 8,              // lane 数/warp = bank 数（ma_spec §1.2）
    parameter U_LINES     = 64,             // 统一 SRAM 总行数（ma_spec §1.6）
    parameter SM_LINES    = 4,              // SM 分区行数（ma_spec §1.6）
    parameter NBANKS      = 8,              // bank 数 = NLANES（ma_spec §1.6）
    parameter ILINE_WORDS = 8               // 每行字数（32B 行，ma_spec §1.6）
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // lsu_l1sm_req：8-lane 锁步请求（mask 门控活跃 lane；shmem 地址已含
    // SHBASE，v1 恒 0；全局地址 4B 对齐由 lsu 保证）
    input  wire                        lsu_l1sm_req_vld,
    input  wire                        lsu_l1sm_req_rw,
    input  wire                        lsu_l1sm_req_sm,
    input  wire [NLANES*DATA_W-1:0]    lsu_l1sm_req_addr,
    input  wire [NLANES*DATA_W-1:0]    lsu_l1sm_req_wdata,
    input  wire [NLANES-1:0]           lsu_l1sm_req_mask,
    output wire                        l1sm_lsu_req_rdy,

    // l1sm_lsu_rsp：装载为 8-lane 读数据、存储恒 0
    output wire                        l1sm_lsu_rsp_vld,
    output wire [NLANES*DATA_W-1:0]    l1sm_lsu_rsp_rdata,
    input  wire                        lsu_l1sm_rsp_rdy,

    // l1sm_memif_req：回填读（rw=0，行对齐）/ 写通（rw=1，字地址 4B）
    output wire                        l1sm_memif_req_vld,
    output wire                        l1sm_memif_req_rw,
    output wire [DATA_W-1:0]           l1sm_memif_req_addr,
    output wire [DATA_W-1:0]           l1sm_memif_req_wdata,
    input  wire                        memif_l1sm_req_rdy,

    // memif_l1sm_rsp：读返回整行、写表示写完成（载荷 0）
    input  wire                        memif_l1sm_rsp_vld,
    input  wire [ILINE_WORDS*DATA_W-1:0] memif_l1sm_rsp_data,
    output wire                        l1sm_memif_rsp_rdy
);

    // ---------------- 派生常量（l1sm_spec §3） ----------------
    localparam TAG_W  = DATA_W - 5;               // tag = addr[31:5]
    localparam NSETS  = U_LINES - SM_LINES;       // L1 组数（基线 60，非 2 的幂）
    localparam VEC_W  = NLANES * DATA_W;
    localparam LINE_W = ILINE_WORDS * DATA_W;

    // ---------------- 类位编码（ma_spec §8） ----------------
    localparam [1:0] CLS_INV = 2'd0;
    localparam [1:0] CLS_L1  = 2'd1;
    localparam [1:0] CLS_SM  = 2'd2;

    // ---------------- 状态编码（l1sm_spec §6.1） ----------------
    localparam [2:0] S_IDLE   = 3'd0;   // 空闲：可接收请求
    localparam [2:0] S_GRP    = 3'd1;   // 行组服务节拍
    localparam [2:0] S_REFILL = 3'd2;   // L1 读缺失回填
    localparam [2:0] S_WT     = 3'd3;   // L1 写逐 lane 写通
    localparam [2:0] S_RSP    = 3'd4;   // 响应呈现

    // ---------------- 内部状态（l1sm_spec §4） ----------------
    reg  [2:0]         state;
    reg                req_rw_q, req_sm_q;
    reg  [DATA_W-1:0]  req_addr_q  [NLANES];
    reg  [DATA_W-1:0]  req_wdata_q [NLANES];
    reg  [NLANES-1:0]  req_mask_q;
    reg  [3:0]         glane_q;          // 服务游标（下一行组起点）
    reg  [3:0]         grp_end_q;        // 当前行组上界（不含）
    reg  [3:0]         wt_lane_q;        // 写通当前 lane
    reg                mreq_done_q;      // 当前 memif 请求已握手（完成后撤 vld）
    reg  [VEC_W-1:0]   rsp_q;            // 响应逐 lane 读数据
    reg                memif_rw_q;
    reg  [DATA_W-1:0]  memif_addr_q, memif_wdata_q;

    // 阵列（寄存器实现，回填整行单沿写入的前提，l1sm_spec §1.2）
    reg  [LINE_W-1:0]  data [U_LINES];
    reg  [TAG_W-1:0]   tag  [U_LINES];
    reg  [1:0]         cls  [U_LINES];
    reg  [U_LINES-1:0] valid;

    // ---------------- 握手组合判据 ----------------
    wire req_fire  = lsu_l1sm_req_vld   && l1sm_lsu_req_rdy;
    wire rsp_fire  = l1sm_lsu_rsp_vld   && lsu_l1sm_rsp_rdy;
    wire mreq_fire = l1sm_memif_req_vld && memif_l1sm_req_rdy;
    wire mrsp_fire = memif_l1sm_rsp_vld && l1sm_memif_rsp_rdy;

    // ---------------- 通道 vld / rdy（状态译码，l1sm_spec §4） ----------------
    assign l1sm_lsu_req_rdy   = (state == S_IDLE);
    assign l1sm_lsu_rsp_vld   = (state == S_RSP);
    assign l1sm_memif_req_vld = ((state == S_REFILL) || (state == S_WT))
                                && !mreq_done_q;
    assign l1sm_memif_rsp_rdy = (state == S_REFILL) || (state == S_WT);
    assign l1sm_lsu_rsp_rdata = rsp_q;
    assign l1sm_memif_req_rw  = memif_rw_q;
    assign l1sm_memif_req_addr  = memif_addr_q;
    assign l1sm_memif_req_wdata = memif_wdata_q;

    // ---------------- 扫描辅助函数（l1sm_spec §5.1） ----------------
    // g 起（含）首个活动 lane；无则 8
    function automatic [3:0] first_active(input [3:0] g);
        integer i;
        reg f;
        begin
            first_active = 4'd8;
            f = 1'b0;
            for (i = 0; i < NLANES; i = i + 1)
                if (!f && (i >= g) && req_mask_q[i]) begin
                    first_active = i;
                    f = 1'b1;
                end
        end
    endfunction

    // g 起行组上界（不含）：首个违约活动 lane 的下标，或 8（l1sm_spec §1.4：
    // 行相同且 bank 互不重复的连续活动 lane 前缀；不活动 lane 跳过不断组）
    function automatic [3:0] grp_end_f(input [3:0] g);
        integer i;
        reg f, done;
        reg [TAG_W-1:0]  row0;
        reg [NBANKS-1:0] used;
        begin
            grp_end_f = 4'd8;
            f = 1'b0;
            done = 1'b0;
            for (i = 0; i < NLANES; i = i + 1) begin
                if (!done && (i >= g) && req_mask_q[i]) begin
                    if (!f) begin
                        f = 1'b1;
                        row0 = req_addr_q[i][DATA_W-1:5];
                        used = {NBANKS{1'b0}};
                        used[req_addr_q[i][4:2]] = 1'b1;
                    end else if ((req_addr_q[i][DATA_W-1:5] == row0)
                                 && !used[req_addr_q[i][4:2]]) begin
                        used[req_addr_q[i][4:2]] = 1'b1;
                    end else begin
                        grp_end_f = i;
                        done = 1'b1;
                    end
                end
            end
        end
    endfunction

    // [l, bnd) 内首个活动 lane；无则 8（写通游标推进用）
    function automatic [3:0] next_active_b(input [3:0] l, input [3:0] bnd);
        integer i;
        reg f;
        begin
            next_active_b = 4'd8;
            f = 1'b0;
            for (i = 0; i < NLANES; i = i + 1)
                if (!f && (i >= l) && (i < bnd) && req_mask_q[i]) begin
                    next_active_b = i;
                    f = 1'b1;
                end
        end
    endfunction

    // g 起（含）是否仍有活动 lane
    function automatic has_any(input [3:0] g);
        begin
            has_any = (req_mask_q >> g) != {NLANES{1'b0}};
        end
    endfunction

    // ---------------- S_GRP 组合决策（l1sm_spec §5） ----------------
    wire [3:0]        fa_w    = first_active(glane_q);       // 行组首活动 lane
    wire              more_w  = (fa_w != 4'd8);
    wire [DATA_W-1:0] faddr_w = req_addr_q[fa_w[2:0]];
    wire [TAG_W-1:0]  row_w   = faddr_w[DATA_W-1:5];         // 行号（SM 行/L1 line）
    wire [3:0]        ge_w    = grp_end_f(glane_q);          // 行组上界
    // L1 set 映射：精确模（NSETS 非 2 的幂，l1sm_spec §3/§9 条 7）
    wire [TAG_W-1:0]  set_w   = row_w % NSETS;
    wire [U_LINES-1:0] phys_w = SM_LINES + set_w;
    wire l1_hit_w = valid[phys_w] && (cls[phys_w] == CLS_L1)
                    && (tag[phys_w] == row_w);
    wire [3:0] wl0_w = next_active_b(glane_q, ge_w);         // 行组首写通 lane

    // ---------------- S_REFILL 回填目标行（自 memif_addr_q 重算） ----------------
    wire [TAG_W-1:0]   rline_w = memif_addr_q[DATA_W-1:5];
    wire [TAG_W-1:0]   rset_w  = rline_w % NSETS;
    wire [U_LINES-1:0] rphys_w = SM_LINES + rset_w;

    // ---------------- S_WT 写通推进目标 ----------------
    wire [3:0] wnext_w = next_active_b(wt_lane_q + 4'd1, grp_end_q);

    integer i;

    // ---------------- 主状态机（l1sm_spec §6） ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            req_rw_q      <= 1'b0;
            req_sm_q      <= 1'b0;
            req_mask_q    <= {NLANES{1'b0}};
            glane_q       <= 4'd0;
            grp_end_q     <= 4'd0;
            wt_lane_q     <= 4'd0;
            mreq_done_q   <= 1'b0;
            rsp_q         <= {VEC_W{1'b0}};
            memif_rw_q    <= 1'b0;
            memif_addr_q  <= {DATA_W{1'b0}};
            memif_wdata_q <= {DATA_W{1'b0}};
            for (i = 0; i < NLANES; i = i + 1) begin
                req_addr_q[i]  <= {DATA_W{1'b0}};
                req_wdata_q[i] <= {DATA_W{1'b0}};
            end
            // 复位钉扎（l1sm_spec §8）：SM 区自指哨兵，L1 区失效，数据全 0
            for (i = 0; i < U_LINES; i = i + 1) begin
                data[i] <= {LINE_W{1'b0}};
                if (i < SM_LINES) begin
                    tag[i]   <= i;
                    cls[i]   <= CLS_SM;
                    valid[i] <= 1'b1;
                end else begin
                    tag[i]   <= {TAG_W{1'b0}};
                    cls[i]   <= CLS_INV;
                    valid[i] <= 1'b0;
                end
            end
        end else begin
            case (state)
            // ----------------------------------------------------
            // S_IDLE：接收请求，锁存载荷、清响应/游标（l1sm_spec §6.2）
            // ----------------------------------------------------
            S_IDLE: begin
                if (req_fire) begin
                    req_rw_q   <= lsu_l1sm_req_rw;
                    req_sm_q   <= lsu_l1sm_req_sm;
                    req_mask_q <= lsu_l1sm_req_mask;
                    for (i = 0; i < NLANES; i = i + 1) begin
                        req_addr_q[i]  <= lsu_l1sm_req_addr[i*DATA_W +: DATA_W];
                        req_wdata_q[i] <= lsu_l1sm_req_wdata[i*DATA_W +: DATA_W];
                    end
                    rsp_q   <= {VEC_W{1'b0}};
                    glane_q <= 4'd0;
                    state   <= S_GRP;
                end
            end
            // ----------------------------------------------------
            // S_GRP：行组服务节拍（l1sm_spec §5.1–5.5）
            // ----------------------------------------------------
            S_GRP: begin
                if (!more_w) begin
                    state <= S_RSP;
                end else begin
                    grp_end_q <= ge_w;
                    if (req_sm_q) begin
                        // ---- SM（恒命中，无写通，l1sm_spec §5.2） ----
                        if (req_rw_q) begin
                            for (i = 0; i < NLANES; i = i + 1)
                                if ((i >= glane_q) && (i < ge_w) && req_mask_q[i])
                                    data[row_w]
                                        [req_addr_q[i][4:2]*DATA_W +: DATA_W]
                                        <= req_wdata_q[i];
                        end else begin
                            for (i = 0; i < NLANES; i = i + 1)
                                if ((i >= glane_q) && (i < ge_w) && req_mask_q[i])
                                    rsp_q[i*DATA_W +: DATA_W]
                                        <= data[row_w]
                                             [req_addr_q[i][4:2]*DATA_W +: DATA_W];
                        end
                        glane_q <= ge_w;
                        if (has_any(ge_w)) state <= S_GRP;
                        else               state <= S_RSP;
                    end else begin
                        // ---- L1（l1sm_spec §5.3–5.5） ----
                        if (!req_rw_q) begin
                            // 读：命中行组并行装载；缺失转回填（重放本行组）
                            if (l1_hit_w) begin
                                for (i = 0; i < NLANES; i = i + 1)
                                    if ((i >= glane_q) && (i < ge_w) && req_mask_q[i])
                                        rsp_q[i*DATA_W +: DATA_W]
                                            <= data[phys_w]
                                                 [req_addr_q[i][4:2]*DATA_W +: DATA_W];
                                glane_q <= ge_w;
                                if (has_any(ge_w)) state <= S_GRP;
                                else               state <= S_RSP;
                            end else begin
                                memif_rw_q    <= 1'b0;
                                memif_addr_q  <= {row_w, 5'b00000};   // 行对齐
                                memif_wdata_q <= {DATA_W{1'b0}};
                                mreq_done_q   <= 1'b0;
                                state         <= S_REFILL;
                            end
                        end else begin
                            // 写：命中更新行内字（缺失不更新，写直通不写
                            // 分配），随后行组逐 lane 写通（升序）
                            if (l1_hit_w) begin
                                for (i = 0; i < NLANES; i = i + 1)
                                    if ((i >= glane_q) && (i < ge_w) && req_mask_q[i])
                                        data[phys_w]
                                            [req_addr_q[i][4:2]*DATA_W +: DATA_W]
                                            <= req_wdata_q[i];
                            end
                            wt_lane_q     <= wl0_w;
                            memif_rw_q    <= 1'b1;
                            memif_addr_q  <= req_addr_q[wl0_w[2:0]];
                            memif_wdata_q <= req_wdata_q[wl0_w[2:0]];
                            mreq_done_q   <= 1'b0;
                            state         <= S_WT;
                        end
                    end
                end
            end
            // ----------------------------------------------------
            // S_REFILL：回填读保持至握手；响应握手拍整行单沿写入并重放
            // 行组（l1sm_spec §5.4/§6.2）
            // ----------------------------------------------------
            S_REFILL: begin
                if (mreq_fire)
                    mreq_done_q <= 1'b1;
                if (mrsp_fire) begin
                    data[rphys_w]  <= memif_l1sm_rsp_data;
                    tag[rphys_w]   <= rline_w;
                    cls[rphys_w]   <= CLS_L1;
                    valid[rphys_w] <= 1'b1;
                    state          <= S_GRP;   // 游标不动，重放该行组
                end
            end
            // ----------------------------------------------------
            // S_WT：写通逐 lane（升序），每笔等写应答后才发下一笔
            // （l1sm_spec §5.5/§6.2）
            // ----------------------------------------------------
            S_WT: begin
                if (mreq_fire)
                    mreq_done_q <= 1'b1;
                if (mrsp_fire) begin
                    if (wnext_w != 4'd8) begin
                        wt_lane_q     <= wnext_w;
                        memif_rw_q    <= 1'b1;
                        memif_addr_q  <= req_addr_q[wnext_w[2:0]];
                        memif_wdata_q <= req_wdata_q[wnext_w[2:0]];
                        mreq_done_q   <= 1'b0;
                    end else begin
                        glane_q <= grp_end_q;
                        if (has_any(grp_end_q)) state <= S_GRP;
                        else                    state <= S_RSP;
                    end
                end
            end
            // ----------------------------------------------------
            // S_RSP：响应保持至握手（l1sm_spec §5.6）
            // ----------------------------------------------------
            S_RSP: begin
                if (rsp_fire) state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
