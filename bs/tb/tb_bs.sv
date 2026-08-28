// =============================================================================
// easy_simt · bs 单元验证 testbench
//
// 参考模型：top/cmodel（bs.c 的 bs_step）经 DPI-C 接入（top/cmodel/dpi_ref.c）。
// 比对方式：事务级等价（ma_spec §1.7：周期只作观测项，不作验收项）——
//   testbench 以同一组消费者决策（sf/ws 的 launch rdy、block_done 注入）
//   分别驱动 DUT 与参考模型，两侧各通道发射序列经计分板逐笔比对（载荷
//   位精确），并检查握手协议（vld 保持/载荷稳定/复位行为）与 bs_top_done。
//
// 验证要点与 bs/docs/bs_spec_v0.1.md §9 的 B1–B7 一一对应。
// =============================================================================
`timescale 1ns/1ps

module tb_bs;

  // ---------------- DPI-C 参考模型（top/cmodel/dpi_ref.c） ----------------
  import "DPI-C" function void ref_bs_init(input int n);
  import "DPI-C" function void ref_bs_cycle(
      input  int sf_rdy, input int ws_rdy,
      input  int bdone_vld, input int bdone_idx,
      output int ref_sf_fire, output int ref_ws_fire,
      output int ref_bdone_consumed, output int ref_done, output int ref_inflight,
      output int ref_sf_block_idx, output int ref_sf_n, output int ref_sf_shbase,
      output int ref_ws_block_idx, output int ref_block_idx);

  // ---------------- 时钟与 DUT 接口 ----------------
  reg clk;
  initial clk = 1'b0;
  always #5 clk = ~clk;                 // 100MHz

  reg          rst_n;
  reg  [31:0]  cfg_n;

  wire         bs_sf_launch_vld;
  wire [31:0]  bs_sf_launch_block_idx, bs_sf_launch_n, bs_sf_launch_shbase;
  reg          sf_bs_launch_rdy;

  wire         bs_ws_launch_vld;
  wire [31:0]  bs_ws_launch_block_idx;
  reg          ws_bs_launch_rdy;

  reg          ws_bs_bdone_vld;
  reg  [31:0]  ws_bs_bdone_block_idx;
  wire         bs_ws_bdone_rdy;
  wire         bs_top_done;

  bs dut (
      .clk(clk),
      .rst_n(rst_n),
      .bs_cfg_n(cfg_n),
      .bs_sf_launch_vld(bs_sf_launch_vld),
      .bs_sf_launch_block_idx(bs_sf_launch_block_idx),
      .bs_sf_launch_n(bs_sf_launch_n),
      .bs_sf_launch_shbase(bs_sf_launch_shbase),
      .sf_bs_launch_rdy(sf_bs_launch_rdy),
      .bs_ws_launch_vld(bs_ws_launch_vld),
      .bs_ws_launch_block_idx(bs_ws_launch_block_idx),
      .ws_bs_launch_rdy(ws_bs_launch_rdy),
      .ws_bs_bdone_vld(ws_bs_bdone_vld),
      .ws_bs_bdone_block_idx(ws_bs_bdone_block_idx),
      .bs_ws_bdone_rdy(bs_ws_bdone_rdy),
      .bs_top_done(bs_top_done)
  );

  // ---------------- 计分板：参考发射序列（先入）对 DUT 发射（后出） ----------------
  localparam SB_DEPTH = 4096;
  reg [31:0] sb_sf_idx [0:SB_DEPTH-1];
  reg [31:0] sb_sf_n   [0:SB_DEPTH-1];
  reg [31:0] sb_sf_shb [0:SB_DEPTH-1];
  integer    sb_sf_head, sb_sf_tail;
  reg [31:0] sb_ws_idx [0:SB_DEPTH-1];
  integer    sb_ws_head, sb_ws_tail;

  // ---------------- 测试配置与响应器状态 ----------------
  integer sf_mode, sf_pct, sf_on, sf_off;   // 0 恒就绪 / 1 随机 / 2 周期
  integer ws_mode, ws_pct, ws_on, ws_off;
  integer bd_lo, bd_hi;                     // block_done 注入延迟区间（拍）
  integer seed;
  integer sf_pat_cnt, ws_pat_cnt;

  // ---------------- 运行状态 ----------------
  integer cyc, errors, total_errors, test_idx;
  integer ref_done_flag, ref_inflight_r;
  integer dut_done, dut_done_cyc, ref_done_cyc;
  integer dut_launch_cnt;                   // 自上次 bdone 后已发射 launch 数
  integer cur_block_idx;                    // 在途块索引（取自 DUT ws_launch 载荷）
  integer bd_armed, bd_cnt, bd_active, bd_fire_last;
  integer sf_fires, ws_fires, bd_fires, exp_blocks;
  integer sf_rdy_dec, ws_rdy_dec, bd_vld_dec;
  integer rd_sf_fire, rd_ws_fire, rd_bd_consumed, rd_done, rd_inflight;
  integer rd_sf_idx, rd_sf_n, rd_sf_shb, rd_ws_idx, rd_block_idx;
  // 协议检查用上一拍采样
  integer prev_sf_vld, prev_sf_rdy;
  reg [31:0] prev_sf_idx, prev_sf_n, prev_sf_shb;
  integer prev_ws_vld, prev_ws_rdy;
  reg [31:0] prev_ws_idx;

  function automatic integer rnd;
    begin
      seed = seed * 1103515245 + 12345;
      rnd  = (seed >> 16) & 32767;
    end
  endfunction

  task err(input string msg);
    begin
      errors      = errors + 1;
      total_errors = total_errors + 1;
      if (errors <= 10)
        $display("[bs-tb][ERR] cyc=%0d %s", cyc, msg);
    end
  endtask

  task sb_push_sf(input integer i, input integer n, input integer s);
    begin
      if (sb_sf_tail - sb_sf_head >= SB_DEPTH) err("sf scoreboard overflow");
      sb_sf_idx[sb_sf_tail % SB_DEPTH] = i;
      sb_sf_n[sb_sf_tail % SB_DEPTH]   = n;
      sb_sf_shb[sb_sf_tail % SB_DEPTH] = s;
      sb_sf_tail = sb_sf_tail + 1;
    end
  endtask

  task sb_push_ws(input integer i);
    begin
      if (sb_ws_tail - sb_ws_head >= SB_DEPTH) err("ws scoreboard overflow");
      sb_ws_idx[sb_ws_tail % SB_DEPTH] = i;
      sb_ws_tail = sb_ws_tail + 1;
    end
  endtask

  task sb_pop_sf(input integer i, input integer n, input integer s);
    begin
      if (sb_sf_head >= sb_sf_tail) begin
        err("DUT bs_sf_launch fired without ref transaction");
      end else begin
        if (sb_sf_idx[sb_sf_head % SB_DEPTH] !== i ||
            sb_sf_n[sb_sf_head % SB_DEPTH]   !== n ||
            sb_sf_shb[sb_sf_head % SB_DEPTH] !== s)
          err($sformatf("bs_sf_launch payload mismatch: dut{%0d,%0d,%0d} ref{%0d,%0d,%0d}",
                        i, n, s,
                        sb_sf_idx[sb_sf_head % SB_DEPTH],
                        sb_sf_n[sb_sf_head % SB_DEPTH],
                        sb_sf_shb[sb_sf_head % SB_DEPTH]));
        sb_sf_head = sb_sf_head + 1;
      end
    end
  endtask

  task sb_pop_ws(input integer i);
    begin
      if (sb_ws_head >= sb_ws_tail) begin
        err("DUT bs_ws_launch fired without ref transaction");
      end else begin
        if (sb_ws_idx[sb_ws_head % SB_DEPTH] !== i)
          err($sformatf("bs_ws_launch payload mismatch: dut %0d ref %0d",
                        i, sb_ws_idx[sb_ws_head % SB_DEPTH]));
        sb_ws_head = sb_ws_head + 1;
      end
    end
  endtask

  // ---------------- 响应器决策（与参考模型共用同一决策） ----------------
  function automatic integer rdy_dec(input integer mode, input integer pct,
                                     input integer on, input integer off,
                                     inout integer pat_cnt);
    begin
      case (mode)
        0: rdy_dec = 1;
        1: rdy_dec = (rnd() % 100) < pct ? 1 : 0;
        2: begin
          if (pat_cnt < on) rdy_dec = 1;
          else              rdy_dec = 0;
          pat_cnt = pat_cnt + 1;
          if (pat_cnt == on + off) pat_cnt = 0;
        end
        default: rdy_dec = 1;
      endcase
    end
  endfunction

  // ---------------- 单个用例 ----------------
  task run_test(input string name, input integer n,
                input integer sf_m, input integer sf_p,
                input integer sf_o, input integer sf_f,
                input integer ws_m, input integer ws_p,
                input integer ws_o, input integer ws_f,
                input integer blo, input integer bhi,
                input integer seed0);
    integer cap, quiet, i;
    integer finished;
    begin
      // ---- 用例配置 ----
      sf_mode = sf_m; sf_pct = sf_p; sf_on = sf_o; sf_off = sf_f;
      ws_mode = ws_m; ws_pct = ws_p; ws_on = ws_o; ws_off = ws_f;
      bd_lo = blo; bd_hi = bhi; seed = seed0;
      sf_pat_cnt = 0; ws_pat_cnt = 0;
      errors = 0;
      exp_blocks = (n + 31) / 32;
      cap = 400 * exp_blocks + 2000;

      // ---- 复位与参考模型初始化 ----
      @(negedge clk);
      rst_n = 1'b0; cfg_n = n;
      sf_bs_launch_rdy = 1'b0; ws_bs_launch_rdy = 1'b0;
      ws_bs_bdone_vld = 1'b0; ws_bs_bdone_block_idx = 32'd0;
      ref_bs_init(n);

      sb_sf_head = 0; sb_sf_tail = 0;
      sb_ws_head = 0; sb_ws_tail = 0;
      cyc = 0; ref_done_flag = 0; ref_inflight_r = 0;
      dut_done = 0; dut_done_cyc = 0; ref_done_cyc = 0;
      dut_launch_cnt = 0; cur_block_idx = 0;
      bd_armed = 0; bd_cnt = 0; bd_active = 0; bd_fire_last = 0;
      sf_fires = 0; ws_fires = 0; bd_fires = 0;
      prev_sf_vld = 0; prev_sf_rdy = 0; prev_ws_vld = 0; prev_ws_rdy = 0;

      // 复位保持 5 拍，期间检查两条 launch 的 vld 恒 0（B6）
      repeat (5) begin
        @(posedge clk);
        if (bs_sf_launch_vld || bs_ws_launch_vld)
          err("launch vld asserted during reset");
      end
      @(negedge clk);
      rst_n = 1'b1;

      // ---- 主循环：negedge 决策/驱动/推进参考，posedge 采样/比对 ----
      finished = 0;
      while (!finished && cyc < cap) begin

        @(negedge clk);
        // -- 响应器决策 --
        sf_rdy_dec = rdy_dec(sf_mode, sf_pct, sf_on, sf_off, sf_pat_cnt);
        ws_rdy_dec = rdy_dec(ws_mode, ws_pct, ws_on, ws_off, ws_pat_cnt);

        // -- block_done 注入（门控：DUT 两侧 launch 均已发射、参考有块在途） --
        if (bd_fire_last) begin
          bd_active = 0;                // DUT 已于上一拍消费
          bd_fire_last = 0;
        end
        if (!bd_active && !bd_armed && dut_launch_cnt == 2 && ref_inflight_r == 1) begin
          bd_armed = 1;
          bd_cnt   = bd_lo + (bd_hi > bd_lo ? rnd() % (bd_hi - bd_lo + 1) : 0);
        end
        if (bd_armed) begin
          if (bd_cnt == 0) begin
            bd_active = 1; bd_armed = 0;
          end else
            bd_cnt = bd_cnt - 1;
        end
        bd_vld_dec = bd_active;

        // -- 驱动 DUT 输入 --
        sf_bs_launch_rdy      = sf_rdy_dec[0];
        ws_bs_launch_rdy      = ws_rdy_dec[0];
        ws_bs_bdone_vld       = bd_vld_dec[0];
        ws_bs_bdone_block_idx = cur_block_idx;

        // -- 同一决策推进参考模型，发射入计分板 --
        ref_bs_cycle(sf_rdy_dec, ws_rdy_dec, bd_vld_dec, cur_block_idx,
                     rd_sf_fire, rd_ws_fire, rd_bd_consumed, rd_done, rd_inflight,
                     rd_sf_idx, rd_sf_n, rd_sf_shb, rd_ws_idx, rd_block_idx);
        if (rd_sf_fire) sb_push_sf(rd_sf_idx, rd_sf_n, rd_sf_shb);
        if (rd_ws_fire) sb_push_ws(rd_ws_idx);
        ref_inflight_r = rd_inflight;
        if (rd_done && !ref_done_flag) begin
          ref_done_flag = 1;
          ref_done_cyc  = cyc + 1;
        end

        @(posedge clk);
        cyc = cyc + 1;

        // -- 协议：vld 保持与载荷稳定（B6） --
        if (prev_sf_vld && !prev_sf_rdy) begin
          if (!bs_sf_launch_vld)
            err("bs_sf_launch_vld dropped while backpressured");
          else if (bs_sf_launch_block_idx !== prev_sf_idx ||
                   bs_sf_launch_n         !== prev_sf_n   ||
                   bs_sf_launch_shbase    !== prev_sf_shb)
            err("bs_sf_launch payload changed while backpressured");
        end
        if (prev_ws_vld && !prev_ws_rdy) begin
          if (!bs_ws_launch_vld)
            err("bs_ws_launch_vld dropped while backpressured");
          else if (bs_ws_launch_block_idx !== prev_ws_idx)
            err("bs_ws_launch payload changed while backpressured");
        end

        // -- 发射采样与计分板比对（B1/B2） --
        if (bs_sf_launch_vld && sf_bs_launch_rdy) begin
          sf_fires = sf_fires + 1;
          sb_pop_sf(bs_sf_launch_block_idx, bs_sf_launch_n, bs_sf_launch_shbase);
          dut_launch_cnt = dut_launch_cnt + 1;
        end
        if (bs_ws_launch_vld && ws_bs_launch_rdy) begin
          ws_fires = ws_fires + 1;
          sb_pop_ws(bs_ws_launch_block_idx);
          cur_block_idx = bs_ws_launch_block_idx;
          dut_launch_cnt = dut_launch_cnt + 1;
        end

        // -- block_done 消费（B3） --
        if (ws_bs_bdone_vld && !bs_ws_bdone_rdy)
          err("bs_ws_bdone_rdy not asserted while block_done pending");
        if (ws_bs_bdone_vld && bs_ws_bdone_rdy) begin
          bd_fires = bd_fires + 1;
          if (ws_bs_bdone_block_idx !== cur_block_idx)
            err("ws_bs_bdone payload mismatch vs in-flight block");
          if (dut_launch_cnt != 2)
            err("block_done consumed without both launches fired");
          dut_launch_cnt = 0;
          bd_fire_last = 1;
        end

        // -- bs_top_done（B4） --
        if (bs_top_done && !ref_done_flag)
          err("bs_top_done asserted before ref done");
        if (bs_top_done && !dut_done) begin
          dut_done = 1;
          dut_done_cyc = cyc;
        end

        // -- 上一拍采样 --
        prev_sf_vld = bs_sf_launch_vld; prev_sf_rdy = sf_bs_launch_rdy;
        prev_sf_idx = bs_sf_launch_block_idx;
        prev_sf_n   = bs_sf_launch_n;
        prev_sf_shb = bs_sf_launch_shbase;
        prev_ws_vld = bs_ws_launch_vld; prev_ws_rdy = ws_bs_launch_rdy;
        prev_ws_idx = bs_ws_launch_block_idx;

        // -- 终止条件 --
        if (ref_done_flag && dut_done &&
            sb_sf_head == sb_sf_tail && sb_ws_head == sb_ws_tail)
          finished = 1;
      end

      if (cyc >= cap)
        err("test timeout (cycle cap reached)");

      // ---- 停机后静默检查（B5）：8 拍内 vld 恒 0、done 保持 ----
      if (dut_done)
      for (i = 0; i < 8; i = i + 1) begin
        @(posedge clk);
        cyc = cyc + 1;
        if (bs_sf_launch_vld || bs_ws_launch_vld)
          err("launch vld asserted after done");
        if (!bs_top_done)
          err("bs_top_done deasserted after done");
      end

      // ---- 收尾核对 ----
      if (!ref_done_flag)          err("ref did not reach done");
      if (!dut_done)               err("DUT did not reach bs_top_done");
      if (sb_sf_head != sb_sf_tail) err($sformatf("sf scoreboard not drained: %0d left",
                                                   sb_sf_tail - sb_sf_head));
      if (sb_ws_head != sb_ws_tail) err($sformatf("ws scoreboard not drained: %0d left",
                                                   sb_ws_tail - sb_ws_head));
      if (sf_fires !== exp_blocks) err($sformatf("sf launch fires %0d != grid %0d",
                                                 sf_fires, exp_blocks));
      if (ws_fires !== exp_blocks) err($sformatf("ws launch fires %0d != grid %0d",
                                                 ws_fires, exp_blocks));
      if (bd_fires !== exp_blocks) err($sformatf("bdone fires %0d != grid %0d",
                                                 bd_fires, exp_blocks));
      if (dut_done && ref_done_flag && dut_done_cyc - ref_done_cyc > 8)
        err($sformatf("done latency too large: %0d cycles", dut_done_cyc - ref_done_cyc));

      $display("[bs-tb] %-10s n=%-5d grid=%-4d sf(m/p/o/f)=%0d/%0d/%0d/%0d ws=%0d/%0d/%0d/%0d bd=[%0d,%0d] cyc=%-7d sf=%d ws=%d bd=%d done_lat=%0d -> %s",
               name, n, exp_blocks,
               sf_mode, sf_pct, sf_on, sf_off,
               ws_mode, ws_pct, ws_on, ws_off,
               bd_lo, bd_hi, cyc, sf_fires, ws_fires, bd_fires,
               dut_done_cyc - ref_done_cyc,
               errors == 0 ? "PASS" : "FAIL");
    end
  endtask

  // ---------------- VCD（可选：+vcd=<path>） ----------------
  string vcd_path;
  initial begin
    if ($value$plusargs("vcd=%s", vcd_path)) begin
      $dumpfile(vcd_path);
      $dumpvars(0, tb_bs);
    end
  end

  // ---------------- 测试序列（B7：背压组合覆盖） ----------------
  initial begin
    total_errors = 0;
    $display("[bs-tb] ref model: top/cmodel bs_step via DPI-C (dpi_ref.c)");

    // 黄金口径：N=1000 grid=32（ma_spec §1.7）
    run_test("golden",    1000, 0,100,0,0,  0,100,0,0,   0, 3,  32'd1);
    // 单块 / 双块边界
    run_test("single",       1, 0,100,0,0,  0,100,0,0,   0, 0,  32'd2);
    run_test("exact2",      64, 0,100,0,0,  0,100,0,0,   0, 2,  32'd3);
    // 背靠背零延迟（两侧恒就绪，block_done 无延迟）
    run_test("b2b",       3200, 0,100,0,0,  0,100,0,0,   0, 0,  32'd4);
    // 随机背压（两侧独立）
    run_test("rand50",    1000, 1, 50,0,0,  1, 50,0,0,   0, 8,  32'd5);
    run_test("rand20",    1000, 1, 20,0,0,  1, 70,0,0,   2,20,  32'd6);
    // 周期背压（长停顿）
    run_test("pat",       1000, 2,100,3,11, 2,100,5,7,   0, 5,  32'd7);
    // 两侧错峰：sf 慢 / ws 慢
    run_test("sf_slow",   1000, 2,100,1,15, 0,100,0,0,   0, 4,  32'd8);
    run_test("ws_slow",   1000, 0,100,0,0,  2,100,1,15,  1, 6,  32'd9);
    // 大延迟 block_done
    run_test("bd_slow",    500, 0,100,0,0,  0,100,0,0,  30,60,  32'd10);

    if (total_errors == 0)
      $display("[bs-tb] SIM PASS (10 tests, 0 errors)");
    else
      $display("[bs-tb] SIM FAIL (%0d errors)", total_errors);
    $finish;
  end

endmodule
