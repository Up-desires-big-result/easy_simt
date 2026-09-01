// =============================================================================
// sram_2rw0r0w_32_128_freepdk45 — 综合视图（synthesis view）
//
// 同名宏的仿真用行为模型由 OpenRAM 生成（make sram 产物，tmp/sram/，负沿
// 读写 + 置毒，纯仿真视图）；yosys 的 memory_map 不展开负沿时钟存储
// （"write clock is incompatible with other clocks"），且 nangate45 无负沿
// 触发器，故综合使用本上升沿等效视图：
//
//   - 读：dout <= mem[addr] 于选中拍上升沿更新，次拍起有效——与负沿模型
//     「锁存地址当拍、半拍后出数」对 rf（上升沿采样）呈现的逐拍时序一致；
//   - 写：选中拍上升沿提交，后续读可见性同负沿模型；同拍同址写读经
//     rf 侧旁路合并兜底（对旧值/新值读出均幂等，已验证）；
//   - 无 #延迟、无 X 置毒、无 $display。
//
// 端口与 OpenRAM 模型逐一对应（USE_POWER_PINS 关闭形态）。
// =============================================================================
`timescale 1ns/1ps

module sram_2rw0r0w_32_128_freepdk45(
// Port 0: RW
    clk0,csb0,web0,addr0,din0,dout0,
// Port 1: RW
    clk1,csb1,web1,addr1,din1,dout1
  );

  parameter DATA_WIDTH = 32 ;
  parameter ADDR_WIDTH = 7 ;
  parameter RAM_DEPTH = 1 << ADDR_WIDTH;
  parameter VERBOSE = 1 ; // 仅为例化兼容（行为模型用），综合视图不使用
  parameter T_HOLD = 1 ;
  parameter DELAY = 3 ;

  input  clk0; // clock
  input  csb0; // active low chip select
  input  web0; // active low write control
  input [ADDR_WIDTH-1:0]  addr0;
  input [DATA_WIDTH-1:0]  din0;
  output reg [DATA_WIDTH-1:0]  dout0;

  input  clk1; // clock
  input  csb1; // active low chip select
  input  web1; // active low write control
  input [ADDR_WIDTH-1:0]  addr1;
  input [DATA_WIDTH-1:0]  din1;
  output reg [DATA_WIDTH-1:0]  dout1;

  reg [DATA_WIDTH-1:0] mem [0:RAM_DEPTH-1];

  // Port 0（先于 Port 1，与行为模型语句顺序一致）
  always @(posedge clk0) begin
    if (!csb0) begin
      if (!web0)
        mem[addr0] <= din0;
      else
        dout0 <= mem[addr0];
    end
  end

  // Port 1
  always @(posedge clk1) begin
    if (!csb1) begin
      if (!web1)
        mem[addr1] <= din1;
      else
        dout1 <= mem[addr1];
    end
  end

endmodule
