# =============================================================================
#  easy_simt · bs（Block Scheduler）时序约束（SDC，TCL 语法）
#
#  消费方式：
#    - make syn bs：从本文件解析时钟周期，施加为 abc -D <ps>（yosys 仅
#      消费时钟周期；IO 延迟约束供后续 STA / 商用综合消费）。
#    - 顶层综合 / STA（DC、OpenROAD、OpenSTA）接入时以本文件为基础扩展。
#
#  约束依据：
#    - 时钟周期 10 ns（100 MHz）：与 testbench 时钟一致（tb_bs.sv 的
#      clk 周期为 10 ns），原型机基线目标。
#    - IO 延迟 max 3 ns（周期的 30%）：模块边界约定，为上下游模块的
#      接口寄存器与互连留出裕量。
# =============================================================================

# ---- 时钟 ----
# bs 唯一时钟域：全片时钟 clk（intf_spec §1.3），上升沿有效
create_clock -period 10 [get_ports {clk}]

# ---- 输入延迟（相对 clk，max 3 ns）----
# rst_n 为异步复位，此处按普通输入约束；严格流程可另行单独处理
# （如 set_false_path 或专用 -clock 定义）
set_input_delay -clock [get_clocks clk] -max 3 [get_ports { \
    rst_n \
    bs_cfg_n[*] \
    sf_bs_launch_rdy \
    ws_bs_launch_rdy \
    ws_bs_bdone_vld \
    ws_bs_bdone_block_idx[*] \
}]

# ---- 输出延迟（相对 clk，max 3 ns）----
set_output_delay -clock [get_clocks clk] -max 3 [get_ports { \
    bs_sf_launch_vld \
    bs_sf_launch_block_idx[*] \
    bs_sf_launch_n[*] \
    bs_sf_launch_shbase[*] \
    bs_ws_launch_vld \
    bs_ws_launch_block_idx[*] \
    bs_ws_bdone_rdy \
    bs_top_done \
}]
