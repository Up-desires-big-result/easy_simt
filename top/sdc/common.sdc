# =============================================================================
#  easy_simt · 通用时序约束（SDC，TCL 语法）
#  所有模块共用（intf_spec §1.3：各模块统一携带 clk/rst_n，clk 为全片时钟）
#
#  消费方式：
#    - make syn <模块>：从本文件解析时钟周期，施加为 abc -D <ps>（yosys
#      仅消费时钟周期；IO 延迟约束供后续 STA / 商用综合消费）
#    - 顶层综合 / STA（DC、OpenROAD、OpenSTA）接入时以本文件为基础扩展
#
#  约束依据：
#    - 时钟周期 10 ns（100 MHz）：与 testbench 时钟一致（各 tb 的
#      clk 周期为 10 ns），原型机基线目标。
#    - IO 延迟 max 3 ns（周期的 30%）：模块边界约定，为上下游模块的
#      接口寄存器与互连留出裕量。
#    - 端口用通用集合（all_inputs / all_outputs）表达，不逐模块枚举
#      端口清单。rst_n 为异步复位，此处按普通输入约束；严格流程可
#      另行单独处理（如 set_false_path 或专用 -clock 定义）。
# =============================================================================

# ---- 时钟 ----
create_clock -period 10 [get_ports {clk}]

# ---- 输入延迟（相对 clk，max 3 ns；时钟端口自身除外）----
set_input_delay -clock [get_clocks clk] -max 3 \
    [remove_from_collection [all_inputs] [get_ports {clk}]]

# ---- 输出延迟（相对 clk，max 3 ns）----
set_output_delay -clock [get_clocks clk] -max 3 [all_outputs]
