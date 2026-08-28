# =============================================================================
#  easy_simt · 仓库根统一 Makefile
#
#  目录约定（详见 README）：
#    top/       顶层单元 + 工具目录：docs/（三份规范）/ rtl/ / tb/ /
#               cmodel/ / assembler/ / kernel/
#    <模块>/    每个硬件子模块与 top 同级，镜像 docs/ + rtl/ + tb/ 结构
#               模块集合：sf ws bs ialu falu lsu icache l1sm memif rf
#    tmp/       一切编译 / 综合 / 仿真的中间产物与报告，与 top 同级；
#               make clean 清空整个 tmp/（此目录不入库）
#
#  已实现：C 事务级模型（top/cmodel/）编译与黄金回归
#  预留  ：RTL 仿真、门级综合（Yosys + nangate45）、C 模型经 DPI-C 接入 SV testbench
#
#  分层约定（为将来 DPI 接入预留）：
#    top/cmodel/*.c（不含 main.c）  -> tmp/build/sim/libeasy_simt_sim.a  模型库
#    top/cmodel/main.c              -> tmp/build/sim/easy_simt_sim        独立回归前端
#    top/cmodel/dpi_ref.c（将来）   -> tmp/dpi/*.so                       SV TB 参考模型前端（预留）
#
#  常用命令（均在仓库根执行）：
#    make / make sim        编译模型库 + 回归可执行（-> tmp/build/sim/）
#    make sim_run           黄金回归（默认 N=1000 WARPS=4 LANES=8 MEMLAT=20）
#    make sim_dbg           ASan/UBSan 调试编译（-> tmp/build/sim_dbg/）
#    make sim_run_dbg       调试版回归
#    make syn <模块>        门级综合（预留：需 <模块>/rtl/ 下 RTL 与 $PDK_ROOT/nangate45）
#    make help              查看全部目标（含预留）
#    make clean             清空 tmp/
#
#  回归输入：kernel 镜像（top/kernel/easy_simt_kernel.hex），
#  BRT 由回归程序自同名 .json 自动装载。
# =============================================================================

CC     ?= gcc
AR     ?= ar
STD    := -std=c99
WARN   := -Wall -Wextra
FP     := -ffp-contract=off          # 对应 PTX 侧 -fmad=false，保证浮点位精确
# 调试构建：老编译器（<4.9）不支持合并写法，可 SAN=-fsanitize=address 覆盖；
# 亦可整体换编译器：make sim_dbg CC=gcc-9
SAN    ?= -fsanitize=address,undefined
CFLAGS := $(STD) $(WARN) $(FP)
LDLIBS := -lm

# ---- 目录 ----
TOP       := top
SIM_DIR   := $(TOP)/cmodel
TMP       := tmp
BUILD     := $(TMP)/build/sim
BUILD_DBG := $(TMP)/build/sim_dbg
# 综合产物：每模块落 $(SYN_DIR)/<模块>/（综合预留）
SYN_DIR   := $(TMP)/syn

# ---- 硬件子模块清单（与 ma_spec §1.4 一致；每个模块目录镜像 docs/rtl/tb）----
MODULES := sf ws bs ialu falu lsu icache l1sm memif rf

# 黄金回归参数（ma_spec §1.7 easy_simt 硬件口径，与模型编译参数一致，可覆盖）
KERNEL ?= $(TOP)/kernel/easy_simt_kernel.hex
N      ?= 1000
WARPS  ?= 4
LANES  ?= 8
MEMLAT ?= 20

CORE_SRCS := $(filter-out $(SIM_DIR)/main.c,$(wildcard $(SIM_DIR)/*.c))
CORE_OBJS := $(patsubst $(SIM_DIR)/%.c,$(BUILD)/%.o,$(CORE_SRCS))
DBG_OBJS  := $(patsubst $(SIM_DIR)/%.c,$(BUILD_DBG)/%.o,$(CORE_SRCS))
LIB       := $(BUILD)/libeasy_simt_sim.a
BIN       := $(BUILD)/easy_simt_sim
BIN_DBG   := $(BUILD_DBG)/easy_simt_sim_dbg

# 综合对象：make syn bs 中出现在目标里的模块名即为 $(MOD)
MOD := $(filter $(MODULES),$(MAKECMDGOALS))

.PHONY: all sim sim_run sim_dbg sim_run_dbg clean help \
        syn rtl rtl_clean wave dpi cosim \
        $(MODULES)

# 允许模块名单独作为目标出现（供 $(MOD) 抓取），本身不做任何事
$(MODULES): ;

all: sim

# ===========================================================================
#  C 事务级模型（top/cmodel/）
# ===========================================================================
sim: $(BIN)

$(LIB): $(CORE_OBJS)
	$(AR) rcs $@ $^

$(BUILD)/%.o: $(SIM_DIR)/%.c $(SIM_DIR)/sim_common.h | $(BUILD)
	$(CC) $(CFLAGS) -O2 -c -o $@ $<

$(BUILD)/main.o: $(SIM_DIR)/main.c $(SIM_DIR)/sim_common.h | $(BUILD)
	$(CC) $(CFLAGS) -O2 -c -o $@ $<

$(BIN): $(BUILD)/main.o $(LIB)
	$(CC) $(CFLAGS) -O2 -o $@ $^ $(LDLIBS)

$(BUILD):
	mkdir -p $@

sim_run: $(BIN)
	./$(BIN) $(KERNEL) --n $(N) --warps $(WARPS) --lanes $(LANES) --memlat $(MEMLAT)

# ---- 调试（ASan + UBSan）----
sim_dbg: $(BIN_DBG)

$(BUILD_DBG)/%.o: $(SIM_DIR)/%.c $(SIM_DIR)/sim_common.h | $(BUILD_DBG)
	$(CC) $(CFLAGS) -O0 -g $(SAN) -c -o $@ $<

$(BUILD_DBG)/main.o: $(SIM_DIR)/main.c $(SIM_DIR)/sim_common.h | $(BUILD_DBG)
	$(CC) $(CFLAGS) -O0 -g $(SAN) -c -o $@ $<

$(BIN_DBG): $(DBG_OBJS) $(BUILD_DBG)/main.o
	$(CC) $(CFLAGS) -O0 -g $(SAN) -o $@ $^ $(LDLIBS)

$(BUILD_DBG):
	mkdir -p $@

sim_run_dbg: $(BIN_DBG)
	./$(BIN_DBG) $(KERNEL) --n $(N) --warps $(WARPS) --lanes $(LANES) --memlat $(MEMLAT)

# ===========================================================================
#  【预留】门级综合 / RTL 仿真 / 波形 / DPI / 协同仿真
#  当前调用会明确报未实现；实现时产物统一落 tmp/ 下。
# ===========================================================================

# 门级综合：make syn <模块>，读 <模块>/rtl/*.v(.sv)，经 Yosys + nangate45 综合，
# 网表与面积/时序报告落 $(SYN_DIR)/<模块>/。工艺库见 README「工艺库（nangate45）」。
syn:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make syn <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能综合一个模块，收到：$(MOD)"; exit 1; \
	else \
	  echo "[预留] syn $(MOD)：门级综合需 $(MOD)/rtl/ 下的 RTL 与 nangate45 工艺库（见 README），尚未实现。"; \
	  echo "        实现后：Yosys 读 $(MOD)/rtl/*.v(.sv) 综合，产物落 $(SYN_DIR)/$(MOD)/。"; \
	  exit 1; \
	fi

# RTL 仿真：iverilog/vvp（或厂商工具）编译运行，入口预计为 <模块>/tb/ 下 testbench
rtl:
	@echo "[预留] rtl：RTL 仿真编译+运行，未实现"
	@exit 1

rtl_clean:
	@echo "[预留] rtl_clean：清理 RTL 仿真产物，未实现"
	@exit 1

# 波形
wave:
	@echo "[预留] wave：生成/查看 RTL 仿真波形，未实现"
	@exit 1

# C 模型编译为 DPI-C 参考模型共享对象，供 SystemVerilog testbench 调用
dpi:
	@echo "[预留] dpi：模型库编译为 DPI-C .so（第二前端 $(SIM_DIR)/dpi_ref.c），未实现"
	@exit 1

# RTL 与 C 参考模型对比回归（依赖 rtl + dpi）
cosim:
	@echo "[预留] cosim：RTL 与 C 参考模型对比回归，未实现"
	@exit 1

# ===========================================================================

clean:
	rm -rf $(TMP)

help:
	@echo "easy_simt 仓库根 Makefile（命令均在仓库根执行；产物统一落 tmp/）"
	@echo ""
	@echo "已实现："
	@echo "  sim / all      编译模型库 + 回归可执行 ($(BIN))"
	@echo "  sim_run        黄金回归  KERNEL=$(KERNEL) N=$(N) WARPS=$(WARPS) LANES=$(LANES) MEMLAT=$(MEMLAT)"
	@echo "  sim_dbg        ASan/UBSan 编译"
	@echo "  sim_run_dbg    调试版回归"
	@echo "  clean          清空 tmp/"
	@echo ""
	@echo "预留（未实现）："
	@echo "  syn <模块>     门级综合（Yosys + nangate45），模块 ∈ { $(MODULES) }"
	@echo "  rtl / rtl_clean  RTL 仿真（iverilog/vvp 或厂商工具）"
	@echo "  wave             波形"
	@echo "  dpi              C 模型编为 DPI-C 参考模型 .so"
	@echo "  cosim            RTL 与参考模型对比回归"
