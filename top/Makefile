# =============================================================================
#  easy_simt · top/ 统一 Makefile
#
#  已实现：C 事务级模型（cmodel/）编译与黄金回归
#  预留  ：RTL 仿真；C 模型经 DPI-C 作为参考模型接入 SystemVerilog testbench
#
#  分层约定（为将来 DPI 接入预留）：
#    cmodel/*.c（不含 main.c）      -> build/sim/libeasy_simt_sim.a  模型库
#    cmodel/main.c                  -> build/sim/easy_simt_sim        独立回归前端
#    cmodel/dpi_ref.c（将来）       -> build/dpi/*.so                 SV TB 参考模型前端（预留）
#
#  常用命令：
#    make / make sim        编译模型库 + 回归可执行
#    make sim_run           黄金回归（默认 N=1000 WARPS=4 LANES=8 MEMLAT=20）
#    make sim_dbg           ASan/UBSan 调试编译
#    make sim_run_dbg       调试版回归
#    make help              查看全部目标（含预留）
#    make clean
#
#  回归输入：kernel 镜像（kernel/easy_simt_kernel.hex），
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

SIM_DIR   := cmodel
BUILD     := build/sim
BUILD_DBG := build/sim_dbg

# 黄金回归参数（ma_spec §1.7 T3 硬件版口径，与模型编译参数一致，可覆盖）
KERNEL ?= kernel/easy_simt_kernel.hex
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

.PHONY: all sim sim_run sim_dbg sim_run_dbg clean help \
        rtl rtl_clean wave dpi cosim

all: sim

# ===========================================================================
#  C 事务级模型（cmodel/）
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
#  【预留】以下目标仅占位，实现时填充；当前调用会明确报未实现
# ===========================================================================

# RTL 仿真：iverilog/vvp（或厂商工具）编译运行，入口预计为 tb/ 下 testbench
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
	@echo "[预留] dpi：模型库编译为 DPI-C .so（第二前端 cmodel/dpi_ref.c），未实现"
	@exit 1

# RTL 与 C 参考模型对比回归（依赖 rtl + dpi）
cosim:
	@echo "[预留] cosim：RTL 与 C 参考模型对比回归，未实现"
	@exit 1

# ===========================================================================

clean:
	rm -rf build

help:
	@echo "easy_simt top/ Makefile"
	@echo ""
	@echo "已实现："
	@echo "  sim / all      编译模型库 + 回归可执行 ($(BIN))"
	@echo "  sim_run        黄金回归  KERNEL=$(KERNEL) N=$(N) WARPS=$(WARPS) LANES=$(LANES) MEMLAT=$(MEMLAT)"
	@echo "  sim_dbg        ASan/UBSan 编译"
	@echo "  sim_run_dbg    调试版回归"
	@echo "  clean          清理 build/"
	@echo ""
	@echo "预留（未实现）："
	@echo "  rtl / rtl_clean  RTL 仿真（iverilog/vvp 或厂商工具）"
	@echo "  wave             波形"
	@echo "  dpi              C 模型编为 DPI-C 参考模型 .so"
	@echo "  cosim            RTL 与参考模型对比回归"
