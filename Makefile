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
#  已实现：C 事务级模型（top/cmodel/）编译与黄金回归；内核镜像自 top/kernel/*.cu 全链生成；
#          RTL 仿真与 DPI-C 协同仿真（VCS，make rtl/cosim <模块>）
#  预留  ：门级综合（Yosys + nangate45）
#
#  内核单一源：top/kernel 只存 CUDA 源码 easy_simt_kernel.cu；
#    .ptx/.hex/.json/.lst 一律由本 Makefile 现场生成到 tmp/kernel/，不入库：
#      nvcc -ptx -arch=$(PTX_ARCH) -fmad=false  .cu -> .ptx
#      python3 top/assembler/easy_simt_assembler.py  .ptx -> .hex/.json/.lst
#
#  分层约定（为将来 DPI 接入预留）：
#    top/cmodel/*.c（不含 main.c、dpi_ref.c） -> tmp/build/sim/libeasy_simt_sim.a  模型库
#    top/cmodel/main.c              -> tmp/build/sim/easy_simt_sim        独立回归前端
#    top/cmodel/dpi_ref.c           -> SV testbench 的 DPI-C 参考模型前端（随 simv 编译；
#                                       make dpi 另作独立 .so 编译校验）
#
#  常用命令（均在仓库根执行）：
#    make / make sim        编译模型库 + 回归可执行（-> tmp/build/sim/）
#    make kernel            自 top/kernel/*.cu 生成内核镜像到 tmp/kernel/（.ptx/.hex/.json/.lst）
#    make sim_run           黄金回归（依赖内核镜像，默认 N=1000 WARPS=4 LANES=8 MEMLAT=20）
#    make sim_dbg           ASan/UBSan 调试编译（-> tmp/build/sim_dbg/）
#    make sim_run_dbg       调试版回归
#    make syn <模块>        门级综合（预留：需 <模块>/rtl/ 下 RTL 与 $PDK_ROOT/nangate45）
#    make rtl <模块>        RTL 仿真（VCS；testbench 为 <模块>/tb/tb_<模块>.sv，WAVE=1 出 VCD）
#    make cosim <模块>      同 rtl（参考模型经 DPI-C 随 simv 一并编译，语义别名）
#    make wave <模块>       波形查看提示
#    make verdi <模块>      VCD 转 FSDB 并拉起 nWave 波形浏览器（缺 VCD 自动生成）
#    make help              查看全部目标（含预留）
#    make clean             清空 tmp/
#
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

# ---- 内核镜像生成链（.cu 单一源 -> ptx -> hex/json/lst，全部落 tmp/kernel）----
KERNEL_DIR := $(TMP)/kernel
KERNEL_SRC := $(TOP)/kernel/easy_simt_kernel.cu
KERNEL_PTX := $(KERNEL_DIR)/easy_simt_kernel.ptx
KERNEL_HEX := $(KERNEL_DIR)/easy_simt_kernel.hex
ASSEMBLER  := $(TOP)/assembler/easy_simt_assembler.py
NVCC     ?= nvcc
PY       ?= python3
# PTX 目标架构（与 ma_spec 硬件口径一致：.target sm_70）
PTX_ARCH ?= sm_70

# ---- 硬件子模块清单（与 ma_spec §1.4 一致；每个模块目录镜像 docs/rtl/tb）----
MODULES := sf ws bs ialu falu lsu icache l1sm memif rf

# 黄金回归参数（ma_spec §1.7 easy_simt 硬件口径，与模型编译参数一致，可覆盖）
KERNEL ?= $(KERNEL_HEX)
N      ?= 1000
WARPS  ?= 4
LANES  ?= 8
MEMLAT ?= 20

CORE_SRCS := $(filter-out $(SIM_DIR)/main.c $(SIM_DIR)/dpi_ref.c,$(wildcard $(SIM_DIR)/*.c))
CORE_OBJS := $(patsubst $(SIM_DIR)/%.c,$(BUILD)/%.o,$(CORE_SRCS))
DBG_OBJS  := $(patsubst $(SIM_DIR)/%.c,$(BUILD_DBG)/%.o,$(CORE_SRCS))
LIB       := $(BUILD)/libeasy_simt_sim.a
BIN       := $(BUILD)/easy_simt_sim
BIN_DBG   := $(BUILD_DBG)/easy_simt_sim_dbg

# 综合对象：make syn bs 中出现在目标里的模块名即为 $(MOD)
MOD := $(filter $(MODULES),$(MAKECMDGOALS))

.PHONY: all sim kernel sim_run sim_dbg sim_run_dbg clean help \
        syn rtl rtl_clean wave verdi dpi cosim \
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

# ===========================================================================
#  内核镜像生成（.cu 单一源 -> ptx -> hex/json/lst，全部落 tmp/kernel/，不入库）
#  依赖 nvcc（CUDA）与 python3；产物随 tmp/ 一并被 make clean 清空。
# ===========================================================================
kernel: $(KERNEL_HEX)

$(KERNEL_DIR):
	mkdir -p $@

$(KERNEL_PTX): $(KERNEL_SRC) | $(KERNEL_DIR)
	$(NVCC) -ptx -arch=$(PTX_ARCH) -fmad=false $< -o $@

# 汇编器一次写出 hex/json/lst，以 hex 为代表目标
$(KERNEL_HEX): $(KERNEL_PTX) $(ASSEMBLER) | $(KERNEL_DIR)
	$(PY) $(ASSEMBLER) $(KERNEL_PTX) -o $(KERNEL_DIR)

# ===========================================================================

sim_run: $(BIN) $(KERNEL_HEX)
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

sim_run_dbg: $(BIN_DBG) $(KERNEL_HEX)
	./$(BIN_DBG) $(KERNEL) --n $(N) --warps $(WARPS) --lanes $(LANES) --memlat $(MEMLAT)

# ===========================================================================
#  【预留】门级综合（实现时产物统一落 tmp/ 下）
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

# ===========================================================================
#  RTL 仿真与 DPI-C 协同仿真（VCS）
#    make rtl <模块> / make cosim <模块>（二者等价）：
#      RTL 取 <模块>/rtl/*.sv，testbench 取 <模块>/tb/tb_<模块>.sv（顶层模块名
#      tb_<模块>）；C 参考模型 top/cmodel/dpi_ref.c（DPI 前端）与模型源文件
#      （不含 main.c）随 simv 一并编译。产物落 $(RTL_DIR)/<模块>/（.gitignore）。
#      以运行日志中出现 "SIM PASS" 为通过判据。
#    WAVE=1 运行时落 VCD 于 $(RTL_DIR)/<模块>/tb_<模块>.vcd。
#    VCS 环境由 recipe 内 source /opt/synopsys/snop18.sh 提供（license 同该脚本）。
# ===========================================================================
VCS      ?= vcs
VCSFLAGS ?= -full64 -sverilog -timescale=1ns/1ps -debug_access+all \
            -cpp g++-4.8 -cc gcc-4.8 -LDFLAGS -Wl,--no-as-needed +vcs+lic+wait
RTL_DIR  := $(TMP)/rtl

# C 参考模型 DPI 前端（第二前端，与模型库同源）
DPI_SRC  := $(SIM_DIR)/dpi_ref.c

rtl:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make rtl <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能仿真一个模块，收到：$(MOD)"; exit 1; \
	else \
	  mkdir -p $(RTL_DIR)/$(MOD) && cd $(RTL_DIR)/$(MOD) && \
	  { . /opt/synopsys/snop18.sh >/dev/null 2>&1 || true; } && \
	  { command -v $(VCS) >/dev/null 2>&1 || { echo "未找到 vcs：请先配置 Synopsys 环境（/opt/synopsys/snop18.sh）"; exit 1; }; } && \
	  ls $(CURDIR)/$(MOD)/rtl/*.sv >/dev/null 2>&1 || { echo "$(MOD)/rtl/ 下无 RTL"; exit 1; }; \
	  $(VCS) $(VCSFLAGS) -top tb_$(MOD) -o simv -Mdir=csrc \
	    -CFLAGS "-std=gnu99 -I$(CURDIR)/$(SIM_DIR) -ffp-contract=off" \
	    $(CURDIR)/$(MOD)/rtl/*.sv $(CURDIR)/$(MOD)/tb/tb_$(MOD).sv \
	    $(addprefix $(CURDIR)/,$(DPI_SRC) $(CORE_SRCS)) \
	  && ./simv $(if $(WAVE),+vcd=tb_$(MOD).vcd) | tee simv.out; \
	  grep -q "SIM PASS" $(CURDIR)/$(RTL_DIR)/$(MOD)/simv.out; \
	fi

# cosim：与 rtl 等价（参考模型经 DPI-C 随 simv 编译），保留预留目标名
cosim:
	@$(MAKE) --no-print-directory rtl $(MOD)

rtl_clean:
	rm -rf $(RTL_DIR)

wave:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make wave <模块>；运行时加 WAVE=1 出波形：make cosim <模块> WAVE=1"; exit 1; \
	else \
	  echo "波形：make cosim $(MOD) WAVE=1，VCD 落 $(RTL_DIR)/$(MOD)/tb_$(MOD).vcd（GTKWave/Verdi 查看）"; \
	fi

# 波形查看：VCD 缺失先生成，转 FSDB 后拉起 nWave（独立波形浏览器，
# 左侧即信号层次树；GUI 后台运行，日志落模块目录）
verdi:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make verdi <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能指定一个模块，收到：$(MOD)"; exit 1; \
	else \
	  if [ ! -f $(RTL_DIR)/$(MOD)/tb_$(MOD).vcd ]; then \
	    echo "未检测到 $(RTL_DIR)/$(MOD)/tb_$(MOD).vcd，先生成：make rtl $(MOD) WAVE=1"; \
	    $(MAKE) --no-print-directory rtl $(MOD) WAVE=1 || exit 1; \
	  fi; \
	  { . /opt/synopsys/snop18.sh >/dev/null 2>&1 || true; }; \
	  command -v nWave    >/dev/null 2>&1 || { echo "未找到 nWave：请先配置 Synopsys 环境（/opt/synopsys/snop18.sh）"; exit 1; }; \
	  command -v vcd2fsdb >/dev/null 2>&1 || { echo "未找到 vcd2fsdb：请先配置 Synopsys 环境（/opt/synopsys/snop18.sh）"; exit 1; }; \
	  cd $(RTL_DIR)/$(MOD) && \
	  vcd2fsdb tb_$(MOD).vcd -o tb_$(MOD).fsdb || exit 1; \
	  echo "拉起 nWave：波形 $(RTL_DIR)/$(MOD)/tb_$(MOD).fsdb（日志：$(RTL_DIR)/$(MOD)/nwave.log）"; \
	  nohup nWave -f tb_$(MOD).fsdb >nwave.log 2>&1 & \
	fi

# DPI 前端独立编译校验（.so）；VCS 路线下参考模型随 simv 编译，不依赖本产物
$(TMP)/dpi:
	mkdir -p $(TMP)/dpi

dpi: $(DPI_SRC) $(CORE_SRCS) $(SIM_DIR)/sim_common.h | $(TMP)/dpi
	$(CC) $(CFLAGS) -O2 -fPIC -shared -I$(SIM_DIR) -o $(TMP)/dpi/libeasy_simt_ref.so \
	  $(DPI_SRC) $(CORE_SRCS) $(LDLIBS)

# ===========================================================================

clean:
	rm -rf $(TMP)

help:
	@echo "easy_simt 仓库根 Makefile（命令均在仓库根执行；产物统一落 tmp/）"
	@echo ""
	@echo "已实现："
	@echo "  sim / all      编译模型库 + 回归可执行 ($(BIN))"
	@echo "  kernel         自 $(KERNEL_SRC) 生成内核镜像到 $(KERNEL_DIR)/ (ptx/hex/json/lst)"
	@echo "  sim_run        黄金回归（先 kernel 后回归）KERNEL=$(KERNEL) N=$(N) WARPS=$(WARPS) LANES=$(LANES) MEMLAT=$(MEMLAT)"
	@echo "  sim_dbg        ASan/UBSan 编译"
	@echo "  sim_run_dbg    调试版回归"
	@echo "  rtl <模块>     RTL 仿真（VCS，testbench 为 <模块>/tb/tb_<模块>.sv）"
	@echo "  cosim <模块>   同 rtl（C 参考模型经 DPI-C 随 simv 编译）"
	@echo "  wave <模块>    波形提示（运行加 WAVE=1 出 VCD）"
	@echo "  verdi <模块>   VCD 转 FSDB 并拉起 nWave 波形浏览器（缺 VCD 自动生成）"
	@echo "  dpi            C 参考模型 DPI 前端独立编译校验（.so）"
	@echo "  rtl_clean      清理 RTL 仿真产物（tmp/rtl/）"
	@echo "  clean          清空 tmp/"
	@echo ""
	@echo "预留（未实现）："
	@echo "  syn <模块>     门级综合（Yosys + nangate45），模块 ∈ { $(MODULES) }"
