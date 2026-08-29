# =============================================================================
#  easy_simt · 仓库根统一 Makefile
#
#  目录约定（详见 README）：
#    top/       顶层单元 + 工具目录：docs/（三份规范）/ rtl/ / tb/ /
#               cmodel/ / assembler/ / kernel/
#    submodules/  10 个硬件子模块（sf ws bs ialu falu lsu icache l1sm memif rf），
#               与 top/ 同级；每个镜像 docs/ + rtl/ + tb/ 结构
#    tmp/       一切编译 / 综合 / 仿真的中间产物与报告，与 top 同级；
#               make clean 清空整个 tmp/（此目录不入库）
#
#  已实现：C 事务级模型（top/cmodel/）编译与黄金回归（make cmodel [run]）；
#          内核镜像自 top/kernel/*.cu 全链生成（make kernel）；
#          RTL 仿真与门级仿真（Verilator 单路线，make rtl/netlist [run|gui] <模块>）；
#          门级综合（Yosys + nangate45，make syn <模块>）
#  预留（均只跑顶层）：面积、门级仿真波形（供 power 使用）、功耗（网表+波形）、
#          性能（顶层 kernel 完成 cycle 数）
#
#  内核单一源：top/kernel 只存 CUDA 源码 easy_simt_kernel.cu；
#    .ptx/.hex/.json/.lst 一律由本 Makefile 现场生成到 tmp/kernel/，不入库：
#      nvcc -ptx -arch=$(PTX_ARCH) -fmad=false  .cu -> .ptx
#      python3 top/assembler/easy_simt_assembler.py  .ptx -> .hex/.json/.lst
#
#  分层约定：
#    top/cmodel/*.c（不含 main.c） -> tmp/build/sim/libeasy_simt_sim.a  模型库
#    top/cmodel/main.c              -> tmp/build/sim/easy_simt_sim        独立回归前端
#
#  常用命令（均在仓库根执行）：
#    make cmodel            编译模型库 + 回归可执行（-> tmp/build/sim/）
#    make cmodel run        编译并执行模型黄金回归（默认 N=1000 WARPS=4 LANES=8 MEMLAT=20）
#    make kernel            自 top/kernel/*.cu 生成内核镜像到 tmp/kernel/（.ptx/.hex/.json/.lst）
#    make rtl <模块>        RTL 仿真编译（Verilator 编译 RTL + C++ harness）
#    make rtl run <模块>    RTL 仿真执行（对 C 参考模型事务级比对，判据 VSIM PASS）
#    make rtl gui <模块>    RTL 仿真执行并拉起 gtkwave 看 VCD
#    make syn <模块>        门级综合（Yosys + nangate45，需 PDK_ROOT，产物落 tmp/syn/<模块>/）
#    make netlist <模块>    门级仿真编译（网表 + 单元行为模型 + harness，Verilator）
#    make netlist run <模块> 门级仿真执行（对 C 参考模型事务级比对）
#    make netlist gui <模块> 门级仿真执行并拉起 gtkwave 看 VCD
#    make help              查看全部目标（含预留）
#    make clean             清空 tmp/
#    预留（均只跑顶层）：make area（顶层综合面积）/ make wave（门级仿真波形，供 power 使用）/
#          make power（顶层功耗）/ make perf（顶层 kernel 完成 cycle 数）
#
# =============================================================================

CC     ?= gcc
AR     ?= ar
STD    := -std=c99
WARN   := -Wall -Wextra
FP     := -ffp-contract=off          # 对应 PTX 侧 -fmad=false，保证浮点位精确
CFLAGS := $(STD) $(WARN) $(FP)
LDLIBS := -lm

# ---- 目录 ----
TOP       := top
SUBMOD_DIR := submodules
SIM_DIR   := $(TOP)/cmodel
TMP       := tmp
BUILD     := $(TMP)/build/sim
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

CORE_SRCS := $(filter-out $(SIM_DIR)/main.c,$(wildcard $(SIM_DIR)/*.c))
CORE_OBJS := $(patsubst $(SIM_DIR)/%.c,$(BUILD)/%.o,$(CORE_SRCS))
LIB       := $(BUILD)/libeasy_simt_sim.a
BIN       := $(BUILD)/easy_simt_sim

# 综合/仿真对象：make syn bs 中出现在目标里的模块名即为 $(MOD)
MOD := $(filter $(MODULES),$(MAKECMDGOALS))
# 子命令：make cmodel run / make rtl run bs 中的 run 仅为标记，命中则编译后继续执行
RUN_IT := $(filter run,$(MAKECMDGOALS))
# 子命令：make rtl gui bs / make netlist gui bs 中的 gui：执行仿真（Verilator
# 原生转储 VCD）并拉起 gtkwave 查看
GUI_IT := $(filter gui,$(MAKECMDGOALS))

.PHONY: all cmodel kernel sim_run clean deps help \
        syn netlist area wave power perf rtl \
        $(MODULES) run gui

# 允许模块名单独作为目标出现（供 $(MOD) 抓取），本身不做任何事
$(MODULES): ;

# run 作为 cmodel/rtl/netlist 的子命令出现（供 $(RUN_IT) 抓取），本身不做任何事
run: ;

# gui 作为 rtl/netlist 的子命令出现（供 $(GUI_IT) 抓取），本身不做任何事
gui: ;

all: cmodel

# ===========================================================================
#  C 事务级模型（top/cmodel/）
# ===========================================================================

# cmodel：编译模型库 + 回归可执行；make cmodel run 编译后接着执行黄金回归
cmodel: $(BIN)
	@if [ -n "$(RUN_IT)" ]; then $(MAKE) --no-print-directory sim_run; fi

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

# 内部目标：黄金回归（cmodel run 调用；参数 N/WARPS/LANES/MEMLAT/KERNEL 可覆盖）
sim_run: $(BIN) $(KERNEL_HEX)
	./$(BIN) $(KERNEL) --n $(N) --warps $(WARPS) --lanes $(LANES) --memlat $(MEMLAT)

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
#  RTL 仿真（Verilator，开源单路线）：
#    make rtl <模块>       仅编译（Verilator 把 RTL 编译为 C++ 并链接 harness）
#    make rtl run <模块>   编译并执行；harness tb_<模块>_vsim.cpp 驱动时钟与
#                          消费者决策，参考侧直链 top/cmodel（bs_step），
#                          记分板逐笔比对；判据：日志出现 VSIM PASS
#    make rtl gui <模块>   执行后拉起 gtkwave 查看原生转储的 VCD
#                          （<模块>.vcd，落 $(RTL_DIR)/<模块>/）
# ===========================================================================
RTL_DIR  := $(TMP)/rtl

rtl:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make rtl <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能仿真一个模块，收到：$(MOD)"; exit 1; \
	else \
	  mkdir -p $(RTL_DIR)/$(MOD) && cd $(RTL_DIR)/$(MOD) && \
	  if [ -x $(CURDIR)/third_party/oss-cad-suite/bin/verilator ]; then \
	    VBIN="$(CURDIR)/third_party/oss-cad-suite/bin/verilator"; \
	  elif command -v verilator >/dev/null 2>&1; then VBIN=verilator; \
	  else echo "未找到 verilator（third_party/oss-cad-suite 或 PATH）"; exit 1; fi; \
	  $$VBIN --exe --cc --trace -Wno-fatal --top-module $(MOD) -Mdir . -o vsim_$(MOD) \
	    $(CURDIR)/$(SUBMOD_DIR)/$(MOD)/rtl/$(MOD).sv \
	    $(CURDIR)/$(SUBMOD_DIR)/$(MOD)/tb/tb_$(MOD)_vsim.cpp \
	    $(addprefix $(CURDIR)/,$(CORE_SRCS)) \
	    -CFLAGS "-I$(CURDIR)/$(SIM_DIR)" || exit 1; \
	  $(MAKE) -C . -f V$(MOD).mk CXX=g++-9 CC=gcc-9 LINK=g++-9 || exit 1; \
	  if [ -n "$(RUN_IT)" ] || [ -n "$(GUI_IT)" ]; then \
	    ./vsim_$(MOD) || exit 1; \
	  fi; \
	  if [ -n "$(GUI_IT)" ]; then \
	    if [ -x $(CURDIR)/third_party/oss-cad-suite/bin/gtkwave ]; then \
	      GW="$(CURDIR)/third_party/oss-cad-suite/bin/gtkwave"; else GW=gtkwave; fi; \
	    echo "拉起 gtkwave：$(MOD).vcd（日志：gtkwave.log）"; \
	    nohup $$GW $(MOD).vcd >gtkwave.log 2>&1 & \
	  fi; \
	fi

# ===========================================================================
#  门级综合（Yosys + nangate45）：
#    make syn <模块>   综合，产物落 $(SYN_DIR)/<模块>/（网表 / stat.rpt / syn.log）
#  路径与单元名单约定见 README「工艺库（nangate45）」：库取
#    $(PDK_ROOT)/nangate45/lib/NangateOpenCellLibrary_typical.lib（PDK_ROOT 自行 export）。
#  约束：取通用约束 $(TOP)/sdc/common.sdc（所有模块共用），解析时钟
#    周期施加为 abc -D <ps>（yosys 仅消费时钟周期；IO 延迟供后续
#    STA / 商用综合消费）；无约束文件时面积优先映射。
# ===========================================================================
YOSYS ?= yosys
NANGATE_LIB = $(PDK_ROOT)/nangate45/lib/NangateOpenCellLibrary_typical.lib

# 公共检查片段（供 syn / netlist 复用，展开为 shell 语句）
PDK_CHECK = if [ -z "$(PDK_ROOT)" ]; then \
	      echo "PDK_ROOT 未设置（见 README「工艺库（nangate45）」）"; exit 1; \
	    fi; \
	    [ -f "$(NANGATE_LIB)" ] || { echo "未找到标准单元库：$(NANGATE_LIB)"; exit 1; }
YOSYS_FIND = if command -v $(YOSYS) >/dev/null 2>&1; then YBIN="$(YOSYS)"; \
	     elif [ -x "$(CURDIR)/third_party/oss-cad-suite/bin/yosys" ]; then YBIN="$(CURDIR)/third_party/oss-cad-suite/bin/yosys"; \
	     elif [ -x "$(PDK_ROOT)/oss-cad-suite/bin/yosys" ]; then YBIN="$(PDK_ROOT)/oss-cad-suite/bin/yosys"; \
	     else echo "未找到 yosys（PATH / third_party / $(PDK_ROOT)/oss-cad-suite/bin/yosys）"; exit 1; fi

syn:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make syn <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能综合一个模块，收到：$(MOD)"; exit 1; \
	else \
	  $(PDK_CHECK); \
	  $(YOSYS_FIND); \
	  RTL_FILES="$$(ls $(CURDIR)/$(SUBMOD_DIR)/$(MOD)/rtl/*.sv 2>/dev/null)"; \
	  [ -n "$$RTL_FILES" ] || { echo "$(MOD)/rtl/ 下无 RTL"; exit 1; }; \
	  DONT_USE=$$(sed -n 's/^export DONT_USE_CELLS = //p' $(PDK_ROOT)/nangate45/config.mk); \
	  DU_FLAGS=""; for c in $$DONT_USE; do DU_FLAGS="$$DU_FLAGS -dont_use $$c"; done; \
	  SDC_FILE="$(CURDIR)/$(TOP)/sdc/common.sdc"; \
	  if [ -f "$$SDC_FILE" ]; then \
	    PERIOD=$$(sed -n 's/^[[:space:]]*create_clock.*-period[[:space:]]\{1,\}\([0-9.]\{1,\}\).*/\1/p' "$$SDC_FILE" | head -1); \
	    [ -n "$$PERIOD" ] || { echo "约束文件解析失败（时钟周期）：$(TOP)/sdc/common.sdc"; exit 1; }; \
	    ABC_DELAY=$$(awk "BEGIN{printf \"%d\", $$PERIOD*1000}"); \
	    ABC_CONSTR="-D $$ABC_DELAY"; \
	    echo "约束：$(TOP)/sdc/common.sdc（时钟周期 $${PERIOD} ns -> abc -D $${ABC_DELAY} ps）"; \
	  else \
	    ABC_CONSTR=""; \
	    echo "无约束文件（$(TOP)/sdc/common.sdc），面积优先映射"; \
	  fi; \
	  mkdir -p $(SYN_DIR)/$(MOD); \
	  { echo "read_verilog -sv $$RTL_FILES"; \
	    echo "hierarchy -top $(MOD)"; \
	    echo "proc; opt; memory; opt; techmap; opt"; \
	    echo "dfflibmap -liberty $(NANGATE_LIB) $$DU_FLAGS"; \
	    echo "abc -liberty $(NANGATE_LIB) $$DU_FLAGS $$ABC_CONSTR"; \
	    echo "opt; clean"; \
	    echo "tee -o $(CURDIR)/$(SYN_DIR)/$(MOD)/stat.rpt stat -liberty $(NANGATE_LIB)"; \
	    echo "write_verilog $(CURDIR)/$(SYN_DIR)/$(MOD)/$(MOD)_netlist.v"; \
	  } > $(SYN_DIR)/$(MOD)/syn.ys; \
	  echo "== syn $(MOD)：yosys + nangate45 =="; \
	  $$YBIN -s $(SYN_DIR)/$(MOD)/syn.ys -l $(SYN_DIR)/$(MOD)/syn.log || exit 1; \
	  echo "综合完成：网表与报告落 $(SYN_DIR)/$(MOD)/（$(MOD)_netlist.v / stat.rpt / syn.log）"; \
	fi

# ===========================================================================
#  门级仿真（Verilator）：
#    make netlist <模块>      仅编译（网表 + 单元行为模型 + harness）
#    make netlist run <模块>  编译并执行，对 C 参考模型事务级比对，判据 VSIM PASS
#    make netlist gui <模块>  执行后拉起 gtkwave 查看 VCD
#  网表取 $(SYN_DIR)/<模块>/<模块>_netlist.v（缺失自动先 make syn <模块>）；
#  单元行为模型由 yosys 从 liberty 现场生成，落 $(NETLIST_DIR)/cells_sim.v
#  （仓库不存单元模型）。
# ===========================================================================
NETLIST_DIR := $(TMP)/netlist

netlist:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make netlist <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能仿真一个模块，收到：$(MOD)"; exit 1; \
	else \
	  mkdir -p $(NETLIST_DIR)/$(MOD) && cd $(NETLIST_DIR)/$(MOD) && \
	  if [ ! -f $(CURDIR)/$(SYN_DIR)/$(MOD)/$(MOD)_netlist.v ]; then \
	    echo "未检测到网表，先综合：make syn $(MOD)"; \
	    $(MAKE) --no-print-directory syn $(MOD) || exit 1; \
	  fi; \
	  if [ ! -f $(NETLIST_DIR)/cells_sim.v ]; then \
	    $(PDK_CHECK); \
	    $(YOSYS_FIND); \
	    echo "== 生成单元行为模型（liberty -> cells_sim.v）=="; \
	    $$YBIN -p "read_liberty -ignore_miss_func -ignore_miss_dir -ignore_miss_data_latch $(NANGATE_LIB); write_verilog $(CURDIR)/$(NETLIST_DIR)/cells_sim.v" || exit 1; \
	  fi; \
	  if [ -x $(CURDIR)/third_party/oss-cad-suite/bin/verilator ]; then \
	    VBIN="$(CURDIR)/third_party/oss-cad-suite/bin/verilator"; \
	  elif command -v verilator >/dev/null 2>&1; then VBIN=verilator; \
	  else echo "未找到 verilator（third_party/oss-cad-suite 或 PATH）"; exit 1; fi; \
	  $$VBIN --exe --cc --trace -Wno-fatal --top-module $(MOD) -Mdir . -o vsim_$(MOD) \
	    $(CURDIR)/$(SYN_DIR)/$(MOD)/$(MOD)_netlist.v \
	    $(CURDIR)/$(NETLIST_DIR)/cells_sim.v \
	    $(CURDIR)/$(SUBMOD_DIR)/$(MOD)/tb/tb_$(MOD)_vsim.cpp \
	    $(addprefix $(CURDIR)/,$(CORE_SRCS)) \
	    -CFLAGS "-I$(CURDIR)/$(SIM_DIR)" || exit 1; \
	  $(MAKE) -C . -f V$(MOD).mk CXX=g++-9 CC=gcc-9 LINK=g++-9 || exit 1; \
	  if [ -n "$(RUN_IT)" ] || [ -n "$(GUI_IT)" ]; then \
	    ./vsim_$(MOD) || exit 1; \
	  fi; \
	  if [ -n "$(GUI_IT)" ]; then \
	    if [ -x $(CURDIR)/third_party/oss-cad-suite/bin/gtkwave ]; then \
	      GW="$(CURDIR)/third_party/oss-cad-suite/bin/gtkwave"; else GW=gtkwave; fi; \
	    echo "拉起 gtkwave：$(MOD).vcd（日志：gtkwave.log）"; \
	    nohup $$GW $(MOD).vcd >gtkwave.log 2>&1 & \
	  fi; \
	fi

# ===========================================================================
#  【预留】面积 / 波形 / 功耗 / 性能（均只跑顶层，实现时产物统一落 tmp/ 下）
#  依赖链（实现时按此接入）：
#    area  <- syn top（顶层综合，依赖顶层 RTL）
#    wave  <- syn top 的网表 + 门级 testbench（跑门级仿真出波形）
#    power <- syn top 的网表 + wave 的波形（网表+波形跑功耗报告）
#    perf  <- 内核镜像（kernel）+ 顶层 rtl/tb（顶层仿真统计完成 cycle 数）
# ===========================================================================

# 面积：make area，只跑顶层：打印顶层综合报告的面积
area:
	@echo "[预留] area：顶层面积需顶层综合（make syn top，依赖顶层 RTL），尚未实现。"
	@echo "        实现后：打印顶层综合报告（单元数 / 芯片面积 / 时序占比），报告落 $(SYN_DIR)/top/。"
	@exit 1

# 波形：make wave，只跑顶层：跑门级仿真生成波形，供 power 功耗分析使用
wave:
	@echo "[预留] wave：门级仿真波形需顶层综合后门级网表（先 make syn top）与门级 testbench，尚未实现。"
	@echo "        实现后：跑门级仿真出波形（VCD），产物落 $(TMP)/wave/，供 power 功耗分析使用。"
	@exit 1

# 功耗：make power，只跑顶层，需综合后门级网表 + wave 生成的门级仿真波形
power:
	@echo "[预留] power：功耗需顶层综合后门级网表（先 make syn top）与门级仿真波形（先 make wave），尚未实现。"
	@echo "        实现后：顶层网表 + 波形跑功耗出报告，产物落 $(TMP)/power/。"
	@exit 1

# 性能：make perf，只跑顶层：从第一个块下发到所有块结束，即同一 kernel 跑完的 cycle 数
perf:
	@echo "[预留] perf：kernel 跑完的 cycle 数（第一个块下发到所有块结束），需顶层 rtl + tb 与内核镜像（先 make kernel），尚未实现。"
	@echo "        实现后：顶层仿真统计完成 cycle 数，报告落 $(TMP)/perf/。"
	@exit 1

# ===========================================================================
#  第三方依赖（third_party/，内容不入库，.gitignore 忽略）
#    make deps：一键拉取 GPGPU-Sim 与 OpenROAD-flow-scripts（仅 nangate45
#    平台），并建 nangate45 软链；完成后重新 source setup.sh 即导出
#    PDK_ROOT / GPGPU_SIM_ROOT。等价手动命令见 README「前置条件」。
# ===========================================================================
THIRD_PARTY := third_party
GPGPU_SIM_URL := https://github.com/gpgpu-sim/gpgpu-sim_distribution.git
ORFS_URL := https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts.git

deps:
	mkdir -p $(THIRD_PARTY)
	@if [ -d $(THIRD_PARTY)/gpgpu-sim/.git ]; then \
	  echo "third_party/gpgpu-sim 已存在，跳过克隆"; \
	else \
	  git clone --depth 1 $(GPGPU_SIM_URL) $(THIRD_PARTY)/gpgpu-sim; \
	fi
	@if [ -d $(THIRD_PARTY)/orfs/.git ]; then \
	  echo "third_party/orfs 已存在，跳过克隆"; \
	else \
	  { git clone --filter=tree:0 --no-checkout --depth 1 $(ORFS_URL) $(THIRD_PARTY)/orfs || \
	    { [ -d $$HOME/pdk/orfs/.git ] && \
	      echo "网络克隆失败，改自 ~/pdk/orfs 本地克隆（--shared，对象经 alternates 引用）" && \
	      git clone --shared --no-checkout $$HOME/pdk/orfs $(THIRD_PARTY)/orfs && \
	      git -C $(THIRD_PARTY)/orfs remote set-url origin $(ORFS_URL); }; } && \
	  git -C $(THIRD_PARTY)/orfs sparse-checkout init && \
	  git -C $(THIRD_PARTY)/orfs sparse-checkout set flow/platforms/nangate45 && \
	  git -C $(THIRD_PARTY)/orfs checkout; \
	fi
	ln -sfn orfs/flow/platforms/nangate45 $(THIRD_PARTY)/nangate45
	@echo "deps 完成：重新 source setup.sh 后 PDK_ROOT / GPGPU_SIM_ROOT 指向 third_party/"

# ===========================================================================

clean:
	rm -rf $(TMP)

help:
	@echo "easy_simt 仓库根 Makefile（命令均在仓库根执行；产物统一落 tmp/）"
	@echo ""
	@echo "已实现："
	@echo "  cmodel           编译事务级模型库"
	@echo "  cmodel run       跑模型黄金回归"
	@echo "  kernel           生成内核镜像"
	@echo "  rtl <模块>       RTL 仿真编译"
	@echo "  rtl run <模块>   RTL 仿真执行（对 C 参考模型事务级比对）"
	@echo "  rtl gui <模块>   RTL 仿真执行并看波形"
	@echo "  syn <模块>       门级综合"
	@echo "  netlist <模块>   门级仿真编译"
	@echo "  netlist run <模块> 门级仿真执行（对 C 参考模型事务级比对）"
	@echo "  netlist gui <模块> 门级仿真执行并看波形"
	@echo "  deps             安装第三方依赖"
	@echo "  clean            清空 tmp/"
	@echo ""
	@echo "预留（未实现，均只跑顶层）："
	@echo "  area             顶层综合面积（单元数 / 芯片面积 / 时序占比）"
	@echo "  wave             门级仿真波形（供 power 使用）"
	@echo "  power            网表+波形跑功耗"
	@echo "  perf             kernel 跑完的 cycle 数（第一个块下发到所有块结束）"
