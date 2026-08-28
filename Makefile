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
#          RTL 编译与仿真执行（VCS，make rtl [run|gui] <模块>）；
#          门级综合（Yosys + nangate45）；门级仿真（make netlist [run|gui] <模块>）
#  预留（均只跑顶层）：面积、门级仿真波形（供 power 使用）、功耗（网表+波形）、
#          性能（顶层 kernel 完成 cycle 数）
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
#    make cmodel            编译模型库 + 回归可执行（-> tmp/build/sim/）
#    make cmodel run        编译并执行模型黄金回归（默认 N=1000 WARPS=4 LANES=8 MEMLAT=20）
#    make kernel            自 top/kernel/*.cu 生成内核镜像到 tmp/kernel/（.ptx/.hex/.json/.lst）
#    make rtl <模块>        RTL 编译（VCS；testbench 为 <模块>/tb/tb_<模块>.sv）
#    make rtl run <模块>    RTL 仿真执行（WAVE=1 出 VCD；参考模型经 DPI-C 随 simv 编译）
#    make rtl gui <模块>    RTL 仿真执行并看波形（tb 直出 FSDB，拉起 nWave）
#    make syn <模块>        门级综合（Yosys + nangate45，需 PDK_ROOT，产物落 tmp/syn/<模块>/）
#    make netlist <模块>    门级仿真编译（综合后网表 + 单元行为模型 + 原 testbench）
#    make netlist run <模块> 门级仿真执行（对 C 参考模型事务级比对，WAVE=1 出门级 VCD）
#    make netlist gui <模块> 门级仿真执行并看波形（tb 直出 FSDB，拉起 nWave）
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
# 子命令：make cmodel run / make rtl run bs 中的 run 仅为标记，命中则编译后继续执行
RUN_IT := $(filter run,$(MAKECMDGOALS))
# 子命令：make rtl gui bs / make netlist gui bs 中的 gui：执行仿真（tb 直出
# FSDB）并拉起波形浏览器
GUI_IT := $(filter gui,$(MAKECMDGOALS))

.PHONY: all sim cmodel kernel sim_run sim_dbg sim_run_dbg clean help \
        syn netlist power perf rtl rtl_clean wave verdi dpi cosim \
        $(MODULES) run gui

# 允许模块名单独作为目标出现（供 $(MOD) 抓取），本身不做任何事
$(MODULES): ;

# run 作为 cmodel/rtl 的子命令出现（供 $(RUN_IT) 抓取），本身不做任何事
run: ;

# gui 作为 rtl/netlist 的子命令出现（供 $(GUI_IT) 抓取），本身不做任何事
gui: ;

all: sim

# ===========================================================================
#  C 事务级模型（top/cmodel/）
# ===========================================================================
sim: $(BIN)

# cmodel：sim 的别名；make cmodel run 编译后接着执行黄金回归
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
#  门级综合（Yosys + nangate45）与面积：
#    make syn <模块>   综合，产物落 $(SYN_DIR)/<模块>/（网表 / stat.rpt / syn.log）
#    make area <模块>  打印综合报告中的面积（先综合）
#  路径与单元名单约定见 README「工艺库（nangate45）」：库取
#    $(PDK_ROOT)/nangate45/lib/NangateOpenCellLibrary_typical.lib（PDK_ROOT 自行 export）。
# ===========================================================================
YOSYS ?= yosys
NANGATE_LIB = $(PDK_ROOT)/nangate45/lib/NangateOpenCellLibrary_typical.lib

# 公共检查片段（供 syn / netlist 复用，展开为 shell 语句）
PDK_CHECK = if [ -z "$(PDK_ROOT)" ]; then \
	      echo "PDK_ROOT 未设置（见 README「工艺库（nangate45）」）"; exit 1; \
	    fi; \
	    [ -f "$(NANGATE_LIB)" ] || { echo "未找到标准单元库：$(NANGATE_LIB)"; exit 1; }
YOSYS_FIND = if command -v $(YOSYS) >/dev/null 2>&1; then YBIN="$(YOSYS)"; \
	     elif [ -x "$(PDK_ROOT)/oss-cad-suite/bin/yosys" ]; then YBIN="$(PDK_ROOT)/oss-cad-suite/bin/yosys"; \
	     else echo "未找到 yosys（PATH 或 $(PDK_ROOT)/oss-cad-suite/bin/yosys）"; exit 1; fi

syn:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make syn <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能综合一个模块，收到：$(MOD)"; exit 1; \
	else \
	  $(PDK_CHECK); \
	  $(YOSYS_FIND); \
	  RTL_FILES="$$(ls $(CURDIR)/$(MOD)/rtl/*.sv 2>/dev/null)"; \
	  [ -n "$$RTL_FILES" ] || { echo "$(MOD)/rtl/ 下无 RTL"; exit 1; }; \
	  DONT_USE=$$(sed -n 's/^export DONT_USE_CELLS = //p' $(PDK_ROOT)/nangate45/config.mk); \
	  DU_FLAGS=""; for c in $$DONT_USE; do DU_FLAGS="$$DU_FLAGS -dont_use $$c"; done; \
	  mkdir -p $(SYN_DIR)/$(MOD); \
	  { echo "read_verilog -sv $$RTL_FILES"; \
	    echo "hierarchy -top $(MOD)"; \
	    echo "proc; opt; memory; opt; techmap; opt"; \
	    echo "dfflibmap -liberty $(NANGATE_LIB) $$DU_FLAGS"; \
	    echo "abc -liberty $(NANGATE_LIB) $$DU_FLAGS"; \
	    echo "opt; clean"; \
	    echo "tee -o $(CURDIR)/$(SYN_DIR)/$(MOD)/stat.rpt stat -liberty $(NANGATE_LIB)"; \
	    echo "write_verilog $(CURDIR)/$(SYN_DIR)/$(MOD)/$(MOD)_netlist.v"; \
	  } > $(SYN_DIR)/$(MOD)/syn.ys; \
	  echo "== syn $(MOD)：yosys + nangate45 =="; \
	  $$YBIN -s $(SYN_DIR)/$(MOD)/syn.ys -l $(SYN_DIR)/$(MOD)/syn.log || exit 1; \
	  echo "综合完成：网表与报告落 $(SYN_DIR)/$(MOD)/（$(MOD)_netlist.v / stat.rpt / syn.log）"; \
	fi

# ===========================================================================
#  门级仿真（netlist 仿真）：
#    make netlist <模块>      仅编译（综合后网表 + 单元行为模型 + 原 testbench）
#    make netlist run <模块>  编译并执行（同一 testbench 对 C 参考模型事务级
#                             比对，判据同 rtl run：SIM PASS）
#  网表取 $(SYN_DIR)/<模块>/<模块>_netlist.v（缺失自动先 make syn <模块>）；
#  单元行为模型由 yosys 从 liberty 现场生成，落 $(NETLIST_DIR)/cells_sim.v
#  （仓库不存单元模型副本）。WAVE=1 执行时落 VCD 于
#  $(NETLIST_DIR)/<模块>/tb_<模块>.vcd（门级波形，将来供顶层 power 使用）。
# ===========================================================================
NETLIST_DIR := $(TMP)/netlist

netlist:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make netlist <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能仿真一个模块，收到：$(MOD)"; exit 1; \
	else \
	  $(PDK_CHECK); \
	  $(YOSYS_FIND); \
	  if [ ! -f $(CURDIR)/$(SYN_DIR)/$(MOD)/$(MOD)_netlist.v ]; then \
	    echo "未检测到网表，先综合：make syn $(MOD)"; \
	    $(MAKE) --no-print-directory syn $(MOD) || exit 1; \
	  fi; \
	  mkdir -p $(NETLIST_DIR); \
	  if [ ! -f $(NETLIST_DIR)/cells_sim.v ]; then \
	    echo "== 生成单元行为模型（liberty → cells_sim.v）=="; \
	    $$YBIN -p "read_liberty -ignore_miss_func -ignore_miss_dir -ignore_miss_data_latch $(NANGATE_LIB); write_verilog $(CURDIR)/$(NETLIST_DIR)/cells_sim.v" || exit 1; \
	  fi; \
	  mkdir -p $(NETLIST_DIR)/$(MOD) && cd $(NETLIST_DIR)/$(MOD) && \
	  { . /opt/synopsys/snop18.sh >/dev/null 2>&1 || true; } && \
	  { command -v $(VCS) >/dev/null 2>&1 || { echo "未找到 vcs：请先配置 Synopsys 环境（/opt/synopsys/snop18.sh）"; exit 1; }; } && \
	  $(VCS) $(VCSFLAGS) -P "$$NOVAS/novas.tab" "$$NOVAS/pli.a" \
	    -top tb_$(MOD) -o simv -Mdir=csrc \
	    -CFLAGS "-std=gnu99 -I$(CURDIR)/$(SIM_DIR) -ffp-contract=off" \
	    $(CURDIR)/$(SYN_DIR)/$(MOD)/$(MOD)_netlist.v \
	    $(CURDIR)/$(NETLIST_DIR)/cells_sim.v \
	    $(CURDIR)/$(MOD)/tb/tb_$(MOD).sv \
	    $(addprefix $(CURDIR)/,$(DPI_SRC) $(CORE_SRCS)) || exit 1; \
	  if [ -n "$(RUN_IT)" ] || [ -n "$(GUI_IT)" ]; then \
	    if [ -n "$(GUI_IT)" ]; then DUMP="+fsdb=tb_$(MOD).fsdb"; \
	    else DUMP="$(if $(WAVE),+vcd=tb_$(MOD).vcd)"; fi; \
	    ./simv $$DUMP | tee simv.out; \
	    grep -q "SIM PASS" $(CURDIR)/$(NETLIST_DIR)/$(MOD)/simv.out || exit 1; \
	    if [ -n "$(GUI_IT)" ]; then \
	      command -v nWave >/dev/null 2>&1 || { echo "未找到 nWave：请先配置 Synopsys 环境（/opt/synopsys/snop18.sh）"; exit 1; }; \
	      echo "拉起 nWave：波形 tb_$(MOD).fsdb（FSDB 直出；日志：nwave.log）"; \
	      nohup nWave -f tb_$(MOD).fsdb >nwave.log 2>&1 & \
	    fi; \
	  fi; \
	fi

# ===========================================================================
#  【预留】面积 / 波形 / 功耗 / 性能（均只跑顶层，实现时产物统一落 tmp/ 下）
# ===========================================================================

# 面积：make area，只跑顶层：打印顶层综合报告的面积
area:
	@echo "[预留] area：顶层面积需顶层综合（make syn top，依赖顶层 RTL），尚未实现。"
	@echo "        实现后：打印顶层综合报告（单元数 / 芯片面积 / 时序占比），报告落 $(SYN_DIR)/top/。"
	@exit 1

# 波形：make wave，只跑顶层：跑门级仿真生成波形，供 power 功耗分析使用
wave:
	@echo "[预留] wave：门级仿真波形需顶层综合后门级网表（top）与门级 testbench，尚未实现。"
	@echo "        实现后：跑门级仿真出波形，产物落 $(TMP)/wave/，供 power 功耗分析使用。"
	@exit 1

# 功耗：make power，只跑顶层，需综合后门级网表 + wave 生成的门级仿真波形
power:
	@echo "[预留] power：功耗需顶层综合后门级网表（top）与门级仿真波形（wave），尚未实现。"
	@echo "        实现后：顶层网表 + 波形跑功耗，产物落 $(TMP)/power/。"
	@exit 1

# 性能：make perf，只跑顶层：从第一个块下发到所有块结束，即同一 kernel 跑完的 cycle 数
perf:
	@echo "[预留] perf：kernel 跑完的 cycle 数（第一个块下发到所有块结束），需顶层 rtl + tb，尚未实现。"
	@echo "        实现后：顶层仿真统计完成 cycle 数，报告落 $(TMP)/perf/。"
	@exit 1

# ===========================================================================
#  RTL 编译与仿真执行（VCS）
#    make rtl <模块>      仅编译；make rtl run <模块>  编译后执行
#      RTL 取 <模块>/rtl/*.sv，testbench 取 <模块>/tb/tb_<模块>.sv（顶层模块名
#      tb_<模块>）；C 参考模型 top/cmodel/dpi_ref.c（DPI 前端）与模型源文件
#      （不含 main.c）随 simv 一并编译。产物落 $(RTL_DIR)/<模块>/（.gitignore）。
#      执行时以运行日志中出现 "SIM PASS" 为通过判据。
#    WAVE=1 执行时落 VCD 于 $(RTL_DIR)/<模块>/tb_<模块>.vcd。
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
	  $(VCS) $(VCSFLAGS) -P "$$NOVAS/novas.tab" "$$NOVAS/pli.a" \
	    -top tb_$(MOD) -o simv -Mdir=csrc \
	    -CFLAGS "-std=gnu99 -I$(CURDIR)/$(SIM_DIR) -ffp-contract=off" \
	    $(CURDIR)/$(MOD)/rtl/*.sv $(CURDIR)/$(MOD)/tb/tb_$(MOD).sv \
	    $(addprefix $(CURDIR)/,$(DPI_SRC) $(CORE_SRCS)) || exit 1; \
	  if [ -n "$(RUN_IT)" ] || [ -n "$(GUI_IT)" ]; then \
	    if [ -n "$(GUI_IT)" ]; then DUMP="+fsdb=tb_$(MOD).fsdb"; \
	    else DUMP="$(if $(WAVE),+vcd=tb_$(MOD).vcd)"; fi; \
	    ./simv $$DUMP | tee simv.out; \
	    grep -q "SIM PASS" $(CURDIR)/$(RTL_DIR)/$(MOD)/simv.out || exit 1; \
	    if [ -n "$(GUI_IT)" ]; then \
	      command -v nWave >/dev/null 2>&1 || { echo "未找到 nWave：请先配置 Synopsys 环境（/opt/synopsys/snop18.sh）"; exit 1; }; \
	      echo "拉起 nWave：波形 tb_$(MOD).fsdb（FSDB 直出；日志：nwave.log）"; \
	      nohup nWave -f tb_$(MOD).fsdb >nwave.log 2>&1 & \
	    fi; \
	  fi; \
	fi

# cosim：保留旧目标名，等价于 make rtl run <模块>（编译并执行、比对参考模型）
cosim:
	@$(MAKE) --no-print-directory rtl run $(MOD)

rtl_clean:
	rm -rf $(RTL_DIR)

# 波形查看：VCD 缺失先生成，转 FSDB 后拉起 nWave（独立波形浏览器，
# 左侧即信号层次树；GUI 后台运行，日志落模块目录）
verdi:
	@if [ -z "$(MOD)" ]; then \
	  echo "用法：make verdi <模块>，模块 ∈ { $(MODULES) }（当前未指定模块）"; exit 1; \
	elif [ $(words $(MOD)) -gt 1 ]; then \
	  echo "一次只能指定一个模块，收到：$(MOD)"; exit 1; \
	else \
	  if [ ! -f $(RTL_DIR)/$(MOD)/tb_$(MOD).vcd ]; then \
	    echo "未检测到 $(RTL_DIR)/$(MOD)/tb_$(MOD).vcd，先生成：make rtl run $(MOD) WAVE=1"; \
	    $(MAKE) --no-print-directory rtl run $(MOD) WAVE=1 || exit 1; \
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
	@echo "  cmodel           编译模型库"
	@echo "  cmodel run       编译并执行模型"
	@echo "  kernel           自 $(KERNEL_SRC) 生成内核镜像到 $(KERNEL_DIR)/ (ptx/hex/json/lst)"
	@echo "  rtl <模块>       RTL 编译（VCS，testbench 为 <模块>/tb/tb_<模块>.sv）"
	@echo "  rtl run <模块>   RTL 仿真执行（VCS，testbench 为 <模块>/tb/tb_<模块>.sv）"
	@echo "  rtl gui <模块>   RTL 仿真执行并看波形（tb 直出 FSDB，拉起 nWave）"
	@echo "  syn <模块>       门级综合（Yosys + nangate45），产物落 tmp/syn/<模块>/"
	@echo "  netlist <模块>   门级仿真编译（综合后网表 + 单元行为模型 + 原 testbench）"
	@echo "  netlist run <模块> 门级仿真执行（对 C 参考模型事务级比对）"
	@echo "  netlist gui <模块> 门级仿真执行并看波形（tb 直出 FSDB，拉起 nWave）"
	@echo "  clean            清空 tmp/"
	@echo ""
	@echo "预留（未实现，均只跑顶层）："
	@echo "  area             顶层综合面积（单元数 / 芯片面积 / 时序占比）"
	@echo "  wave             门级仿真波形（供 power 使用）"
	@echo "  power            网表+波形跑功耗"
	@echo "  perf             kernel 跑完的 cycle 数（第一个块下发到所有块结束）"
