# easy_simt

Golden kernel 专用 SIMT 处理器（easy_simt）设计仓库。

## 前置条件

### GPGPU-Sim（外部基线对照）

本仓库的黄金基线数据（SM7_TITANV 口径）与原口径交叉验证来自 GPGPU-Sim
模拟器；主流程（模型编译、综合、回归）不依赖它。获取与构建（需 CUDA
Toolkit）：

```
git clone https://github.com/gpgpu-sim/gpgpu-sim_distribution.git
```

按其仓库 README 构建并进入 GPGPU-Sim 模式环境。注意：在 GPGPU-Sim 下
运行的 CUDA 程序须以 `nvcc -cudart shared` 编译（cudart 静态链接时
模拟器无法拦截运行时调用）。

### 工艺库（nangate45）

门级综合与时序对照使用 **nangate45** 工艺库：SI2 Nangate Open Cell Library
（版本 `PDKv1.3_v2010_12.Apache.CCL`）套 FreePDK45 设计规则，随
OpenROAD-flow-scripts 仓库发布，无需签协议。标准单元只有 typical 一个工艺角、
无 SDF、无器件模型卡，因此门级面积与时序只作同一口径下的相对比较，不作签核依据。

工艺库放在仓库外的 `$PDK_ROOT`（不入库）：

```
git clone --depth 1 https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts.git $PDK_ROOT/orfs
ln -s $PDK_ROOT/orfs/flow/platforms/nangate45 $PDK_ROOT/nangate45
```

只引用、不复制、不手改，保持单一来源。本节及后续 flow 脚本中的数字出自
`orfs` HEAD `7ff3adf`。

配套工具：综合前端取 oss-cad-suite（较新版本 Yosys，含 yosys-abc；门级映射
命令为 `dfflibmap` + `abc`）；物理验证侧 KLayout ≥ 0.28.8（OpenROAD 侧
`etc/DependencyInstaller.sh` 锁 `klayoutVersion=0.30.7`）。读库自检：

```
LIB=$PDK_ROOT/nangate45/lib/NangateOpenCellLibrary_typical.lib
yosys -p "read_liberty -lib $LIB; stat"      # Imported 135 cell types
```

路径与单元名单一律从 `$PDK_ROOT/nangate45/config.mk` 取，不在本仓库另抄一份：
`TECH_LEF`、`SC_LEF`、`LIB_FILES`、`PLACE_SITE`、`DONT_USE_CELLS`、`FILL_CELLS`、
`TIEHI_CELL_AND_PORT`、`TIELO_CELL_AND_PORT`。

- 存储阵列：库内无 SRAM，用 `lef/`、`lib/` 下的 `fakeram45_*` 宏（各 26 档），
  或按 BSG bsg_fakeram 的 JSON 配置生成新几何；配置字段只有
  `{name, width, depth, banks}`，多口需拆 bank 或多宏拼接。
- 物理验证：DRC 用 `drc/FreePDK45.lydrc`（KLayout 执行，无需厂商授权）；LVS 基准为
  `cdl/NangateOpenCellLibrary.cdl`（135 个 `.SUBCKT`、0 个 `.model`，
  故只能做拓扑比对，不能做晶体管级仿真）。

## 仓库组织

构建入口 `Makefile` 位于仓库根，与 `top/` 同级。每个硬件单元（`top` 及各子模块）
镜像同一结构 `docs/` + `rtl/` + `tb/`；子模块目录与 `top/` 同级平铺。所有编译、
综合、仿真的中间文件与报告统一落仓库根的 `tmp/`（不入库，`make clean` 清空）。

仓库结构：

```
easy_simt/
├── Makefile                 统一入口（仓库根，与 top/ 同级）
├── setup.sh                 导出 PROJ_ROOT；未设置时自动探测 PDK_ROOT
├── README.md  LICENSE  .gitignore
├── tmp/                     所有中间文件与报告（不入库；make clean 清空）
│   ├── build/sim/           C 模型库 + 回归可执行
│   ├── kernel/              easy_simt_kernel.{ptx,hex,json,lst}（make kernel 自 .cu 生成）
│   ├── rtl/<模块>/          RTL 仿真产物（simv / 日志 / 波形）
│   ├── syn/<模块>/          门级综合产物（网表 / stat.rpt / syn.log / syn.ys）
│   └── netlist/             门级仿真产物（cells_sim.v 与各模块 simv / 波形）
├── top/                     顶层互连单元 + 项目级工具目录
│   ├── docs/                isa_spec_v0.1.md / ma_spec_v0.1.md / intf_spec_v0.1.md
│   ├── scripts/             约束文件（common.sdc，所有模块共用）
│   ├── assembler/           easy_simt_assembler.py / easy_simt_assembler_verify.py
│   ├── cmodel/              事务级 C 参考模型（*.c + sim_common.h + DPI 前端 dpi_ref.c）
│   ├── kernel/              easy_simt_kernel.cu（内核单一源）
│   └── rtl/  tb/            顶层互连 RTL 与 testbench（预留）
└── <子模块>/  × 10          每个镜像 docs/ + rtl/ + tb/，与 top/ 同级
    ├── docs/                单元规范 <单元名>_spec.md（bs 已备，其余待补）
    ├── rtl/                 单元 RTL <单元名>.sv（bs 已备，其余待补）
    └── tb/                  单元 testbench tb_<单元名>.sv（bs 已备，其余待补）
```

子模块与 `top/` 同级，各含 `docs/` + `rtl/` + `tb/`。模块全集为 10 个功能模块 +
`top` 顶层互连（见 ma_spec §1.2、§1.4）：

| 目录 | 模块 |
|---|---|
| `sf` | SIMT Frontend |
| `ws` | Warp Scheduler |
| `bs` | Block Scheduler |
| `ialu` | 整数 ALU |
| `falu` | 浮点 ALU |
| `lsu` | Load/Store Unit |
| `icache` | Instruction Cache |
| `l1sm` | L1 + Shared Memory |
| `memif` | Memory Interface |
| `rf` | Register File |

单元内命名：`rtl/<单元名>.sv`、`tb/tb_<单元名>.sv`、`docs/<单元名>_spec.md`，
单元名与目录名一致。模块职责与接口见 `top/docs/ma_spec_v0.1.md` §1.4 与
`intf_spec_v0.1.md`。当前 `bs` 的 spec / RTL / testbench 三件套已备，其余模块待补。

`top/` 兼作项目级工具目录：

- `top/docs/` —— 三份顶层规范：`isa_spec_v0.1.md`（ISA 规范，设计唯一依据）、
  `ma_spec_v0.1.md`（顶层微架构规范）、`intf_spec_v0.1.md`（顶层接口规范，
  模块端口命名与 vld/rdy 协议）
- `top/scripts/` —— 约束文件 `common.sdc`（SDC/TCL，时钟与 IO 延迟约束，
  所有模块共用）
- `top/assembler/` —— PTX → easy_simt ISA 汇编器 `easy_simt_assembler.py` 与
  端到端验证程序 `easy_simt_assembler_verify.py`（内含功能级 ISS）
- `top/cmodel/` —— 事务级（transaction-accurate）C 参考模型：不建模时钟，
  模块间按 vld/rdy 握手传递事务；`dpi_ref.c` 为其经 DPI-C 接入 SystemVerilog
  testbench 的前端（随 simv 编译）
- `top/kernel/` —— 仅存 CUDA 源码 `easy_simt_kernel.cu`（内核单一源）；
  `.ptx/.hex/.json/.lst` 不入库，由 `make kernel` 现场生成到 `tmp/kernel/`
- `top/rtl/`、`top/tb/` —— 顶层互连单元的 RTL 与 testbench（预留）

## 使用

### 环境设置

```
source setup.sh
```

导出 `PROJ_ROOT`（仓库根目录），供脚本路径引用；`PDK_ROOT` 未设置时自动探测
`~/pdk`（要求其中含 `nangate45/`）并回显，也可在 source 前自行 `export` 覆盖。

### Makefile（仓库根执行，产物统一落 `tmp/`）

依赖：make、支持 C99 的 gcc；生成内核镜像另需 nvcc（CUDA）与 python3；
RTL 仿真为 VCS（recipe 内自动装载 Synopsys 环境）；综合为 Yosys（oss-cad-suite）。

```
make cmodel             # 编译模型库              -> tmp/build/sim/
make cmodel run         # 编译并执行模型黄金回归
make kernel             # 自 top/kernel/*.cu 生成内核镜像 -> tmp/kernel/
make rtl <模块>         # RTL 编译（VCS）
make rtl run <模块>     # RTL 仿真执行（WAVE=1 出 VCD；参考模型经 DPI-C 随 simv 编译）
make rtl gui <模块>     # RTL 仿真执行并看波形（tb 直出 FSDB，Verdi 连带设计打开）
make syn <模块>         # 门级综合（Yosys + nangate45，消费 top/scripts/common.sdc）
make netlist <模块>     # 门级仿真编译（网表 + 单元行为模型 + 原 testbench）
make netlist run <模块> # 门级仿真执行（对 C 参考模型事务级比对）
make netlist gui <模块> # 门级仿真执行并看波形（tb 直出 FSDB，Verdi 连带设计打开）
make clean              # 清空 tmp/
make help               # 列出全部目标
```

预留目标（均只跑顶层，待顶层 RTL 落地）：`area`（顶层综合面积）、`wave`
（门级仿真波形，供 `power` 使用）、`power`（网表+波形跑功耗）、`perf`
（kernel 跑完的 cycle 数）。依赖链：`area <- syn top`；`wave <- syn top` 的
网表 + 门级 tb；`power <- syn top` 的网表 + `wave` 的波形；`perf <- 内核镜像 +
顶层 rtl/tb`。

回归参数可覆盖，默认为 ma_spec §1.7 的 easy_simt 硬件口径：

```
make cmodel run KERNEL=tmp/kernel/easy_simt_kernel.hex N=1000 WARPS=4 LANES=8 MEMLAT=20
```

- `WARPS`/`LANES` 必须与模型编译参数（`NWARPS`/`NLANES`）一致，否则回归直接报
  配置不匹配；如需改动请重新编译并以 `-DNWARPS=… -DNLANES=…` 传入。
- `KERNEL` 为指令镜像（每行一条 32 位指令），BRT 由回归程序自同名 `.json` 自动装载。

回归验收项（对内置 CPU 参考自校验，对应 ma_spec §1.7）：

| # | 验收项 | 期望 |
|---|---|---|
| V1 | 功能位精确 | out[] 与 CPU 参考逐位一致（误差 0） |
| V2 | 共享内存访问次数 | 2048 |
| V3 | 数据分支分化 | 125/128 warp（pc=21） |
| V4 | 边界分支分化 | 0 |
| V5 | 无死锁 | `bs_top_done` 正常结束 |

### 内核与汇编器

`top/kernel/easy_simt_kernel.cu` 是内核的**单一源**。推荐的重新生成方式是
`make kernel`，它按全工具链把中间产物与镜像都写到 `tmp/kernel/`（不入库）：

```
nvcc -ptx -arch=sm_70 -fmad=false top/kernel/easy_simt_kernel.cu -o tmp/kernel/easy_simt_kernel.ptx
python3 top/assembler/easy_simt_assembler.py tmp/kernel/easy_simt_kernel.ptx -o tmp/kernel/
```

`.cu -> .ptx` 用 `-arch=sm_70`（对齐 ma_spec 硬件口径 `.target sm_70`）与
`-fmad=false`（乘/加各一条指令、不做 FMA 收缩，对应模型侧 `-ffp-contract=off`）；
汇编器一次写出 hex / json / lst。

汇编器端到端验证程序（内含功能级 ISS）按**原口径**运行（block=256、块内置换
掩码 128，与 GPGPU-Sim 基线语义一致），需配合原版 PTX 使用：

```
python3 top/assembler/easy_simt_assembler_verify.py --ptx <原版 PTX> --warps 32 --lanes 8
python3 top/assembler/easy_simt_assembler_verify.py --selftest        # 软浮点自检
```

注意：`tmp/kernel/` 生成的镜像为 easy_simt 硬件口径（32 线程/块、块内置换掩码 16），
其端到端功能验证由 `make cmodel run` 完成，不适用上述原口径验证程序。

### 黄金测试

```
make cmodel run
```

通过标准：上表 V1–V5 全部 PASS，末行输出 `OVERALL = PASS`。
