# easy_simt

Golden kernel 专用 SIMT 处理器（easy_simt）设计仓库。

## 环境设置

```
source setup.sh
```

导出 `PROJ_ROOT`（仓库根目录），供脚本路径引用。

## 目录约定

构建入口 `Makefile` 位于仓库根，与 `top/` 同级。每个硬件单元（`top` 及各子模块）
镜像同一结构：`docs/` + `rtl/` + `tb/`。子模块目录与 `top/` 同级平铺。

- 单元内命名：`rtl/<单元名>.v`、`tb/<单元名>_tb.v`、`docs/<单元名>_spec.md`，单元名与目录名一致
- 子模块（与 `top/` 同级，各含 `docs/` + `rtl/` + `tb/`）：
  `sf` `ws` `bs` `ialu` `falu` `lsu` `icache` `l1sm` `memif` `rf`
  —— 模块职责与接口见 `top/docs/ma_spec_v0.1.md` §1.4 与 `intf_spec_v0.1.md`
- `top/`：顶层单元，兼作项目级工具目录
  - `top/docs/`：项目级文档
    - `isa_spec_v0.1.md` —— ISA 规范（设计唯一依据）
    - `ma_spec_v0.1.md` —— 顶层微架构规范
    - `intf_spec_v0.1.md` —— 顶层接口规范（模块端口命名与 vld/rdy 协议）
  - `top/assembler/`：PTX → easy_simt ISA 汇编器与端到端验证程序（内含功能级 ISS）
  - `top/cmodel/`：事务级（transaction-accurate）C 参考模型。不建模时钟，模块间按
    vld/rdy 握手传递事务；结构上预留经 DPI-C 作为参考模型接入 SystemVerilog
    testbench（模型库 + 前端分离，见根 `Makefile` 头注）
  - `top/kernel/`：仅存 CUDA 源码 `easy_simt_kernel.cu`（内核单一源）；`.ptx/.hex/.json/.lst`
    不入库，由 `make kernel` 现场生成到 `tmp/kernel/`
  - `top/rtl/`、`top/tb/`：顶层互连单元的 RTL 与 testbench（预留）
- `tmp/`：所有编译、综合、仿真的中间文件与报告统一落此目录（与 `top/` 同级，不入库）；
  `make clean` 清空整个 `tmp/`

## 构建与回归（根 Makefile）

依赖：make、支持 C99 的 gcc；生成内核镜像还需 nvcc（CUDA）与 python3。以下命令均在
**仓库根目录**执行，产物统一落 `tmp/`。

```
make               # release 编译      -> tmp/build/sim/easy_simt_sim
make kernel        # 自 top/kernel/*.cu 生成内核镜像 -> tmp/kernel/{ptx,hex,json,lst}
make sim_run       # 黄金回归（先生成内核镜像，再回归；默认参数）
make sim_dbg       # ASan/UBSan 调试编译 -> tmp/build/sim_dbg/easy_simt_sim_dbg
make sim_run_dbg   # 调试版回归
make syn <模块>    # 门级综合（预留，见下）
make help          # 列出全部目标
make clean         # 清空 tmp/
```

回归变量可覆盖，默认为 ma_spec §1.7 的 easy_simt 硬件口径：

```
make sim_run KERNEL=tmp/kernel/easy_simt_kernel.hex N=1000 WARPS=4 LANES=8 MEMLAT=20
```

- `WARPS`/`LANES` 必须与模型编译参数（`NWARPS`/`NLANES`）一致，否则回归直接报配置不匹配；如需改动请重新编译并以 `-DNWARPS=… -DNLANES=…` 传入。
- 调试版要求编译器支持 `-fsanitize=address,undefined`（gcc ≥ 4.9）；默认编译器较老时可指定 `make sim_run_dbg CC=gcc-9`，或用 `SAN=` 覆盖 sanitizer 选项。
- `KERNEL` 为指令镜像（每行一条 32 位指令），BRT 由回归程序自同名 `.json` 自动装载。

回归验收项（对内置 CPU 参考自校验，对应 ma_spec §1.7）：

| # | 验收项 | 期望 |
|---|---|---|
| V1 | 功能位精确 | out[] 与 CPU 参考逐位一致（误差 0） |
| V2 | 共享内存访问次数 | 2048 |
| V3 | 数据分支分化 | 125/128 warp（pc=21） |
| V4 | 边界分支分化 | 0 |
| V5 | 无死锁 | `bs_top_done` 正常结束 |

综合入口为 `make syn <模块>`（如 `make syn bs`），`<模块>` ∈ `sf ws bs ialu falu lsu
icache l1sm memif rf`。综合前会校验模块名；当前综合流程尚未接入，调用会明确报未实现，
待各模块 `rtl/` 与工艺库就绪后启用。其余预留目标：`rtl`、`rtl_clean`、`wave`、`dpi`、
`cosim`。其中 `dpi`（模型库编译为 DPI-C 共享对象）与 `cosim`（RTL 与 C 参考模型对比回归）
是 C 模型作为参考模型接入 SV testbench 的入口。

## 内核与汇编器

`top/kernel/easy_simt_kernel.cu` 是内核的**单一源**。推荐的重新生成方式是 `make kernel`，
它按全工具链把中间产物与镜像都写到 `tmp/kernel/`（不入库）：

```
nvcc -ptx -arch=sm_70 -fmad=false top/kernel/easy_simt_kernel.cu -o tmp/kernel/easy_simt_kernel.ptx
python3 top/assembler/easy_simt_assembler.py tmp/kernel/easy_simt_kernel.ptx -o tmp/kernel/
```

`.cu -> .ptx` 用 `-arch=sm_70`（对齐 ma_spec 硬件口径 `.target sm_70`）与 `-fmad=false`
（乘/加各一条指令、不做 FMA 收缩，对应模型侧 `-ffp-contract=off`）；汇编器一次写出
hex / json / lst。

汇编器端到端验证程序（内含功能级 ISS）按**原口径**运行（block=256、块内置换掩码 128，与
GPGPU-Sim 基线语义一致），需配合原版 PTX 使用：

```
python3 top/assembler/easy_simt_assembler_verify.py --ptx <原版 PTX> --warps 32 --lanes 8
python3 top/assembler/easy_simt_assembler_verify.py --selftest        # 软浮点自检
```

注意：`tmp/kernel/` 生成的镜像为 easy_simt 硬件口径（32 线程/块、块内置换掩码 16），
其端到端功能验证由 `make sim_run` 完成，不适用上述原口径验证程序。

## 黄金测试

```
make sim_run
```

通过标准：上表 V1–V5 全部 PASS，末行输出 `OVERALL = PASS`。


## 工艺库（nangate45）

门级综合与时序对照使用 **nangate45** 工艺库：SI2 Nangate Open Cell Library
（版本 `PDKv1.3_v2010_12.Apache.CCL`）套 FreePDK45 设计规则，随
OpenROAD-flow-scripts 仓库发布，无需签协议。标准单元只有 typical 一个工艺角、
无 SDF、无器件模型卡，因此门级面积与时序只作同一口径下的相对比较，不作签核依据。

### 获取

工艺库放在仓库外的 `$PDK_ROOT`（不入库；`setup.sh` 只导出 `PROJ_ROOT`，
`PDK_ROOT` 需自行 `export`）：

```
git clone --depth 1 https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts.git $PDK_ROOT/orfs
ln -s $PDK_ROOT/orfs/flow/platforms/nangate45 $PDK_ROOT/nangate45
```

只引用、不复制、不手改，保持单一来源。本节及后续 flow 脚本中的数字出自
`orfs` HEAD `7ff3adf`。

### 使用

综合前端取 oss-cad-suite（Yosys ≥ 0.58，含 yosys-abc）；物理验证侧 KLayout
≥ 0.28.8（OpenROAD 侧 `etc/DependencyInstaller.sh` 锁 `klayoutVersion=0.30.7`）。
读库自检：

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

根 `Makefile` 的综合目标 `make syn <模块>` 当前为占位，工艺库待 RTL 阶段启用。
