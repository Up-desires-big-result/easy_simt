# easy_simt

Golden kernel 专用 SIMT 处理器（easy_simt）设计仓库。

## 环境设置

```
source setup.sh
```

导出 `PROJ_ROOT`（仓库根目录），供脚本路径引用。

## 目录约定

- 每个硬件单元（top 及各子模块）镜像同一结构：`docs/` + `rtl/` + `tb/`
- 单元内命名：`rtl/<单元名>.v`、`tb/<单元名>_tb.v`、`docs/<单元名>_spec.md`，单元名与目录名一致
- `top/docs/`：项目级文档
  - `isa_spec_v0.1.md` —— ISA 规范（设计唯一依据）
  - `ma_spec_v0.1.md` —— 顶层微架构规范
  - `intf_spec_v0.1.md` —— 顶层接口规范（模块端口命名与 vld/rdy 协议）
- `top/assembler/`：PTX → easy_simt ISA 汇编器与端到端验证程序（内含功能级 ISS）
- `top/cmodel/`：事务级（transaction-accurate）C 参考模型。不建模时钟，模块间按
  vld/rdy 握手传递事务；结构上预留经 DPI-C 作为参考模型接入 SystemVerilog
  testbench（模型库 + 前端分离，见 `top/Makefile` 头注）
- `top/kernel/`：CUDA 源码、PTX、机器码镜像（easy_simt_kernel.hex / .json / .lst）
- `top/rtl/`、`top/tb/`：预留（tb/ 只放 SystemVerilog 代码）
- `top/Makefile`：统一构建与回归入口，产物落 `top/build/`

## 构建与回归（top/Makefile）

依赖：make、支持 C99 的 gcc。以下命令均在 `top/` 目录下执行。

```
make               # release 编译      -> build/sim/easy_simt_sim
make sim_run       # 黄金回归（默认参数）
make sim_dbg       # ASan/UBSan 调试编译 -> build/sim_dbg/easy_simt_sim_dbg
make sim_run_dbg   # 调试版回归
make help          # 列出全部目标
make clean         # 清理 build/
```

回归变量可覆盖，默认为 ma_spec §1.7 的 T3 硬件口径：

```
make sim_run KERNEL=kernel/easy_simt_kernel.hex N=1000 WARPS=4 LANES=8 MEMLAT=20
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

预留目标（占位，未实现）：`rtl`、`rtl_clean`、`wave`、`dpi`、`cosim`。其中
`dpi`（模型库编译为 DPI-C 共享对象）与 `cosim`（RTL 与 C 参考模型对比回归）
是 C 模型作为参考模型接入 SV testbench 的入口。

## 内核与汇编器

重新生成内核镜像（产物写入 `-o` 目录，覆盖 hex / json / lst）：

```
python3 top/assembler/easy_simt_assembler.py top/kernel/easy_simt_kernel.ptx -o top/kernel/
```

汇编器端到端验证程序按**原口径**运行（block=256、块内置换掩码 128，与
GPGPU-Sim 基线语义一致），需配合原版 PTX 使用：

```
python3 top/assembler/easy_simt_assembler_verify.py --ptx <原版 PTX> --warps 32 --lanes 8
python3 top/assembler/easy_simt_assembler_verify.py --selftest        # 软浮点自检
```

注意：`top/kernel/` 现为 T3 硬件口径镜像（32 线程/块、块内置换掩码 16），
其端到端功能验证由 `make sim_run` 完成，不适用上述原口径验证程序。

## 黄金测试

```
cd top && make sim_run
```

通过标准：上表 V1–V5 全部 PASS，末行输出 `OVERALL = PASS`。
