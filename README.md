# easy_simt

Golden kernel 专用 SIMT 处理器（easy_simt）设计仓库。

## 目录约定

- 每个硬件单元（top 及各子模块）镜像同一结构：`docs/` + `rtl/` + `tb/`
- 单元内命名：`rtl/<单元名>.v`、`tb/<单元名>_tb.v`、`docs/<单元名>_spec.md`，单元名与目录名一致
- `top/docs/`：项目级文档
  - `isa_spec_v0.1.md` —— ISA 规范（设计唯一依据）
  - `ma_spec_v0.1.md` —— 顶层微架构规范
  - `intf_spec_v0.1.md` —— 顶层接口规范（模块端口命名与 vld/rdy 协议）
- `top/assembler/`：PTX → easy_simt ISA 汇编器与端到端验证程序（内含功能级 ISS）
- `top/simulator/`：微架构级模拟器（规划按 ma_spec 实现）
- `top/kernel/`：CUDA 源码、PTX、机器码镜像（easy_simt_kernel.hex / .json / .lst）

## 黄金测试

```
python3 top/assembler/easy_simt_assembler_verify.py --ptx top/kernel/easy_simt_kernel.ptx
```

通过标准：与 CPU 参考逐位一致（误差 0），共享内存指令 2048。
