# easy_simt

Golden kernel 专用 SIMT 处理器（golden-core）设计仓库。

## 目录约定

- 每个硬件单元（top 及各子模块）镜像同一结构：`docs/` + `rtl/` + `tb/`
- 单元内命名：`rtl/<单元名>.v`、`tb/<单元名>_tb.v`、`docs/<单元名>_spec.md`，单元名与目录名一致
- `top/docs/`：项目级文档
  - `golden_core_isa_spec_v0.1.md` —— ISA 规范（设计唯一依据）
  - `golden-kernel-isa-uarch-draft-v0.1.md` —— 设计草案（背景与微架构记录）
- `top/assembler/`：PTX → golden ISA 汇编器（Python）
- `top/simulator/`：指令级模拟器（ISS）与黄金测试
- `top/program/`：CUDA 源码、PTX、机器码镜像（program.hex / program.json / program.lst）

## 黄金测试

```
python3 top/simulator/run_golden.py --ptx top/program/shmem_diverge.ptx
```

通过标准：与 CPU 参考逐位一致（误差 0），共享内存指令 2048。
