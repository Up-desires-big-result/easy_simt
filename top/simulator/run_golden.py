#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_golden.py — 端到端黄金测试：
  PTX --ptx2gold--> IMEM/BRT 镜像 --ISS 执行--> 输出数组 --对比--> CPU 参考

用法（在虚拟机 ~/sim_run/ptx2gold 下）：
  python3 run_golden.py --ptx ../shmem_diverge.ptx

默认按原始规模运行（block=256 → 8 lane × 32 warp，grid=4，N=1000），
与 GPGPU-Sim 基线语义完全一致；对比标准为逐位一致（运算顺序与舍入一致）。
"""

import argparse
import json
import sys

import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assembler"))

import ptx2gold
from iss import ISS, f32_of_float, float_of_f32, f32_add, f32_mul, f32_neg, \
    f32_gt, selftest


def cpu_reference(in_bits, n, block=256, mask=0x80):
    """与 shmem_diverge.cu 的 cpu_reference 同构（MASK 参数化），
    使用与 ISS 相同的软浮点，保证位精确可比。"""
    c1, c2 = 0x3F800347, 0x38D1B717           # 1.0001f, 0.0001f
    out = [0] * n
    for base in range(0, n, block):
        tmp = []
        for t in range(block):
            gid = base + t
            tmp.append(in_bits[gid] if gid < n else 0)
        for t in range(block):
            gid = base + t
            if gid >= n:
                break
            x = tmp[t ^ mask]
            if f32_gt(x, 0):
                r = x
                for _ in range(8):
                    r = f32_add(f32_mul(r, c1), c2)
            else:
                r = f32_neg(x)
            out[gid] = r
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ptx', required=True)
    ap.add_argument('--n', type=int, default=1000)
    ap.add_argument('--lanes', type=int, default=8)
    ap.add_argument('--warps', type=int, default=32)
    ap.add_argument('--memlat', type=int, default=20)
    ap.add_argument('--outdir', default=None, help='同时保存镜像到该目录')
    args = ap.parse_args()

    block = args.lanes * args.warps
    assert block == 256, \
        '原始 PTX 语义要求 block=256（xor 0x80、tile[256]），当前=%d' % block
    n = args.n
    grid = (n + block - 1) // block

    print('== 0. softfloat 自检 ==')
    if not selftest(verbose=True):
        sys.exit('softfloat selftest FAILED')

    print('== 1. 汇编 PTX -> easy_simt ISA ==')
    prog, _ = ptx2gold.assemble_file(args.ptx)
    m = prog['meta']
    print('  指令数=%d  物理寄存器用到 r%d  BRT 表项=%d'
          % (m['n_instr'], m['max_phys_reg'], len(prog['brt'])))
    if args.outdir:
        import os
        os.makedirs(args.outdir, exist_ok=True)
        with open(os.path.join(args.outdir, 'program.json'), 'w') as f:
            json.dump(prog, f, indent=1)

    print('== 2. 构造输入（与 CUDA 工程一致）==')
    in_base, out_base = 0x00100000, 0x00200000
    params = (in_base, out_base, n)
    iss = ISS(prog, lanes=args.lanes, warps=args.warps, grid=grid,
              params=params, shbase=0, memlat=args.memlat)
    in_bits = [f32_of_float(float(((i % 7) - 3) * 100)) for i in range(n)]
    for i, b in enumerate(in_bits):
        iss.gmem[in_base + 4 * i] = b

    print('== 3. ISS 执行（N=%d, grid=%d, block=%d, MEM_LAT=%d）=='
          % (n, grid, block, args.memlat))
    st = iss.run(verbose=True)

    print('== 4. CPU 参考与比对 ==')
    ref = cpu_reference(in_bits, n, block=block, mask=block // 2)
    out_bits = [iss.gmem.get(out_base + 4 * i, None) for i in range(n)]
    missing = sum(1 for v in out_bits if v is None)
    bitexact = (missing == 0) and all(a == b for a, b in zip(out_bits, ref))
    max_err = 0.0
    for a, b in zip(out_bits, ref):
        if a is None:
            continue
        e = abs(float_of_f32(a) - float_of_f32(b))
        max_err = max(max_err, e)

    print('== 5. 结果汇总 ==')
    print('  RESULT        = %s' % ('PASS' if bitexact else 'FAIL'))
    print('  max_abs_error = %.3e%s'
          % (max_err, '（位精确）' if bitexact and max_err == 0.0 else ''))
    print('  out[0]=%.4f  out[128]=%.4f  out[999]=%.4f'
          % (float_of_f32(out_bits[0]), float_of_f32(out_bits[128]),
             float_of_f32(out_bits[n - 1])))
    print()
    print('  -- ISS 统计 --')
    print('  发射 warp 指令      : %d' % st['issued'])
    print('  lane 等效指令数     : %d   (GPGPU-Sim 基线 37424，口径=线程指令)'
          % st['lane_insns'])
    print('  共享内存 warp 指令  : %d（STS/LDS 处 mask 恒全量，lane 口径=%d）'
          % (st['shmem_ops'], st['shmem_ops'] * args.lanes))
    print('  LDG/STG warp 指令   : %d / %d' % (st['ldg'], st['stg']))
    print('  总调度周期(估算)    : %d   (基线 735，规模不同不可直接比)' % st['cycles'])
    print('  均匀分支次数        : %d' % st['uniform_br'])
    print('  分化事件（按分支 pc）:')
    for pc, c in sorted(st['diverge'].items()):
        print('    pc=%-3d : %d 个 warp 分化' % (pc, c))

    print()
    print('  -- 与 GPGPU-Sim SM7_TITANV 基线对照 --')
    print('  指标                基线        本次 ISS')
    print('  功能验证            PASS        %s' % ('PASS' if bitexact else 'FAIL'))
    print('  最大误差            0           %.3e' % max_err)
    shmem_lane = st['shmem_ops'] * args.lanes   # 该程序在 STS/LDS 处 mask 恒为全量
    print('  共享内存指令        2048        %d (warp %d × %d lane)'
          % (shmem_lane, st['shmem_ops'], args.lanes))
    print('  数据分支分化        32 warp     %d warp（pc=数据分支）'
          % max(st['diverge'].values(), default=0))
    if not bitexact:
        sys.exit('GOLDEN TEST FAILED')


if __name__ == '__main__':
    main()
