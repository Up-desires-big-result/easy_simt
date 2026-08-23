#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
iss.py — golden-core ISA 指令级模拟器（功能模型）。

特性：
  * 完整实现草案 §2.6 的 BR/JOIN + BRT 分化处理语义；
  * 纯标准库 IEEE-754 FP32 RN 软浮点（mul/add/neg/gt），单舍入、位精确；
    输出 subnormal 按 FTZ 处理（草案 §7 风险 1 已注明）；
  * 全局访存可配阻塞延迟 MEM_LAT（功能不受影响，只影响调度交织与周期计数）；
  * 统计：发射的 warp 指令数、按 lane 展开的等效指令数（可与
    GPGPU-Sim 的 37424 直接对比）、共享内存指令数、逐分支分化计数。

用法：
  python3 iss.py --selftest                 # 软浮点自检（对比精确有理数舍入）
  供 run_golden.py 调用；也可独立装载 program.json 单步调试。
"""

import argparse
import json
import random
import struct
from fractions import Fraction

# ---------------- 编码常量（与 ptx2gold.py 保持一致） ----------------
OP_IMAD, OP_IADD, OP_SHL, OP_XOR, OP_ORI, OP_LUI = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
OP_SETP, OP_FMUL, OP_FADD, OP_FNEG = 0x07, 0x08, 0x09, 0x0A
OP_LDG, OP_STG, OP_LDS, OP_STS = 0x0B, 0x0C, 0x0D, 0x0E
OP_LDP, OP_CSRR, OP_BR, OP_JOIN, OP_BAR, OP_RET = 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14

OPNAME = {0x01: 'IMAD', 0x02: 'IADD', 0x03: 'SHL', 0x04: 'XOR', 0x05: 'ORI',
          0x06: 'LUI', 0x07: 'SETP', 0x08: 'FMUL', 0x09: 'FADD', 0x0A: 'FNEG',
          0x0B: 'LDG', 0x0C: 'STG', 0x0D: 'LDS', 0x0E: 'STS', 0x0F: 'LDP',
          0x10: 'CSRR', 0x11: 'BR', 0x12: 'JOIN', 0x13: 'BAR', 0x14: 'RET'}


def s32(u):
    return u - (1 << 32) if u >= (1 << 31) else u


def s_ext(v, bits):
    return v - (1 << bits) if v >= (1 << (bits - 1)) else v


# ---------------- IEEE-754 FP32 RN 软浮点（位操作，单舍入） ----------------
def f32_of_float(x):
    return struct.unpack('<I', struct.pack('<f', x))[0]


def float_of_f32(u):
    return struct.unpack('<f', struct.pack('<I', u & 0xFFFFFFFF))[0]


def _unpack(u):
    """返回 (sign, mant, q)，值 = (-1)^sign * mant * 2^q，mant 为整数。
    正规数：mant∈[2^23,2^24)；非正规数自动规格化；零：mant=0。"""
    u &= 0xFFFFFFFF
    sg = (u >> 31) & 1
    e = (u >> 23) & 0xFF
    m = u & 0x7FFFFF
    if e == 0xFF:
        return (sg, -1, 0) if m else (sg, -2, 0)   # -2=inf, -1=nan 标记
    if e == 0:
        if m == 0:
            return (sg, 0, 0)
        q = -149
        while not (m & (1 << 23)):
            m <<= 1
            q -= 1
        return (sg, m, q)
    return (sg, m | 0x800000, e - 127 - 23)


def _pack(sg, mant, q):
    """mant∈[2^23,2^24)，值=mant*2^q；处理溢出/下溢（FTZ）。"""
    e_field = q + 23 + 127
    if e_field >= 255:
        return (sg << 31) | 0x7F800000             # RN 溢出 → inf
    if e_field <= 0:
        return sg << 31                             # FTZ
    return (sg << 31) | (e_field << 23) | (mant & 0x7FFFFF)


def _rne(mant, q, sgn=None):
    """把任意精度整数 mant（值=mant*2^q）舍入到 24 位尾数。
    sgn 显式给出结果符号（乘法用，尾数乘积恒正）；缺省从 mant 符号推导。"""
    if sgn is None:
        sgn = 1 if mant < 0 else 0
    mant = abs(mant)
    if mant == 0:
        return 0
    lz = mant.bit_length() - 1
    if lz >= 23:
        shift = lz - 23
        if shift > 0:
            dropped = mant & ((1 << shift) - 1)
            mant >>= shift
            q += shift
            half = 1 << (shift - 1)
            if dropped > half or (dropped == half and (mant & 1)):
                mant += 1
                if mant == 1 << 24:
                    mant >>= 1
                    q += 1
    else:
        mant <<= (23 - lz)
        q -= (23 - lz)
    return _pack(sgn, mant, q)


def f32_mul(a, b):
    sa, ma, qa = _unpack(a)
    sb, mb, qb = _unpack(b)
    sg = sa ^ sb
    if ma == -1 or mb == -1:                        # nan
        return 0x7FC00000
    if ma == -2 or mb == -2:                        # inf
        if ma == 0 or mb == 0:
            return 0x7FC00000                       # inf * 0 = nan
        return sg << 31 | 0x7F800000
    if ma == 0 or mb == 0:
        return sg << 31                             # 带符号零（RN 下 +0 优先）
    return _rne(ma * mb, qa + qb, sg)


def f32_add(a, b):
    sa, ma, qa = _unpack(a)
    sb, mb, qb = _unpack(b)
    if ma == -1 or mb == -1:
        return 0x7FC00000
    if ma == -2 or mb == -2:
        if ma == -2 and mb == -2 and sa != sb:
            return 0x7FC00000                       # inf + (-inf) = nan
        sg = sa if ma == -2 else sb
        return sg << 31 | 0x7F800000
    if ma == 0 and mb == 0:
        return (sa & sb) << 31                      # ±0：RN 下仅 -0+-0=-0
    va = -ma if sa else ma
    vb = -mb if sb else mb
    if ma == 0:
        return _rne(vb, qb)
    if mb == 0:
        return _rne(va, qa)
    # 对齐到公共指数 Q = q - da - db：全部左移，和为精确整数，单舍入
    q = max(qa, qb)
    da, db = q - qa, q - qb
    mant = (va << db) + (vb << da)
    return _rne(mant, q - da - db)


def f32_neg(a):
    return (a ^ 0x80000000) & 0xFFFFFFFF


def f32_gt(a, b):
    """a > b（严格）。nan 比较为 False。"""
    sa, ma, qa = _unpack(a)
    sb, mb, qb = _unpack(b)
    if ma == -1 or mb == -1:
        return False
    if ma == -2 or mb == -2:                        # 至少一个 inf
        if ma == -2 and mb == -2:
            return sa == 0 and sb == 1              # +inf > -inf
        if ma == -2:
            return sa == 0                          # +inf > 任何非 +inf
        return sb == 1                              # 任何非 -inf > -inf
    za, zb = (ma == 0), (mb == 0)
    if za and zb:
        return False                                # ±0 == ±0
    if za:
        return sb == 1                              # 0 > 负数
    if zb:
        return sa == 0                              # 正数 > 0
    if sa != sb:
        return sa == 0
    a_gt_abs = (qa > qb) or (qa == qb and ma > mb)
    return a_gt_abs if sa == 0 else not a_gt_abs and (qa, ma) != (qb, mb)


# ---------------- 自检：与精确有理数舍入的独立实现对拍 ----------------
def _f32_of_fraction(fr):
    """独立实现：把精确有理数 RN 舍入到 float32（仅自检用）。"""
    if fr == 0:
        return 0
    sg = 1 if fr < 0 else 0
    fr = abs(fr)

    def pow2(e):
        return Fraction(1 << e) if e >= 0 else Fraction(1, 1 << (-e))

    e = fr.numerator.bit_length() - fr.denominator.bit_length()
    while pow2(e) > fr:
        e -= 1
    while pow2(e + 1) <= fr:
        e += 1
    scaled = fr / pow2(e) * (1 << 23)
    fl = scaled.numerator // scaled.denominator
    rem = scaled - fl
    if rem > Fraction(1, 2) or (rem == Fraction(1, 2) and fl & 1):
        fl += 1
    if fl == 1 << 24:
        fl >>= 1
        e += 1
    ef = e + 127
    if ef >= 255:
        return (sg << 31) | 0x7F800000
    if ef <= 0:
        return sg << 31
    return (sg << 31) | (ef << 23) | (fl & 0x7FFFFF)


def _frac_of_bits(u):
    """独立实现：位模式 → 精确有理数（不复用 _unpack，保证对拍独立性）。"""
    sg = (u >> 31) & 1
    e = (u >> 23) & 0xFF
    m = u & 0x7FFFFF
    if e == 0xFF:
        return None
    if e == 0:
        if m == 0:
            return Fraction(0)
        v = Fraction(m, 1 << 149)
    else:
        mant = (1 << 23) | m
        if e >= 127:
            v = Fraction(mant << (e - 127), 1 << 23)
        else:
            v = Fraction(mant, 1 << (23 + 127 - e))
    return -v if sg else v


def selftest(seed=12345, nrand=3000, verbose=True):
    rnd = random.Random(seed)

    def rand_bits():
        e = rnd.randint(-20, 20)
        m = rnd.getrandbits(23) | 0x800000
        s = rnd.getrandbits(1)
        ef = e + 127
        return (s << 31) | (ef << 23) | (m & 0x7FFFFF)

    bad = 0
    for trial in range(nrand):
        a, b = rand_bits(), rand_bits()
        fa, fb = _frac_of_bits(a), _frac_of_bits(b)
        got_mul = f32_mul(a, b)
        exp_mul = _f32_of_fraction(fa * fb)
        got_add = f32_add(a, b)
        exp_add = _f32_of_fraction(fa + fb)
        if got_mul != exp_mul:
            bad += 1
            if verbose and bad < 5:
                print('MUL MISMATCH a=%08X b=%08X got=%08X exp=%08X'
                      % (a, b, got_mul, exp_mul))
        if got_add != exp_add:
            bad += 1
            if verbose and bad < 5:
                print('ADD MISMATCH a=%08X b=%08X got=%08X exp=%08X'
                      % (a, b, got_add, exp_add))
    # 定向用例
    cases_add = [
        (0x3F800000, 0x3F800000, 0x40000000),      # 1+1=2
        (0x3F800000, 0x33800000, 0x3F800000),      # 1 + 2^-24 → 平局→偶 → 1
        (0x3F800000, 0x34000000, 0x3F800001),      # 1 + 2^-23 精确
        (0x3F800000, 0xBF800000, 0x00000000),      # 1 + (-1) = 0
        (0x00000000, 0x80000000, 0x00000000),      # +0 + -0 = +0
    ]
    cases_mul = [
        (0x3F800000, 0x3F800000, 0x3F800000),
        (0x40000000, 0x3F000000, 0x3F800000),      # 2 * 0.5 = 1
        (0x3F800001, 0x3F800001, 0x3F800002),      # (1+2^-23)^2 → RN
        (0x3F800000, 0x00000000, 0x00000000),
    ]
    for a, b, exp in cases_add:
        got = f32_add(a, b)
        if got != exp:
            bad += 1
            print('ADD CASE FAIL %08X+%08X got=%08X exp=%08X' % (a, b, got, exp))
    for a, b, exp in cases_mul:
        got = f32_mul(a, b)
        if got != exp:
            bad += 1
            print('MUL CASE FAIL %08X*%08X got=%08X exp=%08X' % (a, b, got, exp))
    if verbose:
        print('softfloat selftest: %s (%d random pairs + %d directed cases)'
              % ('PASS' if bad == 0 else 'FAIL(%d)' % bad, nrand,
                 len(cases_add) + len(cases_mul)))
    return bad == 0


# ---------------- ISS ----------------
class Warp:
    __slots__ = ('wid', 'pc', 'mask', 'preds', 'regs', 'stack', 'done', 'bar', 'stall')

    def __init__(self, lanes):
        self.pc = 0
        self.mask = (1 << lanes) - 1
        self.preds = [0, 0, 0, 0]
        self.regs = [[0] * lanes for _ in range(32)]
        self.stack = []
        self.done = False
        self.bar = False
        self.stall = 0


class ISS:
    def __init__(self, prog, lanes=8, warps=32, grid=4,
                 params=(0x100000, 0x200000, 1000), shbase=0, memlat=20):
        self.words = prog['words']
        self.brt = {int(k): v for k, v in prog['brt'].items()}
        self.lanes = lanes
        self.warps_n = warps
        self.grid = grid
        self.params = params
        self.shbase = shbase
        self.memlat = memlat
        self.gmem = {}
        self.stats = {'issued': 0, 'lane_insns': 0, 'shmem_ops': 0,
                      'cycles': 0, 'ldg': 0, 'stg': 0,
                      'diverge': {}, 'uniform_br': 0}

    # ---- 单条指令 ----
    def step(self, w, bid):
        L = self.lanes
        m = w.mask
        word = self.words[w.pc]
        op = (word >> 26) & 0x3F
        pc0 = w.pc
        w.pc += 1
        pop = bin(m).count('1')
        self.stats['issued'] += 1
        self.stats['lane_insns'] += pop

        if op == OP_IMAD:
            rd, ra, rb, rc = (word >> 21) & 31, (word >> 16) & 31, \
                (word >> 11) & 31, (word >> 6) & 31
            for l in range(L):
                if m >> l & 1:
                    v = s32(w.regs[ra][l]) * s32(w.regs[rb][l]) + s32(w.regs[rc][l])
                    w.regs[rd][l] = v & 0xFFFFFFFF
        elif op == OP_IADD:
            rd, ra = (word >> 21) & 31, (word >> 16) & 31
            if (word >> 15) & 1:
                imm = s_ext(word & 0x7FFF, 15)
                for l in range(L):
                    if m >> l & 1:
                        w.regs[rd][l] = (w.regs[ra][l] + imm) & 0xFFFFFFFF
            else:
                rb = (word >> 11) & 31
                for l in range(L):
                    if m >> l & 1:
                        w.regs[rd][l] = (w.regs[ra][l] + w.regs[rb][l]) & 0xFFFFFFFF
        elif op == OP_SHL:
            rd, ra = (word >> 21) & 31, (word >> 16) & 31
            if (word >> 15) & 1:
                sh = word & 0x1F
                for l in range(L):
                    if m >> l & 1:
                        w.regs[rd][l] = (w.regs[ra][l] << sh) & 0xFFFFFFFF
            else:
                rb = (word >> 11) & 31
                for l in range(L):
                    if m >> l & 1:
                        w.regs[rd][l] = (w.regs[ra][l] << (w.regs[rb][l] & 31)) & 0xFFFFFFFF
        elif op == OP_XOR:
            rd, ra, rb = (word >> 21) & 31, (word >> 16) & 31, (word >> 11) & 31
            for l in range(L):
                if m >> l & 1:
                    w.regs[rd][l] = w.regs[ra][l] ^ w.regs[rb][l]
        elif op == OP_ORI:
            rd, ra = (word >> 21) & 31, (word >> 16) & 31
            imm = word & 0xFFFF
            for l in range(L):
                if m >> l & 1:
                    w.regs[rd][l] = w.regs[ra][l] | imm
        elif op == OP_LUI:
            rd = (word >> 21) & 31
            val = ((word >> 1) & 0xFFFFF) << 12
            for l in range(L):
                if m >> l & 1:
                    w.regs[rd][l] = val
        elif op == OP_SETP:
            pd = (word >> 24) & 3
            ra, rb = (word >> 18) & 31, (word >> 13) & 31
            fmt, cond = (word >> 12) & 1, (word >> 9) & 7
            res = 0
            for l in range(L):
                if m >> l & 1:
                    if fmt == 0:
                        a, b = s32(w.regs[ra][l]), s32(w.regs[rb][l])
                        ok = [a < b, a <= b, a == b, a != b, a >= b, a > b][cond]
                    else:
                        a, b = w.regs[ra][l], w.regs[rb][l]
                        if cond == 5:
                            ok = f32_gt(a, b)
                        elif cond == 4:
                            ok = f32_gt(a, b) or a == b
                        elif cond == 0:
                            ok = f32_gt(b, a)
                        elif cond == 1:
                            ok = f32_gt(b, a) or a == b
                        elif cond == 2:
                            ok = a == b
                        else:
                            ok = a != b
                    if ok:
                        res |= 1 << l
            w.preds[pd] = (w.preds[pd] & ~m) | (res & m)
        elif op == OP_FMUL:
            rd, ra, rb = (word >> 21) & 31, (word >> 16) & 31, (word >> 11) & 31
            for l in range(L):
                if m >> l & 1:
                    w.regs[rd][l] = f32_mul(w.regs[ra][l], w.regs[rb][l])
        elif op == OP_FADD:
            rd, ra, rb = (word >> 21) & 31, (word >> 16) & 31, (word >> 11) & 31
            for l in range(L):
                if m >> l & 1:
                    w.regs[rd][l] = f32_add(w.regs[ra][l], w.regs[rb][l])
        elif op == OP_FNEG:
            rd, ra = (word >> 21) & 31, (word >> 16) & 31
            for l in range(L):
                if m >> l & 1:
                    w.regs[rd][l] = f32_neg(w.regs[ra][l])
        elif op in (OP_LDG, OP_STG, OP_LDS, OP_STS):
            rt, ra, rb = (word >> 21) & 31, (word >> 16) & 31, (word >> 11) & 31
            if op in (OP_LDS, OP_STS):
                base = self.shbase
            else:
                base = None
            for l in range(L):
                if not (m >> l & 1):
                    continue
                if base is None:
                    addr = (w.regs[ra][l] + w.regs[rb][l]) & 0xFFFFFFFF
                else:
                    addr = (base + w.regs[rb][l]) & 0xFFFFFFFF
                if op == OP_LDG:
                    assert addr in self.gmem, 'LDG miss @%08X pc=%d' % (addr, pc0)
                    w.regs[rt][l] = self.gmem[addr]
                elif op == OP_STG:
                    self.gmem[addr] = w.regs[rt][l]
                elif op == OP_LDS:
                    assert addr in self.smem, 'LDS miss @%08X pc=%d' % (addr, pc0)
                    w.regs[rt][l] = self.smem[addr]
                else:
                    self.smem[addr] = w.regs[rt][l]
            if op in (OP_LDS, OP_STS):
                self.stats['shmem_ops'] += 1
            elif op == OP_LDG:
                self.stats['ldg'] += 1
                w.stall = self.memlat          # 阻塞式访存
            else:
                self.stats['stg'] += 1
        elif op == OP_LDP:
            rd, k = (word >> 21) & 31, (word >> 16) & 31
            val = self.params[k]
            for l in range(L):
                if m >> l & 1:
                    w.regs[rd][l] = val & 0xFFFFFFFF
        elif op == OP_CSRR:
            rd, sr = (word >> 21) & 31, (word >> 16) & 31
            for l in range(L):
                if m >> l & 1:
                    if sr == 0:
                        v = w.wid * L + l
                    elif sr == 1:
                        v = self.warps_n * L
                    else:
                        v = bid
                    w.regs[rd][l] = v
        elif op == OP_BR:
            psel, u, neg = (word >> 24) & 3, (word >> 23) & 1, (word >> 22) & 1
            target = pc0 + s_ext(word & 0x3FFFFF, 22)
            if u:
                taken = m
            else:
                p = w.preds[psel] & m
                taken = (m & ~p) if neg else p
            nt = m & ~taken
            if taken == 0 or nt == 0:              # 均匀（含全跳过/全进入）
                self.stats['uniform_br'] += 1
                if taken:
                    w.pc = target
            else:                                  # 分化
                self.stats['diverge'][pc0] = self.stats['diverge'].get(pc0, 0) + 1
                r = self.brt.get(pc0)
                assert r is not None, 'missing BRT entry for divergent BR pc=%d' % pc0
                if target == r:                  # 单侧跳过型
                    w.stack.append((r, m))
                    w.mask = nt
                    # pc 已是 pc0+1（落入侧）
                else:                            # 双侧型：先走 taken
                    w.stack.append((r, m))
                    w.stack.append((pc0 + 1, nt))
                    w.mask = taken
                    w.pc = target
        elif op == OP_JOIN:
            target = pc0 + s_ext(word & 0x3FFFFF, 22)
            if w.stack and w.stack[-1][0] == target:
                w.mask = w.stack.pop()[1]
                w.pc = target
            elif w.stack:
                p2, m2 = w.stack.pop()
                w.pc = p2
                w.mask = m2
            else:
                w.pc = target
        elif op == OP_BAR:
            w.bar = True
        elif op == OP_RET:
            assert not w.stack, 'RET with non-empty divergence stack (warp %d)' % w.wid
            w.done = True
        else:
            raise AssertionError('illegal opcode %02X at pc=%d' % (op, pc0))

    # ---- 块执行 ----
    def run(self, verbose=True):
        for bid in range(self.grid):
            self.smem = {}
            warps = [Warp(self.lanes) for _ in range(self.warps_n)]
            for i, w in enumerate(warps):
                w.wid = i
            sweeps = 0
            while True:
                if all(w.done for w in warps):
                    break
                if all(w.done or w.bar for w in warps) and any(w.bar for w in warps):
                    for w in warps:
                        w.bar = False
                    continue
                progress = False
                for w in warps:
                    if w.done or w.bar:
                        continue
                    if w.stall > 0:
                        w.stall -= 1
                        progress = True
                        continue
                    self.step(w, bid)
                    progress = True
                sweeps += 1
                if not progress:
                    for w in warps[:8]:
                        print('  warp%d pc=%d done=%s bar=%s stall=%d mask=%x stack=%r'
                              % (w.wid, w.pc, w.done, w.bar, w.stall, w.mask,
                                 w.stack))
                    raise AssertionError('deadlock at block %d sweep %d' % (bid, sweeps))
            self.stats['cycles'] += sweeps
            if verbose:
                print('block %2d done: %d sweeps' % (bid, sweeps))
        return self.stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prog', nargs='?', help='program.json')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--lanes', type=int, default=8)
    ap.add_argument('--warps', type=int, default=32)
    ap.add_argument('--grid', type=int, default=4)
    ap.add_argument('--memlat', type=int, default=20)
    args = ap.parse_args()
    if args.selftest:
        ok = selftest()
        raise SystemExit(0 if ok else 1)
    with open(args.prog, 'r', encoding='utf-8') as f:
        prog = json.load(f)
    iss = ISS(prog, lanes=args.lanes, warps=args.warps, grid=args.grid,
              memlat=args.memlat)
    n = 1000
    for i in range(n):
        iss.gmem[0x100000 + 4 * i] = f32_of_float(float(((i % 7) - 3) * 100))
    st = iss.run()
    print(json.dumps({k: v for k, v in st.items() if k != 'diverge'}, indent=1))
    print('diverge per BR pc:', st['diverge'])


if __name__ == '__main__':
    main()
