#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ptx2gold.py — 将 golden kernel shmem_diverge 的 PTX 翻译为 golden-core ISA
（硬件 ISA 与微架构草案 v0.1，含勘误：IADD/SHL 立即数模式位为 bit[15]，
立即数占 [14:0] 共 15 位）。

定位：单 kernel 机器的专用翻译器。只支持 shmem_diverge PTX 中实际出现的
指令模式；遇到未支持语法立即报错（assert），不做静默猜测。

主要降级策略：
  * 64 位地址拼接（cvta.to.global / mul.wide.s32 / add.s64）被消除，
    折叠为 LDG/STG 的 base+offset 寻址（偏移 = gid<<2，跨访存点复用）。
  * 共享内存基址（mov.u32 %r, tile 符号）被吸收：LDS/STS 隐含 SHBASE，
    只保留字节偏移寄存器。
  * 浮点字面量常数（1.0001f / 0.0001f）用 LUI+ORI 显式构造，缓存去重。
  * 分支按草案 §2.6 的 BR/JOIN + BRT 方案生成：
      - 条件分支的重聚点由控制流图的立即后支配点（ipdom）计算；
      - 落在路径末尾、跳往重聚点的 bra.uni 改写为 JOIN；
      - 直通到重聚点的路径末端插入 JOIN；
      - 保持 PTX 原始块布局（不做重排，故比手工优化版多 1 条 bra.uni）。

用法：
  python3 ptx2gold.py shmem_diverge.ptx -o out/
输出：
  out/program.json   IMEM 字序列 + BRT + 标号表（ISS / RTL 装载器输入）
  out/program.lst    带 PTX 源注释的汇编清单
  out/program.hex    十六进制镜像
"""

import argparse
import bisect
import json
import os
import re
import sys

# ---------------- ISA 编码（草案 v0.1 §2.4 + bit[15] 勘误） ----------------
OP_IMAD = 0x01
OP_IADD = 0x02
OP_SHL  = 0x03
OP_XOR  = 0x04
OP_ORI  = 0x05
OP_LUI  = 0x06
OP_SETP = 0x07
OP_FMUL = 0x08
OP_FADD = 0x09
OP_FNEG = 0x0A
OP_LDG  = 0x0B
OP_STG  = 0x0C
OP_LDS  = 0x0D
OP_STS  = 0x0E
OP_LDP  = 0x0F
OP_CSRR = 0x10
OP_BR   = 0x11
OP_JOIN = 0x12
OP_BAR  = 0x13
OP_RET  = 0x14

COND = {'lt': 0, 'le': 1, 'eq': 2, 'ne': 3, 'ge': 4, 'gt': 5}
SREG = {'%tid.x': 0, '%ntid.x': 1, '%ctaid.x': 2}


def enc_r3(op, rd, ra, rb, rc):
    return (op << 26) | (rd << 21) | (ra << 16) | (rb << 11) | (rc << 6)


def enc_r(op, rd, ra, rb=0):
    return (op << 26) | (rd << 21) | (ra << 16) | (rb << 11)


def enc_i15(op, rd, ra, imm):
    assert -16384 <= imm < 16384, 'imm15 overflow: %d' % imm
    return (op << 26) | (rd << 21) | (ra << 16) | (1 << 15) | (imm & 0x7FFF)


def enc_ori(rd, ra, imm16):
    assert 0 <= imm16 < 65536, 'imm16 overflow'
    return (OP_ORI << 26) | (rd << 21) | (ra << 16) | imm16


def enc_lui(rd, imm20):
    assert 0 <= imm20 < (1 << 20), 'imm20 overflow'
    return (OP_LUI << 26) | (rd << 21) | (imm20 << 1)


def enc_m(op, rt, ra, rb):
    return (op << 26) | (rt << 21) | (ra << 16) | (rb << 11)


def enc_lds(op, rt, rb):
    # LDS/STS: 地址 = SHBASE + R[rb]，ra 域置 0
    return (op << 26) | (rt << 21) | (rb << 11)


def enc_s(pd, ra, rb, fmt, cond):
    return (OP_SETP << 26) | ((pd & 3) << 24) | (ra << 18) | (rb << 13) \
        | ((fmt & 1) << 12) | ((cond & 7) << 9)


def enc_b(op, psel, u, neg, imm22):
    assert -2**21 <= imm22 < 2**21, 'branch offset overflow: %d' % imm22
    return (op << 26) | ((psel & 3) << 24) | ((u & 1) << 23) \
        | ((neg & 1) << 22) | (imm22 & 0x3FFFFF)


# ---------------- PTX 解析（限定子集） ----------------
RE_PRED = re.compile(r'^@(!?)%p(\d+)\s+')


def op_kind(o):
    """返回 (类型, 值)：mem/reg/f32/int/sym"""
    if o.startswith('[') and o.endswith(']'):
        return 'mem', o[1:-1]
    if o.startswith('%'):
        return 'reg', o
    if re.match(r'^0f[0-9A-Fa-f]{8}$', o):
        return 'f32', int(o[2:], 16)
    if re.match(r'^-?\d+$', o):
        return 'int', int(o)
    return 'sym', o


def parse_ptx(text):
    params = []
    shared_sym, shared_bytes = None, 0
    entry = None
    in_body = False
    lines = []
    for raw in text.splitlines():
        line = raw.split('//')[0].strip()
        if not line:
            continue
        if line.startswith(('.version', '.target', '.address_size')):
            continue
        if line.startswith('.visible .entry'):
            entry = re.search(r'\.entry\s+(\w+)\(', line).group(1)
            continue
        if line.startswith('.param'):
            m = re.match(r'\.param\s+\.(\w+)\s+(\w+)\s*,?', line)
            assert m, 'bad .param line: ' + line
            params.append(m.group(2))
            continue
        if line == '{':
            in_body = True
            continue
        if line == '}':
            break
        if not in_body:
            continue
        if line.startswith('.reg'):
            continue
        m = re.match(r'\.shared\s+.*?\.b8\s+(\w+)\[(\d+)\];', line)
        if m:
            shared_sym, shared_bytes = m.group(1), int(m.group(2))
            continue
        lines.append(line)

    prog = []
    for line in lines:
        s = line.rstrip(';').strip()
        m = re.match(r'^(\w+):\s*$', s)
        if m:
            prog.append({'label': m.group(1)})
            continue
        pred = None
        pm = RE_PRED.match(s)
        if pm:
            pred = (pm.group(1) == '!', int(pm.group(2)))
            s = s[pm.end():]
        parts = s.split(None, 1)
        mn = parts[0]
        ops = [o.strip() for o in parts[1].split(',')] if len(parts) > 1 else []
        mm = mn.split('.')
        prog.append({'pred': pred, 'base': mm[0], 'mods': mm[1:],
                     'ops': ops, 'src': line})
    return {'entry': entry, 'params': params, 'shared_sym': shared_sym,
            'shared_bytes': shared_bytes, 'prog': prog}


# ---------------- CFG 与后支配分析 ----------------
def build_cfg(prog):
    n = len(prog)
    leaders = {0: 'B0'}
    for i, it in enumerate(prog):
        if 'label' in it:
            leaders[i] = it['label']
        elif it['base'] in ('bra', 'ret') and i + 1 < n:
            leaders.setdefault(i + 1, 'A%d' % (i + 1))
    lidx = sorted(leaders)
    succ = {name: [] for name in leaders.values()}
    for k, pos in enumerate(lidx):
        name = leaders[pos]
        end = lidx[k + 1] if k + 1 < len(lidx) else n
        term = None
        for j in range(pos, end):
            it = prog[j]
            if 'label' in it:
                continue
            if it['base'] in ('bra', 'ret'):
                term = it
        nxt = leaders[lidx[k + 1]] if k + 1 < len(lidx) else None
        if term is None:
            if nxt:
                succ[name].append(nxt)
        elif term['base'] == 'ret':
            succ[name].append('EXIT')
        elif 'uni' in term['mods']:
            succ[name].append(term['ops'][0])
        else:
            succ[name].append(term['ops'][0])
            if nxt:
                succ[name].append(nxt)
    return succ, lidx, leaders


def postdom(succ):
    nodes = list(succ) + ['EXIT']
    ALL = frozenset(nodes)
    pd = {'EXIT': frozenset({'EXIT'})}
    for nd in succ:
        pd[nd] = ALL
    changed = True
    while changed:
        changed = False
        for nd in succ:
            ss = succ[nd]
            inter = ALL if not ss else frozenset.intersection(*[pd[s] for s in ss])
            nv = frozenset({nd} | inter)
            if nv != pd[nd]:
                pd[nd] = nv
                changed = True
    return pd


def closest_common(pd, a, b):
    """取 postdom(a) ∩ postdom(b) 中"最近"的公共后支配点（后支配集最大者）"""
    cands = pd[a] & pd[b]
    assert cands, 'no common post-dominator'
    best, best_sz = None, -1
    for c in cands:
        sz = len(pd[c])
        assert sz != best_sz or best is None or c == best, 'ambiguous ipdom'
        if sz > best_sz:
            best, best_sz = c, sz
    return best


# ---------------- 汇编器 ----------------
class Assembler:
    def __init__(self, pt):
        self.pt = pt
        self.prog = pt['prog']
        self.v2p = {}             # 虚寄存器 → 物理寄存器
        self.uses = self._count_uses()
        self.free = list(range(1, 32))
        self.pinned = set()
        self.sym32 = {}           # 32 位符号表达式（共享内存地址）
        self.sym64 = {}           # 64 位符号表达式（全局地址）
        self.byteoff = {}         # vreg(gid) → 已计算的 <<2 物理寄存器
        self.iconsts = {}         # 整数常数寄存器缓存
        self.fconsts = {}         # f32 常数寄存器缓存
        self.out = []             # {'word','asm','src','fixup'}
        self.label_pc = {}
        self.regions = []         # 打开的重聚区域（标签栈）
        self.brt = []             # (分支 pc 索引, 重聚标签)
        self.maxreg = 0
        # 预计算每个条件分支的重聚点
        self.succ, self.lidx, self.leaders = build_cfg(self.prog)
        pd = postdom(self.succ)
        self.ipdom = {}           # prog 索引 → 重聚块名
        blk = self._block_of()
        for k, pos in enumerate(self.lidx):
            end = self.lidx[k + 1] if k + 1 < len(self.lidx) else len(self.prog)
            nxt = self.leaders[self.lidx[k + 1]] if k + 1 < len(self.lidx) else None
            for j in range(pos, end):
                it = self.prog[j]
                if 'label' in it:
                    continue
                if it['base'] == 'bra' and 'uni' not in it['mods']:
                    tgt = it['ops'][0]
                    assert it['pred'] is not None
                    fb = blk[j + 1] if j + 1 < len(self.prog) else 'EXIT'
                    r = closest_common(pd, tgt, fb)
                    assert not r.startswith('A'), 'ipdom is anonymous block?!'
                    self.ipdom[j] = r

    def _block_of(self):
        blk = {}
        for k, pos in enumerate(self.lidx):
            name = self.leaders[pos]
            end = self.lidx[k + 1] if k + 1 < len(self.lidx) else len(self.prog)
            for j in range(pos, end):
                blk[j] = name
        return blk

    def _count_uses(self):
        uses = {}
        for it in self.prog:
            if 'label' in it:
                continue
            ops = it['ops']
            writes = it['base'] not in ('st', 'bar', 'bra', 'ret') and ops \
                and op_kind(ops[0])[0] == 'reg'
            start = 1 if writes else 0
            for o in ops[start:]:
                k, v = op_kind(o)
                if k == 'reg':
                    uses[v] = uses.get(v, 0) + 1
                elif k == 'mem':
                    k2, v2 = op_kind(v)
                    if k2 == 'reg':
                        uses[v2] = uses.get(v2, 0) + 1
        return uses

    # ---- 寄存器分配 ----
    def alloc(self):
        assert self.free, 'register file exhausted'
        p = self.free.pop(0)
        self.maxreg = max(self.maxreg, p)
        return p

    def free_reg(self, p):
        if p in self.pinned or p == 0:
            return
        bisect.insort(self.free, p)

    def define(self, v):
        if v in self.v2p:            # 重定义（如 %f23 / %f24）：复用物理寄存器
            return self.v2p[v]
        p = self.alloc()
        self.v2p[v] = p
        return p

    def use(self, v):
        assert v in self.v2p, 'use of undefined vreg ' + v
        return self.v2p[v]

    def consume(self, v):
        if v in self.uses:
            self.uses[v] -= 1
        if v in self.v2p and self.uses.get(v, 0) <= 0:
            self.free_reg(self.v2p[v])

    # ---- 发射 ----
    def emit(self, word, asm, src='', fixup=None):
        self.out.append({'word': word, 'asm': asm, 'src': src, 'fixup': fixup})
        return len(self.out) - 1

    def emit_label(self, lab):
        while self.regions and self.regions[-1] == lab:
            prev = self.out[-1]['asm'].split()[0] if self.out else ''
            if prev not in ('BR', 'RET'):      # 前一条直通才需要补 JOIN
                self.emit(enc_b(OP_JOIN, 0, 1, 0, 0), 'JOIN %s' % lab,
                          ';; 插入：区域汇合', (lab, 'join'))
            self.regions.pop()
        self.label_pc[lab] = len(self.out)

    # ---- 常数与派生值 ----
    def get_iconst(self, v, src):
        if v == 0:
            return 0
        if v in self.iconsts:
            return self.iconsts[v]
        t = self.alloc()
        self.pinned.add(t)
        if -16384 <= v < 16384:
            self.emit(enc_i15(OP_IADD, t, 0, v),
                      'IADD r%d, r0, #%d' % (t, v), src)
        else:
            self.emit(enc_lui(t, (v >> 12) & 0xFFFFF),
                      'LUI r%d, #0x%05X' % (t, (v >> 12) & 0xFFFFF), src)
            self.emit(enc_ori(t, t, v & 0xFFFF),
                      'ORI r%d, r%d, #0x%04X' % (t, t, v & 0xFFFF), src)
        self.iconsts[v] = t
        return t

    def get_fconst(self, bits, src):
        if bits == 0:
            return 0
        if bits in self.fconsts:
            return self.fconsts[bits]
        t = self.alloc()
        self.pinned.add(t)
        self.emit(enc_lui(t, (bits >> 12) & 0xFFFFF),
                  'LUI r%d, #0x%05X' % (t, (bits >> 12) & 0xFFFFF), src)
        self.emit(enc_ori(t, t, bits & 0xFFFF),
                  'ORI r%d, r%d, #0x%04X' % (t, t, bits & 0xFFFF), src)
        self.fconsts[bits] = t
        return t

    def get_byteoff(self, v, src):
        """gid<<2 偏移寄存器，跨访存点缓存复用"""
        if v in self.byteoff:
            self.consume(v)
            return self.byteoff[v]
        p = self.use(v)
        t = self.alloc()
        self.pinned.add(t)
        self.emit(enc_i15(OP_SHL, t, p, 2), 'SHL r%d, r%d, #2' % (t, p), src)
        self.byteoff[v] = t
        self.consume(v)
        return t

    # ---- 单条指令降级 ----
    def lower(self, it, idx):
        b, mods, ops, src = it['base'], it['mods'], it['ops'], it['src']

        if b == 'ld' and mods and mods[0] == 'param':
            d = ops[0]
            pname = ops[1].strip('[]')
            k = self.pt['params'].index(pname)
            p = self.define(d)
            self.emit((OP_LDP << 26) | (p << 21) | (k << 16),
                      'LDP r%d, #%d' % (p, k), src)
            if mods[1] == 'u64':
                self.sym64[d] = ('base', p)
                self.pinned.add(p)      # 被符号表达式长期引用，禁止复用
            return

        if b == 'cvta':
            s = ops[1]
            assert s in self.sym64, 'cvta of unknown pointer'
            self.sym64[ops[0]] = self.sym64[s]
            self.consume(s)
            return                                   # 无指令

        if b == 'mul' and 'wide' in mods:
            assert op_kind(ops[2]) == ('int', 4), 'mul.wide scale != 4'
            self.sym64[ops[0]] = ('off4', ops[1])    # 源 vreg 延迟到访存时 consume
            return                                   # 无指令

        if b == 'add' and mods == ['s64']:
            a, c = ops[1], ops[2]
            ea, ec = self.sym64[a], self.sym64[c]
            if ea[0] == 'off4':
                ea, ec = ec, ea
                a, c = c, a
            assert ea[0] == 'base' and ec[0] == 'off4'
            self.sym64[ops[0]] = ('addr', ea, ec)
            self.consume(a)
            self.consume(c)
            return                                   # 无指令

        if b == 'ld' and 'global' in mods:
            d = ops[0]
            addr = op_kind(ops[1])[1]
            assert addr in self.sym64 and self.sym64[addr][0] == 'addr'
            _, ebase, eoff = self.sym64[addr]
            t = self.get_byteoff(eoff[1], src)
            p = self.define(d)
            self.emit(enc_m(OP_LDG, p, ebase[1], t),
                      'LDG r%d, r%d, r%d' % (p, ebase[1], t), src)
            self.consume(addr)
            return

        if b == 'st' and 'global' in mods:
            addr = op_kind(ops[0])[1]
            assert addr in self.sym64 and self.sym64[addr][0] == 'addr'
            _, ebase, eoff = self.sym64[addr]
            t = self.get_byteoff(eoff[1], src)
            rt = self.use(ops[1])
            self.emit(enc_m(OP_STG, rt, ebase[1], t),
                      'STG r%d, r%d, r%d' % (rt, ebase[1], t), src)
            self.consume(addr)
            self.consume(ops[1])
            return

        if b == 'mov' and mods == ['u32'] and ops[1] in SREG:
            p = self.define(ops[0])
            self.emit((OP_CSRR << 26) | (p << 21) | (SREG[ops[1]] << 16),
                      'CSRR r%d, %s' % (p, ops[1]), src)
            return

        if b == 'mov' and mods == ['u32'] and op_kind(ops[1])[0] == 'sym':
            assert ops[1] == self.pt['shared_sym'], 'unknown symbol ' + ops[1]
            self.sym32[ops[0]] = ('shbase',)
            return                                   # SHBASE 隐含于 LDS/STS

        if b == 'mov' and mods == ['f32']:
            k, v = op_kind(ops[1])
            assert k == 'f32' and v == 0, 'only mov.f32 0.0 supported'
            p = self.define(ops[0])
            self.emit(enc_i15(OP_IADD, p, 0, 0), 'IADD r%d, r0, #0' % p, src)
            return

        if b == 'mad' and mods == ['lo', 's32']:
            pd_ = self.define(ops[0])
            ra, rb, rc = (self.use(ops[i]) for i in (1, 2, 3))
            self.emit(enc_r3(OP_IMAD, pd_, ra, rb, rc),
                      'IMAD r%d, r%d, r%d, r%d' % (pd_, ra, rb, rc), src)
            for v in ops[1:4]:
                self.consume(v)
            return

        if b == 'setp':
            fmt = 1 if mods[1] == 'f32' else 0
            pdst = int(ops[0][2:])
            assert 0 <= pdst < 4
            ra = self.use(ops[1])
            k, v = op_kind(ops[2])
            if k == 'reg':
                rb = self.use(ops[2])
                self.consume(ops[2])
            else:
                assert k == 'f32' and v == 0, 'setp literal must be 0.0'
                rb = 0
            self.emit(enc_s(pdst, ra, rb, fmt, COND[mods[0]]),
                      'SETP p%d, r%d, r%d, %s_%s' % (pdst, ra, rb,
                                                      'F' if fmt else 'I',
                                                      mods[0].upper()), src)
            self.consume(ops[1])
            return

        if b == 'add' and mods == ['s32']:
            a = ops[1]
            if a in self.sym32 and self.sym32[a] == ('shbase',):
                self.sym32[ops[0]] = ('shoff', ops[2])
                self.consume(a)
                return                               # 折叠进 STS/LDS
            pd_ = self.define(ops[0])
            ra, rb = self.use(ops[1]), self.use(ops[2])
            self.emit(enc_r(OP_IADD, pd_, ra, rb),
                      'IADD r%d, r%d, r%d' % (pd_, ra, rb), src)
            self.consume(ops[1])
            self.consume(ops[2])
            return

        if b == 'shl':
            pd_ = self.define(ops[0])
            ra = self.use(ops[1])
            k, v = op_kind(ops[2])
            assert k == 'int' and 0 <= v < 32
            self.emit(enc_i15(OP_SHL, pd_, ra, v),
                      'SHL r%d, r%d, #%d' % (pd_, ra, v), src)
            self.consume(ops[1])
            return

        if b == 'xor':
            pd_ = self.define(ops[0])
            ra = self.use(ops[1])
            k, v = op_kind(ops[2])
            assert k == 'int'
            c = self.get_iconst(v, src)
            self.emit(enc_r(OP_XOR, pd_, ra, c),
                      'XOR r%d, r%d, r%d' % (pd_, ra, c), src)
            self.consume(ops[1])
            return

        if b == 'st' and 'shared' in mods:
            ref = op_kind(ops[0])[1]
            assert ref in self.sym32 and self.sym32[ref][0] == 'shoff'
            offv = self.sym32[ref][1]
            offp = self.use(offv)
            rt = self.use(ops[1])
            self.emit(enc_lds(OP_STS, rt, offp),
                      'STS r%d, r%d' % (rt, offp), src)
            self.consume(offv)
            self.consume(ref)
            self.consume(ops[1])
            return

        if b == 'ld' and 'shared' in mods:
            ref = op_kind(ops[1])[1]
            assert ref in self.sym32 and self.sym32[ref][0] == 'shoff'
            offv = self.sym32[ref][1]
            offp = self.use(offv)
            pd_ = self.define(ops[0])
            self.emit(enc_lds(OP_LDS, pd_, offp),
                      'LDS r%d, r%d' % (pd_, offp), src)
            self.consume(offv)
            self.consume(ref)
            return

        if b == 'mul' and mods == ['rn', 'f32']:
            pd_ = self.define(ops[0])
            ra = self.use(ops[1])
            k, v = op_kind(ops[2])
            assert k == 'f32'
            c = self.get_fconst(v, src)
            self.emit(enc_r(OP_FMUL, pd_, ra, c),
                      'FMUL r%d, r%d, r%d' % (pd_, ra, c), src)
            self.consume(ops[1])
            return

        if b == 'add' and mods == ['rn', 'f32']:
            pd_ = self.define(ops[0])
            ra = self.use(ops[1])
            k, v = op_kind(ops[2])
            assert k == 'f32'
            c = self.get_fconst(v, src)
            self.emit(enc_r(OP_FADD, pd_, ra, c),
                      'FADD r%d, r%d, r%d' % (pd_, ra, c), src)
            self.consume(ops[1])
            return

        if b == 'neg' and mods == ['f32']:
            pd_ = self.define(ops[0])
            ra = self.use(ops[1])
            self.emit(enc_r(OP_FNEG, pd_, ra), 'FNEG r%d, r%d' % (pd_, ra), src)
            self.consume(ops[1])
            return

        if b == 'bar':
            assert ops == ['0']
            self.emit(OP_BAR << 26, 'BAR', src)
            return

        if b == 'bra':
            tgt = ops[0]
            if it['pred'] is not None:
                neg, pid = it['pred']
                r = self.ipdom[idx]
                i = self.emit(enc_b(OP_BR, pid, 0, 1 if neg else 0, 0),
                              'BR p%d, %s' % (pid, tgt), src, (tgt, 'br'))
                self.regions.append(r)
                self.brt.append((i, r))
            elif 'uni' in mods:
                if self.regions and self.regions[-1] == tgt:
                    self.emit(enc_b(OP_JOIN, 0, 1, 0, 0), 'JOIN %s' % tgt, src,
                              (tgt, 'join'))
                else:
                    self.emit(enc_b(OP_BR, 0, 1, 0, 0), 'BR always, %s' % tgt,
                              src, (tgt, 'br'))
            else:
                raise AssertionError('unsupported bra form: ' + src)
            return

        if b == 'ret':
            self.emit(OP_RET << 26, 'RET', src)
            return

        raise AssertionError('unsupported PTX instruction: ' + src)

    # ---- 主流程 ----
    def assemble(self):
        for idx, it in enumerate(self.prog):
            if 'label' in it:
                self.emit_label(it['label'])
                continue
            self.lower(it, idx)
        assert not self.regions, 'unclosed regions: %r' % self.regions
        # 回填分支偏移与 BRT
        for i, s in enumerate(self.out):
            if s['fixup']:
                lab, kind = s['fixup']
                assert lab in self.label_pc, 'unresolved label ' + lab
                off = self.label_pc[lab] - i
                w = s['word']
                op = (w >> 26) & 0x3F
                psel = (w >> 24) & 3
                u = (w >> 23) & 1
                neg = (w >> 22) & 1
                s['word'] = enc_b(op, psel, u, neg, off)
        brt = {}
        for i, lab in self.brt:
            brt[i] = self.label_pc[lab]
        words = [s['word'] for s in self.out]
        prog = {
            'words': words,
            'brt': {str(k): v for k, v in brt.items()},
            'labels': self.label_pc,
            'meta': {
                'n_instr': len(words),
                'max_phys_reg': self.maxreg,
                'entry': self.pt['entry'],
                'shared_bytes': self.pt['shared_bytes'],
                'n_params': len(self.pt['params']),
            },
        }
        return prog, self.out


def assemble_file(ptx_path):
    with open(ptx_path, 'r', encoding='utf-8') as f:
        pt = parse_ptx(f.read())
    a = Assembler(pt)
    return a.assemble()


def main():
    ap = argparse.ArgumentParser(description='PTX(shmem_diverge) -> golden-core ISA')
    ap.add_argument('ptx', help='PTX 源文件')
    ap.add_argument('-o', '--outdir', default='out', help='输出目录')
    args = ap.parse_args()

    prog, listing = assemble_file(args.ptx)
    os.makedirs(args.outdir, exist_ok=True)
    with open(os.path.join(args.outdir, 'program.json'), 'w', encoding='utf-8') as f:
        json.dump(prog, f, indent=1)
    with open(os.path.join(args.outdir, 'program.hex'), 'w', encoding='utf-8') as f:
        for w in prog['words']:
            f.write('%08X\n' % w)
    with open(os.path.join(args.outdir, 'program.lst'), 'w', encoding='utf-8') as f:
        inv = {v: k for k, v in prog['labels'].items()}
        brt = {int(k): v for k, v in prog['brt'].items()}
        for i, s in enumerate(listing):
            lab = inv.get(i)
            if lab:
                f.write('%s:\n' % lab)
            note = ''
            if i in brt:
                note = '   ;; BRT -> pc %d' % brt[i]
            f.write('%3d  %08X  %-28s %s%s\n'
                    % (i, s['word'], s['asm'], s['src'], note))
    m = prog['meta']
    print('assemble ok: %d instrs, regs r0..r%d, brt entries=%d'
          % (m['n_instr'], m['max_phys_reg'], len(prog['brt'])))
    for k, v in sorted(prog['labels'].items(), key=lambda kv: kv[1]):
        print('  label %-8s -> pc %d' % (k, v))
    for k, v in sorted(prog['brt'].items(), key=lambda kv: int(kv[0])):
        print('  BRT   pc %-4d -> pc %d' % (int(k), v))


if __name__ == '__main__':
    main()
