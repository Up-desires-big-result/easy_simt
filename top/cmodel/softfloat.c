/* ==========================================================================
 * softfloat.c — IEEE-754 binary32 RN 软浮点（C 移植）
 *
 * 与 easy_simt_assembler_verify.py 的 Python 实现逐位一致：
 *   - 单舍入（RN，roundTiesToEven）；
 *   - 非规格化输入先规格化参与运算；
 *   - 非规格化/下溢结果按 FTZ 冲刷为带符号 ±0；
 *   - 上溢 → ±∞；NaN 输入 → 静默 NaN（0x7FC00000）。
 *
 * 加法路径中尾数对齐移位可达 ~253 位，超出 64 位范围，故用 320 位
 * 定量整数精确求和后再单次舍入（与 Python 大整数语义等价，
 * 舍入判定经 guard/round/sticky 三位归约，与精确比较等价）。
 * ========================================================================== */
#include "sim_common.h"

/* ---------------- 解包：值 = (-1)^sg * mant * 2^q ----------------
 * mant: >0 规格化尾数（含隐含位，∈[2^23,2^24)）；0 零；-1 NaN；-2 inf */
typedef struct { int sg; int64_t mant; int q; } fp_t;

static fp_t funpack(uint32_t u)
{
    fp_t r;
    int e = (u >> 23) & 0xFF;
    uint32_t m = u & 0x7FFFFF;
    r.sg = (u >> 31) & 1;
    if (e == 0xFF) {
        r.mant = m ? -1 : -2;
        r.q = 0;
        return r;
    }
    if (e == 0) {
        if (m == 0) { r.mant = 0; r.q = 0; return r; }
        r.q = -149;                       /* 非规格化：规格化处理 */
        while (!(m & (1u << 23))) { m <<= 1; r.q--; }
        r.mant = (int64_t)m;
        return r;
    }
    r.mant = (int64_t)(m | 0x800000);
    r.q = e - 127 - 23;
    return r;
}

/* ---------------- 打包：mant∈[2^23,2^24)，值 = mant*2^q ---------------- */
static uint32_t fpack(int sg, uint32_t mant, int q)
{
    int e = q + 23 + 127;
    if (e >= 255)
        return ((uint32_t)sg << 31) | 0x7F800000u;   /* RN 溢出 → inf */
    if (e <= 0)
        return (uint32_t)sg << 31;                   /* FTZ */
    return ((uint32_t)sg << 31) | ((uint32_t)e << 23) | (mant & 0x7FFFFF);
}

/* ---------------- 乘法（尾数积 < 2^48，64 位足够） ---------------- */
uint32_t f32_mul(uint32_t a, uint32_t b)
{
    fp_t A = funpack(a), B = funpack(b);
    int sg = A.sg ^ B.sg;
    if (A.mant == -1 || B.mant == -1)
        return 0x7FC00000u;
    if (A.mant == -2 || B.mant == -2) {
        if (A.mant == 0 || B.mant == 0)
            return 0x7FC00000u;                      /* inf * 0 = NaN */
        return ((uint32_t)sg << 31) | 0x7F800000u;
    }
    if (A.mant == 0 || B.mant == 0)
        return (uint32_t)sg << 31;                   /* 带符号零 */

    uint64_t mant = (uint64_t)A.mant * (uint64_t)B.mant;
    int q = A.q + B.q;
    int lz = 63 - __builtin_clzll(mant);
    if (lz >= 23) {
        int shift = lz - 23;
        if (shift > 0) {
            uint64_t dropped = mant & ((1ULL << shift) - 1);
            mant >>= shift;
            q += shift;
            uint64_t half = 1ULL << (shift - 1);
            if (dropped > half || (dropped == half && (mant & 1))) {
                mant++;
                if (mant == (1ULL << 24)) { mant >>= 1; q++; }
            }
        }
    } else {
        mant <<= (23 - lz);
        q -= (23 - lz);
    }
    return fpack(sg, (uint32_t)mant, q);
}

/* ---------------- 加法用 320 位定量整数 ---------------- */
#define BW 5
typedef struct { uint64_t w[BW]; } big_t;

static void big_zero(big_t *b)
{
    for (int i = 0; i < BW; i++) b->w[i] = 0;
}

static void big_from(big_t *b, uint64_t v)
{
    big_zero(b);
    b->w[0] = v;
}

static void big_shl(big_t *b, int k)
{
    big_t t;
    int ws = k >> 6, bs = k & 63;
    big_zero(&t);
    for (int i = 0; i < BW; i++) {
        if (!b->w[i]) continue;
        int d = i + ws;
        if (d < BW) t.w[d] |= b->w[i] << bs;
        if (bs && d + 1 < BW) t.w[d + 1] |= b->w[i] >> (64 - bs);
    }
    *b = t;
}

static void big_add(big_t *a, const big_t *b)
{
    uint64_t c = 0;
    for (int i = 0; i < BW; i++) {
        unsigned __int128 s = (unsigned __int128)a->w[i] + b->w[i] + c;
        a->w[i] = (uint64_t)s;
        c = (uint64_t)(s >> 64);
    }
}

static int big_cmp(const big_t *a, const big_t *b)
{
    for (int i = BW - 1; i >= 0; i--)
        if (a->w[i] != b->w[i])
            return a->w[i] < b->w[i] ? -1 : 1;
    return 0;
}

static void big_sub(big_t *a, const big_t *b)   /* 要求 a >= b */
{
    uint64_t borrow = 0;
    for (int i = 0; i < BW; i++) {
        unsigned __int128 d =
            (unsigned __int128)a->w[i] - b->w[i] - borrow;
        a->w[i] = (uint64_t)d;
        borrow = (d >> 64) ? 1 : 0;
    }
}

static int big_bitlen(const big_t *b)
{
    for (int i = BW - 1; i >= 0; i--)
        if (b->w[i])
            return i * 64 + (64 - __builtin_clzll(b->w[i]));
    return 0;
}

static int big_bit(const big_t *b, int i)
{
    return (int)((b->w[i >> 6] >> (i & 63)) & 1);
}

static int big_sticky_below(const big_t *b, int k)   /* bits [0,k) 或 */
{
    for (int i = 0; i < (k >> 6); i++)
        if (b->w[i]) return 1;
    int r = k & 63;
    if (r) {
        uint64_t m = (1ULL << r) - 1;
        if (b->w[k >> 6] & m) return 1;
    }
    return 0;
}

static uint32_t big_bits(const big_t *b, int off, int len)
{
    uint32_t v = 0;
    for (int i = 0; i < len; i++)
        if (big_bit(b, off + i))
            v |= 1u << i;
    return v;
}

/* 对 |M|*2^q 单舍入（M 已取绝对值，符号由 sg 给出）；M=0 → +0 */
static uint32_t rne_big(int sg, const big_t *M, int q)
{
    int bl = big_bitlen(M);
    if (bl == 0)
        return 0;
    int lz = bl - 1;
    uint32_t mant;
    int qq = q;
    if (lz >= 23) {
        int shift = lz - 23;
        mant = big_bits(M, shift, 24);
        qq += shift;
        if (shift > 0) {
            int guard = big_bit(M, shift - 1);
            int sticky = big_sticky_below(M, shift - 1);
            int lsb = (int)(mant & 1);
            if (guard && (sticky || lsb)) {
                mant++;
                if (mant == (1u << 24)) { mant >>= 1; qq++; }
            }
        }
    } else {
        mant = big_bits(M, 0, 24) << (23 - lz);
        qq -= (23 - lz);
    }
    return fpack(sg, mant, qq);
}

/* 单操作数舍入（_rne 的有符号 64 位版本，用于一操作数为零的退化） */
static uint32_t rne_i64(int sg, int64_t v, int q)
{
    if (v == 0) return 0;
    big_t M;
    big_from(&M, (uint64_t)(v < 0 ? -v : v));
    return rne_big(sg, &M, q);
}

uint32_t f32_add(uint32_t a, uint32_t b)
{
    fp_t A = funpack(a), B = funpack(b);
    if (A.mant == -1 || B.mant == -1)
        return 0x7FC00000u;
    if (A.mant == -2 || B.mant == -2) {
        if (A.mant == -2 && B.mant == -2 && A.sg != B.sg)
            return 0x7FC00000u;                      /* inf + (-inf) */
        int sg = (A.mant == -2) ? A.sg : B.sg;
        return ((uint32_t)sg << 31) | 0x7F800000u;
    }
    if (A.mant == 0 && B.mant == 0)
        return (uint32_t)((A.sg & B.sg) << 31);      /* RN：仅 -0+-0=-0 */
    if (A.mant == 0)
        return rne_i64(B.sg, B.mant, B.q);
    if (B.mant == 0)
        return rne_i64(A.sg, A.mant, A.q);

    int q = A.q > B.q ? A.q : B.q;
    int da = q - A.q, db = q - B.q;
    big_t X, Y, M;
    big_from(&X, (uint64_t)A.mant);
    big_shl(&X, db);
    big_from(&Y, (uint64_t)B.mant);
    big_shl(&Y, da);

    int sgn;
    if (A.sg == B.sg) {
        M = X;
        big_add(&M, &Y);
        sgn = A.sg;
    } else {
        int c = big_cmp(&X, &Y);
        if (c == 0)
            return 0;                                /* 精确抵消 → +0 */
        if (c > 0) { M = X; big_sub(&M, &Y); sgn = A.sg; }
        else       { M = Y; big_sub(&M, &X); sgn = B.sg; }
    }
    return rne_big(sgn, &M, q - da - db);
}

uint32_t f32_neg(uint32_t a)
{
    return (a ^ 0x80000000u) & 0xFFFFFFFFu;
}

/* a > b（严格），NaN 比较为假（镜像 Python f32_gt） */
int f32_gt(uint32_t a, uint32_t b)
{
    fp_t A = funpack(a), B = funpack(b);
    if (A.mant == -1 || B.mant == -1)
        return 0;
    if (A.mant == -2 || B.mant == -2) {
        if (A.mant == -2 && B.mant == -2)
            return A.sg == 0 && B.sg == 1;
        if (A.mant == -2)
            return A.sg == 0;
        return B.sg == 1;
    }
    int za = (A.mant == 0), zb = (B.mant == 0);
    if (za && zb) return 0;
    if (za) return B.sg == 1;
    if (zb) return A.sg == 0;
    if (A.sg != B.sg)
        return A.sg == 0;
    int a_gt_abs = (A.q > B.q) || (A.q == B.q && A.mant > B.mant);
    if (A.sg == 0)
        return a_gt_abs;
    return !a_gt_abs && !(A.q == B.q && A.mant == B.mant);
}
