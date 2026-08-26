/* ==========================================================================
 * main.c — 独立回归前端（golden regression）
 *
 * 装载内核镜像（.hex）与 BRT（同名 .json），按黄金口径构造输入，
 * 事务驱动运行至 bs_top_done，随后与内置 CPU 参考逐位比对（V1），
 * 并输出验收项（V2–V5）与统计对照。
 *
 * 黄金口径（ma_spec §1.7 T3 硬件版）：
 *   N=1000，grid=32 块 × 32 线程/块（NWARPS=4 × NLANES=8），MASK=16，
 *   in[i]=((i%7)-3)*100（f32），in_base=0x00100000，out_base=0x00200000。
 * ========================================================================== */
#include "sim_common.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t f32_of_float(float x)
{
    uint32_t u;
    memcpy(&u, &x, 4);
    return u;
}

static float float_of_f32(uint32_t u)
{
    float x;
    memcpy(&x, &u, 4);
    return x;
}

/* CPU 参考：与 verify 脚本 cpu_reference 同构（block/mask 参数化），
 * 使用同一软浮点，位精确可比。 */
static void cpu_reference(const uint32_t *in, uint32_t *out, int n,
                          int block, uint32_t xmask)
{
    const uint32_t c1 = 0x3F800347, c2 = 0x38D1B717;  /* 1.0001f, 0.0001f */
    uint32_t *tmp = (uint32_t *)malloc((size_t)block * 4);
    for (int base = 0; base < n; base += block) {
        for (int t = 0; t < block; t++) {
            int gid = base + t;
            tmp[t] = gid < n ? in[gid] : 0;
        }
        for (int t = 0; t < block; t++) {
            int gid = base + t;
            if (gid >= n)
                break;
            uint32_t x = tmp[t ^ xmask];
            uint32_t r;
            if (f32_gt(x, 0)) {
                r = x;
                for (int k = 0; k < 8; k++)
                    r = f32_add(f32_mul(r, c1), c2);
            } else {
                r = f32_neg(x);
            }
            out[gid] = r;
        }
    }
    free(tmp);
}

static const char *OPNAME[OP_NCOUNT] = {
    "?",    "IMAD", "IADD", "SHL",  "XOR",  "ORI",  "LUI",  "SETP",
    "FMUL", "FADD", "FNEG", "LDG",  "STG",  "LDS",  "STS",  "LDP",
    "CSRR", "BR",   "JOIN", "BAR",  "RET"
};

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr,
                "usage: %s KERNEL.hex [--n N] [--warps W] [--lanes L] "
                "[--memlat M]\n"
                "       BRT 自同名 .json 装载（可 --brt 指定）\n",
                argv[0]);
        return 2;
    }

    const char *hex = argv[1];
    const char *brt_path = 0;
    int n = 1000, warps = NWARPS, lanes = NLANES, memlat = 20;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--n") && i + 1 < argc) n = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--warps") && i + 1 < argc)
            warps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--lanes") && i + 1 < argc)
            lanes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--memlat") && i + 1 < argc)
            memlat = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--brt") && i + 1 < argc)
            brt_path = argv[++i];
    }
    if (warps != NWARPS || lanes != NLANES) {
        fprintf(stderr,
                "配置不匹配：本模型编译参数 NWARPS=%d NLANES=%d，"
                "命令行 warps=%d lanes=%d\n", NWARPS, NLANES, warps, lanes);
        return 2;
    }

    sim_t *s = (sim_t *)calloc(1, sizeof(sim_t));
    sim_init(s);
    s->n = n;
    s->params[2] = (uint32_t)n;
    s->grid = (n + NWARPS * NLANES - 1) / (NWARPS * NLANES);
    s->memif.memlat = memlat;

    if (sim_load_hex(s, hex) <= 0)
        return 2;
    /* 默认同名 .json */
    char auto_json[1024];
    if (!brt_path) {
        snprintf(auto_json, sizeof auto_json, "%s", hex);
        char *dot = strrchr(auto_json, '.');
        if (dot)
            strcpy(dot, ".json");
        else
            strcat(auto_json, ".json");
        brt_path = auto_json;
    }
    if (sim_load_brt_json(s, brt_path) < 0)
        return 2;

    /* 构造输入（与 CUDA 工程一致） */
    for (int i = 0; i < n && i < GMEM_WORDS; i++)
        s->memif.gmem_in[i] =
            f32_of_float((float)(((i % 7) - 3) * 100));

    printf("== easy_simt transaction-accurate C model ==\n");
    printf("config: N=%d grid=%d block=%d (%d warps x %d lanes) "
           "MEM_LAT=%d\n",
           n, s->grid, NWARPS * NLANES, NWARPS, NLANES, memlat);

    int rc = sim_run(s);

    /* ---- V1：与内置 CPU 参考位精确比对 ---- */
    int block = NWARPS * NLANES;
    uint32_t *in = s->memif.gmem_in;
    uint32_t *out = s->memif.gmem_out;
    uint32_t *ref = (uint32_t *)malloc((size_t)n * 4);
    uint32_t *in_ref = (uint32_t *)malloc((size_t)n * 4);
    for (int i = 0; i < n; i++)
        in_ref[i] = i < GMEM_WORDS ? in[i] : 0;
    cpu_reference(in_ref, ref, n, block, (uint32_t)(block / 2));
    int missing = 0, bitexact = 1;
    double max_err = 0.0;
    for (int i = 0; i < n; i++) {
        if (i >= GMEM_WORDS) { missing = 1; continue; }
        if (out[i] != ref[i]) {
            bitexact = 0;
            double e = fabsf(float_of_f32(out[i]) - float_of_f32(ref[i]));
            if (e > max_err) max_err = e;
            if (i < 8)
                printf("  MISMATCH out[%d]=%08X ref=%08X\n",
                       i, out[i], ref[i]);
        }
    }
    bitexact = bitexact && !missing && !rc;

    printf("RESULT = %s%s\n", bitexact ? "PASS" : "FAIL",
           bitexact ? "（位精确）" : "");
    printf("max_abs_error = %.3e\n", max_err);

    /* ---- 验收项 V2–V5 ---- */
    uint64_t shmem_lane = s->st.shmem_ops * NLANES;
    uint64_t div_total = 0, div_max = 0;
    int div_max_pc = -1;
    for (int pc = 0; pc < IMEM_WORDS; pc++) {
        if (s->st.diverge[pc]) {
            div_total += s->st.diverge[pc];
            if (s->st.diverge[pc] > div_max) {
                div_max = s->st.diverge[pc];
                div_max_pc = pc;
            }
        }
    }
    int total_warps = s->grid * NWARPS;
    printf("\n验收项：\n");
    printf("  V1 功能位精确      : %s\n", bitexact ? "PASS" : "FAIL");
    printf("  V2 共享内存访问    : %llu（期望 2048）%s\n",
           (unsigned long long)shmem_lane,
           shmem_lane == 2048 ? "PASS" : "FAIL");
    printf("  V3 数据分支分化    : %llu/%d warp（pc=%d，期望 %d）%s\n",
           (unsigned long long)div_max, total_warps, div_max_pc,
           total_warps - 3, div_max == (uint64_t)(total_warps - 3)
               ? "PASS" : "FAIL");
    printf("  V4 边界分支分化    : 0（边界 BR 走均匀分支，"
           "uniform_br=%llu）%s\n",
           (unsigned long long)s->st.uniform_br,
           s->st.diverge[9] == 0 ? "PASS" : "FAIL");
    printf("  V5 无死锁          : %s（rounds=%llu）\n",
           s->done && !s->err ? "PASS" : "FAIL",
           (unsigned long long)s->rounds);

    /* ---- 统计与对照 ---- */
    printf("\n-- C model 统计 --\n");
    printf("  发射 warp 指令      : %llu（ISS 硬件口径 6322）%s\n",
           (unsigned long long)s->st.issued,
           s->st.issued == 6322 ? "" : "  <== 不一致");
    printf("  lane 等效指令数     : %llu（期望 37280）%s\n",
           (unsigned long long)s->st.lane_insns,
           s->st.lane_insns == 37280 ? "" : "  <== 不一致");
    printf("  共享内存 warp 指令  : %llu（warp x %d lane = %llu）\n",
           (unsigned long long)s->st.shmem_ops, NLANES,
           (unsigned long long)shmem_lane);
    printf("  LDG/STG warp 指令   : %llu / %llu（期望 125/125）\n",
           (unsigned long long)s->st.ldg, (unsigned long long)s->st.stg);
    printf("  均匀分支次数        : %llu（期望 387）\n",
           (unsigned long long)s->st.uniform_br);
    printf("  分化事件（按分支 pc）:\n");
    for (int pc = 0; pc < IMEM_WORDS; pc++)
        if (s->st.diverge[pc])
            printf("    pc=%-3d : %llu 个 warp 分化\n", pc,
                   (unsigned long long)s->st.diverge[pc]);
    printf("  发射指令分布        :\n");
    for (int op = 1; op < OP_NCOUNT; op++)
        if (s->st.op_count[op])
            printf("    %-5s : %llu\n", OPNAME[op],
                   (unsigned long long)s->st.op_count[op]);
    printf("\n-- 存储子系统 --\n");
    printf("  icache 缺失         : %llu\n",
           (unsigned long long)s->st.icache_miss);
    printf("  L1 数据缺失         : %llu\n",
           (unsigned long long)s->st.l1_miss);
    printf("  l1sm 跨行/bank 冲突 : %llu（黄金程序期望 0）\n",
           (unsigned long long)s->st.l1sm_conflict);
    printf("  memif 读/写/取指回填: %llu / %llu / %llu\n",
           (unsigned long long)s->st.memif_rd,
           (unsigned long long)s->st.memif_wr,
           (unsigned long long)s->st.memif_icache_rd);
    printf("  通道发射事务总数    : %llu（轮询轮数 %llu）\n",
           (unsigned long long)s->st.fires,
           (unsigned long long)s->rounds);

    if (s->err)
        printf("\nERROR FLAG: sf_err=%d memif_err=%d\n",
               s->sf.err, s->memif.err);

    free(ref);
    free(in_ref);
    int v_pass = bitexact && shmem_lane == 2048
              && div_max == (uint64_t)(total_warps - 3)
              && s->st.diverge[9] == 0 && s->done && !s->err;
    printf("\nOVERALL = %s\n", v_pass ? "PASS" : "FAIL");
    free(s);
    return v_pass ? 0 : 1;
}
