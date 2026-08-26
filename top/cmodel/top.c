/* ==========================================================================
 * top.c — 顶层驱动（对应 RTL 的 top：只做互连与全局停顿传递，自身无逻辑）
 *
 * 事务驱动：固定顺序轮询各模块 step()；一次 vld&&rdy 传递记一次发射；
 * bs_top_done 拉高即成功结束；整轮无任何发射且未结束判死锁（验收 V5）。
 * ========================================================================== */
#include "sim_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void sim_init(sim_t *s)
{
    memset(s, 0, sizeof(*s));
    s->n = 1000;
    s->nwarps = NWARPS;
    s->nlanes = NLANES;
    s->grid = (s->n + NWARPS * NLANES - 1) / (NWARPS * NLANES);
    s->params[0] = 0x00100000;     /* in_base  */
    s->params[1] = 0x00200000;     /* out_base */
    s->params[2] = (uint32_t)s->n;
    s->memif.in_base = 0x00100000;
    s->memif.out_base = 0x00200000;
    s->memif.memlat = 20;
    s->sf.fetch_warp = -1;
    s->sf.rd_warp = -1;
    s->ws.grant_warp = -1;
}

/* 程序镜像：每行一条 32 位指令（16 进制） */
int sim_load_hex(sim_t *s, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "cannot open kernel image: %s\n", path);
        return -1;
    }
    char line[64];
    int n = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '\n' || line[0] == '\r' || line[0] == '\0')
            continue;
        if (n >= IMEM_WORDS) {
            fprintf(stderr, "kernel image exceeds IMEM_WORDS=%d\n",
                    IMEM_WORDS);
            fclose(f);
            return -1;
        }
        s->memif.imem[n++] = (uint32_t)strtoul(line, 0, 16);
    }
    fclose(f);
    s->memif.imem_n = n;
    s->n_instr = n;
    return n;
}

/* BRT：从 kernel json 的 "brt": { "pc": rpc, ... } 装载（最小解析） */
int sim_load_brt_json(sim_t *s, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "cannot open brt json: %s\n", path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return -1; }
    if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) {
        free(buf); fclose(f); return -1;
    }
    buf[sz] = 0;
    fclose(f);

    char *p = strstr(buf, "\"brt\"");
    if (p) {
        p = strchr(p, '{');
        if (p) {
            p++;
            while (*p && *p != '}') {
                while (*p && *p != '"' && *p != '}')
                    p++;
                if (*p != '"')
                    break;
                long key = strtol(p + 1, &p, 10);
                p = strchr(p, ':');
                if (!p)
                    break;
                long val = strtol(p + 1, &p, 10);
                if (key >= 0 && key < IMEM_WORDS) {
                    s->brt_valid[key] = 1;
                    s->brt_rpc[key] = (uint32_t)val;
                }
            }
        }
    }
    free(buf);
    return 0;
}

/* 复位分发：块启动时随 launch 触发（谓词清零、SM 行钉扎） */
void sim_block_start(sim_t *s)
{
    memset(s->ialu.pred, 0, sizeof(s->ialu.pred));
    for (int i = 0; i < SM_LINES; i++) {
        s->l1sm.cls[i] = CLS_SM;
        s->l1sm.valid[i] = 1;
        s->l1sm.tag[i] = (uint32_t)i;      /* 自指哨兵 */
    }
    s->l1sm.shbase = 0;                    /* v1 单块在途恒 0 */
}

/* 全系统挂起检测：任一通道有未消费事务、任一模块有在途状态即为活 */
static int sim_pending(sim_t *s)
{
    if (s->bs_sf_launch.vld || s->bs_ws_launch.vld || s->ws_bs_bdone.vld ||
        s->ws_sf_grant.vld || s->sf_ws_stall.vld || s->sf_ws_bar.vld ||
        s->lsu_ws_stall.vld || s->sf_icache_req.vld ||
        s->icache_sf_rsp.vld || s->sf_rf_rd.vld || s->rf_sf_rddata.vld ||
        s->sf_ialu_issue.vld || s->sf_falu_issue.vld ||
        s->sf_lsu_issue.vld || s->ialu_sf_br.vld ||
        s->ialu_rf_wb.vld || s->falu_rf_wb.vld || s->lsu_rf_wb.vld ||
        s->ialu_sf_wbdone.vld || s->falu_sf_wbdone.vld ||
        s->lsu_sf_wbdone.vld || s->lsu_l1sm_req.vld ||
        s->l1sm_lsu_rsp.vld || s->icache_memif_req.vld ||
        s->memif_icache_rsp.vld || s->l1sm_memif_req.vld ||
        s->memif_l1sm_rsp.vld)
        return 1;
    if (s->memif.busy || s->icache.miss || s->icache.rsp_pending ||
        s->l1sm.busy || s->lsu.busy || s->ialu.has_issue ||
        s->falu.has_issue)
        return 1;
    if (s->sf.fetch_warp >= 0 || s->sf.fetch_pend || s->sf.rd_warp >= 0)
        return 1;
    if (s->ws.bar_count > 0 || s->ws.grant_warp >= 0 || s->ws.bdone_sent)
        return 1;
    if (s->lsu.stall_sent)
        return 1;
    for (int w = 0; w < NWARPS; w++) {
        int st = s->sf.w[w].state;
        if (st != WS_IDLE && st != WS_DONE && st != WS_BAR)
            return 1;
        if (s->sf.des_reason[w] != s->sf.cur_reason[w])
            return 1;
        if (s->sf.bar_pending[w])
            return 1;
    }
    return 0;
}

int sim_run(sim_t *s)
{
    const uint64_t LIMIT = 50000000ULL;
    s->bs.started = 1;
    while (!s->done && !s->err) {
        int fired = 0;
        fired += bs_step(s);
        fired += ws_step(s);
        fired += sf_step(s);
        fired += rf_step(s);
        fired += ialu_step(s);
        fired += falu_step(s);
        fired += lsu_step(s);
        fired += icache_step(s);
        fired += l1sm_step(s);
        fired += memif_step(s);
        s->rounds++;
        if (!fired && !sim_pending(s)) {
            fprintf(stderr,
                    "DEADLOCK: 无事务发射且系统无挂起 (round %llu)，"
                    "V5 违例\n",
                    (unsigned long long)s->rounds);
            s->err = 1;
            return 1;
        }
        if (s->rounds > LIMIT) {
            fprintf(stderr, "TIMEOUT: rounds > %llu（疑似活锁）\n",
                    (unsigned long long)LIMIT);
            s->err = 1;
            return 1;
        }
    }
    return s->err ? 1 : 0;
}
