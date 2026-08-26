/* ==========================================================================
 * icache.c — Instruction Cache（ma_spec §8）
 *
 * 直接映射，32B 行 = 8 条指令，默认 16 行（512B）。缺失阻塞：缺失期间
 * 取指请求挂起（sf 侧记 IMISS），经 memif 回填整行后返回指令。
 * 无预取、无无效化（程序只读）。
 * ========================================================================== */
#include "sim_common.h"

int icache_step(sim_t *s)
{
    icache_t *c = &s->icache;
    int fired = 0;

    /* ---- 命中/回填完成：回送指令 ---- */
    if (c->rsp_pending) {
        if (!c->rsp_sent) {
            if (!s->icache_sf_rsp.vld) {
                s->icache_sf_rsp.p.inst = c->rsp_inst;
                s->icache_sf_rsp.vld = 1;
                c->rsp_sent = 1;
            }
        } else if (!s->icache_sf_rsp.vld) {
            c->rsp_pending = 0;
            c->rsp_sent = 0;
            s->st.fires++;
            fired++;
        }
    }

    /* ---- 缺失在途：发回填请求，等回填数据 ---- */
    if (c->miss) {
        if (!c->req_sent) {
            if (!s->icache_memif_req.vld) {
                /* 行对齐字节地址 */
                s->icache_memif_req.p.addr = (c->miss_pc & ~7u) << 2;
                s->icache_memif_req.vld = 1;
                c->req_sent = 1;
            }
        } else if (s->memif_icache_rsp.vld) {
            uint32_t line = (c->miss_pc >> 3) % ICACHE_LINES;
            for (int i = 0; i < ILINE_WORDS; i++)
                c->data[line][i] = s->memif_icache_rsp.p.line[i];
            c->tag[line] = c->miss_pc >> 3;
            c->valid[line] = 1;
            c->rsp_inst = s->memif_icache_rsp.p.line[c->miss_pc & 7u];
            c->rsp_pending = 1;
            c->miss = 0;
            c->req_sent = 0;
            s->memif_icache_rsp.vld = 0;
            s->st.fires++;
            fired++;
        }
    }

    /* ---- 接收取指请求 ---- */
    if (!c->miss && !c->rsp_pending && s->sf_icache_req.vld) {
        uint32_t pc = s->sf_icache_req.p.pc;
        s->sf_icache_req.vld = 0;
        s->st.fires++;
        fired++;

        uint32_t t = pc >> 3;
        uint32_t line = t % ICACHE_LINES;
        if (c->valid[line] && c->tag[line] == t) {
            c->rsp_inst = c->data[line][pc & 7u];
            c->rsp_pending = 1;
        } else {
            s->st.icache_miss++;
            c->miss = 1;
            c->miss_pc = pc;
        }
    }
    return fired;
}
