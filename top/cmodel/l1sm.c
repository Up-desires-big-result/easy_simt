/* ==========================================================================
 * l1sm.c — L1 + Shared Memory（统一 SRAM，ma_spec §9）
 *
 * L1 与共享内存共用一块统一 SRAM：数据阵列按字偏移分 8 bank + 一份
 * tag 阵列 {class, valid, tag}。内部按 is_shmem 分流到两套 tag 规约：
 *   - SM 侧：钉扎自指哨兵（块启动写 tag:=自身索引，不缺失、不可替换）；
 *   - L1 侧：偏移索引（物理行整体位于 SM 区之上），正常 tag 直接映射，
 *     缺失阻塞、回填整行后覆写分配；写直通不写分配。
 * 8-bank 锁步：单行且无 bank 冲突单拍完成；跨行/冲突记为例外并计数
 * （功能上逐 lane 串行完成，仍是同一笔请求/响应事务）。
 * ========================================================================== */
#include "sim_common.h"

enum { L_IDLE = 0, L_SVC, L_RFILL_SET, L_RFILL_WAIT, L_WR_SET, L_WR_WAIT,
       L_RSP };

int l1sm_step(sim_t *s)
{
    l1sm_t *m = &s->l1sm;
    int fired = 0;
    int nsets = U_LINES - SM_LINES;

    /* ---- 响应排空 ---- */
    if (m->busy && m->stage == L_RSP) {
        if (!m->rsp_sent) {
            if (!s->l1sm_lsu_rsp.vld) {
                s->l1sm_lsu_rsp.p = m->rsp;
                s->l1sm_lsu_rsp.vld = 1;
                m->rsp_sent = 1;
            }
        } else if (!s->l1sm_lsu_rsp.vld) {
            m->rsp_sent = 0;
            m->busy = 0;
            m->stage = L_IDLE;
            s->st.fires++;
            fired++;
        }
        return fired;
    }

    if (m->busy) {
        switch (m->stage) {
        case L_SVC: {
            /* 找下一个活动 lane */
            int l = m->glane;
            while (l < NLANES && !((m->req.mask >> l) & 1))
                l++;
            if (l >= NLANES) {          /* 全部完成 */
                m->stage = L_RSP;
                break;
            }
            uint32_t addr = m->req.addr[l];
            if (m->req.sm) {
                uint32_t row = (addr - m->shbase) >> 5;
                uint32_t word = (addr >> 2) & 7u;
                if (row >= SM_LINES || m->cls[row] != CLS_SM
                    || !m->valid[row] || m->tag[row] != row) {
                    s->err = 1;         /* 钉扎完整性校验失败 */
                    break;
                }
                if (m->req.rw)
                    m->data[row][word] = m->req.wdata[l];
                else
                    m->rsp.rdata[l] = m->data[row][word];
                m->glane = l + 1;       /* SM 恒单拍，逐 lane 连做 */
            } else {
                uint32_t line = addr >> 5;
                uint32_t phys = SM_LINES + line % (uint32_t)nsets;
                uint32_t word = (addr >> 2) & 7u;
                int hit = m->valid[phys] && m->cls[phys] == CLS_L1
                       && m->tag[phys] == line;
                if (!m->req.rw) {
                    if (hit) {
                        m->rsp.rdata[l] = m->data[phys][word];
                        m->glane = l + 1;
                    } else {
                        s->st.l1_miss++;
                        m->stage = L_RFILL_SET;   /* 阻塞回填 */
                    }
                } else {
                    if (hit)                       /* 写直通：命中更新行内字 */
                        m->data[phys][word] = m->req.wdata[l];
                    m->stage = L_WR_SET;           /* 写直通等 BRESP 应答 */
                }
            }
            break;
        }
        case L_RFILL_SET:
            if (!m->req_sent && !s->l1sm_memif_req.vld) {
                uint32_t addr = m->req.addr[m->glane];
                s->l1sm_memif_req.p.rw = 0;
                s->l1sm_memif_req.p.addr = (addr >> 5) << 5;  /* 行对齐 */
                s->l1sm_memif_req.p.wdata = 0;
                s->l1sm_memif_req.vld = 1;
                m->req_sent = 1;
                m->stage = L_RFILL_WAIT;
            }
            break;
        case L_RFILL_WAIT:
            if (s->memif_l1sm_rsp.vld) {
                uint32_t addr = m->req.addr[m->glane];
                uint32_t line = addr >> 5;
                uint32_t phys = SM_LINES + line % (uint32_t)nsets;
                for (int i = 0; i < ILINE_WORDS; i++)
                    m->data[phys][i] = s->memif_l1sm_rsp.p.line[i];
                m->cls[phys] = CLS_L1;
                m->valid[phys] = 1;
                m->tag[phys] = line;
                s->memif_l1sm_rsp.vld = 0;
                m->req_sent = 0;
                m->stage = L_SVC;       /* 重放该 lane */
                s->st.fires++;
                fired++;
            }
            break;
        case L_WR_SET:
            if (!m->req_sent && !s->l1sm_memif_req.vld) {
                uint32_t addr = m->req.addr[m->glane];
                s->l1sm_memif_req.p.rw = 1;
                s->l1sm_memif_req.p.addr = addr;  /* 字地址，4B 窄传 */
                s->l1sm_memif_req.p.wdata = m->req.wdata[m->glane];
                s->l1sm_memif_req.vld = 1;
                m->req_sent = 1;
                m->stage = L_WR_WAIT;
            }
            break;
        case L_WR_WAIT:
            if (s->memif_l1sm_rsp.vld) {   /* AXI 写完成应答 */
                s->memif_l1sm_rsp.vld = 0;
                m->req_sent = 0;
                m->glane++;
                m->stage = L_SVC;
                s->st.fires++;
                fired++;
            }
            break;
        default:
            break;
        }
    }

    /* ---- 接收 lsu 请求 ---- */
    if (!m->busy && s->lsu_l1sm_req.vld) {
        m->req = s->lsu_l1sm_req.p;
        s->lsu_l1sm_req.vld = 0;
        s->st.fires++;
        fired++;
        m->busy = 1;
        m->glane = 0;
        m->stage = L_SVC;
        for (int l = 0; l < NLANES; l++)
            m->rsp.rdata[l] = 0;

        /* 单行无冲突判定（例外计数，不影响功能） */
        int conf = 0;
        uint32_t row0 = 0xFFFFFFFFu;
        int word_seen[NBANKS] = {0};
        for (int l = 0; l < NLANES; l++) {
            if (!((m->req.mask >> l) & 1))
                continue;
            uint32_t a = m->req.addr[l];
            uint32_t r = m->req.sm ? (a - m->shbase) >> 5 : a >> 5;
            uint32_t w = (a >> 2) & 7u;
            if (row0 == 0xFFFFFFFFu) row0 = r;
            if (r != row0) conf = 1;
            if (word_seen[w]) conf = 1;
            word_seen[w] = 1;
        }
        if (conf)
            s->st.l1sm_conflict++;
    }
    return fired;
}
