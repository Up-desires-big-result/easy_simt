/* ==========================================================================
 * lsu.c — Load/Store Unit（ma_spec §7）
 *
 * 保持薄：不含 tag 比较、不含阵列/bank。per-lane 地址生成（基址+偏移，
 * 共享再叠 SHBASE）、active mask 门控、8-lane 锁步一拍发出一个 8-lane
 * 请求、请求分 shmem/global 两路、装载数据引导写回、向 ws 报停顿。
 *
 * sf 侧载荷约定：
 *   LDG: opa=逐 lane 偏移，imm=均匀基址（约束 C2）；
 *   STG: opa=逐 lane 数据，opb=逐 lane 偏移，imm=均匀基址；
 *   LDS: opa=逐 lane 偏移；  STS: opa=数据，opb=偏移。
 * 地址对齐违例为架构错误（isa_spec §1.10）。
 * ========================================================================== */
#include "sim_common.h"

int lsu_step(sim_t *s)
{
    lsu_t *u = &s->lsu;
    int fired = 0;
    int is_store, is_sm;

    switch (u->req_stage) {
    case 1:    /* 请求待发射 */
        if (!s->lsu_l1sm_req.vld) {
            s->lsu_l1sm_req.p = u->req;
            s->lsu_l1sm_req.vld = 1;
            u->req_stage = 2;
        }
        break;
    case 2:    /* 请求已置位，等 l1sm 接收 */
        if (!s->lsu_l1sm_req.vld) {
            u->req_stage = 3;
            s->st.fires++;
            fired++;
        }
        break;
    case 3:    /* 等 l1sm 响应 */
        if (s->l1sm_lsu_rsp.vld) {
            is_store = u->req.rw;
            if (!is_store) {
                for (int l = 0; l < NLANES; l++)
                    u->wb.wdata[l] = s->l1sm_lsu_rsp.p.rdata[l];
                u->req_stage = 4;
            } else {
                u->req_stage = 5;      /* 存储不写 rf，直接 wbdone */
            }
            s->l1sm_lsu_rsp.vld = 0;
            s->st.fires++;
            fired++;
        }
        break;
    case 4:    /* 装载写回待发射 */
        if (!u->req_sent) {
            if (!s->lsu_rf_wb.vld) {
                s->lsu_rf_wb.p = u->wb;
                s->lsu_rf_wb.vld = 1;
                u->req_sent = 1;
            }
        } else if (!s->lsu_rf_wb.vld) {
            u->req_sent = 0;
            u->req_stage = 5;
            s->st.fires++;
            fired++;
        }
        break;
    case 5:    /* wbdone 待发射 */
        if (!u->req_sent) {
            if (!s->lsu_sf_wbdone.vld) {
                s->lsu_sf_wbdone.p = u->wbd;
                s->lsu_sf_wbdone.vld = 1;
                u->req_sent = 1;
            }
        } else if (!s->lsu_sf_wbdone.vld) {
            u->req_sent = 0;
            u->req_stage = 0;
            u->busy = 0;
            s->st.fires++;
            fired++;
        }
        break;
    default:
        break;
    }

    /* 空闲且停顿上报未清：补发清除（重试直至成功） */
    if (!u->busy && u->stall_sent) {
        if (!s->lsu_ws_stall.vld) {
            s->lsu_ws_stall.p.warp_id = u->iss.warp_id;
            s->lsu_ws_stall.p.reason = R_NONE;
            s->lsu_ws_stall.vld = 1;
            u->stall_sent = 0;
        }
    }

    /* ---- 输入：接收发射 ---- */
    if (!u->busy && u->req_stage == 0 && s->sf_lsu_issue.vld) {
        lsu_issue_t is = s->sf_lsu_issue.p;
        s->sf_lsu_issue.vld = 0;
        s->st.fires++;
        fired++;

        u->iss = is;
        u->busy = 1;
        is_store = (is.opcode == OP_STG || is.opcode == OP_STS);
        is_sm = (is.opcode == OP_LDS || is.opcode == OP_STS);

        u->req.rw = is_store;
        u->req.sm = is_sm;
        u->req.mask = is.lane_mask;
        for (int l = 0; l < NLANES; l++) {
            u->req.addr[l] = 0;
            u->req.wdata[l] = 0;
            if (!((is.lane_mask >> l) & 1))
                continue;
            uint32_t addr = 0;
            switch (is.opcode) {
            case OP_LDG: addr = is.opa[l] + is.imm; break;
            case OP_STG: addr = is.opb[l] + is.imm; break;
            case OP_LDS: addr = is.shbase + is.opa[l]; break;
            case OP_STS: addr = is.shbase + is.opb[l]; break;
            default: s->err = 1; break;
            }
            if (addr & 3u)
                s->err = 1;            /* 未对齐：架构错误 */
            u->req.addr[l] = addr;
            if (is_store)
                u->req.wdata[l] = is.opa[l];
        }
        u->wb.warp_id = is.warp_id;
        u->wb.rd = is.rd;
        u->wb.lane_mask = is.lane_mask;
        u->wbd.warp_id = is.warp_id;
        u->wbd.rd = is.rd;
        u->req_stage = 1;

        /* 上报访存停顿（LMISS） */
        if (!u->stall_sent && !s->lsu_ws_stall.vld) {
            s->lsu_ws_stall.p.warp_id = is.warp_id;
            s->lsu_ws_stall.p.reason = R_LMISS;
            s->lsu_ws_stall.vld = 1;
            u->stall_sent = 1;
        }
    }
    return fired;
}
