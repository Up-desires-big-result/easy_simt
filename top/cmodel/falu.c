/* ==========================================================================
 * falu.c — Floating-point ALU（ma_spec §6）
 *
 * FMUL/FADD/FNEG，IEEE-754 binary32、RN 舍入（软浮点实现，见 softfloat.c，
 * 与 ISS 位精确一致，含 FTZ）。数据通路 8 lane 并行、锁步。
 * ========================================================================== */
#include "sim_common.h"

int falu_step(sim_t *s)
{
    falu_t *u = &s->falu;
    int fired = 0;

    /* ---- 输出排空：wb → wbdone ---- */
    if (u->wb_stage == 1) {
        if (!u->wb_sent) {
            if (!s->falu_rf_wb.vld) {
                s->falu_rf_wb.p = u->wb;
                s->falu_rf_wb.vld = 1;
                u->wb_sent = 1;
            }
        } else if (!s->falu_rf_wb.vld) {
            u->wb_stage = 2;
            u->wb_sent = 0;
            s->st.fires++;
            fired++;
        }
    }
    if (u->wb_stage == 2) {
        if (!u->wbd_sent) {
            if (!s->falu_sf_wbdone.vld) {
                s->falu_sf_wbdone.p = u->wbd;
                s->falu_sf_wbdone.vld = 1;
                u->wbd_sent = 1;
            }
        } else if (!s->falu_sf_wbdone.vld) {
            u->wb_stage = 0;
            u->wbd_sent = 0;
            u->has_issue = 0;
            s->st.fires++;
            fired++;
        }
    }

    /* ---- 输入：接收发射 ---- */
    if (u->has_issue)
        return fired;
    if (s->sf_falu_issue.vld) {
        falu_issue_t is = s->sf_falu_issue.p;
        s->sf_falu_issue.vld = 0;
        s->st.fires++;
        fired++;
        u->has_issue = 1;

        int w = is.warp_id;
        uint8_t m = is.lane_mask;
        for (int l = 0; l < NLANES; l++)
            u->wb.wdata[l] = 0;
        for (int l = 0; l < NLANES; l++) {
            if (!((m >> l) & 1))
                continue;
            switch (is.opcode) {
            case OP_FMUL:
                u->wb.wdata[l] = f32_mul(is.opa[l], is.opb[l]);
                break;
            case OP_FADD:
                u->wb.wdata[l] = f32_add(is.opa[l], is.opb[l]);
                break;
            case OP_FNEG:
                u->wb.wdata[l] = f32_neg(is.opa[l]);
                break;
            default:
                s->err = 1;          /* 非法分发 */
                break;
            }
        }
        u->wb.warp_id = w;
        u->wb.rd = is.rd;
        u->wb.lane_mask = m;
        u->wbd.warp_id = w;
        u->wbd.rd = is.rd;
        u->wb_stage = 1;
    }
    return fired;
}
