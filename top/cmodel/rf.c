/* ==========================================================================
 * rf.c — Register File（ma_spec §11）
 *
 * NWARPS × NLANES × 32 × 32b，每 lane 独立副本；R0 恒零（写忽略、读恒 0）。
 * 读：译码双读口（rs1/rs2），应答与请求严格顺序对应（单在途）。
 * 写：单写口三源固定优先级仲裁 lsu > ialu > falu，lane 掩码随路。
 * ========================================================================== */
#include "sim_common.h"

int rf_step(sim_t *s)
{
    rf_t *r = &s->rf;
    int fired = 0;

    /* ---- 写口：三源固定优先级，每轮至多一笔 ---- */
    chan_wb_t *wb = 0;
    if (s->lsu_rf_wb.vld)
        wb = &s->lsu_rf_wb;
    else if (s->ialu_rf_wb.vld)
        wb = &s->ialu_rf_wb;
    else if (s->falu_rf_wb.vld)
        wb = &s->falu_rf_wb;
    if (wb) {
        int w = wb->p.warp_id, rd = wb->p.rd;
        if (rd != 0) {
            for (int l = 0; l < NLANES; l++)
                if ((wb->p.lane_mask >> l) & 1)
                    r->r[w][rd][l] = wb->p.wdata[l];
        }
        wb->vld = 0;
        s->st.fires++;
        fired++;
    }

    /* ---- 读口：请求 → 应答（单在途，顺序对应） ---- */
    if (s->sf_rf_rd.vld && !s->rf_sf_rddata.vld) {
        int w = s->sf_rf_rd.p.warp_id;
        int rs1 = s->sf_rf_rd.p.rs1, rs2 = s->sf_rf_rd.p.rs2;
        for (int l = 0; l < NLANES; l++) {
            s->rf_sf_rddata.p.a[l] = rs1 ? r->r[w][rs1][l] : 0;
            s->rf_sf_rddata.p.b[l] = rs2 ? r->r[w][rs2][l] : 0;
        }
        s->rf_sf_rddata.vld = 1;
        s->sf_rf_rd.vld = 0;
        s->st.fires++;
        fired++;
    }
    return fired;
}
