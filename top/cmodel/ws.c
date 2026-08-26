/* ==========================================================================
 * ws.c — Warp Scheduler（ma_spec §3）
 *
 * 只管 warp 粒度行为：按 warp id 顺序轮转选发射（2 位指针）；汇聚停顿源
 * （sf/lsu 两来源）；bar.sync 到达计数、凑齐 NWARPS 统一释放；
 * 块完成判定（全 warp DONE 后发 block_done）。
 *
 * 停顿聚合：warp 可发射 ⟺ 无任何停顿源（任一非 NONE 即停）。
 * ========================================================================== */
#include "sim_common.h"

static int warp_runnable(sim_t *s, int w)
{
    ws_t *sc = &s->ws;
    return sc->launched
        && !sc->done[w]
        && !sc->barrier[w]
        && sc->sf_reason[w] == R_NONE
        && sc->lsu_reason[w] == R_NONE;
}

int ws_step(sim_t *s)
{
    ws_t *sc = &s->ws;
    int fired = 0;

    /* ---- 输入事务 ---- */
    if (s->bs_ws_launch.vld) {
        sc->launched = 1;
        sc->block_id = s->bs_ws_launch.p.block_idx;
        for (int w = 0; w < NWARPS; w++) {
            sc->sf_reason[w] = R_NONE;
            sc->lsu_reason[w] = R_NONE;
            sc->barrier[w] = 0;
            sc->done[w] = 0;
        }
        sc->bar_count = 0;
        sc->ptr = 0;
        sc->grant_warp = -1;
        sc->bdone_sent = 0;
        s->bs_ws_launch.vld = 0;
        s->st.fires++;
        fired++;
    }

    if (s->sf_ws_stall.vld) {
        int w = s->sf_ws_stall.p.warp_id;
        sc->sf_reason[w] = s->sf_ws_stall.p.reason;
        if (s->sf_ws_stall.p.reason == R_DONE)
            sc->done[w] = 1;
        s->sf_ws_stall.vld = 0;
        s->st.fires++;
        fired++;
    }

    if (s->lsu_ws_stall.vld) {
        int w = s->lsu_ws_stall.p.warp_id;
        sc->lsu_reason[w] = s->lsu_ws_stall.p.reason;
        s->lsu_ws_stall.vld = 0;
        s->st.fires++;
        fired++;
    }

    if (s->sf_ws_bar.vld) {
        int w = s->sf_ws_bar.p.warp_id;
        if (!sc->barrier[w]) {
            sc->barrier[w] = 1;
            sc->bar_count++;
        }
        s->sf_ws_bar.vld = 0;
        s->st.fires++;
        fired++;
        if (sc->bar_count == NWARPS) {       /* 凑齐：统一释放 */
            for (int i = 0; i < NWARPS; i++)
                sc->barrier[i] = 0;
            sc->bar_count = 0;
        }
    }

    /* ---- 输出：发射授予（自指针起按 warp id 顺序找第一个可发射） ---- */
    if (sc->grant_warp >= 0 && !s->ws_sf_grant.vld) {
        /* 授予已被 sf 接收：指针推进到其下一个 */
        sc->ptr = (sc->grant_warp + 1) % NWARPS;
        sc->grant_warp = -1;
        s->st.fires++;
        fired++;
    }
    if (sc->grant_warp < 0 && !s->ws_sf_grant.vld && sc->launched) {
        for (int i = 0; i < NWARPS; i++) {
            int w = (sc->ptr + i) % NWARPS;
            if (!warp_runnable(s, w))
                continue;
            /* sf 侧 rdy：该 warp 处于可启动新指令的状态 */
            int st = s->sf.w[w].state;
            if (st != WS_IDLE && st != WS_BAR)
                continue;
            s->ws_sf_grant.p.warp_id = w;
            s->ws_sf_grant.vld = 1;
            sc->grant_warp = w;
            break;
        }
    }

    /* ---- 输出：块完成 ---- */
    if (sc->launched) {
        int all_done = 1;
        for (int w = 0; w < NWARPS; w++)
            if (!sc->done[w]) { all_done = 0; break; }
        if (all_done && !s->ws_bs_bdone.vld && !sc->bdone_sent) {
            s->ws_bs_bdone.p.block_idx = sc->block_id;
            s->ws_bs_bdone.vld = 1;
            sc->bdone_sent = 1;
        }
        if (sc->bdone_sent && !s->ws_bs_bdone.vld) {
            sc->bdone_sent = 0;
            sc->launched = 0;              /* 本块交回，等下一块 launch */
        }
    }
    return fired;
}
