/* ==========================================================================
 * bs.c — Block Scheduler（ma_spec §4）
 *
 * 纯块派发：推进 grid、下发启动上下文 {blockIdx, N, SHBASE}、收 block_done
 * 拉下一块；MAX_BLOCKS_INFLIGHT=1（v1 串行，参数位保留）。
 *
 * 握手时序约定（全模型统一）：生产者置 vld 并保持，消费者在自己 step
 * 内清零完成接收；同一轮询轮内后序模块可见先序模块置位的 vld。
 * ========================================================================== */
#include "sim_common.h"

int bs_step(sim_t *s)
{
    bs_t *b = &s->bs;
    int fired = 0;

    /* 收 block_done：推进下一块或停机 */
    if (s->ws_bs_bdone.vld) {
        b->inflight = 0;
        b->block_idx++;
        s->ws_bs_bdone.vld = 0;
        s->st.fires++;
        fired++;
        if (b->block_idx >= (uint32_t)s->grid) {
            b->done = 1;                 /* bs_top_done 锁存 */
            s->done = 1;
        }
    }

    /* 派发：两条 launch 通道同拍发起，两侧均握手成功后块启动完成 */
    if (b->started && !b->inflight && !b->done) {
        if (!s->bs_sf_launch.vld && !s->sf.active) {
            s->bs_sf_launch.p.block_idx = b->block_idx;
            s->bs_sf_launch.p.n = (uint32_t)s->n;
            s->bs_sf_launch.p.shbase = 0;      /* v1 单块在途恒 0 */
            s->bs_sf_launch.vld = 1;
        }
        if (!s->bs_ws_launch.vld && !s->ws.launched) {
            s->bs_ws_launch.p.block_idx = b->block_idx;
            s->bs_ws_launch.p.n = (uint32_t)s->n;
            s->bs_ws_launch.p.shbase = 0;
            s->bs_ws_launch.vld = 1;
        }
        /* 两侧均已接收 → 启动完成，触发复位分发 */
        if (s->sf.active && s->ws.launched) {
            b->inflight = 1;
            sim_block_start(s);
        }
    }
    return fired;
}
