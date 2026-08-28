/* ==========================================================================
 * dpi_ref.c — C 事务级模型的 DPI-C 参考模型前端（Makefile dpi/cosim）
 *
 * 为 SystemVerilog testbench 提供 C 模型参考接口。当前服务 bs 单元验证
 * （bs/tb/tb_bs.sv）：
 *
 *   ref_bs_init   — 初始化参考上下文：N 与 grid 口径与 top/cmodel 的
 *                   sim_init 一致（grid = ceil(N / (NWARPS*NLANES))），
 *                   bs.started 置 1。
 *   ref_bs_cycle  — 按 testbench 给定的本拍消费者握手决策（sf/ws 的
 *                   launch rdy）与 block_done 输入推进一轮：调用
 *                   bs_step()，并在本函数内完成消费者侧对 launch 的接收
 *                   与 block_done 时序镜像（外部 testbench 不直接触碰
 *                   sim_t）。返回参考侧各通道发射与状态。
 *
 * 时序镜像口径（与 top/cmodel 的 ws.c/sf.c 行为一致）：
 *   - 真实流程中，ws 发出 block_done 前，sf 已在全 warp ret 后释放块
 *     占用（active 清零），故注入 block_done 前先清 sf.active；
 *   - ws 在 block_done 被消费后清 launched。
 *
 * 注：bs_step() 于启动完成时调用 sim_block_start()（谓词清零、SM 行
 * 钉扎），为顶层互连位置的副作用，参考侧照常执行，不作为比对项。
 * ========================================================================== */
#include <string.h>
#include "sim_common.h"

static sim_t g_ref;

void ref_bs_init(int n)
{
    memset(&g_ref, 0, sizeof g_ref);
    g_ref.n = n;
    g_ref.grid = (n + NWARPS * NLANES - 1) / (NWARPS * NLANES);
    g_ref.bs.started = 1;
}

void ref_bs_cycle(int sf_rdy, int ws_rdy, int bdone_vld, int bdone_idx,
                  int *ref_sf_fire, int *ref_ws_fire,
                  int *ref_bdone_consumed, int *ref_done, int *ref_inflight,
                  int *ref_sf_block_idx, int *ref_sf_n, int *ref_sf_shbase,
                  int *ref_ws_block_idx, int *ref_block_idx)
{
    sim_t *s = &g_ref;
    int had_bdone;

    /* ---- block_done 注入（仅协议内：有块在途时才注入，由 testbench 保证） ---- */
    if (bdone_vld && s->bs.inflight && !s->ws_bs_bdone.vld) {
        s->sf.active = 0;               /* sf 已于 ws 发 block_done 前释放块占用 */
        s->ws_bs_bdone.vld = 1;
        s->ws_bs_bdone.p.block_idx = (uint32_t)bdone_idx;
    }
    had_bdone = s->ws_bs_bdone.vld;

    /* ---- 推进参考模型 ---- */
    bs_step(s);

    *ref_bdone_consumed = had_bdone && !s->ws_bs_bdone.vld;
    if (*ref_bdone_consumed)
        s->ws.launched = 0;             /* ws 在 block_done 被消费后清 launched */

    /* ---- 消费者侧接收 launch（rdy 决策来自 testbench） ---- */
    *ref_sf_fire = 0;
    *ref_ws_fire = 0;
    if (s->bs_sf_launch.vld && sf_rdy) {
        *ref_sf_fire      = 1;
        *ref_sf_block_idx = (int)s->bs_sf_launch.p.block_idx;
        *ref_sf_n         = (int)s->bs_sf_launch.p.n;
        *ref_sf_shbase    = (int)s->bs_sf_launch.p.shbase;
        s->bs_sf_launch.vld = 0;
        s->sf.active = 1;
    }
    if (s->bs_ws_launch.vld && ws_rdy) {
        *ref_ws_fire      = 1;
        *ref_ws_block_idx = (int)s->bs_ws_launch.p.block_idx;
        s->bs_ws_launch.vld = 0;
        s->ws.launched = 1;
    }

    /* ---- 状态返回 ---- */
    *ref_done      = s->bs.done;
    *ref_inflight  = s->bs.inflight;
    *ref_block_idx = (int)s->bs.block_idx;
}
