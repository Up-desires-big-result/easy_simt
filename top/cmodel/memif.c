/* ==========================================================================
 * memif.c — Memory Interface（ma_spec §10）
 *
 * 片外唯一通道：仲裁 icache 与 l1sm 的回填/写通请求，固定优先级
 * （icache 优先），单请求在途。C 模型将片外行为收拢在本模块：
 *   - 片外存储（指令段 + 两段全局数据）作为行为级后备；
 *   - AXI 从设备固定延迟 MEM_LAT 建模为响应倒计时（intf_spec §10：
 *     该延迟归 tb 侧 AXI 从设备，memif 自身不加延迟）；
 *   - 非 OKAY 响应在 C 模型中不存在，越界访问置 memif_top_err。
 * ========================================================================== */
#include "sim_common.h"

/* ---------------- 片外后备存储访问 ---------------- */
static int gmem_region(sim_t *s, uint32_t addr, uint32_t *off)
{
    memif_t *m = &s->memif;
    if (addr >= m->in_base && addr < m->in_base + 4u * GMEM_WORDS) {
        *off = (addr - m->in_base) >> 2;
        return 1;
    }
    if (addr >= m->out_base && addr < m->out_base + 4u * GMEM_WORDS) {
        *off = (addr - m->out_base) >> 2;
        return 2;
    }
    return 0;
}

static uint32_t gmem_read(sim_t *s, uint32_t addr)
{
    uint32_t off;
    int r = gmem_region(s, addr, &off);
    memif_t *m = &s->memif;
    if (r == 1) return m->gmem_in[off];
    if (r == 2) return m->gmem_out[off];
    m->err = 1;
    s->err = 1;
    return 0;
}

static void gmem_write(sim_t *s, uint32_t addr, uint32_t v)
{
    uint32_t off;
    int r = gmem_region(s, addr, &off);
    memif_t *m = &s->memif;
    if (r == 1) { m->gmem_in[off] = v; return; }
    if (r == 2) { m->gmem_out[off] = v; return; }
    m->err = 1;
    s->err = 1;
}

int memif_step(sim_t *s)
{
    memif_t *m = &s->memif;
    int fired = 0;

    /* ---- 在途事务：延迟倒计时 → 提交 → 响应 ---- */
    if (m->busy) {
        if (m->countdown > 0) {
            m->countdown--;
            fired++;               /* 延迟时隙计为活动，防误判死锁 */
        } else if (!m->rsp_set) {
            /* 到期：提交并置位响应 */
            if (m->rw) {
                gmem_write(s, m->addr, m->wdata);
                s->st.memif_wr++;
                for (int i = 0; i < ILINE_WORDS; i++)
                    s->memif_l1sm_rsp.p.line[i] = 0;
                s->memif_l1sm_rsp.vld = 1;
            } else if (m->to_icache) {
                uint32_t w0 = m->addr >> 2;
                for (int i = 0; i < ILINE_WORDS; i++)
                    s->memif_icache_rsp.p.line[i] =
                        (w0 + i < (uint32_t)m->imem_n)
                            ? m->imem[w0 + i] : 0;
                s->memif_icache_rsp.vld = 1;
                s->st.memif_icache_rd++;
            } else {
                for (int i = 0; i < ILINE_WORDS; i++)
                    s->memif_l1sm_rsp.p.line[i] =
                        gmem_read(s, m->addr + 4u * i);
                s->memif_l1sm_rsp.vld = 1;
                s->st.memif_rd++;
            }
            m->rsp_set = 1;
            fired++;               /* 提交响应计为活动 */
        } else {
            /* 等消费者取走响应 */
            int taken = m->to_icache ? !s->memif_icache_rsp.vld
                                     : !s->memif_l1sm_rsp.vld;
            if (taken) {
                m->busy = 0;
                m->rsp_set = 0;
                s->st.fires++;
                fired++;
            }
        }
    }

    /* ---- 接收请求：固定优先级 icache 优先，单在途 ---- */
    if (!m->busy) {
        if (s->icache_memif_req.vld) {
            m->busy = 1;
            m->to_icache = 1;
            m->rw = 0;
            m->addr = s->icache_memif_req.p.addr;
            m->countdown = m->memlat;
            s->icache_memif_req.vld = 0;
            s->st.fires++;
            fired++;
        } else if (s->l1sm_memif_req.vld) {
            m->busy = 1;
            m->to_icache = 0;
            m->rw = s->l1sm_memif_req.p.rw;
            m->addr = s->l1sm_memif_req.p.addr;
            m->wdata = s->l1sm_memif_req.p.wdata;
            m->countdown = m->memlat;
            s->l1sm_memif_req.vld = 0;
            s->st.fires++;
            fired++;
        }
    }
    return fired;
}
