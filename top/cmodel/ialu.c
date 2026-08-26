/* ==========================================================================
 * ialu.c — Integer ALU（ma_spec §5）
 *
 * 整数运算 + 分支解析。数据通路 8 lane 并行、锁步。
 * 谓词寄存器 P0..P3 物理上驻留本模块（SETp 写、BR 读，"直接喂分支判定"，
 * ma_spec §5），块启动时清零。
 *
 * sf 侧载荷约定（均为 sf 译码期归一化后的形式）：
 *   - IADD/SHL/ORI：opa=R[ra]，opb=第二源（寄存器值或立即数广播）；
 *   - LUI/LDP/CSRR：opa=写入值广播，直通产出（偏差 C3）；
 *   - SETP：rd=pd，imm=(fmt<<3)|cond；
 *   - BR：rd=psel，imm=(u<<31)|(neg<<30)|target。
 * ========================================================================== */
#include "sim_common.h"

static int32_t s32(uint32_t u)
{
    return (int32_t)u;
}

int ialu_step(sim_t *s)
{
    ialu_t *u = &s->ialu;
    int fired = 0;

    /* ---- 输出排空：br → wb → wbdone（置位后等消费者清零） ---- */
    if (u->br_stage) {
        if (!u->br_sent) {
            if (!s->ialu_sf_br.vld) {
                s->ialu_sf_br.p = u->br;
                s->ialu_sf_br.vld = 1;
                u->br_sent = 1;
            }
        } else if (!s->ialu_sf_br.vld) {
            u->br_stage = 0;
            u->br_sent = 0;
            u->has_issue = 0;
            s->st.fires++;
            fired++;
        }
    }
    if (u->wb_stage == 1) {
        if (!u->wb_sent) {
            if (!s->ialu_rf_wb.vld) {
                s->ialu_rf_wb.p = u->wb;
                s->ialu_rf_wb.vld = 1;
                u->wb_sent = 1;
            }
        } else if (!s->ialu_rf_wb.vld) {
            u->wb_stage = 2;
            u->wb_sent = 0;
            s->st.fires++;
            fired++;
        }
    }
    if (u->wb_stage == 2) {
        if (!u->wbd_sent) {
            if (!s->ialu_sf_wbdone.vld) {
                s->ialu_sf_wbdone.p = u->wbd;
                s->ialu_sf_wbdone.vld = 1;
                u->wbd_sent = 1;
            }
        } else if (!s->ialu_sf_wbdone.vld) {
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
    if (!u->has_issue && s->sf_ialu_issue.vld) {
        ialu_issue_t is = s->sf_ialu_issue.p;
        s->sf_ialu_issue.vld = 0;
        s->st.fires++;
        fired++;
        u->has_issue = 1;
        u->wb_stage = 0;
        u->br_stage = 0;

        int w = is.warp_id;
        uint8_t m = is.lane_mask;
        lanev_t out;
        for (int l = 0; l < NLANES; l++) out[l] = 0;

        switch (is.opcode) {
        case OP_IMAD:
            for (int l = 0; l < NLANES; l++)
                if ((m >> l) & 1) {
                    int64_t v = (int64_t)s32(is.opa[l]) * s32(is.opb[l])
                              + s32(is.opc[l]);
                    out[l] = (uint32_t)v;
                }
            break;
        case OP_IADD:
            for (int l = 0; l < NLANES; l++)
                if ((m >> l) & 1)
                    out[l] = is.opa[l] + is.opb[l];
            break;
        case OP_SHL:
            for (int l = 0; l < NLANES; l++)
                if ((m >> l) & 1)
                    out[l] = is.opa[l] << (is.opb[l] & 31);
            break;
        case OP_XOR:
            for (int l = 0; l < NLANES; l++)
                if ((m >> l) & 1)
                    out[l] = is.opa[l] ^ is.opb[l];
            break;
        case OP_ORI:
            for (int l = 0; l < NLANES; l++)
                if ((m >> l) & 1)
                    out[l] = is.opa[l] | is.opb[l];
            break;
        case OP_LUI:
        case OP_LDP:
        case OP_CSRR:
            for (int l = 0; l < NLANES; l++)
                if ((m >> l) & 1)
                    out[l] = is.opa[l];       /* 直通（偏差 C3） */
            break;
        case OP_SETP: {
            int fmt = (is.imm >> 3) & 1, cond = is.imm & 7;
            uint32_t res = 0;
            for (int l = 0; l < NLANES; l++) {
                if (!((m >> l) & 1)) continue;
                int ok;
                if (fmt == 0) {
                    int32_t a = s32(is.opa[l]), b = s32(is.opb[l]);
                    ok = cond == 0 ? a < b  : cond == 1 ? a <= b
                       : cond == 2 ? a == b : cond == 3 ? a != b
                       : cond == 4 ? a >= b : a > b;
                } else {
                    uint32_t a = is.opa[l], b = is.opb[l];
                    ok = cond == 5 ? f32_gt(a, b)
                       : cond == 4 ? (f32_gt(a, b) || a == b)
                       : cond == 0 ? f32_gt(b, a)
                       : cond == 1 ? (f32_gt(b, a) || a == b)
                       : cond == 2 ? (a == b)
                       :             (a != b);
                }
                if (ok) res |= 1u << l;
            }
            int pd = is.rd;
            u->pred[w][pd] = (uint8_t)((u->pred[w][pd] & ~m) | (res & m));
            /* 无寄存器写回：直接走 wbdone */
            u->wbd.warp_id = w;
            u->wbd.rd = pd;
            u->wb_stage = 2;
            return fired;
        }
        case OP_BR: {
            int psel = is.rd;
            int br_u = (is.imm >> 31) & 1;
            int neg = (is.imm >> 30) & 1;
            uint32_t target = is.imm & 0x3FFFFFFFu;
            uint8_t p = u->pred[w][psel];
            uint8_t taken;
            if (br_u)
                taken = m;
            else
                taken = neg ? (uint8_t)(m & (uint8_t)~p)
                            : (uint8_t)(m & p);
            u->br.warp_id = w;
            u->br.taken = taken;
            u->br.target = target;
            u->br.brt_idx = 0;       /* sf 按分支 pc 查 BRT */
            u->br_stage = 1;
            return fired;
        }
        default:
            s->err = 1;              /* 非法分发 */
            return fired;
        }

        /* 通用写回路径 */
        u->wb.warp_id = w;
        u->wb.rd = is.rd;
        u->wb.lane_mask = m;
        for (int l = 0; l < NLANES; l++)
            u->wb.wdata[l] = out[l];
        u->wbd.warp_id = w;
        u->wbd.rd = is.rd;
        u->wb_stage = 1;
    }
    return fired;
}
