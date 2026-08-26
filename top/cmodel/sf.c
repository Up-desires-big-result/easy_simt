/* ==========================================================================
 * sf.c — SIMT Frontend（ma_spec §2）
 *
 * 每 warp 一份 PC、active mask、分化栈（深 4）；取指发起、定长译码、
 * 立即数扩展、ld.param/csr 值构造；冒险检测与互锁（记分板，互锁不转发）；
 * issue 分派；SIMT 分化控制（判定在 ialu，处置在本模块）。
 *
 * 每 warp 指令流：授权 → 取指（IMISS）→ 译码/冒险（HAZARD）→ 读 rf →
 * 发射（EXEC/BRSTALL）→ wbdone/分支决议 → 回到可授权态。
 * ========================================================================== */
#include "sim_common.h"

#define FULL_MASK ((1u << NLANES) - 1)

/* ---------------- 辅助 ---------------- */
static uint32_t sext32(uint32_t v, int bits)
{
    uint32_t sign = 1u << (bits - 1);
    v &= (sign << 1) - 1;
    return (v & sign) ? (v | ~((sign << 1) - 1)) : v;
}

static int uniform_lane(const lanev_t v, uint8_t mask, uint32_t *out)
{
    int seen = 0;
    uint32_t x = 0;
    for (int l = 0; l < NLANES; l++) {
        if (!((mask >> l) & 1))
            continue;
        if (!seen) { x = v[l]; seen = 1; }
        else if (v[l] != x)
            return 0;
    }
    *out = x;
    return 1;
}

static void set_des(sim_t *s, int w, int reason)
{
    s->sf.des_reason[w] = reason;
}

/* ---------------- 译码 ---------------- */
typedef struct {
    int op, rd, ra, rb, rc, pd, psel, u, neg, fmt, cond;
    uint32_t imm;               /* 扩展立即数 / 分支目标 */
} dec_t;

static int decode(uint32_t w, uint32_t pc0, dec_t *d)
{
    d->op = (w >> 26) & 0x3F;
    d->rd = d->ra = d->rb = d->rc = d->pd = d->psel = 0;
    d->u = d->neg = d->fmt = d->cond = 0;
    d->imm = 0;
    switch (d->op) {
    case OP_IMAD:
        d->rd = (w >> 21) & 31; d->ra = (w >> 16) & 31;
        d->rb = (w >> 11) & 31; d->rc = (w >> 6) & 31;
        return 0;
    case OP_IADD:
    case OP_SHL:
        d->rd = (w >> 21) & 31; d->ra = (w >> 16) & 31;
        if ((w >> 15) & 1)
            d->imm = d->op == OP_IADD ? sext32(w & 0x7FFF, 15) : (w & 0x1F);
        else
            d->rb = (w >> 11) & 31;
        return 0;
    case OP_XOR:
    case OP_FMUL:
    case OP_FADD:
        d->rd = (w >> 21) & 31; d->ra = (w >> 16) & 31;
        d->rb = (w >> 11) & 31;
        return 0;
    case OP_FNEG:
        d->rd = (w >> 21) & 31; d->ra = (w >> 16) & 31;
        return 0;
    case OP_ORI:
        d->rd = (w >> 21) & 31; d->ra = (w >> 16) & 31;
        d->imm = w & 0xFFFF;
        return 0;
    case OP_LUI:
        d->rd = (w >> 21) & 31;
        d->imm = ((w >> 1) & 0xFFFFF) << 12;
        return 0;
    case OP_SETP:
        d->pd = (w >> 24) & 3;
        d->ra = (w >> 18) & 31; d->rb = (w >> 13) & 31;
        d->fmt = (w >> 12) & 1; d->cond = (w >> 9) & 7;
        return 0;
    case OP_LDG:
    case OP_STG:
        d->rd = (w >> 21) & 31; d->ra = (w >> 16) & 31;
        d->rb = (w >> 11) & 31;
        return 0;
    case OP_LDS:
    case OP_STS:
        d->rd = (w >> 21) & 31; d->rb = (w >> 11) & 31;
        return 0;
    case OP_LDP:
    case OP_CSRR:
        d->rd = (w >> 21) & 31; d->ra = (w >> 16) & 31;   /* ra 域承载 k */
        return d->ra <= 2 ? 0 : -1;
    case OP_BR:
    case OP_JOIN:
        d->psel = (w >> 24) & 3; d->u = (w >> 23) & 1;
        d->neg = (w >> 22) & 1;
        d->imm = pc0 + sext32(w & 0x3FFFFF, 22);          /* 目标 */
        return 0;
    case OP_BAR:
    case OP_RET:
        return 0;
    default:
        return -1;
    }
}

/* 冒险判定用：源/目的寄存器位集 */
static uint32_t haz_mask(const dec_t *d)
{
    uint32_t m = 0;
    switch (d->op) {
    case OP_IMAD:
        m |= (1u << d->ra) | (1u << d->rb) | (1u << d->rc);
        break;
    case OP_IADD:
    case OP_SHL:
        m |= 1u << d->ra;
        if (d->rb) m |= 1u << d->rb;
        break;
    case OP_XOR:
    case OP_FMUL:
    case OP_FADD:
    case OP_SETP:
        m |= (1u << d->ra) | (1u << d->rb);
        break;
    case OP_FNEG:
    case OP_ORI:
        m |= 1u << d->ra;
        break;
    case OP_LDG:
    case OP_STG:
        m |= (1u << d->ra) | (1u << d->rb);
        break;
    case OP_LDS:
    case OP_STS:
        m |= 1u << d->rb;
        break;
    default:
        break;
    }
    /* 目的（写寄存器的指令） */
    switch (d->op) {
    case OP_IMAD: case OP_IADD: case OP_SHL: case OP_XOR: case OP_ORI:
    case OP_LUI: case OP_FMUL: case OP_FADD: case OP_FNEG:
    case OP_LDP: case OP_CSRR:
        m |= 1u << d->rd;
        break;
    case OP_LDG:
    case OP_LDS:
        m |= 1u << d->rd;
        break;
    case OP_STG:
    case OP_STS:
        m |= 1u << d->rd;          /* rt 亦为源，已在上面按位计入 */
        break;
    default:
        break;
    }
    return m;
}

/* 读口阶段规划：返回阶段数并填充端口 */
static int read_plan(const dec_t *d, int p1[2], int p2[2])
{
    p1[0] = p1[1] = p2[0] = p2[1] = 0;
    switch (d->op) {
    case OP_IMAD:
        p1[0] = d->ra; p1[1] = d->rb; p2[0] = d->rc; p2[1] = 0;
        return 2;
    case OP_IADD:
    case OP_SHL:
        p1[0] = d->ra; p1[1] = d->rb;
        return 1;
    case OP_XOR:
    case OP_FMUL:
    case OP_FADD:
    case OP_SETP:
        p1[0] = d->ra; p1[1] = d->rb;
        return 1;
    case OP_FNEG:
    case OP_ORI:
        p1[0] = d->ra; p1[1] = 0;
        return 1;
    case OP_LDG:
        p1[0] = d->ra; p1[1] = d->rb;
        return 1;
    case OP_STG:
        p1[0] = d->ra; p1[1] = d->rb; p2[0] = d->rd; p2[1] = 0;
        return 2;
    case OP_LDS:
        p1[0] = d->rb; p1[1] = 0;
        return 1;
    case OP_STS:
        p1[0] = d->rb; p1[1] = 0; p2[0] = d->rd; p2[1] = 0;
        return 2;
    default:
        return 0;
    }
}

static void sf_err(sim_t *s)
{
    s->sf.err = 1;
    s->err = 1;
}

/* ---------------- 前置声明 ---------------- */
static void try_read_next(sim_t *s, int w);
static void issue_ready(sim_t *s, int w);

/* ---------------- 发射 ---------------- */
static void issue_ready(sim_t *s, int w)
{
    sf_t *f = &s->sf;
    sf_warp_t *wp = &f->w[w];
    dec_t d;
    if (decode(wp->inst, wp->pc, &d) < 0) { sf_err(s); return; }
    uint32_t pc0 = wp->pc;

    uint8_t mask = (uint8_t)wp->mask;

    /* ---- ialu 目的（含 LDP/CSRR 直通、BR） ---- */
    if (d.op == OP_IMAD || d.op == OP_IADD || d.op == OP_SHL ||
        d.op == OP_XOR || d.op == OP_ORI || d.op == OP_LUI ||
        d.op == OP_SETP || d.op == OP_LDP || d.op == OP_CSRR) {
        if (s->sf_ialu_issue.vld || s->ialu.has_issue) {
            wp->state = WS_ISSUE;
            return;
        }
        ialu_issue_t *p = &s->sf_ialu_issue.p;
        p->opcode = d.op;
        p->rd = d.op == OP_SETP ? d.pd : d.rd;
        p->warp_id = w;
        p->lane_mask = mask;
        p->pc = pc0;
        p->imm = 0;
        for (int l = 0; l < NLANES; l++) {
            p->opa[l] = 0; p->opb[l] = 0; p->opc[l] = 0;
        }
        switch (d.op) {
        case OP_IMAD:
            for (int l = 0; l < NLANES; l++) {
                p->opa[l] = wp->rd_data_a[l];
                p->opb[l] = wp->rd_data_b[l];
                p->opc[l] = wp->rd_data_c[l];
            }
            break;
        case OP_IADD:
        case OP_SHL:
        case OP_XOR:
            for (int l = 0; l < NLANES; l++) {
                p->opa[l] = wp->rd_data_a[l];
                p->opb[l] = d.rb ? wp->rd_data_b[l] : d.imm;
            }
            break;
        case OP_ORI:
            for (int l = 0; l < NLANES; l++) {
                p->opa[l] = wp->rd_data_a[l];
                p->opb[l] = d.imm;
            }
            break;
        case OP_LUI:
            for (int l = 0; l < NLANES; l++)
                p->opa[l] = d.imm;
            break;
        case OP_SETP:
            p->imm = (uint32_t)((d.fmt << 3) | d.cond);
            for (int l = 0; l < NLANES; l++) {
                p->opa[l] = wp->rd_data_a[l];
                p->opb[l] = wp->rd_data_b[l];
            }
            break;
        case OP_LDP:
            for (int l = 0; l < NLANES; l++)
                p->opa[l] = s->params[d.ra];
            break;
        case OP_CSRR:
            for (int l = 0; l < NLANES; l++)
                p->opa[l] = d.ra == 0 ? (uint32_t)(w * NLANES + l)
                          : d.ra == 1 ? (uint32_t)(NWARPS * NLANES)
                          : f->block_idx;
            break;
        }
        s->sf_ialu_issue.vld = 1;
        goto issued_common;
    }

    /* ---- falu 目的 ---- */
    if (d.op == OP_FMUL || d.op == OP_FADD || d.op == OP_FNEG) {
        if (s->sf_falu_issue.vld || s->falu.has_issue) {
            wp->state = WS_ISSUE;
            return;
        }
        falu_issue_t *p = &s->sf_falu_issue.p;
        p->opcode = d.op;
        p->rd = d.rd;
        p->warp_id = w;
        p->lane_mask = mask;
        for (int l = 0; l < NLANES; l++) {
            p->opa[l] = wp->rd_data_a[l];
            p->opb[l] = d.rb ? wp->rd_data_b[l] : 0;
        }
        s->sf_falu_issue.vld = 1;
        goto issued_common;
    }

    /* ---- lsu 目的 ---- */
    if (d.op == OP_LDG || d.op == OP_STG || d.op == OP_LDS ||
        d.op == OP_STS) {
        if (s->sf_lsu_issue.vld || s->lsu.busy) {
            wp->state = WS_ISSUE;
            return;
        }
        lsu_issue_t *p = &s->sf_lsu_issue.p;
        p->opcode = d.op;
        p->rd = d.rd;
        p->warp_id = w;
        p->lane_mask = mask;
        p->shbase = f->shbase;
        p->imm = 0;
        for (int l = 0; l < NLANES; l++) {
            p->opa[l] = 0; p->opb[l] = 0;
        }
        uint32_t base;
        switch (d.op) {
        case OP_LDG:
            /* 基址须为活动 lane 同值（约束 C2） */
            if (uniform_lane(wp->rd_data_a, mask, &base)) {
                p->imm = base;
                for (int l = 0; l < NLANES; l++)
                    p->opa[l] = wp->rd_data_b[l];
            } else if (uniform_lane(wp->rd_data_b, mask, &base)) {
                p->imm = base;
                for (int l = 0; l < NLANES; l++)
                    p->opa[l] = wp->rd_data_a[l];
            } else {
                sf_err(s);
                return;
            }
            break;
        case OP_STG:
            if (!uniform_lane(wp->rd_data_a, mask, &base)) {
                sf_err(s);
                return;
            }
            p->imm = base;
            for (int l = 0; l < NLANES; l++) {
                p->opa[l] = wp->rd_data_c[l];   /* 数据 R[rt] */
                p->opb[l] = wp->rd_data_b[l];   /* 偏移 R[rb] */
            }
            break;
        case OP_LDS:
            for (int l = 0; l < NLANES; l++)
                p->opa[l] = wp->rd_data_a[l];
            break;
        case OP_STS:
            for (int l = 0; l < NLANES; l++) {
                p->opa[l] = wp->rd_data_c[l];   /* 数据 R[rt] */
                p->opb[l] = wp->rd_data_a[l];   /* 偏移 R[rb] */
            }
            break;
        }
        s->sf_lsu_issue.vld = 1;
        goto issued_common;
    }

    /* ---- BR ---- */
    if (d.op == OP_BR) {
        if (s->sf_ialu_issue.vld || s->ialu.has_issue) {
            wp->state = WS_ISSUE;
            return;
        }
        ialu_issue_t *p = &s->sf_ialu_issue.p;
        p->opcode = OP_BR;
        p->rd = d.psel;
        p->warp_id = w;
        p->lane_mask = mask;
        p->pc = pc0;
        p->imm = ((uint32_t)d.u << 31) | ((uint32_t)d.neg << 30)
               | (d.imm & 0x3FFFFFFFu);
        for (int l = 0; l < NLANES; l++) {
            p->opa[l] = 0; p->opb[l] = 0; p->opc[l] = 0;
        }
        s->sf_ialu_issue.vld = 1;
        wp->br_pc = pc0;
        wp->state = WS_BR;
        set_des(s, w, R_BRSTALL);
        wp->pc = pc0;                  /* 目标由决议确定 */
        /* 记分板不涉分支；发射即完成本指令的前端职责 */
        return;
    }

    sf_err(s);
    return;

issued_common:
    ;
    /* 记分板与访存在途登记 */
    int writes_reg = 0;
    switch (d.op) {
    case OP_SETP:
        break;
    default:
        writes_reg = 1;
        break;
    }
    if (writes_reg && d.rd != 0)
        f->sb_busy[w] |= 1u << d.rd;
    wp->rd_pending = writes_reg ? d.rd : -1;
    if (d.op == OP_LDG || d.op == OP_STG || d.op == OP_LDS || d.op == OP_STS)
        f->lsu_out[w]++;
    wp->pc = pc0 + 1;
    wp->state = WS_EXEC;
    set_des(s, w, R_HAZARD);
}

/* ---------------- 读口推进 ---------------- */
static void try_read_next(sim_t *s, int w)
{
    sf_warp_t *wp = &s->sf.w[w];
    dec_t d;
    if (decode(wp->inst, wp->pc, &d) < 0) { sf_err(s); return; }
    int p1[2], p2[2];
    int phases = read_plan(&d, p1, p2);

    if (wp->rd_phase == 0) {
        if (phases == 0) {
            issue_ready(s, w);
            return;
        }
        if (!s->sf_rf_rd.vld && s->sf.rd_warp < 0) {
            s->sf_rf_rd.p.warp_id = w;
            s->sf_rf_rd.p.rs1 = p1[0];
            s->sf_rf_rd.p.rs2 = p1[1];
            s->sf_rf_rd.vld = 1;
            s->sf.rd_warp = w;
            wp->rd_phase = 1;
            wp->state = WS_RD1;
        } else {
            wp->state = WS_HAZ;        /* 读口忙，下轮重试 */
        }
        return;
    }
    if (wp->rd_phase == 2) {           /* 两阶段均完成 */
        issue_ready(s, w);
        return;
    }
}

/* ---------------- 译码入口 ---------------- */
static void decode_and_advance(sim_t *s, int w)
{
    sf_t *f = &s->sf;
    sf_warp_t *wp = &f->w[w];
    uint32_t pc0 = wp->pc;
    dec_t d;
    if (decode(wp->inst, pc0, &d) < 0) { sf_err(s); return; }

    /* 统计：译码接收即计入（无冲刷，必达发射） */
    s->st.issued++;
    s->st.lane_insns += (uint64_t)__builtin_popcount(wp->mask);
    s->st.op_count[d.op]++;
    if (d.op == OP_LDS || d.op == OP_STS) s->st.shmem_ops++;
    if (d.op == OP_LDG) s->st.ldg++;
    if (d.op == OP_STG) s->st.stg++;

    wp->rd_phase = 0;
    wp->rd_pending = -1;

    switch (d.op) {
    case OP_BAR:
        if (wp->mask != FULL_MASK) { sf_err(s); return; }   /* 架构错误 */
        wp->pc = pc0 + 1;
        if (f->lsu_out[w] == 0 && !s->sf_ws_bar.vld) {
            s->sf_ws_bar.p.warp_id = w;
            s->sf_ws_bar.p.block_id = f->block_idx;
            s->sf_ws_bar.vld = 1;
            wp->state = WS_BAR;
            set_des(s, w, R_NONE);
        } else {
            f->bar_pending[w] = 1;
            wp->state = WS_HAZ;
            set_des(s, w, R_HAZARD);
        }
        return;
    case OP_RET:
        if (wp->dsp != 0) { sf_err(s); return; }            /* 架构错误 */
        wp->pc = pc0 + 1;
        wp->state = WS_DONE;
        set_des(s, w, R_DONE);
        return;
    case OP_JOIN: {
        uint32_t target = d.imm;
        if (wp->dsp > 0 && wp->rpc[wp->dsp - 1] == target) {
            wp->mask = wp->dmask[wp->dsp - 1];
            wp->dsp--;
            wp->pc = target;
        } else if (wp->dsp > 0) {
            wp->pc = wp->rpc[wp->dsp - 1];
            wp->mask = wp->dmask[wp->dsp - 1];
            wp->dsp--;
        } else {
            wp->pc = target;
        }
        wp->state = WS_IDLE;
        set_des(s, w, R_NONE);
        return;
    }
    case OP_BR:
        wp->br_pc = pc0;
        wp->state = WS_ISSUE;          /* 走 ialu 决议 */
        set_des(s, w, R_BRSTALL);
        /* 借 issue_ready 的 BR 路径发射：需先经读口规划（BR 无读） */
        issue_ready(s, w);
        return;
    default:
        break;
    }

    /* 记分板冒险检测（互锁不转发） */
    if (f->sb_busy[w] & haz_mask(&d)) {
        wp->state = WS_HAZ;
        set_des(s, w, R_HAZARD);
        return;
    }
    try_read_next(s, w);
}

/* ---------------- 分支处置 ---------------- */
static void apply_branch(sim_t *s, int w, const br_t *br)
{
    sf_t *f = &s->sf;
    sf_warp_t *wp = &f->w[w];
    uint8_t taken = br->taken;
    uint8_t m = (uint8_t)wp->mask;
    uint8_t nt = (uint8_t)(m & (uint8_t)~taken);
    uint32_t target = br->target;
    uint32_t pc0 = wp->br_pc;

    if (taken == 0 || nt == 0) {
        /* 均匀（含全跳过/全进入） */
        s->st.uniform_br++;
        wp->pc = taken ? target : pc0 + 1;
    } else {
        s->st.diverge[pc0]++;
        if (!s->brt_valid[pc0]) { sf_err(s); return; }     /* 架构错误 */
        uint32_t R = s->brt_rpc[pc0];
        if (target == R) {
            /* 单侧跳过型 */
            if (wp->dsp >= DIV_STACK_DEPTH) { sf_err(s); return; }
            wp->rpc[wp->dsp] = R;
            wp->dmask[wp->dsp] = m;
            wp->dsp++;
            wp->mask = nt;
            wp->pc = pc0 + 1;
        } else {
            /* 双侧型：先走 taken */
            if (wp->dsp + 2 > DIV_STACK_DEPTH) { sf_err(s); return; }
            wp->rpc[wp->dsp] = R;
            wp->dmask[wp->dsp] = m;
            wp->dsp++;
            wp->rpc[wp->dsp] = pc0 + 1;
            wp->dmask[wp->dsp] = nt;
            wp->dsp++;
            wp->mask = taken;
            wp->pc = target;
        }
    }
    wp->state = WS_IDLE;
    set_des(s, w, R_NONE);
}

/* ========================================================================== */
int sf_step(sim_t *s)
{
    sf_t *f = &s->sf;
    int fired = 0;

    /* ================= 输入事务 ================= */

    /* 块启动：复位各 warp PC 与 SIMT 状态 */
    if (s->bs_sf_launch.vld && !f->active) {
        f->active = 1;
        f->block_idx = s->bs_sf_launch.p.block_idx;
        f->n = s->bs_sf_launch.p.n;
        f->shbase = s->bs_sf_launch.p.shbase;
        f->fetch_warp = -1;
        f->fetch_pend = 0;
        f->rd_warp = -1;
        for (int w = 0; w < NWARPS; w++) {
            sf_warp_t *wp = &f->w[w];
            wp->pc = 0;
            wp->mask = FULL_MASK;
            wp->dsp = 0;
            wp->state = WS_IDLE;
            wp->inst = 0;
            wp->rd_pending = -1;
            wp->br_pc = 0;
            wp->rd_phase = 0;
            f->sb_busy[w] = 0;
            f->lsu_out[w] = 0;
            f->des_reason[w] = R_NONE;
            f->cur_reason[w] = R_NONE;
            f->bar_pending[w] = 0;
        }
        s->bs_sf_launch.vld = 0;
        s->st.fires++;
        fired++;
    }

    /* 取指响应 */
    if (s->icache_sf_rsp.vld && f->fetch_warp >= 0) {
        int w = f->fetch_warp;
        f->w[w].inst = s->icache_sf_rsp.p.inst;
        f->fetch_warp = -1;
        s->icache_sf_rsp.vld = 0;
        s->st.fires++;
        fired++;
        decode_and_advance(s, w);
    }

    /* rf 读应答 */
    if (s->rf_sf_rddata.vld && f->rd_warp >= 0) {
        int w = f->rd_warp;
        sf_warp_t *wp = &f->w[w];
        if (wp->state == WS_RD1 && wp->rd_phase == 1) {
            for (int l = 0; l < NLANES; l++) {
                wp->rd_data_a[l] = s->rf_sf_rddata.p.a[l];
                wp->rd_data_b[l] = s->rf_sf_rddata.p.b[l];
            }
            f->rd_warp = -1;
            s->rf_sf_rddata.vld = 0;
            s->st.fires++;
            fired++;
            /* 需要第二阶段（IMAD/STG/STS）则续读，否则发射 */
            dec_t d;
            decode(wp->inst, wp->pc, &d);
            int p1[2], p2[2];
            if (read_plan(&d, p1, p2) == 2) {
                if (!s->sf_rf_rd.vld) {
                    s->sf_rf_rd.p.warp_id = w;
                    s->sf_rf_rd.p.rs1 = p2[0];
                    s->sf_rf_rd.p.rs2 = p2[1];
                    s->sf_rf_rd.vld = 1;
                    f->rd_warp = w;
                    wp->rd_phase = 2;
                    wp->state = WS_RD2;
                } else {
                    wp->state = WS_HAZ;
                    wp->rd_phase = -2;     /* 待重发第二阶段读 */
                }
            } else {
                issue_ready(s, w);
            }
        } else if (wp->state == WS_RD2 && wp->rd_phase == 2) {
            for (int l = 0; l < NLANES; l++)
                wp->rd_data_c[l] = s->rf_sf_rddata.p.a[l];
            f->rd_warp = -1;
            s->rf_sf_rddata.vld = 0;
            s->st.fires++;
            fired++;
            issue_ready(s, w);
        }
    }

    /* 分支决议 */
    if (s->ialu_sf_br.vld) {
        int w = s->ialu_sf_br.p.warp_id;
        apply_branch(s, w, &s->ialu_sf_br.p);
        s->ialu_sf_br.vld = 0;
        s->st.fires++;
        fired++;
    }

    /* 写回完成（三源） */
    chan_wbdone_t *wbd = 0;
    int from_lsu = 0;
    if (s->lsu_sf_wbdone.vld) { wbd = &s->lsu_sf_wbdone; from_lsu = 1; }
    else if (s->ialu_sf_wbdone.vld) wbd = &s->ialu_sf_wbdone;
    else if (s->falu_sf_wbdone.vld) wbd = &s->falu_sf_wbdone;
    if (wbd) {
        int w = wbd->p.warp_id;
        int rd = wbd->p.rd;
        if (rd != 0)
            f->sb_busy[w] &= ~(1u << rd);
        if (from_lsu && f->lsu_out[w] > 0)
            f->lsu_out[w]--;
        if (f->w[w].state == WS_EXEC) {
            f->w[w].state = WS_IDLE;
            set_des(s, w, R_NONE);
        }
        wbd->vld = 0;
        s->st.fires++;
        fired++;
    }

    /* ================= 授权接收（单在途取指，取指资源忙则背压） ================= */
    if (s->ws_sf_grant.vld && f->fetch_warp < 0 && !f->fetch_pend) {
        int w = s->ws_sf_grant.p.warp_id;
        if (f->w[w].state == WS_IDLE || f->w[w].state == WS_BAR) {
            s->ws_sf_grant.vld = 0;
            s->st.fires++;
            fired++;
            /* 启动新指令：发起取指 */
            f->w[w].state = WS_FETCH;
            f->fetch_warp = w;
            f->fetch_pend = 1;
            set_des(s, w, R_IMISS);
        }
    }

    /* ================= 各 warp 内部推进 ================= */
    for (int w = 0; w < NWARPS; w++) {
        sf_warp_t *wp = &f->w[w];
        switch (wp->state) {
        case WS_FETCH:
            if (f->fetch_pend && f->fetch_warp == w &&
                !s->sf_icache_req.vld) {
                s->sf_icache_req.p.pc = wp->pc;
                s->sf_icache_req.vld = 1;
                f->fetch_pend = 0;
            }
            break;
        case WS_HAZ:
            if (f->bar_pending[w]) {
                /* 屏障：等 LSU 排空 + 通道可用 */
                if (f->lsu_out[w] == 0 && !s->sf_ws_bar.vld) {
                    s->sf_ws_bar.p.warp_id = w;
                    s->sf_ws_bar.p.block_id = f->block_idx;
                    s->sf_ws_bar.vld = 1;
                    f->bar_pending[w] = 0;
                    wp->state = WS_BAR;
                    set_des(s, w, R_NONE);
                }
            } else if (wp->rd_phase == -1) {
                try_read_next(s, w);
            } else if (wp->rd_phase == -2) {
                /* 第二阶段读重发 */
                dec_t d;
                decode(wp->inst, wp->pc, &d);
                int p1[2], p2[2];
                read_plan(&d, p1, p2);
                if (!s->sf_rf_rd.vld && f->rd_warp < 0) {
                    s->sf_rf_rd.p.warp_id = w;
                    s->sf_rf_rd.p.rs1 = p2[0];
                    s->sf_rf_rd.p.rs2 = p2[1];
                    s->sf_rf_rd.vld = 1;
                    f->rd_warp = w;
                    wp->rd_phase = 2;
                    wp->state = WS_RD2;
                }
            } else {
                /* 记分板互锁：清除后重新进入读/发射 */
                dec_t d;
                if (decode(wp->inst, wp->pc, &d) == 0 &&
                    !(f->sb_busy[w] & haz_mask(&d)))
                    try_read_next(s, w);
            }
            break;
        case WS_ISSUE:
            issue_ready(s, w);
            break;
        default:
            break;
        }
    }

    /* ================= 停顿原因上报 ================= */
    for (int w = 0; w < NWARPS; w++) {
        if (f->des_reason[w] != f->cur_reason[w] && !s->sf_ws_stall.vld) {
            s->sf_ws_stall.p.warp_id = w;
            s->sf_ws_stall.p.reason = f->des_reason[w];
            s->sf_ws_stall.vld = 1;
            f->cur_reason[w] = f->des_reason[w];
        }
    }

    /* 全 warp 结束：释放块占用（供下一块 launch 握手） */
    if (f->active) {
        int all_done = 1;
        for (int w = 0; w < NWARPS; w++)
            if (f->w[w].state != WS_DONE) { all_done = 0; break; }
        if (all_done)
            f->active = 0;
    }

    if (f->err)
        s->err = 1;
    return fired;
}
