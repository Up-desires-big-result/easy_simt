# l1sm — L1 + Shared Memory（统一 SRAM）模块规范

版本：v0.1
日期：2026-09-15
依据文档：`top/docs/ma_spec_v0.1.md`（§1.2、§1.4、§1.6、§8）、`top/docs/intf_spec_v0.1.md`（§1、§8）、`top/cmodel/l1sm.c`、`top/cmodel/lsu.c`、`top/cmodel/top.c`、`top/cmodel/sim_common.h`。
适用范围：本文档定义 l1sm（L1 + Shared Memory，统一 SRAM）的模块级设计：端口、参数、内部状态、访问路径、服务节拍、状态机、通道时序与协议约束，是 `l1sm/rtl/l1sm.sv` 实现与 `l1sm/tb/` 验证的直接依据。
修订记录：（首版）

---

## 1. 总体

### 1.1 模块定位与职责

l1sm 是统一数据存储：L1 数据缓存与共享内存**共用一块统一 SRAM**（数据阵列 + 一份 tag 阵列），行的概念归本模块管（ma_spec §1.4）。职责：

- 接收 lsu 的 8-lane 请求，按 `sm` 位分流到 SM / L1 两套 tag 规约；
- **8-bank 锁步访问**：一拍对同行的全部无冲突 lane 并行读/写（单行无 bank 冲突单拍完成，§1.4）；
- L1 缺失阻塞：经 memif 回填整行后重放；写直通不写分配（WT_NOALLOC）；
- SM 钉扎：不可缺失、不可被替换；
- 写通存储逐字下发 memif（4B 窄传），等写应答返回才算完成。

边界：不含片外访问（memif 职责）；不含地址生成与 mask 门控（lsu 职责，shmem 地址已含 SHBASE，ma_spec §6）；无 coalesce（8-bank 按字偏移天然分流，仅跨行/bank 冲突才串行）。

### 1.2 存储组织

| 项 | 值 | 说明 |
|---|---|---|
| 数据阵列 | `U_LINES` 行 × 32B | 每行 8 字，按字偏移分 `NBANKS=8` bank（bank=addr[4:2]，与 NLANES 对齐） |
| tag 阵列 | `U_LINES` 项，每项 `{class[1:0], valid, tag}` | class：00=INVALID、01=L1、10=SM（钉扎）；无脏位（写直通不写分配） |
| SM 分区 | 行 `[0, SM_LINES)` | 复位时钉扎（§1.3、§8） |
| L1 分区 | 行 `[SM_LINES, U_LINES)`，`nsets = U_LINES − SM_LINES` | 直接映射，set 映射见 §1.3 |

基线 `U_LINES=64`（2KB）、`SM_LINES=4`（128B）、`NBANKS=8`。数据阵列与 tag 阵列均为寄存器实现（回填整行单沿写入，同 icache 先例）；SRAM 宏实现为后续优化项。

### 1.3 tag 管理策略

**SM 侧（钉扎自指哨兵）**：复位时对 `[0, SM_LINES)` 每行写一次 tag：`tag := 该行自身索引`（自指哨兵）、`class := SM`、`valid := 1`、钉扎。SM 请求 `sm_row = addr >> 5`（v1 shbase 恒 0，§3），直接落物理行 sm_row，查 tag 做 `class==SM && valid && tag==sm_row` 的恒等/类校验——SM **不会缺失、不可被替换**，tag 阵列在此兼任完整性校验。v1 复位时钉扎一次即等价每块重钉（钉扎幂等：L1 分配永不触碰 SM 区，重复钉扎写同值）；多块并发时增设块启动触发与 shbase 配置（参数位，§3）。

**L1 侧（偏移索引、正常 tag）**：`set = line mod nsets`（`line = addr >> 5`，`nsets=60` 非 2 的幂，取精确模，不近似），物理行 = `SM_LINES + set`；命中条件 `class==L1 && valid && tag==line`。缺失阻塞、回填整行后以 `class=L1、valid=1、tag=line` 覆写分配（直接映射）。L1 索引空间整体偏移到 SM 区之上，**两套规约永不互相别名**，L1 替换永不触碰 SM 钉扎行。

### 1.4 服务节拍与冲突串行化

**节拍（beat）**：本模块的服务单位。一拍内完成一次**行级 tag 比较**与该行内各 bank 的**并行访问**；一拍至多服务一个行组。

**行组划分**：自当前服务游标起按 lane 升序扫描活动 lane，行组 = 行相同且 bank 互不重复的连续活动 lane 集合（bank 重复含同字访问：两 lane 同 bank 即分属不同行组）。

**单拍条件**：请求的全部活动 lane 落在同一行且 8 个 bank 互不冲突——**一拍完成**（黄金程序 stride-1 访问恒满足，与基线"无 bank conflict"一致）。

**冲突串行化**：跨行或 bank 冲突时按行组逐拍串行服务（每拍一个行组，lane 升序），属例外而非常态——不是逐 lane 串行的基线机制。同 bank（含同字）访问按 lane 升序串行，语义与参考模型一致（同字写以后写 lane 为准）。

**例外计数**：cmodel 统计 `l1sm_conflict`（跨行或 bank 冲突的请求数，黄金程序期望 0）；本模块无计数端口，冲突由验证侧按激励统计（§10）。

### 1.5 时序模型

- 服务数据通路（tag 比较、bank 并行读写）在行组节拍内组合完成，结果于该拍末沿入寄存器；输出通道 `vld` 为寄存器输出（状态译码），`vld && rdy` 同拍为高的时钟末沿完成握手（intf_spec §1.2）；
- 回填整行（8 字）于 memif 响应握手拍末沿**单沿写入**数据阵列（寄存器阵列实现的前提）；
- 片外固定延迟 `MEM_LAT` 归 tb 侧从设备（与 memif_spec §1.5 同构：请求握手拍末沿装载倒计时、每拍递减、计数到 0 呈现响应，呈现拍 = 请求握手拍 + MEM_LAT + 1，含 MEM_LAT=0），本模块自身不加延迟；
- 复位为低电平有效异步复位（intf_spec §1.3），复位期间全部 `vld = 0`。

### 1.6 与 C 模型的对应关系

`top/cmodel/l1sm.c` 的 `l1sm_step()` 为事务级参考。**服务时序口径差异（已声明）**：cmodel 逐活动 lane 串行服务（`L_SVC` 每步一个 lane），本模块按行组 8-bank 并行——两侧**边界事务序列完全一致**（请求→响应逐笔对应、回填与写通事务的笔数与顺序相同、载荷位精确），周期不具可比性，验证采用事务级比对（§10）；cmodel 切换为 bank 并行时序为待办项。

| C 模型（`l1sm_t` / `l1sm_step`） | l1sm RTL |
|---|---|
| `busy && stage==L_SVC` 逐 lane 服务（升序游标 `glane`） | `S_GRP` 按行组服务（升序游标 `glane_q`，行组划分 §1.4） |
| SM 读/写：`glane++` 同步推进 | SM 行组节拍内 bank 并行读/写 |
| L1 读命中：`rsp.rdata[l]` 装载、`glane++` | L1 命中行组节拍内并行装载 |
| L1 读缺失：`L_RFILL_SET/WAIT` → 整行写入 → 重放该 lane | `S_REFILL` → 整行单沿写入 → 重放该行组 |
| 存储命中更新行内字、缺失不更新（写直通不写分配） | 同义（行组节拍内并行更新 / 缺失不更新） |
| `L_WR_SET/WAIT` 逐 lane 写通（字地址 4B）→ `glane++` | `S_WT` 逐 lane 写通（升序，笔数与顺序同） |
| `L_RSP` 响应呈现保持至握手 | `S_RSP` 同 |
| 空闲拍接收 lsu 请求、`rsp.rdata` 清零 | `S_IDLE` 接收、`rsp_q` 清零 |
| SM 钉扎完整性校验失败置 `s->err` | 无错误端口：请求源保证合法（§9 条 3），协议外行为不作约定 |
| `l1sm_conflict` 例外计数 | 无端口，验证侧统计（§1.4） |
| `sim_block_start()` 每块重钉 tag、置 `shbase` | 复位时钉扎一次（幂等，§1.3）；shbase 恒 0 不设端口（§3） |

### 1.7 验收口径

功能验收为事务级等价（ma_spec §1.7：周期只作观测项，不作验收项；本模块与 cmodel 服务时序口径不同，§1.6）：

- `lsu_l1sm_req` 接收 → `l1sm_lsu_rsp` 交付逐笔对应，装载 `rdata` 位精确、存储 `rdata` 恒 0；
- `l1sm_memif_req` / `memif_l1sm_rsp` 事务序列（回填与写通的笔数、顺序、载荷）与参考模型逐笔一致；
- 单行无冲突请求单服务节拍完成（周期级断言，§10 L3）；
- SM 钉扎永不被替换、L1 替换永不触碰 SM 区。

---

## 2. 端口

端口命名、方向与位宽见 intf_spec §8（8 端口：`lsu_l1sm_req` / `l1sm_lsu_rsp` / `l1sm_memif_req` / `memif_l1sm_rsp` 四通道各 vld+载荷+rdy）。`clk`/`rst_n` 按 intf_spec §1.3 携带。模块补充如下：

- 无配置端口：`shbase` v1 恒 0（lsu 侧已叠加，ma_spec §6），本模块不再扣减；多块并发时增设块启动触发与 `shbase` 配置输入（参数位，§3）；
- 无错误端口：SM 钉扎完整性校验在 cmodel 内为安全网，本模块按请求源合法性约束执行（§9 条 3）。

---

## 3. 参数与配置

| 参数 | 基线值 | 含义 |
|---|---|---|
| `DATA_W` | 32 | 数据/地址位宽（intf_spec §1.4） |
| `NWARPS` | 4 | warp 数/块（ma_spec §1.2，未直接使用，保持模块参数一致性） |
| `NLANES` | 8 | lane 数/warp = bank 数（ma_spec §1.2） |
| `U_LINES` | 64 | 统一 SRAM 总行数（ma_spec §1.6） |
| `SM_LINES` | 4 | SM 分区行数（ma_spec §1.6） |
| `NBANKS` | 8 | bank 数 = NLANES（ma_spec §1.6） |
| `ILINE_WORDS` | 8 | 每行字数（32B 行，ma_spec §1.6） |

派生量：`TAG_W = DATA_W − 5`（tag = addr[31:5]，行号全宽比较）；`NSETS = U_LINES − SM_LINES`（L1 组数，**非 2 的幂**，基线 60）；`LW = log2(ILINE_WORDS)`；行组内 bank 索引 = `addr[4:2]`。约束（RTL 几何守卫）：`U_LINES`/`SM_LINES`/`ILINE_WORDS`/`NBANKS` 均须为 2 的幂且 `SM_LINES < U_LINES`；`NSETS` 为一般整数，set 映射取**精确模**（`line mod NSETS`，与 cmodel 一致，不近似为位截取——否则行映射与参考模型分叉、边界事务序列失配）。`MEM_LAT` 不在本模块内（tb 从设备参数，§1.5）。

---

## 4. 内部状态

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `state` | 3 | `S_IDLE` | 状态机，见 §6 |
| `req_rw` / `req_sm` | 1/1 | 0 | 在途请求读写/共享位 |
| `req_addr_q[8]` | 8×32 | 0 | 在途请求逐 lane 字地址（mask 外 lane 为 0） |
| `req_wdata_q[8]` | 8×32 | 0 | 在途请求逐 lane 写数据（mask 外 lane 为 0） |
| `req_mask_q` | 8 | 0 | 活动 lane 掩码 |
| `glane_q` | 4 | 0 | 服务游标（下一待服务 lane） |
| `grp_end_q` | 4 | 0 | 当前行组上界（不含） |
| `wt_lane_q` | 4 | 0 | 写通逐 lane 游标 |
| `rsp_q[8]` | 8×32 | 0 | 响应逐 lane 读数据（存储恒 0） |
| `memif_rw_q` / `memif_addr_q` / `memif_wdata_q` | 1/32/32 | 0 | memif 请求载荷寄存（呈现期稳定） |
| `mreq_done_q` | 1 | 0 | 当前 memif 请求已握手（握手后撤 `vld`，下一笔装载时清零） |
| `data[U_LINES][ILINE_WORDS]` | 64×8×32 | 0 | 数据阵列（复位清零，§8） |
| `cls[U_LINES]` | 64×2 | 行分区见 §8 | 类位阵列 |
| `valid[U_LINES]` | 64 | 行分区见 §8 | 有效位阵列 |
| `tag[U_LINES]` | 64×27 | 行分区见 §8 | tag 阵列 |

组合输出：`l1sm_lsu_req_rdy = (state == S_IDLE)`；`l1sm_lsu_rsp_vld = (state == S_RSP)`；`l1sm_memif_req_vld = ((state == S_REFILL) || (state == S_WT)) && !mreq_done_q`（载荷自寄存值导出；握手完成拍起撤 `vld`，等响应/应答到达后再呈现下一笔）。请求接收拍 `rsp_q` 清零、`glane_q` 清零；载荷在 `vld` 保持期内稳定。

---

## 5. 行为

### 5.1 行组扫描（`S_GRP`）

自 `glane_q` 起按 lane 升序取活动 lane；行组 = 与首 lane 同行（`addr>>5` 相同；SM 侧为 `sm_row = addr>>5`）且 bank（`addr[4:2]`）互不重复的连续活动 lane 集合，`grp_end_q` 为其上界。无剩余活动 lane → `S_RSP`。

### 5.2 SM 访问（行组节拍，恒命中）

`sm_row` 越出 `[0, SM_LINES)` 或钉扎校验失败属协议外（§9 条 3）。读：行组各 lane 并行读 `data[sm_row][bank]` 装入 `rsp_q`；写：行组各 lane 并行写 `data[sm_row][bank]`（同字多写按 lane 升序后写为准——同 bank 必分属不同行组，天然串行保证）。`glane_q` 推进至 `grp_end_q`，转 `S_GRP`（仍有剩余）或 `S_RSP`。SM 无缺失、无写通。

### 5.3 L1 读（行组节拍 + 缺失回填）

行级 tag 比较（整行组同 hit）：命中则行组各 lane 并行读 `data[phys][bank]` 装入 `rsp_q`、推进游标；缺失转 `S_REFILL`（回填完成后回到 `S_GRP` 重放该行组，此时必命中）。缺失期间请求阻塞（单在途）。

### 5.4 回填（`S_REFILL`）

`l1sm_memif_req`：`rw=0`、`addr` = 行组首 lane 地址行对齐（`addr & ~31`）。memif 响应握手拍末沿整行（8 字）单沿写入 `data[phys]`，并置 `cls=L1、valid=1、tag=line`（直接映射覆写分配），转 `S_GRP` 重放。

### 5.5 L1 写（行组节拍 + 逐 lane 写通）

命中：行组各 lane 并行写 `data[phys][bank]`；缺失：不更新行（写直通不写分配，tag 不变）。随后转 `S_WT` 逐 lane 写通：每 lane 一笔 `l1sm_memif_req{rw=1, addr=字地址, wdata}`（4B 窄传），按 lane 升序，每笔等 `memif_l1sm_rsp` 写应答（载荷 0）后才发下一笔；全部完成后 `glane_q` 推进至 `grp_end_q`，转 `S_GRP` 或 `S_RSP`。**写通存储等写应答返回才算完成**（ma_spec §8）。

### 5.6 响应（`S_RSP`）

`l1sm_lsu_rsp` 呈现 `rsp_q`（装载为读数据、存储恒 0），保持至握手，次拍回 `S_IDLE`。响应与请求严格顺序对应（单在途），不回带地址。

---

## 6. 状态机

### 6.1 状态定义

| 状态 | 编码 | 含义 |
|---|---|---|
| `S_IDLE` | 3'd0 | 空闲：`l1sm_lsu_req_rdy=1`，接收请求 |
| `S_GRP` | 3'd1 | 行组服务节拍：扫描行组 + 行级 tag 比较 + bank 并行访问（§5.1–5.3、5.5） |
| `S_REFILL` | 3'd2 | L1 读缺失回填：呈现 memif 读请求，响应握手拍整行写入并重放行组 |
| `S_WT` | 3'd3 | L1 写逐 lane 写通：呈现 memif 写请求，等写应答，lane 升序逐笔 |
| `S_RSP` | 3'd4 | 响应呈现：`l1sm_lsu_rsp_vld=1` 保持至握手 |

### 6.2 状态行为

- `S_IDLE`：`lsu_l1sm_req` 握手拍末沿锁存请求、`rsp_q` 清零、`glane_q=0`，转 `S_GRP`；
- `S_GRP`：按 §5 分派——无剩余活动 lane 转 `S_RSP`；SM 行组服务后推进游标转 `S_GRP`/`S_RSP`；L1 命中服务后推进游标转 `S_GRP`/`S_RSP`（读）或转 `S_WT`（写，`wt_lane_q=glane_q`，游标待写通完成后推进）；L1 读缺失转 `S_REFILL`；
- `S_REFILL`：memif 读请求保持至握手；`memif_l1sm_rsp` 握手拍末沿整行写入 + tag 更新，转 `S_GRP`（游标不动，重放该行组）；
- `S_WT`：当前 lane 的 memif 写请求保持至握手；写应答握手后 `wt_lane_q++`，至 `grp_end_q` 则推进 `glane_q` 并转 `S_GRP`/`S_RSP`，否则留在 `S_WT`；
- `S_RSP`：`l1sm_lsu_rsp` 保持至握手，次拍转 `S_IDLE`。

### 6.3 状态迁移表

| 现态 | 条件 | 次态 | 动作 |
|---|---|---|---|
| `S_IDLE` | 请求握手 | `S_GRP` | 锁存请求、清 `rsp_q`/`glane_q` |
| `S_GRP` | 无剩余活动 lane | `S_RSP` | — |
| `S_GRP` | SM / L1 命中服务完成 | `S_GRP` 或 `S_RSP` | 游标推进至 `grp_end_q` |
| `S_GRP` | L1 写（命中或缺失） | `S_WT` | `wt_lane_q = glane_q`（命中先行组更新） |
| `S_GRP` | L1 读缺失 | `S_REFILL` | 装载回填地址 |
| `S_REFILL` | memif 读请求握手 → 响应握手 | `S_GRP` | 整行单沿写入、tag 更新（重放行组） |
| `S_WT` | 写应答握手且 `wt_lane_q+1 < grp_end_q` | `S_WT` | `wt_lane_q++` |
| `S_WT` | 写应答握手且末 lane | `S_GRP` 或 `S_RSP` | `glane_q = grp_end_q` |
| `S_RSP` | 响应握手 | `S_IDLE` | — |

---

## 7. 通道时序

### 7.1 SM 读（单行无冲突，单服务节拍）

```
拍       T0   T1   T2   T3
state    IDLE GRP  RSP  IDLE
req_vld  1    0    0    0
req_rdy  1    -    -    -
rsp_vld  0    0    1    0
rsp_rdy  -    -    1    -
```

T0 末沿请求握手并锁存；T1 行组节拍（tag 恒等校验 + bank 并行读，`rsp_q` 末沿装载）；T2 呈现响应；T2 末沿响应握手，T3 回空闲。L1 读命中同形（tag 比较替代恒等校验）。

### 7.2 L1 读缺失（回填后重放）

```
拍       T0   T1   T2   T3   ...  TL        TL+1  TL+2
state    IDLE GRP  RFILL RFILL     RFILL     GRP   RSP
mreq_vld      0    1    1    ...   1         0     0
mreq_rdy      -    1    -          -         -     -
mrsp_vld                        ...  1(握手)  0     0
```

T1 判缺失；T2 呈现 memif 读请求（行对齐地址），T2 末沿握手、从设备装载倒计时；TL（= T2 + MEM_LAT + 1）响应到达，TL 末沿整行写入 + tag 更新；TL+1 重放行组（命中读）；TL+2 呈现响应。多行组/多缺失请求逐行组重复此段。

### 7.3 L1 写（写通，单 lane）

T0 请求握手；T1 行组节拍（命中则行内字更新）；T2 起呈现 memif 写请求（字地址 4B），握手后从设备倒计时；写应答（载荷 0）握手后转 `S_RSP` 呈现响应（`rdata` 恒 0）。多 lane 写通按 lane 升序逐笔重复 {呈现 → 握手 → 等应答}，笔数与顺序和参考模型一致（§1.6）。

### 7.4 冲突串行化

跨行/bank 冲突请求：`S_GRP` 逐行组多拍服务（每拍一个行组，lane 升序），末行组完成后转 `S_RSP`；响应仍为单笔（请求粒度）。单行无冲突请求恒单服务节拍（§10 L3 周期级断言）。

---

## 8. 复位与上电行为

- `rst_n = 0`（异步）：`state` 回 `S_IDLE`，全部控制寄存器与 `rsp_q` 按 §4 复位值清零，输出 `vld` 恒 0；
- **复位钉扎**：复位期间完成 SM 区初始化——`[0, SM_LINES)` 每行 `tag := 行索引`、`cls := SM`、`valid := 1`；`[SM_LINES, U_LINES)` 每行 `cls := INV`、`valid := 0`、`tag := 0`；数据阵列全 0（与 cmodel `sim_init` 全零一致）；
- `rst_n` 释放：即处于可服务状态，不依赖任何启动握手；v1 无块启动触发（§1.3 幂等声明）。

---

## 9. 协议约束

1. `vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）；
2. `vld` 不组合依赖于 `rdy`：三条输出通道 `vld` 均为状态译码/寄存器输出；`l1sm_lsu_req_rdy = (state==S_IDLE)` 为纯状态译码，不依赖请求通道 `vld`；
3. 请求源保证合法载荷（协议外行为不作约定）：SM 地址落于 `[0, SM_LINES×32)` 且 4B 对齐（isa_spec §1.10；lsu 已保证对齐检查）；SM 钉扎完整性校验在 cmodel 内为安全网（置 `s->err`），本模块无错误端口；
4. 单请求/响应在途、严格顺序对应、不回带地址（intf_spec §8）：`l1sm_lsu_req_rdy` 仅在 `S_IDLE` 为 1；`l1sm_memif_req` 每笔保持至握手（`mreq_done_q` 置位后撤 `vld`）、等响应/应答后才呈现下一笔，同一时刻至多一笔在途；
5. 写通存储等 memif 写应答返回才算完成（ma_spec §8）；写直通不写分配：存储缺失不更新行、不改 tag；
6. 回填整行单沿写入、以 `class=L1` 覆写分配（直接映射）；L1 分配永不触碰 SM 钉扎行（索引空间偏移保证，§1.3）；
7. `set = line mod NSETS` 精确模（§3），与参考模型行映射一致。

---

## 10. 验证要点

验证以事务级等价为准（§1.7），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| L1 | 请求→响应事务 | 逐笔对应（顺序 + 笔数），装载 `rdata` 位精确、存储 `rdata` 恒 0，与参考模型一致 |
| L2 | memif 事务序列 | 回填（行对齐读）与写通（字地址写）的笔数、顺序、载荷与参考模型逐笔一致 |
| L3 | 单拍条件 | 单行无冲突请求恰一服务节拍（接收握手后第 2 拍呈现响应，含 §7.1 拍序） |
| L4 | 冲突串行化 | 跨行/bank 冲突按行组逐拍服务、lane 升序；同字多写后写 lane 为准 |
| L5 | SM 钉扎 | SM 行恒命中；L1 分配/替换永不触碰 `[0, SM_LINES)`；多次回填后 SM 访问仍正确 |
| L6 | 写直通不写分配 | 存储缺失不改 tag；后续同行访问仍缺失（再回填） |
| L7 | 协议 | `vld` 保持与载荷稳定（§9 条 1）；`rdy` 与在途状态一致；复位期间 `vld` 恒 0 |
| L8 | 背压 | 各通道 `rdy` 任意模式（含长背压、错峰握手）下 L1–L7 成立 |

参考模型：testbench（Verilator C++ harness）直接链接 `top/cmodel`（`l1sm_step`），**事务级比对**（边界事务序列一致且同序；服务时序口径差异见 §1.6，周期不比对）。tb 扮 lsu 侧（请求源 + 响应消费者，随机背压）与 memif 侧从设备（倒计时 = MEM_LAT，与 memif_spec §1.5 同构；两段全局数据区行为级后备，读留行余量）。判据：日志出现 VSIM PASS。
