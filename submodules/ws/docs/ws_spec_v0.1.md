# ws — Warp Scheduler（SIMT 前端与调度合一）模块规范

版本：v0.1
日期：2026-09-15
依据文档：`top/docs/ma_spec_v0.1.md`（§1.2、§1.3、§1.4、§1.6、§2）、`top/docs/intf_spec_v0.1.md`（§1、§2）、`top/docs/isa_spec_v0.1.md`（§1.8 指令编码、§1.10 对齐约束、§19 BRT）、`top/cmodel/ws.c`、`top/cmodel/top.c`、`top/cmodel/sim_common.h`。
适用范围：本文档定义 ws（Warp Scheduler，SIMT 前端与 warp 调度合一）的模块级设计：端口、参数、内部状态、每 warp 指令流状态机、调度策略、分化控制、通道时序与协议约束，是 `ws/rtl/ws.sv` 实现与 `ws/tb/` 验证的直接依据。
修订记录：（首版）

---

## 1. 总体

### 1.1 模块定位与职责

ws 是 SIMT 前端与 warp 调度合一的控制前端（ma_spec §1.4）：每 warp 一份 PC、active mask（8b）、分化栈（深 4，表项 `{mask, 重聚PC}`）；取指发起、定长译码、立即数扩展、`ld.param`/`ld.special` 值构造；冒险检测与互锁（记分板，互锁不转发）；issue 分派（ialu/falu/lsu 三路）；SIMT 分化控制（判定在 ialu，处置在本模块）；RUN_TO_DONE 单 warp 独占调度；汇聚停顿源（lsu）；`bar.sync` 到达计数与凑齐统一释放；块完成判定（全 warp DONE 后发 `block_done`）。

边界：不碰块级决策（bs 职责）；不执行运算（ialu/falu 职责）；不生成访存地址（lsu 职责）；不含指令缓存（icache 职责，取指请求与指令回送经通道交互）。

### 1.2 执行约定（与 ma_spec §1.3 一致）

- **单 warp 独占、阻塞至完成**（`WS_POLICY = RUN_TO_DONE`）：任一时刻至多一个 warp 在途（取指/译码/读口/发射在途均属"在途"）；其停顿由整机等待，不切换 warp；仅 `ret`（回合结束置 DONE）与 `bar.sync` 到达（置 BARRIER）结束回合；
- **互锁不转发**：数据冒险一律停顿，直到写回完成（记分板清除）后重新发射；
- **无分支预测、无冲刷**：遇 BR 阻塞取指，等 ialu 决议（taken 向量 → 本模块定出下一 PC）再取下一条；JOIN 由本模块本地处置（分化栈，偏差 C4）；BAR/RET 不下发执行单元，由本模块直接处置；
- **单在途取指、单在途 rf 读**：同一时刻至多一个取指请求、一笔 rf 读在途（cmodel `fetch_warp`/`rd_warp` 口径）。

### 1.3 每 warp 指令流（与 cmodel 状态机逐拍对应）

```
调度选中（IDLE/BAR，RUN_TO_DONE） → 取指（FETCH，IMISS）
  → 译码/冒险（HAZ，记分板互锁） → 读 rf（RD1/RD2，两阶段按需）
  → 发射（EXEC / BR：BRSTALL） → wbdone/分支决议 → 回到可调度态
RET → DONE；BAR（LSU 排空前 HAZ 等待）→ BAR，凑齐释放回 IDLE
```

### 1.4 时序模型

- 输出通道 `vld` 为寄存器输出或状态译码，`vld && rdy` 同拍为高的时钟末沿完成握手（intf_spec §1.2）；
- 译码、冒险判定、读口规划在取指响应消费拍 / 重试拍组合完成，非阻塞跳转（RET/BAR/JOIN）与读口发起、发射装载于对应拍末沿生效；
- 分支决议消费拍组合计算新 PC 与 mask 更新，末沿写入 warp 上下文；
- 复位为低电平有效异步复位（intf_spec §1.3），复位期间全部输出 `vld = 0`。

### 1.5 与 C 模型的对应关系

`top/cmodel/ws.c` 的 `ws_step()` 为事务级参考，RTL 逐拍镜像（同 lsu/memif/icache 先例：cmodel 每步一事件，RTL 每事件一拍，必要时带检测拍气泡）。关键对应：

| C 模型（`ws_t`/`ws_step`） | ws RTL |
|---|---|
| `cur_warp`（RUN_TO_DONE 独占指针） | 寄存器 `cur_warp_q` |
| 授予：`cur_warp` 可发射即置 `WS_FETCH`/`fetch_pend` | 调度拍直接装载取指（单在途） |
| `ptr`（轮转，已废弃） | 无（RUN_TO_DONE） |
| `fetch_warp`/`fetch_pend`（单在途取指） | `fetch_warp_q`/`fetch_pend_q` |
| `rd_warp`（单在途 rf 读） | `rd_warp_q` |
| `sb_busy[NWARPS]`（记分板 32 位/态） | `sb_busy_q[NWARPS]` |
| `lsu_out[NWARPS]`（未退休访存计数，BAR 可见性） | `lsu_out_q[NWARPS]` |
| `des_reason`（停顿原因，调度内部读取） | 逐 warp 状态译码（不设独立寄存器） |
| `bar_pending[w]`（LSU 未排空等待） | 逐 warp 状态编码（BAR 等待子态） |
| `launched`/`active`（块占用双标志） | `launched_q`/`active_q` |
| `bdone_sent`（block_done 在途） | `bdone_sent_q` |
| RET：置 DONE + `sched_advance_done` | 同构（拍末沿） |
| BAR：到达计数 + `sched_advance_barrier`；凑齐释放回最小 id | 同构 |
| 接收 launch：复位全部 warp 上下文与调度状态 | `S_LAUNCH` 等价初始化（§5） |
| `ws_err`（非法指令/分化栈溢出/架构错误） | `ws_top_err` 输出（锁存） |

### 1.6 验收口径

功能验收为事务级等价（ma_spec §1.7）：全通道事务序列（取指请求/指令回送、rf 读/读应答、三路 issue、分支决议、三路 wbdone、lsu 停顿、launch/bdone）逐笔一致、载荷位精确；周期只作观测项（本模块与 cmodel 逐拍镜像，逐拍锁步比对成立，见 §10）。

---

## 2. 端口

端口命名、方向与位宽见 intf_spec §2（八组通道 + `ws_top_err`）。`clk`/`rst_n` 按 intf_spec §1.3 携带。模块补充如下：

- `ws_bs_launch_rdy`：空闲（非 `launched` 且非 `active`）时为 1；
- `ws_lsu_stall_rdy` 恒 1：lsu 停顿上报为提示性状态同步（正确性由调度独占保证）；
- `ws_ialu_br_rdy`/`ws_{ialu,falu,lsu}_wbdone_rdy`：本模块非阻塞决议消费，恒 1（决议在途至多一笔，cmodel 同拍消费口径）；
- `ws_top_err`（输出，1 位）：非法指令译码、分化栈溢出、`bar.sync` 掩码非全 1、RET 时分化栈非空、LDG/STG 基址非均匀（偏差 C2）——锁存至复位。

---

## 3. 参数与配置

| 参数 | 基线值 | 含义 |
|---|---|---|
| `DATA_W` | 32 | 数据/地址/载荷位宽（intf_spec §1.4） |
| `NWARPS` | 4 | warp 数/块（ma_spec §1.2） |
| `NLANES` | 8 | lane 数/warp（ma_spec §1.2） |
| `REG_AW` | 5 | 寄存器地址位宽（intf_spec §1.4） |
| `OPCODE_W` | 5 | 操作码位宽（intf_spec §1.4） |
| `BRT_IW` | 2 | BRT 表项索引位宽（intf_spec §1.4） |

派生量：`DIV_STACK_DEPTH = 4`（分化栈深，ma_spec §1.6）；`WARP_IW = log2(NWARPS)`；`VEC_W = NLANES×DATA_W`；BRT 表 `NLANES` 无关，容量 `BRT_ENTRIES = 4`、每项 32 位（重聚 PC），只读，由汇编器随程序提供（黄金程序 3 项 {9→13, 21→46, 46→49}）。BRT 载入方式为模块补充：单元验证由 tb 经激励隐式覆盖（分支目标与重聚点匹配时无栈操作），顶层集成时经程序镜像装载（预留，非本模块端口）。

---

## 4. 内部状态

### 4.1 块级

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `launched_q` | 1 | 0 | 本块已收 launch（调度侧占用） |
| `active_q` | 1 | 0 | 前端侧占用（全 warp DONE 释放） |
| `block_idx_q` | 32 | 0 | 块索引（CSRR k=2 上下文） |
| `n_q` | 32 | 0 | 每块线程总数（ld.param k=2 值来源之一，载荷保存） |
| `shbase_q` | 32 | 0 | 共享内存基址（随 issue 包下发） |
| `bar_arr_q[NWARPS]` | 1×4 | 0 | 到达屏障 |
| `bar_cnt_q` | 3 | 0 | 到达计数 |
| `cur_warp_q` | 2 | 0 | RUN_TO_DONE 独占指针 |
| `done_q[NWARPS]` | 1×4 | 0 | warp 已 ret |
| `bdone_sent_q` | 1 | 0 | block_done 已置位待消费 |
| `err_q` | 1 | 0 | `ws_top_err` 锁存 |

### 4.2 每 warp（×NWARPS）

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `wstate_q` | 4 | `W_IDLE` | 每 warp 指令流状态机（§5） |
| `pc_q` | 32 | 0 | 程序计数器（字索引） |
| `mask_q` | 8 | 0 | active mask |
| `rpc_q[4]` | 32×4 | 0 | 分化栈：重聚 PC |
| `dmask_q[4]` | 8×4 | 0 | 分化栈：掩码 |
| `dsp_q` | 3 | 0 | 分化栈指针 |
| `inst_q` | 32 | 0 | 已取回指令 |
| `rd_phase_q` | 3 | 0 | 读口阶段（0 未始 / 1 第一阶段 / 2 第二阶段 / 6/7 重发编码） |
| `br_pc_q` | 32 | 0 | 在途 BR 的 pc（BRT 查表用） |
| `rd_a_q`/`rd_b_q`/`rd_c_q` | 8×32 | 0 | 读口数据暂存（a/b 第一阶段、c 第二阶段） |

### 4.3 全局在途与记分板

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `fetch_warp_q` | 2+1 | -1（编码 4） | 在途取指 warp（-1 无） |
| `fetch_pend_q` | 1 | 0 | 取指请求待置位 |
| `rd_warp_q` | 2+1 | -1 | 在途 rf 读 warp |
| `sb_busy_q[NWARPS]` | 32×4 | 0 | 记分板：每 warp 32 寄存器忙位 |
| `lsu_out_q[NWARPS]` | 3×4 | 0 | 未退休访存计数（BAR 可见性） |
| `lsu_reason_q[NWARPS]` | 3×4 | 0 | lsu 停顿锁存（LMISS/R_NONE） |

---

## 5. 每 warp 状态机

### 5.1 状态定义（镜像 cmodel WS_*，ws_spec 编码独立）

| 状态 | 编码 | 含义 |
|---|---|---|
| `W_IDLE` | 4'd0 | 可调度（等待调度选中） |
| `W_FETCH` | 4'd1 | 取指请求呈现（`fetch_pend`） |
| `W_HAZ` | 4'd2 | 阻塞重试：记分板互锁 / 读口忙 / 屏障等 LSU 排空 |
| `W_RD1` | 4'd3 | 第一阶段 rf 读在途 |
| `W_RD2` | 4'd4 | 第二阶段 rf 读在途 |
| `W_ISSUE` | 4'd5 | 发射目标忙（issue 通道 vld 被占/执行单元在途），重试 |
| `W_EXEC` | 4'd6 | 已发射等 wbdone（记分板待清） |
| `W_BR` | 4'd7 | BR 已发射等 ialu 决议 |
| `W_BAR` | 4'd8 | 已到达屏障，等凑齐释放 |
| `W_DONE` | 4'd9 | 已 ret |

### 5.2 状态行为

- `W_IDLE`/`W_BAR`：调度选中（RUN_TO_DONE，§6）拍末沿转 `W_FETCH`、装载 `fetch_warp_q`/`fetch_pend_q`；
- `W_FETCH`：`fetch_pend_q=1` 且取指请求通道空闲时呈现 `ws_icache_req{pc}`，握手拍末沿清 `fetch_pend_q`；指令回送（`icache_ws_rsp`）消费拍末沿锁存 `inst_q`、清 `fetch_warp_q`，随即进入译码（同拍组合判定，§5.3）；
- `W_HAZ`：按阻塞原因重试——屏障等待（LSU 排空即到达）、读口重发（`rd_phase` 重发编码）、记分板互锁（清除后重进读/发射）；
- `W_RD1`/`W_RD2`：读应答消费拍末沿锁存数据并推进（需第二阶段则续发 `ws_rf_rd`，否则进发射判定）；
- `W_ISSUE`：目标执行单元空闲且 issue 通道空闲时发射，末沿转 `W_EXEC`（BR 转 `W_BR`）；
- `W_EXEC`：对应 `wbdone` 消费拍末沿回 `W_IDLE`（同拍清记分板位）；
- `W_BR`：`ialu_ws_br` 消费拍末沿按决议写 PC/mask/分化栈，回 `W_IDLE`（均匀）或保持（分化路径由 JOIN 收敛）；
- `W_BAR`：凑齐释放拍回 `W_IDLE`（四 warp 同拍）；
- `W_DONE`：终态（块释放判定用）。

### 5.3 译码与非阻塞处置（取指响应消费拍组合判定）

- 非法操作码 → `ws_top_err`；
- **RET**：分化栈须空，否则 `ws_top_err`；PC+1、转 `W_DONE`、置 `done_q[w]`、`sched_advance_done`；
- **BAR**：掩码须全 1，否则 `ws_top_err`；PC+1；LSU 已排空（`lsu_out_q[w]==0`）即到达（计数 + `sched_advance_barrier`、转 `W_BAR`），未排空转 `W_HAZ`（屏障等待子态）；
- **JOIN**：本地处置——栈顶重聚 PC 等于 JOIN 目标则弹栈恢复 mask；栈非空但不匹配则跳栈顶；栈空则跳目标；回 `W_IDLE`；
- **BR**：锁存 `br_pc_q`、转 `W_ISSUE` 走 ialu 发射（无读口）；
- 其余（含 SETP/LDP/CSRR 直通类）：记分板检测（源或目的命中 `sb_busy_q[w]` 即转 `W_HAZ`），通过则按 `read_plan` 发起 rf 读（0/1/2 阶段）或直接进发射判定。

### 5.4 发射分派（W_ISSUE 重试拍 / 读口完成拍）

- **ialu 目的**（IMAD/IADD/SHL/XOR/ORI/LUI/SETP/LDP/CSRR）：`ws_ialu_issue` 空闲且 ialu 无在途（顶层独占保证，通道忙为结构兜底）时发射；载荷按 isa_spec 归一化（立即数扩展、LDP 广播参数值、CSRR 构造 lane id/NWARP×NLANES/blockIdx）；
- **falu 目的**（FMUL/FADD/FNEG）：`ws_falu_issue` 同上；
- **lsu 目的**（LDG/STG/LDS/STS）：`ws_lsu_issue` 同上（含 `shbase_q` 随路；LDG/STG 基址均匀性校验，违反置 `ws_top_err`，偏差 C2）；
- 发射成功：记分板置目的位（写寄存器指令且 rd≠0）、`lsu_out_q[w]++`（访存类）、PC+1、转 `W_EXEC`（BR 转 `W_BR`）。

### 5.5 分支决议处置（`ialu_ws_br` 消费拍）

- taken=0 或 not-taken=0：**均匀分支**——PC 取目标（全 taken）或 br_pc+1（全不跳），不压栈；
- 分化：压 `{当前 mask, 重聚 PC}` 入栈（单侧型压 1 项：跳过侧顺序流；双侧型压 2 项：先走 taken），mask 置 taken 子集，PC 置目标；栈溢出置 `ws_top_err`；
- 重聚 PC 来源：BRT 查表（分支 pc 为索引）；黄金程序 3 项。

---

## 6. 调度（RUN_TO_DONE，ma_spec §2）

- 块启动后自 warp 0 开始（`cur_warp_q = 0`）；
- 授予条件：`cur_warp_q` 可发射（未 DONE、未 BARRIER、无停顿原因：无 lsu LMISS 锁存、状态为 `W_IDLE`/`W_BAR`）且无在途取指（单在途约束）；
- 回合结束点：RET（`sched_advance_done`：id 序环扫下一非 DONE）、bar.sync 到达（`sched_advance_barrier`：下一非 DONE 且非 BARRIER——屏障等待条件只能由后续 warp 到达满足，切换为被迫而非主动）；
- 其余停顿（HAZARD/IMISS/LMISS/BRSTALL）不切换：整机阻塞等待当前 warp 恢复；
- 屏障凑齐：四者 BARRIER 同拍清除、计数复位，`cur_warp_q` 置最小 id 非 DONE warp；
- 块完成：全 warp DONE → 呈现 `ws_bs_bdone{block_idx}`，消费后清 `launched_q`；全 warp `W_DONE` 同拍释放 `active_q`（供下一块 launch）。

---

## 7. 通道时序

### 7.1 调度选中 → 取指（无背压链）

```
拍        T0   T1   T2   T3   T4
wstate    IDLE FETCH FETCH RD1/ EXEC ...
                    (握手)  译码
ireq_vld       0    1    0
ireq_rdy       -    1    -
```

T0 调度选中；T1 呈现取指请求（pc 组合自 `pc_q`）；T2 假设命中（icache 首拍即回送或 MEM_LAT 后）指令回送被消费，末沿锁存 `inst_q` 并组合译码（冒险判定通过则发起 rf 读）；后续按 §5.2 推进。

### 7.2 记分板互锁与重试

冒险命中拍转 `W_HAZ`；`wbdone` 消费拍清 `sb_busy_q` 位，同拍或次拍 `W_HAZ` 重试（译码组合重判定）通过则进读/发射。互锁期间不切换 warp（RUN_TO_DONE）。

### 7.3 屏障

warp A 到达（LSU 已排空）：末沿置 `bar_arr_q[A]`、计数 +1、转 `W_BAR`、调度推进；末 warp 到达拍凑齐：四者同拍回 `W_IDLE`、计数清零、`cur_warp_q` 回最小 id。LSU 未排空的 warp 在 `W_HAZ`（屏障等待子态）轮询 `lsu_out_q[w]`，排空拍即到达。

### 7.4 分支阻塞

BR 发射后 `W_BR` 阻塞取指（无预测）；`ialu_ws_br` 呈现即消费（rdy 恒 1），消费拍末沿写新 PC/mask/分化栈，次拍可调度。

---

## 8. 复位与上电行为

- `rst_n = 0`（异步）：全部寄存器按 §4 复位值清零，输出 `vld` 恒 0；
- `rst_n` 释放：处于空闲（未 launched），等待 `bs_ws_launch`；
- launch 接收拍末沿：初始化全部 warp 上下文（PC=0、mask=全 1、栈深 0、`W_IDLE`）、调度状态（`cur_warp_q=0`、计数清零、DONE 清零）、记分板/`lsu_out` 清零，置 `launched_q`/`active_q`。

---

## 9. 协议约束

1. `vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）；
2. `vld` 不组合依赖于 `rdy`（本模块输出通道均为寄存器/状态译码）；
3. 请求源与响应方约定：icache 指令回送严格对应单在途取指；rf 读应答严格顺序对应单在途读（不回带地址）；ialu 决议与三路 wbdone 的 `warp_id` 均指向本模块当前在途指令（顶层独占保证单笔在途）；
4. 停顿汇聚：warp 可发射 ⟺ 无任何停顿源；`lsu_ws_stall` 为提示性状态同步（LMISS 随发射握手当拍、R_NONE 于空闲拍），`ws_lsu_stall_rdy` 恒 1；
5. 分支重定向一律等 ialu 决议完成后生效；决议前本模块阻塞取指，无冲刷（ma_spec §1.3）；
6. `bar.sync` 不下发执行单元；屏障可见性由 `lsu_out_q` 兜底（到达前该 warp 在 lsu 无未退休访存）；
7. BRT 只读；分化栈深 4，溢出置 `ws_top_err`；
8. 本模块错误条件（锁存 `ws_top_err`）：非法操作码、分化栈溢出、BAR 掩码非全 1、RET 栈非空、LDG/STG 基址非均匀（C2）。

---

## 10. 验证要点

验证以事务级等价为准（§1.6），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| W1 | 全通道事务序列 | 八组通道事务与参考模型逐笔一致（顺序 + 载荷位精确） |
| W2 | RUN_TO_DONE | 在途至多一个 warp；仅 RET/BAR 到达切换；HAZARD/IMISS/BRSTALL 期间无其他 warp 取指 |
| W3 | 记分板互锁 | 冒险停顿至 wbdone 清除；不提前发射 |
| W4 | 分化/重聚 | 均匀分支不压栈；分化压栈走 taken；JOIN 弹栈恢复 mask；栈深边界 |
| W5 | 屏障 | 凑齐统一释放；释放后最小 id 恢复；LSU 未排空延迟到达 |
| W6 | 块完成 | 全 DONE 发 bdone；bdone 消费后可收下一 launch |
| W7 | 错误注入 | 非法指令/BAR 掩码/RET 栈非空置 `ws_top_err` 锁存 |
| W8 | 协议 | `vld` 保持与载荷稳定；复位期间 `vld` 恒 0；背压任意模式下 W1–W7 成立 |

参考模型：testbench（Verilator C++ harness）直接链接 `top/cmodel`（`ws_step`），**逐拍锁步比对**（本模块与 cmodel 同为逐事件推进，与 lsu/memif/icache 先例一致）：tb 扮 icache（指令存储 + 命中/缺失延迟可控 + 回送）、rf（读应答随机延迟）、ialu（SETP/BR 决议：谓词求值 + 分支目标/BRT 回注）、falu/lsu（issue 消费 + wbdone 随机延迟）、bs（launch 序列）与 lsu 停顿注入。判据：日志出现 VSIM PASS。
