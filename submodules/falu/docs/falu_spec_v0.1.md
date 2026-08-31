# falu — Floating-point ALU 模块规范

版本：v0.1
日期：2026-08-31
依据文档：`top/docs/ma_spec_v0.1.md`（§1.3、§1.4、§1.7、§6）、`top/docs/intf_spec_v0.1.md`（§1、§2、§6）、`top/docs/isa_spec_v0.1.md`（§1.1、§1.3、§1.8、§9–§11）、`top/cmodel/falu.c`、`top/cmodel/softfloat.c`、`top/cmodel/sim_common.h`。
适用范围：本文档定义 falu（Floating-point ALU）的模块级设计：端口、参数、内部状态、数据通路、状态机、通道时序与协议约束，是 `falu/rtl/falu.sv` 实现与 `falu/tb/` 验证的直接依据。
修订记录：2026-08-31 初版，结构沿用模块级规范骨架（bs_spec/ialu_spec 先例）；§5.4 数据通路窄化：对齐路径改为 50 位定点场 + 对齐 sticky（原 300/301 位精确和差），位精确验收口径不变。

---

## 1. 总体

### 1.1 模块定位与职责

falu 是浮点执行单元，职责为 **FMUL / FADD / FNEG**（ma_spec §6）：

- 按 IEEE-754 binary32、舍入模式固定 roundTiesToEven（RN）执行乘法与加法；取负仅翻转符号位（isa_spec §9–§11）；
- 数据通路 **8 lane 并行、锁步**：一拍对 8 个 lane 同时运算，产出 `wdata[8×32]`（写回数据）。

边界：浮点**比较**（SETP fmt=1 的 `fgt`）在 ialu 实现（ialu_spec §5.3），本模块无比较与谓词端口；整数运算与分支解析归 ialu，LDG/STG/LDS/STS 归 lsu。本模块无分支通道。

### 1.2 指令集范围与完成路径

本模块接受 3 种操作码（sf 分派结果，与 cmodel 分派集合一致），完成路径同一：

| 操作码 | 助记符 | 完成路径 |
|---|---|---|
| 0x08 | FMUL | issue → wb → wbdone |
| 0x09 | FADD | issue → wb → wbdone |
| 0x0A | FNEG | issue → wb → wbdone |

同一时刻至多一条指令在途；完成路径内各通道事务严格顺序发出（§5、§6、§7）。

### 1.3 sf 侧载荷约定

载荷均为 sf 译码期归一化后的形式（与 `top/cmodel/falu.c` 头注一致）：

| 操作码 | 载荷约定 |
|---|---|
| FMUL / FADD | `opa = R[ra]`，`opb = R[rb]` |
| FNEG | `opa = R[ra]`；`opb` 不使用（本模块不读取该字段） |

issue 载荷为 `sf_falu_issue_{opcode,rd,warp_id,lane_mask,opa,opb}`（intf_spec §2），不携带 `pc`/`imm`/`opc` 字段。

### 1.4 数值约定

全部运算与 `top/cmodel/softfloat.c` 位精确一致（该实现与 `assembler/easy_simt_assembler_verify.py` 的 Python 参考逐位一致），要点：

- 唯一浮点格式 IEEE-754 binary32；舍入为**单舍入**（RN，roundTiesToEven），舍入判定经 guard/round/sticky 三位归约，与精确比较等价；
- 非规格化输入先规格化后参与运算；
- 非规格化/下溢结果按 FTZ 冲刷为带符号 ±0；
- 上溢（舍入后指数 ≥ 255）→ ±∞，符号按运算确定；
- NaN 输入 → 静默 NaN，编码固定 `0x7FC00000`；`inf × 0`、`inf + (−inf)` 亦产生该静默 NaN；
- 不产生异常标志（isa_spec §9）；无舍入模式控制（isa_spec §1.1）。

### 1.5 时序模型

- 所有输出为寄存器输出，`clk` 上升沿更新；
- 模块间握手统一遵循 intf_spec §1.2 的 vld/rdy 协议：`vld && rdy` 同时为高的时钟上升沿发生一次传输；`vld` 拉起后源模块保持 `vld` 与全部载荷稳定直至握手完成；`vld` 不组合依赖于 `rdy`；
- 复位为低电平有效异步复位（intf_spec §1.3），复位期间全部 `vld = 0`；
- 单条指令在途（对应 cmodel `has_issue`）：`falu_sf_issue_rdy` 仅在空闲时为 1；
- 运算在 issue 握手当拍完成，结果于该拍末沿进入输出级寄存器；自握手后拍起按 §1.2 的完成路径顺序发出事务。

### 1.6 与 C 模型的对应关系

`top/cmodel/falu.c` 的 `falu_step()` 为事务级参考，其语义与本文档的对应：

| C 模型（`falu_t` / `falu_step`） | falu RTL |
|---|---|
| `has_issue` | `state != S_IDLE` |
| `wb_stage == 1` | `state == S_WB`（`falu_rf_wb_vld`） |
| `wb_stage == 2` | `state == S_WBD`（`falu_sf_wbdone_vld`） |
| `f32_mul` / `f32_add` / `f32_neg`（softfloat.c） | 每 lane 组合数据通路（§5），位精确一致 |
| 未命中 `lane_mask` 的 lane `wdata[l] = 0` | lane 门控（§5.1） |
| 消费者清零 `vld` 表示收走 | 下游 `rdy` 握手 |
| `s->err`（非法 opcode 置错） | 不实现：协议约束 sf 只发合法操作码（§9 条 3），本模块无错误端口 |

偏差说明：

1. C 模型在同一个 `falu_step()` 内可先排空上一条指令的末笔输出、再接受新 issue（同 step 周转）；RTL 状态迁移至少一拍。两者事务序列相同，周期只作观测项（ma_spec §1.7）。

### 1.7 验收口径

功能验收为事务级等价（ma_spec §1.7：周期只作观测项，不作验收项）：

- 三条通道（`sf_falu_issue`、`falu_rf_wb`、`falu_sf_wbdone`）的事务序列逐笔一致，载荷位精确；
- 每条指令的完成路径与发出顺序与 §1.2 一致；
- 握手不变量：`vld` 保持期载荷稳定；复位期间 `vld` 恒 0；`falu_sf_issue_rdy` 与在途状态一致。

---

## 2. 端口

端口命名、方向与位宽见 intf_spec §6（issue 载荷字段见 intf_spec §2；单源规则见 intf_spec §1）。`clk`/`rst_n` 按 intf_spec §1.3 携带。无模块级增设端口。

模块补充（行为约束，详见 §9）：

- `falu_sf_issue_rdy` 仅在 `S_IDLE` 为 1；
- `sf_falu_issue_opb` 在 FNEG 时不被读取（§1.3）。

---

## 3. 参数与配置

| 参数 | 基线值 | 含义 |
|---|---|---|
| `DATA_W` | 32 | 数据/地址/载荷位宽（intf_spec §1.4） |
| `NWARPS` | 4 | warp 数/块（ma_spec §1.2） |
| `NLANES` | 8 | lane 数/warp（ma_spec §1.2） |
| `REG_AW` | 5 | 寄存器地址位宽（intf_spec §1.4） |
| `OPCODE_W` | 5 | 操作码位宽（intf_spec §1.4） |

派生量：`WARP_IW = $clog2(NWARPS)`，`VEC_W = NLANES×DATA_W`。binary32 数值通路位宽由格式定死（指数 8 位、尾数 23 位），不随参数缩放。本模块无配置输入端口。

---

## 4. 内部状态

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `state` | 2 | `S_IDLE` | 状态机，见 §6 |
| `wb_warp_id` | 2 | 0 | wb 载荷 |
| `wb_rd` | 5 | 0 | wb 载荷 |
| `wb_lane_mask` | 8 | 0 | wb 载荷 |
| `wb_wdata` | 256 | 0 | wb 载荷 |
| `wbd_warp_id` | 2 | 0 | wbdone 载荷 |
| `wbd_rd` | 5 | 0 | wbdone 载荷 |

组合输出：`falu_sf_issue_rdy = (state == S_IDLE)`；`falu_rf_wb_vld = (state == S_WB)`；`falu_sf_wbdone_vld = (state == S_WBD)`。两条输出通道的载荷在各自 `vld` 保持期内由寄存值导出，稳定。

---

## 5. 数据通路

运算全部在 issue 握手当拍对 issue 载荷组合完成，结果于该拍末沿随状态迁移进入输出级寄存器（§6）。以下 `l` 均指 lane `l ∈ [0, NLANES)`。

### 5.1 lane 门控

```
out[l] = lane_mask[l] ? res[l] : 0
```

非活动 lane 的 `wdata` 恒为 0（与 cmodel 一致：未命中掩码的 lane 不写入结果）。

### 5.2 操作数解包

对 32 位输入 `x`：`sg = x[31]`，`exp = x[30:23]`，`frac = x[22:0]`。类别判定：

| 类别 | 条件 |
|---|---|
| NaN | `exp = 0xFF` 且 `frac ≠ 0` |
| Inf | `exp = 0xFF` 且 `frac = 0` |
| Zero | `exp = 0` 且 `frac = 0` |
| Denorm | `exp = 0` 且 `frac ≠ 0` |
| Normal | 其余 |

有限非零值统一表示为 `值 = (−1)^sg × mant × 2^q`（与 `softfloat.c` 的 `funpack` 一致）：

- Normal：`mant = {1, frac}`（含隐含位，∈ [2^23, 2^24)），`q = exp − 150`；
- Denorm：规格化处理——设 `p` 为 `frac` 最高置位位（0..22），`mant = frac << (23 − p)`（∈ [2^23, 2^24)），`q = p − 172`；
- Zero/Inf/NaN：`mant`/`q` 无效，按 §5.3–§5.5 的特殊值分支处置。

### 5.3 FMUL

特殊值按以下顺序判定（与 `f32_mul` 一致）：

1. 任一操作数为 NaN：结果 `0x7FC00000`；
2. 任一操作数为 Inf：另一者为 Zero 时结果 `0x7FC00000`（inf × 0）；否则结果 ±∞，符号 `sg = A.sg ⊕ B.sg`；
3. 任一操作数为 Zero：结果带符号零 `{sg, 0}`，符号 `sg = A.sg ⊕ B.sg`。

正常路径（两操作数均有限非零）：

1. 尾数积 `prod = A.mant × B.mant`（48 位，∈ [2^46, 2^48)），指数 `q = A.q + B.q`；
2. 规格化：`prod` 最高位为 bit47 时右移 24，否则（最高位为 bit46）右移 23，得到 24 位尾数域 `keep`（bit23 为 1），`q` 加上相应移位量；
3. 单舍入（RN）：`guard` 为被移出的最高位，`sticky` 为其余被移出位的或，`round_up = guard ∧ (sticky ∨ keep[0])`；`keep` 加 1 后若进位至 2^24，则右移一位、`q` 再加 1；
4. 打包：`e = q + 150`。`e ≥ 255` → ±∞（上溢，符号 `sg`）；`e ≤ 0` → 带符号 ±0（FTZ）；否则结果 `{sg, e[7:0], keep[22:0]}`。

### 5.4 FADD

特殊值按以下顺序判定（与 `f32_add` 一致）：

1. 任一操作数为 NaN：结果 `0x7FC00000`；
2. 任一操作数为 Inf：两者均为 Inf 且符号相异时结果 `0x7FC00000`（inf + (−inf)）；否则结果 ±∞，符号取 Inf 操作数的符号；
3. 两者均为 Zero：结果带符号零，符号 `A.sg ∧ B.sg`（RN 下仅 −0 + −0 = −0）；
4. 仅一者为 Zero：结果为另一操作数的**规格化形式**——Normal 原样通过；Denorm 按 FTZ 冲刷为带符号 ±0（对应 `f32_add` 经 `rne_i64` 的退化路径）。

正常路径（两操作数均有限非零）：

1. 对齐（50 位定点场，标度 `2^(q_max − 25)`，`q_max = max(A.q, B.q)`）：幅值较大者的尾数置于场位 [48:25]；幅值较小者的尾数按**真实指数差** `delta = |A.q − B.q|` 右移，移出场的尾数位全部或进**对齐 sticky** `sticky_al`（`delta ≤ 25` 时无位丢失，`sticky_al = 0`；移位量最大为 `q` 域全幅差 276，场宽不随移位量增长）；
2. 符号相同：`M_sum = L + S`，结果符号取 `A.sg`；符号相异：`M_sum = L − S`（`L` 为大幅值一侧），结果符号取大幅值操作数的符号；`M_sum = 0` 时精确抵消，结果 +0；
3. 对齐借位修正：异号且 `sticky_al = 1` 时，真值为 `M_sum − D`（`0 < D < 1` 场单位），按整数部分 `M = M_sum − 1` 舍入，丢失量并入最终 sticky；其余情形 `M = M_sum`；
4. 单舍入（RN）：设 `lz` 为 `M` 最高置位位。`lz ≥ 23` 时右移 `lz − 23` 位，取 24 位尾数域（bit23 为 1），`guard` 为被移出的最高位，`sticky` 为其余被移出位的或**再或上** `sticky_al`，舍入进位至 2^24 时右移一位、指数加 1；`lz < 23` 时左移 `23 − lz` 位对齐至 bit23，无舍入（深抵消仅发生于 `delta ≤ 1`，此时 `sticky_al = 0`）；`qq = q_max + lz − 48`（含进位加 1）；
5. 打包：同 §5.3 条 4（`e = qq + 150`，`e ≥ 255` → ±∞，`e ≤ 0` → FTZ 带符号 ±0，否则 `{sgn, e[7:0], mant[22:0]}`）。

与 `softfloat.c` 的 320 位定量整数精确求和的位精确一致性由 guard/round/sticky 保证：场内保留位精确参与和差，被移出位仅经 `sticky_al` 进入舍入判定，判定结论与精确值一致。

### 5.5 FNEG

```
res[l] = { ~opa[l][31], opa[l][30:0] }
```

符号位取反，对 ±0、±∞、NaN 同样只翻转符号位；无舍入、无异常（isa_spec §11）。`opb` 不参与。

---

## 6. 状态机

### 6.1 状态定义

| 状态 | 编码 | 含义 |
|---|---|---|
| `S_IDLE` | 2'd0 | 空闲，可接受 issue |
| `S_WB` | 2'd1 | 写回段：`falu_rf_wb_vld` 保持至握手 |
| `S_WBD` | 2'd2 | 写回完成段：`falu_sf_wbdone_vld` 保持至握手 |

复位释放后进入 `S_IDLE`。

### 6.2 状态行为

`S_IDLE`：

1. `falu_sf_issue_rdy = 1`；
2. issue 握手时当拍完成运算（§5），锁存 wb 载荷（`warp_id/rd/lane_mask/wdata`）与 wbdone 载荷（`warp_id/rd`），转 `S_WB`。

`S_WB`：`falu_rf_wb_vld = 1`；握手（`wb_fire`）后转 `S_WBD`。

`S_WBD`：`falu_sf_wbdone_vld = 1`；握手（`wbd_fire`）后转 `S_IDLE`。

### 6.3 状态迁移表

| 现态 | 条件 | 次态 | 动作 |
|---|---|---|---|
| `S_IDLE` | issue 握手 | `S_WB` | 锁存 wb/wbdone 载荷 |
| `S_IDLE` | 其余 | `S_IDLE` | — |
| `S_WB` | `wb_fire` | `S_WBD` | — |
| `S_WB` | 其余 | `S_WB` | — |
| `S_WBD` | `wbd_fire` | `S_IDLE` | — |
| `S_WBD` | 其余 | `S_WBD` | — |

---

## 7. 通道时序

表中各行为该拍内的电平；握手发生在 `vld` 与 `rdy` 同高那拍的时钟末沿，状态与 `vld` 于次拍生效。

### 7.1 全部指令（wb、wbdone 背靠背握手）

```
拍        T0   T1   T2   T3
state     IDLE WB   WBD  IDLE
iss_vld   1    0    0    0
issue_rdy 1    0    0    1
wb_vld    0    1    0    0
wb_rdy    x    1    x    x
wbd_vld   0    0    1    0
wbd_rdy   x    x    1    x
```

T0 末沿 issue 握手（运算当拍完成、结果入级）；T1 起 `wb_vld` 拉起，T1 末沿握手；T2 起 `wbd_vld` 拉起，T2 末沿握手；T3 起回到 `S_IDLE`，可接受下一条。

### 7.2 背压保持

`wb_rdy` 拉低 2 拍示例：

```
拍        T0   T1   T2   T3   T4   T5
state     IDLE WB   WB   WB   WBD  IDLE
wb_vld    0    1    1    1    0    0
wb_rdy    x    0    0    1    x    x
wbd_vld   0    0    0    0    1    0
```

`vld` 保持期（T1–T3）载荷稳定（§9 条 1）；握手于 T3 末沿发生，T4 起进入 `S_WBD`。两条输出通道的背压相互独立。

---

## 8. 复位与上电行为

- `rst_n = 0`（异步）：`state` 回 `S_IDLE`，全部输出级寄存器按 §4 复位值清零，两条输出通道 `vld = 0`；
- `rst_n` 释放：进入 `S_IDLE`，`falu_sf_issue_rdy = 1`，等待 sf 发射，不依赖任何启动握手；
- 本模块无跨指令持久状态（无谓词、无上下文），复位后即为可用的初始状态。

---

## 9. 协议约束

1. `vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）；
2. `vld` 不组合依赖于 `rdy`：两条输出通道 `vld` 均由状态寄存器译码，满足该条；
3. sf 只发射合法操作码集合 {FMUL, FADD, FNEG}（sf 分派结果，与 cmodel 分派集合一致）；协议外操作码的行为不作约定（cmodel 置错误标志，本模块无错误端口）；
4. 单条指令在途：sf 不在上一条指令完成前向本模块发射新指令（cmodel `has_issue` 口径）；`falu_sf_issue_rdy` 仅在 `S_IDLE` 为 1；
5. `rd = 0` 时本模块照常发出 wb/wbdone：R0 写忽略由 rf 执行（intf_spec §11），记分板清除由 sf 按 `rd != 0` 执行；
6. `lane_mask` 为发射时 active mask 快照，随路至写回（intf_spec §6 说明）；本模块不修改该快照。

---

## 10. 验证要点

验证以事务级等价为准（§1.7），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| F1 | issue 握手 | DUT 接受当且仅当参考模型消费，逐拍一致 |
| F2 | 完成路径与顺序 | 全部指令 wb 先于 wbdone；全局事务序列与参考逐笔一致 |
| F3 | 载荷位精确 | wb：`warp_id/rd/lane_mask/wdata`（含非活动 lane 为 0）；wbdone：`warp_id/rd` |
| F4 | FMUL 数值类别 | NaN/±∞/±0/非规格化/规格化数两两组合：inf×0、上溢→±∞、下溢/非规格化结果→FTZ ±0、非规格化输入先规格化、舍入临界（tie 到偶、tie±1） |
| F5 | FADD 数值类别 | 同类别组合外加：inf+(−inf)、异号精确抵消→+0、x+0（x 为规格化/非规格化/±0）、符号位规则（−0+−0=−0）、大指数差对齐 |
| F6 | FNEG | ±0、±∞、NaN（含多种编码）、非规格化、规格化数符号位翻转，其余位不变 |
| F7 | 协议 | `vld && !rdy` 期间载荷不变、`vld` 不撤（§9 条 1）；复位期间 `vld` 恒 0 |
| F8 | `falu_sf_issue_rdy` 与背压 | 无在途为 1、有在途为 0；两条输出通道消费者 `rdy` 任意组合（恒 1、随机、周期图案，含长背压与背靠背零延迟）下 F1–F7 成立 |
| F9 | 无死锁 | 全部事务在限界拍数内排空 |

参考模型：testbench（Verilator C++ harness）直接链接 `top/cmodel`（`falu_step`），参考事务序列与 DUT 事务序列在线比对。
