# ialu — Integer ALU 模块规范

版本：v0.1
日期：2026-08-30
依据文档：`top/docs/ma_spec_v0.1.md`（§1.3、§1.4、§1.7、§5）、`top/docs/intf_spec_v0.1.md`（§1、§2、§5）、`top/docs/isa_spec_v0.1.md`（§1.3、§1.7、§1.8、§2–§8、§16–§18）、`top/cmodel/ialu.c`、`top/cmodel/softfloat.c`（`f32_gt`）、`top/cmodel/sim_common.h`（偏差 C3、C5）。
适用范围：本文档定义 ialu（Integer ALU）的模块级设计：端口、参数、内部状态、数据通路、状态机、通道时序与协议约束，是 `ialu/rtl/ialu.sv` 实现与 `ialu/tb/` 验证的直接依据。

---

## 1. 总体

### 1.1 模块定位与职责

ialu 是整数执行单元，职责为**整数运算 + 分支解析**（ma_spec §5）：

- 数据通路 **8 lane 并行、锁步**：一拍对 8 个 lane 同时运算，产出 `wdata[8×32]`（写回数据）与 `taken[7:0]`（每 lane 分支判定）；
- 谓词寄存器 P0..P3 物理上驻留本模块（ma_spec §5）：SETP 写、BR 读，直接喂分支判定；
- 分支解析：算出每 lane taken 向量、跳转目标、BRT 表项索引，经 `ialu_sf_br` 回注 sf。

边界：分支的**判定**在本模块，**处置**（压栈、换 mask、重定向）在 sf（ma_spec §1.4）；JOIN 由 sf 本地处置、不经本模块（sim_common.h 偏差 C4）；BAR/RET 不下发执行单元（ma_spec §2）；FMUL/FADD/FNEG 归 falu，LDG/STG/LDS/STS 归 lsu。

### 1.2 指令集范围与完成路径

本模块接受 10 种操作码（sf 分派结果，与 cmodel 分派集合一致），按完成路径分三类：

| 操作码 | 助记符 | 完成路径 |
|---|---|---|
| 0x01 | IMAD | issue → wb → wbdone |
| 0x02 | IADD | issue → wb → wbdone |
| 0x03 | SHL | issue → wb → wbdone |
| 0x04 | XOR | issue → wb → wbdone |
| 0x05 | ORI | issue → wb → wbdone |
| 0x06 | LUI | issue → wb → wbdone |
| 0x0F | LDP | issue → wb → wbdone |
| 0x10 | CSRR | issue → wb → wbdone |
| 0x07 | SETP | issue → wbdone（仅谓词写，无寄存器写回） |
| 0x11 | BR | issue → br（无写回） |

同一时刻至多一条指令在途；完成路径内各通道事务严格顺序发出（§5、§6）。

### 1.3 sf 侧载荷约定

载荷均为 sf 译码期归一化后的形式（与 `top/cmodel/ialu.c` 头注一致）：

| 操作码 | 载荷约定 |
|---|---|
| IADD/SHL/XOR | `opa = R[ra]`；`opb` = 第二源（寄存器值或立即数广播，由 sf 选定） |
| ORI | `opa = R[ra]`；`opb` = 零扩展立即数广播 |
| LUI | `opa` = `imm20 << 12` 广播，直通产出（偏差 C3） |
| LDP/CSRR | `opa` = 参数/特殊寄存器值广播（CSRR 的 TID 为逐 lane 值），直通产出（偏差 C3） |
| IMAD | `opa = R[ra]`，`opb = R[rb]`，`opc = R[rc]`（第三源，偏差 C5，intf_spec §2 已补勘误） |
| SETP | `rd = pd`；`imm = (fmt<<3) | cond`；`opa = R[ra]`，`opb = R[rb]` |
| BR | `rd = psel`；`imm = (u<<31) | (neg<<30) | target`，`target` 为目标字地址（sf 译码已算出绝对目标） |

`pc` 字段：本模块接收但不使用——BRT 查表以 sf 自行记录的分支 pc 进行，`ialu_sf_br_brt_idx` 恒置 0（与 cmodel `brt_idx = 0`、sf 按分支 pc 查 BRT 的口径一致）。

### 1.4 谓词寄存器

规模 `NWARPS × 4 × NLANES` 位（每 warp 4 个谓词、各 8 lane 位图）：

- 写：SETP 在 issue 握手拍按 `lane_mask` 门控写入比较结果，非活动 lane 位不变（isa_spec §8）；
- 读：BR 在 issue 握手拍读取 `pred[warp][psel]` 参与 taken 计算；
- 上电复位全清零，对应 isa_spec §1.3 初始状态（谓词全 0）。

块切换时的谓词清零（isa_spec §1.3）**不在本模块实现**：ialu 无块启动端口，该副作用归属顶层复位分发，与 bs_spec §1.5 对 `sim_block_start()`（谓词清零、SM 行钉扎）的处置一致。黄金程序中块内对每个谓词寄存器的首次读均先有对该寄存器的 SETP，块初值不可观测；若顶层无法提供块级清零，程序约定为"块内读某谓词寄存器前必先有对它的 SETP"。

### 1.5 时序模型

- 所有输出为寄存器输出，`clk` 上升沿更新；
- 模块间握手统一遵循 intf_spec §1.2 的 vld/rdy 协议：`vld && rdy` 同时为高的时钟上升沿发生一次传输；`vld` 拉起后源模块保持 `vld` 与全部载荷稳定直至握手完成；`vld` 不组合依赖于 `rdy`；
- 复位为低电平有效异步复位（intf_spec §1.3），复位期间全部 `vld = 0`；
- 单条指令在途（对应 cmodel `has_issue`）：`ialu_sf_issue_rdy` 仅在空闲时为 1；
- 运算在 issue 握手当拍完成，结果于该拍末沿进入输出级寄存器；自握手后拍起按 §1.2 的完成路径顺序发出事务。

### 1.6 与 C 模型的对应关系

`top/cmodel/ialu.c` 的 `ialu_step()` 为事务级参考，其语义与本文档的对应：

| C 模型（`ialu_t` / `ialu_step`） | ialu RTL |
|---|---|
| `has_issue` | `state != S_IDLE` |
| `wb_stage == 1` | `state == S_WB`（`ialu_rf_wb_vld`） |
| `wb_stage == 2` | `state == S_WBD`（`ialu_sf_wbdone_vld`） |
| `br_stage == 1` | `state == S_BR`（`ialu_sf_br_vld`） |
| 排空顺序 br → wb → wbdone 的固定检查序 | 各指令类型完成路径互斥，单路径内顺序发出 |
| `pred[w][pd]`（每 warp 8 lane 位图） | 谓词寄存器阵列 `pred` |
| 消费者清零 `vld` 表示收走 | 下游 `rdy` 握手 |
| 消费侧（sf/rf）有 `vld` 即收走 | testbench 中消费者 `rdy` 可任意背压，协议内行为一致 |
| `s->err`（非法 opcode 置错） | 不实现：协议约束 sf 只发合法操作码（§9 条 3），本模块无错误端口 |

偏差说明：

1. C 模型在同一个 `ialu_step()` 内可先排空上一条指令的末笔输出、再接受新 issue（同 step 周转）；RTL 状态迁移至少一拍。两者事务序列相同，周期只作观测项（ma_spec §1.7）。
2. fmt=1 比较的 eq/ne 按位比较（与 cmodel 口径一致），见 §5.3。

### 1.7 验收口径

功能验收为事务级等价（ma_spec §1.7：周期只作观测项，不作验收项）：

- 四条通道（`sf_ialu_issue`、`ialu_sf_br`、`ialu_rf_wb`、`ialu_sf_wbdone`）的事务序列逐笔一致，载荷位精确；
- 每条指令的完成路径与发出顺序与 §1.2 一致；
- 握手不变量：`vld` 保持期载荷稳定；复位期间 `vld` 恒 0；`ialu_sf_issue_rdy` 与在途状态一致。

---

## 2. 端口

端口命名与位宽与 intf_spec §5 一致（issue 载荷含 `opc`，见 intf_spec 勘误与 §1.3）；`clk`/`rst_n` 按 intf_spec §1.3 携带。

| 端口 | I/O | 位宽 | 所属通道 | 说明 |
|---|---|---|---|---|
| `clk` | 输入 | 1 | — | 时钟，上升沿有效 |
| `rst_n` | 输入 | 1 | — | 异步复位，低有效 |
| `sf_ialu_issue_vld` | 输入 | 1 | sf_ialu_issue | sf 发射 |
| `sf_ialu_issue_opcode` | 输入 | 5 | sf_ialu_issue | 操作码（`OPCODE_W`） |
| `sf_ialu_issue_rd` | 输入 | 5 | sf_ialu_issue | 目的寄存器；SETP 时承载 `pd`，BR 时承载 `psel` |
| `sf_ialu_issue_warp_id` | 输入 | 2 | sf_ialu_issue | warp 号 |
| `sf_ialu_issue_lane_mask` | 输入 | 8 | sf_ialu_issue | 发射时 active mask 快照 |
| `sf_ialu_issue_pc` | 输入 | 32 | sf_ialu_issue | 接收但内部不使用（§1.3） |
| `sf_ialu_issue_imm` | 输入 | 32 | sf_ialu_issue | SETP：`(fmt<<3)|cond`；BR：`(u<<31)|(neg<<30)|target`；其余为 0 |
| `sf_ialu_issue_opa` | 输入 | 256 | sf_ialu_issue | 逐 lane 源 A（`NLANES×DATA_W`） |
| `sf_ialu_issue_opb` | 输入 | 256 | sf_ialu_issue | 逐 lane 源 B |
| `sf_ialu_issue_opc` | 输入 | 256 | sf_ialu_issue | 逐 lane 源 C，仅 IMAD 有效（偏差 C5） |
| `ialu_sf_issue_rdy` | 输出 | 1 | sf_ialu_issue | 仅 `S_IDLE` 为 1 |
| `ialu_sf_br_vld` | 输出 | 1 | ialu_sf_br | 分支决议 |
| `ialu_sf_br_warp_id` | 输出 | 2 | ialu_sf_br | warp 号 |
| `ialu_sf_br_taken` | 输出 | 8 | ialu_sf_br | 每 lane taken 向量 |
| `ialu_sf_br_target` | 输出 | 32 | ialu_sf_br | 目标字地址 |
| `ialu_sf_br_brt_idx` | 输出 | 2 | ialu_sf_br | 恒 0（sf 按分支 pc 查 BRT，§1.3） |
| `sf_ialu_br_rdy` | 输入 | 1 | ialu_sf_br | sf 侧握手 |
| `ialu_rf_wb_vld` | 输出 | 1 | ialu_rf_wb | 写回 |
| `ialu_rf_wb_warp_id` | 输出 | 2 | ialu_rf_wb | warp 号 |
| `ialu_rf_wb_rd` | 输出 | 5 | ialu_rf_wb | 目的寄存器 |
| `ialu_rf_wb_lane_mask` | 输出 | 8 | ialu_rf_wb | 写使能（发射时快照随路，intf_spec §6 说明） |
| `ialu_rf_wb_wdata` | 输出 | 256 | ialu_rf_wb | 逐 lane 写数据 |
| `rf_ialu_wb_rdy` | 输入 | 1 | ialu_rf_wb | rf 侧握手 |
| `ialu_sf_wbdone_vld` | 输出 | 1 | ialu_sf_wbdone | 写回完成 |
| `ialu_sf_wbdone_warp_id` | 输出 | 2 | ialu_sf_wbdone | warp 号 |
| `ialu_sf_wbdone_rd` | 输出 | 5 | ialu_sf_wbdone | 目的寄存器（SETP 为 `pd`），供 sf 清记分板 |
| `sf_ialu_wbdone_rdy` | 输入 | 1 | ialu_sf_wbdone | sf 侧握手 |

---

## 3. 参数与配置

| 参数 | 默认 | 含义 |
|---|---|---|
| `DATA_W` | 32 | 数据/地址/载荷位宽（intf_spec §1.4） |
| `NWARPS` | 4 | warp 数/块（ma_spec §1.2） |
| `NLANES` | 8 | lane 数/warp（ma_spec §1.2） |
| `REG_AW` | 5 | 寄存器地址位宽（intf_spec §1.4） |
| `OPCODE_W` | 5 | 操作码位宽（intf_spec §1.4） |
| `BRT_IW` | 2 | BRT 表项索引位宽（intf_spec §1.4） |

派生量：`WARP_IW = $clog2(NWARPS)`，`VEC_W = NLANES×DATA_W`，谓词阵列总位宽 `NWARPS×4×NLANES`。本模块无配置输入端口，无块级上下文（谓词清零归属见 §1.4）。

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
| `br_warp_id` | 2 | 0 | br 载荷 |
| `br_taken` | 8 | 0 | br 载荷 |
| `br_target` | 32 | 0 | br 载荷 |
| `pred` | NWARPS×4×NLANES | 全 0 | 谓词寄存器阵列（§1.4） |

组合输出：`ialu_sf_issue_rdy = (state == S_IDLE)`；`ialu_rf_wb_vld = (state == S_WB)`；`ialu_sf_wbdone_vld = (state == S_WBD)`；`ialu_sf_br_vld = (state == S_BR)`；`ialu_sf_br_brt_idx = 0`。三条输出通道的载荷在各自 `vld` 保持期内由寄存值导出，稳定。

---

## 5. 数据通路

运算全部在 issue 握手当拍对 issue 载荷组合完成，结果于该拍末沿随状态迁移进入输出级寄存器（§6）。以下 `l` 均指 lane `l ∈ [0, NLANES)`。

### 5.1 lane 门控

```
out[l] = lane_mask[l] ? res[l] : 0
```

非活动 lane 的 `wdata` 恒为 0（与 cmodel 一致：未命中掩码的 lane 不写入结果）。

### 5.2 整数运算

| 操作码 | 每 lane 运算 | 说明 |
|---|---|---|
| IMAD | `(signed(opa) × signed(opb) + signed(opc)) mod 2^32` | 有符号 32×32 乘取低 32 位再加 `opc`，模 2^32（对应 mad.lo.s32，isa_spec §2） |
| IADD | `opa + opb`（mod 2^32） | isa_spec §3 |
| SHL | `opa << opb[4:0]` | 移位量取低 5 位（isa_spec §4） |
| XOR | `opa ^ opb` | isa_spec §5 |
| ORI | `opa | opb` | `opb` 为零扩展立即数广播（isa_spec §6） |
| LUI / LDP / CSRR | `opa` | 直通（偏差 C3，§1.3） |

IMAD 的低 32 位结果只依赖乘积低 32 位与 `opc` 的低 32 位之和（mod 2^32），RTL 按此实现，与 cmodel 的 64 位中间值截断等价。

### 5.3 SETP 比较

`fmt = imm[3]`，`cond = imm[2:0]`。每 lane 比较 `opa` 与 `opb`，结果位记 `ok[l]`。

fmt = 0（32 位有符号整数比较，isa_spec §8）：

| cond | 含义 | 判定 |
|---|---|---|
| 0 | lt | `signed(opa) < signed(opb)` |
| 1 | le | lt 或 eq |
| 2 | eq | `opa == opb` |
| 3 | ne | `opa != opb` |
| 4 | ge | 非 lt |
| 5 | gt | 非 lt 且非 eq |

fmt = 1（binary32 浮点比较）：

| cond | 判定 |
|---|---|
| 0 | `fgt(opb, opa)` |
| 1 | `fgt(opb, opa)` 或 `opa == opb`（按位） |
| 2 | `opa == opb`（按位） |
| 3 | `opa != opb`（按位） |
| 4 | `fgt(opa, opb)` 或 `opa == opb`（按位） |
| 5 | `fgt(opa, opb)` |

`fgt(x, y)` 为 IEEE-754 有序大于比较，与 `top/cmodel/softfloat.c` 的 `f32_gt` 逐条对齐：

1. 任一操作数为 NaN（`exp = 0xFF` 且 `mant != 0`）：结果为假；
2. 两者均为零（`exp = 0` 且 `mant = 0`，不区分符号）：结果为假；
3. 一者为零：`0 > y` 当且仅当 `y` 为负；`x > 0` 当且仅当 `x` 为正；
4. 符号相异：正者大；
5. 符号相同：正数比幅值（`{exp, mant}` 无符号比较），负数幅值小者大。

按位 eq/ne 与按位幅值比较对非规格化数同样成立（`exp = 0` 的幅值编码天然小于任何规格化数）。口径说明：eq/ne 为**按位**比较（与 cmodel 一致），即 `+0` 与 `−0` 不相等、编码相同的两个 NaN 相等；该边界行为以 cmodel 为基线。

谓词写入：SETP 的 issue 握手拍，对活动 lane 写 `pred[warp][pd][l] = ok[l]`，非活动 lane 保持（§1.4）。

### 5.4 BR 决议

`psel = rd[1:0]`，`u = imm[31]`，`neg = imm[30]`，`target = imm[29:0]`，`p = pred[warp][psel]`：

```
taken = u ? lane_mask
      : neg ? (lane_mask & ~p)
      :       (lane_mask & p)
```

对应 isa_spec §18 的 taken 集合定义；`brt_idx` 恒 0（§1.3）。BR 不产生写回与写回完成事务。

---

## 6. 状态机

### 6.1 状态定义

| 状态 | 编码 | 含义 |
|---|---|---|
| `S_IDLE` | 2'd0 | 空闲，可接受 issue |
| `S_WB` | 2'd1 | 写回段：`ialu_rf_wb_vld` 保持至握手 |
| `S_WBD` | 2'd2 | 写回完成段：`ialu_sf_wbdone_vld` 保持至握手 |
| `S_BR` | 2'd3 | 分支决议段：`ialu_sf_br_vld` 保持至握手 |

复位释放后进入 `S_IDLE`。

### 6.2 状态行为

`S_IDLE`：

1. `ialu_sf_issue_rdy = 1`；
2. issue 握手时按操作码分流，当拍完成运算与谓词更新（SETP），结果入对应输出级寄存器：
   - ALU 类（IMAD/IADD/SHL/XOR/ORI/LUI/LDP/CSRR）：锁存 wb 载荷与 wbdone 载荷，转 `S_WB`；
   - SETP：锁存 wbdone 载荷（`rd = pd`），转 `S_WBD`；
   - BR：锁存 br 载荷，转 `S_BR`。

`S_WB`：`ialu_rf_wb_vld = 1`；握手（`wb_fire`）后转 `S_WBD`。

`S_WBD`：`ialu_sf_wbdone_vld = 1`；握手（`wbd_fire`）后转 `S_IDLE`。

`S_BR`：`ialu_sf_br_vld = 1`；握手（`br_fire`）后转 `S_IDLE`。

### 6.3 状态迁移表

| 现态 | 条件 | 次态 | 动作 |
|---|---|---|---|
| `S_IDLE` | issue 握手，操作码为 ALU 类 | `S_WB` | 锁存 wb/wbdone 载荷 |
| `S_IDLE` | issue 握手，操作码为 SETP | `S_WBD` | 谓词写入；锁存 wbdone 载荷 |
| `S_IDLE` | issue 握手，操作码为 BR | `S_BR` | 锁存 br 载荷 |
| `S_IDLE` | 其余 | `S_IDLE` | — |
| `S_WB` | `wb_fire` | `S_WBD` | — |
| `S_WB` | 其余 | `S_WB` | — |
| `S_WBD` | `wbd_fire` | `S_IDLE` | — |
| `S_WBD` | 其余 | `S_WBD` | — |
| `S_BR` | `br_fire` | `S_IDLE` | — |
| `S_BR` | 其余 | `S_BR` | — |

---

## 7. 通道时序

表中各行为该拍内的电平；握手发生在 `vld` 与 `rdy` 同高那拍的时钟末沿，状态与 `vld` 于次拍生效。

### 7.1 ALU 类指令（wb、wbdone 背靠背握手）

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

### 7.2 SETP 与 BR

```
拍        T0   T1   T2          拍        T0   T1   T2
state     IDLE WBD  IDLE        state     IDLE BR   IDLE
wbd_vld   0    1    0           br_vld    0    1    0
wbd_rdy   x    1    x           br_rdy    x    1    x
```

SETP 在 T0 末沿完成谓词写入；BR 在 T0 末沿锁存 taken 向量（谓词读取为当拍组合）。

### 7.3 背压保持

`wb_rdy` 拉低 2 拍示例：

```
拍        T0   T1   T2   T3   T4   T5
state     IDLE WB   WB   WB   WBD  IDLE
wb_vld    0    1    1    1    0    0
wb_rdy    x    0    0    1    x    x
wbd_vld   0    0    0    0    1    0
```

`vld` 保持期（T1–T3）载荷稳定（§9 条 1）；握手于 T3 末沿发生，T4 起进入 `S_WBD`。三条输出通道的背压相互独立。

---

## 8. 复位与上电行为

- `rst_n = 0`（异步）：`state` 回 `S_IDLE`，全部输出级寄存器与谓词阵列按 §4 复位值清零，三条输出通道 `vld = 0`；
- `rst_n` 释放：进入 `S_IDLE`，`ialu_sf_issue_rdy = 1`，等待 sf 发射，不依赖任何启动握手；
- 谓词的块级清零不在本模块（§1.4）。

---

## 9. 协议约束

1. `vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）；
2. `vld` 不组合依赖于 `rdy`：三条输出通道 `vld` 均由状态寄存器译码，满足该条；
3. sf 只发射合法操作码集合 {IMAD, IADD, SHL, XOR, ORI, LUI, LDP, CSRR, SETP, BR}（sf 分派结果，与 cmodel 分派集合一致）；协议外操作码的行为不作约定（cmodel 置错误标志，本模块无错误端口）；
4. 单条指令在途：sf 不在上一条指令完成前向本模块发射新指令（cmodel `has_issue` 口径）；`ialu_sf_issue_rdy` 仅在 `S_IDLE` 为 1；
5. `sf_ialu_issue_pc` 接收但不使用；`ialu_sf_br_brt_idx` 恒 0（BRT 查表由 sf 按分支 pc 完成，§1.3）；
6. `rd = 0` 时本模块照常发出 wb/wbdone：R0 写忽略由 rf 执行（intf_spec §11），记分板清除由 sf 按 `rd != 0` 执行；
7. SETP 的 `rd` 字段承载 `pd ∈ [0,3]`，BR 的 `rd` 字段承载 `psel ∈ [0,3]`（isa_spec §1.7 S/B 型编码）；
8. `lane_mask` 为发射时 active mask 快照，随路至写回（intf_spec §6 说明）；本模块不修改该快照。

---

## 10. 验证要点

验证以事务级等价为准（§1.7），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| I1 | issue 握手 | DUT 接受当且仅当参考模型消费，逐拍一致 |
| I2 | 完成路径与顺序 | ALU 类 wb 先于 wbdone；SETP 仅 wbdone；BR 仅 br；全局事务序列与参考逐笔一致 |
| I3 | 载荷位精确 | br：`warp_id/taken/target/brt_idx=0`；wb：`warp_id/rd/lane_mask/wdata`（含非活动 lane 为 0）；wbdone：`warp_id/rd` |
| I4 | 谓词行为 | SETP 按 `lane_mask` 门控写入；BR 的 taken 覆盖 `u`/`neg`/`psel` 与部分掩码组合；各 warp 谓词独立 |
| I5 | SETP 比较域 | fmt=0 全 6 种 `cond` × 边界整数；fmt=1 全 6 种 `cond` × NaN/±∞/±0/非规格化/±规格化数 |
| I6 | 协议 | `vld && !rdy` 期间载荷不变、`vld` 不撤（§9 条 1）；复位期间 `vld` 恒 0 |
| I7 | `ialu_sf_issue_rdy` | 与在途状态一致：无在途为 1，有在途为 0 |
| I8 | 背压 | 三条输出通道消费者 `rdy` 任意组合（恒 1、随机、周期图案，含长背压与背靠背零延迟）下 I1–I7 成立 |
| I9 | 无死锁 | 全部事务在限界拍数内排空 |

参考模型：testbench（Verilator C++ harness）直接链接 `top/cmodel`（`ialu_step`），参考事务序列与 DUT 事务序列在线比对。
