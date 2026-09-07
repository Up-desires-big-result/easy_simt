# lsu — Load/Store Unit 模块规范

版本：v0.1
日期：2026-09-07
依据文档：`top/docs/ma_spec_v0.1.md`（§1.2、§1.3、§1.7、§7）、`top/docs/intf_spec_v0.1.md`（§1、§2、§7、§11）、`top/docs/isa_spec_v0.1.md`（§1.10）、`top/cmodel/lsu.c`、`top/cmodel/sim_common.h`（偏差 C1、C2）。
适用范围：本文档定义 lsu（Load/Store Unit）的模块级设计：端口、参数、内部状态、数据通路、控制逻辑、状态机、通道时序与协议约束，是 `lsu/rtl/lsu.sv` 实现与 `lsu/tb/` 验证的直接依据。
修订记录：2026-09-07 初版，结构沿用模块级规范骨架（bs_spec/ialu_spec/falu_spec/rf_spec 先例）。

---

## 1. 总体

### 1.1 模块定位与职责

lsu 是访存单元（ma_spec §7），职责：

- per-lane 地址生成（基址+偏移，共享内存再叠 SHBASE）；
- active mask 门控（发射时 active mask 快照，intf_spec §2）；
- **8-lane 锁步**：一条访存指令在一拍内对全部活跃 lane 同时生成地址、一拍发出一个 8-lane 请求（不逐 lane 串行，ma_spec §7）；
- 请求分 shmem/global 两路（`sm` 标志）；
- 装载数据引导写回；
- 向 ws 报停顿（LMISS 置位 / R_NONE 清除）。

边界：本模块保持薄——不含 tag 比较、不含阵列/bank（l1sm 职责，ma_spec §9）；不检测地址对齐（对齐违例为架构错误，isa_spec §1.10，由 sf 侧约束，C 模型置错误标志，本模块无错误端口）；不校验 LDG/STG 基址的 lane 间同值约束（约束 C2，sf 校验）；无分支、谓词与取指通路。请求的命中/缺失/bank 冲突处理与写通路交付均由 l1sm 完成，本模块只消费其应答。

### 1.2 指令集范围与完成路径

| 操作码 | 访问 | 完成路径 |
|---|---|---|
| LDG | 全局装载 | issue → req → rsp → wb → wbdone |
| LDS | 共享装载 | 同 LDG |
| STG | 全局存储 | issue → req → rsp（写应答）→ wbdone |
| STS | 共享存储 | 同 STG |

存储不写 rf（intf_spec §6 写回通道说明），其完成由 `lsu_sf_wbdone` 上报；写通存储等写应答返回才算完成（ma_spec §7）。装载与存储均消费 `l1sm_lsu_rsp`（装载为读数据，存储为写应答）。

### 1.3 sf 侧载荷约定

载荷为 sf 译码归一化后的形式（`top/cmodel/lsu.c` 头注）：

- LDG：`opa`=逐 lane 偏移，`imm`=均匀基址（约束 C2）；
- STG：`opa`=逐 lane 数据，`opb`=逐 lane 偏移，`imm`=均匀基址；
- LDS：`opa`=逐 lane 偏移；STS：`opa`=数据，`opb`=偏移；
- `shbase` 为 bs 启动时装载的共享内存基址（`bs_sf_launch`，intf_spec §4），随 issue 包下发（ma_spec §7）；shmem 地址已含 SHBASE（intf_spec §7）；
- `sf_lsu_issue_opb` 为偏差 C1 增设字段（intf_spec §2 载荷清单未含，与 cmodel 一致，见 §2）；
- 地址对齐违例为架构错误（isa_spec §1.10）：本模块不检测，激励须保证活跃 lane 地址 4 字节对齐。

### 1.4 时序模型

- `lsu_sf_issue_rdy`、`lsu_l1sm_req_vld`、`lsu_l1sm_rsp_rdy`、`lsu_rf_wb_vld`、`lsu_sf_wbdone_vld` 均由状态寄存器译码（§4）；`lsu_ws_stall_vld` 除寄存保持项外含两个组合置位项（LMISS 随发射握手当拍、R_NONE 于空闲拍，§5.4），不组合依赖于本通道 `rdy`；
- 状态机与 C 模型逐拍对应（§1.5）：请求握手后 1 拍为请求接收检测拍（`S_WAIT`）、写回握手后 1 拍为写回接收检测拍（`S_WBW`）；
- 单条指令在途：自发射握手至 wbdone 握手期间不再接受发射（`lsu_sf_issue_rdy` 仅 `S_IDLE` 为 1）；请求/响应单在途、严格顺序对应、不回带地址（`lsu_l1sm_rsp_rdy` 仅 `S_RSP` 为 1）；
- 复位（低电平有效异步）期间全部输出 `vld = 0`；
- 模块间握手统一遵循 intf_spec §1.2 的 vld/rdy 协议。

### 1.5 与 C 模型的对应关系

`top/cmodel/lsu.c` 的 `lsu_step()` 为事务级参考，其语义与本文档的对应：

| C 模型（`lsu_t` / `lsu_step`） | lsu RTL |
|---|---|
| `req_stage` 0：空闲，接受发射（stage 5 检测后同拍可再接受） | `S_IDLE`（wbdone 接收检测拍兼发射拍） |
| `req_stage` 1：请求置位 | `S_REQ`（`lsu_l1sm_req_vld = 1`） |
| `req_stage` 2：请求接收检测 | `S_WAIT` |
| `req_stage` 3：等响应并消费 | `S_RSP`（`lsu_l1sm_rsp_rdy = 1`） |
| `req_stage` 4：wb 置位 / 接收检测 | `S_WB` / `S_WBW` |
| `req_stage` 5：wbdone 置位 / 接收检测 | `S_WBD` / `S_IDLE` |
| `busy` | `state != S_IDLE` |
| 接受发射同拍置位 LMISS（`!stall_sent && !lsu_ws_stall.vld`） | `lmiss_go` 组合置位（§5.4） |
| 空闲拍补发 R_NONE 清除（`!busy && stall_sent`） | `clear_go` 组合置位（§5.4） |
| `stall_sent`（LMISS 已上报、清除未发） | `stall_sent` 寄存器 |
| 通道在途（`lsu_ws_stall.vld` 置位未消费） | `stall_busy`（在途消息锁存，§5.4） |
| `u->iss.warp_id`（R_NONE 载荷：最近一次发射） | `stall_warp` 寄存器（发射握手拍末沿更新） |
| 装载 `wb.wdata[l] = rsp.rdata[l]`（全 8 lane） | `S_RSP` 握手拍末沿 `wb_wdata <= rdata`（§5.3） |
| mask 外 lane `req.addr/wdata = 0` | 同义（§5.1/§5.2） |
| 消费者清零 `vld` 表示收走 | 下游 `rdy` 握手 |

顺序语义说明：C 模型每级通道事件（置位、接收检测）各占一步；RTL 以对应状态逐拍镜像（含 `S_WAIT`/`S_WBW` 两个接收检测拍），testbench 按拍锁步比对方能成立（§10）。testbench 参考推进顺序为「置激励（issue / rsp 呈现 / 各 `rdy`）→ `lsu_step` → 按同拍 `rdy` 消费参考输出」，与 C 模型顶层轮询顺序下源置位、宿消费的传递方向一致。

### 1.6 验收口径

功能验收为事务级等价（ma_spec §1.7：周期只作观测项，不作验收项）：

- 五条通道的事务序列（顺序 + 载荷）与参考位精确：`lsu_l1sm_req`（`rw`/`sm`/`mask`/`addr`/`wdata` 逐 lane）、`l1sm_lsu_rsp` 消费拍、`lsu_rf_wb`（`warp_id`/`rd`/`lane_mask`/`wdata` 逐 lane）、`lsu_sf_wbdone`、`lsu_ws_stall`（`warp_id`/`reason`）；
- 停顿上报语义：LMISS 随发射握手当拍置位、R_NONE 于空闲拍置位、通道在途或清除未发时发射的指令不报 LMISS、R_NONE 可跨指令滞留（§5.4）；
- 握手不变量：`vld` 保持期载荷稳定；复位期间全部输出 `vld = 0`；`rdy` 与内部状态一致（§1.4）。

---

## 2. 端口

端口命名、方向与位宽见 intf_spec §7（issue 载荷字段见 intf_spec §2；单源规则见 intf_spec §1）。`clk`/`rst_n` 按 intf_spec §1.3 携带。无模块级增设端口。

模块补充（行为约束，详见 §9）：

- `lsu_sf_issue_rdy` 仅在 `S_IDLE` 为 1；
- `lsu_l1sm_rsp_rdy` 仅在 `S_RSP` 为 1；
- `lsu_ws_stall_vld` 含组合置位项（§5.4），不组合依赖于 `ws_lsu_stall_rdy`；
- `sf_lsu_issue_opb` 为偏差 C1 增设字段：intf_spec §2 的 `sf_lsu_issue` 载荷清单未含第二个逐 lane 操作数，存储类指令需要「每 lane 数据 + 每 lane 偏移」两个向量，cmodel 与本模块均含 `opb`（LDG/LDS 不读取）；
- `sf_lsu_issue_imm`/`sf_lsu_issue_shbase` 为 32 位（intf_spec §4 `bs_sf_launch` 同宽下发）。

---

## 3. 参数与配置

| 参数 | 基线值 | 含义 |
|---|---|---|
| `DATA_W` | 32 | 数据/地址/载荷位宽（intf_spec §1.4） |
| `NWARPS` | 4 | warp 数/块（ma_spec §1.2） |
| `NLANES` | 8 | lane 数/warp（ma_spec §1.2） |
| `REG_AW` | 5 | 寄存器地址位宽（intf_spec §1.4） |
| `OPCODE_W` | 5 | 操作码位宽（intf_spec §1.4） |

派生量：`WARP_IW = $clog2(NWARPS)`，`VEC_W = NLANES×DATA_W`。地址通路位宽由 `DATA_W` 定死，不随参数缩放。本模块无配置输入端口。

---

## 4. 内部状态

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `state` | 3 | `S_IDLE` | 状态机，见 §6 |
| `req_rw` | 1 | 0 | 请求载荷：0 读 1 写 |
| `req_sm` | 1 | 0 | 请求载荷：1 共享 0 全局 |
| `req_mask` | 8 | 0 | 请求载荷：active mask 快照 |
| `req_addr` | 256 | 0 | 请求载荷：逐 lane 地址（mask 外 lane 为 0） |
| `req_wdata` | 256 | 0 | 请求载荷：逐 lane 写数据（mask 外 lane 为 0） |
| `wb_warp_id` | 2 | 0 | wb 载荷 |
| `wb_rd` | 5 | 0 | wb 载荷 |
| `wb_lane_mask` | 8 | 0 | wb 载荷 |
| `wb_wdata` | 256 | 0 | wb 载荷（装载：响应数据逐 lane 拷入） |
| `wbd_warp_id` | 2 | 0 | wbdone 载荷 |
| `wbd_rd` | 5 | 0 | wbdone 载荷 |
| `stall_sent` | 1 | 0 | LMISS 已上报、清除（R_NONE）未发 |
| `stall_warp` | 2 | 0 | 最近一次发射的 warp_id（R_NONE 载荷） |
| `stall_busy` | 1 | 0 | 停顿消息在途（组合置位未被消费则锁存） |
| `stall_pwarp` | 2 | 0 | 在途停顿消息载荷 |
| `stall_preason` | 3 | 0 | 在途停顿消息载荷（`R_NONE`） |

组合输出：`lsu_sf_issue_rdy = (state == S_IDLE)`；`lsu_l1sm_req_vld = (state == S_REQ)`；`lsu_l1sm_rsp_rdy = (state == S_RSP)`；`lsu_rf_wb_vld = (state == S_WB)`；`lsu_sf_wbdone_vld = (state == S_WBD)`；`lsu_ws_stall_vld = stall_busy || lmiss_go || clear_go`（§5.4）。四条状态译码通道的载荷在各自 `vld` 保持期内由寄存值导出，稳定。

---

## 5. 数据通路

地址生成在 issue 握手当拍对 issue 载荷组合完成，请求/写回载荷于该拍末沿随状态迁移入级（§6）。以下 `l` 均指 lane `l ∈ [0, NLANES)`。

### 5.1 lane 门控与地址生成

```
act[l]  = lane_mask[l]
off[l]  = STG/STS ? opb[l] : opa[l]
base    = LDS/STS ? shbase  : imm
addr[l] = act[l] ? base + off[l] : 0        （32 位模运算）
```

操作码区分 LDG/STG/LDS/STS（§1.3 载荷约定）；mask 外 lane 地址置 0（与 C 模型一致）。

### 5.2 请求数据拼装

```
rw      = STG/STS
sm      = LDS/STS
mask    = lane_mask
wdata[l] = (act[l] && rw) ? opa[l] : 0
```

存储数据取 `opa`（§1.3）；装载 `wdata` 为 0。`rw` 0=读 1=写，`sm` 1=共享 0=全局（intf_spec §7）。

### 5.3 装载数据引导写回

`S_RSP` 握手拍末沿 `wb_wdata[l] <= rdata[l]`（全 8 lane 拷贝，含 mask 外 lane，与 C 模型一致）；写门控由 `wb_lane_mask`（issue 快照随路）交 rf 解释（intf_spec §11）。`rd = 0` 时照常发出 wb/wbdone：R0 写忽略由 rf 执行。存储不走本路径（§1.2）。

### 5.4 停顿上报通路

```
lmiss_go = issue_fire && !stall_sent && !stall_busy     （LMISS 置位）
clear_go = (state == S_IDLE) && stall_sent && !stall_busy （R_NONE 清除）
stall_vld = stall_busy || lmiss_go || clear_go
载荷：lmiss_go -> {发射 warp_id, R_LMISS}
      clear_go -> {stall_warp, R_NONE}
      其余      -> {stall_pwarp, stall_preason}（在途锁存值）
```

- LMISS 随发射握手当拍组合置位（与 C 模型同拍上报口径一致，§1.5）；R_NONE 于空闲拍（`S_IDLE` 且 `stall_sent = 1` 且通道空闲）组合置位，载荷 `warp_id` 取最近一次发射的 `stall_warp`；
- 置位当拍未被消费（`ws_lsu_stall_rdy = 0`）则于拍末沿锁存（`stall_busy`），其后由寄存器保持至握手；
- 通道在途（`stall_busy`）或清除未发（`stall_sent`）时发射的指令不报 LMISS（含 R_NONE 置位与发射同拍的情形，此时 `stall_sent` 仍为 1），该指令亦无后续 R_NONE——与 C 模型一致（§1.6）；
- `stall_sent` 于 LMISS 置位拍末沿置 1、R_NONE 置位拍末沿清 0（登记于置位，不待消费）；R_NONE 可跨指令滞留（`stall_sent = 1` 期间又接受发射时，清除等待下次空闲）。

---

## 6. 状态机

### 6.1 状态定义

| 状态 | 编码 | 行为 |
|---|---|---|
| `S_IDLE` | 0 | 空闲：`lsu_sf_issue_rdy = 1`；兼作 wbdone 接收检测拍（上一拍握手后回到本状态即完成）与 R_NONE 置位拍（§5.4） |
| `S_REQ` | 1 | 请求段：`lsu_l1sm_req_vld = 1`，载荷为 §5.1/§5.2 入级值，保持至握手 |
| `S_WAIT` | 2 | 请求接收检测拍（镜像 C 模型 `req_stage` 2，§1.5）：无条件迁移 `S_RSP` |
| `S_RSP` | 3 | 响应段：`lsu_l1sm_rsp_rdy = 1`，等待并消费 `l1sm_lsu_rsp` |
| `S_WB` | 4 | 写回段（仅装载）：`lsu_rf_wb_vld = 1`，保持至握手 |
| `S_WBW` | 5 | 写回接收检测拍（镜像 C 模型 `req_stage` 4 检测，§1.5）：无条件迁移 `S_WBD` |
| `S_WBD` | 6 | 写回完成段：`lsu_sf_wbdone_vld = 1`，保持至握手 |

### 6.2 状态行为

- `S_IDLE`：issue 握手当拍组合完成地址生成与请求拼装（§5.1/§5.2），载荷于拍末沿入级，迁移 `S_REQ`；
- `S_RSP`：响应握手拍末沿，装载将 `rdata` 全 8 lane 拷入 `wb_wdata`（§5.3）并迁移 `S_WB`；存储（`req_rw = 1`）直接迁移 `S_WBD`；
- `S_REQ`/`S_WB`/`S_WBD`：各自通道保持至握手（§7.3 背压）。

### 6.3 状态迁移表

| 当前 | 条件 | 次态 |
|---|---|---|
| `S_IDLE` | `issue_fire` | `S_REQ` |
| `S_REQ` | `req_fire` | `S_WAIT` |
| `S_WAIT` | 无条件 | `S_RSP` |
| `S_RSP` | `rsp_fire && req_rw = 0`（装载） | `S_WB` |
| `S_RSP` | `rsp_fire && req_rw = 1`（存储） | `S_WBD` |
| `S_WB` | `wb_fire` | `S_WBW` |
| `S_WBW` | 无条件 | `S_WBD` |
| `S_WBD` | `wbd_fire` | `S_IDLE` |

---

## 7. 通道时序

表中各行为该拍内的电平；握手发生在 `vld` 与 `rdy` 同高那拍的时钟末沿，状态与 `vld` 于次拍生效。

### 7.1 装载全流程（LDG/LDS，无背压）

```
拍         T0   T1   T2   T3   T4   T5   T6   T7
state      IDLE REQ  WAIT RSP  WB   WBW  WBD  IDLE
iss_vld    1    0    0    0    0    0    0    0
issue_rdy  1    0    0    0    0    0    0    1
req_vld    0    1    0    0    0    0    0    0
req_rdy    x    1    x    x    x    x    x    x
rsp_vld    0    0    0    1    0    0    0    0
rsp_rdy    0    0    0    1    0    0    0    0
wb_vld     0    0    0    0    1    0    0    0
wb_rdy     x    x    x    x    1    x    x    x
wbd_vld    0    0    0    0    0    0    1    0
wbd_rdy    x    x    x    x    x    x    1    x
stall_vld  1*   0    0    0    0    0    0    1**
```

T0 末沿 issue 握手（地址/数据当拍组合生成、入级；LMISS 同拍组合置位*，§5.4）；T1 请求呈现并握手；T2 请求接收检测拍；T3 响应到达并被消费（装载：`rdata` 入级）——响应最早于请求握手后第 2 拍被消费，更早到达则保持至该拍；T4 写回呈现并握手；T5 写回接收检测拍；T6 wbdone 呈现并握手；T7 回到 `S_IDLE`，R_NONE 于该拍组合置位**（若 LMISS 未清，§7.4）。

### 7.2 存储流程（STG/STS，无背压）

```
拍         T0   T1   T2   T3   T4   T5
state      IDLE REQ  WAIT RSP  WBD  IDLE
req_vld    0    1    0    0    0    0
rsp_vld    0    0    0    1    0    0
wbd_vld    0    0    0    0    1    0
```

写应答于 T3 消费后直接进入 `S_WBD`（不写 rf，§1.2），T5 回到 `S_IDLE`。

### 7.3 背压保持

`req_rdy` 拉低 2 拍示例：

```
拍         T1   T2   T3   T4
state      REQ  REQ  REQ  WAIT
req_vld    1    1    1    0
req_rdy    0    0    1    x
```

`vld` 保持期（T1–T3）载荷稳定（§9 条 1）；握手于 T3 末沿发生。`rsp_vld` 晚到同理（`S_RSP` 保持 `rsp_rdy = 1` 等待）；`wb_rdy`/`wbd_rdy` 背压与 falu 同式（各通道 `vld` 由状态译码保持）。各通道背压相互独立。

### 7.4 停顿上报

```
（1）LMISS 同拍置位、当拍消费：      （2）R_NONE 空闲拍置位：
拍         T0        T1             拍         T6   T7         T8
stall_vld  1(LMISS)  0              stall_vld  0    1(R_NONE)  0
stall_rdy  1         x              stall_rdy  x    1          x
                                       state     WBD  IDLE       IDLE/…
```

（1）LMISS 随发射握手拍 T0 组合置位（载荷为发射 `warp_id`），当拍 `rdy = 1` 则 T0 末沿握手，否则锁存保持（§5.4）。（2）指令完成回到 `S_IDLE` 的拍（T7）R_NONE 组合置位（载荷为最近一次发射的 `warp_id`）；若停顿通道在途（LMISS/R_NONE 未消费），置位推迟至在途消息消费后的空闲拍，可跨指令滞留（§5.4）。通道在途或清除未发时发射的指令不报 LMISS（§1.6）。

---

## 8. 复位与上电行为

- `rst_n = 0`（异步）：`state` 回 `S_IDLE`，全部寄存器按 §4 复位值清零，五条输出通道 `vld = 0`；
- `rst_n` 释放：进入 `S_IDLE`，`lsu_sf_issue_rdy = 1`，等待 sf 发射，不依赖任何启动握手；
- 本模块无跨指令持久状态（`stall_warp` 为最近一次发射的 warp_id，复位 0；停顿上报状态机复位后无在途、无待发清除），复位后即为可用的初始状态。

---

## 9. 协议约束

1. `vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）：四条状态译码通道由状态保持；`lsu_ws_stall` 组合置位未被消费则锁存保持（§5.4）；
2. `vld` 不组合依赖于本通道 `rdy`：四条状态译码通道由状态寄存器译码；`lsu_ws_stall_vld` 不依赖 `ws_lsu_stall_rdy`，但含发射握手组合项（§5.4，与 C 模型同拍上报口径一致）；
3. sf 只发射合法操作码集合 {LDG, STG, LDS, STS}（sf 分派结果，与 cmodel 分派集合一致）；协议外操作码的行为不作约定（cmodel 置错误标志，本模块无错误端口）；地址对齐违例为架构错误（isa_spec §1.10），本模块不检测（§1.3）；
4. 单条指令在途：sf 不在上一条指令完成前向本模块发射新指令（cmodel `busy` 口径）；`lsu_sf_issue_rdy` 仅在 `S_IDLE` 为 1；
5. `rd = 0` 时装载照常发出 wb/wbdone：R0 写忽略由 rf 执行（intf_spec §11），记分板清除由 sf 按 `rd != 0` 执行；存储不写 rf，其完成由 `lsu_sf_wbdone` 上报（intf_spec §6 说明）；
6. `lane_mask` 为发射时 active mask 快照，随路至写回（intf_spec §2）；本模块不修改该快照；mask 外 lane 请求地址/数据为 0（§5.1/§5.2），装载 `wb_wdata` 仍全 8 lane 拷贝响应数据（§5.3，与 C 模型一致）；
7. 请求/响应单在途、严格顺序对应、不回带地址（intf_spec §7）：`lsu_l1sm_rsp_rdy` 仅在 `S_RSP` 为 1；响应最早于请求握手后第 2 拍被消费（`S_WAIT` 检测拍，§7.1）；
8. 写通存储等写应答返回才算完成（ma_spec §7）：STG/STS 于 `S_RSP` 消费写应答后进入 `S_WBD`（§6.3）。

---

## 10. 验证要点

验证以事务级等价为准（§1.6），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| L1 | 请求事务 | `rw`/`sm`/`mask` 与操作码/掩码一致，`addr`/`wdata` 逐 lane 位精确（四操作码各自的基址+偏移组合、mask 外 lane 为 0），与参考逐拍一致 |
| L2 | 响应消费 | 消费拍与参考逐拍一致；装载 `wb.wdata` 为响应数据全 8 lane 拷贝 |
| L3 | 写回完成 | wb（`warp_id`/`rd`/`lane_mask`/`wdata`）与 wbdone 载荷位精确、顺序与参考一致；存储不发 wb |
| L4 | 停顿上报 | LMISS 随发射握手当拍置位、R_NONE 空闲拍置位、通道在途/清除未发时发射不报 LMISS、R_NONE 跨指令滞留，`warp_id`/`reason` 与参考逐拍一致 |
| L5 | lane 掩码 | 全 0/单 bit/交替/全 1 等掩码下 L1–L3 成立 |
| L6 | 边界 | 地址 0、最大对齐值、32 位回绕、`imm = 0`、`shbase = 0`、`rd = 0`（R0） |
| L7 | 协议 | `vld && !rdy` 期间载荷不变、`vld` 不撤（条 1）；复位期间五通道 `vld` 恒 0；`issue_rdy` 与在途状态一致 |
| L8 | 背压 | 四消费者 `rdy` 任意组合（恒 1、随机、重背压）与响应延迟任意（含长延迟）下 L1–L7 成立 |
| L9 | 无死锁 | 全部事务在限界拍数内排空（含停顿通道重背压下 R_NONE 最终消费） |

参考模型：testbench（Verilator C++ harness）直接链接 `top/cmodel`（`lsu_step`），扮演 l1sm（消费请求、按随机延迟呈现响应）与 rf/sf/ws 消费者，参考事务序列与 DUT 事务序列按拍在线比对（§1.5）。
