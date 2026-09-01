# rf — Register File 模块规范

版本：v0.1
日期：2026-09-01
依据文档：`top/docs/ma_spec_v0.1.md`（§1.2、§1.3、§1.4、§11）、`top/docs/intf_spec_v0.1.md`（§1、§11）、`top/docs/isa_spec_v0.1.md`（§1.3）、`top/cmodel/rf.c`、`top/cmodel/sim_common.h`。
适用范围：本文档定义 rf（Register File）的模块级设计：端口、参数、内部状态、数据通路、控制逻辑、通道时序与协议约束，是 `rf/rtl/rf.sv` 实现与 `rf/tb/` 验证的直接依据。
修订记录：2026-09-01 初版，结构沿用模块级规范骨架（bs_spec/ialu_spec/falu_spec 先例）。

---

## 1. 总体

### 1.1 模块定位与职责

rf 是寄存器堆（ma_spec §11），职责：

- 保存通用寄存器 R0–R31：8 lane × 32 寄存器 × 32b，每 lane 独立副本（ma_spec §1.2）；
- R0 恒零：写 R0 被忽略，读 R0 恒 0（isa_spec §1.3）；
- 读：译码时双读口（rs1/rs2），操作数随 issue 包下发（ma_spec §11）；
- 写：单写口，三源固定优先级仲裁 **lsu > ialu > falu**；写使能随 `lane_mask` 随路（sf 发射时 active mask 快照，intf_spec §11 写回通道说明）。

边界：本模块只含通用寄存器；谓词寄存器 P0–P3 在 ialu（ialu_spec §1.4），特殊寄存器经 CSRR 由 ialu 直通（`sim_common.h` 偏差 C3），均不经本模块。本模块不实现模块间转发/旁路（数据冒险由 sf 记分板互锁解决，ma_spec §1.3），无分支、谓词与错误端口；§5.2 的模块内同拍写旁路仅为与 C 模型写先读后语义对齐，不构成转发机制（§9 条 6）。

### 1.2 通道与事务路径

| 通道 | 方向 | 事务路径 |
|---|---|---|
| `sf_rf_rd` / `rf_sf_rddata` | sf → rf → sf | 读请求握手 → 读应答握手；单在途，应答与请求严格顺序对应，不回带地址（intf_spec §11） |
| `lsu_rf_wb` | lsu → rf | 写握手当拍完成（阵列于该拍末沿更新） |
| `ialu_rf_wb` | ialu → rf | 同上 |
| `falu_rf_wb` | falu → rf | 同上 |

三写口每拍至多一笔握手（固定优先级仲裁，§5.3）；读请求握手与写握手相互独立，可同拍发生。

### 1.3 R0 语义

- 写 `rd = 0`：握手照常完成（写源正常推进），阵列不更新；
- 读 `rs1 = 0` / `rs2 = 0`：对应 `a` / `b` 恒为全 0，与阵列内容无关。

### 1.4 时序模型

- 应答通道为寄存器输出（`rf_sf_rddata_{vld,a,b}`），`clk` 上升沿更新；三写口 `rdy` 与 `rf_sf_rd_rdy` 为组合输出；
- 模块间握手统一遵循 intf_spec §1.2 的 vld/rdy 协议；复位（低电平有效异步）期间 `rf_sf_rddata_vld = 0`；
- 读：请求握手当拍对阵列组合采样，数据于该拍末沿进入应答寄存器，`rf_sf_rddata_vld` 自次拍起有效；
- 写：握手当拍末沿按 `lane_mask` 更新阵列；
- 单在途读事务：无在途应答时 `rf_sf_rd_rdy = 1`；在途应答于本拍被消费（`rf_sf_rddata_vld && sf_rf_rddata_rdy`）时，同拍末沿可接受下一请求（零间隙周转，与 C 模型同轮周转语义一致，§1.5）。

### 1.5 与 C 模型的对应关系

`top/cmodel/rf.c` 的 `rf_step()` 为事务级参考，其语义与本文档的对应：

| C 模型（`rf_t` / `rf_step`） | rf RTL |
|---|---|
| `r[w][rd][l]` | 存储阵列 `mem[{warp_id, reg}]`（§5.1） |
| 写口每轮至多一笔，`lsu > ialu > falu` | 三源组合仲裁，每拍至多一笔握手（§5.3） |
| `rd != 0` 方写，`lane_mask` 逐 lane 门控 | 同义（§5.3） |
| 读请求→应答（`!rf_sf_rddata.vld` 时转换） | `rsp_vld` 寄存器；`rf_sf_rd_rdy`（§1.4、§6） |
| `rs ? 读 : 0` | R0 读恒零（§5.2） |
| 消费者清零 `vld` 表示收走 | 下游 `rdy` 握手 |

顺序语义说明：C 模型顶层轮询顺序为 sf → rf → ialu/falu/lsu（`top/cmodel/top.c`），且 `rf_step` 内部先处理写口、后转换读请求，故读请求可见**此前已完成握手**的全部写（含同一 `rf_step` 内先应用的写），不可见同轮执行单元新发出的写。RTL 读数据于请求握手当拍对阵列组合采样（内容为该拍之前完成握手的全部写），另设同拍写旁路（§5.2）覆盖「写握手与读请求握手同拍且地址相同」的情形，与 `rf_step` 写先读后逐事务一致。

### 1.6 验收口径

功能验收为事务级等价（ma_spec §1.7：周期只作观测项，不作验收项）：

- 读应答事务序列与请求严格顺序对应，载荷（`a`/`b` 逐 lane）位精确；
- 写接受：三写口每拍握手来源与参考一致（至多一笔、固定优先级），被忽略的写（`rd = 0`、`lane_mask` 外 lane）经后续读出验证；
- 握手不变量：`vld` 保持期载荷稳定；复位期间 `rf_sf_rddata_vld` 恒 0；`rf_sf_rd_rdy` 与在途应答状态一致（§1.4）。

---

## 2. 端口

端口命名、方向与位宽见 intf_spec §11（单源规则见 intf_spec §1）。`clk`/`rst_n` 按 intf_spec §1.3 携带。无模块级增设端口。

模块补充（行为约束，详见 §9）：

- `rf_sf_rd_rdy = !rsp_vld || sf_rf_rddata_rdy`（无在途应答，或在途应答于本拍被消费，§1.4）；
- `rf_lsu_wb_rdy` 恒 1；`rf_ialu_wb_rdy = !lsu_rf_wb_vld`；`rf_falu_wb_rdy = !lsu_rf_wb_vld && !ialu_rf_wb_vld`（三源固定优先级，每拍至多一笔握手）。

---

## 3. 参数与配置

| 参数 | 基线值 | 含义 |
|---|---|---|
| `DATA_W` | 32 | 数据位宽（intf_spec §1.4） |
| `NWARPS` | 4 | warp 数/块（ma_spec §1.2） |
| `NLANES` | 8 | lane 数/warp（ma_spec §1.2） |
| `REG_AW` | 5 | 寄存器地址位宽，32 个寄存器（intf_spec §1.4） |

派生量：`WARP_IW = $clog2(NWARPS)`，`VEC_W = NLANES×DATA_W`，`DEPTH = NWARPS×2^REG_AW`（基线 128 字），存储容量 `DEPTH×VEC_W`（基线 32768 bit）。本模块无配置输入端口。

---

## 4. 内部状态

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `mem[0..DEPTH-1]` | `VEC_W` | 全 0 | 存储阵列，索引 `{warp_id, reg}`（§5.1） |
| `rsp_vld` | 1 | 0 | 在途读应答（即 `rf_sf_rddata_vld`） |
| `rsp_a` | `VEC_W` | 0 | 应答载荷 a（rs1 读出） |
| `rsp_b` | `VEC_W` | 0 | 应答载荷 b（rs2 读出） |

组合输出：`rf_sf_rddata_vld = rsp_vld`；`rf_sf_rd_rdy` 与三写口 `rdy` 见 §2 模块补充。应答载荷在 `vld` 保持期内由寄存值导出，稳定。

阵列复位全 0 与 C 模型初始状态一致；isa_spec §1.3 要求程序先写后读，全 0 复位值为实现选择而非架构保证。

---

## 5. 数据通路

### 5.1 存储阵列与寻址

阵列 `mem` 共 `DEPTH` 字、每字 `VEC_W` 位，索引为 `{warp_id, reg}`（`WARP_IW + REG_AW` 位）；每字内为 `NLANES` 个 `DATA_W` lane 字段，第 `l` 个 lane 占 `[l*DATA_W +: DATA_W]`。warp 间、寄存器间、lane 间均物理独立。

### 5.2 读路径

请求握手当拍组合采样，末沿入应答寄存器：

```
base_a = (rs1 != 0) ? mem[{warp_id, rs1}] : 0
base_b = (rs2 != 0) ? mem[{warp_id, rs2}] : 0
```

同拍写旁路：设本拍三源仲裁命中的写为 `{wb_warp_id, wb_rd, wb_lane_mask, wb_wdata}`（§5.3），当写握手与读请求握手同拍发生、`wb_rd != 0` 且 `{wb_warp_id, wb_rd}` 与 `{warp_id, rs1}`（或 `rs2`）相同时，该地址读出值逐 lane 取：

```
a[l] = (wb_lane_mask[l]) ? wb_wdata[l] : base_a[l]
```

即与 `rf_step` 写先读后的顺序一致（§1.5）；其余情形取 `base_a` / `base_b`。采样结果于末沿进入 `rsp_a` / `rsp_b`，`rsp_vld` 置 1。

### 5.3 写路径与三源仲裁

三写口载荷同型（`{warp_id, rd, lane_mask, wdata}`，intf_spec §11）。多源 `vld` 同拍为高时按固定优先级授予（ma_spec §11）：

1. `lsu_rf_wb_vld` 为高：授予 lsu；
2. 否则 `ialu_rf_wb_vld` 为高：授予 ialu；
3. 否则 `falu_rf_wb_vld` 为高：授予 falu。

授予源握手，其余源 `rdy = 0`、保持 `vld` 与载荷等待（intf_spec §1.2 条 3）；每拍至多一笔写握手。

握手且 `rd != 0` 时，末沿逐 lane 更新：

```
lane_mask[l] = 1 : mem[{warp_id, rd}][l] <- wdata[l]
lane_mask[l] = 0 : 保持原值
```

（isa_spec §1.3 写回规则：mask 外 lane 的目标寄存器保持原值。）`rd = 0` 时握手完成但阵列不更新（§1.3）。

---

## 6. 控制逻辑

本模块无指令级状态机，唯一时序状态为在途读应答 `rsp_vld`：

| 状态 | 含义 |
|---|---|
| `rsp_vld = 0` | 空闲，`rf_sf_rd_rdy = 1`，可接受读请求 |
| `rsp_vld = 1` | 应答在途，`rf_sf_rddata_vld` 保持至握手 |

状态迁移表：

| 现态 | 条件 | 次态 | 动作 |
|---|---|---|---|
| 0 | 读请求握手 | 1 | 采样读数据入 `rsp_a`/`rsp_b`（§5.2） |
| 0 | 其余 | 0 | — |
| 1 | 应答握手 | 0 | — |
| 1 | 应答握手且读请求握手（同拍） | 1 | 旧应答交付，新读数据入 `rsp_a`/`rsp_b`（零间隙周转，§1.4） |
| 1 | 其余 | 1 | — |

`rsp_vld = 1` 且应答未被消费时 `rf_sf_rd_rdy = 0`，读请求不可能握手，故不存在「应答在途且未被消费、又接受新请求」的情形。写路径无状态：仲裁与阵列更新均为组合判定、当拍末沿生效。

---

## 7. 通道时序

表中各行为该拍内的电平；握手发生在 `vld` 与 `rdy` 同高那拍的时钟末沿。

### 7.1 读请求 → 应答（零间隙周转）

连续两笔读请求，应答均当拍被消费：

```
拍               T0   T1   T2   T3
sf_rf_rd_vld     1    1    0    0
rf_sf_rd_rdy     1    1    1    1
rf_sf_rddata_vld 0    1    1    0
sf_rf_rddata_rdy x    1    1    x
```

T0 末沿请求 R1 握手（当拍采样、数据入级）；T1 起应答 R1 有效并于 T1 末沿握手，同拍末沿请求 R2 握手（`rd_rdy` 因应答被消费而为 1）；T2 起应答 R2 有效，T2 末沿握手后回到空闲。

### 7.2 应答背压保持

应答在途、`sf_rf_rddata_rdy` 拉低 2 拍：

```
拍               T0   T1   T2   T3   T4
rf_sf_rddata_vld 0    1    1    1    0
sf_rf_rddata_rdy x    0    0    1    x
rf_sf_rd_rdy     1    0    0    1    1
```

`vld` 保持期（T1–T3）载荷稳定（§9 条 1）；握手于 T3 末沿发生。保持期内新读请求因 `rd_rdy = 0` 等待；T3 同拍若有请求则末沿握手（零间隙周转）。

### 7.3 三源写竞争

lsu/ialu/falu 的 `vld` 同拍为高：

```
拍               T0   T1   T2
lsu_rf_wb_vld    1    0    0
ialu_rf_wb_vld   1    1    0
falu_rf_wb_vld   1    1    1
rf_lsu_wb_rdy    1    1    1
rf_ialu_wb_rdy   0    1    1
rf_falu_wb_rdy   0    0    1
```

每拍至多一笔写握手：T0 授予 lsu，T1 授予 ialu，T2 授予 falu；未授予源保持 `vld` 与载荷（§9 条 2）。

---

## 8. 复位与上电行为

- `rst_n = 0`（异步）：`rsp_vld` 回 0，`rsp_a`/`rsp_b` 清零，存储阵列全部清 0；`rf_sf_rddata_vld = 0`；
- `rst_n` 释放：`rf_sf_rd_rdy = 1`，三写口 `rdy` 按 §2 模块补充取值，等待读请求与写回，不依赖任何启动握手；
- 阵列全 0 为初始状态（§4），块启动不改变阵列内容（程序先写后读，isa_spec §1.3）。

---

## 9. 协议约束

1. `rf_sf_rddata_vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）；输入通道（`sf_rf_rd` 与三写口）的 `vld` 保持由源模块（sf/ialu/falu/lsu）负责；
2. 三写口被仲裁阻塞期间（`vld && !rdy`），源模块保持 `vld` 与载荷稳定直至握手；
3. `vld` 不组合依赖于 `rdy`：`rf_sf_rddata_vld` 由 `rsp_vld` 寄存器导出；`rf_sf_rd_rdy` 依赖 `rsp_vld` 与 `sf_rf_rddata_rdy`、三写口 `rdy` 依赖各自及更高优先级源的 `vld`，均不违反该条。`rf_sf_rd_rdy` 对 `sf_rf_rddata_rdy` 的依赖要求 sf 侧 `sf_rf_rddata_rdy` 由 sf 内部状态生成、不依赖 `rf_sf_rd_rdy`，避免组合环；
4. 读请求单在途：sf 不在上一笔应答完成前发出新请求；在途应答未消费时 `rf_sf_rd_rdy = 0` 兜底；
5. 三写口拓扑固定为 ialu/falu/lsu 三源（intf_spec §1.7 连接矩阵），无第四写源；
6. 全系统中，某寄存器的写握手先于其 `wbdone`、先于 sf 清记分板后重新发射的读请求（sf 互锁，ma_spec §1.3），故同地址的写握手与读请求握手在正常运行中不同拍发生；§5.2 同拍写旁路仅为与 `rf_step` 写先读后语义逐事务一致，不构成转发机制。

---

## 10. 验证要点

验证以事务级等价为准（§1.6），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| F1 | 写接受 | 三写口每拍握手来源与参考一致（至多一笔、`lsu > ialu > falu`），逐拍一致 |
| F2 | 三源竞争 | 两两及三源同拍 `vld`：按优先级逐拍依次授予，未授予源保持 |
| F3 | 读应答顺序与载荷 | 应答与请求严格顺序对应；`a`/`b` 逐 lane 位精确（含各 warp/寄存器组合） |
| F4 | R0 | 写 R0 被忽略（任意数据写后读恒 0）；`rs1`/`rs2 = 0` 读恒 0；未写过的寄存器读为复位值 0 |
| F5 | lane 掩码 | `lane_mask` 命中 lane 更新、未命中 lane 保持原值；含全 0/单 bit/交替/全 1 掩码 |
| F6 | 读写顺序 | 写后读、读写交错、同拍写读（含同地址，经 §5.2 旁路）与参考逐事务一致 |
| F7 | 协议 | `vld && !rdy` 期间应答载荷不变、`vld` 不撤（条 1）；复位期间 `rf_sf_rddata_vld` 恒 0 |
| F8 | `rf_sf_rd_rdy` 与背压 | 与在途应答状态一致（§1.4）；消费者 `rdy` 任意组合（恒 1、随机、周期图案，含长背压与零间隙背靠背）下 F1–F7 成立 |
| F9 | 无死锁 | 全部事务在限界拍数内排空 |

参考模型：testbench（Verilator C++ harness）直接链接 `top/cmodel`（`rf_step`），参考事务序列与 DUT 事务序列在线比对。
