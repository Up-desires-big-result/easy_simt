# bs — Block Scheduler 模块规范

版本：v0.1
日期：2026-08-28
依据文档：`top/docs/ma_spec_v0.1.md`（§1.2、§1.4、§1.6、§4）、`top/docs/intf_spec_v0.1.md`（§1、§4）、`top/cmodel/bs.c`、`top/cmodel/top.c`、`top/cmodel/sim_common.h`。
适用范围：本文档定义 bs（Block Scheduler）的模块级设计：端口、参数、内部状态、状态机、通道时序与协议约束，是 `bs/rtl/bs.sv` 实现与 `bs/tb/` 验证的直接依据。

---

## 1. 总体

### 1.1 模块定位与职责

bs 是块级调度器，职责为**纯块派发**（ma_spec §1.4、§4）：

- 推进 grid：自块 0 起逐块递增 `blockIdx`，直至 grid 边界；
- 下发启动上下文 `{blockIdx, N, SHBASE}`：同一块的启动上下文经 `bs_sf_launch` 与 `bs_ws_launch` 两条通道**同拍发起**，两侧均握手成功后该块启动完成；
- 收 `block_done`：收到后拉下一块，越界则 grid 结束、锁存 `bs_top_done`。

边界：bs 不做屏障、不碰每周期行为（ma_spec §4）。块内各 warp 的发射、停顿、屏障、完成判定归 ws；取指、译码、分化控制归 sf。

### 1.2 块级控制流

串行块执行：`MAX_BLOCKS_INFLIGHT = 1`（ma_spec §1.2、§1.6），任一时刻至多一块在途。

流程：上电（复位释放）即启动第 0 块，无独立启动握手端口；收到 `block_done` 后 `blockIdx++`，未越界则继续派发下一块，越界则 grid 结束、停机（`bs_top_done` 锁存为 1 并保持）。

`grid` 由 N 导出，与 C 模型 `sim_init` 一致：

```
grid = ceil(N / (NWARPS × NLANES)) = (N + NWARPS×NLANES − 1) >> log2(NWARPS×NLANES)
```

其中 `NWARPS = 4`、`NLANES = 8`，每块线程数 `NWARPS × NLANES = 32`。黄金口径 `N = 1000` 时 `grid = 32`。

### 1.3 启动上下文

| 字段 | 位宽 | 值 | 去处 |
|---|---|---|---|
| `block_idx` | 32 | 当前块索引，自 0 递增 | sf（`ld.param` k=2 与 CSRR blockIdx 上下文）、ws（块归属与屏障 `block_id` 比对） |
| `n` | 32 | 每块线程总数 N | 仅随 `bs_sf_launch` 下发，sf 保存 |
| `shbase` | 32 | 共享内存基址 | 仅随 `bs_sf_launch` 下发，经 sf 随 `sf_lsu_issue` 进入 lsu 作共享内存基址 |

v1 单块在途，`shbase` 恒为 0（ma_spec §4）。`SHBASE` 分区逻辑与 `MAX_BLOCKS_INFLIGHT` 参数的归属在 bs（ma_spec §1.4），v1 取最简值，不实现分区计算。

### 1.4 时序模型

- 所有输出为寄存器输出，`clk` 上升沿更新；
- 模块间握手统一遵循 intf_spec §1.2 的 vld/rdy 协议：`vld && rdy` 同时为高的时钟上升沿发生一次传输；`vld` 拉起后源模块保持 `vld` 与全部载荷稳定直至握手完成；`vld` 不组合依赖于 `rdy`；
- 复位为低电平有效异步复位（intf_spec §1.3），复位期间全部 `vld = 0`。

### 1.5 与 C 模型的对应关系

`top/cmodel/bs.c` 的 `bs_step()` 为事务级参考，其语义与本文档状态机的对应：

| C 模型（`bs_t` / `bs_step`） | bs RTL |
|---|---|
| `started`（`sim_run` 置 1） | 复位释放后直接进入派发状态，无对应寄存器 |
| `block_idx` | 寄存器 `block_idx` |
| `inflight` | `state == S_RUN` |
| `done`（`bs_top_done` 锁存） | 寄存器 `bs_top_done`，置 1 后保持 |
| 先收 `block_done`（`block_idx++`、判越界）再派发下一块 | `S_RUN` 内消费 `ws_bs_bdone`，同一状态迁移内完成 `block_idx` 递增与下一块派发入口 |
| 两条 launch 通道各自置 `vld`，`sf.active && ws.launched` 判启动完成 | `S_LAUNCH` 内两条通道各自保持 `vld` 至各自握手，两侧均握手后进入 `S_RUN` |
| `bs_step` 收 `block_done` 不校验载荷 `block_idx` | RTL 同样不校验 `ws_bs_bdone_block_idx`（载荷比对由 testbench 执行） |
| `bs_ws_bdone` 有 `vld` 即消费 | `bs_ws_bdone_rdy` 恒为 1 |
| `sim_block_start()`（谓词清零、SM 行钉扎）于启动完成时在 `bs_step` 内调用 | 属复位分发，为顶层互连位置的副作用，不是 bs 的端口行为，不在本模块实现 |

偏差说明：C 模型在任意状态收到 `ws_bs_bdone` 均消费；RTL 按 §8 协议约束仅在 `S_RUN` 接受。协议内两者行为一致，协议外不以 C 模型为准。

### 1.6 验收口径

功能验收为事务级等价（ma_spec §1.7：周期只作观测项，不作验收项）：

- 三条通道（`bs_sf_launch`、`bs_ws_launch`、`ws_bs_bdone`）的发射序列逐笔一致，载荷位精确；
- `bs_top_done` 在最后一块 `block_done` 后拉高并保持；
- `bs_top_done` 后全部输出 `vld` 恒 0。

---

## 2. 端口

端口命名与位宽与 intf_spec §4 一致；`clk`/`rst_n` 按 intf_spec §1.3 携带；`bs_cfg_n` 为本规范增设的模块级配置输入（见 §3）。

| 端口 | I/O | 位宽 | 所属通道 | 说明 |
|---|---|---|---|---|
| `clk` | 输入 | 1 | — | 时钟，上升沿有效 |
| `rst_n` | 输入 | 1 | — | 异步复位，低有效 |
| `bs_cfg_n` | 输入 | 32 | — | 配置：N（每块线程总数），静态，见 §3 |
| `bs_sf_launch_vld` | 输出 | 1 | bs_sf_launch | 向 sf 发起块启动 |
| `bs_sf_launch_block_idx` | 输出 | 32 | bs_sf_launch | 启动上下文：块索引 |
| `bs_sf_launch_n` | 输出 | 32 | bs_sf_launch | 启动上下文：N |
| `bs_sf_launch_shbase` | 输出 | 32 | bs_sf_launch | 启动上下文：共享内存基址，v1 恒 0 |
| `sf_bs_launch_rdy` | 输入 | 1 | bs_sf_launch | sf 侧握手 |
| `bs_ws_launch_vld` | 输出 | 1 | bs_ws_launch | 向 ws 发起块启动 |
| `bs_ws_launch_block_idx` | 输出 | 32 | bs_ws_launch | 启动上下文：块索引 |
| `ws_bs_launch_rdy` | 输入 | 1 | bs_ws_launch | ws 侧握手 |
| `ws_bs_bdone_vld` | 输入 | 1 | ws_bs_bdone | ws 上报块完成 |
| `ws_bs_bdone_block_idx` | 输入 | 32 | ws_bs_bdone | 完成块索引（bs 不校验） |
| `bs_ws_bdone_rdy` | 输出 | 1 | ws_bs_bdone | 恒 1：bs 始终接受 `block_done` |
| `bs_top_done` | 输出 | 1 | 顶层 | grid 结束，锁存输出（intf_spec §1.6） |

`bs_sf_launch` 与 `bs_ws_launch` 同拍发起，两侧均握手成功后块启动完成（intf_spec §4）。

---

## 3. 参数与配置

| 参数 | 默认 | 含义 |
|---|---|---|
| `DATA_W` | 32 | 数据/地址/载荷位宽（intf_spec §1.4） |
| `NWARPS` | 4 | warp 数/块（ma_spec §1.2） |
| `NLANES` | 8 | lane 数/warp（ma_spec §1.2） |

配置输入 `bs_cfg_n`：

- 来源：intf_spec §1.6 顶层信号表未含 N 的配置来源，本规范在模块级增设 `bs_cfg_n`，由顶层端口供给，复位释放前生效并保持静态；
- 用途：作为 `bs_sf_launch_n` 载荷；内部导出 `grid = (bs_cfg_n + NWARPS×NLANES − 1) / (NWARPS×NLANES)`（§1.2），用于停机判定；
- 采样：每周期寄存一次，仅使用寄存值。静态约束下寄存值与端口值一致。

---

## 4. 内部状态

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `state` | 2 | `S_LAUNCH` | 状态机，见 §5 |
| `block_idx` | 32 | 0 | 当前（派发中/在途）块索引 |
| `sf_done` | 1 | 0 | 本块 `bs_sf_launch` 已完成握手 |
| `ws_done` | 1 | 0 | 本块 `bs_ws_launch` 已完成握手 |
| `n_r` | 32 | 0 | `bs_cfg_n` 寄存值 |
| `grid_r` | 32 | 0 | grid 寄存值 |
| `bs_top_done` | 1 | 0 | grid 结束锁存 |
| `bs_sf_launch_vld` | 1 | 0 | 输出寄存器 |
| `bs_ws_launch_vld` | 1 | 0 | 输出寄存器 |

组合输出：`bs_sf_launch_block_idx = block_idx`、`bs_ws_launch_block_idx = block_idx`、`bs_sf_launch_n = n_r`、`bs_sf_launch_shbase = 0`、`bs_ws_bdone_rdy = 1`。`block_idx` 与 `n_r` 在 `S_LAUNCH` 期间不变，载荷在 `vld` 保持期间稳定。

---

## 5. 状态机

### 5.1 状态定义

| 状态 | 编码 | 含义 |
|---|---|---|
| `S_LAUNCH` | 2'd0 | 派发：两条 launch 通道发起并保持至各自握手 |
| `S_RUN` | 2'd1 | 在途：块已启动，等待 `block_done` |
| `S_DONE` | 2'd2 | 停机：grid 结束，`bs_top_done` 锁存 |

复位释放后进入 `S_LAUNCH`，对应上电启动第 0 块（ma_spec §4）。

### 5.2 状态行为

`S_LAUNCH`：

1. `sf_done = 0` 时 `bs_sf_launch_vld` 拉起并保持；若该拍 `bs_sf_launch_vld && sf_bs_launch_rdy`，则 `sf_done` 置 1、`bs_sf_launch_vld` 清 0；
2. `ws_done = 0` 时 `bs_ws_launch_vld` 拉起并保持；若该拍 `bs_ws_launch_vld && ws_bs_launch_rdy`，则 `ws_done` 置 1、`bs_ws_launch_vld` 清 0；
3. 两侧均完成（含本拍完成）时转入 `S_RUN`，块启动完成。

两条通道握手相互独立，可同拍完成，也可先后完成；先完成一侧不再重新拉起 `vld`。

`S_RUN`：

1. `bs_ws_bdone_rdy` 恒 1；
2. `ws_bs_bdone_vld` 为高即握手：若 `block_idx + 1 ≥ grid_r`，转入 `S_DONE` 且 `bs_top_done` 置 1；否则 `block_idx` 递增，`sf_done`/`ws_done` 清 0，转入 `S_LAUNCH` 派发下一块；
3. 其余拍无动作。

`S_DONE`：

1. `bs_top_done` 保持 1；
2. 两条 launch 通道 `vld` 恒 0，不再派发。

### 5.3 状态迁移表

| 现态 | 条件 | 次态 | 动作 |
|---|---|---|---|
| `S_LAUNCH` | 两侧握手均完成（含本拍） | `S_RUN` | — |
| `S_LAUNCH` | 其余 | `S_LAUNCH` | 按 §5.2 维护各通道 `vld`/`*_done` |
| `S_RUN` | `ws_bs_bdone_vld` 且 `block_idx + 1 ≥ grid_r` | `S_DONE` | `bs_top_done` 置 1 |
| `S_RUN` | `ws_bs_bdone_vld` 且 `block_idx + 1 < grid_r` | `S_LAUNCH` | `block_idx++`，`sf_done`/`ws_done` 清 0 |
| `S_RUN` | 其余 | `S_RUN` | — |
| `S_DONE` | — | `S_DONE` | — |

---

## 6. 通道时序

### 6.1 正常块派发（两侧同拍握手）

表中各行为该拍内的电平；握手发生在 `vld` 与 `rdy` 同高那拍的时钟末沿，状态与 `vld` 于次拍生效。

```
拍       T0   T1   T2   T3   T4   T5   T6
state    LAUN LAUN RUN  RUN  LAUN LAUN RUN
sf_vld   0    1    0    0    0    1    0
sf_rdy   x    1    x    x    x    1    x
ws_vld   0    1    0    0    0    1    0
ws_rdy   x    1    x    x    x    1    x
bdone    0    0    0    1    0    0    0
blk_idx  0    0    0    0    1    1    1
```

复位释放后进入 `S_LAUNCH`（T0），T1 两条通道同拍拉起；T1 末沿两侧同拍握手，T2 起块 0 在途。T3 内 `block_done` 有效，T3 末沿握手，`block_idx` 递增，T4 起派发块 1，T5 末沿块 1 握手，T6 起在途。

### 6.2 背压与错峰握手

两侧 `rdy` 独立，握手可不同拍。示例：sf 侧背压 2 拍、ws 侧第 1 拍即握手：

```
拍      T0   T1   T2   T3   T4
state   LAUN LAUN LAUN LAUN RUN
sf_vld       0    1    1    1    0
sf_rdy       -    0    0    1    -
ws_vld       0    1    0    0    0
ws_rdy       -    1    -    -    -
ws_done      0    0    1    1    1
```

ws 侧 T1 握手后 `ws_done` 置 1、`bs_ws_launch_vld` 清 0 且不再拉起；`bs_sf_launch_vld` 保持至 T3 握手（载荷在保持期内稳定，见 §8）；T4 转入 `S_RUN`。

### 6.3 最后一块完成停机

`S_RUN` 内 `ws_bs_bdone_vld` 握手且 `block_idx + 1 ≥ grid_r`：次拍起 `state = S_DONE`、`bs_top_done = 1` 并保持，两条 launch 通道 `vld` 恒 0。

---

## 7. 复位与上电行为

- `rst_n = 0`（异步）：全部状态寄存器与输出 `vld` 按 §4 复位值清零，`bs_top_done = 0`；
- `rst_n` 释放：进入 `S_LAUNCH`，以 `block_idx = 0` 开始派发，不等待任何启动握手；
- `bs_cfg_n` 须在复位释放前生效（§3）。

---

## 8. 协议约束

1. `vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）；
2. `vld` 不组合依赖于 `rdy`；RTL 全部 `vld` 为寄存器输出，满足该条；
3. `ws_bs_bdone` 仅在 `S_RUN` 出现：ws 只在其收到本块 `launch` 之后发送 `block_done`。RTL 仅在 `S_RUN` 接受 `block_done`，协议外输入的行为不作约定（C 模型在任意状态消费 `block_done`，仅协议内行为作对照基准）；
4. `bs_ws_bdone_rdy` 恒 1；`ws_bs_bdone_block_idx` 不参与 bs 逻辑，正确性由上游协议与 testbench 比对保证；
5. `bs_cfg_n` 自复位释放起静态。

---

## 9. 验证要点

验证以事务级等价为准（§1.6），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| B1 | `bs_sf_launch` 发射序列 | 共 `grid` 笔，`block_idx` 自 0 递增、`n = bs_cfg_n`、`shbase = 0`，与参考模型逐笔一致 |
| B2 | `bs_ws_launch` 发射序列 | 共 `grid` 笔，`block_idx` 与 `bs_sf_launch` 同块一致，与参考模型逐笔一致 |
| B3 | `ws_bs_bdone` 消费 | 每笔在 `S_RUN` 内消费，`bs_ws_bdone_rdy` 恒 1 |
| B4 | `bs_top_done` | 最后一块 `block_done` 后拉高并保持；此前恒 0 |
| B5 | 停机后 | 两条 launch 通道 `vld` 恒 0 |
| B6 | 协议 | `vld` 保持与载荷稳定（§8 条 1）；复位期间 `vld` 恒 0 |
| B7 | 背压 | 两侧 `rdy` 任意组合（含长背压、错峰握手、背靠背零延迟）下 B1–B6 成立 |

参考模型：`top/cmodel` 经 DPI-C 接入 testbench（`top/cmodel/dpi_ref.c` 为 DPI 前端），参考事务序列与 DUT 事务序列在线比对。
