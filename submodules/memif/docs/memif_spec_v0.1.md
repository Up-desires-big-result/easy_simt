# memif — Memory Interface 模块规范

版本：v0.1
日期：2026-09-10
依据文档：`top/docs/ma_spec_v0.1.md`（§1.2、§1.7、§10）、`top/docs/intf_spec_v0.1.md`（§1、§8、§9、§10）、`top/cmodel/memif.c`、`top/cmodel/sim_common.h`（偏差声明：MEM_LAT 建模）。
适用范围：本文档定义 memif（Memory Interface）的模块级设计：端口、参数、内部状态、数据通路、控制逻辑、状态机、通道时序与协议约束，是 `memif/rtl/memif.sv` 实现与 `memif/tb/` 验证的直接依据。
修订记录：2026-09-10 初版，结构沿用模块级规范骨架（bs_spec/ialu_spec/falu_spec/rf_spec/lsu_spec 先例）。

---

## 1. 总体

### 1.1 模块定位与职责

memif 是片外唯一通道（ma_spec §10），职责：

- 仲裁 icache 与 l1sm 的回填/写通请求：**固定优先级（icache 优先）**，**单请求在途**；
- 对外为 AXI4 主设备（全五通道 AW/W/B/AR/R，信号级见 intf_spec §10）；
- 读（回填）：单拍整行（`arlen=0`、`arsize=3'b101` 32B/拍）；
- 写（写通）：4B 窄传（`awlen=0`、`awsize=3'b010`），`wdata` 按 `awaddr[4:2]` 定位、`wstrb` 置对应 4 字节，等 BRESP 返回才算完成；
- 错误检测：`rresp/bresp` 非 OKAY 置 `memif_top_err` 并保持（§5.4）。

边界：本模块不含片外存储与固定延迟——片外存储为 tb 侧 AXI 从设备的行为级后备，片外固定延迟 `MEM_LAT`（默认 20，与 ISS 基线同参）由 tb 的 AXI 从设备建模，本模块自身不加延迟（intf_spec §10）；本模块不含 tag 比较、阵列/bank（icache/l1sm 职责）。请求队列留参数位，v1 不实现（ma_spec §10）。

### 1.2 通道与事务路径

| 通道 | 方向 | 事务路径 |
|---|---|---|
| `icache_memif_req` → `memif_icache_rsp` | icache → memif → icache | 读请求握手（与 AXI AR 握手同拍）→ 从设备延迟 → R 拍桥接为内部响应握手 |
| `l1sm_memif_req` → `memif_l1sm_rsp` | l1sm → memif → l1sm | 读：同 icache 路径；写：请求握手（与 AW/W 握手同拍）→ 从设备延迟 → B 拍桥接（`line=0`） |
| AXI AW/W | memif → 从设备 | 写请求（单拍、4B 窄传） |
| AXI B | 从设备 → memif | 写应答 |
| AXI AR | memif → 从设备 | 读请求（单拍、32B 整行） |
| AXI R | 从设备 → memif | 读数据（单拍整行） |

### 1.3 访存语义与地址口径

本模块内部载荷不含存储语义：AXI 请求只携带地址/数据，哪些地址返回什么由 tb 侧 AXI 从设备实现。从设备的地址口径与 C 模型片外后备存储一致（`top/cmodel/top.c`）：

- 指令段 `[0, 4×IMEM_WORDS)`：按字索引 `imem[addr>>2]`，超出程序长度 `imem_n` 的字返回 0（C 模型 `imem[w0+i] < imem_n` 口径）；
- 全局输入段 `[in_base, in_base+4×GMEM_WORDS)`（in_base=0x00100000）与输出段 `[out_base, ...)`（out_base=0x00200000）：按字读写。

两套错误机制对应：C 模型无非 OKAY 响应，越界访问置错误标志（cmodel `memif.err`）；RTL 侧不检查地址范围，从设备错误响应（`rresp/bresp` 非 OKAY）置 `memif_top_err`（intf_spec §10）。单元验证的激励生成保证 icache 请求地址落在指令段、l1sm 请求地址落在两段全局数据区内（读留 8 字余量，§10）。

### 1.4 时序模型

- **请求通道组合直通**：`axi_arvalid`/`axi_awvalid`/`axi_wvalid` 随内部请求组合呈现，内部请求握手与 AXI 请求通道握手**同拍**完成（§7）；`memif_icache_req_rdy`/`memif_l1sm_req_rdy` 组合依赖 AXI 对应 `ready`；
- **响应通道组合桥接**：`memif_icache_rsp_vld`/`memif_l1sm_rsp_vld` 由状态译码与 `axi_rvalid`/`axi_bvalid` 组合导出（载荷直通 `axi_rdata` 或写应答恒 0）；`axi_rready`/`axi_bready` 由内部消费者 `rdy` 门控，从设备保持 `rvalid`/`bvalid` 至本模块接收；
- 本模块自身不加延迟：`MEM_LAT` 归 tb 侧从设备（请求握手拍末沿装载倒计时、计数到 0 呈现 `rvalid`/`bvalid`，与 C 模型 countdown 逐步同构，§1.5）；
- 单请求在途：自内部请求握手至内部响应握手期间不再受理（§6）；
- 复位（低电平有效异步）期间全部输出 `vld = 0`、`memif_top_err = 0`；
- 模块间握手统一遵循 intf_spec §1.2 的 vld/rdy 协议。

### 1.5 与 C 模型的对应关系

`top/cmodel/memif.c` 的 `memif_step()` 为事务级参考，其语义与本文档的对应：

| C 模型（`memif_t` / `memif_step`） | memif RTL + tb 从设备 |
|---|---|
| `busy` | `state != S_IDLE` |
| `to_icache` / `rw` | `to_icache_q` / `rw_q` |
| `countdown = memlat`（接收拍置） | 从设备请求握手拍末沿装载 `MEM_LAT` 倒计时 |
| 倒计时递减（每步） | 从设备每拍递减 |
| 到期提交响应（接收拍+MEM_LAT+1） | 从设备呈现 `rvalid`/`bvalid`（请求握手拍+MEM_LAT+1），组合桥接为内部响应 |
| `rsp_set` 等消费者取走 | `rvalid`/`bvalid` 保持至 `rready`/`bready`（内部响应握手） |
| 取走后同步受理下一请求 | 响应握手次拍回 `S_IDLE` 受理 |
| `imem`/`gmem_in`/`gmem_out` 片外后备 | tb 从设备存储（同初始化、同写序） |
| `gmem_write`（提交拍） | 从设备 B 握手拍末沿按 `wstrb` 提交 |
| 写应答 `line=0` | 同义（§5.3） |
| `memlat`（main.c `--memlat`） | tb 从设备 `MEM_LAT`（含 0） |
| 非 OKAY 不存在 / 越界置 `err` | `rresp/bresp` 非 OKAY → `memif_top_err`（§5.4） |

顺序语义说明：C 模型接收请求的步内置 `countdown`，经 `MEM_LAT` 步递减后于接收拍+MEM_LAT+1 提交响应；tb 从设备在请求握手拍末沿装载倒计时、计数到 0 呈现响应，呈现拍与提交拍逐拍一致（含 `MEM_LAT=0`：接收次拍即响应）。testbench 参考推进顺序为「置激励（两路请求 / 两路 `rdy`）→ `memif_step` → 按同拍 `rdy` 消费参考响应」，内部请求受理、内部响应消费、AXI 握手三类事件逐拍比对（§10）。

### 1.6 验收口径

功能验收为事务级等价（ma_spec §1.7：周期只作观测项，不作验收项）：

- 内部请求受理拍与来源（两源同拍挂起时 icache 优先）与参考逐拍一致；
- 内部响应消费拍与载荷（回填整行 8 字、写应答 `line=0`）位精确；
- AXI 请求格式：`arlen=awlen=0`、`arsize=3'b101`、`awsize=3'b010`、`arburst=awburst=2'b01`、ID 恒 0、`wlast=1`、`wstrb` 按 `awaddr[4:2]` 定位、`wdata` 行内字定位；
- `memif_top_err`：非 OKAY 呈现拍次拍置位并保持至复位（注入拍事务序列不受影响）；
- 握手不变量：`vld` 保持期载荷稳定；复位期间全部输出 `vld = 0`；`rdy` 与在途状态一致（§1.4）。

---

## 2. 端口

端口命名、方向与位宽见 intf_spec §10（内部侧 + 对外 AXI4 五通道；AXI 参数见 intf_spec §1.4；单源规则见 intf_spec §1）。`clk`/`rst_n` 按 intf_spec §1.3 携带。无模块级增设端口。

模块补充（行为约束，详见 §9）：

- `memif_icache_req_rdy`/`memif_l1sm_req_rdy` 组合依赖 AXI 对应 `ready`（请求组合直通，§1.4）；`memif_l1sm_req_rdy` 另依赖 `icache_memif_req_vld`（固定优先级）；
- `memif_icache_rsp_vld`/`memif_l1sm_rsp_vld` 组合依赖 `axi_rvalid`/`axi_bvalid`（响应组合桥接，§1.4），不组合依赖于本通道 `rdy`；
- `axi_rid`/`axi_bid`/`axi_rlast` 不检测（单在途、单拍，ID 恒 0 由本模块发出）。

---

## 3. 参数与配置

| 参数 | 基线值 | 含义 |
|---|---|---|
| `AXI_ID_W` | 4 | AXI ID 位宽，v1 恒置 0（intf_spec §1.4） |
| `AXI_ADDR_W` | 32 | AXI 地址位宽（intf_spec §1.4） |
| `AXI_DATA_W` | 256 | AXI 数据位宽 = 1 行（intf_spec §1.4） |
| `AXI_STRB_W` | 32 | AXI 字节选通位宽 = AXI_DATA_W/8（intf_spec §1.4） |
| `ILINE_WORDS` | 8 | 指令行字数（ma_spec §1.6） |

派生量：`LINE_W = ILINE_WORDS×32`。约束：`AXI_DATA_W = LINE_W`（回填整行口径，RTL 含几何守卫）；写窄传定位要求 `AXI_DATA_W` 为 32 的倍数。本模块无配置输入端口；片外延迟 `MEM_LAT` 不在本模块内（tb 从设备参数，§1.4）。

---

## 4. 内部状态

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `state` | 2 | `S_IDLE` | 状态机，见 §6 |
| `to_icache_q` | 1 | 0 | 在途响应目标：1=icache 0=l1sm |
| `rw_q` | 1 | 0 | 在途事务：0=读 1=写 |
| `aw_done_q` | 1 | 0 | `S_WREQ`：AW 已握手（写请求通道补齐，§6.2） |
| `w_done_q` | 1 | 0 | `S_WREQ`：W 已握手 |
| `err_q` | 1 | 0 | 非 OKAY 粘滞（`memif_top_err`） |

组合输出：`memif_icache_req_rdy = (state==S_IDLE) && axi_arready`；`memif_l1sm_req_rdy` 见 §5.1；`axi_arvalid`/`axi_awvalid`/`axi_wvalid` 见 §5.2；`axi_rready`/`axi_bready`、`memif_*_rsp_vld` 见 §5.3；`memif_top_err = err_q`。两条内部响应通道的载荷在 `vld` 保持期内由 `axi_rdata`（读）或恒 0（写）导出，从设备保持呈现期稳定。

---

## 5. 数据通路

### 5.1 请求仲裁（组合，icache 优先）

```
ic_sel = icache_memif_req_vld
l1_sel = !icache_memif_req_vld && l1sm_memif_req_vld
req_wr = l1_sel && l1sm_memif_req_rw

memif_icache_req_rdy = (state==S_IDLE) && axi_arready
memif_l1sm_req_rdy   = (state==S_IDLE) && !icache_memif_req_vld
                        && (l1sm_memif_req_rw ? (axi_awready && axi_wready)
                                               : axi_arready)
                      || (state==S_WREQ) && aw_ok && w_ok
```

icache 请求恒为读；两源同拍挂起时 icache 优先（l1sm 的 `rdy` 被 `icache_memif_req_vld` 压制）。写请求的内部握手与 AW/W 两通道握手同时完成（同拍或经 `S_WREQ` 补齐，§6.2）。

### 5.2 AXI 请求格式化（组合直通）

```
req_addr = ic_sel ? icache_memif_req_addr : l1sm_memif_req_addr
wsel     = req_addr[4:2]                     // 写窄传字定位

读：arvalid = (state==S_IDLE) && !req_wr && (任一请求挂起)
    araddr = req_addr, arid = 0, arlen = 0, arsize = 3'b101, arburst = 2'b01
写：awvalid/wvalid = 写请求挂起（未完成，S_WREQ 下按 aw_done_q/w_done_q 抑制）
    awaddr = req_addr, awid = 0, awlen = 0, awsize = 3'b010, awburst = 2'b01
    wdata  = req_wdata << (wsel×32)，其余字节 0
    wstrb  = 4'hF << (wsel×4)，wlast = 1
```

`valid` 随内部请求组合呈现并保持至握手（内部请求源保持 `vld`，协议 §9 条 1）；内部请求握手与 AXI 请求握手同拍。

### 5.3 响应桥接（组合）

```
axi_rready = (state==S_RSP) && !rw_q && (to_icache_q ? icache_memif_rsp_rdy
                                                     : l1sm_memif_rsp_rdy)
axi_bready = (state==S_RSP) && rw_q && l1sm_memif_rsp_rdy

memif_icache_rsp_vld  = (state==S_RSP) && to_icache_q && axi_rvalid
memif_icache_rsp_data = axi_rdata
memif_l1sm_rsp_vld    = (state==S_RSP) && !to_icache_q && (rw_q ? axi_bvalid
                                                                : axi_rvalid)
memif_l1sm_rsp_data   = rw_q ? 0 : axi_rdata
```

读：R 拍数据整行直通（icache 回填 / l1sm 读）；写：B 拍桥接为 `memif_l1sm_rsp`，载荷恒 0（C 模型写应答口径，§1.5）。AXI 拍完成（`rvalid&&rready` / `bvalid&&bready`）与内部响应握手为同一事件；内部消费者未就绪时 `rready`/`bready` 为 0，从设备保持呈现。

### 5.4 错误检测

响应呈现拍（`state==S_RSP` 且 `rvalid`/`bvalid` 有效）检测 `rresp`/`bresp` 非 OKAY，次拍起 `memif_top_err = 1` 并保持至复位（粘滞）；响应照常交付（数据不作约定）。本模块不检测地址范围（§1.3）。

---

## 6. 状态机

### 6.1 状态定义

| 状态 | 编码 | 行为 |
|---|---|---|
| `S_IDLE` | 0 | 空闲：读请求同拍入 AXI（AR 握手 = 内部受理）；写请求 AW/W 同拍完成或转 `S_WREQ` |
| `S_WREQ` | 1 | 写请求通道补齐：AW/W 握手分离时保持未完成通道的 `valid`，两通道均完成后内部受理并入 `S_RSP`（tb 从设备请求通道恒 ready，单元验证不激发，为一般 AXI 从设备预留） |
| `S_RSP` | 2 | 响应等待：桥接 AXI R/B 拍为内部响应（§5.3），握手后回 `S_IDLE` |

### 6.2 状态行为

- `S_IDLE`：读请求握手拍末沿锁存 `to_icache_q`/`rw_q` 入 `S_RSP`；写请求当拍 AW/W 均握手则同拍入 `S_RSP`，仅部分握手则入 `S_WREQ` 并登记 `aw_done_q`/`w_done_q`；
- `S_WREQ`：抑制已握手通道的 `valid`，未握手通道保持；两通道均完成拍内部受理、清登记位、入 `S_RSP`；
- `S_RSP`：内部响应握手（= AXI R/B 拍，§5.3）次拍回 `S_IDLE`，可受理下一请求。

### 6.3 状态迁移表

| 当前 | 条件 | 次态 |
|---|---|---|
| `S_IDLE` | 写请求 && `aw_ok && w_ok`（同拍完成） | `S_RSP` |
| `S_IDLE` | 写请求 && 部分握手 | `S_WREQ` |
| `S_IDLE` | `ar_fire`（读受理） | `S_RSP` |
| `S_WREQ` | `aw_ok && w_ok` | `S_RSP` |
| `S_RSP` | `rsp_fire`（R/B 拍 = 内部响应握手） | `S_IDLE` |

---

## 7. 通道时序

表中各行为该拍内的电平；握手发生在 `vld` 与 `rdy` 同高那拍的时钟末沿。设 `MEM_LAT = L`（tb 从设备，§1.4）。

### 7.1 读全流程（icache 回填，无背压）

```
拍          T0   T1  ... TL   TL+1   TL+2
state       IDLE (S_RSP ...........)  IDLE
ireq_vld    1    0                  0/1（下一请求）
ireq_rdy    1    0                  1
arvalid     1    0                  0/1
arready     1    -                  -
rvalid      0    0   ... 0    1     0
rready      0    0   ... 0    1     0
rsp_vld     0    0   ... 0    1     0
rsp_rdy     x    x   ... x    1     x
```

T0 内部请求握手与 AR 握手同拍完成；从设备 T0 末沿装载倒计时，第 L 拍末沿计数到 0、TL+1 呈现 `rvalid`（`MEM_LAT=0` 时 T0 末沿即装载即呈现、T1 呈现）；TL+1 R 拍与内部响应握手同时完成，TL+2 回 `S_IDLE` 可受理下一请求。l1sm 读同构（响应走 `memif_l1sm_rsp`）。

### 7.2 写全流程（l1sm 写通，无背压）

```
拍          T0   T1  ... TL   TL+1   TL+2
state       IDLE (S_RSP ...........)  IDLE
awvalid     1    0                  0/1
wvalid      1    0                  0/1
aw/wready   1    -                  -
bvalid      0    0   ... 0    1     0
bready      0    0   ... 0    1     0
rsp_vld     0    0   ... 0    1     0
rsp_data    -    -   ... -    0     -
```

T0 内部写请求握手与 AW/W 握手同拍（`awaddr[4:2]` 定位 `wdata`/`wstrb`，§5.2）；从设备 B 通道倒计时同构；TL+1 B 拍桥接为 `memif_l1sm_rsp`（载荷 0）并同时完成内部响应握手；写数据由从设备于 B 握手拍末沿按 `wstrb` 提交。

### 7.3 背压（内部响应消费延迟）

`rsp_rdy` 拉低示例（读）：

```
拍          TL+1  TL+2  TL+3
rsp_vld     1     1     1
rsp_rdy     0     0     1
rready      0     0     1
rvalid      1     1     1
```

内部消费者未就绪时 `rready`/`bready` 为 0，从设备保持 `rvalid`/`bvalid` 及载荷（协议 §9 条 1）；握手于 TL+3 末沿完成，次拍回 `S_IDLE`。请求通道背压（`arready`/`awready`/`wready` 拉低）下 `valid` 保持、内部 `rdy` 随之压制；写请求 AW/W 握手分离经 `S_WREQ` 补齐（§6.2）。

### 7.4 与 C 模型的拍序对齐

| 事件 | C 模型 | RTL + tb 从设备 |
|---|---|---|
| 请求受理 | 接收步置 `countdown` | 请求握手拍（AXI 请求握手同拍），从设备装载倒计时 |
| 响应提交 | 受理步 + `MEM_LAT` + 1 | `rvalid`/`bvalid` 呈现拍（请求握手拍 + `MEM_LAT` + 1），组合桥接为内部响应 |
| 响应消费 | 消费者清 `vld` 的步 | 内部响应握手拍（= AXI R/B 拍） |
| 再受理 | 消费步次步（同步受理） | 响应握手次拍（`S_IDLE`） |

`MEM_LAT = 0` 时两侧均为受理次拍提交（从设备装载即呈现）。

---

## 8. 复位与上电行为

- `rst_n = 0`（异步）：`state` 回 `S_IDLE`，全部寄存器按 §4 复位值清零；输出 `vld`（`memif_*_rsp_vld`、`axi_*valid`）恒 0（复位期间内部请求 `vld = 0`），`memif_top_err = 0`；
- `rst_n` 释放：进入 `S_IDLE`，两路内部请求 `rdy` 随 AXI `ready` 开放，不依赖任何启动握手；
- 本模块无跨事务持久状态（在途上下文随事务清零、错误标志仅复位清除），复位后即为可用的初始状态。

---

## 9. 协议约束

1. `vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）：内部请求 `vld` 由源保持；内部响应 `vld` 由从设备 `rvalid`/`bvalid` 保持（经 §5.3 桥接）；AXI 请求 `valid` 随内部请求保持；
2. `vld` 不组合依赖于本通道 `rdy`：内部响应 `vld` 依赖 `rvalid`/`bvalid`（从设备侧，不依赖本模块 `rready`）；AXI 请求 `valid` 依赖内部请求 `vld`（源侧）；
3. 内部请求源只发合法载荷：icache 恒读、l1sm 读/写（`rw`），地址对齐由上游保证（isa_spec §1.10 口径归上游，本模块不检测）；
4. 单请求在途：`S_RSP`/`S_WREQ` 期间两路内部请求 `rdy = 0`；
5. 固定优先级 icache 优先（ma_spec §10）：两源同拍挂起时 l1sm `rdy` 被 `icache_memif_req_vld` 压制；
6. AXI v1 约束（intf_spec §10）：ID 恒 0；`arburst=awburst=2'b01`（INCR）；读 `arlen=0`、`arsize=3'b101` 单拍整行；写 `awlen=0`、`awsize=3'b010` 4B 窄传、`wdata` 按 `awaddr[4:2]` 定位、`wstrb` 置对应 4 字节、`wlast=1`、等 BRESP 返回才算完成；
7. `axi_rid`/`axi_bid`/`axi_rlast` 不检测（单在途、单拍整行）；
8. `rresp`/`bresp` 非 OKAY：响应呈现拍检测、次拍起 `memif_top_err = 1` 并保持至复位；响应照常交付（§5.4）；
9. 写请求 AW/W 握手分离时经 `S_WREQ` 补齐，内部受理以两通道均完成为准（§6.2）；tb 从设备请求通道恒 ready，单元验证不激发该路径。

---

## 10. 验证要点

验证以事务级等价为准（§1.6），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| M1 | 请求受理与仲裁 | 受理拍与来源和参考逐拍一致；两源同拍挂起 icache 优先；单在途 |
| M2 | AXI 请求格式 | `arlen=0`/`arsize=32B`/`awlen=0`/`awsize=4B`/INCR/ID=0/`wlast=1`；`wstrb` 与 `wdata` 按 `awaddr[4:2]` 定位；AXI 请求握手与内部受理同拍对应 |
| M3 | 读响应载荷 | 回填整行 8 字位精确（含指令段越界字为 0、段首/段尾行、非对齐行读） |
| M4 | 写通与写后读 | 写按 `wstrb` 字节选通提交；后续读返回已写数据（段尾最后字、`wsel` 0..7 全覆盖、数据极值） |
| M5 | 写应答 | `memif_l1sm_rsp` 载荷恒 0；消费拍与参考一致 |
| M6 | 错误注入 | SLVERR 注入拍事务序列不受影响（数据仍正确、锁步成立）；`memif_top_err` 呈现拍次拍置位并保持 |
| M7 | 协议 | `vld && !rdy` 期间载荷不变、`vld` 不撤（条 1）；复位期间全部 `vld` 恒 0、`memif_top_err = 0`；`rdy` 与在途状态一致 |
| M8 | 延迟任意 | `MEM_LAT` 取 0/1/2/3/5/8 下 M1–M7 成立；响应消费随机背压（恒 1、50%、20%） |
| M9 | 无死锁 | 全部事务在限界拍数内排空 |

参考模型：testbench（Verilator C++ harness）直接链接 `top/cmodel`（`memif_step`），扮演 tb 侧 AXI4 从设备（地址口径 §1.3、倒计时同构 §1.5、写按 `wstrb` 提交、SLVERR 注入）与内部侧双请求源/双响应消费者（随机背压），参考事务序列与 DUT 事务序列按拍在线比对。激励生成保证 icache 请求地址在指令段内、l1sm 请求地址在两段全局数据区内（读留 8 字余量，§1.3）。
