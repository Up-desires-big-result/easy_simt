# easy_simt 顶层接口规范

版本：v0.1（规范文档）
日期：2026-08-25
适用范围：本文档定义 easy_simt SIMT 处理器 v1 的**模块级接口**：端口命名规则、vld/rdy 握手协议、全部模块间通道的信号级定义、`memif` 对外 AXI4 总线约定与顶层信号。上游依据为同目录 `ma_spec_v0.1.md`（顶层微架构规范，其 §5 为语义级接口）与 `isa_spec_v0.1.md`（ISA 规范）。模块内部实现不在本文档范围；本文档是各模块端口定义文档与 `top` 连线的直接依据。

---

## 1. 通用约定

### 1.1 命名规则

模块间信号一律采用 **`源_宿_功能`** 三段前缀：

- `sf_bs_XXX`：sf 给到 bs 的信号；`bs_sf_XXX`：bs 给到 sf 的信号；其余模块对依此类推。
- 每条通道由一对 `*_vld` / `*_rdy` 与若干载荷信号组成；载荷信号同样携带该通道的 `源_宿_` 前缀。
- `vld` 恒由源模块驱动，`rdy` 恒由宿模块驱动，一一对应，不允许多驱动或扇出。

### 1.2 vld/rdy 协议

模块与模块之间的接口**统一**采用 vld/rdy 握手：

1. 点对点：单生产者、单消费者。
2. `vld` 与 `rdy` 同时为高的时钟上升沿发生一次传输。
3. `vld` 拉起后，源模块必须保持 `vld` 与全部载荷稳定，直至握手完成。
4. `vld` 不得组合依赖于 `rdy`（防死锁与组合环）；`rdy` 可以依赖 `vld`。
5. 复位（`rst_n=0`）期间所有 `vld=0`；接收侧 `rdy` 取值自定。
6. 通道间相互独立，背压沿链路自然传播，不设隐式信用。

### 1.3 时钟与复位

所有模块统一携带 `clk`（输入，上升沿有效）与 `rst_n`（输入，低电平有效异步复位）。`top` 只做互连与时钟/复位分发，自身无逻辑。

### 1.4 位宽参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `NLANES` | 8 | lane 数/warp |
| `NWARPS` | 4 | warp 数/块 |
| `REG_AW` | 5 | 寄存器地址位宽（32 个寄存器） |
| `OPCODE_W` | 5 | 操作码位宽（20 条指令） |
| `ADDR_W` | 32 | 地址/立即数/参数位宽 |
| `DATA_W` | 32 | 数据位宽 |
| `LINE_W` | 256 | 缓存行位宽（32B） |
| `REASON_W` | 3 | 停顿原因编码位宽 |
| `BRT_IW` | 2 | BRT 表项索引位宽（4 项） |
| `AXI_ID_W` | 4 | AXI ID 位宽（v1 恒置 0） |
| `AXI_ADDR_W` | 32 | AXI 地址位宽 |
| `AXI_DATA_W` | 256 | AXI 数据位宽（= 1 行） |
| `AXI_STRB_W` | 32 | AXI 字节选通位宽 |

### 1.5 停顿原因编码（`*_reason`）

| 编码 | 名称 | 含义 |
|---|---|---|
| 0 | NONE | 无停顿（清除） |
| 1 | HAZARD | 记分板互锁 |
| 2 | IMISS | 取指缺失 |
| 3 | LMISS | 数据访存未返回 |
| 4 | BARRIER | 已到达 `bar.sync` 等待释放 |
| 5 | BRSTALL | 分支决议中，取指阻塞 |
| 6 | DONE | 该 warp 已执行 `ret` |

---

## 2. 顶层信号

| 信号 | 位宽 | 方向 | 来源 | 说明 |
|---|---|---|---|---|
| `clk` | 1 | 输入 | — | 全片时钟 |
| `rst_n` | 1 | 输入 | — | 全片复位，低有效 |
| `axi_*` | — | 主设备 | memif | 对外 AXI4 全总线，见 §5 |
| `bs_top_done` | 1 | 输出 | bs | grid 结束（最后一块 `block_done` 后锁存为 1） |
| `sf_top_err` | 1 | 输出 | sf | 错误标志：非法指令、分化栈溢出（锁存） |
| `memif_top_err` | 1 | 输出 | memif | AXI 非 OKAY 响应（锁存） |

程序装入方式：程序镜像由 tb 预置于外部 AXI 存储，经 `icache→memif` 回填写入指令流，不设独立装载端口。

---

## 3. 连接矩阵

行为源、列为宿，格内为请求通道名（应答通道在 §4 中与请求通道成对定义）。

| 源 \ 宿 | ws | sf | bs | ialu | falu | lsu | icache | l1sm | memif | rf |
|---|---|---|---|---|---|---|---|---|---|---|
| bs | `bs_ws_launch` | `bs_sf_launch` | | | | | | | | |
| ws | | `ws_sf_grant` | `ws_bs_bdone` | | | | | | | |
| sf | `sf_ws_stall`、`sf_ws_bar` | | | `sf_ialu_issue` | `sf_falu_issue` | `sf_lsu_issue` | `sf_icache_req` | | | `sf_rf_rd` |
| ialu | | `ialu_sf_br`、`ialu_sf_wbdone` | | | | | | | | `ialu_rf_wb` |
| falu | | `falu_sf_wbdone` | | | | | | | | `falu_rf_wb` |
| lsu | `lsu_ws_stall` | `lsu_sf_wbdone` | | | | | | `lsu_l1sm_req` | | `lsu_rf_wb` |
| icache | | `icache_sf_rsp` | | | | | | | `icache_memif_req` | |
| l1sm | | | | | | `l1sm_lsu_rsp` | | | `l1sm_memif_req` | |
| memif | | | | | | | `memif_icache_rsp` | `memif_l1sm_rsp` | | |
| rf | | `rf_sf_rddata` | | | | | | | | |

与 `ma_spec` §5 的 18 条语义接口的映射见附录 B。

---

## 4. 通道定义（信号级）

方向列以"源→宿"表示。所有通道均含隐含的 `clk`/`rst_n`，下表不再重复列出。

### 4.1 块启动：`bs_sf_launch`（bs → sf）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `bs_sf_launch_vld` | 1 | bs→sf | 块启动请求 |
| `sf_bs_launch_rdy` | 1 | sf→bs | sf 已接收并复位本块各 warp 的 PC 与 SIMT 状态 |
| `bs_sf_launch_block_idx` | ADDR_W | bs→sf | 块号，供 `ld.param` |
| `bs_sf_launch_n` | ADDR_W | bs→sf | 参数 N，供 `ld.param` |
| `bs_sf_launch_shbase` | ADDR_W | bs→sf | 共享内存分区基址 |

### 4.2 块启动通告：`bs_ws_launch`（bs → ws）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `bs_ws_launch_vld` | 1 | bs→ws | 块启动通告 |
| `ws_bs_launch_rdy` | 1 | ws→bs | ws 已复位屏障计数与 warp 归属 |
| `bs_ws_launch_block_idx` | ADDR_W | bs→ws | 块号（屏障计数索引） |

`bs_sf_launch` 与 `bs_ws_launch` 同拍发起，两侧均握手成功后块启动完成。

### 4.3 块完成：`ws_bs_bdone`（ws → bs）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `ws_bs_bdone_vld` | 1 | ws→bs | 本块 4 个 warp 均已 `ret` |
| `bs_ws_bdone_rdy` | 1 | bs→ws | bs 已接收 |
| `ws_bs_bdone_block_idx` | ADDR_W | ws→bs | 完成的块号 |

### 4.4 发射授予：`ws_sf_grant`（ws → sf）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `ws_sf_grant_vld` | 1 | ws→sf | 本周期授予发射 |
| `sf_ws_grant_rdy` | 1 | sf→ws | sf 接受（0 表示该 warp 实际不可发射，ws 下周期换人） |
| `ws_sf_grant_warp_id` | 2 | ws→sf | 被授予的 warp |

正确性由本握手兜底：停顿通道（4.5/4.21）仅为调度提示，允许滞后一拍。

### 4.5 停顿上报：`sf_ws_stall`（sf → ws）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `sf_ws_stall_vld` | 1 | sf→ws | 某 warp 停顿状态变化 |
| `ws_sf_stall_rdy` | 1 | ws→sf | ws 已接收 |
| `sf_ws_stall_warp_id` | 2 | sf→ws | warp 号 |
| `sf_ws_stall_reason` | REASON_W | sf→ws | 新状态（含 NONE=清除） |

同拍多个 warp 状态变化时，sf 逐拍串行上报；上报窗口内调度偏差由 4.4 的 `grant_rdy` 兜底。

### 4.6 屏障到达：`sf_ws_bar`（sf → ws）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `sf_ws_bar_vld` | 1 | sf→ws | warp 发射 `bar.sync`（前提：该 warp LSU 已排空） |
| `ws_sf_bar_rdy` | 1 | ws→sf | ws 已接收 |
| `sf_ws_bar_warp_id` | 2 | sf→ws | warp 号 |
| `sf_ws_bar_block_id` | ADDR_W | sf→ws | 所属块号 |

### 4.7 取指请求：`sf_icache_req`（sf → icache）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `sf_icache_req_vld` | 1 | sf→icache | 取指请求（每周期至多一个） |
| `icache_sf_req_rdy` | 1 | sf←icache | icache 接受 |
| `sf_icache_req_pc` | ADDR_W | sf→icache | 取指地址 |

### 4.8 取指应答：`icache_sf_rsp`（icache → sf）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `icache_sf_rsp_vld` | 1 | icache→sf | 指令返回；缺失时阻塞至回填完成后才拉起 |
| `sf_icache_rsp_rdy` | 1 | sf→icache | sf 已接收 |
| `icache_sf_rsp_inst` | DATA_W | icache→sf | 32 位指令 |

等待本应答期间，sf 将相应 warp 记为 IMISS。

### 4.9 寄存器读请求：`sf_rf_rd`（sf → rf）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `sf_rf_rd_vld` | 1 | sf→rf | 读请求（ID 级，单在途） |
| `rf_sf_rd_rdy` | 1 | rf→sf | rf 接受 |
| `sf_rf_rd_warp_id` | 2 | sf→rf | warp 号 |
| `sf_rf_rd_rs1` | REG_AW | sf→rf | 源寄存器 1 |
| `sf_rf_rd_rs2` | REG_AW | sf→rf | 源寄存器 2 |

### 4.10 寄存器读应答：`rf_sf_rddata`（rf → sf）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `rf_sf_rddata_vld` | 1 | rf→sf | 读数据返回 |
| `sf_rf_rddata_rdy` | 1 | sf→rf | sf 已接收 |
| `rf_sf_rddata_a` | NLANES×DATA_W | rf→sf | rs1 的 8 lane 数据 |
| `rf_sf_rddata_b` | NLANES×DATA_W | rf→sf | rs2 的 8 lane 数据 |

应答与请求严格顺序对应（单在途），不回带地址。

### 4.11 整数发射：`sf_ialu_issue`（sf → ialu）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `sf_ialu_issue_vld` | 1 | sf→ialu | 发射请求 |
| `ialu_sf_issue_rdy` | 1 | ialu→sf | ialu 接受 |
| `sf_ialu_issue_opcode` | OPCODE_W | sf→ialu | 操作码 |
| `sf_ialu_issue_rd` | REG_AW | sf→ialu | 目的寄存器 |
| `sf_ialu_issue_warp_id` | 2 | sf→ialu | warp 号（随路） |
| `sf_ialu_issue_lane_mask` | NLANES | sf→ialu | 发射时 active mask 快照（随路至写回） |
| `sf_ialu_issue_pc` | ADDR_W | sf→ialu | 本指令 PC（分支用） |
| `sf_ialu_issue_opa` | NLANES×DATA_W | sf→ialu | 源操作数 A（per-lane） |
| `sf_ialu_issue_opb` | NLANES×DATA_W | sf→ialu | 源操作数 B（per-lane） |
| `sf_ialu_issue_imm` | ADDR_W | sf→ialu | 符号扩展立即数 |

### 4.12 浮点发射：`sf_falu_issue`（sf → falu）

信号集与 4.11 相同，但无 `pc` 字段（`sf_falu_issue_{vld,opcode,rd,warp_id,lane_mask,opa,opb}` + `falu_sf_issue_rdy`）。

### 4.13 访存发射：`sf_lsu_issue`（sf → lsu）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `sf_lsu_issue_vld` | 1 | sf→lsu | 发射请求 |
| `lsu_sf_issue_rdy` | 1 | lsu→sf | lsu 接受 |
| `sf_lsu_issue_opcode` | OPCODE_W | sf→lsu | 操作码（区分 ld/st、shmem/global） |
| `sf_lsu_issue_rd` | REG_AW | sf→lsu | 目的寄存器（仅装载有效） |
| `sf_lsu_issue_warp_id` | 2 | sf→lsu | warp 号（随路） |
| `sf_lsu_issue_lane_mask` | NLANES | sf→lsu | active mask 快照（门控 + 随路） |
| `sf_lsu_issue_opa` | NLANES×DATA_W | sf→lsu | 基址寄存器值（per-lane） |
| `sf_lsu_issue_imm` | ADDR_W | sf→lsu | 偏移（符号扩展） |
| `sf_lsu_issue_shbase` | ADDR_W | sf→lsu | 共享内存基址（来自块上下文） |

### 4.14 分支解析回注：`ialu_sf_br`（ialu → sf）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `ialu_sf_br_vld` | 1 | ialu→sf | EX 级分支解析结果（仅分支指令产生） |
| `sf_ialu_br_rdy` | 1 | sf→ialu | sf 已接收 |
| `ialu_sf_br_warp_id` | 2 | ialu→sf | warp 号 |
| `ialu_sf_br_taken` | NLANES | ialu→sf | 每 lane taken 向量 |
| `ialu_sf_br_target` | ADDR_W | ialu→sf | 跳转目标（绝对地址） |
| `ialu_sf_br_brt_idx` | BRT_IW | ialu→sf | BRT 表项索引 |

### 4.15 整数写回：`ialu_rf_wb`（ialu → rf）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `ialu_rf_wb_vld` | 1 | ialu→rf | 写回请求 |
| `rf_ialu_wb_rdy` | 1 | rf→ialu | rf 接受（三源仲裁，见 §6.1） |
| `ialu_rf_wb_warp_id` | 2 | ialu→rf | warp 号 |
| `ialu_rf_wb_rd` | REG_AW | ialu→rf | 目的寄存器（rd=0 时 rf 忽略） |
| `ialu_rf_wb_lane_mask` | NLANES | ialu→rf | 写使能（发射时快照随路） |
| `ialu_rf_wb_wdata` | NLANES×DATA_W | ialu→rf | 写数据（per-lane） |

### 4.16 浮点写回：`falu_rf_wb`（falu → rf）

信号集与 4.15 相同（`falu_rf_wb_*` / `rf_falu_wb_rdy`）。

### 4.17 访存写回：`lsu_rf_wb`（lsu → rf）

信号集与 4.15 相同（`lsu_rf_wb_*` / `rf_lsu_wb_rdy`）。**仅装载**使用本通道；存储不写 rf，其完成由 4.20 上报。

### 4.18 整数写回完成：`ialu_sf_wbdone`（ialu → sf）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `ialu_sf_wbdone_vld` | 1 | ialu→sf | 写回完成（记分板清除） |
| `sf_ialu_wbdone_rdy` | 1 | sf→ialu | sf 已接收 |
| `ialu_sf_wbdone_warp_id` | 2 | ialu→sf | warp 号 |
| `ialu_sf_wbdone_rd` | REG_AW | ialu→sf | 被清除的目的寄存器 |

### 4.19 浮点写回完成：`falu_sf_wbdone`（falu → sf）

信号集与 4.18 相同（`falu_sf_wbdone_*` / `sf_falu_wbdone_rdy`）。

### 4.20 访存写回完成：`lsu_sf_wbdone`（lsu → sf）

信号集与 4.18 相同（`lsu_sf_wbdone_*` / `sf_lsu_wbdone_rdy`）。装载与存储均上报；`rd` 字段对存储无意义（sf 按发射时登记的指令类型自行区分，用于 LSU 排空判定与记分板）。

### 4.21 访存停顿上报：`lsu_ws_stall`（lsu → ws）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `lsu_ws_stall_vld` | 1 | lsu→ws | 有全局访存请求未返回（置位）/ 全部返回（清除） |
| `ws_lsu_stall_rdy` | 1 | ws→lsu | ws 已接收 |
| `lsu_ws_stall_warp_id` | 2 | lsu→ws | warp 号 |
| `lsu_ws_stall_reason` | REASON_W | lsu→ws | 恒为 LMISS 或 NONE |

### 4.22 访存请求：`lsu_l1sm_req`（lsu → l1sm）

lane 串行：lsu 每拍至多发出一个 lane 的请求。

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `lsu_l1sm_req_vld` | 1 | lsu→l1sm | 请求有效 |
| `l1sm_lsu_req_rdy` | 1 | l1sm→lsu | l1sm 接受 |
| `lsu_l1sm_req_rw` | 1 | lsu→l1sm | 0=读，1=写 |
| `lsu_l1sm_req_sm` | 1 | lsu→l1sm | 1=共享内存，0=全局（经 L1） |
| `lsu_l1sm_req_addr` | ADDR_W | lsu→l1sm | 地址（shmem 已含 SHBASE） |
| `lsu_l1sm_req_wdata` | DATA_W | lsu→l1sm | 写数据（写时有效） |

### 4.23 访存应答：`l1sm_lsu_rsp`（l1sm → lsu）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `l1sm_lsu_rsp_vld` | 1 | l1sm→lsu | 应答有效；L1 缺失时阻塞至回填完成 |
| `lsu_l1sm_rsp_rdy` | 1 | lsu→l1sm | lsu 已接收 |
| `l1sm_lsu_rsp_rdata` | DATA_W | l1sm→lsu | 读数据（写应答时无效） |

读、写均有应答（写应答表示已交付写通路径），与请求顺序对应。

### 4.24 指令回填请求：`icache_memif_req`（icache → memif）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `icache_memif_req_vld` | 1 | icache→memif | 回填请求 |
| `memif_icache_req_rdy` | 1 | memif→icache | memif 接受（忙时为 0） |
| `icache_memif_req_addr` | ADDR_W | icache→memif | 行对齐地址 |

### 4.25 指令回填应答：`memif_icache_rsp`（memif → icache）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `memif_icache_rsp_vld` | 1 | memif→icache | 回填数据返回 |
| `icache_memif_rsp_rdy` | 1 | icache→memif | icache 已接收 |
| `memif_icache_rsp_data` | LINE_W | memif→icache | 整行 256 位 |

### 4.26 数据回填/写通请求：`l1sm_memif_req`（l1sm → memif）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `l1sm_memif_req_vld` | 1 | l1sm→memif | 请求有效 |
| `memif_l1sm_req_rdy` | 1 | memif→l1sm | memif 接受（忙时为 0） |
| `l1sm_memif_req_rw` | 1 | l1sm→memif | 0=读回填，1=写通（store） |
| `l1sm_memif_req_addr` | ADDR_W | l1sm→memif | 读为行对齐地址；写为字地址 |
| `l1sm_memif_req_wdata` | DATA_W | l1sm→memif | 写数据（写时有效） |

### 4.27 数据回填/写通应答：`memif_l1sm_rsp`（memif → l1sm）

| 信号 | 位宽 | 方向 | 说明 |
|---|---|---|---|
| `memif_l1sm_rsp_vld` | 1 | memif→l1sm | 应答有效（读：数据返回；写：AXI 写完成） |
| `l1sm_memif_rsp_rdy` | 1 | l1sm→memif | l1sm 已接收 |
| `memif_l1sm_rsp_data` | LINE_W | memif→l1sm | 整行数据（仅读有效） |

写通存储为阻塞式：store 在本应答返回后才算完成，保证 `ret`/`bar.sync` 语义下数据已离开本模块。

---

## 5. memif 对外 AXI4 接口

memif 为 **AXI4 主设备**，全五通道（AW/W/B/AR/R）。v1 约束：

1. 单在途事务：同一时刻至多一笔 AXI 事务；内部仲裁固定优先级 **icache 优先**于 l1sm。
2. `AXI_ID_W` 位 ID 恒置 0；`awburst/arburst = 2'b01`（INCR）。
3. 读（回填）：`arlen=8'h00`、`arsize=3'b101`（32B/拍），单拍整行。
4. 写（写通）：`awlen=8'h00`、`awsize=3'b010`（4B 窄传输），`wdata` 按 `awaddr[4:2]` 定位，`wstrb` 置对应 4 字节，其余为 0；`wlast=1`。
5. 响应：`rresp/bresp` 非 `2'b00`（OKAY）时置 `memif_top_err` 并保持（锁存）。
6. 片外延迟：v1 的 `MEM_LAT` 建模职责在 **tb 的 AXI 从设备存储模型**，memif 本身为桥接+仲裁，不额外加延迟。

信号表（全部为 memif 侧方向）：

| 通道 | 信号 | 位宽 | 方向 |
|---|---|---|---|
| AW | `axi_awid` | AXI_ID_W | 输出 |
| | `axi_awaddr` | AXI_ADDR_W | 输出 |
| | `axi_awlen` | 8 | 输出 |
| | `axi_awsize` | 3 | 输出 |
| | `axi_awburst` | 2 | 输出 |
| | `axi_awvalid` | 1 | 输出 |
| | `axi_awready` | 1 | 输入 |
| W | `axi_wdata` | AXI_DATA_W | 输出 |
| | `axi_wstrb` | AXI_STRB_W | 输出 |
| | `axi_wlast` | 1 | 输出 |
| | `axi_wvalid` | 1 | 输出 |
| | `axi_wready` | 1 | 输入 |
| B | `axi_bid` | AXI_ID_W | 输入 |
| | `axi_bresp` | 2 | 输入 |
| | `axi_bvalid` | 1 | 输入 |
| | `axi_bready` | 1 | 输出 |
| AR | `axi_arid` | AXI_ID_W | 输出 |
| | `axi_araddr` | AXI_ADDR_W | 输出 |
| | `axi_arlen` | 8 | 输出 |
| | `axi_arsize` | 3 | 输出 |
| | `axi_arburst` | 2 | 输出 |
| | `axi_arvalid` | 1 | 输出 |
| | `axi_arready` | 1 | 输入 |
| R | `axi_rid` | AXI_ID_W | 输入 |
| | `axi_rdata` | AXI_DATA_W | 输入 |
| | `axi_rresp` | 2 | 输入 |
| | `axi_rlast` | 1 | 输入 |
| | `axi_rvalid` | 1 | 输入 |
| | `axi_rready` | 1 | 输出 |

AXI 通道遵循 AXI4 协议（valid/ready、写序、响应规则），不受 §1.2 的"vld 不依赖 rdy"之外的额外约束；其 valid/ready 语义与本规范一致。

---

## 6. 与 `ma_spec` 的衔接说明

1. **rf 写控制并入写回通道**：`ma_spec` §5 接口 #9（sf→rf 写使能）取消。理由：lane_mask 快照已随 issue 包下发（§10），由执行单元随路带回（4.15–4.17），避免 sf 控制与单元数据两路在 rf 对齐；三源写口仲裁（固定优先级 `lsu > ialu > falu`）置于 rf 内部，`top` 保持无逻辑。
2. **rf 物理组织定标**：4 个 warp 并发（交织）执行要求各自独立的寄存器状态，rf 容量定为 `NWARPS × NLANES × 32 × DATA_W`，所有 rf 通道均携带 `warp_id`。
3. **MEM_LAT 落点**：引入 AXI 外接口后，片外延迟由 tb 的 AXI 从设备模型承担，`memif` 不再内置延迟模型；参数 `MEM_LAT` 移交 tb。
4. **停顿通道语义**：`sf_ws_stall`、`lsu_ws_stall` 为提示性状态同步，允许滞后；发射正确性由 `ws_sf_grant` 握手兜底（sf 以 `grant_rdy=0` 拒收不可发射的 warp）。

---

## 附录 A：模块端口总表

每个模块均含 `clk`（输入）与 `rst_n`（输入），下表不再重复。

### A.1 sf

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `bs_sf_launch_{vld,block_idx,n,shbase}` | 输入 | 1/32/32/32 | 4.1 |
| `sf_bs_launch_rdy` | 输出 | 1 | 4.1 |
| `ws_sf_grant_{vld,warp_id}` | 输入 | 1/2 | 4.4 |
| `sf_ws_grant_rdy` | 输出 | 1 | 4.4 |
| `sf_ws_stall_{vld,warp_id,reason}` | 输出 | 1/2/3 | 4.5 |
| `ws_sf_stall_rdy` | 输入 | 1 | 4.5 |
| `sf_ws_bar_{vld,warp_id,block_id}` | 输出 | 1/2/32 | 4.6 |
| `ws_sf_bar_rdy` | 输入 | 1 | 4.6 |
| `sf_icache_req_{vld,pc}` | 输出 | 1/32 | 4.7 |
| `icache_sf_req_rdy` | 输入 | 1 | 4.7 |
| `icache_sf_rsp_{vld,inst}` | 输入 | 1/32 | 4.8 |
| `sf_icache_rsp_rdy` | 输出 | 1 | 4.8 |
| `sf_rf_rd_{vld,warp_id,rs1,rs2}` | 输出 | 1/2/5/5 | 4.9 |
| `rf_sf_rd_rdy` | 输入 | 1 | 4.9 |
| `rf_sf_rddata_{vld,a,b}` | 输入 | 1/256/256 | 4.10 |
| `sf_rf_rddata_rdy` | 输出 | 1 | 4.10 |
| `sf_ialu_issue_*` | 输出 | 见 4.11 | 4.11 |
| `ialu_sf_issue_rdy` | 输入 | 1 | 4.11 |
| `sf_falu_issue_*` | 输出 | 见 4.12 | 4.12 |
| `falu_sf_issue_rdy` | 输入 | 1 | 4.12 |
| `sf_lsu_issue_*` | 输出 | 见 4.13 | 4.13 |
| `lsu_sf_issue_rdy` | 输入 | 1 | 4.13 |
| `ialu_sf_br_{vld,warp_id,taken,target,brt_idx}` | 输入 | 1/2/8/32/2 | 4.14 |
| `sf_ialu_br_rdy` | 输出 | 1 | 4.14 |
| `{ialu,falu,lsu}_sf_wbdone_{vld,warp_id,rd}` | 输入 | 1/2/5 | 4.18–4.20 |
| `sf_{ialu,falu,lsu}_wbdone_rdy` | 输出 | 1 | 4.18–4.20 |
| `sf_top_err` | 输出 | 1 | 顶层 |

### A.2 ws

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `bs_ws_launch_{vld,block_idx}` | 输入 | 1/32 | 4.2 |
| `ws_bs_launch_rdy` | 输出 | 1 | 4.2 |
| `ws_bs_bdone_{vld,block_idx}` | 输出 | 1/32 | 4.3 |
| `bs_ws_bdone_rdy` | 输入 | 1 | 4.3 |
| `ws_sf_grant_{vld,warp_id}` | 输出 | 1/2 | 4.4 |
| `sf_ws_grant_rdy` | 输入 | 1 | 4.4 |
| `sf_ws_stall_{vld,warp_id,reason}` | 输入 | 1/2/3 | 4.5 |
| `ws_sf_stall_rdy` | 输出 | 1 | 4.5 |
| `sf_ws_bar_{vld,warp_id,block_id}` | 输入 | 1/2/32 | 4.6 |
| `ws_sf_bar_rdy` | 输出 | 1 | 4.6 |
| `lsu_ws_stall_{vld,warp_id,reason}` | 输入 | 1/2/3 | 4.21 |
| `ws_lsu_stall_rdy` | 输出 | 1 | 4.21 |

### A.3 bs

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `bs_sf_launch_{vld,block_idx,n,shbase}` | 输出 | 1/32/32/32 | 4.1 |
| `sf_bs_launch_rdy` | 输入 | 1 | 4.1 |
| `bs_ws_launch_{vld,block_idx}` | 输出 | 1/32 | 4.2 |
| `ws_bs_launch_rdy` | 输入 | 1 | 4.2 |
| `ws_bs_bdone_{vld,block_idx}` | 输入 | 1/32 | 4.3 |
| `bs_ws_bdone_rdy` | 输出 | 1 | 4.3 |
| `bs_top_done` | 输出 | 1 | 顶层 |

### A.4 ialu

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_ialu_issue_*` | 输入 | 见 4.11 | 4.11 |
| `ialu_sf_issue_rdy` | 输出 | 1 | 4.11 |
| `ialu_sf_br_{vld,warp_id,taken,target,brt_idx}` | 输出 | 1/2/8/32/2 | 4.14 |
| `sf_ialu_br_rdy` | 输入 | 1 | 4.14 |
| `ialu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输出 | 1/2/5/8/256 | 4.15 |
| `rf_ialu_wb_rdy` | 输入 | 1 | 4.15 |
| `ialu_sf_wbdone_{vld,warp_id,rd}` | 输出 | 1/2/5 | 4.18 |
| `sf_ialu_wbdone_rdy` | 输入 | 1 | 4.18 |

### A.5 falu

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_falu_issue_*` | 输入 | 见 4.12 | 4.12 |
| `falu_sf_issue_rdy` | 输出 | 1 | 4.12 |
| `falu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输出 | 1/2/5/8/256 | 4.16 |
| `rf_falu_wb_rdy` | 输入 | 1 | 4.16 |
| `falu_sf_wbdone_{vld,warp_id,rd}` | 输出 | 1/2/5 | 4.19 |
| `sf_falu_wbdone_rdy` | 输入 | 1 | 4.19 |

### A.6 lsu

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_lsu_issue_*` | 输入 | 见 4.13 | 4.13 |
| `lsu_sf_issue_rdy` | 输出 | 1 | 4.13 |
| `lsu_l1sm_req_{vld,rw,sm,addr,wdata}` | 输出 | 1/1/1/32/32 | 4.22 |
| `l1sm_lsu_req_rdy` | 输入 | 1 | 4.22 |
| `l1sm_lsu_rsp_{vld,rdata}` | 输入 | 1/32 | 4.23 |
| `lsu_l1sm_rsp_rdy` | 输出 | 1 | 4.23 |
| `lsu_ws_stall_{vld,warp_id,reason}` | 输出 | 1/2/3 | 4.21 |
| `ws_lsu_stall_rdy` | 输入 | 1 | 4.21 |
| `lsu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输出 | 1/2/5/8/256 | 4.17 |
| `rf_lsu_wb_rdy` | 输入 | 1 | 4.17 |
| `lsu_sf_wbdone_{vld,warp_id,rd}` | 输出 | 1/2/5 | 4.20 |
| `sf_lsu_wbdone_rdy` | 输入 | 1 | 4.20 |

### A.7 icache

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_icache_req_{vld,pc}` | 输入 | 1/32 | 4.7 |
| `icache_sf_req_rdy` | 输出 | 1 | 4.7 |
| `icache_sf_rsp_{vld,inst}` | 输出 | 1/32 | 4.8 |
| `sf_icache_rsp_rdy` | 输入 | 1 | 4.8 |
| `icache_memif_req_{vld,addr}` | 输出 | 1/32 | 4.24 |
| `memif_icache_req_rdy` | 输入 | 1 | 4.24 |
| `memif_icache_rsp_{vld,data}` | 输入 | 1/256 | 4.25 |
| `icache_memif_rsp_rdy` | 输出 | 1 | 4.25 |

### A.8 l1sm

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `lsu_l1sm_req_{vld,rw,sm,addr,wdata}` | 输入 | 1/1/1/32/32 | 4.22 |
| `l1sm_lsu_req_rdy` | 输出 | 1 | 4.22 |
| `l1sm_lsu_rsp_{vld,rdata}` | 输出 | 1/32 | 4.23 |
| `lsu_l1sm_rsp_rdy` | 输入 | 1 | 4.23 |
| `l1sm_memif_req_{vld,rw,addr,wdata}` | 输出 | 1/1/32/32 | 4.26 |
| `memif_l1sm_req_rdy` | 输入 | 1 | 4.26 |
| `memif_l1sm_rsp_{vld,data}` | 输入 | 1/256 | 4.27 |
| `l1sm_memif_rsp_rdy` | 输出 | 1 | 4.27 |

### A.9 memif

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `icache_memif_req_{vld,addr}` | 输入 | 1/32 | 4.24 |
| `memif_icache_req_rdy` | 输出 | 1 | 4.24 |
| `memif_icache_rsp_{vld,data}` | 输出 | 1/256 | 4.25 |
| `icache_memif_rsp_rdy` | 输入 | 1 | 4.25 |
| `l1sm_memif_req_{vld,rw,addr,wdata}` | 输入 | 1/1/32/32 | 4.26 |
| `memif_l1sm_req_rdy` | 输出 | 1 | 4.26 |
| `memif_l1sm_rsp_{vld,data}` | 输出 | 1/256 | 4.27 |
| `l1sm_memif_rsp_rdy` | 输入 | 1 | 4.27 |
| `axi_*` | 见 §5 | — | AXI4 |
| `memif_top_err` | 输出 | 1 | 顶层 |

### A.10 rf

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_rf_rd_{vld,warp_id,rs1,rs2}` | 输入 | 1/2/5/5 | 4.9 |
| `rf_sf_rd_rdy` | 输出 | 1 | 4.9 |
| `rf_sf_rddata_{vld,a,b}` | 输出 | 1/256/256 | 4.10 |
| `sf_rf_rddata_rdy` | 输入 | 1 | 4.10 |
| `ialu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输入 | 1/2/5/8/256 | 4.15 |
| `rf_ialu_wb_rdy` | 输出 | 1 | 4.15 |
| `falu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输入 | 1/2/5/8/256 | 4.16 |
| `rf_falu_wb_rdy` | 输出 | 1 | 4.16 |
| `lsu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输入 | 1/2/5/8/256 | 4.17 |
| `rf_lsu_wb_rdy` | 输出 | 1 | 4.17 |

---

## 附录 B：与 `ma_spec` §5 语义接口的映射

| ma_spec §5 # | 语义接口 | 本文档通道 |
|---|---|---|
| 1 | block_launch（bs→sf,ws） | 4.1 `bs_sf_launch`、4.2 `bs_ws_launch` |
| 2 | block_done（ws→bs） | 4.3 `ws_bs_bdone` |
| 3 | grant（ws→sf） | 4.4 `ws_sf_grant` |
| 4 | stall_reason（sf→ws） | 4.5 `sf_ws_stall` |
| 5 | barrier_arrive（sf→ws） | 4.6 `sf_ws_bar` |
| 6 | fetch（sf→icache） | 4.7 `sf_icache_req` |
| 7 | fetch_resp（icache→sf） | 4.8 `icache_sf_rsp` |
| 8 | rf_read（sf→rf） | 4.9 `sf_rf_rd` + 4.10 `rf_sf_rddata` |
| 9 | rf_wctrl（sf→rf） | 取消，并入 4.15–4.17（见 §6.1） |
| 10 | issue（sf→ialu/falu/lsu） | 4.11 / 4.12 / 4.13 |
| 11 | branch_res（ialu→sf） | 4.14 `ialu_sf_br` |
| 12 | wb（单元→rf） | 4.15 / 4.16 / 4.17 |
| 13 | wb_done（单元→sf） | 4.18 / 4.19 / 4.20 |
| 14 | lsu_stall（lsu→ws） | 4.21 `lsu_ws_stall` |
| 15 | mem_req（lsu→l1sm） | 4.22 `lsu_l1sm_req` |
| 16 | mem_resp（l1sm→lsu） | 4.23 `l1sm_lsu_rsp` |
| 17 | refill_req（icache/l1sm→memif） | 4.24 / 4.26 |
| 18 | refill_data（memif→icache/l1sm） | 4.25 / 4.27 |

---

## 附录 C：一致性自查

1. 每条通道的 `*_vld` 与 `*_rdy` 恰好一对，分属源、宿两个模块，附录 A 中两侧端口一一对应。
2. 连接矩阵（§3）中每个非空格对应 §4 中唯一通道定义；无孤立端口（除 `clk`/`rst_n`/顶层观测信号外）。
3. `ma_spec` §5 的 18 条语义接口全部落位（附录 B），其中 #9 经论证取消（§6.1），不产生缺失。
4. 三源写回（4.15–4.17）与写回完成（4.18–4.20）成对出现，覆盖全部执行单元。
