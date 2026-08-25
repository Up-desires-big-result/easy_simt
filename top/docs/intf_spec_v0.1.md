# easy_simt 顶层接口规范

版本：v0.1（规范文档；v0.2 重构：宏观内容收拢至第 1 节，模块逐节成文）
日期：2026-08-25
适用范围：本文档定义 easy_simt SIMT 处理器 v1 的**模块级接口**：端口命名规则、vld/rdy 握手协议、全部模块间通道的信号级定义、`memif` 对外 AXI4 总线约定与顶层信号。上游依据为同目录 `ma_spec_v0.1.md`（顶层微架构规范）与 `isa_spec_v0.1.md`（ISA 规范）。本文档是各模块端口定义文档与 `top` 连线的直接依据。

---

## 1. 总体约定

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

### 1.6 顶层信号

| 信号 | 位宽 | 方向 | 来源 | 说明 |
|---|---|---|---|---|
| `clk` | 1 | 输入 | — | 全片时钟 |
| `rst_n` | 1 | 输入 | — | 全片复位，低有效 |
| `axi_*` | — | 主设备 | memif | 对外 AXI4 全总线，见 §10 |
| `bs_top_done` | 1 | 输出 | bs | grid 结束（最后一块 `block_done` 后锁存为 1） |
| `sf_top_err` | 1 | 输出 | sf | 错误标志：非法指令、分化栈溢出（锁存） |
| `memif_top_err` | 1 | 输出 | memif | AXI 非 OKAY 响应（锁存） |

程序装入方式：程序镜像由 tb 预置于外部 AXI 存储，经 `icache→memif` 回填写入指令流，不设独立装载端口。

### 1.7 连接矩阵

行为源、列为宿，格内为请求通道名（应答通道在 §2–§11 中与请求通道成对定义）。

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

### 1.8 与 ma_spec 语义接口的映射

| ma_spec 语义接口 | 本文档通道 |
|---|---|
| block_launch（bs→sf,ws） | `bs_sf_launch`、`bs_ws_launch` |
| block_done（ws→bs） | `ws_bs_bdone` |
| grant（ws→sf） | `ws_sf_grant` |
| stall_reason（sf→ws） | `sf_ws_stall` |
| barrier_arrive（sf→ws） | `sf_ws_bar` |
| fetch / fetch_resp（sf↔icache） | `sf_icache_req` / `icache_sf_rsp` |
| rf_read / 读应答（sf↔rf） | `sf_rf_rd` / `rf_sf_rddata` |
| rf_wctrl（sf→rf） | 取消，并入 `ialu_rf_wb`/`falu_rf_wb`/`lsu_rf_wb`（lane_mask 随路，见 §6 说明） |
| issue（sf→ialu/falu/lsu） | `sf_ialu_issue` / `sf_falu_issue` / `sf_lsu_issue` |
| branch_res（ialu→sf） | `ialu_sf_br` |
| wb（单元→rf） | `ialu_rf_wb` / `falu_rf_wb` / `lsu_rf_wb` |
| wb_done（单元→sf） | `ialu_sf_wbdone` / `falu_sf_wbdone` / `lsu_sf_wbdone` |
| lsu_stall（lsu→ws） | `lsu_ws_stall` |
| mem_req / mem_resp（lsu↔l1sm） | `lsu_l1sm_req` / `l1sm_lsu_rsp` |
| refill_req（icache/l1sm→memif） | `icache_memif_req` / `l1sm_memif_req` |
| refill_data（memif→icache/l1sm） | `memif_icache_rsp` / `memif_l1sm_rsp` |

### 1.9 一致性自查

1. 每条通道的 `*_vld` 与 `*_rdy` 恰好一对，分属源、宿两个模块，§2–§11 中两侧端口一一对应。
2. 连接矩阵（§1.7）中每个非空格对应唯一通道定义；无孤立端口（除 `clk`/`rst_n`/顶层观测信号外）。
3. ma_spec 语义接口全部落位（§1.8），其中 rf_wctrl 经论证取消，不产生缺失。
4. 三源写回（`*_rf_wb`）与写回完成（`*_sf_wbdone`）成对出现，覆盖全部执行单元。

---

## 2. sf — SIMT Frontend 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `bs_sf_launch_{vld,block_idx,n,shbase}` | 输入 | 1/32/32/32 | bs_sf_launch |
| `sf_bs_launch_rdy` | 输出 | 1 | bs_sf_launch |
| `ws_sf_grant_{vld,warp_id}` | 输入 | 1/2 | ws_sf_grant |
| `sf_ws_grant_rdy` | 输出 | 1 | ws_sf_grant |
| `sf_ws_stall_{vld,warp_id,reason}` | 输出 | 1/2/3 | sf_ws_stall |
| `ws_sf_stall_rdy` | 输入 | 1 | sf_ws_stall |
| `sf_ws_bar_{vld,warp_id,block_id}` | 输出 | 1/2/32 | sf_ws_bar |
| `ws_sf_bar_rdy` | 输入 | 1 | sf_ws_bar |
| `sf_icache_req_{vld,pc}` | 输出 | 1/32 | sf_icache_req |
| `icache_sf_req_rdy` | 输入 | 1 | sf_icache_req |
| `icache_sf_rsp_{vld,inst}` | 输入 | 1/32 | icache_sf_rsp |
| `sf_icache_rsp_rdy` | 输出 | 1 | icache_sf_rsp |
| `sf_rf_rd_{vld,warp_id,rs1,rs2}` | 输出 | 1/2/5/5 | sf_rf_rd |
| `rf_sf_rd_rdy` | 输入 | 1 | sf_rf_rd |
| `rf_sf_rddata_{vld,a,b}` | 输入 | 1/256/256 | rf_sf_rddata |
| `sf_rf_rddata_rdy` | 输出 | 1 | rf_sf_rddata |
| `sf_ialu_issue_*` | 输出 | 见下 | sf_ialu_issue |
| `ialu_sf_issue_rdy` | 输入 | 1 | sf_ialu_issue |
| `sf_falu_issue_*` | 输出 | 见下 | sf_falu_issue |
| `falu_sf_issue_rdy` | 输入 | 1 | sf_falu_issue |
| `sf_lsu_issue_*` | 输出 | 见下 | sf_lsu_issue |
| `lsu_sf_issue_rdy` | 输入 | 1 | sf_lsu_issue |
| `ialu_sf_br_{vld,warp_id,taken,target,brt_idx}` | 输入 | 1/2/8/32/2 | ialu_sf_br |
| `sf_ialu_br_rdy` | 输出 | 1 | ialu_sf_br |
| `{ialu,falu,lsu}_sf_wbdone_{vld,warp_id,rd}` | 输入 | 1/2/5 | *_sf_wbdone |
| `sf_{ialu,falu,lsu}_wbdone_rdy` | 输出 | 1 | *_sf_wbdone |
| `sf_top_err` | 输出 | 1 | 顶层 |

issue 载荷（sf 为源）：`sf_ialu_issue_{opcode,rd,warp_id,lane_mask,pc,opa,opb,imm}`；`sf_falu_issue_{opcode,rd,warp_id,lane_mask,opa,opb}`；`sf_lsu_issue_{opcode,rd,warp_id,lane_mask,opa,imm,shbase}`。其中 `opa/opb` 为 NLANES×DATA_W，`lane_mask` 为发射时 active mask 快照、随路至写回。`sf_ws_stall`/`sf_ws_bar` 为提示性状态同步，正确性由 `ws_sf_grant` 握手兜底（sf 以 `grant_rdy=0` 拒收不可发射的 warp）。

## 3. ws — Warp Scheduler 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `bs_ws_launch_{vld,block_idx}` | 输入 | 1/32 | bs_ws_launch |
| `ws_bs_launch_rdy` | 输出 | 1 | bs_ws_launch |
| `ws_bs_bdone_{vld,block_idx}` | 输出 | 1/32 | ws_bs_bdone |
| `bs_ws_bdone_rdy` | 输入 | 1 | ws_bs_bdone |
| `ws_sf_grant_{vld,warp_id}` | 输出 | 1/2 | ws_sf_grant |
| `sf_ws_grant_rdy` | 输入 | 1 | ws_sf_grant |
| `sf_ws_stall_{vld,warp_id,reason}` | 输入 | 1/2/3 | sf_ws_stall |
| `ws_sf_stall_rdy` | 输出 | 1 | sf_ws_stall |
| `sf_ws_bar_{vld,warp_id,block_id}` | 输入 | 1/2/32 | sf_ws_bar |
| `ws_sf_bar_rdy` | 输出 | 1 | sf_ws_bar |
| `lsu_ws_stall_{vld,warp_id,reason}` | 输入 | 1/2/3 | lsu_ws_stall |
| `ws_lsu_stall_rdy` | 输出 | 1 | lsu_ws_stall |

## 4. bs — Block Scheduler 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `bs_sf_launch_{vld,block_idx,n,shbase}` | 输出 | 1/32/32/32 | bs_sf_launch |
| `sf_bs_launch_rdy` | 输入 | 1 | bs_sf_launch |
| `bs_ws_launch_{vld,block_idx}` | 输出 | 1/32 | bs_ws_launch |
| `ws_bs_launch_rdy` | 输入 | 1 | bs_ws_launch |
| `ws_bs_bdone_{vld,block_idx}` | 输入 | 1/32 | ws_bs_bdone |
| `bs_ws_bdone_rdy` | 输出 | 1 | ws_bs_bdone |
| `bs_top_done` | 输出 | 1 | 顶层 |

`bs_sf_launch` 与 `bs_ws_launch` 同拍发起，两侧均握手成功后块启动完成。

## 5. ialu — Integer ALU 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_ialu_issue_*` | 输入 | 见 §2 | sf_ialu_issue |
| `ialu_sf_issue_rdy` | 输出 | 1 | sf_ialu_issue |
| `ialu_sf_br_{vld,warp_id,taken,target,brt_idx}` | 输出 | 1/2/8/32/2 | ialu_sf_br |
| `sf_ialu_br_rdy` | 输入 | 1 | ialu_sf_br |
| `ialu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输出 | 1/2/5/8/256 | ialu_rf_wb |
| `rf_ialu_wb_rdy` | 输入 | 1 | ialu_rf_wb |
| `ialu_sf_wbdone_{vld,warp_id,rd}` | 输出 | 1/2/5 | ialu_sf_wbdone |
| `sf_ialu_wbdone_rdy` | 输入 | 1 | ialu_sf_wbdone |

## 6. falu — Floating-point ALU 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_falu_issue_*` | 输入 | 见 §2 | sf_falu_issue |
| `falu_sf_issue_rdy` | 输出 | 1 | sf_falu_issue |
| `falu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输出 | 1/2/5/8/256 | falu_rf_wb |
| `rf_falu_wb_rdy` | 输入 | 1 | falu_rf_wb |
| `falu_sf_wbdone_{vld,warp_id,rd}` | 输出 | 1/2/5 | falu_sf_wbdone |
| `sf_falu_wbdone_rdy` | 输入 | 1 | falu_sf_wbdone |

**写回通道说明（ialu/falu/lsu 共用）**：ma_spec 原 `sf→rf 写使能` 语义接口取消，lane_mask 改由 issue 包快照、随路带回，故写使能并入各 `*_rf_wb` 的 `lane_mask` 字段。三源写口仲裁（固定优先级 `lsu > ialu > falu`）置于 rf 内部，`top` 保持无逻辑。`lsu_rf_wb` 仅装载使用；存储不写 rf，其完成由 `lsu_sf_wbdone` 上报。

## 7. lsu — Load/Store Unit 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_lsu_issue_*` | 输入 | 见 §2 | sf_lsu_issue |
| `lsu_sf_issue_rdy` | 输出 | 1 | sf_lsu_issue |
| `lsu_l1sm_req_{vld,rw,sm,addr,wdata,mask}` | 输出 | 1/1/1/256/256/8 | lsu_l1sm_req |
| `l1sm_lsu_req_rdy` | 输入 | 1 | lsu_l1sm_req |
| `l1sm_lsu_rsp_{vld,rdata}` | 输入 | 1/256 | l1sm_lsu_rsp |
| `lsu_l1sm_rsp_rdy` | 输出 | 1 | l1sm_lsu_rsp |
| `lsu_ws_stall_{vld,warp_id,reason}` | 输出 | 1/2/3 | lsu_ws_stall |
| `ws_lsu_stall_rdy` | 输入 | 1 | lsu_ws_stall |
| `lsu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输出 | 1/2/5/8/256 | lsu_rf_wb |
| `rf_lsu_wb_rdy` | 输入 | 1 | lsu_rf_wb |
| `lsu_sf_wbdone_{vld,warp_id,rd}` | 输出 | 1/2/5 | lsu_sf_wbdone |
| `sf_lsu_wbdone_rdy` | 输入 | 1 | lsu_sf_wbdone |

`lsu_l1sm_req`：8-lane 锁步，一拍一个 8-lane 请求（addr/wdata 各 8×32b，mask 门控活跃 lane）；`rw` 0=读 1=写，`sm` 1=共享 0=全局；shmem 地址已含 SHBASE。`l1sm_lsu_rsp` 一拍返回 8-lane 读数据（单行单拍）；跨行/bank 冲突/缺失时整拍停顿。写应答表示已交付写通路径。

## 8. icache — Instruction Cache 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_icache_req_{vld,pc}` | 输入 | 1/32 | sf_icache_req |
| `icache_sf_req_rdy` | 输出 | 1 | sf_icache_req |
| `icache_sf_rsp_{vld,inst}` | 输出 | 1/32 | icache_sf_rsp |
| `sf_icache_rsp_rdy` | 输入 | 1 | icache_sf_rsp |
| `icache_memif_req_{vld,addr}` | 输出 | 1/32 | icache_memif_req |
| `memif_icache_req_rdy` | 输入 | 1 | icache_memif_req |
| `memif_icache_rsp_{vld,data}` | 输入 | 1/256 | memif_icache_rsp |
| `icache_memif_rsp_rdy` | 输出 | 1 | memif_icache_rsp |

`icache_sf_rsp` 命中一拍返回；缺失时阻塞至回填完成后才拉起 `vld`，等待期间 sf 将相应 warp 记为 IMISS。

## 9. l1sm — L1 + Shared Memory 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `lsu_l1sm_req_{vld,rw,sm,addr,wdata,mask}` | 输入 | 1/1/1/256/256/8 | lsu_l1sm_req |
| `l1sm_lsu_req_rdy` | 输出 | 1 | lsu_l1sm_req |
| `l1sm_lsu_rsp_{vld,rdata}` | 输出 | 1/256 | l1sm_lsu_rsp |
| `lsu_l1sm_rsp_rdy` | 输入 | 1 | l1sm_lsu_rsp |
| `l1sm_memif_req_{vld,rw,addr,wdata}` | 输出 | 1/1/32/32 | l1sm_memif_req |
| `memif_l1sm_req_rdy` | 输入 | 1 | l1sm_memif_req |
| `memif_l1sm_rsp_{vld,data}` | 输入 | 1/256 | memif_l1sm_rsp |
| `l1sm_memif_rsp_rdy` | 输出 | 1 | memif_l1sm_rsp |

`l1sm_memif_req`：`rw` 0=读回填（addr 行对齐），1=写通（addr 字地址、`wdata` 有效）。`memif_l1sm_rsp` 读返回整行、写表示 AXI 写完成。

## 10. memif — Memory Interface 端口（含 AXI4）

内部侧：

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `icache_memif_req_{vld,addr}` | 输入 | 1/32 | icache_memif_req |
| `memif_icache_req_rdy` | 输出 | 1 | icache_memif_req |
| `memif_icache_rsp_{vld,data}` | 输出 | 1/256 | memif_icache_rsp |
| `icache_memif_rsp_rdy` | 输入 | 1 | memif_icache_rsp |
| `l1sm_memif_req_{vld,rw,addr,wdata}` | 输入 | 1/1/32/32 | l1sm_memif_req |
| `memif_l1sm_req_rdy` | 输出 | 1 | l1sm_memif_req |
| `memif_l1sm_rsp_{vld,data}` | 输出 | 1/256 | memif_l1sm_rsp |
| `l1sm_memif_rsp_rdy` | 输入 | 1 | memif_l1sm_rsp |
| `memif_top_err` | 输出 | 1 | 顶层 |

对外 AXI4（memif 为主设备，全五通道）：

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

v1 约束：单在途事务；内部仲裁固定优先级 **icache 优先**；ID 恒置 0；`awburst/arburst=2'b01`（INCR）。读（回填）：`arlen=0`、`arsize=3'b101`（32B/拍），单拍整行。写（写通）：`awlen=0`、`awsize=3'b010`（4B 窄传），`wdata` 按 `awaddr[4:2]` 定位、`wstrb` 置对应 4 字节，等 BRESP 返回才算完成。`rresp/bresp` 非 OKAY 置 `memif_top_err` 并保持。片外延迟 `MEM_LAT` 由 tb 的 AXI 从设备建模，memif 不额外加延迟。

## 11. rf — Register File 端口

| 端口 | I/O | 位宽 | 所属通道 |
|---|---|---|---|
| `sf_rf_rd_{vld,warp_id,rs1,rs2}` | 输入 | 1/2/5/5 | sf_rf_rd |
| `rf_sf_rd_rdy` | 输出 | 1 | sf_rf_rd |
| `rf_sf_rddata_{vld,a,b}` | 输出 | 1/256/256 | rf_sf_rddata |
| `sf_rf_rddata_rdy` | 输入 | 1 | rf_sf_rddata |
| `ialu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输入 | 1/2/5/8/256 | ialu_rf_wb |
| `rf_ialu_wb_rdy` | 输出 | 1 | ialu_rf_wb |
| `falu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输入 | 1/2/5/8/256 | falu_rf_wb |
| `rf_falu_wb_rdy` | 输出 | 1 | falu_rf_wb |
| `lsu_rf_wb_{vld,warp_id,rd,lane_mask,wdata}` | 输入 | 1/2/5/8/256 | lsu_rf_wb |
| `rf_lsu_wb_rdy` | 输出 | 1 | lsu_rf_wb |

读应答与请求严格顺序对应（单在途），不回带地址。`rd=0` 时 rf 忽略写（R0 恒零）。三源写口仲裁 `lsu > ialu > falu` 置于 rf 内部。
