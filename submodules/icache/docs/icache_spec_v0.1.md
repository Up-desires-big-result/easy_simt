# icache — Instruction Cache 模块规范

版本：v0.1
日期：2026-09-11
依据文档：`top/docs/ma_spec_v0.1.md`（§1.2、§1.6、§8）、`top/docs/intf_spec_v0.1.md`（§1、§8、§10）、`top/cmodel/icache.c`、`top/cmodel/sim_common.h`。
适用范围：本文档定义 icache（Instruction Cache）的模块级设计：端口、参数、内部状态、数据通路、控制逻辑、状态机、通道时序与协议约束，是 `icache/rtl/icache.sv` 实现与 `icache/tb/` 验证的直接依据。
修订记录：2026-09-11 初版，结构沿用模块级规范骨架（bs_spec/ialu_spec/falu_spec/rf_spec/lsu_spec/memif_spec 先例）。

---

## 1. 总体

### 1.1 模块定位与职责

icache 是直接映射指令缓存（ma_spec §8），职责：

- tag 比较与行阵列归本模块：32B 行 = 8 条指令（`ILINE_WORDS=8`），默认 16 行 512B（`ICACHE_LINES=16`，黄金程序静态 50 条=200B，留裕量）；
- 命中返回：受理取指请求，命中时受理拍+1 返回指令字；
- 缺失阻塞：缺失期间取指请求挂起（sf 侧将相应 warp 记为 IMISS），经 memif 回填整行后返回指令；
- 无预取、无无效化（程序只读，上电后内容不变，ma_spec §8）。

边界：不含片外访问（memif 职责，ma_spec §10）；不含取指请求生成与 IMISS 停顿记录（sf 职责，ma_spec §2）；回填数据为 256b 单拍整行交付（memif 回填口径，intf_spec §10）。

### 1.2 通道与事务路径

| 通道 | 方向 | 事务路径 |
|---|---|---|
| `sf_icache_req` → `icache_sf_rsp` | sf → icache → sf | 取指请求握手 → 命中：受理拍+1 呈现指令；缺失：回填完成后呈现指令 |
| `icache_memif_req` → `memif_icache_rsp` | icache → memif → icache | 回填请求握手（行对齐字节地址）→ 从设备延迟 → 整行数据握手 |

### 1.3 取指语义与地址口径

- pc 为字索引（字节地址 >> 2）：行 tag = pc >> log2(ILINE_WORDS)，行索引 = tag mod ICACHE_LINES（直接映射），行内字偏移 = pc & (ILINE_WORDS-1)；
- 回填请求地址 = (pc & ~(ILINE_WORDS-1)) << 2，即行对齐字节地址（C 模型同式）；
- 回填数据 256b = 8 字整行；缺失行整行覆盖分配（直接映射替换），`tag`/`valid` 同拍更新；
- 指令段哪些地址返回什么由 memif 侧实现（memif_spec §1.3 口径：按字索引 `imem`，超出程序长度 `imem_n` 的字返回 0）。本模块不检测地址范围。

### 1.4 时序模型

- 四态状态机（§6）：`S_IDLE` 受理取指请求并组合查找 tag/data，命中锁存指令转 `S_RSP`、缺失锁存 pc 转 `S_REQ`；
- 请求受理与 tag 比较同拍（组合查找）；`icache_memif_req_vld`/`icache_sf_rsp_vld` 均为状态译码，不组合依赖于本通道 `rdy`；
- 单事务在途：缺失在途或响应在途期间 `icache_sf_req_rdy = 0`；
- `icache_memif_rsp_rdy = (state == S_REFILL)`：等待回填数据期间恒就绪，呈现即握手；
- 片外固定延迟 `MEM_LAT` 归 tb 侧从设备（intf_spec §10；与 memif_spec §1.5 同构：请求握手拍末沿装载倒计时、每拍递减、计数到 0 呈现响应，呈现拍 = 请求握手拍 + MEM_LAT + 1，含 MEM_LAT=0），本模块自身不加延迟；
- 复位（低电平有效异步）期间全部输出 `vld = 0`；
- 模块间握手统一遵循 intf_spec §1.2 的 vld/rdy 协议。

### 1.5 与 C 模型的对应关系

`top/cmodel/icache.c` 的 `icache_step()` 为事务级参考，其语义与本文档的对应：

| C 模型（`icache_t` / `icache_step`） | icache RTL + tb 从设备 |
|---|---|
| `miss`（缺失在途） | `state == S_REQ/S_REFILL` |
| `rsp_pending`（指令待回送） | `state == S_RSP` |
| `miss_pc` | `miss_pc_q` |
| `req_sent`（回填请求已置位待消费） | `S_REQ` 期间 `icache_memif_req_vld` 保持至握手 |
| `rsp_sent`（响应已置位待消费） | `S_RSP` 期间 `icache_sf_rsp_vld` 保持至握手 |
| `rsp_inst` | `rsp_inst_q` |
| 命中：受理步内查阵列、置 `rsp_pending` | `S_IDLE` 握手拍组合查找，命中锁存 `rsp_inst_q` 转 `S_RSP` |
| 缺失：受理步置 `miss` | `S_IDLE` 握手拍锁存 `miss_pc_q` 转 `S_REQ` |
| 响应置位步（受理步/回填步+1 呈现指令） | `S_RSP` 呈现拍 |
| 响应清除步（消费检出，同步可受理新请求） | `S_RSP` 握手拍，次拍 `S_IDLE` 受理新请求 |
| 回填请求置位步（受理步+1） | `S_REQ` 呈现拍（受理拍+1） |
| 回填步（消费 rsp、写行、置 `rsp_pending`） | `S_REFILL` 握手拍（写行、锁存指令） |
| `data`/`tag`/`valid` 阵列 | 同名阵列（§4） |
| memif countdown（memif_step，呈现=受理+MEM_LAT+1） | tb 从设备倒计时（memif_spec §1.5 同构） |

顺序语义说明：`icache_step` 三段顺序执行（响应段 → 缺失段 → 请求接收段）；`miss` 与 `rsp_pending` 置位路径互斥；响应清除步与请求受理步可同拍发生（清除后请求接收段同步受理），对应 RTL 的 `S_RSP` 握手次拍 `S_IDLE` 受理新请求。testbench 参考推进顺序为「置激励（取指 pc / 从设备整行）→ `icache_step` → 按同拍 `rdy` 消费」，取指请求受理、指令消费、回填请求呈现、回填数据消费四类事件逐拍比对（§10）。

### 1.6 验收口径

功能验收为事务级等价（ma_spec §1.7：周期只作观测项，不作验收项）：

- 取指请求受理拍与参考逐拍一致；
- 指令消费拍与指令字位精确（命中、回填两路径）；
- 回填请求呈现/受理拍与地址位精确（行对齐字节地址）；
- 回填数据消费拍一致，回填落阵列后同行命中返回一致（含冲突替换、指令段越界字为 0）；
- 缺失/响应在途期间不接受新取指请求（IMISS 窗口）；
- 握手不变量：`vld` 保持期载荷稳定；复位期间全部输出 `vld = 0`；`rdy` 与状态一致（§1.4）；
- `MEM_LAT` 取 0/1/2/3/5/8 与随机背压下上述各项成立。

---

## 2. 端口

端口命名、方向与位宽见 intf_spec §8（8 端口：`sf_icache_req` / `icache_sf_rsp` / `icache_memif_req` / `memif_icache_rsp` 四通道各 vld+载荷+rdy）。`clk`/`rst_n` 按 intf_spec §1.3 携带。无模块级增设端口。

模块补充（行为约束，详见 §9）：

- `icache_sf_req_rdy = (state == S_IDLE)`：空闲即就绪，不组合依赖于请求 `vld` 以外的状态；
- `icache_memif_rsp_rdy = (state == S_REFILL)`：等待回填期间恒就绪（呈现即握手）；
- `icache_memif_req_vld` / `icache_sf_rsp_vld` 为状态译码，不组合依赖于本通道 `rdy`。

---

## 3. 参数与配置

| 参数 | 基线值 | 含义 |
|---|---|---|
| `ICACHE_LINES` | 16 | 行数（512B，ma_spec §1.6） |
| `ILINE_WORDS` | 8 | 每行字数（32B 行 = 8 条指令，ma_spec §1.6） |

派生量：`LINE_W = ILINE_WORDS×32`（回填整行位宽，intf_spec §8 的 256b 口径）；`LW = log2(ILINE_WORDS)`；`LI = log2(ICACHE_LINES)`；`TAG_W = 32−LW`（行 tag = pc>>LW 全宽比较）。约束（RTL 几何守卫）：`ICACHE_LINES`/`ILINE_WORDS` 均须为 ≥2 的 2 的幂（行索引/字偏移按位截取，对应 C 模型的 mod/掩码运算）。本模块无配置输入端口；`MEM_LAT` 不在本模块内（tb 从设备参数，§1.4）。

---

## 4. 内部状态

| 寄存器 | 位宽 | 复位值 | 说明 |
|---|---|---|---|
| `state` | 2 | `S_IDLE` | 状态机，见 §6 |
| `miss_pc_q` | 32 | 0 | 缺失在途的取指 pc |
| `rsp_inst_q` | 32 | 0 | 待回送 sf 的指令字 |
| `data[ICACHE_LINES]` | LINE_W/行 | — | 行数据阵列（valid 门控，不复位） |
| `tag[ICACHE_LINES]` | TAG_W/行 | — | 行 tag = pc>>LW（不复位） |
| `valid[ICACHE_LINES]` | 1/行 | 0 | 行有效位（复位清零） |

组合输出：`icache_sf_req_rdy`/`icache_memif_rsp_rdy`/`icache_memif_req_vld`/`icache_sf_rsp_vld` 见 §2；`icache_memif_req_addr` 由 `miss_pc_q` 组合导出（§5.2）；`icache_sf_rsp_inst = rsp_inst_q`。`S_IDLE` 期间对请求 pc 组合查找（§5.1）。

---

## 5. 数据通路

### 5.1 取指查找（S_IDLE 组合）

```
ld_idx = pc[LW+LI-1 : LW]        // 行索引 = tag mod ICACHE_LINES
pc_tag = pc[31 : LW]             // 行 tag
hit    = valid[ld_idx] && (tag[ld_idx] == pc_tag)
rd_word = data[ld_idx][pc[LW-1:0]]   // 行内字偏移选取
```

受理拍（`sf_icache_req_vld && icache_sf_req_rdy`）完成查找：命中锁存 `rsp_inst_q <= rd_word` 转 `S_RSP`；缺失锁存 `miss_pc_q <= pc` 转 `S_REQ`（C 模型受理步同构，§1.5）。

### 5.2 回填请求地址形成（组合）

```
icache_memif_req_addr = (miss_pc_q >> LW) << (LW+2)
```

即 `(pc & ~(ILINE_WORDS-1)) << 2`：行对齐字节地址（§1.3，C 模型同式）。

### 5.3 回填写行与指令选取（S_REFILL 握手拍）

```
wr_idx  = miss_pc_q[LW+LI-1 : LW]
wr_tag  = miss_pc_q[31 : LW]
wr_word = memif_icache_rsp_data[miss_pc_q[LW-1:0]]

data[wr_idx] <= memif_icache_rsp_data    // 整行覆盖分配（直接映射替换）
tag[wr_idx]  <= wr_tag
valid[wr_idx] <= 1
rsp_inst_q   <= wr_word                  // 自回填数据行内选取（与写阵列同值）
```

回填握手拍写入阵列并锁存指令，次拍 `S_RSP` 呈现（C 模型回填步同构，§1.5）。

---

## 6. 状态机

### 6.1 状态定义

| 状态 | 编码 | 行为 |
|---|---|---|
| `S_IDLE` | 0 | 空闲：受理取指请求，组合查找（§5.1）；命中转 `S_RSP`、缺失转 `S_REQ` |
| `S_REQ` | 1 | 回填请求呈现：`icache_memif_req_vld=1` 保持至握手，握手后转 `S_REFILL` |
| `S_REFILL` | 2 | 回填等待：`icache_memif_rsp_rdy=1`，握手拍写行并锁存指令（§5.3），转 `S_RSP` |
| `S_RSP` | 3 | 指令呈现：`icache_sf_rsp_vld=1` 保持至握手，握手后回 `S_IDLE` |

### 6.2 状态行为

- `S_IDLE`：受理拍锁存命中指令或缺失 pc（§5.1）；
- `S_REQ`：请求地址组合导出（§5.2），`vld` 保持至 `memif` 握手；
- `S_REFILL`：呈现即握手（rdy 恒 1），握手拍写行 + 锁存指令；
- `S_RSP`：`vld` 保持至 sf 握手，握手次拍起可受理新请求（C 模型响应清除步与请求受理步同拍发生的对应，§1.5）。

### 6.3 状态迁移表

| 当前 | 条件 | 次态 |
|---|---|---|
| `S_IDLE` | `req_fire` && 命中 | `S_RSP` |
| `S_IDLE` | `req_fire` && 缺失 | `S_REQ` |
| `S_REQ` | `mreq_fire`（回填请求握手） | `S_REFILL` |
| `S_REFILL` | `mrsp_fire`（回填数据握手） | `S_RSP` |
| `S_RSP` | `rsp_fire`（指令握手） | `S_IDLE` |

---

## 7. 通道时序

表中各行为该拍内的电平；握手发生在 `vld` 与 `rdy` 同高那拍的时钟末沿。设 `MEM_LAT = L`（tb 从设备，§1.4）。

### 7.1 命中全流程（无背压）

```
拍          T0    T1    T2
state       IDLE  RSP   IDLE
req_vld     1     x     0/1（下一请求）
req_rdy     1     0     1
rsp_vld     0     1     0
rsp_rdy     x     1     x
rsp_inst    -     指令   -
```

T0 受理请求并组合查找；T1 呈现指令、完成握手；T2 回 `S_IDLE` 可受理下一请求。

### 7.2 缺失回填全流程（MEM_LAT=L，无背压）

```
拍          T0    T1    T2  ... T(L+1)  T(L+2)  T(L+3)  T(L+4)
state       IDLE  REQ   (REFILL ........)       RSP     IDLE
req_vld     1     0     0       0      0       0       0/1
req_rdy     1     0     0       0      0       0       1
mreq_vld    0     1     0       0      0       0       0
mreq_addr   -     行对齐 -      -      -       -       -
mreq_rdy    x     1     -       -      -       -       -
mrsp_vld    0     0     0  ...  0      1       0       0
mrsp_rdy    x     x     1  ...  1      1       x       x
rsp_vld     0     0     0       0      0       1       0
rsp_inst    -     -     -       -      -       指令    -
```

T0 受理并判缺失；T1 呈现回填请求并握手（从设备于 T1 末沿装载倒计时 L）；从设备计数到 0 于 T(L+2) 呈现整行（L=0 时 T2 即呈现），该拍完成回填握手、写行并锁存指令；T(L+3) 呈现指令；T(L+4) 回 `S_IDLE`。IMISS 窗口为 T0 的次拍至 T(L+3) 的握手拍（`req_rdy=0` 期间）。

### 7.3 背压

`sf_icache_rsp_rdy` 拉低（指令消费延迟）：

```
拍          Tq    Tq+1  Tq+2
rsp_vld     1     1     1
rsp_rdy     0     0     1
```

`S_RSP` 保持 `vld` 与指令稳定至握手（协议 §9 条 1），次拍回 `S_IDLE`。回填请求背压（`memif_icache_req_rdy` 拉低）同构：`S_REQ` 保持 `vld` 与地址稳定至握手，从设备倒计时自握手拍起算，后续拍序整体顺延。

### 7.4 与 C 模型的拍序对齐

| 事件 | C 模型 | RTL + tb 从设备 |
|---|---|---|
| 取指请求受理 | 受理步（请求接收段：清 `sf_icache_req.vld`，命中置 `rsp_pending`/缺失置 `miss`） | `S_IDLE` 握手拍（命中锁存 `rsp_inst_q`/缺失锁存 `miss_pc_q`） |
| 命中/回填后指令呈现 | 置位步（响应段：`icache_sf_rsp.vld=1`） | `S_RSP` 呈现拍（受理拍/回填握手拍+1） |
| 指令消费检出 | 清除步（响应段：`rsp_sent` 且通道空 → 清 `rsp_pending`，同拍可受理新请求） | `S_RSP` 握手拍，次拍 `S_IDLE` 受理新请求 |
| 回填请求置位 | 置位步（缺失段：`icache_memif_req.vld=1`） | `S_REQ` 呈现拍（受理拍+1） |
| 回填请求受理 | memif_step 受理步 | tb 从设备握手拍（DUT 转 `S_REFILL`） |
| 回填数据呈现 | countdown 到期提交（受理步+MEM_LAT+1） | tb 从设备呈现拍（请求握手拍+MEM_LAT+1，memif_spec §1.5 同构） |
| 回填数据消费 | 回填步（缺失段：写行、置 `rsp_pending`、清 `memif_icache_rsp.vld`） | `S_REFILL` 握手拍（写行、锁存指令） |

`MEM_LAT = 0` 时回填数据呈现拍为请求握手次拍，两侧一致。

---

## 8. 复位与上电行为

- `rst_n = 0`（异步）：`state` 回 `S_IDLE`，`miss_pc_q`/`rsp_inst_q` 清零，`valid` 全清（行数据/tag 不复位，由 `valid` 门控）；输出 `vld`（`icache_memif_req_vld`、`icache_sf_rsp_vld`）恒 0；
- `rst_n` 释放：进入 `S_IDLE`，`icache_sf_req_rdy` 即开放，不依赖任何启动握手；
- 复位后 cache 为全空（全部行 invalid），首次访问各行均缺失回填。

---

## 9. 协议约束

1. `vld` 拉起后保持，与载荷一同稳定至握手完成（intf_spec §1.2）：指令呈现（`S_RSP`）与回填请求呈现（`S_REQ`）均由本模块保持；
2. `vld` 不组合依赖于本通道 `rdy`：两者均为状态译码（§2）；
3. 请求源只发合法载荷：pc 为字索引，取指地址合法性由 sf 保证（ma_spec §2 口径），本模块不检测；
4. 单事务在途：非 `S_IDLE` 期间 `icache_sf_req_rdy = 0`；
5. `S_REFILL` 期间 `icache_memif_rsp_rdy` 恒 1（呈现即握手）；回填数据只在存在在途回填事务时呈现（tb 从设备按请求起算）；
6. 行数据只经回填路径写入（程序只读，无无效化，ma_spec §8）。

---

## 10. 验证要点

验证以事务级等价为准（§1.6），观测点：

| # | 检查项 | 期望 |
|---|---|---|
| I1 | 请求受理 | 受理拍与参考逐拍一致；缺失/响应在途不接受新请求 |
| I2 | 命中路径 | 受理拍+1 呈现、指令字位精确；顺序取指每行 1 缺失 + 7 命中 |
| I3 | 缺失回填 | 回填请求地址 = 行对齐字节地址、位精确；呈现/受理拍与参考一致；回填数据呈现拍消费（MEM_LAT 口径） |
| I4 | 数据通路 | 回填后同行命中返回回填数据；冲突替换（同索引不同 tag）；指令段越界字为 0 |
| I5 | 背压 | 指令消费背压：`vld`/指令保持；回填请求背压：`vld`/地址保持、拍序顺延 |
| I6 | 协议 | `vld` 不组合依赖 `rdy`；复位期间全部 `vld = 0`；`rdy` 与状态一致 |
| I7 | 延迟任意 | `MEM_LAT` 取 0/1/2/3/5/8 下 I1–I6 成立；随机背压（恒 1、50%、20%、pattern） |
| I8 | 边界 | pc=0、程序长度边界行（跨 `imem_n`）、越界行全 0、最高字、数据极值 |
| I9 | 无死锁 | 全部事务在限界拍数内排空 |

参考模型：testbench（Verilator C++ harness）直接链接 `top/cmodel`（`icache_step`），扮演 sf 侧（取指请求源 + 指令消费者，随机背压）与 memif 侧从设备（倒计时同构 §1.4、指令段后备存储按 `imem_n` 截 0、请求通道随机就绪），参考事务序列与 DUT 事务序列按拍在线比对。
