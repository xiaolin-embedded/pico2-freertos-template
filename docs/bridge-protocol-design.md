# USB → SPI Slave 桥接协议设计

- 文档状态：与当前实现同步（协议版本 2.0）
- 适用固件：`src/main.c` + `src/bridge_protocol.c/.h`
- 上位机参考实现：`tools/usb_loader.py`、`tools/usb_loader.cpp`
- 最后更新：2026-10-24

## 1. 背景与需求

RP2350（Pico 2）作为 USB CDC → SPI slave 的数据桥：

- Linux 上位机通过 USB CDC 周期性下发数据，实时性一般，可能每 10 ms 下发 10 帧或更多。
- 外部 SPI master 以 1 kHz 周期读取，每次固定 64 字节。
- 上位机下发和 SPI master 读取完全异步，MCU 内部用大缓冲解耦。
- 需要交互协议：下发成功/失败、具体错误码、缓冲水位和统计信息。
- 不实现 USB 固件升级；固件烧录继续使用 RP2350 ROM 的 BOOTSEL + UF2。

## 2. 设计目标与非目标

目标：

- 上位机突发写入不丢数据，SPI 侧始终有数据可读。
- 数据按字节流严格保序，64 字节事务边界不漂移。
- 环形缓冲满时返回明确错误码和已接收字节数，主机可续传。
- ACK 丢失后的重发不会造成重复入队。
- 协议层不依赖 Pico SDK/FreeRTOS，可在 PC 上单独测试。

非目标：

- 不做 flash 擦写、固件升级、跳转。
- 不解析 SPI 的 CS 边界（设备只看到字节流）。
- 不支持随机寻址/回放；数据是消费型流。
- 不追求多主机、多从机或复杂路由。

## 3. 系统架构与数据通路

```
Linux host
  │ USB CDC bulk OUT
  ▼
RP2350 USBCTRL ──IRQ──▶ TinyUSB ──▶ CDC RX FIFO (8 KiB)
                                          │
                 main.c: usb_rx_task (core 0, prio 4, 1 ms)
                                          │  up to 8 × 1 KiB per tick
                                          ▼
                              bridge_protocol_feed()  (byte FSM)
                                          │ valid packet
                                          ▼
                                   dispatch_packet()
                              ┌───────────┴───────────┐
                              ▼                       ▼
                     WRITE → usb_ring_write      response → CDC TX FIFO (4 KiB)
                     (384 KiB SPSC ring)         → USB IN → host
                              │
                              ▼
                 SpiPump (core 0, prio 6, 1 ms)
                              │
                              ▼
                    8 KiB staging ring (512 B prefetch)
                              │
                              ▼
                    DMA ENDLESS + ring mode
                              │
                              ▼
                      SPI0 TX FIFO → master
```

任务与核心：

| 核心 | 任务 | 优先级 | 职责 |
|---|---|---:|---|
| 0 | `UsbRx` | 4 | 轮询 CDC FIFO、协议解析、写 ring、发响应 |
| 0 | `SpiPump` | 6 | 每 1 ms 补 staging，跟踪 DMA 位置，空时补零 |
| 1 | `Blink` | 1 | 状态 LED，每 500 ms 翻转 |

SPI：

- SPI0 slave，**Mode 1（CPOL=0, CPHA=1）**，8-bit，MSB first。
- MISO = GP19（SPI0 TX），MOSI = GP16（SPI0 RX），CS = GP17，SCK = GP18。
- Mode 1 是必需的：PL022 在 SPH=0 时 CS 有效期间会冻结从机移位寄存器，无法在一次 CS 拉低期间连续读出 64 字节。若 master 固定 Mode 0，只能在每字节之间抬 CS。

## 4. 帧格式

```
0xAA | chksum | len_lo | len_hi | payload[len] | 0x55
```

| 字段 | 长度 | 说明 |
|---|---:|---|
| `0xAA` | 1 | 包头 |
| `chksum` | 1 | 8-bit 累加和，使整包（含本字节和 0x55）之和为 0 |
| `len` | 2 | payload 长度，小端，范围 2..1019 |
| `payload` | len | 命令数据 |
| `0x55` | 1 | 包尾 |

校验算法：

```text
发送端：chksum = 0，求所有字节之和 sum，chksum = (uint8_t)(0 - sum)
接收端：对所有字节求和，结果为 0 且最后一字节为 0x55 才合法
```

- 包总长度 = `len + 5`，最大 1024 字节。
- `payload[0]` 是命令码；`payload[1]` 在请求中保留为 0，在响应中表示状态。
- 多字节字段默认小端；错误码例外，按“类型字节 + 代码字节”发送（高字节在前）。
- 校验失败、长度超限或包尾错误的包直接丢弃，不回复；主机超时重试。
- 解析器是纯字节流状态机，不要求包边界和 USB 传输边界对齐；一个包可以跨多次 `in_chars`，一次 `in_chars` 也可以包含多个包。

## 5. 命令集

状态字节：

| 值 | 名称 | 含义 |
|---:|---|---|
| `0x01` | `SUCCESS` | 成功 |
| `0x02` | `BUSY` | 保留，当前未使用 |
| `0x03` | `ERROR` | 失败，命令相关的数据区带错误码 |

### 5.1 WRITE_DATA (`0x01`)

请求 payload：

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | `0x01` |
| 1 | u8 | 保留，0 |
| 2 | u16 LE | `seq`，主机序号，原样回显 |
| 4 | u32 LE | `offset`，本次数据在主机流中的绝对字节位置 |
| 8 | u8[N] | 数据，N 必须是 64 的倍数，且 N ≤ 960 |

响应 payload（20 字节）：

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | `0x01` |
| 1 | u8 | 状态 |
| 2 | u16 LE | `seq` |
| 4 | u32 LE | `offset` 回显 |
| 8 | u16 LE | `accepted`，本次实际入队字节数 |
| 10 | u32 LE | `stream_accepted`，设备当前流位置 |
| 14 | u32 LE | `free`，写后 ring 剩余空间 |
| 18 | u16 BE | 错误码（类型字节在前） |

规则：

- `N % 64 == 0` 且 `0 < N ≤ 960`；否则返回 `ERR_NOT_ALIGNED` 或 `ERR_INVALID_LENGTH`。
- 设备只在 `offset == stream_accepted` 时接收数据；否则返回 `ERR_OFFSET` 和当前 `stream_accepted`，不写入任何字节。
- ring 空间不足时只接收前缀，`accepted < N`，`err = ERR_RING_FULL`；主机从 `data + accepted` 继续。
- 因为 ring 容量和每次追加都是 64 对齐的，`accepted` 始终是 64 的倍数，不会把一帧拆开。
- `stream_accepted` 是 32 位回绕计数；主机与设备都按 2^32 取模处理。

### 5.2 GET_STATUS (`0x02`)

请求 payload：`0x02, 0x00`（恰好 2 字节）。

响应 payload：`0x02, 0x01` + 13 个 u32 LE，共 54 字节。

| 序号 | 字段 | 说明 |
|---:|---|---|
| 1 | `capacity` | ring 容量，393216 B |
| 2 | `used` | 当前 ring 占用 |
| 3 | `free` | 当前 ring 剩余 |
| 4 | `stage_ahead` | staging ring 中领先 DMA 的字节数 |
| 5 | `spi_consumed` | DMA 已从 staging 取走的字节（含 FIFO 预取） |
| 6 | `zero_fill` | 因 ring 空而补的零字节 |
| 7 | `checksum_errors` | 校验/包尾错误丢弃的包数 |
| 8 | `bad_packets` | 超长、`len < 2` 或未知命令的包数 |
| 9 | `ring_full_events` | 被部分接收的 WRITE 次数 |
| 10 | `underrun_events` | pump 落后 DMA 的安全重同步次数 |
| 11 | `packets_ok` | 通过校验并进入分派的包数（含未知命令和 STATUS 查询） |
| 12 | `stream_accepted` | 开机以来累计接收字节数（32 位回绕） |
| 13 | `stream_offset` | `stream_accepted % 64`，正常应恒为 0 |

### 5.3 CLEAR_COUNTERS (`0x03`)

请求 payload：`0x03, 0x00`。响应 payload：`0x03, 0x01`。

清除：`checksum_errors`、`bad_packets`、`ring_full_events`、`packets_ok`、`zero_fill`、`underrun_events`。

不清除：ring 中的数据、`stream_accepted`、`spi_consumed`。因此清计数器不会破坏主机的流偏移。

### 5.4 GET_VERSION (`0x04`)

请求 payload：`0x04, 0x00`。响应 payload：`0x04, 0x01, major, minor, fw_major, fw_minor`。

当前值：协议 2.0，固件 1.0。

## 6. 流控与数据完整性

### 6.1 32 位流偏移

每条 WRITE 都携带 `offset`，设备只在它等于 `stream_accepted` 时接收。带来的性质：

- **ACK 丢失可安全重发**：主机超时后用同一个 `offset` 重发；如果设备其实已经写入，会返回 `ERR_OFFSET` 和新的 `stream_accepted`，主机据此确认已入队，不会重复写。
- **部分接收可续传**：设备返回 `accepted` 和新的 `stream_accepted`，主机从下一个字节继续。
- **乱序/越位包被拒绝**：`offset` 不匹配的错误包不会写入 ring。

主机侧处理 `ERR_OFFSET` 的规则：

```text
delta = (rsp.stream_accepted - offset) mod 2^32
if delta == 0:           什么也没写入，等待后重发
elif delta <= 本块剩余:  delta 字节已入队，offset += delta，继续发剩余部分
else:                    设备位置与主机预期不一致，报错/重新同步
```

### 6.2 满缓冲与 64 字节对齐

- 上位机数据总长度必须是 64 的倍数；文件最后不足一帧时补零。
- 设备对每条 WRITE 强制 `N % 64 == 0`，因此每次入队都以整帧为单位。
- 主 ring 容量 393216 = 6144 × 64，staging ring 8192 = 128 × 64。
- pump 的补数按 64 向下取整。DMA 读指针是按字节前进的，如果不取整，主 ring 尾部会偏移，partial WRITE 可能拆开一帧。这个约束是数据完整性的关键。
- `stream_offset` 可用于诊断；正常应始终为 0。

### 6.3 校验与错误处理

- 8-bit 累加和只用于快速发现组包/同步错误；USB 本身的 CRC16 才是链路完整性保障。
- 校验失败：静默丢弃、`checksum_errors++`，不回复。主机 WRITE 超时后重发，offset 保证幂等。
- 未知命令/非法长度：返回 `ERROR` + 对应错误码。
- USB 断开重连：解析器状态机复位，ring、计数器、`stream_accepted` 保留；主机查询 STATUS 后从 `stream_accepted` 继续。

### 6.4 主机侧并发建议

- 一次只发一个 WRITE 并读完响应，再发下一个。CDC TX FIFO 为 4 KiB；主机长期不读响应会阻塞 USB 任务。
- 参考工具采用“一发一收”，在此模式下 960 B/包的 ACK 回环远高于 64 kB/s 的需求。
- 如果未来要流水线发送，需要额外的序号/窗口机制；当前协议用 offset 可以扩展，但参考工具未实现。

## 7. 相对原协议样本的适配

原样本是一个 STM32 固件升级协议：64 字节最大包、8-bit 长度、`0xB1 WRITE_DATA` / `0xB2 READ_DATA` / `0xB3 FINISH_UPGRADE` / `0xF0 GET_VERSION`，并包含 flash 写入和跳转代码。

本桥接只借用它的**帧结构、校验思路和“命令 + 状态 + 错误码”应答模型**，其余按本场景重做：

| 项目 | 原样本 | 本桥接 |
|---|---|---|
| 最大包 | 64 B | 1024 B |
| 长度字段 | 8-bit | 16-bit |
| 每条 WRITE 数据 | 32 B（flash 页） | 最多 960 B（15 × 64 B 帧） |
| 命令码 | `0xB1/0xB2/0xB3/0xF0` | `0x01/0x02/0x03/0x04` |
| 寻址 | flash 页号 | 32-bit 流偏移 |
| 对齐 | 32 B flash 写 | 64 B SPI 事务 |
| 升级/flash | 有 | **完全移除** |
| 校验 | 8-bit 累加和 | 保留同样算法 |
| 状态/错误码 | success/busy/error + 16-bit err | 保留风格，错误码改为桥接专用 |

为什么改包长：

- 64 B 包去掉帧头/命令头后最多约 58 B，而样本实际只带 32 B 数据；64 kB/s 需要约 1200–2000 个 WRITE/秒。
- 逐包 ACK 的 USB 回环在 1–2 ms 量级，无法稳定支撑这个请求速率。
- 改成一个 WRITE 最多 960 B 后，64 kB/s 只需约 67 个 WRITE/秒，单包一发一收就有充足余量。
- 16-bit 长度是承载 960 B 数据的必要条件；帧头和校验风格保持不变。

为什么移除升级命令：

- 本产品的固件烧录明确使用 Pico 2 ROM 的 BOOTSEL/UF2，不需要应用层 flash 协议。
- 保留 flash 写/跳转代码会增加误操作和变砖风险，也不属于桥接职责。
- 因此命令码重新编号，避免与原样本的 `FINISH_UPGRADE` 等语义混淆。

为什么增加 offset：

- 原样本是“页号 + 数据”的升级写入，重复写同一页问题不大；桥接是消费型字节流，重复入队会直接破坏数据。
- offset 让“ACK 丢失后的重发”和“ring 满后的续传”都变成可判定的幂等操作。

## 8. 缓冲与时序

| 项目 | 数值 | 说明 |
|---|---:|---|
| SPI 消费率 | 64,000 B/s | 64 B / 1 ms |
| 主 ring | 384 KiB | 约 6.0 s |
| CDC RX FIFO | 8 KiB | 约 128 ms @ 64 kB/s |
| CDC TX FIFO | 4 KiB | 可缓存约 160 条 WRITE 响应 |
| staging ring | 8 KiB | 128 帧 |
| staging 预取 | 512 B | 约 8 ms |
| pump 周期 | 1 ms | core 0，优先级 6 |
| USB 轮询周期 | 1 ms | core 0，优先级 4 |

主机按 10 ms 下发 10 帧（640 B）时，平均正好 64,000 B/s。ring 可以吸收数秒的突发或停顿；如果主机持续低于 64 kB/s，SPI 侧最终会读到补零，`zero_fill` 增长。

上位机建议：

- 启动 SPI master 前先预装一段数据（例如 1–6 秒），避免 master 先读到零。
- 持续运行时保持 ring 有一定水位；不要依赖 USB 的毫秒级实时性。
- master 停止读取时，ring 会满；WRITE 返回 `ERR_RING_FULL`，这是正常背压，不是协议错误。

## 9. 上位机接入指南

### 9.1 组包/解包

- 组包：`AA | chksum | len LE | payload | 55`；checksum 按第 4 节计算。
- 解包：扫描 `0xAA`，读 16-bit 长度，读 payload 和 `0x55`，校验全包和为 0；失败则丢 1 字节重新同步。
- 建议在主机侧按 1–4 KiB 批量读取 USB 数据，维护一个接收缓冲，不要假设一次 `read` 就是一个完整包。

### 9.2 文件/流下发

```text
status = GET_STATUS()
offset = status.stream_accepted          # 默认追加；或显式指定 --start-offset 0
for each block of data (length multiple of 64, <= 960):
    pos = 0
    while pos < len(block):
        rsp = WRITE(seq++, offset, block[pos:])
        if rsp.err == SUCCESS:
            offset += rsp.accepted; pos += rsp.accepted
        elif rsp.err == RING_FULL:
            offset += rsp.accepted; pos += rsp.accepted
            sleep(1 ms)                  # 等 master 消费
        elif rsp.err == OFFSET:
            delta = (rsp.stream_accepted - offset) mod 2^32
            if delta <= len(block) - pos:
                offset += delta; pos += delta
            else:
                report error / resync
        else:
            report error code
```

- 文件末尾不足 64 B 时用 0x00 补齐，并记录补齐字节数。
- 默认从设备 `stream_accepted` 继续（追加）；要从 0 重新开始，需要复位 MCU，或明确知道设备位置时用 `--start-offset`。
- 消费型数据不可回放；没有 flush/seek 命令。

### 9.3 健康检查

- `checksum_errors`、`bad_packets` 应保持 0。
- `stream_offset` 应保持 0。
- ring 非空时 `zero_fill` 不应增长；增长说明主机供数不足。
- `ring_full_events` 增长表示主机写入速度超过 SPI 消费速度，属于正常背压。
- `spi_consumed` 含 DMA FIFO 预取，不能精确等同于 master 已接收字节；`stage_ahead` 是已预取但尚未被 DMA 取走的量。

## 10. 错误码

| 错误码 | 名称 | 触发场景 |
|---:|---|---|
| `0x0000` | `ERR_SUCCESS` | 成功 |
| `0x0101` | `ERR_INVALID_LENGTH` | payload 为 0、超长或命令参数长度不对 |
| `0x0104` | `ERR_UNSUPPORT` | 未知命令 |
| `0x0106` | `ERR_CHECKSUM` | 校验错误（只计数，不回复） |
| `0x0201` | `ERR_RING_FULL` | ring 空间不足，只接收了前缀 |
| `0x0202` | `ERR_NOT_ALIGNED` | WRITE 数据长度不是 64 的倍数 |
| `0x0203` | `ERR_OFFSET` | `offset` 与设备 `stream_accepted` 不一致 |
| `0xFFFF` | `ERR_UNEXPECT` | 预留的未预期错误 |

## 11. 已知限制与测试建议

已知限制：

- 当前只做了主机侧协议测试（含 ring 满和丢 ACK 重发），尚未完成 Pico 2 硬件在环验证。
- 设备不感知 CS/事务边界；master 必须每次 CS 恰好读 64 字节。如果 master 多读/少读，字节流边界会偏移。
- 没有 flush/回放命令；已经消费的数据无法重发。
- 没有单独的“master 已读事务数”计数器，`spi_consumed` 只是 DMA 喂数，不能替代 64 B 事务计数。
- 8-bit 校验和强度有限；链路可靠性主要依赖 USB CRC。
- master 停顿时 staging 中可能有最多约 512 B 的旧数据/补零，恢复后会先被读走。
- 32 位 `stream_accepted` 在 64 kB/s 下约 18.6 小时回绕一次；按模 2^32 比较即可正常工作。

硬件测试建议：

1. master 设为 **Mode 1**，CS 整段拉低，1 kHz 读 64 B。
2. 预装一段数据，确认 `stream_accepted` 与下发字节数一致，`stream_offset == 0`。
3. 连续按 10 ms/10 帧下发，观察 `zero_fill` 不增长、`checksum_errors`/`bad_packets` 为 0。
4. 暂停 master，确认 WRITE 返回 `ERR_RING_FULL` 且主机能续传。
5. 人为制造 ACK 丢失/重发，确认设备不重复入队（offset 机制）。
6. 用逻辑分析仪确认 CS 拉低期间 MISO 连续输出 64 字节，且 Mode 1 时序正确。

## 12. 参考代码

| 内容 | 位置 |
|---|---|
| 协议常量、状态结构、回调接口 | `src/bridge_protocol.h` |
| 帧解析、命令处理、状态响应 | `src/bridge_protocol.c` |
| ring、pump、DMA、任务、SPI 初始化 | `src/main.c` |
| Python 上位机参考实现 | `tools/usb_loader.py` |
| C++17 上位机参考实现 | `tools/usb_loader.cpp` |
| 用户手册 | `README.md` |
