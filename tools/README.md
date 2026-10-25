# USB-SPI 桥接上位机测试工具

本目录提供两个功能等价的上位机参考工具：

| 文件 | 语言 | 说明 |
|---|---|---|
| `usb_stream.py` | Python 3 + pyserial | 方便快速测试、自动探测串口 |
| `usb_stream.cpp` | C++17（Linux/POSIX） | 无第三方依赖，适合持续/高吞吐测试 |

两者都只负责 **USB 侧的数据下发和状态读取**，不烧录固件；固件烧录继续使用 BOOTSEL + UF2。

协议、命令和错误码见：

- [`../README.md`](../README.md)
- [`../docs/bridge-protocol-design.md`](../docs/bridge-protocol-design.md)

## 1. C++ 编译

在仓库根目录执行：

```bash
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -Werror \
    -o build/usb_stream tools/usb_stream.cpp
```

之后示例统一使用 `./build/usb_stream`。也可以进入 `tools/` 目录编译：

```bash
cd tools
g++ -std=c++17 -O2 -Wall -Wextra -Werror -o usb_stream usb_stream.cpp
./usb_stream /dev/ttyACM1 --status
```

Python 工具需要 pyserial：

```bash
pip install pyserial
```

## 2. 串口

Pico 2 的 USB CDC 默认 VID:PID 为 `2e8a:0009`，Linux 下通常枚举成
`/dev/ttyACM0`、`/dev/ttyACM1` 等。

- **Python**：不带 `-p` 时自动选第一个 `2e8a:0009`；如果有多个 CDC 设备或设备是
  `/dev/ttyACM1`，显式指定：

  ```bash
  python3 tools/usb_stream.py -p /dev/ttyACM1 --status
  ```

- **C++**：端口是第一个位置参数，必须显式给出：

  ```bash
  ./build/usb_stream /dev/ttyACM1 --status
  ```

普通用户通常需要加入 `dialout` 组：

```bash
sudo usermod -aG dialout "$USER"   # 重新登录后生效
```

下面的示例都使用 `/dev/ttyACM1`，请按实际枚举结果替换。

## 3. 先确认设备在线

```bash
# Python
python3 tools/usb_stream.py -p /dev/ttyACM1 --version
python3 tools/usb_stream.py -p /dev/ttyACM1 --status

# C++
./build/usb_stream /dev/ttyACM1 --version
./build/usb_stream /dev/ttyACM1 --status
```

`--version` 应返回 `protocol 2.0, firmware 1.0`。程序 banner 还应显示
`SPI0 slave mode 1`。

`--status` 的关键字段：

| 字段 | 含义 |
|---|---|
| `capacity` / `used` / `free` | ring 容量和占用 |
| `stage_ahead` | pump 预取到 staging 的量，正常约 500 B |
| `spi_consumed` | DMA 向 SPI 喂出的累计字节（含 FIFO 预取） |
| `zero_fill` | ring 空时补的零；正常供数时应保持不变 |
| `checksum_errors` / `bad_packets` | 应为 0 |
| `ring_full_events` | WRITE 被部分接收的次数，正常背压 |
| `underrun_events` | pump/DMA 重同步，正常应为 0 |
| `stream_accepted` | 设备累计接收的字节数（32 位回绕） |
| `stream_offset` | 应为 0 |

## 4. 有限数据测试

### 4.1 下发文件（一次）

```bash
# Python
python3 tools/usb_stream.py -p /dev/ttyACM1 data.bin

# C++
./build/usb_stream /dev/ttyACM1 data.bin
```

文件按 64 字节帧下发；最后一个不足 64 B 的块会补零并给出 warning。默认从设备
当前的 `stream_accepted` 追加，不覆盖已有数据。

### 4.2 有限测试图样

图样规则：帧索引 N（从 0 开始）的 64 字节全部等于 `N & 0xFF`，即：

```
第 1 帧 0x00 x64
第 2 帧 0x01 x64
...
第 256 帧 0xFF x64
第 257 帧 0x00 x64（回绕）
```

发送 260 帧可以覆盖一整轮并验证回绕：

```bash
# Python
python3 tools/usb_stream.py -p /dev/ttyACM1 \
    --pattern --frames 260 --start-offset 0

# C++
./build/usb_stream /dev/ttyACM1 \
    --pattern --frames 260 --start-offset 0
```

`--start-offset 0` 只有在 **MCU 刚复位、设备 stream_accepted 为 0** 时才成立；
如果设备已经接收过数据，去掉这个参数让工具自动追加，否则会得到 `ERR_OFFSET`。

## 5. 持续数据测试（长时间抓波形）

持续模式适合让逻辑分析仪长时间观察；工具会维持设备 ring 的水位，而不是一次性
把 384 KiB 填满。

```bash
# 持续图样，默认维持 200 ms 水位，每 5 s 打印状态；Ctrl-C 停止
python3 tools/usb_stream.py -p /dev/ttyACM1 --pattern
./build/usb_stream /dev/ttyACM1 --pattern

# 持续 60 秒，500 ms 水位，每秒打印一次状态
python3 tools/usb_stream.py -p /dev/ttyACM1 --loop --duration 60 \
    --watermark-ms 500 --report 1 data.bin

# 循环播放文件，直到 Ctrl-C
./build/usb_stream /dev/ttyACM1 --loop data.bin

# 每次 WRITE 只带一帧（默认 960 B = 15 帧）
python3 tools/usb_stream.py -p /dev/ttyACM1 --pattern --chunk 64
```

连续模式的行为：

- 先预装到 `--watermark-ms`（默认 200 ms）对应的字节数，再持续补数；
- 用 WRITE ACK 里的 `free` 做流控，超过水位时按 `(used - target) / 64 B/ms` 等待；
- 一次只保留一个 WRITE 在途；`ERR_RING_FULL` 是正常背压，工具会自动续传；
- `--report S` 每 S 秒打印一次 STATUS 摘要，`0` 表示关闭；
- `--duration S` 到时停止；省略或 `0` 表示直到 Ctrl-C；
- `--watermark-ms 0` 关闭节流，按 ring 满背压发送（类似一次预装）。

`--report` 输出示例：

```
[   12.3s] used=12800 free=380416 stage=504 spi=789000 zero=0 full=2 under=0 crc=0 bad=0 stream=16640 off=0
```

## 6. 常用参数

| 参数 | Python | C++ | 说明 |
|---|:---:|:---:|---|
| `-p/--port` | ✓ | 位置参数 | C++ 端口是第一个参数 |
| `--status` / `--version` / `--clear-counters` | ✓ | ✓ | 查询或清计数器 |
| `-c/--chunk N` | ✓ | ✓ | WRITE 数据量，64..960，64 的倍数 |
| `--pattern` | ✓ | ✓ | 发送测试图样 |
| `--frames N` | ✓ | ✓ | 图样帧数；省略或 0 = 连续 |
| `--loop` | ✓ | ✓ | 文件循环播放 |
| `--watermark-ms N` | ✓ | ✓ | 连续模式目标水位，默认 200 ms |
| `--report S` | ✓ | ✓ | 状态打印周期，默认 5 s |
| `--duration S` | ✓ | ✓ | 连续运行时长，默认无限 |
| `--start-offset N` | ✓ | ✓ | 设备绝对流偏移；默认自动追加 |
| `--stall-timeout S` | ✓ | — | 无进展超时；C++ 固定 10 s |
| `--retries N` | ✓ | — | 响应超时重试；C++ 固定 5 次 |
| `-b/--baud` | ✓ | — | USB CDC 忽略该值 |

## 7. 和 SPI master 的配合

- SPI master 必须使用 **Mode 1（CPOL=0, CPHA=1）**、8-bit、MSB first、CS 低有效；
  一次 CS 拉低读满 64 字节。
- 1 kHz × 64 B 时平均速率是 64,000 B/s；主机平均供数要 ≥ 这个速率。
- **上位机不发数据时 MISO 不会自己产生数据**：固件在收到第一笔 USB 数据前不会启动
  DMA，TX FIFO 为空，MISO 空闲。先用 `--pattern` 或文件下发一笔数据。
- SPI master 的 MOSI 不参与桥接；它只读 MISO，数据源是 PC 的 USB。
- 逻辑分析仪的 SPI 解码器也要设成 **Mode 1**，否则可能看到 `0x01 → 0x80` 这类
  “右旋 1 bit”的假象。

## 8. 常见问题

**`USB CDC device 2e8a:0009 not found`**

检查 USB 线、`lsusb`、`/dev/ttyACM*`，以及是否在 `dialout` 组；多设备时用
`-p /dev/ttyACM1` 明确指定。

**`device offset 0x... does not match host 0x...`**

设备已经接收过数据，而工具被要求从某个旧 offset 开始。复位 MCU 后用
`--start-offset 0`，或去掉 `--start-offset` 让工具自动追加。

**`ring stalled; is the SPI master clocking?`**

ring 已满但 master 没有消费。检查 master 的 CS/SCK、Mode 1、共地，以及 master 是否
真的在发时钟。

**MISO 没有波形 / 全是固定值**

先确认已经通过 USB 下发过数据（`--status` 的 `stream_accepted > 0`），再确认
MISO 接在 Pico 的 **pin 25 / GP19**，并且 master/解码器是 Mode 1。

**`zero_fill` 持续增长**

主机供数不足。提高生产速率、增大 `--watermark-ms`，或先预装更多数据；也可能
master 的读取速率高于 64,000 B/s。

**`ERR_RING_FULL` / `ring_full_events` 增长**

主机写入快于 SPI 消费，属于正常背压；工具会自动等待并续传，不是错误。

## 9. 快速验证流程

```bash
# 1. 编译 C++ 工具（仓库根目录）
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -Werror \
    -o build/usb_stream tools/usb_stream.cpp

# 2. 确认版本和状态
python3 tools/usb_stream.py -p /dev/ttyACM1 --version
python3 tools/usb_stream.py -p /dev/ttyACM1 --status

# 3. 有限图样，验证 64 字节帧和 0xFF -> 0x00 回绕
python3 tools/usb_stream.py -p /dev/ttyACM1 \
    --pattern --frames 260 --start-offset 0

# 4. 长时间连续图样，逻辑分析仪持续观察
python3 tools/usb_stream.py -p /dev/ttyACM1 \
    --pattern --watermark-ms 200 --report 5

# 5. Ctrl-C 后查看最终 STATUS
```
