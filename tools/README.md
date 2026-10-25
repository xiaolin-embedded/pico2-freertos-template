# Host Test Tools for the USB-SPI Bridge

This directory contains two equivalent host-side reference tools:

| File | Language | Description |
|---|---|---|
| `usb_stream.py` | Python 3 + pyserial | Quick tests with automatic port detection |
| `usb_stream.cpp` | C++17 (Linux/POSIX) | No third-party dependencies; suited to long or high-throughput runs |

Both tools only handle the **USB side**: data loading and status queries. They do
not flash firmware; use BOOTSEL + UF2 for that.

Protocol, commands, and error codes:

- [`../README.md`](../README.md)
- [`../docs/bridge-protocol-design.md`](../docs/bridge-protocol-design.md)

## 1. Build the C++ Tool

From the repository root:

```bash
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -Werror \
    -o build/usb_stream tools/usb_stream.cpp
```

The examples below use `./build/usb_stream`. You can also build inside `tools/`:

```bash
cd tools
g++ -std=c++17 -O2 -Wall -Wextra -Werror -o usb_stream usb_stream.cpp
./usb_stream /dev/ttyACM1 --status
```

The Python tool requires pyserial:

```bash
pip install pyserial
```

## 2. Serial Port

The Pico 2 USB CDC device uses VID:PID `2e8a:0009`. On Linux it normally appears
as `/dev/ttyACM0`, `/dev/ttyACM1`, etc.

- **Python**: without `-p`, the tool selects the first `2e8a:0009` device. If
  several CDC devices are present, or the board is `/dev/ttyACM1`, specify it:

  ```bash
  python3 tools/usb_stream.py -p /dev/ttyACM1 --status
  ```

- **C++**: the port is the first positional argument and is mandatory:

  ```bash
  ./build/usb_stream /dev/ttyACM1 --status
  ```

Regular users usually need to be in the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"   # log out and back in
```

All examples below use `/dev/ttyACM1`; replace it with the actual device path.

## 3. Confirm the Device Is Online

```bash
# Python
python3 tools/usb_stream.py -p /dev/ttyACM1 --version
python3 tools/usb_stream.py -p /dev/ttyACM1 --status

# C++
./build/usb_stream /dev/ttyACM1 --version
./build/usb_stream /dev/ttyACM1 --status
```

`--version` should report `protocol 2.0, firmware 1.0`. The firmware banner
should also show `SPI0 slave mode 1`.

Key STATUS fields:

| Field | Meaning |
|---|---|
| `capacity` / `used` / `free` | Ring capacity and occupancy |
| `stage_ahead` | Bytes prefetched into the staging ring; normally about 500 B |
| `spi_consumed` | Bytes fed to the SPI TX FIFO (includes FIFO lookahead) |
| `zero_fill` | Zero bytes inserted when the ring was empty; should stay constant while the host keeps up |
| `checksum_errors` / `bad_packets` | Should be 0 |
| `ring_full_events` | WRITEs partially accepted; normal backpressure |
| `underrun_events` | Pump/DMA resynchronisation; should be 0 |
| `stream_accepted` | Total bytes accepted by the device (32-bit wrapping) |
| `stream_offset` | Should be 0 |

## 4. Finite Data Tests

### 4.1 One-Shot File Load

```bash
# Python
python3 tools/usb_stream.py -p /dev/ttyACM1 data.bin

# C++
./build/usb_stream /dev/ttyACM1 data.bin
```

Data is sent in 64-byte frames. A trailing block shorter than 64 B is padded
with zeros and reported with a warning. By default the tool appends at the
device's current `stream_accepted`; it does not overwrite existing data.

### 4.2 Finite Test Pattern

Pattern rule: frame index N (starting at 0) is 64 bytes all equal to
`N & 0xFF`:

```
Frame 1   (index 0)    0x00 x64
Frame 2   (index 1)    0x01 x64
...
Frame 256 (index 255)  0xFF x64
Frame 257 (index 256)  0x00 x64   (wrap)
```

Sending 260 frames covers one full cycle and verifies the wrap:

```bash
# Python
python3 tools/usb_stream.py -p /dev/ttyACM1 \
    --pattern --frames 260 --start-offset 0

# C++
./build/usb_stream /dev/ttyACM1 \
    --pattern --frames 260 --start-offset 0
```

`--start-offset 0` is valid only after an **MCU reset**, when the device's
`stream_accepted` is 0. If the device has already accepted data, omit
`--start-offset` and the tool will append automatically; otherwise the device
returns `ERR_OFFSET`.

## 5. Continuous Streaming (Long Logic-Analyzer Runs)

Continuous mode keeps the device ring at a target level instead of filling the
whole 384 KiB once.

```bash
# Endless pattern with a 200 ms device buffer; STATUS every 5 s; Ctrl-C stops
python3 tools/usb_stream.py -p /dev/ttyACM1 --pattern
./build/usb_stream /dev/ttyACM1 --pattern

# Stream for 60 s with a 500 ms buffer and a report every second
python3 tools/usb_stream.py -p /dev/ttyACM1 --loop --duration 60 \
    --watermark-ms 500 --report 1 data.bin

# Replay a file until Ctrl-C
./build/usb_stream /dev/ttyACM1 --loop data.bin

# Send one 64-byte frame per WRITE (the default is 960 B = 15 frames)
python3 tools/usb_stream.py -p /dev/ttyACM1 --pattern --chunk 64
```

Continuous mode:

- Preloads the ring to `--watermark-ms` (default 200 ms), then keeps it there.
- Uses the WRITE ACK's `free` field for flow control and waits roughly
  `(used - target) / 64 B/ms` when above the target.
- Keeps one WRITE in flight. `ERR_RING_FULL` is normal backpressure; the tool
  resumes automatically.
- `--report S` prints a STATUS summary every S seconds (`0` disables it).
- `--duration S` stops after S seconds; omit it or use `0` to run until Ctrl-C.
- `--watermark-ms 0` disables throttling and fills the ring until WRITE
  backpressure (similar to a one-shot preload).

Example report line:

```
[   12.3s] used=12800 free=380416 stage=504 spi=789000 zero=0 full=2 under=0 crc=0 bad=0 stream=16640 off=0
```

## 6. Common Options

| Option | Python | C++ | Description |
|---|:---:|:---:|---|
| `-p/--port` | yes | positional | C++ takes the port as the first argument |
| `--status` / `--version` / `--clear-counters` | yes | yes | Query or clear counters |
| `-c/--chunk N` | yes | yes | WRITE payload size, 64..960, multiple of 64 |
| `--pattern` | yes | yes | Send the test pattern |
| `--frames N` | yes | yes | Pattern frame count; omit or 0 = continuous |
| `--loop` | yes | yes | Replay the input file continuously |
| `--watermark-ms N` | yes | yes | Continuous-mode target level, default 200 ms |
| `--report S` | yes | yes | Status report period, default 5 s |
| `--duration S` | yes | yes | Continuous run duration; default unlimited |
| `--start-offset N` | yes | yes | Absolute device stream offset; default appends |
| `--stall-timeout S` | yes | no | Stall timeout; C++ uses a fixed 10 s |
| `--retries N` | yes | no | Response-timeout retries; C++ uses a fixed 5 |
| `-b/--baud` | yes | no | Ignored by USB CDC |

## 7. Working with the SPI Master

- The SPI master must use **Mode 1 (CPOL=0, CPHA=1)**, 8-bit, MSB first, with
  active-low CS, and must read all 64 bytes while CS stays low.
- At 1 kHz x 64 B the average rate is 64,000 B/s; the host must supply at least
  that average rate.
- **The host must send data first**: before the first USB WRITE, the firmware
  does not start the DMA, the TX FIFO is empty, and MISO is idle. Load a file or
  run `--pattern` to start the stream.
- The SPI master's MOSI is not used by the bridge; it only reads MISO. The data
  source is the PC over USB.
- Configure the logic analyzer's SPI decoder for **Mode 1** as well. A Mode 0
  decoder can make `0x01` look like `0x80` due to a one-bit rotation.

## 8. Troubleshooting

**`USB CDC device 2e8a:0009 not found`**

Check the USB cable, `lsusb`, `/dev/ttyACM*`, and group membership. With
multiple devices, specify `-p /dev/ttyACM1`.

**`device offset 0x... does not match host 0x...`**

The device has already accepted data and the host was told to start at an old
offset. Reset the MCU and use `--start-offset 0`, or omit `--start-offset` so
the tool appends at the current device offset.

**`ring stalled; is the SPI master clocking?`**

The ring is full but the master is not consuming. Check the master's CS/SCK,
Mode 1, common ground, and whether it is actually clocking.

**No MISO activity / a fixed value**

First confirm that data was sent over USB (`stream_accepted > 0` in STATUS).
Then confirm MISO is connected to Pico **pin 25 / GP19**, and that both the
master and the analyzer decoder use Mode 1.

**`zero_fill` keeps increasing**

The host is not supplying data fast enough. Increase the production rate,
raise `--watermark-ms`, or preload more data. The master may also be reading
faster than 64,000 B/s.

**`ERR_RING_FULL` / `ring_full_events` increasing**

The host is writing faster than SPI consumes. This is normal backpressure; the
tool waits and resumes automatically. It is not a protocol error.

## 9. Quick Verification Flow

```bash
# 1. Build the C++ tool (repository root)
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -Werror \
    -o build/usb_stream tools/usb_stream.cpp

# 2. Check version and status
python3 tools/usb_stream.py -p /dev/ttyACM1 --version
python3 tools/usb_stream.py -p /dev/ttyACM1 --status

# 3. Finite pattern: verify 64-byte frames and the 0xFF -> 0x00 wrap
python3 tools/usb_stream.py -p /dev/ttyACM1 \
    --pattern --frames 260 --start-offset 0

# 4. Long continuous pattern for the logic analyzer
python3 tools/usb_stream.py -p /dev/ttyACM1 \
    --pattern --watermark-ms 200 --report 5

# 5. Press Ctrl-C to stop and print the final STATUS
```
