# Pico 2 USB → SPI Slave Bridge

RP2350 (Raspberry Pi Pico 2) firmware that turns the board into a **USB-to-SPI-slave
data bridge**, built on FreeRTOS SMP with static-only memory allocation.

- An external SPI master reads **64 bytes every 1 ms** (exactly **64,000 B/s**), SPI Mode 1, MSB first.
- A host PC preloads data over USB CDC with a compact packet protocol (framing, checksum, status/error codes), extended for 64-byte-aligned streaming.
- A **384 KiB** main ring buffer caches preloaded data; an **8 KiB** staging ring feeds the SPI peripheral via DMA.
- DMA runs in RP2350 **ENDLESS** transfer-count mode with address ring wrapping, so feeding the SPI TX FIFO costs no CPU time.
- When no preloaded data is available, the bridge transmits zeros.
- Core 0 runs the whole bridge (USB + SPI pump); core 1 only toggles the status LED every 500 ms.

## Data Path

```
Host PC ──USB CDC──> UsbRx task ──WRITE──> 384 KiB SPSC ring
(core 0, prio 4)                              │
                                              │ 1 ms pump (core 0, prio 6)
                                              ▼
                                       8 KiB staging ring
                                              │
                                   DMA ENDLESS + ring mode
                                              │
                                              ▼
                                    SPI0 TX FIFO ──> external master
```

## Hardware

| SPI signal | Pico 2 GPIO | Header pin | Direction |
|------------|-------------|------------|-----------|
| MISO       | GP19        | 25         | slave → master |
| CS         | GP17        | 22         | master → slave |
| GND        | GND         | 23         | common ground |
| SCK        | GP18        | 24         | master → slave |
| MOSI       | GP16        | 21         | master → slave (unused by the bridge) |

- SPI0 in slave mode, **Mode 1 (CPOL=0, CPHA=1)**, 8-bit frames, MSB first.
  Mode 0 is not usable when the master holds CS low for a whole 64-byte read:
  the PL022 slave freezes its shift register while CS is asserted when SPH=0,
  so it cannot load the next byte from the TX FIFO. Mode 1 allows CS to stay
  low for the whole transaction. If the master is fixed at Mode 0, it must
  raise CS between bytes.
- **3.3 V logic only.** Connect grounds together.
- On RP2350 the SPI peripheral's TX pin is always the data output and RX always the data
  input, regardless of the bus role. As a slave, **GP19 (SPI0 TX) drives the master's
  MISO**, and **GP16 (SPI0 RX) receives the master's MOSI**.
- CS has an internal pull-up and SCK a pull-down (CPOL=0). The RX input is not used by
  the bridge, but its FIFO is drained. The peripheral tristates its output when
  deselected, so it is safe on a shared bus.

## Build and Flash

`pico-sdk/` and `FreeRTOS-Kernel/` are git submodules; no environment variables are required.

```bash
git submodule update --init --recursive
cmake -S . -B build
cmake --build build -j4
picotool load -fx build/pico2-freertos-template.uf2
```

Requirements: Pico SDK >= 2.3.0, `arm-none-eabi-gcc`, CMake >= 3.12, and an **RP2350**
board. The build intentionally fails on RP2040 (`#error`) because DMA endless mode is RP2350-only.

## USB Protocol

The wire format uses compact `0xAA ... 0x55` framing with a command/status/error
payload, and a 16-bit length so one packet can carry several 64-byte SPI frames.
The implementation is in `src/bridge_protocol.c` / `.h`; it has no Pico SDK
dependencies, and `main.c` supplies the ring and USB callbacks.

This bridge deliberately has **no flash or firmware-upgrade commands**. Firmware
flashing stays with the RP2350 ROM BOOTSEL/UF2 path.

See [docs/bridge-protocol-design.md](docs/bridge-protocol-design.md) for the full
design, the adaptation rationale, and host integration notes.

```
0xAA | chksum | len_lo | len_hi | payload[len] | 0x55
```

- `chksum` makes the 8-bit sum of the complete packet, including the checksum
  byte and the rear `0x55`, zero (two's complement of the sum).
- `len` is little-endian, 2..1019; the total packet is `len + 5` bytes.
- `payload[0]` is the command; `payload[1]` is reserved (0) in requests and
  carries the status in responses.
- Multi-byte fields are little-endian, except error codes, which are sent as a
  type byte followed by a code byte (high byte first).

A packet with a bad length, checksum, or rear byte is dropped. Checksum errors
are counted in `checksum_errors`; the host retries a WRITE after its response
timeout and the stream offset prevents duplication.

### Commands

| Cmd | Name | Request payload | Response payload |
|-----|------|-----------------|------------------|
| `0x01` | WRITE_DATA | `seq[2], offset[4], data[N]` | `seq[2], offset[4], accepted[2], stream_accepted[4], free[4], err[2]` |
| `0x02` | GET_STATUS | none | 13 × u32, see below |
| `0x03` | CLEAR_COUNTERS | none | none |
| `0x04` | GET_VERSION | none | `major, minor, fw_major, fw_minor` |

Status byte: `0x01` success, `0x02` busy (reserved), `0x03` error.

WRITE_DATA rules:

- `N` must be a multiple of **64** and at most **960** (15 frames). The device
  rejects other lengths with `ERR_NOT_ALIGNED` or `ERR_INVALID_LENGTH`. This
  keeps every ring append frame-aligned, so the SPI master always sees whole
  64-byte transactions.
- `offset` is the absolute byte position in the host stream. The device only
  accepts a packet when `offset == stream_accepted`. A retry after a lost ACK
  therefore returns `ERR_OFFSET` plus the current `stream_accepted` instead of
  queuing the same bytes twice.
- When the ring is full, `accepted < N` and `err = ERR_RING_FULL`. The host
  waits and resends `data + accepted` at the new offset. `accepted` is always a
  multiple of 64 because the ring capacity and every append are frame-aligned.
- The stream is append-only. On connect the loader reads GET_STATUS and
  continues at the device's `stream_accepted`, unless `--start-offset` is
  given. Restarting from zero requires an MCU reset; there is no flush command
  and consumed bytes cannot be replayed.
- Send one WRITE packet at a time and read its response before sending the
  next. The device has a 4 KiB CDC TX buffer; a host that never drains
  responses can stall the USB task. The reference loaders do this.
- CLEAR_COUNTERS clears `checksum_errors`, `bad_packets`, `ring_full_events`,
  `packets_ok`, `zero_fill` and `underrun_events`. It does not touch buffered
  data, `stream_accepted` or `spi_consumed`.

### Status fields (GET_STATUS, 13 × u32)

| Field | Description |
|-------|-------------|
| `capacity` | Ring size (393216 bytes) |
| `used`, `free` | Current ring occupancy |
| `stage_ahead` | Bytes staged ahead of the DMA read position |
| `spi_consumed` | Bytes fed into the SPI TX FIFO (includes FIFO lookahead) |
| `zero_fill` | Bytes synthesized as zeros because the main ring was empty |
| `checksum_errors` | Packets dropped due to checksum/rear mismatch |
| `bad_packets` | Oversized/unknown command or invalid-length packets |
| `ring_full_events` | WRITE packets that were only partially accepted |
| `underrun_events` | Pump fell behind the DMA (safety resync) |
| `packets_ok` | Valid packets dispatched |
| `stream_accepted` | Total bytes accepted since boot (32-bit wrapping) |
| `stream_offset` | `stream_accepted % 64`; should always be 0 |

### Error codes

| Code | Name | Meaning |
|------|------|---------|
| `0x0000` | `ERR_SUCCESS` | success |
| `0x0101` | `ERR_INVALID_LENGTH` | zero/oversized payload |
| `0x0104` | `ERR_UNSUPPORT` | unknown command |
| `0x0106` | `ERR_CHECKSUM` | checksum error (counter only) |
| `0x0201` | `ERR_RING_FULL` | packet partially accepted |
| `0x0202` | `ERR_NOT_ALIGNED` | data length not a multiple of 64 |
| `0x0203` | `ERR_OFFSET` | stream offset mismatch |
| `0xFFFF` | `ERR_UNEXPECT` | unexpected error |

## Host Stream Tool

The repository includes `tools/usb_stream.py` (requires Python 3 and `pyserial`):

```bash
pip install pyserial

# Counters, protocol version, clear counters
python3 tools/usb_stream.py --status
python3 tools/usb_stream.py --version
python3 tools/usb_stream.py --clear-counters

# Preload a file (auto-detects USB VID:PID 2e8a:0009)
# By default the loader continues at the device's stream_accepted (append).
python3 tools/usb_stream.py data.bin
python3 tools/usb_stream.py -p /dev/ttyACM1 --start-offset 0 data.bin

# Stream from stdin
cat data.bin | python3 tools/usb_stream.py -
```

A dependency-free C++17 stream tool (Linux/POSIX, `termios` + `poll`) is also provided:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -o usb_stream tools/usb_stream.cpp
./usb_stream /dev/ttyACM1 --status
./usb_stream /dev/ttyACM1 --version
./usb_stream /dev/ttyACM1 data.bin
cat data.bin | ./usb_stream /dev/ttyACM1 -
```

**Port selection:** the Python tool auto-detects VID:PID `2e8a:0009` and uses
the first match. If the board enumerates as `/dev/ttyACM1`, or several CDC
devices are present, pass `-p /dev/ttyACM1`. The C++ tool always takes the port
as its first argument.

### Test pattern

Both tools can generate a test stream instead of reading a file. Frame index N
(starts at 0) is 64 bytes, all filled with byte `N & 0xFF`: the first frame is
`0x00`, frame index 255 is `0xFF`, and index 256 wraps to `0x00` again. Without
`--frames` the pattern runs until Ctrl-C; `--frames N` stops after N frames.

```bash
# Endless pattern on /dev/ttyACM1
python3 tools/usb_stream.py -p /dev/ttyACM1 --pattern --start-offset 0

# 260 frames: covers 0x00..0xFF and the wrap back to 0x00
python3 tools/usb_stream.py -p /dev/ttyACM1 --pattern --frames 260 --start-offset 0
./usb_stream /dev/ttyACM1 --pattern --frames 260 --start-offset 0
```

Each SPI 64-byte read should contain 64 copies of the same value; the next
frame is the next value. The tool batches frames into USB WRITE payloads
(default 960 B = 15 frames); add `--chunk 64` if you want one 64-byte frame
per WRITE. Use a freshly reset MCU (or `--start-offset 0`) for a deterministic
first frame; otherwise the pattern is appended after existing data. `--pattern`
cannot be combined with a file.

Files are sent in **64-byte frames**; a trailing partial frame is padded with
zero bytes and reported with a warning. WRITE payloads are at most 960 bytes
(15 frames), and a full/partial ring is handled automatically via
`ERR_RING_FULL` and the stream offset. If the SPI master is not running, the
ring fills up and the loader stops after `--stall-timeout` seconds (default 10).

The stream is **consumed and append-only**: each byte is sent to the master at
most once, and a retried packet is not queued twice. Loading the same file
twice appends it twice. To start a new stream from offset 0, reset the MCU
(or pass `--start-offset` only when you know the device is at that offset).

On Linux the port usually belongs to the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"   # log out and back in
```

## Buffer and Real-Time Behavior

- **384 KiB** holds exactly **6 seconds** of data at the 64,000 B/s drain rate.
- The pump task runs every 1 ms on core 0 at priority 6 and keeps up to 512 bytes
  (8 ms worth) staged ahead of the DMA. The USB task also runs on core 0 at priority 4,
  so the pump preempts it; the 512-byte prefetch and 8 KiB staging ring absorb normal
  USB IRQ/scheduling jitter. Core 1 only runs the status LED task (500 ms toggle).
- The DMA read cursor advances byte-by-byte, but the pump refills the staging ring in
  whole 64-byte units. That keeps the main-ring tail and free space frame-aligned, so a
  partial WRITE accept never splits a 64-byte transaction.
- The DMA never stops: on an empty main ring the pump stages zeros, so the master always
  clocks valid data. Because up to 512 bytes are staged ahead of the DMA, data loaded while
  the master is idle can be preceded by up to 512 already-staged bytes (≈8 ms at the nominal rate).
- Sustained preload throughput is limited by the master's drain rate once the ring is full.
  A master that reads slower than 64,000 B/s will drain the ring proportionally slower.
- The bridge is robust to USB disconnects: the partial packet parser resets on reconnect,
  the device keeps its stream offset, and buffered data survives. Query GET_STATUS to
  resume appending at `stream_accepted`.

## Customization

At the top of `src/main.c`:

- `BRIDGE_SPI`, `BRIDGE_PIN_TX`/`RX`/`CS`/`SCK`, `BRIDGE_SPI_CPOL`/`BRIDGE_SPI_CPHA` —
  peripheral, pins, mode. `TX` is the slave output (master MISO) and `RX` the slave input.
- `USB_RING_SIZE` — main ring size (default 384 KiB). Keep it smaller than the available RAM.
- `STAGE_RING_BITS` — staging ring size (`1 << STAGE_RING_BITS` bytes). It must stay
  power-of-two and is aligned to its size; the DMA ring limit is 32 KiB.
- `STAGE_PREFETCH`, `PUMP_PERIOD_MS` — real-time tradeoff.

Protocol configuration lives in `src/bridge_protocol.h`:

- `BRIDGE_FRAME_BYTES` / `BRIDGE_WRITE_MAX_DATA` — SPI frame size and the largest
  WRITE payload (15 frames = 960 bytes). Keep both multiples of the frame size.
- `BRIDGE_PACKET_MAX_SIZE` — USB packet buffer limit (1024 bytes);
  `BRIDGE_PACKET_MAX_PAYLOAD` follows from it.
- `BRIDGE_CMD_*`, `BRIDGE_ERR_*`, protocol/firmware version numbers.

FreeRTOS tunables live in `FreeRTOSConfig_examples_common.h`; the app uses static
allocation only, so new tasks must use `xTaskCreateStatic()`.

USB CDC options are set in `CMakeLists.txt`:

- `CFG_TUD_CDC_RX_BUFSIZE=8192` — larger endpoint buffering for high-rate preloads.
- `CFG_TUD_CDC_TX_BUFSIZE=4096` — room for several WRITE responses so the USB task
  does not block on TX.
- `PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1` — accept hosts that do not assert DTR.

## File Structure

```
pico2-freertos-template/
├── CMakeLists.txt                        # Build target, static allocation, stdio USB options
├── src/main.c                            # Bridge app (SPI slave, DMA, ring, tasks, entry point)
├── src/bridge_protocol.c                 # USB packet parser/commands (no SDK dependencies)
├── src/bridge_protocol.h                 # Protocol constants, ops callbacks, state
├── docs/bridge-protocol-design.md        # Protocol design and adaptation notes
├── tools/usb_stream.py                   # Reference host loader / STATUS tool (pyserial)
├── tools/usb_stream.cpp                  # C++17 host loader (Linux/POSIX)
├── cmake/                                # SDK / FreeRTOS import helpers
├── FreeRTOSConfig.h                      # FreeRTOS configuration entry point
├── FreeRTOSConfig_examples_common.h      # Tunables
├── pico-sdk/              (submodule)    # Raspberry Pi Pico SDK 2.3.0
└── FreeRTOS-Kernel/       (submodule)    # FreeRTOS kernel (RP2350 SMP port)
```

## Dependencies

| Dependency | Source | Version |
|-----------|--------|---------|
| [pico-sdk](https://github.com/raspberrypi/pico-sdk) | submodule `pico-sdk/` | 2.3.0 |
| [FreeRTOS-Kernel](https://github.com/raspberrypi/FreeRTOS-Kernel) | submodule `FreeRTOS-Kernel/` | RP2350 port support |
| [tinyusb](https://github.com/hathach/tinyusb) | pico-sdk submodule | 0.18.0 |
| [pyserial](https://pyserial.readthedocs.io/) | host, pip | for `tools/usb_stream.py` |

## FAQ

**Q: `usb_stream.py` cannot find the device**
A: Pass `--port` explicitly and check `lsusb` for `2e8a:0009`. On Linux, make sure your
user is in the `dialout` group (`/dev/ttyACM0` permissions).

**Q: The loader reports `ERR_RING_FULL` / `ring stalled`**
A: The SPI master is not clocking data out, so the 384 KiB ring cannot drain. Verify SCK,
CS, common ground, and **Mode 1** on the master. `ERR_RING_FULL` itself is normal
backpressure when the host runs ahead of the 64,000 B/s drain.

**Q: The loader reports `ERR_OFFSET` / "device offset does not match"**
A: The device's `stream_accepted` does not match the host's offset. This is expected after
a retried WRITE (the loader handles it automatically) or after a host restart. Query
`--status` and start at the reported `stream_accepted`, or reset the MCU to restart the
stream from zero.

**Q: `underrun_events` or `zero_fill` keeps increasing**
A: The host is not supplying data fast enough. Check the loader's progress output and USB
connection. `zero_fill` alone is expected whenever the ring runs empty.

**Q: Building for RP2040 fails**
A: This bridge uses RP2350 DMA endless mode. Use a Pico 2 / Pico 2 W (`PICO_BOARD=pico2`
or `pico2_w`).
