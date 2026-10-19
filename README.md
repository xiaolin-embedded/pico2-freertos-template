# Pico 2 USB → SPI Slave Bridge

RP2350 (Raspberry Pi Pico 2) firmware that turns the board into a **USB-to-SPI-slave
data bridge**, built on FreeRTOS SMP with static-only memory allocation.

- An external SPI master reads **64 bytes every 1 ms** (exactly **64,000 B/s**), SPI Mode 0, MSB first.
- A host PC preloads data over USB CDC with a framed, CRC-protected, ACK-flow-controlled protocol.
- A **384 KiB** main ring buffer caches preloaded data; an **8 KiB** staging ring feeds the SPI peripheral via DMA.
- DMA runs in RP2350 **ENDLESS** transfer-count mode with address ring wrapping, so feeding the SPI TX FIFO costs no CPU time.
- When no preloaded data is available, the bridge transmits zeros.

## Data Path

```
Host PC ──USB CDC──> UsbRx task ──WRITE──> 384 KiB SPSC ring
(core 0, prio 4)                              │
                                              │ 1 ms pump (core 1, prio 6)
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

- SPI0 in slave mode, Mode 0 (CPOL=0, CPHA=0), 8-bit frames, MSB first.
- **3.3 V logic only.** Connect grounds together.
- On RP2350 the SPI peripheral's TX pin is always the data output and RX always the data
  input, regardless of the bus role. As a slave, **GP19 (SPI0 TX) drives the master's
  MISO**, and **GP16 (SPI0 RX) receives the master's MOSI**.
- CS has an internal pull-up and SCK a pull-down in Mode 0. The RX input is not used by
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

All multi-byte fields are little endian.

```
AA 55 | type(1) | seq(2) | len(2) | payload(len) | crc16(2)
```

- `crc16` is CRC-16/CCITT-FALSE: polynomial `0x1021`, init `0xFFFF`, no reflection.
- CRC covers `type` through the end of `payload` (the 5 header bytes plus payload).
- `seq` is echoed in the reply so late/stale replies can be discarded.

### Commands

| Type | Name | Payload | Reply |
|------|------|---------|-------|
| `0x01` | WRITE | 0–4096 data bytes, appended to the ring | `0x81` ACK |
| `0x02` | STATUS | none | `0x82` STATUS |

### ACK (`0x81`, 12-byte payload)

| Offset | Type | Field |
|--------|------|-------|
| 0 | u32 | `accepted` — bytes actually queued (may be less than sent) |
| 4 | u32 | `used` — bytes currently in the ring |
| 8 | u32 | `free` — free space in the ring |

A short accept means the ring is full: resend the remaining bytes later. The reference
loader implements this flow control.

### STATUS (`0x82`, 40-byte payload)

| Field | Description |
|-------|-------------|
| `capacity` | Ring size (393216 bytes) |
| `used`, `free` | Current ring occupancy |
| `stage_ahead` | Bytes staged ahead of the DMA read position |
| `spi_consumed` | Bytes fed into the SPI TX FIFO (includes FIFO lookahead) |
| `zero_fill` | Bytes synthesized as zeros because the main ring was empty |
| `crc_errors` | Frames dropped due to CRC mismatch |
| `bad_frames` | Valid-CRC frames with an unknown type or oversized payload |
| `underrun_events` | Pump fell behind the DMA |
| `ring_full_events` | WRITE payloads that were only partially accepted |

## Host Loader

The repository includes `tools/usb_loader.py` (requires Python 3 and `pyserial`):

```bash
pip install pyserial

# Bridge counters only
python3 tools/usb_loader.py --status

# Preload a file (auto-detects USB VID:PID 2e8a:0009)
python3 tools/usb_loader.py data.bin

# Explicit port; status is printed after loading anyway
python3 tools/usb_loader.py -p /dev/ttyACM0 -s data.bin

# Stream from stdin
cat data.bin | python3 tools/usb_loader.py -
```

A dependency-free C++17 loader (Linux/POSIX, `termios` + `poll`) is also provided:

```bash
g++ -std=c++17 -O2 -o usb_loader tools/usb_loader.cpp
./usb_loader /dev/ttyACM0 --status
./usb_loader /dev/ttyACM0 data.bin
```

Data is **consumed**: each byte is sent to the master at most once. Loading the same file
twice queues it twice. If the SPI master is not running, the ring fills up and the loader
stops with a timeout after `--stall-timeout` seconds (default 10).

On Linux the port usually belongs to the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"   # log out and back in
```

## Buffer and Real-Time Behavior

- **384 KiB** holds exactly **6 seconds** of data at the 64,000 B/s drain rate.
- The pump task runs every 1 ms on core 1 at priority 6 and keeps up to 512 bytes
  (8 ms worth) staged ahead of the DMA. The USB task runs on core 0, so USB traffic
  cannot delay the pump.
- The DMA never stops: on an empty main ring the pump stages zeros, so the master always
  clocks valid data. Because up to 512 bytes are staged ahead of the DMA, data loaded while
  the master is idle can be preceded by up to 512 already-staged bytes (≈8 ms at the nominal rate).
- Sustained preload throughput is limited by the master's drain rate once the ring is full.
  A master that reads slower than 64,000 B/s will drain the ring proportionally slower.
- The bridge is robust to USB disconnects: the partial frame parser resets on reconnect,
  and buffered data survives.

## Customization

At the top of `src/hello_freertos.c`:

- `BRIDGE_SPI`, `BRIDGE_PIN_TX`/`RX`/`CS`/`SCK`, `BRIDGE_SPI_CPOL`/`BRIDGE_SPI_CPHA` —
  peripheral, pins, mode. `TX` is the slave output (master MISO) and `RX` the slave input.
- `USB_RING_SIZE` — main ring size (default 384 KiB). Keep it smaller than the available RAM.
- `STAGE_RING_BITS` — staging ring size (`1 << STAGE_RING_BITS` bytes). It must stay
  power-of-two and is aligned to its size; the DMA ring limit is 32 KiB.
- `STAGE_PREFETCH`, `PUMP_PERIOD_MS` — real-time tradeoff.

FreeRTOS tunables live in `FreeRTOSConfig_examples_common.h`; the app uses static
allocation only, so new tasks must use `xTaskCreateStatic()`.

USB CDC options are set in `CMakeLists.txt`:

- `CFG_TUD_CDC_RX_BUFSIZE=8192` — larger endpoint buffering for high-rate preloads.
- `PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1` — accept hosts that do not assert DTR.

## File Structure

```
pico2-freertos-template/
├── CMakeLists.txt                        # Build target, static allocation, stdio USB options
├── src/hello_freertos.c                  # Bridge firmware (SPI slave, DMA, USB protocol, tasks)
├── tools/usb_loader.py                   # Reference host loader / STATUS tool
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
| [pyserial](https://pyserial.readthedocs.io/) | host, pip | for `tools/usb_loader.py` |

## FAQ

**Q: `usb_loader.py` cannot find the device**
A: Pass `--port` explicitly and check `lsusb` for `2e8a:0009`. On Linux, make sure your
user is in the `dialout` group (`/dev/ttyACM0` permissions).

**Q: The loader times out with "ring stayed full"**
A: The SPI master is not clocking data out, so the 384 KiB ring cannot drain. Verify SCK,
CS, common ground, and Mode 0 on the master.

**Q: `underrun_events` or `zero_fill` keeps increasing**
A: The host is not supplying data fast enough. Check the loader's progress output and USB
connection. `zero_fill` alone is expected whenever the ring runs empty.

**Q: Building for RP2040 fails**
A: This bridge uses RP2350 DMA endless mode. Use a Pico 2 / Pico 2 W (`PICO_BOARD=pico2`
or `pico2_w`).
