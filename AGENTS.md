# AGENTS.md

FreeRTOS-based USB-to-SPI-slave bridge for Raspberry Pi Pico 2 (RP2350, ARM-S / Cortex-M33). Single build target `pico2-freertos-template`: dual-core FreeRTOS SMP with **static-only** memory allocation. See `README.md` for user-facing docs.

## Build & Flash

No environment variables required — `pico-sdk/` and `FreeRTOS-Kernel/` are git submodules auto-detected by `cmake/pico_sdk_import.cmake` and `cmake/FreeRTOS_Kernel_import.cmake`.

```bash
# one-time dependency setup
git submodule update --init --recursive

# build
cmake -S . -B build
cmake --build build -j4

# flash (BOOTSEL mode for UF2 / picotool)
picotool load -fx build/pico2-freertos-template.uf2
```

- Board is baked into the top of root `CMakeLists.txt`: `set(PICO_BOARD pico2 CACHE STRING ...)` → `PICO_PLATFORM=rp2350-arm-s` (**ARM Cortex-M33, not RISC-V**). Override with `-DPICO_BOARD=pico2_w` etc.
- Outputs via `pico_add_extra_outputs`: `.elf`, `.uf2`, `.bin`, `.hex`, `.dis`, `.elf.map`.
- System `arm-none-eabi-gcc` (13.x) + cmake >= 3.12 + ninja/make is sufficient. The `~/.pico-sdk/` toolchain-manager (Ninja, GCC 15.2) is installed by the VS Code Pico extension and is **not** required — though the existing `build/` dir was configured that way (Ninja, Release).
- Gotcha: if `PICO_SDK_PATH` is set in the environment, it **overrides** submodule auto-detection. The SDK version check requires >= 2.3.0 (`FATAL_ERROR` otherwise).
- Root `pico_sdk_import.cmake` is gitignored (VS Code extension auto-generates it); the tracked copy is `cmake/pico_sdk_import.cmake`.

## Architecture

- App code lives in **`src/main.c`**; the USB packet protocol is a separate, SDK-independent module in **`src/bridge_protocol.c` / `src/bridge_protocol.h`**. There is **no `include/` dir**; headers sit next to their sources, and the repo root remains the include path (that's how `FreeRTOSConfig.h` is found). The full protocol design and adaptation rationale are in `docs/bridge-protocol-design.md`.
- **App is a USB CDC → SPI0 slave bridge.** External master reads 64 B every 1 ms (**Mode 1**, MSB first). Slave output (master MISO) = GP19/SPI0 TX, slave input (master MOSI) = GP16/SPI0 RX, CS = GP17, SCK = GP18. The peripheral's TX/RX directions do not flip with the bus role. Mode 1 is required: with SPH=0 the PL022 freezes its shift register while CS is low and cannot load the next byte from the TX FIFO.
- Startup chain: `main()` → `stdio_init_all()` → `bridge_hardware_init()` (SPI slave + DMA channel config, DMA **not** started) → banner `printf` → `vLaunch()` → tasks → `vTaskStartScheduler()`.
- Tasks, all static: core 0 runs the bridge — `UsbRx` (prio 4; parses the USB packet protocol, writes 384 KiB SPSC ring) and `SpiPump` (prio 6; 1 ms period, tracks DMA position, keeps an 8 KiB staging ring ≥512 B ahead, zero-fills on empty). Core 1 runs only `Blink` (prio 1; on-board LED, 500 ms toggle).
- Data path: USB ring (384 KiB, C11 atomics head/tail, non-power-of-two) → staging ring (8 KiB, power-of-two, size-aligned) → DMA endless mode (`dma_encode_endless_transfer_count()`, ring mode wraps read address) → SPI TX FIFO. The pump starts the DMA lazily on the first host data so the first frame is not preceded by zeros. It refills in whole 64-byte units because the DMA cursor is byte-granular; this keeps main-ring free space aligned for partial WRITE accepts.
- Link libs: `FreeRTOS-Kernel-Static`, `pico_stdlib`, `hardware_spi`, `hardware_dma`. USB stdio enabled via `pico_enable_stdio_usb`; `pico_async_context_freertos` is **not** used.
- Build-time defines: `CFG_TUD_CDC_RX_BUFSIZE=8192`, `CFG_TUD_CDC_TX_BUFSIZE=4096`, `PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1`. RP2350-only (`#error` on RP2040).
- Wire protocol (host tools: `tools/usb_loader.py`, `tools/usb_loader.cpp`; bridge-specific, with no flash or upgrade commands): `0xAA | chksum | len[2] | payload[len] | 0x55`, additive checksum with total sum 0. Commands: WRITE_DATA 0x01 (`seq[2], offset[4], data[N]`, N multiple of 64 and ≤960) → response with `accepted`, `stream_accepted`, `free`, `err`; GET_STATUS 0x02; CLEAR_COUNTERS 0x03; GET_VERSION 0x04. The 32-bit `offset` equals `stream_accepted` and makes lost-ACK retries idempotent. Ring full → `ERR_RING_FULL` (0x0201) + accepted count; non-aligned → `ERR_NOT_ALIGNED` (0x0202); offset mismatch → `ERR_OFFSET` (0x0203).
- Peripherals used: USB CDC, SPI0 slave, one DMA channel, on-board LED (GPIO 25; `pico2_w` uses cyw43). No I2C/PWM/ADC/PIO.

## Conventions & constraints

- **Static allocation only**: `configSUPPORT_STATIC_ALLOCATION=1` and `configSUPPORT_DYNAMIC_ALLOCATION=0` are forced via compile definitions in root `CMakeLists.txt` (the header defaults are the opposite). New tasks **must** use `xTaskCreateStatic()` with static `StackType_t`/`StaticTask_t` buffers. `configTOTAL_HEAP_SIZE` (128 KB) is **inert** in this build — no `heap_*.c` is linked.
- FreeRTOS config split: `FreeRTOSConfig.h` is a thin wrapper — edit **`FreeRTOSConfig_examples_common.h`** for tunables (tick 1000 Hz, `configMAX_PRIORITIES=32`, `configNUMBER_OF_CORES=2` SMP, `configUSE_CORE_AFFINITY=1`).
- New source files: add to `add_executable(...)` in root `CMakeLists.txt`; task functions and creation belong in `src/main.c`, while wire-protocol code belongs in `src/bridge_protocol.c`.
- `pico-sdk/` and `FreeRTOS-Kernel/` are vendor submodules — **read-only**, never edit. The FreeRTOS RP2350 SMP port lives at `FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2350_ARM_NTZ/` and defines the `FreeRTOS-Kernel-Static` library target.
- No CI workflows, no tests, no linter config in this repo.
