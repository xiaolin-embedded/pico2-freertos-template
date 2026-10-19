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

- All app code lives in **`src/hello_freertos.c`** — a single translation unit. There is **no `include/` dir**; the repo root is the include path (that's how `FreeRTOSConfig.h` is found).
- **App is a USB CDC → SPI0 slave bridge.** External master reads 64 B every 1 ms (Mode 0, MSB first). Slave output (master MISO) = GP19/SPI0 TX, slave input (master MOSI) = GP16/SPI0 RX, CS = GP17, SCK = GP18. The peripheral's TX/RX directions do not flip with the bus role.
- Startup chain: `main()` → `stdio_init_all()` → `bridge_hardware_init()` (SPI slave + DMA channel config, DMA **not** started) → banner `printf` → `vLaunch()` → tasks → `vTaskStartScheduler()`.
- Tasks, all static: `UsbRx` (prio 4, core 0; parses framed USB protocol, writes 384 KiB SPSC ring), `SpiPump` (prio 6, core 1; 1 ms period, tracks DMA position, keeps an 8 KiB staging ring ≥512 B ahead, zero-fills on empty), `Blink` (prio 1, core 0; on-board LED).
- Data path: USB ring (384 KiB, C11 atomics head/tail, non-power-of-two) → staging ring (8 KiB, power-of-two, size-aligned) → DMA endless mode (`dma_encode_endless_transfer_count()`, ring mode wraps read address) → SPI TX FIFO. The pump starts the DMA lazily on the first host data so the first frame is not preceded by zeros.
- Link libs: `FreeRTOS-Kernel-Static`, `pico_stdlib`, `hardware_spi`, `hardware_dma`. USB stdio enabled via `pico_enable_stdio_usb`; `pico_async_context_freertos` is **not** used.
- Build-time defines: `CFG_TUD_CDC_RX_BUFSIZE=8192`, `PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1`. RP2350-only (`#error` on RP2040).
- Wire protocol (host tool: `tools/usb_loader.py`): `AA 55 | type | seq[2] | len[2] | payload | crc16[2]`, CRC-16/CCITT-FALSE over type..payload. WRITE=0x01 → ACK 0x81 (`accepted, used, free`); STATUS=0x02 → 0x82 (10 × u32 counters).
- Peripherals used: USB CDC, SPI0 slave, one DMA channel, on-board LED (GPIO 25; `pico2_w` uses cyw43). No I2C/PWM/ADC/PIO.

## Conventions & constraints

- **Static allocation only**: `configSUPPORT_STATIC_ALLOCATION=1` and `configSUPPORT_DYNAMIC_ALLOCATION=0` are forced via compile definitions in root `CMakeLists.txt` (the header defaults are the opposite). New tasks **must** use `xTaskCreateStatic()` with static `StackType_t`/`StaticTask_t` buffers. `configTOTAL_HEAP_SIZE` (128 KB) is **inert** in this build — no `heap_*.c` is linked.
- FreeRTOS config split: `FreeRTOSConfig.h` is a thin wrapper — edit **`FreeRTOSConfig_examples_common.h`** for tunables (tick 1000 Hz, `configMAX_PRIORITIES=32`, `configNUMBER_OF_CORES=2` SMP, `configUSE_CORE_AFFINITY=1`).
- New source files: add to `add_executable(...)` in root `CMakeLists.txt`; task functions and creation belong in `src/hello_freertos.c` (template convention).
- `pico-sdk/` and `FreeRTOS-Kernel/` are vendor submodules — **read-only**, never edit. The FreeRTOS RP2350 SMP port lives at `FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2350_ARM_NTZ/` and defines the `FreeRTOS-Kernel-Static` library target.
- No CI workflows, no tests, no linter config in this repo.
