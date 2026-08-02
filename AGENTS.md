# AGENTS.md

FreeRTOS template for Raspberry Pi Pico 2 (RP2350, ARM-S / Cortex-M33). Single build target `pico2-freertos-template`: dual-core FreeRTOS SMP with **static-only** memory allocation. See `README.md` for user-facing docs.

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
- Root `pico_sdk_import.cmake` is gitignored (VS Code extension auto-generates it); the tracked copy is `cmake/pico_sdk_import.cmake`. README's file-tree still lists the root one — stale.

## Architecture

- All app code lives in **`src/hello_freertos.c`** — a single translation unit. There is **no `include/` dir**; the repo root is the include path (that's how `FreeRTOSConfig.h` is found).
- Startup chain: `main()` → `stdio_init_all()` (UART0/USB-CDC serial) → `vLaunch()` → `xTaskCreateStatic(MainThread)` → `vTaskStartScheduler()`.
- `main_task` (MainThread, prio 2, pinned to core 1 in SMP) spawns `BlinkThread` (prio 1) and the SDK async-context worker task (prio 4, created inside `async_context_freertos_init()`).
- Peripherals used: only stdio serial + on-board LED (GPIO 25 via `PICO_DEFAULT_LED_PIN`). A `pico2_w` build (`-DPICO_BOARD=pico2_w`) switches the LED to the cyw43 WiFi chip. No DMA/I2C/SPI/PWM/ADC/PIO.

## Conventions & constraints

- **Static allocation only**: `configSUPPORT_STATIC_ALLOCATION=1` and `configSUPPORT_DYNAMIC_ALLOCATION=0` are forced via compile definitions in root `CMakeLists.txt` (the header defaults are the opposite). New tasks **must** use `xTaskCreateStatic()` with static `StackType_t`/`StaticTask_t` buffers. `configTOTAL_HEAP_SIZE` (128 KB) is **inert** in this build — no `heap_*.c` is linked.
- FreeRTOS config split: `FreeRTOSConfig.h` is a thin wrapper — edit **`FreeRTOSConfig_examples_common.h`** for tunables (tick 1000 Hz, `configMAX_PRIORITIES=32`, `configNUMBER_OF_CORES=2` SMP, `configUSE_CORE_AFFINITY=1`).
- New source files: add to `add_executable(...)` in root `CMakeLists.txt`; task functions and creation belong in `src/hello_freertos.c` (template convention).
- `pico-sdk/` and `FreeRTOS-Kernel/` are vendor submodules — **read-only**, never edit. The FreeRTOS RP2350 SMP port lives at `FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2350_ARM_NTZ/` and defines the `FreeRTOS-Kernel-Static` library target.
- No CI workflows, no tests, no linter config in this repo.
