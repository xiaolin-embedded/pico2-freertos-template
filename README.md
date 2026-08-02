# Pico 2 FreeRTOS Template

A FreeRTOS template project for Raspberry Pi Pico 2 (RP2350 ARM-S), extracted from [pico-examples](https://github.com/raspberrypi/pico-examples).

## Overview

This template provides a single FreeRTOS build target, with source code in `src/hello_freertos.c`:

| Build Target | Description | FreeRTOS Memory Management |
|-------------|------|---------------------------|
| `pico2-freertos-template` | Dual-core FreeRTOS SMP + static memory allocation | Static |

The program will:
1. Create an LED blinking task (using the Pico 2 on-board LED)
2. Create a main task that prints "Hello from main task count=N" in a loop
3. Create an async worker that periodically prints "Hello from worker count=N"
4. Display which core each task is running on (in SMP mode)

## Quick Start

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential git

# macOS
brew install cmake arm-none-eabi-gcc git
```

### Clone and Build

```bash
# 1. Clone the repository (recursively pull all submodules)
git clone --recurse-submodules <your-repo-url>
cd pico2-freertos-template

# 2. If the repository was already cloned, initialize submodules
# git submodule update --init --recursive

# 3. Build
mkdir build && cd build
cmake ..
make -j4
```

### Flashing to Pico 2

**Method 1: UF2 Drag-and-Drop**
1. Hold down the BOOTSEL button on Pico 2
2. Connect to the computer via USB
3. Release the button; Pico 2 will appear as a USB drive
4. Drag `build/pico2-freertos-template.uf2` onto the drive

**Method 2: picotool**
```bash
picotool load build/pico2-freertos-template.uf2
picotool reboot
```

### View Serial Output

```bash
# Linux
screen /dev/ttyACM0 115200

# macOS
screen /dev/tty.usbmodem* 115200

# Exit: Ctrl+A then K then Y
```

## Dependencies

This project manages all dependencies via git submodules — no environment variables needed:

| Dependency | Source | Version |
|-----------|------|---------|
| [pico-sdk](https://github.com/raspberrypi/pico-sdk) | submodule `pico-sdk/` | 2.3.0 |
| [FreeRTOS-Kernel](https://github.com/raspberrypi/FreeRTOS-Kernel) | submodule `FreeRTOS-Kernel/` | RP2350 port support |
| [tinyusb](https://github.com/hathach/tinyusb) | pico-sdk submodule | 0.18.0 |
| [lwip](https://github.com/lwip-tcpip/lwip) | pico-sdk submodule | STABLE-2_2_1 |
| [mbedtls](https://github.com/Mbed-TLS/mbedtls) | pico-sdk submodule | v3.6.6 |
| [btstack](https://github.com/bluekitchen/btstack) | pico-sdk submodule | v1.8.2 |
| [cyw43-driver](https://github.com/georgerobotics/cyw43-driver) | pico-sdk submodule | v1.1.1 |

All submodules are pulled at once with `git submodule update --init --recursive`.

## File Structure

```
pico2-freertos-template/
├── CMakeLists.txt                        # Top-level CMake, defines the build target
├── src/                                  # User application code
│   └── hello_freertos.c                  #   FreeRTOS example source
├── cmake/                                # CMake helper scripts
│   ├── pico_sdk_import.cmake             #   Auto-locates pico-sdk submodule
│   └── FreeRTOS_Kernel_import.cmake      #   Auto-locates FreeRTOS-Kernel submodule
├── FreeRTOSConfig.h                      # FreeRTOS configuration entry point
├── FreeRTOSConfig_examples_common.h      # Full FreeRTOS configuration parameters
├── .gitignore
├── pico-sdk/              (submodule)    # Raspberry Pi Pico SDK 2.3.0
│   ├── lib/tinyusb/       (submodule)    #   USB stack
│   ├── lib/lwip/          (submodule)    #   TCP/IP stack
│   ├── lib/mbedtls/       (submodule)    #   TLS encryption library
│   ├── lib/btstack/       (submodule)    #   Bluetooth stack
│   └── lib/cyw43-driver/  (submodule)    #   WiFi chip driver
└── FreeRTOS-Kernel/       (submodule)    # FreeRTOS kernel (RP2350 port)
```

## Customization Guide

### Modifying FreeRTOS Configuration

Edit `FreeRTOSConfig_examples_common.h` to adjust:
- `configTICK_RATE_HZ` — System tick frequency (default 1000 Hz)
- `configMAX_PRIORITIES` — Maximum number of task priorities
- `configMINIMAL_STACK_SIZE` — Minimum task stack size

### Adding New FreeRTOS Tasks

1. Add your task function in `src/hello_freertos.c`
2. Create the task in `vLaunch()` using `xTaskCreateStatic()`
3. If adding new source files, add them to `add_executable` in `CMakeLists.txt`

### Specifying SDK Path

By default, the SDK is auto-located from the `pico-sdk/` submodule. To use a different SDK:

```bash
cmake .. -DPICO_SDK_PATH=/path/to/your/pico-sdk
```

### Building for Other Pico Boards

```bash
# Raspberry Pi Pico (RP2040)
cmake .. -DPICO_BOARD=pico

# Pico 2 W (RP2350 + WiFi)
cmake .. -DPICO_BOARD=pico2_w
```

## FAQ

**Q: cmake reports "FreeRTOS location was not specified"**
A: Make sure `git submodule update --init --recursive` has been run, and the `FreeRTOS-Kernel/` directory exists.

**Q: cmake reports "PICO_SDK_PATH not found"**
A: Make sure `git submodule update --init --recursive` has been run, and the `pico-sdk/` directory exists. Alternatively, manually specify `-DPICO_SDK_PATH=/path/to/pico-sdk`.

**Q: No serial output**
A: Make sure Pico 2 is connected to the computer via USB, check `/dev/ttyACM0` permissions, or try `sudo chmod 666 /dev/ttyACM0`.
