# Pico 2 FreeRTOS Template

适用于 Raspberry Pi Pico 2 (RP2350 ARM-S) 的 FreeRTOS 模板工程，从 [pico-examples](https://github.com/raspberrypi/pico-examples) 提取。

## 工程简介

本模板提供 3 个 FreeRTOS 编译变体，共享同一份源码 `hello_freertos.c`：

| 编译目标 | 说明 | FreeRTOS 内存管理 |
|---------|------|-------------------|
| `hello_freertos_one_core` | 单核 FreeRTOS | Heap-4 (动态) |
| `hello_freertos_two_cores` | 双核 FreeRTOS SMP | Heap-4 (动态) |
| `hello_freertos_static_allocation` | 双核 + 静态内存分配 | Static |

程序运行后会：
1. 创建 LED 闪烁任务（使用 Pico 2 板载 LED）
2. 创建主任务循环打印 "Hello from main task count=N"
3. 创建异步 worker 定时打印 "Hello from worker count=N"
4. 在双核模式下显示每个任务运行在哪个核心

## 快速开始

### 前置条件

```bash
# Ubuntu/Debian
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential git

# macOS
brew install cmake arm-none-eabi-gcc git
```

### 克隆与编译

```bash
# 1. 克隆仓库（递归拉取所有 submodule）
git clone --recurse-submodules <your-repo-url>
cd pico2_freertos_template

# 2. 如果是已克隆的仓库，需要初始化 submodule
# git submodule update --init --recursive

# 3. 编译
mkdir build && cd build
cmake ..
make -j4                          # 编译所有 3 个目标

# 或只编译特定目标
make -j4 hello_freertos_two_cores
```

### 烧录到 Pico 2

**方法一：UF2 拖拽**
1. 按住 Pico 2 的 BOOTSEL 按钮
2. 通过 USB 连接电脑
3. 松开按钮，Pico 2 会以 U 盘形式出现
4. 将 `build/hello_freertos_two_cores.uf2` 拖入 U 盘

**方法二：picotool**
```bash
picotool load build/hello_freertos_two_cores.uf2
picotool reboot
```

### 查看串口输出

```bash
# Linux
screen /dev/ttyACM0 115200

# macOS
screen /dev/tty.usbmodem* 115200

# 退出: Ctrl+A 然后 K 然后 Y
```

## 依赖说明

本项目通过 git submodule 管理所有依赖，无需手动设置环境变量：

| 依赖 | 来源 | 版本 |
|------|------|------|
| [pico-sdk](https://github.com/raspberrypi/pico-sdk) | submodule `pico-sdk/` | 2.3.0 |
| [FreeRTOS-Kernel](https://github.com/raspberrypi/FreeRTOS-Kernel) | submodule `FreeRTOS-Kernel/` | RP2350 端口支持 |
| [tinyusb](https://github.com/hathach/tinyusb) | pico-sdk 子模块 | 0.18.0 |
| [lwip](https://github.com/lwip-tcpip/lwip) | pico-sdk 子模块 | STABLE-2_2_1 |
| [mbedtls](https://github.com/Mbed-TLS/mbedtls) | pico-sdk 子模块 | v3.6.6 |
| [btstack](https://github.com/bluekitchen/btstack) | pico-sdk 子模块 | v1.8.2 |
| [cyw43-driver](https://github.com/georgerobotics/cyw43-driver) | pico-sdk 子模块 | v1.1.1 |

所有子模块通过 `git submodule update --init --recursive` 一次性拉取。

## 文件结构

```
pico2_freertos_template/
├── CMakeLists.txt                        # 顶层 CMake，定义 3 个编译目标
├── hello_freertos.c                      # FreeRTOS 示例源码
├── FreeRTOSConfig.h                      # FreeRTOS 配置入口
├── FreeRTOSConfig_examples_common.h      # 完整 FreeRTOS 配置参数
├── FreeRTOS_Kernel_import.cmake          # 自动定位 FreeRTOS-Kernel submodule
├── pico_sdk_import.cmake                 # 自动定位 pico-sdk submodule
├── .gitignore
├── pico-sdk/              (submodule)    # Raspberry Pi Pico SDK 2.3.0
│   ├── lib/tinyusb/       (submodule)    #   USB 协议栈
│   ├── lib/lwip/          (submodule)    #   TCP/IP 协议栈
│   ├── lib/mbedtls/       (submodule)    #   TLS 加密库
│   ├── lib/btstack/       (submodule)    #   蓝牙协议栈
│   └── lib/cyw43-driver/  (submodule)    #   WiFi 芯片驱动
└── FreeRTOS-Kernel/       (submodule)    # FreeRTOS 内核 (RP2350 端口)
```

## 自定义指南

### 修改 FreeRTOS 配置

编辑 `FreeRTOSConfig_examples_common.h`，可调整：
- `configTICK_RATE_HZ` — 系统滴答频率 (默认 1000 Hz)
- `configTOTAL_HEAP_SIZE` — 动态堆大小 (默认 128 KB)
- `configMAX_PRIORITIES` — 最大任务优先级数
- `configMINIMAL_STACK_SIZE` — 最小任务栈大小

### 添加新的 FreeRTOS 任务

1. 在 `hello_freertos.c` 中添加任务函数
2. 在 `vLaunch()` 中用 `xTaskCreate()` 创建任务
3. 如需新的编译变体，在 `CMakeLists.txt` 中添加新的 `add_executable` 块

### 指定 SDK 路径

默认自动从 `pico-sdk/` submodule 定位。如需使用其他 SDK：

```bash
cmake .. -DPICO_SDK_PATH=/path/to/your/pico-sdk
```

### 编译给其他 Pico 板子

```bash
# Raspberry Pi Pico (RP2040)
cmake .. -DPICO_BOARD=pico

# Pico 2 W (RP2350 + WiFi)
cmake .. -DPICO_BOARD=pico2_w
```

## 常见问题

**Q: cmake 报 "FreeRTOS location was not specified"**
A: 确保 `git submodule update --init --recursive` 已执行，`FreeRTOS-Kernel/` 目录存在。

**Q: cmake 报 "PICO_SDK_PATH not found"**
A: 确保 `git submodule update --init --recursive` 已执行，`pico-sdk/` 目录存在。或手动指定 `-DPICO_SDK_PATH=/path/to/pico-sdk`。

**Q: 串口没有输出**
A: 确认 Pico 2 通过 USB 连接到电脑，检查 `/dev/ttyACM0` 权限，或尝试 `sudo chmod 666 /dev/ttyACM0`。
