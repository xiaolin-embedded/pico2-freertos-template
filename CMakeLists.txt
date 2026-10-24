# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.12)

# Set the target board BEFORE pulling in the SDK, so the SDK picks up the correct board
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
# pico_sdk_import.cmake auto-finds the SDK from the pico-sdk submodule
include(cmake/pico_sdk_import.cmake)

project(pico2-freertos-template C CXX ASM)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Require SDK version >= 2.3.0
if (PICO_SDK_VERSION_STRING VERSION_LESS "2.3.0")
    message(FATAL_ERROR "Pico SDK version 2.3.0 or higher is required. Found: ${PICO_SDK_VERSION_STRING}")
endif()

# Pull in FreeRTOS-Kernel (must be after pico_sdk_init)
# FreeRTOS_Kernel_import.cmake auto-finds the kernel from the FreeRTOS-Kernel submodule
include(cmake/FreeRTOS_Kernel_import.cmake)

# ============================================================================
# pico2-freertos-template — FreeRTOS SMP with static memory allocation
# ============================================================================
add_executable(pico2-freertos-template
    src/main.c
    src/bridge_protocol.c
)
target_include_directories(pico2-freertos-template PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(pico2-freertos-template PRIVATE
    FreeRTOS-Kernel-Static
    pico_stdlib
    hardware_spi
    hardware_dma
)
target_compile_definitions(pico2-freertos-template PRIVATE
    configSUPPORT_STATIC_ALLOCATION=1
    configSUPPORT_DYNAMIC_ALLOCATION=0
    # Larger CDC RX FIFO: the host preloads data faster than the SPI drain rate.
    CFG_TUD_CDC_RX_BUFSIZE=8192
    # Room for several WRITE responses so the USB task does not block on TX.
    CFG_TUD_CDC_TX_BUFSIZE=4096
    # Accept data from custom hosts that do not assert DTR.
    PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
)
pico_enable_stdio_usb(pico2-freertos-template 1)
pico_add_extra_outputs(pico2-freertos-template)
