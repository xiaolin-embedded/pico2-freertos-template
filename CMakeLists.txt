cmake_minimum_required(VERSION 3.12)

# Set the target board BEFORE pulling in the SDK, so the SDK picks up the correct board
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
# pico_sdk_import.cmake auto-finds the SDK from the pico-sdk submodule
include(cmake/pico_sdk_import.cmake)

project(pico2_freertos_template C CXX ASM)

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
# hello_freertos_one_core — FreeRTOS on 1 core (dynamic heap)
# ============================================================================
add_executable(hello_freertos_one_core
    src/hello_freertos.c
)
target_include_directories(hello_freertos_one_core PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(hello_freertos_one_core PRIVATE
    pico_async_context_freertos
    FreeRTOS-Kernel-Heap4
    pico_stdlib
)
target_compile_definitions(hello_freertos_one_core PRIVATE
    configNUMBER_OF_CORES=1
)
pico_add_extra_outputs(hello_freertos_one_core)

# ============================================================================
# hello_freertos_two_cores — FreeRTOS SMP on 2 cores (dynamic heap)
# ============================================================================
add_executable(hello_freertos_two_cores
    src/hello_freertos.c
)
target_include_directories(hello_freertos_two_cores PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(hello_freertos_two_cores PRIVATE
    pico_async_context_freertos
    FreeRTOS-Kernel-Heap4
    pico_stdlib
)
pico_add_extra_outputs(hello_freertos_two_cores)

# ============================================================================
# hello_freertos_static_allocation — FreeRTOS SMP with static memory allocation
# ============================================================================
add_executable(hello_freertos_static_allocation
    src/hello_freertos.c
)
target_include_directories(hello_freertos_static_allocation PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(hello_freertos_static_allocation PRIVATE
    pico_async_context_freertos
    FreeRTOS-Kernel-Static
    pico_stdlib
)
target_compile_definitions(hello_freertos_static_allocation PRIVATE
    configSUPPORT_STATIC_ALLOCATION=1
    configSUPPORT_DYNAMIC_ALLOCATION=0
)
pico_add_extra_outputs(hello_freertos_static_allocation)
