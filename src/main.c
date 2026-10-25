/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * USB CDC -> SPI slave bridge for RP2350.
 *
 * Packet format (compact framing with an additive checksum):
 *   0xAA | chksum | len_lo | len_hi | payload[len] | 0x55
 *   chksum makes the 8-bit sum of the complete packet zero. The length is
 *   16-bit so one packet can carry several 64-byte frames.
 *
 * Commands (payload[0], payload[1] reserved in requests):
 *   0x01 WRITE_DATA  seq[2], offset[4], data[N]  (N % 64 == 0, N <= 960)
 *   0x02 GET_STATUS  no arguments
 *   0x03 CLEAR_COUNTERS
 *   0x04 GET_VERSION
 * Responses set payload[1] to STATUS_*; WRITE_DATA reports accepted bytes
 * and the device stream offset so a retry after a lost ACK is idempotent.
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bridge_protocol.h"

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

#if !PICO_RP2350
#error "This bridge requires RP2350 DMA endless mode"
#endif

// ---------------------------------------------------------------------------
// SPI slave configuration
// ---------------------------------------------------------------------------

#ifndef BRIDGE_SPI
#define BRIDGE_SPI          spi0
#endif
// In slave mode the peripheral's TX pin is the data output (master MISO) and
// its RX pin is the data input (master MOSI), regardless of the bus role.
#ifndef BRIDGE_PIN_TX
#define BRIDGE_PIN_TX       19
#endif
#ifndef BRIDGE_PIN_RX
#define BRIDGE_PIN_RX       16
#endif
#ifndef BRIDGE_PIN_CS
#define BRIDGE_PIN_CS       17
#endif
#ifndef BRIDGE_PIN_SCK
#define BRIDGE_PIN_SCK      18
#endif
#ifndef BRIDGE_SPI_CPOL
#define BRIDGE_SPI_CPOL     SPI_CPOL_0
#endif
// PL022 freezes the slave shift register while CS is low when SPH=0, so a
// master that holds CS low for a 64-byte transaction needs Mode 1.
#ifndef BRIDGE_SPI_CPHA
#define BRIDGE_SPI_CPHA     SPI_CPHA_1
#endif

// ---------------------------------------------------------------------------
// Buffer and protocol sizing
// ---------------------------------------------------------------------------

// Data preloaded by the host, drained by the SPI master at 1 kHz * 64 B.
#define USB_RING_SIZE       (384u * 1024u)
// Short staging ring read by DMA. Kept ahead of the DMA so a delayed pump
// cannot make the SPI master read stale bytes.
#define STAGE_RING_BITS     13u
#define STAGE_RING_SIZE     (1u << STAGE_RING_BITS)
#define STAGE_PREFETCH      512u

#define USB_CHUNK_SIZE      1024u
#define USB_DRAIN_CHUNKS    8

// ---------------------------------------------------------------------------
// Task configuration
// ---------------------------------------------------------------------------

#define USB_TASK_PRIORITY       (tskIDLE_PRIORITY + 4UL)
#define PUMP_TASK_PRIORITY      (tskIDLE_PRIORITY + 6UL)
#define BLINK_TASK_PRIORITY     (tskIDLE_PRIORITY + 1UL)

// Core 0 runs the whole bridge (USB + SPI pump); core 1 only blinks the LED.
#define USB_TASK_CORE           0
#define PUMP_TASK_CORE          0
#define BLINK_TASK_CORE         1

#define USB_TASK_STACK_SIZE     1024
#define PUMP_TASK_STACK_SIZE    512
#define BLINK_TASK_STACK_SIZE   512

#define PUMP_PERIOD_MS          1
// Toggle the status LED every 500 ms (1 Hz square wave).
#define BLINK_PERIOD_MS         500

#ifndef USE_LED
#define USE_LED                 1
#endif

_Static_assert((STAGE_RING_SIZE & (STAGE_RING_SIZE - 1u)) == 0, "stage ring must be power of two");
_Static_assert((USB_RING_SIZE % BRIDGE_FRAME_BYTES) == 0, "main ring must be frame aligned");
_Static_assert((STAGE_RING_SIZE % BRIDGE_FRAME_BYTES) == 0, "stage ring must be frame aligned");
_Static_assert((STAGE_PREFETCH % BRIDGE_FRAME_BYTES) == 0, "prefetch must be frame aligned");
_Static_assert((BRIDGE_WRITE_MAX_DATA % BRIDGE_FRAME_BYTES) == 0, "write payload must stay frame aligned");
_Static_assert(BRIDGE_PACKET_MAX_PAYLOAD >= BRIDGE_WRITE_HEADER_SIZE + BRIDGE_WRITE_MAX_DATA, "packet buffer too small");

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t usb_ring[USB_RING_SIZE];

// DMA ring mode wraps the read address on a (1 << STAGE_RING_BITS) boundary,
// so the buffer must be aligned to its size.
static uint8_t stage_ring[STAGE_RING_SIZE] __attribute__((aligned(STAGE_RING_SIZE)));

static uint8_t usb_chunk[USB_CHUNK_SIZE];
static uint8_t pump_chunk[256];

// Static protocol instance (the payload buffer is its largest member).
static bridge_protocol_t protocol;

// Single producer (USB task) / single consumer (pump task) indices. The totals
// are monotonic and always interpreted as 32-bit differences.
static _Atomic uint32_t ring_head;
static _Atomic uint32_t ring_tail;
static uint32_t ring_head_idx;
static uint32_t ring_tail_idx;

static uint32_t stage_produced;
static volatile uint32_t stage_ahead_bytes;
static volatile uint32_t spi_consumed_bytes;
static volatile uint32_t zero_fill_bytes;
static volatile uint32_t underrun_events;

static uint dma_channel;

// ---------------------------------------------------------------------------
// Large ring buffer (USB producer -> pump consumer)
// ---------------------------------------------------------------------------

static size_t usb_ring_write(const uint8_t *src, size_t len) {
    uint32_t head = atomic_load_explicit(&ring_head, memory_order_relaxed);
    uint32_t tail = atomic_load_explicit(&ring_tail, memory_order_acquire);
    size_t space = USB_RING_SIZE - (size_t)(head - tail);

    if (len > space) {
        len = space;
    }

    size_t first = MIN(len, USB_RING_SIZE - ring_head_idx);
    memcpy(&usb_ring[ring_head_idx], src, first);
    if (len > first) {
        memcpy(&usb_ring[0], src + first, len - first);
    }

    ring_head_idx += (uint32_t)len;
    if (ring_head_idx >= USB_RING_SIZE) {
        ring_head_idx -= USB_RING_SIZE;
    }
    atomic_store_explicit(&ring_head, head + (uint32_t)len, memory_order_release);
    return len;
}

static size_t usb_ring_read(uint8_t *dst, size_t len) {
    uint32_t head = atomic_load_explicit(&ring_head, memory_order_acquire);
    uint32_t tail = atomic_load_explicit(&ring_tail, memory_order_relaxed);
    size_t available = (size_t)(head - tail);

    if (len > available) {
        len = available;
    }

    size_t first = MIN(len, USB_RING_SIZE - ring_tail_idx);
    memcpy(dst, &usb_ring[ring_tail_idx], first);
    if (len > first) {
        memcpy(dst + first, &usb_ring[0], len - first);
    }

    ring_tail_idx += (uint32_t)len;
    if (ring_tail_idx >= USB_RING_SIZE) {
        ring_tail_idx -= USB_RING_SIZE;
    }
    atomic_store_explicit(&ring_tail, tail + (uint32_t)len, memory_order_release);
    return len;
}

static void usb_ring_stats(uint32_t *used, uint32_t *free_bytes) {
    uint32_t head = atomic_load_explicit(&ring_head, memory_order_acquire);
    uint32_t tail = atomic_load_explicit(&ring_tail, memory_order_acquire);
    uint32_t value = head - tail;
    *used = value;
    *free_bytes = USB_RING_SIZE - value;
}

static bool usb_ring_has_data(void) {
    uint32_t head = atomic_load_explicit(&ring_head, memory_order_acquire);
    uint32_t tail = atomic_load_explicit(&ring_tail, memory_order_acquire);
    return head != tail;
}

// ---------------------------------------------------------------------------
// Protocol callbacks (the wire protocol lives in bridge_protocol.c)
// ---------------------------------------------------------------------------

static void protocol_send(const uint8_t *data, size_t len) {
    stdio_usb.out_chars((const char *)data, (int)len);
}

static size_t protocol_write(const uint8_t *data, size_t len) {
    return usb_ring_write(data, len);
}

static void protocol_get_app_stats(bridge_app_stats_t *stats) {
    usb_ring_stats(&stats->used, &stats->free_bytes);
    stats->capacity = USB_RING_SIZE;
    stats->stage_ahead = stage_ahead_bytes;
    stats->spi_consumed = spi_consumed_bytes;
    stats->zero_fill = zero_fill_bytes;
    stats->underrun_events = underrun_events;
}

static void protocol_clear_app_counters(void) {
    zero_fill_bytes = 0;
    underrun_events = 0;
}

static const bridge_protocol_ops_t protocol_ops = {
    .send = protocol_send,
    .write = protocol_write,
    .get_app_stats = protocol_get_app_stats,
    .clear_app_counters = protocol_clear_app_counters,
};

// ---------------------------------------------------------------------------
// Staging ring (pump producer -> DMA consumer)
// ---------------------------------------------------------------------------

static void stage_write(const uint8_t *src, size_t len) {
    size_t idx = stage_produced & (STAGE_RING_SIZE - 1u);
    size_t first = MIN(len, STAGE_RING_SIZE - idx);
    memcpy(&stage_ring[idx], src, first);
    if (len > first) {
        memcpy(&stage_ring[0], src + first, len - first);
    }
    stage_produced += (uint32_t)len;
    // The DMA reads this buffer independently; complete the stores first.
    __dsb();
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

static void usb_rx_task(__unused void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    bool was_connected = false;

    for (;;) {
        bool connected = stdio_usb_connected();
        if (connected && !was_connected) {
            bridge_protocol_reset_parser(&protocol);
        }
        was_connected = connected;

        for (int i = 0; i < USB_DRAIN_CHUNKS; i++) {
            int count = stdio_usb.in_chars((char *)usb_chunk, sizeof(usb_chunk));
            if (count <= 0) {
                break;
            }
            bridge_protocol_feed(&protocol, usb_chunk, (size_t)count);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

static void spi_pump_task(__unused void *params) {
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_rd = 0;
    uint32_t consumed = 0;
    bool dma_started = false;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PUMP_PERIOD_MS));

        if (dma_started) {
            uint32_t rd = dma_channel_hw_addr(dma_channel)->read_addr & (STAGE_RING_SIZE - 1u);
            consumed += (rd - last_rd) & (STAGE_RING_SIZE - 1u);
            last_rd = rd;
            spi_consumed_bytes = consumed;
        }

        int32_t ahead = (int32_t)(stage_produced - consumed);
        if (ahead < 0) {
            // The DMA caught up with the pump; resynchronise and report it.
            underrun_events++;
            stage_produced = consumed;
            ahead = 0;
        }

        bool has_data = usb_ring_has_data();
        if (dma_started) {
            if ((uint32_t)ahead < STAGE_PREFETCH) {
                // The DMA read cursor advances one byte at a time, so `ahead`
                // is not frame aligned. Round the refill down: the main-ring
                // tail must only move in whole 64-byte frames, otherwise a
                // partial WRITE accept could split a frame.
                uint32_t fill = (STAGE_PREFETCH - (uint32_t)ahead) &
                                ~(BRIDGE_FRAME_BYTES - 1u);
                while (fill) {
                    size_t chunk = fill > sizeof(pump_chunk) ? sizeof(pump_chunk) : fill;
                    size_t got = usb_ring_read(pump_chunk, chunk);
                    if (got < chunk) {
                        memset(pump_chunk + got, 0, chunk - got);
                        zero_fill_bytes += (uint32_t)(chunk - got);
                    }
                    stage_write(pump_chunk, chunk);
                    fill -= (uint32_t)chunk;
                }
            }
        } else if (has_data) {
            // Start only once there is real data, so the first frame is not
            // preceded by a prefetch window of zeros.
            uint32_t fill = STAGE_PREFETCH;
            while (fill) {
                size_t chunk = fill > sizeof(pump_chunk) ? sizeof(pump_chunk) : fill;
                size_t got = usb_ring_read(pump_chunk, chunk);
                if (got == 0) {
                    break;
                }
                stage_write(pump_chunk, got);
                fill -= (uint32_t)got;
            }
            if (stage_produced > 0) {
                last_rd = dma_channel_hw_addr(dma_channel)->read_addr & (STAGE_RING_SIZE - 1u);
                dma_channel_start(dma_channel);
                dma_started = true;
            }
        }
        stage_ahead_bytes = stage_produced - consumed;

        // The bridge does not use MOSI; drain RX so the PL022 does not stay full.
        while (spi_is_readable(BRIDGE_SPI)) {
            (void)spi_get_hw(BRIDGE_SPI)->dr;
        }
    }
}

#if USE_LED
static void set_led(bool on) {
#if defined PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
#endif
}

static void init_led(void) {
#if defined PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    hard_assert(cyw43_arch_init() == PICO_OK);
    set_led(false);
#endif
}

static void blink_task(__unused void *params) {
    init_led();
    bool on = false;
    for (;;) {
        set_led(on);
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));
    }
}
#endif

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

static void bridge_hardware_init(void) {
    gpio_set_function(BRIDGE_PIN_TX, GPIO_FUNC_SPI);
    gpio_set_function(BRIDGE_PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(BRIDGE_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(BRIDGE_PIN_CS, GPIO_FUNC_SPI);
    gpio_pull_up(BRIDGE_PIN_CS);
#if BRIDGE_SPI_CPOL == SPI_CPOL_0
    gpio_pull_down(BRIDGE_PIN_SCK);
#else
    gpio_pull_up(BRIDGE_PIN_SCK);
#endif

    // The baud rate is ignored in slave mode; SCK is driven by the master.
    spi_init(BRIDGE_SPI, 1000 * 1000);
    spi_set_slave(BRIDGE_SPI, true);
    spi_set_format(BRIDGE_SPI, 8, BRIDGE_SPI_CPOL, BRIDGE_SPI_CPHA, SPI_MSB_FIRST);

    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, spi_get_dreq(BRIDGE_SPI, true));
    channel_config_set_ring(&config, false, STAGE_RING_BITS);
    dma_channel_configure(dma_channel, &config,
                          &spi_get_hw(BRIDGE_SPI)->dr,
                          stage_ring,
                          dma_encode_endless_transfer_count(),
                          false);
    // The pump starts the channel when the first host data arrives.
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void vLaunch(void) {
    TaskHandle_t handle;

#if configSUPPORT_STATIC_ALLOCATION
    static StackType_t usb_stack[USB_TASK_STACK_SIZE];
    static StaticTask_t usb_tcb;
    handle = xTaskCreateStatic(usb_rx_task, "UsbRx", USB_TASK_STACK_SIZE, NULL,
                               USB_TASK_PRIORITY, usb_stack, &usb_tcb);
#else
    xTaskCreate(usb_rx_task, "UsbRx", USB_TASK_STACK_SIZE, NULL, USB_TASK_PRIORITY, &handle);
#endif
    configASSERT(handle);
#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    vTaskCoreAffinitySet(handle, 1u << USB_TASK_CORE);
#endif

#if configSUPPORT_STATIC_ALLOCATION
    static StackType_t pump_stack[PUMP_TASK_STACK_SIZE];
    static StaticTask_t pump_tcb;
    handle = xTaskCreateStatic(spi_pump_task, "SpiPump", PUMP_TASK_STACK_SIZE, NULL,
                               PUMP_TASK_PRIORITY, pump_stack, &pump_tcb);
#else
    xTaskCreate(spi_pump_task, "SpiPump", PUMP_TASK_STACK_SIZE, NULL, PUMP_TASK_PRIORITY, &handle);
#endif
    configASSERT(handle);
#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    vTaskCoreAffinitySet(handle, 1u << PUMP_TASK_CORE);
#endif

#if USE_LED
#if configSUPPORT_STATIC_ALLOCATION
    static StackType_t blink_stack[BLINK_TASK_STACK_SIZE];
    static StaticTask_t blink_tcb;
    handle = xTaskCreateStatic(blink_task, "Blink", BLINK_TASK_STACK_SIZE, NULL,
                               BLINK_TASK_PRIORITY, blink_stack, &blink_tcb);
#else
    xTaskCreate(blink_task, "Blink", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, &handle);
#endif
    configASSERT(handle);
#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    vTaskCoreAffinitySet(handle, 1u << BLINK_TASK_CORE);
#endif
#endif

    vTaskStartScheduler();

    for (;;) {
        tight_loop_contents();
    }
}

int main(void) {
    stdio_init_all();
    bridge_hardware_init();
    bridge_protocol_init(&protocol, &protocol_ops);

    printf("USB-SPI bridge ready: %u KiB ring, SPI0 slave mode %d, "
           "MISO=%d CS=%d SCK=%d MOSI=%d, protocol %u.%u\n",
           USB_RING_SIZE / 1024u,
           (BRIDGE_SPI_CPOL == SPI_CPOL_1 ? 2 : 0) +
               (BRIDGE_SPI_CPHA == SPI_CPHA_1 ? 1 : 0),
           BRIDGE_PIN_TX, BRIDGE_PIN_CS, BRIDGE_PIN_SCK, BRIDGE_PIN_RX,
           BRIDGE_PROTOCOL_VERSION_MAJOR, BRIDGE_PROTOCOL_VERSION_MINOR);

    vLaunch();
    return 0;
}
