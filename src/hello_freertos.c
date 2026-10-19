/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * USB CDC -> SPI slave bridge for RP2350.
 *
 * Wire protocol (USB CDC, little endian):
 *   AA 55 | type | seq[2] | len[2] | payload[len] | crc16[2]
 *   crc16 is CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload.
 *
 * Host -> device:
 *   0x01 WRITE   payload is appended to the big ring buffer.
 *   0x02 STATUS  request counters/levels.
 *
 * Device -> host:
 *   0x81 ACK     u32 accepted, u32 used, u32 free
 *   0x82 STATUS  u32 capacity, used, free, stage_ahead, spi_consumed,
 *                zero_fill, crc_errors, bad_frames, underrun_events, ring_full
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
#ifndef BRIDGE_SPI_CPHA
#define BRIDGE_SPI_CPHA     SPI_CPHA_0
#endif

// ---------------------------------------------------------------------------
// Buffer sizing
// ---------------------------------------------------------------------------

// Data preloaded by the host, drained by the SPI master at 1 kHz * 64 B.
#define USB_RING_SIZE       (384u * 1024u)
// Short staging ring read by DMA. Kept ahead of the DMA so a delayed pump
// cannot make the SPI master read stale bytes.
#define STAGE_RING_BITS     13u
#define STAGE_RING_SIZE     (1u << STAGE_RING_BITS)
#define STAGE_PREFETCH      512u

#define FRAME_MAX_PAYLOAD   4096u
#define USB_CHUNK_SIZE      1024u
#define USB_DRAIN_CHUNKS    8

#define CMD_WRITE           0x01u
#define CMD_STATUS          0x02u
#define RSP_ACK             0x81u
#define RSP_STATUS          0x82u

// ---------------------------------------------------------------------------
// Task configuration
// ---------------------------------------------------------------------------

#define USB_TASK_PRIORITY       (tskIDLE_PRIORITY + 4UL)
#define PUMP_TASK_PRIORITY      (tskIDLE_PRIORITY + 6UL)
#define BLINK_TASK_PRIORITY     (tskIDLE_PRIORITY + 1UL)

#define USB_TASK_CORE           0
#define PUMP_TASK_CORE          1
#define BLINK_TASK_CORE         0

#define USB_TASK_STACK_SIZE     1024
#define PUMP_TASK_STACK_SIZE    512
#define BLINK_TASK_STACK_SIZE   512

#define PUMP_PERIOD_MS          1
#define BLINK_PERIOD_MS         2000

#ifndef USE_LED
#define USE_LED                 1
#endif

_Static_assert((STAGE_RING_SIZE & (STAGE_RING_SIZE - 1u)) == 0, "stage ring must be power of two");

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t usb_ring[USB_RING_SIZE];

// DMA ring mode wraps the read address on a (1 << STAGE_RING_BITS) boundary,
// so the buffer must be aligned to its size.
static uint8_t stage_ring[STAGE_RING_SIZE] __attribute__((aligned(STAGE_RING_SIZE)));

static uint8_t frame_buf[FRAME_MAX_PAYLOAD];
static uint8_t usb_chunk[USB_CHUNK_SIZE];
static uint8_t pump_chunk[256];

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
static volatile uint32_t crc_errors;
static volatile uint32_t bad_frames;
static volatile uint32_t ring_full_events;

static uint dma_channel;

// ---------------------------------------------------------------------------
// CRC16-CCITT-FALSE
// ---------------------------------------------------------------------------

static const uint16_t crc16_tab[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
};

static inline uint16_t crc16_byte(uint16_t crc, uint8_t byte) {
    crc = (uint16_t)((crc << 4) ^ crc16_tab[((crc >> 12) ^ (byte >> 4)) & 0x0F]);
    crc = (uint16_t)((crc << 4) ^ crc16_tab[((crc >> 12) ^ (byte & 0x0F)) & 0x0F]);
    return crc;
}

// ---------------------------------------------------------------------------
// Large ring buffer (USB producer -> pump consumer)
// ---------------------------------------------------------------------------

static inline void put_u32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

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
// USB frame parser
// ---------------------------------------------------------------------------

typedef enum {
    PARSE_MAGIC0,
    PARSE_MAGIC1,
    PARSE_HEADER,
    PARSE_PAYLOAD,
    PARSE_CRC_LO,
    PARSE_CRC_HI,
    PARSE_DISCARD,
} parse_state_t;

static struct {
    parse_state_t state;
    uint8_t header[5];
    uint32_t header_pos;
    uint8_t type;
    uint16_t seq;
    uint16_t payload_len;
    uint32_t payload_pos;
    uint16_t crc;
    uint16_t crc_rx;
    uint32_t discard_left;
} parser;

static void parser_reset(void) {
    memset(&parser, 0, sizeof(parser));
    parser.state = PARSE_MAGIC0;
}

static void send_frame(uint8_t type, uint16_t seq, const uint8_t *payload, size_t len) {
    uint8_t out[9 + 40];
    if (len > 40) {
        return;
    }

    out[0] = 0xAA;
    out[1] = 0x55;
    out[2] = type;
    out[3] = (uint8_t)seq;
    out[4] = (uint8_t)(seq >> 8);
    out[5] = (uint8_t)len;
    out[6] = (uint8_t)(len >> 8);
    if (len) {
        memcpy(&out[7], payload, len);
    }

    uint16_t crc = 0xFFFF;
    for (size_t i = 2; i < 7 + len; i++) {
        crc = crc16_byte(crc, out[i]);
    }
    out[7 + len] = (uint8_t)crc;
    out[8 + len] = (uint8_t)(crc >> 8);

    stdio_usb.out_chars((const char *)out, (int)(9 + len));
}

static void handle_write(uint16_t seq, const uint8_t *data, size_t len) {
    size_t accepted = usb_ring_write(data, len);
    if (accepted != len) {
        ring_full_events++;
    }

    uint32_t used, free_bytes;
    usb_ring_stats(&used, &free_bytes);

    uint8_t payload[12];
    put_u32(&payload[0], (uint32_t)accepted);
    put_u32(&payload[4], used);
    put_u32(&payload[8], free_bytes);
    send_frame(RSP_ACK, seq, payload, sizeof(payload));
}

static void handle_status(uint16_t seq) {
    uint32_t used, free_bytes;
    usb_ring_stats(&used, &free_bytes);

    uint8_t payload[40];
    put_u32(&payload[0], USB_RING_SIZE);
    put_u32(&payload[4], used);
    put_u32(&payload[8], free_bytes);
    put_u32(&payload[12], stage_ahead_bytes);
    put_u32(&payload[16], spi_consumed_bytes);
    put_u32(&payload[20], zero_fill_bytes);
    put_u32(&payload[24], crc_errors);
    put_u32(&payload[28], bad_frames);
    put_u32(&payload[32], underrun_events);
    put_u32(&payload[36], ring_full_events);
    send_frame(RSP_STATUS, seq, payload, sizeof(payload));
}

static void dispatch_frame(uint8_t type, uint16_t seq, const uint8_t *payload, size_t len) {
    switch (type) {
    case CMD_WRITE:
        handle_write(seq, payload, len);
        break;
    case CMD_STATUS:
        handle_status(seq);
        break;
    default:
        bad_frames++;
        break;
    }
}

static void parser_feed(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (parser.state) {
        case PARSE_MAGIC0:
            if (byte == 0xAA) {
                parser.state = PARSE_MAGIC1;
            }
            break;

        case PARSE_MAGIC1:
            if (byte == 0x55) {
                parser.state = PARSE_HEADER;
                parser.header_pos = 0;
                parser.crc = 0xFFFF;
            } else if (byte != 0xAA) {
                parser.state = PARSE_MAGIC0;
            }
            break;

        case PARSE_HEADER:
            parser.header[parser.header_pos++] = byte;
            parser.crc = crc16_byte(parser.crc, byte);
            if (parser.header_pos == sizeof(parser.header)) {
                parser.type = parser.header[0];
                parser.seq = (uint16_t)(parser.header[1] | (parser.header[2] << 8));
                parser.payload_len = (uint16_t)(parser.header[3] | (parser.header[4] << 8));
                parser.payload_pos = 0;
                if (parser.payload_len > FRAME_MAX_PAYLOAD) {
                    bad_frames++;
                    parser.discard_left = (uint32_t)parser.payload_len + 2;
                    parser.state = PARSE_DISCARD;
                } else {
                    parser.state = parser.payload_len ? PARSE_PAYLOAD : PARSE_CRC_LO;
                }
            }
            break;

        case PARSE_PAYLOAD:
            frame_buf[parser.payload_pos++] = byte;
            parser.crc = crc16_byte(parser.crc, byte);
            if (parser.payload_pos == parser.payload_len) {
                parser.state = PARSE_CRC_LO;
            }
            break;

        case PARSE_CRC_LO:
            parser.crc_rx = byte;
            parser.state = PARSE_CRC_HI;
            break;

        case PARSE_CRC_HI:
            parser.crc_rx |= (uint16_t)(byte << 8);
            if (parser.crc_rx == parser.crc) {
                dispatch_frame(parser.type, parser.seq, frame_buf, parser.payload_len);
            } else {
                crc_errors++;
            }
            parser.state = PARSE_MAGIC0;
            break;

        case PARSE_DISCARD:
            if (--parser.discard_left == 0) {
                parser.state = PARSE_MAGIC0;
            }
            break;
        }
    }
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
            parser_reset();
        }
        was_connected = connected;

        for (int i = 0; i < USB_DRAIN_CHUNKS; i++) {
            int count = stdio_usb.in_chars((char *)usb_chunk, sizeof(usb_chunk));
            if (count <= 0) {
                break;
            }
            parser_feed(usb_chunk, (size_t)count);
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
                uint32_t fill = STAGE_PREFETCH - (uint32_t)ahead;
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

    printf("USB-SPI bridge ready: %u KiB ring, SPI0 slave mode %d, "
           "MISO=%d CS=%d SCK=%d MOSI=%d\n",
           USB_RING_SIZE / 1024u,
           (BRIDGE_SPI_CPOL == SPI_CPOL_0 && BRIDGE_SPI_CPHA == SPI_CPHA_0) ? 0 : 3,
           BRIDGE_PIN_TX, BRIDGE_PIN_CS, BRIDGE_PIN_SCK, BRIDGE_PIN_RX);

    vLaunch();
    return 0;
}
