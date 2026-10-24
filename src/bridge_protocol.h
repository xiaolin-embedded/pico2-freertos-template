#ifndef BRIDGE_PROTOCOL_H
#define BRIDGE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

// Packet format:
//   0xAA | chksum | len_lo | len_hi | payload[len] | 0x55
// chksum makes the 8-bit sum of the complete packet zero. payload[0] is the
// command; payload[1] is reserved in requests and holds the status in replies.
#define BRIDGE_PACKET_HEAD          0xAAu
#define BRIDGE_PACKET_REAR          0x55u
#define BRIDGE_PACKET_MAX_SIZE      1024u
#define BRIDGE_PACKET_OVERHEAD      5u
#define BRIDGE_PACKET_MAX_PAYLOAD   (BRIDGE_PACKET_MAX_SIZE - BRIDGE_PACKET_OVERHEAD)

#define BRIDGE_FRAME_BYTES          64u
#define BRIDGE_WRITE_MAX_DATA       (15u * BRIDGE_FRAME_BYTES)
#define BRIDGE_WRITE_HEADER_SIZE    8u  // cmd, status, seq[2], offset[4]

#define BRIDGE_PROTOCOL_VERSION_MAJOR   2u
#define BRIDGE_PROTOCOL_VERSION_MINOR   0u
#define BRIDGE_FIRMWARE_VERSION_MAJOR   1u
#define BRIDGE_FIRMWARE_VERSION_MINOR   0u

#define BRIDGE_CMD_WRITE_DATA       0x01u
#define BRIDGE_CMD_GET_STATUS       0x02u
#define BRIDGE_CMD_CLEAR_COUNTERS   0x03u
#define BRIDGE_CMD_GET_VERSION      0x04u

#define BRIDGE_STATUS_SUCCESS       0x01u
#define BRIDGE_STATUS_BUSY          0x02u
#define BRIDGE_STATUS_ERROR         0x03u

#define BRIDGE_ERR_SUCCESS          0x0000u
#define BRIDGE_ERR_INVALID_LENGTH   0x0101u
#define BRIDGE_ERR_UNSUPPORT        0x0104u
#define BRIDGE_ERR_CHECKSUM         0x0106u
#define BRIDGE_ERR_RING_FULL        0x0201u
#define BRIDGE_ERR_NOT_ALIGNED      0x0202u
#define BRIDGE_ERR_OFFSET           0x0203u
#define BRIDGE_ERR_UNEXPECT         0xFFFFu

// Counters supplied by the application when GET_STATUS is handled. The ring
// level is sampled on demand, so it always reflects the current state.
typedef struct {
    uint32_t capacity;
    uint32_t used;
    uint32_t free_bytes;
    uint32_t stage_ahead;
    uint32_t spi_consumed;
    uint32_t zero_fill;
    uint32_t underrun_events;
} bridge_app_stats_t;

typedef struct {
    void (*send)(const uint8_t *data, size_t len);
    size_t (*write)(const uint8_t *data, size_t len);
    void (*get_app_stats)(bridge_app_stats_t *stats);
    void (*clear_app_counters)(void);
} bridge_protocol_ops_t;

typedef enum {
    BRIDGE_PARSER_WAIT_HEAD,
    BRIDGE_PARSER_WAIT_CHKSUM,
    BRIDGE_PARSER_WAIT_LEN_LO,
    BRIDGE_PARSER_WAIT_LEN_HI,
    BRIDGE_PARSER_WAIT_DATA,
    BRIDGE_PARSER_WAIT_REAR,
} bridge_parser_state_t;

// Static instance; the payload buffer is the only large member.
typedef struct {
    const bridge_protocol_ops_t *ops;
    bridge_parser_state_t state;
    uint8_t sum;
    uint16_t len;
    uint16_t pos;
    uint8_t payload[BRIDGE_PACKET_MAX_PAYLOAD];
    uint32_t checksum_errors;
    uint32_t bad_packets;
    uint32_t ring_full_events;
    uint32_t packets_ok;
    uint32_t stream_accepted;
} bridge_protocol_t;

void bridge_protocol_init(bridge_protocol_t *protocol,
                          const bridge_protocol_ops_t *ops);
void bridge_protocol_reset_parser(bridge_protocol_t *protocol);
void bridge_protocol_feed(bridge_protocol_t *protocol, const uint8_t *data,
                          size_t len);

#endif  // BRIDGE_PROTOCOL_H
