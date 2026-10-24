#include "bridge_protocol.h"

#include <string.h>

// Largest reply is GET_STATUS: 2 + 13 * 4 bytes of payload, plus framing.
#define BRIDGE_RESPONSE_MAX 64u

static inline void put_u16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static inline void put_u16_be(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static inline void put_u32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static inline uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void reset_parser(bridge_protocol_t *protocol) {
    protocol->state = BRIDGE_PARSER_WAIT_HEAD;
    protocol->sum = 0;
    protocol->len = 0;
    protocol->pos = 0;
}

void bridge_protocol_init(bridge_protocol_t *protocol,
                          const bridge_protocol_ops_t *ops) {
    protocol->ops = ops;
    reset_parser(protocol);
    protocol->checksum_errors = 0;
    protocol->bad_packets = 0;
    protocol->ring_full_events = 0;
    protocol->packets_ok = 0;
    protocol->stream_accepted = 0;
}

void bridge_protocol_reset_parser(bridge_protocol_t *protocol) {
    reset_parser(protocol);
}

static void send_packet(bridge_protocol_t *protocol, uint8_t cmd,
                        uint8_t status, const uint8_t *data, size_t len) {
    const size_t payload_len = 2 + len;
    if (payload_len > BRIDGE_PACKET_MAX_PAYLOAD ||
        payload_len + BRIDGE_PACKET_OVERHEAD > BRIDGE_RESPONSE_MAX) {
        return;
    }

    uint8_t out[BRIDGE_RESPONSE_MAX];
    out[0] = BRIDGE_PACKET_HEAD;
    out[1] = 0;
    out[2] = (uint8_t)payload_len;
    out[3] = (uint8_t)(payload_len >> 8);
    out[4] = cmd;
    out[5] = status;
    if (len) {
        memcpy(&out[6], data, len);
    }

    const size_t total = payload_len + BRIDGE_PACKET_OVERHEAD;
    out[total - 1] = BRIDGE_PACKET_REAR;

    uint8_t sum = 0;
    for (size_t i = 0; i < total; i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[1] = (uint8_t)(0u - sum);

    protocol->ops->send(out, total);
}

static void send_error(bridge_protocol_t *protocol, uint8_t cmd, uint16_t err) {
    uint8_t data[2];
    put_u16_be(data, err);
    send_packet(protocol, cmd, BRIDGE_STATUS_ERROR, data, sizeof(data));
}

// WRITE_DATA request:  cmd, reserved, seq[2], offset[4], data[N]
// WRITE_DATA response: cmd, status, seq[2], offset[4], accepted[2],
//                      stream_accepted[4], free[4], err[2]
static void send_write_response(bridge_protocol_t *protocol, uint16_t seq,
                                uint32_t offset, uint16_t accepted,
                                uint16_t err) {
    bridge_app_stats_t stats;
    protocol->ops->get_app_stats(&stats);

    uint8_t data[18];
    put_u16(&data[0], seq);
    put_u32(&data[2], offset);
    put_u16(&data[6], accepted);
    put_u32(&data[8], protocol->stream_accepted);
    put_u32(&data[12], stats.free_bytes);
    put_u16_be(&data[16], err);
    send_packet(protocol, BRIDGE_CMD_WRITE_DATA,
                err == BRIDGE_ERR_SUCCESS ? BRIDGE_STATUS_SUCCESS
                                          : BRIDGE_STATUS_ERROR,
                data, sizeof(data));
}

static void handle_write(bridge_protocol_t *protocol, const uint8_t *payload,
                         size_t len) {
    if (len < BRIDGE_WRITE_HEADER_SIZE) {
        send_write_response(protocol, 0, 0, 0, BRIDGE_ERR_INVALID_LENGTH);
        return;
    }

    const uint16_t seq = get_u16(&payload[2]);
    const uint32_t offset = get_u32(&payload[4]);
    const size_t data_len = len - BRIDGE_WRITE_HEADER_SIZE;
    const uint8_t *data = &payload[BRIDGE_WRITE_HEADER_SIZE];

    if (data_len == 0 || data_len > BRIDGE_WRITE_MAX_DATA) {
        send_write_response(protocol, seq, offset, 0,
                            BRIDGE_ERR_INVALID_LENGTH);
        return;
    }
    if ((data_len % BRIDGE_FRAME_BYTES) != 0) {
        send_write_response(protocol, seq, offset, 0,
                            BRIDGE_ERR_NOT_ALIGNED);
        return;
    }
    if (offset != protocol->stream_accepted) {
        // A retry after a lost ACK must not duplicate already queued data.
        send_write_response(protocol, seq, offset, 0, BRIDGE_ERR_OFFSET);
        return;
    }

    const size_t accepted = protocol->ops->write(data, data_len);
    if (accepted != data_len) {
        protocol->ring_full_events++;
    }
    protocol->stream_accepted += (uint32_t)accepted;
    send_write_response(protocol, seq, offset, (uint16_t)accepted,
                        accepted == data_len ? BRIDGE_ERR_SUCCESS
                                             : BRIDGE_ERR_RING_FULL);
}

static void handle_status(bridge_protocol_t *protocol) {
    bridge_app_stats_t stats;
    protocol->ops->get_app_stats(&stats);

    uint8_t data[52];
    put_u32(&data[0], stats.capacity);
    put_u32(&data[4], stats.used);
    put_u32(&data[8], stats.free_bytes);
    put_u32(&data[12], stats.stage_ahead);
    put_u32(&data[16], stats.spi_consumed);
    put_u32(&data[20], stats.zero_fill);
    put_u32(&data[24], protocol->checksum_errors);
    put_u32(&data[28], protocol->bad_packets);
    put_u32(&data[32], protocol->ring_full_events);
    put_u32(&data[36], stats.underrun_events);
    put_u32(&data[40], protocol->packets_ok);
    put_u32(&data[44], protocol->stream_accepted);
    put_u32(&data[48], protocol->stream_accepted % BRIDGE_FRAME_BYTES);
    send_packet(protocol, BRIDGE_CMD_GET_STATUS, BRIDGE_STATUS_SUCCESS, data,
                sizeof(data));
}

static void handle_clear_counters(bridge_protocol_t *protocol) {
    protocol->checksum_errors = 0;
    protocol->bad_packets = 0;
    protocol->ring_full_events = 0;
    protocol->packets_ok = 0;
    protocol->ops->clear_app_counters();
    send_packet(protocol, BRIDGE_CMD_CLEAR_COUNTERS, BRIDGE_STATUS_SUCCESS,
                NULL, 0);
}

static void handle_version(bridge_protocol_t *protocol) {
    static const uint8_t version[4] = {
        BRIDGE_PROTOCOL_VERSION_MAJOR,
        BRIDGE_PROTOCOL_VERSION_MINOR,
        BRIDGE_FIRMWARE_VERSION_MAJOR,
        BRIDGE_FIRMWARE_VERSION_MINOR,
    };
    send_packet(protocol, BRIDGE_CMD_GET_VERSION, BRIDGE_STATUS_SUCCESS,
                version, sizeof(version));
}

static void dispatch_packet(bridge_protocol_t *protocol,
                            const uint8_t *payload, size_t len) {
    if (len < 2) {
        protocol->bad_packets++;
        return;
    }

    const uint8_t cmd = payload[0];
    protocol->packets_ok++;

    switch (cmd) {
    case BRIDGE_CMD_WRITE_DATA:
        handle_write(protocol, payload, len);
        break;
    case BRIDGE_CMD_GET_STATUS:
        if (len != 2) {
            send_error(protocol, cmd, BRIDGE_ERR_INVALID_LENGTH);
        } else {
            handle_status(protocol);
        }
        break;
    case BRIDGE_CMD_CLEAR_COUNTERS:
        if (len != 2) {
            send_error(protocol, cmd, BRIDGE_ERR_INVALID_LENGTH);
        } else {
            handle_clear_counters(protocol);
        }
        break;
    case BRIDGE_CMD_GET_VERSION:
        if (len != 2) {
            send_error(protocol, cmd, BRIDGE_ERR_INVALID_LENGTH);
        } else {
            handle_version(protocol);
        }
        break;
    default:
        protocol->bad_packets++;
        send_error(protocol, cmd, BRIDGE_ERR_UNSUPPORT);
        break;
    }
}

void bridge_protocol_feed(bridge_protocol_t *protocol, const uint8_t *data,
                          size_t len) {
    for (size_t i = 0; i < len; i++) {
        const uint8_t byte = data[i];

        switch (protocol->state) {
        case BRIDGE_PARSER_WAIT_HEAD:
            if (byte == BRIDGE_PACKET_HEAD) {
                protocol->sum = byte;
                protocol->state = BRIDGE_PARSER_WAIT_CHKSUM;
            }
            break;

        case BRIDGE_PARSER_WAIT_CHKSUM:
            protocol->sum = (uint8_t)(protocol->sum + byte);
            protocol->state = BRIDGE_PARSER_WAIT_LEN_LO;
            break;

        case BRIDGE_PARSER_WAIT_LEN_LO:
            protocol->sum = (uint8_t)(protocol->sum + byte);
            protocol->len = byte;
            protocol->state = BRIDGE_PARSER_WAIT_LEN_HI;
            break;

        case BRIDGE_PARSER_WAIT_LEN_HI:
            protocol->sum = (uint8_t)(protocol->sum + byte);
            protocol->len |= (uint16_t)(byte << 8);
            protocol->pos = 0;
            if (protocol->len > BRIDGE_PACKET_MAX_PAYLOAD) {
                protocol->bad_packets++;
                reset_parser(protocol);
            } else if (protocol->len == 0) {
                protocol->state = BRIDGE_PARSER_WAIT_REAR;
            } else {
                protocol->state = BRIDGE_PARSER_WAIT_DATA;
            }
            break;

        case BRIDGE_PARSER_WAIT_DATA:
            protocol->sum = (uint8_t)(protocol->sum + byte);
            protocol->payload[protocol->pos++] = byte;
            if (protocol->pos == protocol->len) {
                protocol->state = BRIDGE_PARSER_WAIT_REAR;
            }
            break;

        case BRIDGE_PARSER_WAIT_REAR:
            protocol->sum = (uint8_t)(protocol->sum + byte);
            if (byte == BRIDGE_PACKET_REAR && protocol->sum == 0) {
                dispatch_packet(protocol, protocol->payload, protocol->len);
            } else {
                protocol->checksum_errors++;
            }
            reset_parser(protocol);
            break;
        }
    }
}
