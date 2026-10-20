#include "uwb_sitl_protocol.h"

#include <string.h>

#define UWB_SITL_VERSION 1U
#define UWB_SITL_RANGE_VECTOR_SIZE 16U

static uint16_t read_be16(const uint8_t *bytes) {
    return (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static uint32_t read_be32(const uint8_t *bytes) {
    return ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) | (uint32_t)bytes[3];
}

static uint64_t read_be64(const uint8_t *bytes) {
    return ((uint64_t)read_be32(bytes) << 32) | read_be32(bytes + 4U);
}

static void write_be16(uint8_t *bytes, uint16_t value) {
    bytes[0] = (uint8_t)(value >> 8);
    bytes[1] = (uint8_t)value;
}

static void write_be32(uint8_t *bytes, uint32_t value) {
    bytes[0] = (uint8_t)(value >> 24);
    bytes[1] = (uint8_t)(value >> 16);
    bytes[2] = (uint8_t)(value >> 8);
    bytes[3] = (uint8_t)value;
}

static void write_be64(uint8_t *bytes, uint64_t value) {
    write_be32(bytes, (uint32_t)(value >> 32));
    write_be32(bytes + 4U, (uint32_t)value);
}

static int node_id_valid(uint8_t node_id, int allow_broker) {
    return node_id <= 5U || (allow_broker && node_id == UWB_SITL_BROKER_ID);
}

static int node_to_node_kind(uint8_t kind) {
    return (kind >= UWB_SITL_RANGE_REQUEST &&
            kind <= UWB_SITL_MEASUREMENT);
}

static int validate_packet(const uwb_sitl_packet_t *packet) {
    const uint8_t kind = (uint8_t)packet->kind;
    if (packet->flags != 0U || packet->payload_length > UWB_SITL_MAX_PAYLOAD ||
        !node_id_valid(packet->source_id, 1) ||
        !node_id_valid(packet->destination_id, 1)) {
        return -1;
    }

    uint16_t expected_length;
    switch (kind) {
    case UWB_SITL_REGISTER:
        expected_length = 2U;
        break;
    case UWB_SITL_REGISTER_ACK:
    case UWB_SITL_RANGE_REQUEST:
    case UWB_SITL_TIME_SYNC:
        expected_length = 0U;
        break;
    case UWB_SITL_RANGE_RESPONSE:
    case UWB_SITL_MEASUREMENT:
        expected_length = UWB_SITL_RANGE_VECTOR_SIZE;
        break;
    case UWB_SITL_ERROR:
        expected_length = 1U;
        break;
    case UWB_SITL_MESSAGE_REQUEST:
    case UWB_SITL_MESSAGE_DELIVERY:
    case UWB_SITL_MESSAGE_RESPONSE:
        if (packet->payload_length < 4U || packet->payload[0] != 0xDEU ||
            packet->payload[1] != packet->source_id ||
            packet->payload[2] != packet->destination_id) {
            return -1;
        }
        expected_length = packet->payload_length;
        break;
    default:
        return -1;
    }
    if (packet->payload_length != expected_length) {
        return -1;
    }

    if (kind == UWB_SITL_REGISTER) {
        if (packet->source_id == UWB_SITL_BROKER_ID ||
            packet->destination_id != UWB_SITL_BROKER_ID ||
            packet->sim_time_ns != 0U || read_be16(packet->payload) == 0U) {
            return -1;
        }
    } else if (kind == UWB_SITL_REGISTER_ACK) {
        if (packet->source_id != UWB_SITL_BROKER_ID ||
            packet->destination_id == UWB_SITL_BROKER_ID ||
            packet->sim_time_ns == 0U) {
            return -1;
        }
    } else if (kind == UWB_SITL_TIME_SYNC) {
        if (packet->source_id != UWB_SITL_BROKER_ID ||
            packet->destination_id == UWB_SITL_BROKER_ID ||
            packet->sim_time_ns == 0U) {
            return -1;
        }
    } else if (node_to_node_kind(kind)) {
        if (packet->source_id == UWB_SITL_BROKER_ID ||
            packet->destination_id == UWB_SITL_BROKER_ID ||
            packet->source_id == packet->destination_id ||
            packet->sim_time_ns == 0U) {
            return -1;
        }
    } else if (kind == UWB_SITL_ERROR) {
        if (packet->source_id != UWB_SITL_BROKER_ID ||
            packet->destination_id == UWB_SITL_BROKER_ID) {
            return -1;
        }
    }

    if ((kind == UWB_SITL_RANGE_RESPONSE || kind == UWB_SITL_MEASUREMENT) &&
        read_be32(packet->payload) == 0U) {
        return -1;
    }
    if (kind != UWB_SITL_REGISTER && kind != UWB_SITL_ERROR &&
        packet->sim_time_ns == 0U) {
        return -1;
    }
    return 0;
}

uint32_t uwb_sitl_crc32(const uint8_t *bytes, size_t length) {
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t index = 0U; index < length; ++index) {
        crc ^= bytes[index];
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

int uwb_sitl_accept_fresh(uwb_sitl_freshness_guard_t *guard,
                          const uwb_sitl_packet_t *packet,
                          uint64_t now_sim_time_ns,
                          uint64_t max_age_ns) {
    if (guard == NULL || packet == NULL || max_age_ns == 0U ||
        packet->sim_time_ns == 0U || packet->sim_time_ns > now_sim_time_ns ||
        now_sim_time_ns - packet->sim_time_ns > max_age_ns ||
        (guard->initialized != 0U &&
         (packet->sequence <= guard->last_sequence ||
          packet->sim_time_ns <= guard->last_sim_time_ns))) {
        return -1;
    }
    guard->last_sequence = packet->sequence;
    guard->last_sim_time_ns = packet->sim_time_ns;
    guard->initialized = 1U;
    return 0;
}

int uwb_sitl_encode(const uwb_sitl_packet_t *packet, uint8_t *output,
                    size_t output_capacity, size_t *output_length) {
    if (packet == NULL || output == NULL || output_length == NULL ||
        validate_packet(packet) != 0) {
        return -1;
    }
    const size_t frame_length = UWB_SITL_HEADER_SIZE +
                                (size_t)packet->payload_length +
                                UWB_SITL_CRC_SIZE;
    if (output_capacity < frame_length) {
        return -1;
    }

    memcpy(output, "NCSU", 4U);
    output[4] = UWB_SITL_VERSION;
    output[5] = (uint8_t)packet->kind;
    output[6] = packet->flags;
    output[7] = 0U;
    write_be32(output + 8U, packet->sequence);
    write_be64(output + 12U, packet->sim_time_ns);
    output[20] = packet->source_id;
    output[21] = packet->destination_id;
    write_be16(output + 22U, packet->payload_length);
    memcpy(output + UWB_SITL_HEADER_SIZE, packet->payload,
           packet->payload_length);
    write_be32(output + frame_length - UWB_SITL_CRC_SIZE,
               uwb_sitl_crc32(output, frame_length - UWB_SITL_CRC_SIZE));
    *output_length = frame_length;
    return 0;
}

int uwb_sitl_decode(const uint8_t *bytes, size_t length,
                    uwb_sitl_packet_t *packet) {
    if (bytes == NULL || packet == NULL ||
        length < UWB_SITL_HEADER_SIZE + UWB_SITL_CRC_SIZE ||
        memcmp(bytes, "NCSU", 4U) != 0 || bytes[4] != UWB_SITL_VERSION ||
        bytes[7] != 0U) {
        return -1;
    }
    const uint16_t payload_length = read_be16(bytes + 22U);
    const size_t expected_length = UWB_SITL_HEADER_SIZE +
                                   (size_t)payload_length + UWB_SITL_CRC_SIZE;
    if (payload_length > UWB_SITL_MAX_PAYLOAD || length != expected_length ||
        read_be32(bytes + length - UWB_SITL_CRC_SIZE) !=
            uwb_sitl_crc32(bytes, length - UWB_SITL_CRC_SIZE)) {
        return -1;
    }

    memset(packet, 0, sizeof(*packet));
    packet->kind = (uwb_sitl_kind_t)bytes[5];
    packet->flags = bytes[6];
    packet->sequence = read_be32(bytes + 8U);
    packet->sim_time_ns = read_be64(bytes + 12U);
    packet->source_id = bytes[20];
    packet->destination_id = bytes[21];
    packet->payload_length = payload_length;
    memcpy(packet->payload, bytes + UWB_SITL_HEADER_SIZE, payload_length);
    return validate_packet(packet);
}
