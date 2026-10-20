#ifndef UWB_SITL_PROTOCOL_H
#define UWB_SITL_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define UWB_SITL_HEADER_SIZE 24U
#define UWB_SITL_CRC_SIZE 4U
#define UWB_SITL_MAX_PAYLOAD 118U
#define UWB_SITL_BROKER_ID 255U

typedef enum {
    UWB_SITL_REGISTER = 1,
    UWB_SITL_REGISTER_ACK = 2,
    UWB_SITL_RANGE_REQUEST = 3,
    UWB_SITL_RANGE_RESPONSE = 4,
    UWB_SITL_MESSAGE_REQUEST = 5,
    UWB_SITL_MESSAGE_DELIVERY = 6,
    UWB_SITL_MESSAGE_RESPONSE = 7,
    UWB_SITL_MEASUREMENT = 8,
    UWB_SITL_TIME_SYNC = 9,
    UWB_SITL_ERROR = 255
} uwb_sitl_kind_t;

typedef struct {
    uwb_sitl_kind_t kind;
    uint8_t flags;
    uint32_t sequence;
    uint64_t sim_time_ns;
    uint8_t source_id;
    uint8_t destination_id;
    uint16_t payload_length;
    uint8_t payload[UWB_SITL_MAX_PAYLOAD];
} uwb_sitl_packet_t;

typedef struct {
    uint32_t last_sequence;
    uint64_t last_sim_time_ns;
    uint8_t initialized;
} uwb_sitl_freshness_guard_t;

/* Return zero on success and -1 for invalid fields, capacity, or bytes. */
int uwb_sitl_encode(const uwb_sitl_packet_t *packet, uint8_t *output,
                    size_t output_capacity, size_t *output_length);
int uwb_sitl_decode(const uint8_t *bytes, size_t length,
                    uwb_sitl_packet_t *packet);
uint32_t uwb_sitl_crc32(const uint8_t *bytes, size_t length);
/* Accept increasing sequence/time within the simulation-age bound.
 * Callers keep one guard per packet kind and source/destination flow. */
int uwb_sitl_accept_fresh(uwb_sitl_freshness_guard_t *guard,
                          const uwb_sitl_packet_t *packet,
                          uint64_t now_sim_time_ns,
                          uint64_t max_age_ns);

#endif
