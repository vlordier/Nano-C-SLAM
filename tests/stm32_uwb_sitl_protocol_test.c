#include <stdio.h>
#include <string.h>

#include "uwb_sitl_protocol.h"

static const uint8_t registration_golden[] = {
    0x4e, 0x43, 0x53, 0x55, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff,
    0x00, 0x02, 0x4d, 0xf1, 0x14, 0x1c, 0x28, 0xf6,
};
static const uint8_t range_response_golden[] = {
    0x4e, 0x43, 0x53, 0x55, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x02, 0xdf, 0xd1, 0xc0, 0x40, 0x01, 0x00,
    0x00, 0x10, 0x00, 0x00, 0x09, 0x29, 0xff, 0xff, 0xff, 0x9c, 0x00,
    0x00, 0x00, 0xc8, 0x00, 0x00, 0x05, 0xdc, 0x6d, 0x10, 0xc9, 0xa8,
};
static const uint8_t message_request_golden[] = {
    0x4e, 0x43, 0x53, 0x55, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x42, 0x40, 0x00, 0x01,
    0x00, 0x05, 0xde, 0x00, 0x01, 0x07, 0x42, 0x78, 0x6c, 0x8d, 0x8e,
};
static const uint8_t time_sync_golden[] = {
    0x4e, 0x43, 0x53, 0x55, 0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x01, 0x2a, 0x05, 0xf2, 0x00, 0xff, 0x00,
    0x00, 0x00, 0x31, 0xfd, 0xe6, 0x0f,
};

int main(void) {
    uwb_sitl_packet_t packet = {
        .kind = UWB_SITL_REGISTER,
        .sequence = 0U,
        .sim_time_ns = 0U,
        .source_id = 1U,
        .destination_id = UWB_SITL_BROKER_ID,
        .payload_length = 2U,
        .payload = {0x4dU, 0xf1U},
    };
    uint8_t encoded[UWB_SITL_HEADER_SIZE + UWB_SITL_MAX_PAYLOAD +
                    UWB_SITL_CRC_SIZE];
    size_t encoded_length = 0U;
    if (uwb_sitl_encode(&packet, encoded, sizeof(encoded), &encoded_length) !=
            0 ||
        encoded_length != sizeof(registration_golden) ||
        memcmp(encoded, registration_golden, sizeof(registration_golden)) !=
            0) {
        fputs("C registration encoding differs from the Python golden vector\n",
              stderr);
        return 1;
    }

    packet.kind = UWB_SITL_TIME_SYNC;
    packet.sequence = 1U;
    packet.sim_time_ns = UINT64_C(5000000000);
    packet.source_id = UWB_SITL_BROKER_ID;
    packet.destination_id = 0U;
    packet.payload_length = 0U;
    if (uwb_sitl_encode(&packet, encoded, sizeof(encoded), &encoded_length) !=
            0 ||
        encoded_length != sizeof(time_sync_golden) ||
        memcmp(encoded, time_sync_golden, sizeof(time_sync_golden)) != 0) {
        fputs("C time-sync encoding differs from the Python golden vector\n",
              stderr);
        return 1;
    }

    uwb_sitl_packet_t decoded;
    if (uwb_sitl_decode(registration_golden, sizeof(registration_golden),
                        &decoded) != 0 ||
        decoded.kind != UWB_SITL_REGISTER || decoded.source_id != 1U ||
        decoded.destination_id != UWB_SITL_BROKER_ID ||
        decoded.payload_length != 2U || decoded.payload[0] != 0x4dU ||
        decoded.payload[1] != 0xf1U) {
        fputs("C registration golden vector failed to decode\n", stderr);
        return 1;
    }

    packet.kind = UWB_SITL_RANGE_RESPONSE;
    packet.sequence = 7U;
    packet.sim_time_ns = UINT64_C(12345000000);
    packet.source_id = 1U;
    packet.destination_id = 0U;
    packet.payload_length = 16U;
    const uint8_t range_payload[] = {
        0x00, 0x00, 0x09, 0x29, 0xff, 0xff, 0xff, 0x9c,
        0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x05, 0xdc,
    };
    memcpy(packet.payload, range_payload, sizeof(range_payload));
    if (uwb_sitl_encode(&packet, encoded, sizeof(encoded), &encoded_length) !=
            0 ||
        encoded_length != sizeof(range_response_golden) ||
        memcmp(encoded, range_response_golden,
               sizeof(range_response_golden)) != 0) {
        fputs("C range encoding differs from the Python golden vector\n",
              stderr);
        return 1;
    }

    uwb_sitl_freshness_guard_t guard = {0};
    if (uwb_sitl_accept_fresh(&guard, &packet, UINT64_C(13345000000),
                              UINT64_C(1000000000)) != 0 ||
        uwb_sitl_accept_fresh(&guard, &packet, UINT64_C(13345000000),
                              UINT64_C(1000000000)) == 0) {
        fputs("freshness guard accepted a duplicate packet\n", stderr);
        return 1;
    }
    uwb_sitl_packet_t repeated_time = packet;
    repeated_time.sequence++;
    if (uwb_sitl_accept_fresh(&guard, &repeated_time,
                              UINT64_C(13345000000),
                              UINT64_C(1000000000)) == 0 ||
        uwb_sitl_accept_fresh(&guard, &packet, UINT64_C(14345000001),
                              UINT64_C(1000000000)) == 0 ||
        uwb_sitl_accept_fresh(&guard, &packet, UINT64_C(12344999999),
                              UINT64_C(1000000000)) == 0) {
        fputs("freshness guard accepted invalid simulation time\n", stderr);
        return 1;
    }

    packet.kind = UWB_SITL_MESSAGE_REQUEST;
    packet.sequence = 3U;
    packet.sim_time_ns = UINT64_C(1000000);
    packet.source_id = 0U;
    packet.destination_id = 1U;
    packet.payload_length = 5U;
    const uint8_t message_payload[] = {0xdeU, 0U, 1U, 7U, 0x42U};
    memcpy(packet.payload, message_payload, sizeof(message_payload));
    if (uwb_sitl_encode(&packet, encoded, sizeof(encoded), &encoded_length) !=
            0 ||
        encoded_length != sizeof(message_request_golden) ||
        memcmp(encoded, message_request_golden,
               sizeof(message_request_golden)) != 0) {
        fputs("C message encoding differs from the Python golden vector\n",
              stderr);
        return 1;
    }

    uint8_t corrupted[sizeof(registration_golden)];
    memcpy(corrupted, registration_golden, sizeof(corrupted));
    corrupted[24] ^= 1U;
    if (uwb_sitl_decode(corrupted, sizeof(corrupted), &decoded) == 0) {
        fputs("C decoder accepted a bad CRC\n", stderr);
        return 1;
    }
    packet.destination_id = 2U;
    if (uwb_sitl_encode(&packet, encoded, sizeof(encoded), &encoded_length) ==
        0) {
        fputs("C encoder accepted an invalid REGISTER route\n", stderr);
        return 1;
    }
    puts("C UWB codec matches golden vector and rejects malformed input");
    return 0;
}
