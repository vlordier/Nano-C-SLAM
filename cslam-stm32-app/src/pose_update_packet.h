#ifndef POSE_UPDATE_PACKET_H
#define POSE_UPDATE_PACKET_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum {
    POSE_UPDATE_PACKET_SIZE = 113,
    POSE_UPDATE_ENTRY_SIZE = 14,
    POSE_UPDATE_MAX_ENTRIES = 8,
};

// The wire format is one count byte followed by up to eight (id, x, y, yaw)
// entries. The packet is padded to a fixed length for the UWB transport.
static inline uint8_t pose_update_packet_count(const uint8_t *data, size_t len) {
    if (data == NULL || len != POSE_UPDATE_PACKET_SIZE || data[0] == 0 ||
        data[0] > POSE_UPDATE_MAX_ENTRIES)
        return 0;
    return data[0];
}

static inline bool pose_update_packet_read(const uint8_t *data, size_t len,
                                           uint8_t index, int16_t *pose_id,
                                           float pose[3]) {
    const uint8_t count = pose_update_packet_count(data, len);
    if (count == 0 || index >= count || pose_id == NULL || pose == NULL)
        return false;

    const uint8_t *entry = data + 1 + POSE_UPDATE_ENTRY_SIZE * index;
    int16_t decoded_id;
    float decoded_pose[3];
    memcpy(&decoded_id, entry, sizeof(decoded_id));
    memcpy(decoded_pose, entry + sizeof(decoded_id), sizeof(decoded_pose));
    if (decoded_id < 0 || !isfinite(decoded_pose[0]) ||
        !isfinite(decoded_pose[1]) || !isfinite(decoded_pose[2]))
        return false;

    *pose_id = decoded_id;
    memcpy(pose, decoded_pose, sizeof(decoded_pose));
    return true;
}

#endif
