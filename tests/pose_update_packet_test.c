#include "../cslam-stm32-app/src/pose_update_packet.h"

#include <assert.h>

int main(void) {
    uint8_t packet[POSE_UPDATE_PACKET_SIZE] = {0};
    packet[0] = 2;
    int16_t first_id = 7;
    int16_t second_id = 8;
    const float first_pose[3] = {1.25f, -2.0f, 0.5f};
    const float second_pose[3] = {3.0f, 4.0f, -0.25f};
    memcpy(packet + 1, &first_id, sizeof(first_id));
    memcpy(packet + 3, first_pose, sizeof(first_pose));
    memcpy(packet + 1 + POSE_UPDATE_ENTRY_SIZE, &second_id, sizeof(second_id));
    memcpy(packet + 3 + POSE_UPDATE_ENTRY_SIZE, second_pose, sizeof(second_pose));

    int16_t decoded_id = -1;
    float decoded_pose[3] = {0};
    assert(pose_update_packet_count(packet, sizeof(packet)) == 2);
    bool decoded =
        pose_update_packet_read(packet, sizeof(packet), 1, &decoded_id, decoded_pose);
    assert(decoded);
    assert(decoded_id == second_id);
    assert(decoded_pose[0] == second_pose[0]);
    assert(decoded_pose[1] == second_pose[1]);
    assert(decoded_pose[2] == second_pose[2]);

    decoded =
        pose_update_packet_read(packet, sizeof(packet), 2, &decoded_id, decoded_pose);
    assert(!decoded);
    decoded = pose_update_packet_read(packet, sizeof(packet) - 1, 0, &decoded_id,
                                      decoded_pose);
    assert(!decoded);
    packet[0] = POSE_UPDATE_MAX_ENTRIES;
    memcpy(packet + 1 + POSE_UPDATE_ENTRY_SIZE * 7, &second_id, sizeof(second_id));
    memcpy(packet + 3 + POSE_UPDATE_ENTRY_SIZE * 7, second_pose, sizeof(second_pose));
    decoded =
        pose_update_packet_read(packet, sizeof(packet), 7, &decoded_id, decoded_pose);
    assert(decoded);
    assert(decoded_id == second_id);
    packet[0] = POSE_UPDATE_MAX_ENTRIES + 1;
    assert(pose_update_packet_count(packet, sizeof(packet)) == 0);
    packet[0] = 2;
    second_id = -1;
    memcpy(packet + 1 + POSE_UPDATE_ENTRY_SIZE, &second_id, sizeof(second_id));
    decoded =
        pose_update_packet_read(packet, sizeof(packet), 1, &decoded_id, decoded_pose);
    assert(!decoded);
    const float invalid_pose[3] = {NAN, 4.0f, 0.0f};
    memcpy(packet + 3 + POSE_UPDATE_ENTRY_SIZE, invalid_pose, sizeof(invalid_pose));
    second_id = 8;
    memcpy(packet + 1 + POSE_UPDATE_ENTRY_SIZE, &second_id, sizeof(second_id));
    decoded =
        pose_update_packet_read(packet, sizeof(packet), 1, &decoded_id, decoded_pose);
    assert(!decoded);
    return 0;
}
