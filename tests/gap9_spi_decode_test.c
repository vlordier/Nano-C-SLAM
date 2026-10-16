#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../cslam-gap9-app/spi/spi_cmds_decode.h"

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
            abort(); \
        } \
    } while (0)

static void add_pose_packet_checksum(uint8_t packet[SPI_PKT_SIZE],
                                     const pcl_pose_t *pose) {
    uint32_t checksum = 0;
    const uint8_t *bytes = (const uint8_t *)pose;
    for (size_t index = 0; index < sizeof(*pose); index++)
        checksum += bytes[index];
    memcpy(&packet[sizeof(*pose) + 2], &checksum, sizeof(checksum));
}

static void test_pose_packet_round_trip(void) {
    uint8_t packet[SPI_PKT_SIZE] = {0};
    pcl_pose_t expected = {0};
    expected.id = 17;
    expected.timestamp = 123456;
    expected.pos[0] = 1.25f;
    expected.pos[1] = -2.5f;
    expected.pos[2] = 0.75f;
    for (int direction = 0; direction < 4; direction++)
        for (int column = 0; column < 8; column++)
            expected.tof[direction][column] = (int16_t)(direction * 100 + column);

    packet[0] = 'S';
    packet[1] = POSE_CMD;
    memcpy(&packet[2], &expected, sizeof(expected));
    add_pose_packet_checksum(packet, &expected);
    packet[SPI_PKT_SIZE - 1] = 'E';

    pcl_pose_t actual;
    CHECK(spi_decode_pose(packet, &actual) == SPI_OK);
    CHECK(actual.id == expected.id);
    CHECK(actual.timestamp == expected.timestamp);
    CHECK(memcmp(actual.pos, expected.pos, sizeof(expected.pos)) == 0);
    CHECK(memcmp(actual.tof, expected.tof, sizeof(expected.tof)) == 0);
}

static void test_pose_packet_rejects_wrong_command_and_checksum(void) {
    uint8_t packet[SPI_PKT_SIZE] = {0};
    pcl_pose_t pose = {0};
    packet[1] = SCAN_CMD;
    CHECK(spi_decode_pose(packet, &pose) == SPI_WRONG_CMD);

    packet[1] = POSE_CMD;
    packet[2] = 1;
    CHECK(spi_decode_pose(packet, &pose) == SPI_CHECKSUM_ERR);
}

int main(void) {
    test_pose_packet_round_trip();
    test_pose_packet_rejects_wrong_command_and_checksum();
    return 0;
}
