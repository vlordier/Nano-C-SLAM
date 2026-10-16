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

static void test_external_pose_packet_round_trip(void) {
    uint8_t packet[SPI_PKT_SIZE] = {0};
    ext_aug_pose_t expected = {0};
    expected.id = 23;
    expected.drone_id = 0x12345678;
    expected.pos[0] = 3.5f;
    expected.pos[1] = -4.25f;
    expected.pos[2] = 1.0f;
    expected.tof[DIR_RIGHT][7] = 2300;

    packet[0] = 'S';
    packet[1] = EXT_POSE_CMD;
    memcpy(&packet[2], &expected, sizeof(expected));
    uint32_t checksum = 0;
    const uint8_t *bytes = (const uint8_t *)&expected;
    for (size_t index = 0; index < sizeof(expected); index++)
        checksum += bytes[index];
    memcpy(&packet[sizeof(expected) + 2], &checksum, sizeof(checksum));
    packet[SPI_PKT_SIZE - 1] = 'E';

    ext_aug_pose_t actual;
    CHECK(spi_decode_ext_pose(packet, &actual) == SPI_OK);
    CHECK(actual.id == expected.id);
    CHECK(actual.drone_id == expected.drone_id);
    CHECK(memcmp(actual.pos, expected.pos, sizeof(expected.pos)) == 0);
    CHECK(memcmp(actual.tof, expected.tof, sizeof(expected.tof)) == 0);
}

static void test_external_pose_update_packet_round_trip(void) {
    uint8_t packet[SPI_PKT_SIZE] = {0};
    const int16_t pose_id = 42;
    const int16_t drone_id = 3;
    const float position[3] = {1.0f, -2.0f, 0.25f};
    memcpy(&packet[2], &pose_id, sizeof(pose_id));
    memcpy(&packet[4], &drone_id, sizeof(drone_id));
    memcpy(&packet[6], position, sizeof(position));
    uint32_t checksum = 0;
    const uint8_t *position_bytes = (const uint8_t *)position;
    for (size_t index = 0; index < sizeof(position); index++)
        checksum += position_bytes[index];
    memcpy(&packet[18], &checksum, sizeof(checksum));
    packet[0] = 'S';
    packet[1] = UPD_EXT_POSE_CMD;
    packet[SPI_PKT_SIZE - 1] = 'E';

    ext_pose_upd_t actual;
    CHECK(spi_decode_ext_pose_upd(packet, &actual) == SPI_OK);
    CHECK(actual.pose_id == pose_id);
    CHECK(actual.drone_id == drone_id);
    CHECK(memcmp(actual.pos, position, sizeof(position)) == 0);
}

static void test_scan_and_pose_request_packets(void) {
    uint8_t packet[SPI_PKT_SIZE] = {0};
    const int16_t scan_id = 0x1234;
    memcpy(&packet[2], &scan_id, sizeof(scan_id));
    const uint32_t scan_checksum = packet[2] + packet[3];
    memcpy(&packet[4], &scan_checksum, sizeof(scan_checksum));
    packet[0] = 'S';
    packet[1] = SCAN_CMD;

    int16_t scan_info[2] = {0};
    CHECK(spi_decode_scan_info(packet, scan_info) == SPI_OK);
    CHECK(scan_info[0] == scan_id);

    memset(packet, 0, sizeof(packet));
    const int16_t first_pose = 0x1234;
    const int16_t pose_count = 19;
    memcpy(&packet[2], &first_pose, sizeof(first_pose));
    memcpy(&packet[4], &pose_count, sizeof(pose_count));
    uint32_t request_checksum = 0;
    for (size_t index = 2; index < 6; index++)
        request_checksum += packet[index];
    memcpy(&packet[6], &request_checksum, sizeof(request_checksum));
    packet[0] = 'S';
    packet[1] = POSE_REQ_CMD;

    int16_t pose_request[2] = {0};
    CHECK(spi_decode_pose_req(packet, pose_request) == SPI_OK);
    CHECK(pose_request[0] == first_pose);
    CHECK(pose_request[1] == pose_count);
}

int main(void) {
    test_pose_packet_round_trip();
    test_pose_packet_rejects_wrong_command_and_checksum();
    test_external_pose_packet_round_trip();
    test_external_pose_update_packet_round_trip();
    test_scan_and_pose_request_packets();
    return 0;
}
