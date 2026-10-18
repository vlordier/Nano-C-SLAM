#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "gap9_gvsoc_spi_trace_transport.h"
#include "spi_cmds_decode.h"
#include "spi_driver.h"

#define INVALID_POSE_PACKET 0
#define POSE_PACKET_COUNT 60
#define TRACE_PACKET_COUNT 66

static uint8_t trace_packets[TRACE_PACKET_COUNT][SPI_PKT_SIZE];
static uint8_t trace_response[sizeof(slam_job_t)];
static uint16_t next_packet;
static uint16_t trace_response_length;

static bool has_constraint_edge(int16_t expected_a, int16_t expected_b) {
    for (int32_t index = 0; index < pcl_get_constraint_count(); ++index) {
        int16_t from;
        int16_t to;
        if (!pcl_get_constraint_endpoints(index, &from, &to)) return false;
        if ((from == expected_a && to == expected_b) ||
            (from == expected_b && to == expected_a))
            return true;
    }
    return false;
}

static uint32_t byte_sum(const uint8_t *bytes, size_t size) {
    uint32_t sum = 0;
    for (size_t index = 0; index < size; ++index) sum += bytes[index];
    return sum;
}

static uint8_t *next_frame(uint8_t command) {
    uint8_t *frame = trace_packets[next_packet++];
    memset(frame, 0, SPI_PKT_SIZE);
    frame[0] = 'S';
    frame[1] = command;
    frame[SPI_PKT_SIZE - 1] = 'E';
    return frame;
}

static void append_pose(int16_t id) {
    static const int16_t ranges_mm[4] = {1000, 1200, 900, 1100};
    pcl_pose_t pose;
    memset(&pose, 0, sizeof(pose));
    pose.id = id;
    pose.timestamp = (int32_t)id * 100;
    float angle = (float)id * 0.02f;
    pose.pos[0] = 0.15f * cosf(angle);
    pose.pos[1] = 0.15f * sinf(angle);
    pose.pos[2] = angle + (float)M_PI_2;
    for (int direction = 0; direction < 4; ++direction) {
        for (int column = 0; column < 8; ++column)
            pose.tof[direction][column] = -1;
        pose.tof[direction][3] = ranges_mm[direction];
    }

    uint8_t *frame = next_frame(POSE_CMD);
    memcpy(&frame[2], &pose, sizeof(pose));
    uint32_t checksum = byte_sum((const uint8_t *)&pose, sizeof(pose));
    memcpy(&frame[sizeof(pose) + 2], &checksum, sizeof(checksum));
}

static void append_invalid_pose(void) {
    append_pose(0);
    trace_packets[INVALID_POSE_PACKET][sizeof(pcl_pose_t) + 2] ^= 1;
}

static void append_scan(int16_t pose_id) {
    uint8_t *frame = next_frame(SCAN_CMD);
    memcpy(&frame[2], &pose_id, sizeof(pose_id));
    uint32_t checksum = byte_sum(&frame[2], sizeof(pose_id));
    memcpy(&frame[4], &checksum, sizeof(checksum));
}

static void build_trace(void) {
    next_packet = 0;
    append_invalid_pose();
    for (int16_t id = 0; id < POSE_PACKET_COUNT; ++id) {
        append_pose(id);
        if ((id + 1) % 20 == 0) append_scan((int16_t)(id - 19));
    }
    append_scan(40); /* Duplicate scan must not add another scan origin. */
    (void)next_frame(OPTIMIZE_CMD);
    next_packet = 0;
}

int spi_slave_driver_init(void) {
    build_trace();
    return 0;
}

void spi_rcv_blocking(void *data, size_t length) {
    if (data == NULL || length != SPI_PKT_SIZE ||
        next_packet >= TRACE_PACKET_COUNT)
        return;
    memcpy(data, trace_packets[next_packet++], SPI_PKT_SIZE);
}

void spi_send_blocking(void *data, size_t length) {
    if (data == NULL || length > sizeof(trace_response)) return;
    memcpy(trace_response, data, length);
    trace_response_length = (uint16_t)length;
}

bool gap9_gvsoc_spi_trace_has_packet(void) {
    return next_packet < TRACE_PACKET_COUNT;
}

uint16_t gap9_gvsoc_spi_trace_packet_count(void) { return next_packet; }

size_t gap9_gvsoc_spi_trace_response_size(void) {
    return trace_response_length;
}

int gap9_gvsoc_spi_trace_validate(int16_t pose_packets_decoded,
                                  int16_t poses_inserted,
                                  int16_t bad_pose_checksums,
                                  int16_t scan_packets_decoded,
                                  int16_t scans_registered,
                                  int16_t optimize_packets,
                                  int32_t accepted_constraints,
                                  int32_t optimizer_edges,
                                  const slam_job_t *job) {
    if (job == NULL) return 0;
    slam_job_t response = {0};
    if (trace_response_length == sizeof(response))
        memcpy(&response, trace_response, sizeof(response));
    return next_packet == TRACE_PACKET_COUNT && pose_packets_decoded == 60 &&
           poses_inserted == 60 && bad_pose_checksums == 1 &&
           scan_packets_decoded == 4 && scans_registered == 3 &&
           optimize_packets == 1 && accepted_constraints == 2 &&
           has_constraint_edge(0, 20) && has_constraint_edge(0, 40) &&
           optimizer_edges >= 2 && job->number_of_poses == 60 &&
           job->has_nan == 0 && job->main_solver_returned == 1 &&
           job->successful_solver_call_count >= 1 &&
           job->optimization_completed == 1 &&
           trace_response_length == sizeof(response) &&
           memcmp(&response, job, sizeof(response)) == 0;
}
