#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "FreeRTOS.h"

static TickType_t task_tick;

TickType_t xTaskGetTickCount(void) { return task_tick; }

void test_debug_print(const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    vfprintf(stderr, format, arguments);
    va_end(arguments);
}

#define FRAME_SIZE 540U
#define PAYLOAD_OFFSET 24U

uint32_t tof_matrix_sitl_crc32(const uint8_t *bytes, size_t length);
int tof_matrix_sitl_decode(
    const uint8_t *packet, size_t length, uint32_t *sequence,
    uint64_t *sim_time_ns, int16_t matrix[4][8][8]);
int tof_matrix_init(void);
int get_tof_matrix(uint8_t sensor_id, int16_t matrix[][8]);

static void write_be32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static void write_be64(uint8_t *p, uint64_t value) {
    write_be32(p, (uint32_t)(value >> 32));
    write_be32(p + 4, (uint32_t)value);
}

static void finalize(uint8_t packet[FRAME_SIZE]) {
    write_be32(packet + FRAME_SIZE - 4U,
               tof_matrix_sitl_crc32(packet, FRAME_SIZE - 4U));
}

static void make_frame(uint8_t packet[FRAME_SIZE]) {
    memset(packet, 0, FRAME_SIZE);
    memcpy(packet, "NCTF", 4U);
    packet[4] = 1U;
    packet[5] = 4U;
    packet[6] = 8U;
    packet[7] = 8U;
    write_be32(packet + 8U, 42U);
    write_be64(packet + 12U, UINT64_C(123456789));
    write_be32(packet + 20U, 512U);
    for (size_t i = 0; i < 256U; ++i) {
        packet[PAYLOAD_OFFSET + i * 2U] = (uint8_t)(i >> 8);
        packet[PAYLOAD_OFFSET + i * 2U + 1U] = (uint8_t)i;
    }
    packet[PAYLOAD_OFFSET + 2U] = 0xffU;
    packet[PAYLOAD_OFFSET + 3U] = 0xffU;
    finalize(packet);
}

static int expect_rejected(uint8_t packet[FRAME_SIZE], size_t length) {
    int16_t matrix[4][8][8];
    uint32_t sequence = 0U;
    uint64_t sim_time = 0U;
    return tof_matrix_sitl_decode(packet, length, &sequence, &sim_time,
                                  matrix) != 0;
}

static int test_udp_provider(const uint8_t packet[FRAME_SIZE]) {
    const int sender = socket(AF_INET, SOCK_DGRAM, 0);
    if (sender < 0 || tof_matrix_init() != 0) {
        perror("UDP provider init");
        return 1;
    }
    struct sockaddr_in destination;
    memset(&destination, 0, sizeof(destination));
    destination.sin_family = AF_INET;
    destination.sin_port = htons(39951U);
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (sendto(sender, packet, FRAME_SIZE, 0,
               (struct sockaddr *)&destination, sizeof(destination)) !=
        (ssize_t)FRAME_SIZE) {
        perror("UDP provider send");
        close(sender);
        return 1;
    }
    int16_t sensor[8][8];
    int first_result = 1;
    for (unsigned attempt = 0U; attempt < 20U && first_result != 0;
         ++attempt) {
        usleep(1000U);
        first_result = get_tof_matrix(0U, sensor);
    }
    const int first_value = sensor[0][0];
    const int second_value = sensor[0][1];
    const int third_result = get_tof_matrix(3U, sensor);
    const int last_value = sensor[7][7];
    if (first_result != 0 || first_value != 0 || second_value != -1 ||
        third_result != 0 || last_value != 255) {
        fprintf(stderr, "UDP provider data: sensor0=%d,%d sensor3=%d\n",
                first_value, second_value, last_value);
        fprintf(stderr, "results: %d %d\n", first_result, third_result);
        close(sender);
        return 1;
    }
    task_tick = 501U;
    if (get_tof_matrix(0U, sensor) != 1) {
        fputs("UDP provider stale frame accepted\n", stderr);
        close(sender);
        return 1;
    }
    close(sender);
    return 0;
}

int main(void) {
    uint8_t packet[FRAME_SIZE];
    int16_t matrix[4][8][8];
    uint32_t sequence = 0U;
    uint64_t sim_time = 0U;
    make_frame(packet);
    if (tof_matrix_sitl_decode(packet, sizeof(packet), &sequence, &sim_time,
                               matrix) != 0 ||
        sequence != 42U || sim_time != UINT64_C(123456789) ||
        matrix[0][0][0] != 0 || matrix[0][0][1] != -1 ||
        matrix[3][7][7] != 255) {
        fputs("valid frame decode failed\n", stderr);
        return 1;
    }
    if (setenv("NCS_SITL_TOF_UDP_PORT", "39951", 1) != 0 ||
        test_udp_provider(packet) != 0) {
        fputs("UDP provider freshness/coherence failed\n", stderr);
        return 1;
    }
    make_frame(packet);
    if (!expect_rejected(packet, sizeof(packet) - 1U)) {
        fputs("short frame accepted\n", stderr);
        return 1;
    }
    packet[4] = 2U;
    finalize(packet);
    if (!expect_rejected(packet, sizeof(packet))) {
        fputs("unsupported version accepted\n", stderr);
        return 1;
    }
    make_frame(packet);
    packet[PAYLOAD_OFFSET] = 0x0fU;
    packet[PAYLOAD_OFFSET + 1U] = 0xa1U; /* 4001 mm */
    finalize(packet);
    if (!expect_rejected(packet, sizeof(packet))) {
        fputs("out-of-range distance accepted\n", stderr);
        return 1;
    }
    make_frame(packet);
    packet[PAYLOAD_OFFSET + 1U] ^= 1U;
    if (!expect_rejected(packet, sizeof(packet))) {
        fputs("bad CRC accepted\n", stderr);
        return 1;
    }
    puts("SITL ToF frame decoder passed");
    return 0;
}
