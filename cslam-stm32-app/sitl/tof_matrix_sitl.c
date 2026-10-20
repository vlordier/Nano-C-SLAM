/*
 * UDP-backed 4x8x8 ToF provider for Linux CrazySim SITL.
 *
 * Datagram format is documented in docs/sitl/tof-matrix-udp.md. This file is
 * Linux SITL-only; embedded builds continue to use the physical ToF driver.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "task.h"
#include "tof_matrix.h"

#define TOF_SENSOR_COUNT 4U
#define TOF_SIDE 8U
#define TOF_MAX_MM 4000
#define TOF_MAX_AGE_MS 500U
#define TOF_HEADER_SIZE 24U
#define TOF_PAYLOAD_SIZE (TOF_SENSOR_COUNT * TOF_SIDE * TOF_SIDE * 2U)
#define TOF_FRAME_SIZE (TOF_HEADER_SIZE + TOF_PAYLOAD_SIZE + 4U)
#define TOF_DEFAULT_PORT 19951U
#define TOF_REGISTRATION_SIZE 10U
#define TOF_REGISTRATION_INTERVAL_MS 250U
#ifndef NCS_SITL_NANO_NODE_ID
#define NCS_SITL_NANO_NODE_ID 0
#endif

static int receiver_fd = -1;
static struct sockaddr_in bridge_address;
static int16_t latest_frame[TOF_SENSOR_COUNT][TOF_SIDE][TOF_SIDE];
/* The provider is polled by tof_task only. Keep its 540-byte packet and
 * 512-byte candidate off the task's limited stack. */
static uint8_t receive_packet[TOF_FRAME_SIZE];
static int16_t candidate_frame[TOF_SENSOR_COUNT][TOF_SIDE][TOF_SIDE];
static uint32_t latest_sequence;
static uint64_t latest_sim_time_ns;
static TickType_t latest_receive_tick;
static TickType_t last_registration_tick;
static uint8_t frame_ready;

/* Exposed for hardware-independent host tests of the wire decoder. */
uint32_t tof_matrix_sitl_crc32(const uint8_t *bytes, size_t length) {
    uint32_t crc = UINT32_C(0xffffffff);
    if (bytes == NULL) {
        return 0U;
    }
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8U; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    return ~crc;
}

static uint16_t read_be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t read_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static void write_be32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static uint64_t read_be64(const uint8_t *p) {
    return ((uint64_t)read_be32(p) << 32) | read_be32(p + 4);
}

int tof_matrix_sitl_decode(const uint8_t *packet, size_t length,
                           uint32_t *sequence, uint64_t *sim_time_ns,
                           int16_t matrix[TOF_SENSOR_COUNT][TOF_SIDE][TOF_SIDE]) {
    if (packet == NULL || sequence == NULL || sim_time_ns == NULL ||
        matrix == NULL || length != TOF_FRAME_SIZE ||
        memcmp(packet, "NCTF", 4U) != 0 || packet[4] != 1U ||
        packet[5] != TOF_SENSOR_COUNT || packet[6] != TOF_SIDE ||
        packet[7] != TOF_SIDE || read_be32(packet + 20) != TOF_PAYLOAD_SIZE ||
        tof_matrix_sitl_crc32(packet, length - 4U) !=
            read_be32(packet + length - 4U)) {
        return -1;
    }

    const uint32_t decoded_sequence = read_be32(packet + 8);
    const uint64_t decoded_sim_time = read_be64(packet + 12);
    if (decoded_sim_time == 0U) {
        return -1;
    }
    size_t offset = TOF_HEADER_SIZE;
    for (size_t sensor = 0; sensor < TOF_SENSOR_COUNT; ++sensor) {
        for (size_t cell = 0; cell < TOF_SIDE * TOF_SIDE; ++cell) {
            const uint16_t raw = read_be16(packet + offset);
            if (raw != UINT16_MAX && raw > TOF_MAX_MM) {
                return -1;
            }
            const int16_t mm = raw == UINT16_MAX ? -1 : (int16_t)raw;
            matrix[sensor][cell / TOF_SIDE][cell % TOF_SIDE] = mm;
            offset += 2U;
        }
    }
    *sequence = decoded_sequence;
    *sim_time_ns = decoded_sim_time;
    return 0;
}

static uint16_t configured_port(void) {
    const char *value = getenv("NCS_SITL_TOF_UDP_PORT");
    if (value == NULL || value[0] == '\0') {
        return TOF_DEFAULT_PORT;
    }
    char *end = NULL;
    const unsigned long port = strtoul(value, &end, 10);
    if (end == value || *end != '\0' || port == 0UL || port > 65535UL) {
        return 0U;
    }
    return (uint16_t)port;
}

static uint16_t configured_bridge_port(void) {
    const char *value = getenv("NCS_SITL_TOF_BRIDGE_PORT");
    if (value == NULL || value[0] == '\0') {
        return TOF_DEFAULT_PORT;
    }
    char *end = NULL;
    const unsigned long port = strtoul(value, &end, 10);
    if (end == value || *end != '\0' || port == 0UL || port > 65535UL) {
        return 0U;
    }
    return (uint16_t)port;
}

static void send_registration(void) {
    uint8_t packet[TOF_REGISTRATION_SIZE] = {
        'N', 'C', 'T', 'R', 1U, (uint8_t)NCS_SITL_NANO_NODE_ID};
    write_be32(packet + sizeof(packet) - 4U,
               tof_matrix_sitl_crc32(packet, sizeof(packet) - 4U));
    (void)sendto(receiver_fd, packet, sizeof(packet), 0,
                 (const struct sockaddr *)&bridge_address,
                 sizeof(bridge_address));
    last_registration_tick = xTaskGetTickCount();
}

tof_err_code_e tof_matrix_init(void) {
    if (receiver_fd >= 0) {
        return TOF_SUCCESS;
    }
    const uint16_t port = configured_port();
    const uint16_t bridge_port = configured_bridge_port();
    if (port == 0U || bridge_port == 0U) {
        DEBUG_PRINT("NCS_SITL_DEVICE_ERROR device=tof_matrix reason=bad_port\n");
        return TOF_CONFIG_ERROR;
    }
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        DEBUG_PRINT("NCS_SITL_DEVICE_ERROR device=tof_matrix reason=socket\n");
        return TOF_CONFIG_ERROR;
    }
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        close(fd);
        return TOF_CONFIG_ERROR;
    }
    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    const char *bridge_host = getenv("NCS_SITL_TOF_BRIDGE_HOST");
    if (bridge_host == NULL || bridge_host[0] == '\0') {
        bridge_host = "192.168.5.2";
    }
    memset(&bridge_address, 0, sizeof(bridge_address));
    bridge_address.sin_family = AF_INET;
    bridge_address.sin_port = htons(bridge_port);
    if (inet_pton(AF_INET, bridge_host, &bridge_address.sin_addr) != 1) {
        close(fd);
        return TOF_CONFIG_ERROR;
    }
    if (bind(fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        DEBUG_PRINT("NCS_SITL_DEVICE_ERROR device=tof_matrix reason=bind\n");
        close(fd);
        return TOF_CONFIG_ERROR;
    }
    receiver_fd = fd;
    send_registration();
    DEBUG_PRINT("NCS_SITL_DEVICE_READY device=tof_matrix port=%u\n", port);
    return TOF_SUCCESS;
}

tof_err_code_e config_sensors(VL53L5CX_Configuration *device,
                              uint16_t new_i2c_address) {
    (void)device;
    (void)new_i2c_address;
    return TOF_CONFIG_ERROR;
}

static void receive_latest_frame(void) {
    const TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - last_registration_tick) >=
        M2T(TOF_REGISTRATION_INTERVAL_MS)) {
        send_registration();
    }
    for (;;) {
        struct sockaddr_in sender;
        socklen_t sender_length = (socklen_t)sizeof(sender);
        const ssize_t received = recvfrom(
            receiver_fd, receive_packet, sizeof(receive_packet),
            MSG_DONTWAIT | MSG_TRUNC,
            (struct sockaddr *)&sender, &sender_length);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                frame_ready = 0U;
            }
            return;
        }
        if ((size_t)received != sizeof(receive_packet)) {
            continue;
        }
        if (sender.sin_family != AF_INET ||
            sender.sin_addr.s_addr != bridge_address.sin_addr.s_addr ||
            sender.sin_port != bridge_address.sin_port) {
            continue;
        }
        uint32_t sequence;
        uint64_t sim_time_ns;
        if (tof_matrix_sitl_decode(receive_packet, sizeof(receive_packet),
                                   &sequence, &sim_time_ns,
                                   candidate_frame) != 0) {
            continue;
        }
        const uint32_t sequence_delta = sequence - latest_sequence;
        if (frame_ready &&
            (sequence_delta == 0U || sequence_delta >= UINT32_C(0x80000000) ||
             sim_time_ns <= latest_sim_time_ns)) {
            continue;
        }
        memcpy(latest_frame, candidate_frame, sizeof(latest_frame));
        latest_sequence = sequence;
        latest_sim_time_ns = sim_time_ns;
        latest_receive_tick = xTaskGetTickCount();
        frame_ready = 1U;
    }
}

static int frame_is_fresh(void) {
    return frame_ready &&
           (TickType_t)(xTaskGetTickCount() - latest_receive_tick) <=
               M2T(TOF_MAX_AGE_MS);
}

tof_err_code_e get_tof_matrix(uint8_t sensor_id, int16_t matrix[][8]) {
    if (matrix == NULL || sensor_id >= TOF_SENSOR_COUNT) {
        return TOF_DATA_ERROR;
    }
    if (receiver_fd < 0) {
        return TOF_NOT_READY;
    }
    /* Poll once at sensor zero so all four reads observe one coherent frame. */
    if (sensor_id == 0U) {
        receive_latest_frame();
    }
    if (!frame_is_fresh()) {
        return TOF_NOT_READY;
    }
    memcpy(matrix, latest_frame[sensor_id], sizeof(latest_frame[sensor_id]));
    return TOF_SUCCESS;
}

tof_err_code_e get_tof_row(uint8_t sensor_id, uint8_t row_idx, int16_t *row) {
    if (row == NULL || sensor_id >= TOF_SENSOR_COUNT || row_idx >= TOF_SIDE) {
        return TOF_DATA_ERROR;
    }
    int16_t matrix[TOF_SIDE][TOF_SIDE];
    const tof_err_code_e result = get_tof_matrix(sensor_id, matrix);
    if (result != TOF_SUCCESS) {
        return result;
    }
    memcpy(row, matrix[row_idx], sizeof(matrix[row_idx]));
    return TOF_SUCCESS;
}
