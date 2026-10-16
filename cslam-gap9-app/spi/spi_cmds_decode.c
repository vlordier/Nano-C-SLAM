/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#define CMD_OFFSET 1
#define ID_OFFSET 2
#define TIMESTAMP_OFFSET 6
#define DRONE_ID_OFFSET 6
#define POS_OFFSET 10
#define TOF_OFFSET 22

#include "spi_cmds_decode.h"

_Static_assert(sizeof(pcl_pose_t) + 6 <= SPI_PKT_SIZE,
               "pose packet exceeds SPI frame size");
_Static_assert(sizeof(ext_aug_pose_t) + 6 <= SPI_PKT_SIZE,
               "external pose packet exceeds SPI frame size");
_Static_assert(offsetof(pcl_pose_t, timestamp) == 4 &&
                   offsetof(pcl_pose_t, pos) == 8 &&
                   offsetof(pcl_pose_t, tof) == 20 && sizeof(pcl_pose_t) == 84,
               "pose layout no longer matches the STM32 SPI wire format");
_Static_assert(offsetof(ext_aug_pose_t, drone_id) == 4 &&
                   offsetof(ext_aug_pose_t, pos) == 8 &&
                   offsetof(ext_aug_pose_t, tof) == 20 &&
                   sizeof(ext_aug_pose_t) == 84,
               "external pose layout no longer matches the STM32 SPI format");

static int valid_frame(const uint8_t *buffer, size_t buffer_size) {
    return buffer != NULL && buffer_size >= SPI_PKT_SIZE && buffer[0] == 'S' &&
           buffer[SPI_PKT_SIZE - 1] == 'E';
}

static int16_t read_int16(const uint8_t *buffer, size_t offset) {
    int16_t value;
    memcpy(&value, &buffer[offset], sizeof(value));
    return value;
}

static int32_t read_int32(const uint8_t *buffer, size_t offset) {
    int32_t value;
    memcpy(&value, &buffer[offset], sizeof(value));
    return value;
}

static uint32_t read_uint32(const uint8_t *buffer, size_t offset) {
    uint32_t value;
    memcpy(&value, &buffer[offset], sizeof(value));
    return value;
}

/* Packet structure:
        'S'
        CMD
        ID
        TIMESTAMP
        POS
        TOF
        CHECKSUM
        'E'
*/
spi_transfer_t spi_decode_pose(const uint8_t *buffer, size_t buffer_size,
                               pcl_pose_t *pose) {
    if (!valid_frame(buffer, buffer_size) || pose == NULL) return SPI_ERR;
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != POSE_CMD) return SPI_WRONG_CMD;

    pcl_pose_t decoded;
    memset(&decoded, 0, sizeof(decoded));
    decoded.id = read_int16(buffer, ID_OFFSET);
    decoded.timestamp = read_int32(buffer, TIMESTAMP_OFFSET);
    memcpy(decoded.pos, &buffer[POS_OFFSET], sizeof(decoded.pos));
    memcpy(decoded.tof, &buffer[TOF_OFFSET], sizeof(decoded.tof));

    uint32_t checksum = 0;
    for (size_t i = 0; i < sizeof(decoded); i++)
        checksum += ((const uint8_t *)&decoded)[i];

    uint32_t checksum_stm32 = read_uint32(buffer, sizeof(decoded) + 2);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    *pose = decoded;
    return SPI_OK;
}

spi_transfer_t spi_decode_ext_pose(const uint8_t *buffer, size_t buffer_size,
                                   ext_aug_pose_t *pose) {
    if (!valid_frame(buffer, buffer_size) || pose == NULL) return SPI_ERR;
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != EXT_POSE_CMD) return SPI_WRONG_CMD;

    ext_aug_pose_t decoded;
    memset(&decoded, 0, sizeof(decoded));
    decoded.id = read_int16(buffer, ID_OFFSET);
    decoded.drone_id = read_int32(buffer, DRONE_ID_OFFSET);
    memcpy(decoded.pos, &buffer[POS_OFFSET], sizeof(decoded.pos));
    memcpy(decoded.tof, &buffer[TOF_OFFSET], sizeof(decoded.tof));

    uint32_t checksum = 0;
    for (size_t i = 0; i < sizeof(decoded); i++)
        checksum += ((const uint8_t *)&decoded)[i];

    uint32_t checksum_stm32 = read_uint32(buffer, sizeof(decoded) + 2);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    *pose = decoded;
    return SPI_OK;
}

spi_transfer_t spi_decode_ext_pose_upd(const uint8_t *buffer,
                                       size_t buffer_size,
                                       ext_pose_upd_t *pos_upd) {
    if (!valid_frame(buffer, buffer_size) || pos_upd == NULL) return SPI_ERR;
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != UPD_EXT_POSE_CMD) return SPI_WRONG_CMD;

    ext_pose_upd_t decoded;
    memset(&decoded, 0, sizeof(decoded));
    decoded.pose_id = read_int16(buffer, ID_OFFSET);
    decoded.drone_id = read_int16(buffer, ID_OFFSET + 2);
    memcpy(decoded.pos, &buffer[ID_OFFSET + 4], sizeof(decoded.pos));

    uint32_t checksum = 0;
    for (size_t i = 0; i < sizeof(decoded.pos); i++)
        checksum += ((const uint8_t *)decoded.pos)[i];

    uint32_t checksum_stm32 = read_uint32(buffer, 18);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    *pos_upd = decoded;
    return SPI_OK;
}

spi_transfer_t spi_decode_scan_info(const uint8_t *buffer, size_t buffer_size,
                                    int16_t *scan_info) {
    if (!valid_frame(buffer, buffer_size) || scan_info == NULL) return SPI_ERR;
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != SCAN_CMD) return SPI_WRONG_CMD;

    int16_t decoded_scan = read_int16(buffer, 2);

    int32_t checksum = 0;
    for (int16_t i = 2; i < 4; i++) checksum += buffer[i];

    int32_t checksum_stm32 = read_int16(buffer, 4);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    scan_info[0] = decoded_scan;
    return SPI_OK;
}

spi_transfer_t spi_decode_pose_req(const uint8_t *buffer, size_t buffer_size,
                                   int16_t *pose_request) {
    if (!valid_frame(buffer, buffer_size) || pose_request == NULL)
        return SPI_ERR;
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != POSE_REQ_CMD) return SPI_WRONG_CMD;

    int16_t decoded_request[2] = {read_int16(buffer, 2), read_int16(buffer, 4)};

    int32_t checksum = 0;
    for (int16_t i = 2; i < 6; i++) checksum += buffer[i];

    int32_t checksum_stm32 = read_int16(buffer, 6);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    memcpy(pose_request, decoded_request, sizeof(decoded_request));
    return SPI_OK;
}
