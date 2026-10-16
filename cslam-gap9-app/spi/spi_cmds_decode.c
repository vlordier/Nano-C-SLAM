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
spi_transfer_t spi_decode_pose(uint8_t *buffer, pcl_pose_t *pose) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != POSE_CMD) return SPI_WRONG_CMD;

    memset(pose, 0, sizeof(pcl_pose_t));
    pose->id = read_int16(buffer, ID_OFFSET);
    pose->timestamp = read_int32(buffer, TIMESTAMP_OFFSET);
    memcpy(pose->pos, &buffer[POS_OFFSET], 3 * sizeof(float));
    memcpy(pose->tof, &buffer[TOF_OFFSET], 4 * 8 * sizeof(int16_t));

    uint32_t checksum = 0;
    for (size_t i = 0; i < sizeof(pcl_pose_t); i++)
        checksum += *((uint8_t *)pose + i);

    uint32_t checksum_stm32 = read_uint32(buffer, sizeof(pcl_pose_t) + 2);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}

spi_transfer_t spi_decode_ext_pose(uint8_t *buffer, ext_aug_pose_t *pose) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != EXT_POSE_CMD) return SPI_WRONG_CMD;

    memset(pose, 0, sizeof(ext_aug_pose_t));
    pose->id = read_int16(buffer, ID_OFFSET);
    pose->drone_id = read_int32(buffer, DRONE_ID_OFFSET);
    memcpy(pose->pos, &buffer[POS_OFFSET], 3 * sizeof(float));
    memcpy(pose->tof, &buffer[TOF_OFFSET], 4 * 8 * sizeof(int16_t));

    uint32_t checksum = 0;
    for (size_t i = 0; i < sizeof(ext_aug_pose_t); i++)
        checksum += *((uint8_t *)pose + i);

    uint32_t checksum_stm32 = read_uint32(buffer, sizeof(ext_aug_pose_t) + 2);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}

spi_transfer_t spi_decode_ext_pose_upd(uint8_t *buffer,
                                       ext_pose_upd_t *pos_upd) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != UPD_EXT_POSE_CMD) return SPI_WRONG_CMD;

    memset(pos_upd, 0, sizeof(ext_pose_upd_t));
    pos_upd->pose_id = read_int16(buffer, ID_OFFSET);
    pos_upd->drone_id = read_int16(buffer, ID_OFFSET + 2);
    memcpy(pos_upd->pos, &buffer[ID_OFFSET + 4], 3 * sizeof(float));

    uint32_t checksum = 0;
    for (size_t i = 0; i < 3 * sizeof(float); i++)
        checksum += *((uint8_t *)pos_upd->pos + i);

    uint32_t checksum_stm32 = read_uint32(buffer, 18);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}

spi_transfer_t spi_decode_scan_info(uint8_t *buffer, int16_t *scan_info) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != SCAN_CMD) return SPI_WRONG_CMD;

    scan_info[0] = read_int16(buffer, 2);

    int32_t checksum = 0;
    for (int16_t i = 2; i < 4; i++) checksum += buffer[i];

    int32_t checksum_stm32 = read_int16(buffer, 4);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}

spi_transfer_t spi_decode_pose_req(uint8_t *buffer, int16_t *pose_request) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != POSE_REQ_CMD) return SPI_WRONG_CMD;

    pose_request[0] = read_int16(buffer, 2);
    pose_request[1] = read_int16(buffer, 4);

    int32_t checksum = 0;
    for (int16_t i = 2; i < 6; i++) checksum += buffer[i];

    int32_t checksum_stm32 = read_int16(buffer, 6);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}
