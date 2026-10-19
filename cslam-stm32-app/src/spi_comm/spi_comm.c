/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "spi_comm.h"
#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "scan.h"
#include "spi_wait.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

#define MAX_PKT_SIZE (97)
#define POSE_RESPONSE_CAPACITY 50
#define GAP9_RESPONSE_READY_TIMEOUT_TICKS M2T(100)
#define GAP9_BUSY_TIMEOUT_TICKS M2T(10000)
#define GPIO_POLL_TICKS M2T(1)

typedef enum {
    POSE_CMD = 0,
    SCAN_CMD = 1,
    OPTIMIZE_CMD = 2,
    POSE_REQ_CMD = 3,
    CHECK_CMD = 4,
    EXT_AUG_POSE_CMD = 5,
    UPD_POSE_CMD = 6,
} spi_cmd_t;

static uint8_t to_send[2 * MAX_PKT_SIZE];
static uint8_t to_recv[601];

static int read_gap9_busy(void *context) {
    (void)context;
    return digitalRead(DECK_GPIO_IO2);
}

static uint32_t get_freertos_ticks(void *context) {
    (void)context;
    return (uint32_t)xTaskGetTickCount();
}

static void delay_freertos_ticks(void *context, uint32_t ticks) {
    (void)context;
    vTaskDelay((TickType_t)ticks);
}

static bool wait_gap9_busy_level(int expected_level, uint32_t timeout_ticks) {
    const spi_wait_ops_t ops = {
        .context = NULL,
        .read_level = read_gap9_busy,
        .get_ticks = get_freertos_ticks,
        .delay_ticks = delay_freertos_ticks,
    };
    return spi_wait_for_level(&ops, expected_level, timeout_ticks,
                              GPIO_POLL_TICKS);
}

void spi_dummy_clk_cycles() {
    uint8_t spi_dummy_rev = 0, spi_dummy_tr = 0;
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    spiExchange(1, &spi_dummy_tr, &spi_dummy_rev);
    spiEndTransaction();
}

void spi_send_aug_pose(aug_pose_t *pose) {
    to_send[0] = 'S';
    to_send[1] = POSE_CMD;
    size_t buf_idx = 2;
    memcpy(&to_send[buf_idx], pose, sizeof(aug_pose_t));

    uint32_t checksum = 0;
    for (size_t i = 0; i < sizeof(aug_pose_t); i++) {
        checksum += *((uint8_t *)pose + i);
    }
    buf_idx += sizeof(aug_pose_t);
    memcpy(&to_send[buf_idx], &checksum, sizeof(uint32_t));

    buf_idx += sizeof(uint32_t);
    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();
}

void spi_send_ext_aug_pose(ext_aug_pose_t *pose) {
    to_send[0] = 'S';
    to_send[1] = EXT_AUG_POSE_CMD;
    size_t buf_idx = 2;
    memcpy(&to_send[buf_idx], pose, sizeof(ext_aug_pose_t));

    uint32_t checksum = 0;
    for (size_t i = 0; i < sizeof(ext_aug_pose_t); i++) {
        checksum += *((uint8_t *)pose + i);
    }
    buf_idx += sizeof(ext_aug_pose_t);
    memcpy(&to_send[buf_idx], &checksum, sizeof(uint32_t));

    buf_idx += sizeof(uint32_t);
    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();
}

void spi_send_ext_pose_upd(float *pose, int16_t id, int16_t drone_id) {
    to_send[0] = 'S';
    to_send[1] = UPD_POSE_CMD;
    size_t buf_idx = 2;

    memcpy(&to_send[buf_idx], &id, sizeof(int16_t));
    buf_idx += sizeof(int16_t);

    memcpy(&to_send[buf_idx], &drone_id, sizeof(int16_t));
    buf_idx += sizeof(int16_t);

    memcpy(&to_send[buf_idx], pose, 3 * sizeof(float));
    buf_idx += 3 * sizeof(float);

    uint32_t checksum = 0;
    for (size_t i = 0; i < 3 * sizeof(float); i++) {
        checksum += *((uint8_t *)pose + i);
    }

    memcpy(&to_send[buf_idx], &checksum, sizeof(uint32_t));
    buf_idx += sizeof(uint32_t);

    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();
}

bool spi_send_scan_info(int16_t scan_pose_id) {
    to_send[0] = 'S';
    to_send[1] = SCAN_CMD;
    memcpy(&to_send[2], &scan_pose_id, sizeof(int16_t));

    uint32_t checksum = 0;
    for (int16_t i = 2; i < 4; i++) {
        checksum += to_send[i];
    }
    memcpy(&to_send[4], &checksum, sizeof(uint32_t));

    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    vTaskDelay(M2T(5));
    DEBUG_PRINT("SPI sent scan info!\n");
    if (!wait_gap9_busy_level(0, GAP9_BUSY_TIMEOUT_TICKS)) {
        DEBUG_PRINT("GAP9 scan processing timed out\n");
        return false;
    }

    vTaskDelay(M2T(2));
    return true;
}

bool spi_send_opt_cmd(slam_job_t *job) {
    if (job == NULL)
        return false;
    memset(job, 0, sizeof(slam_job_t));
    to_send[0] = 'S';
    to_send[1] = OPTIMIZE_CMD;
    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    vTaskDelay(M2T(5));
    if (!wait_gap9_busy_level(0, GAP9_BUSY_TIMEOUT_TICKS)) {
        DEBUG_PRINT("GAP9 optimization timed out\n");
        return false;
    }
    vTaskDelay(M2T(3));

    // Wait feedback
    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(sizeof(slam_job_t), to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    memcpy(job, to_recv, sizeof(slam_job_t));
    return true;
}

void spi_send_aug_pose_req(int16_t from_id, int16_t nr_of_poses) {
    to_send[0] = 'S';
    to_send[1] = POSE_REQ_CMD;
    memcpy(&to_send[2], &from_id, sizeof(int16_t));
    memcpy(&to_send[4], &nr_of_poses, sizeof(int16_t));

    uint32_t checksum = 0;
    for (int16_t i = 2; i < 6; i++) {
        checksum += to_send[i];
    }
    memcpy(&to_send[6], &checksum, sizeof(uint32_t));

    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();
}

bool spi_rcv_poses(float *poses, int16_t from_id, int16_t nr_of_poses) {
    if (poses == NULL || from_id < 0 || nr_of_poses <= 0 ||
        nr_of_poses > POSE_RESPONSE_CAPACITY) {
        return false;
    }
    spi_send_aug_pose_req(from_id, nr_of_poses);

    if (!wait_gap9_busy_level(1, GAP9_RESPONSE_READY_TIMEOUT_TICKS)) {
        DEBUG_PRINT("GAP9 pose response was not ready\n");
        return false;
    }
    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(3 * 4 * nr_of_poses, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    if (!wait_gap9_busy_level(0, GAP9_RESPONSE_READY_TIMEOUT_TICKS)) {
        DEBUG_PRINT("GAP9 pose response did not complete\n");
        return false;
    }

    memcpy(poses, to_recv, 3 * sizeof(float) * nr_of_poses);
    return true;
}
