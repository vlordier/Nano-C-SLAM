/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "FreeRTOS.h"
#include "debug.h"
#include "pm.h"
#include "semphr.h"
#include "tof_matrix.h"
#include <string.h>

extern SemaphoreHandle_t tofSemaphore0;
extern SemaphoreHandle_t tofSemaphore1;
int16_t tof_data_buf[4][8][8];
static volatile uint8_t tof_init = 0;

#define TOF_INIT_RETRY_INITIAL_MS 100
#define TOF_INIT_RETRY_MAX_MS 5000

static TickType_t next_retry_delay(TickType_t delay) {
    if (delay >= M2T(TOF_INIT_RETRY_MAX_MS / 2))
        return M2T(TOF_INIT_RETRY_MAX_MS);
    return delay * 2;
}

void tof_task(void *parameters) {
    (void)parameters;
    TickType_t retry_delay = M2T(TOF_INIT_RETRY_INITIAL_MS);
    for (;;) {
        tof_err_code_e error = tof_matrix_init();
        if (error == TOF_SUCCESS) {
            DEBUG_PRINT(
                "ToF sensor matrix configured; waiting for first data\n");
            break;
        }

        DEBUG_PRINT("ToF sensor matrix initialization failed (%d); retrying in "
                    "%lu ms\n",
                    error, (unsigned long)retry_delay);
        vTaskDelay(retry_delay);
        retry_delay = next_retry_delay(retry_delay);
    }

    TickType_t prevWakeTime = xTaskGetTickCount();
    while (1) {
        int16_t matrix[8][8];
        // if(pmIsBatteryLow()) DEBUG_PRINT("Battery Low!\n");

        for (int16_t i = 0; i < 4; i++) {
            tof_err_code_e e = get_tof_matrix(i, matrix);
            uint16_t err_cnt = 0;
            while (e != TOF_SUCCESS) {
                err_cnt++;
                if (err_cnt % 10 == 0) {
                    xSemaphoreTake(tofSemaphore0, 0);
                    xSemaphoreTake(tofSemaphore1, 0);
                }
                if (err_cnt >= 100) {
                    DEBUG_PRINT(
                        "ToF sensor %d data unavailable (%d); retrying\n", i,
                        e);
                    err_cnt = 0;
                }
                vTaskDelay(M2T(5));
                e = get_tof_matrix(i, matrix);
            }
            memcpy(&tof_data_buf[i], matrix, 8 * 8 * sizeof(int16_t));
            vTaskDelay(M2T(1));
        }

        if (!tof_init) {
            tof_init = 1;
            DEBUG_PRINT("ToF sensor matrix ready with fresh data\n");
        }
        xSemaphoreGive(tofSemaphore0);
        xSemaphoreGive(tofSemaphore1);

        vTaskDelayUntil(&prevWakeTime, M2T(68));
    }
}

uint8_t tof_is_init() { return tof_init; }
