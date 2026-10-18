/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "math.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "crtp.h"
#include "debug.h"
#include "task.h"

#include "i2c_expander.h"
#include "tof_matrix.h"
#include "vl53l5cx_api.h"

#define NR_OF_SENSORS 4
#define NR_OF_PIXELS 64
#define TOF_I2C_ADDR 0x56

static VL53L5CX_Configuration tof_dev;
static VL53L5CX_ResultsData tof_data;
static uint8_t tof_i2c_addresses[NR_OF_SENSORS];

tof_err_code_e tof_matrix_init() {
    DEBUG_PRINT("Configured for %d ToF sensor(s) \n", NR_OF_SENSORS);
    // Power-cycle all sensors so a retry can recover after partial setup.
    if (!I2C_expander_initialize()) {
        DEBUG_PRINT("Failed to reset ToF sensor power enables\n");
        (void)I2C_expander_initialize();
        return TOF_CONFIG_ERROR;
    }

    // Define the address of each ToF matrix sensor
    for (uint8_t i = 0; i < NR_OF_SENSORS; i++)
        tof_i2c_addresses[i] = TOF_I2C_ADDR + 2 + 2 * i;

    // Configure the ToF sensor(s)
    for (uint8_t i = 0; i < NR_OF_SENSORS; i++) {
        if (!I2C_expander_set_pin(i, 1)) {
            DEBUG_PRINT("Failed to enable ToF sensor %u\n", i);
            (void)I2C_expander_initialize();
            return TOF_CONFIG_ERROR;
        }

        // Configure the current sensor
        tof_err_code_e e = config_sensors(&tof_dev, tof_i2c_addresses[i]);
        if (e == TOF_SUCCESS) {
            uint8_t status = vl53l5cx_start_ranging(&tof_dev); // Start ranging
            if (status == 0) {
                DEBUG_PRINT("Sensor %d started ranging! \n", i);
            } else {
                DEBUG_PRINT("Failed to start ToF sensor %u: %u\n", i, status);
                (void)I2C_expander_initialize();
                return TOF_CONFIG_ERROR;
            }
        } else {
            DEBUG_PRINT("Failed to configure ToF sensor %u: %u\n", i, e);
            (void)I2C_expander_initialize();
            return TOF_CONFIG_ERROR;
        }
    }

    return TOF_SUCCESS;
}

tof_err_code_e config_sensors(VL53L5CX_Configuration *p_dev,
                              uint16_t new_i2c_address) {
    p_dev->platform =
        VL53L5CX_DEFAULT_I2C_ADDRESS; // use default adress for first use

    // Initialize the sensor
    if (vl53l5cx_init(p_dev) != VL53L5CX_STATUS_OK)
        return TOF_CONFIG_ERROR;

    // Change I2C address
    if (vl53l5cx_set_i2c_address(p_dev, new_i2c_address) != VL53L5CX_STATUS_OK)
        return TOF_CONFIG_ERROR;
    if (vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8) !=
        VL53L5CX_STATUS_OK)
        return TOF_CONFIG_ERROR;

    // 15Hz frame rate
    if (vl53l5cx_set_ranging_frequency_hz(p_dev, 15) != VL53L5CX_STATUS_OK)
        return TOF_CONFIG_ERROR;
    if (vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST) !=
        VL53L5CX_STATUS_OK)
        return TOF_CONFIG_ERROR;
    if (vl53l5cx_set_ranging_mode(p_dev, VL53L5CX_RANGING_MODE_CONTINUOUS) !=
        VL53L5CX_STATUS_OK)
        return TOF_CONFIG_ERROR;

    return TOF_SUCCESS;
}

tof_err_code_e get_tof_matrix(uint8_t sensor_id, int16_t matrix[][8]) {
    tof_dev.platform = tof_i2c_addresses[sensor_id];

    uint8_t ranging_ready = 255;
    uint8_t get_data_success = 255;
    vl53l5cx_check_data_ready(&tof_dev, &ranging_ready); // poll for data-ready

    tof_err_code_e ret_value = TOF_SUCCESS;

    if (ranging_ready == 1) {
        get_data_success = vl53l5cx_get_ranging_data(&tof_dev, &tof_data);
        if (get_data_success == VL53L5CX_STATUS_OK) {
            memcpy(&matrix[0][0], &tof_data.distance_mm[0],
                   64 * sizeof(int16_t));
            for (int16_t i = 0; i < 64; i++) {
                if (tof_data.nb_target_detected[i] < 1)
                    matrix[i / 8][i % 8] = -1;

                if ((tof_data.target_status[i] != 5) &&
                    (tof_data.target_status[i] != 9))
                    matrix[i / 8][i % 8] = -1;
            }
        } else
            ret_value = TOF_DATA_ERROR;
    } else
        ret_value = TOF_NOT_READY;

    return ret_value;
}

tof_err_code_e get_tof_row(uint8_t sensor_id, uint8_t row_idx, int16_t *row) {
    int16_t matrix[8][8];
    tof_err_code_e ret_value = get_tof_matrix(sensor_id, matrix);
    memcpy(row, &matrix[row_idx][0], 8 * sizeof(int16_t));

    return ret_value;
}
