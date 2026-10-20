/*
 * Explicit fail-closed ToF provider for the Linux SITL build.
 *
 * This source is selected instead of the physical I2C/VL53L5CX stack. A real
 * Gazebo matrix transport will replace this provider before any flight or
 * mapping acceptance run. Returning an error keeps tof_daq from declaring
 * stale or zero-filled matrices ready.
 */

#include <stddef.h>
#include <string.h>

#include "debug.h"
#include "tof_matrix.h"

tof_err_code_e tof_matrix_init(void) {
    DEBUG_PRINT(
        "NCS_SITL_DEVICE_MISSING device=tof_matrix provider=unconfigured\n");
    return TOF_CONFIG_ERROR;
}

tof_err_code_e config_sensors(VL53L5CX_Configuration *device,
                              uint16_t new_i2c_address) {
    (void)device;
    (void)new_i2c_address;
    return TOF_CONFIG_ERROR;
}

tof_err_code_e get_tof_matrix(uint8_t sensor_id, int16_t matrix[][8]) {
    if (matrix == NULL || sensor_id >= 4U) {
        return TOF_DATA_ERROR;
    }
    memset(matrix, 0, 8U * 8U * sizeof(matrix[0][0]));
    return TOF_NOT_READY;
}

tof_err_code_e get_tof_row(uint8_t sensor_id, uint8_t row_idx, int16_t *row) {
    if (row == NULL || sensor_id >= 4U || row_idx >= 8U) {
        return TOF_DATA_ERROR;
    }
    memset(row, 0, 8U * sizeof(row[0]));
    return TOF_NOT_READY;
}
