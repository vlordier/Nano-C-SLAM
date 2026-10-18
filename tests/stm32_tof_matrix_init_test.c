#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "i2c_expander.h"
#include "i2cdev.h"
#include "tof_matrix.h"
#include "vl53l5cx_api.h"

static unsigned int i2c_operation;
static unsigned int fail_i2c_operation;
static unsigned int sensor_api_call;
static unsigned int fail_sensor_api_call;
static uint8_t output_register;
static uint8_t ranging_ready;
static uint8_t ranging_data_status;
static VL53L5CX_ResultsData ranging_results;

void test_debug_print(const char *format, ...) {
    (void)format;
}

bool i2cdevReadByte(int device, uint8_t address, uint8_t reg, uint8_t *value) {
    (void)device;
    (void)address;
    i2c_operation++;
    if (i2c_operation == fail_i2c_operation)
        return false;
    assert(reg == OUTPUT_REG_ADDRESS);
    *value = output_register;
    return true;
}

bool i2cdevWriteByte(int device, uint8_t address, uint8_t reg, uint8_t value) {
    (void)device;
    (void)address;
    i2c_operation++;
    if (i2c_operation == fail_i2c_operation)
        return false;
    if (reg == OUTPUT_REG_ADDRESS)
        output_register = value;
    return true;
}

static uint8_t sensor_status(void) {
    sensor_api_call++;
    return sensor_api_call == fail_sensor_api_call ? 1 : VL53L5CX_STATUS_OK;
}

uint8_t vl53l5cx_init(VL53L5CX_Configuration *device) {
    (void)device;
    return sensor_status();
}
uint8_t vl53l5cx_set_i2c_address(VL53L5CX_Configuration *device, uint16_t address) {
    (void)device;
    (void)address;
    return sensor_status();
}
uint8_t vl53l5cx_set_resolution(VL53L5CX_Configuration *device, uint8_t resolution) {
    (void)device;
    (void)resolution;
    return sensor_status();
}
uint8_t vl53l5cx_set_ranging_frequency_hz(VL53L5CX_Configuration *device,
                                          uint8_t frequency) {
    (void)device;
    (void)frequency;
    return sensor_status();
}
uint8_t vl53l5cx_set_target_order(VL53L5CX_Configuration *device, uint8_t order) {
    (void)device;
    (void)order;
    return sensor_status();
}
uint8_t vl53l5cx_set_ranging_mode(VL53L5CX_Configuration *device, uint8_t mode) {
    (void)device;
    (void)mode;
    return sensor_status();
}
uint8_t vl53l5cx_start_ranging(VL53L5CX_Configuration *device) {
    (void)device;
    return sensor_status();
}
uint8_t vl53l5cx_check_data_ready(VL53L5CX_Configuration *device, uint8_t *ready) {
    (void)device;
    *ready = ranging_ready;
    return VL53L5CX_STATUS_OK;
}
uint8_t vl53l5cx_get_ranging_data(VL53L5CX_Configuration *device,
                                  VL53L5CX_ResultsData *results) {
    (void)device;
    *results = ranging_results;
    return ranging_data_status;
}

static void reset_mocks(void) {
    i2c_operation = 0;
    fail_i2c_operation = 0;
    sensor_api_call = 0;
    fail_sensor_api_call = 0;
    output_register = UINT8_MAX;
    ranging_ready = 0;
    ranging_data_status = VL53L5CX_STATUS_OK;
    for (unsigned int i = 0; i < 64; i++) {
        ranging_results.distance_mm[i] = (int16_t)(100 + i);
        ranging_results.nb_target_detected[i] = 1;
        ranging_results.target_status[i] = 5;
    }
}

int main(void) {
    reset_mocks();
    assert(tof_matrix_init() == TOF_SUCCESS);
    assert(sensor_api_call == 28);
    assert(output_register == 0x0f);

    for (unsigned int failed_step = 1; failed_step <= 28; failed_step++) {
        reset_mocks();
        fail_sensor_api_call = failed_step;
        assert(tof_matrix_init() == TOF_CONFIG_ERROR);
        assert(sensor_api_call == failed_step);
        assert(output_register == 0);

        fail_sensor_api_call = 0;
        sensor_api_call = 0;
        assert(tof_matrix_init() == TOF_SUCCESS);
        assert(output_register == 0x0f);
    }

    reset_mocks();
    for (unsigned int failed_step = 1; failed_step <= 11; failed_step++) {
        reset_mocks();
        fail_i2c_operation = failed_step;
        assert(tof_matrix_init() == TOF_CONFIG_ERROR);
        assert(output_register == 0);

        fail_i2c_operation = 0;
        i2c_operation = 0;
        sensor_api_call = 0;
        assert(tof_matrix_init() == TOF_SUCCESS);
        assert(output_register == 0x0f);
    }

    reset_mocks();
    assert(!I2C_expander_set_pin(0, 2));
    assert(i2c_operation == 0);
    fail_i2c_operation = 1;
    assert(!I2C_expander_set_pin(0, 1));
    assert(i2c_operation == 1);

    int16_t matrix[8][8];
    assert(get_tof_matrix(4, matrix) == TOF_CONFIG_ERROR);
    assert(get_tof_matrix(0, NULL) == TOF_CONFIG_ERROR);

    ranging_ready = 1;
    assert(get_tof_matrix(0, matrix) == TOF_SUCCESS);
    assert(matrix[0][0] == 100 && matrix[7][7] == 163);
    int16_t row[8];
    assert(get_tof_row(0, 3, row) == TOF_SUCCESS);
    assert(row[0] == 124 && row[7] == 131);
    assert(get_tof_row(0, 8, row) == TOF_CONFIG_ERROR);
    assert(get_tof_row(0, 0, NULL) == TOF_CONFIG_ERROR);

    ranging_ready = 0;
    for (unsigned int i = 0; i < 8; i++)
        row[i] = 42;
    assert(get_tof_row(0, 2, row) == TOF_NOT_READY);
    for (unsigned int i = 0; i < 8; i++)
        assert(row[i] == -1);

    ranging_ready = 1;
    ranging_data_status = 1;
    for (unsigned int i = 0; i < 8; i++)
        row[i] = 42;
    assert(get_tof_row(0, 2, row) == TOF_DATA_ERROR);
    for (unsigned int i = 0; i < 8; i++)
        assert(row[i] == -1);

    puts("STM32 ToF initialization fault injection passed");
    return 0;
}
