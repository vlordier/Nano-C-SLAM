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
    *ready = 0;
    return VL53L5CX_STATUS_OK;
}
uint8_t vl53l5cx_get_ranging_data(VL53L5CX_Configuration *device,
                                  VL53L5CX_ResultsData *results) {
    (void)device;
    (void)results;
    return VL53L5CX_STATUS_OK;
}

static void reset_mocks(void) {
    i2c_operation = 0;
    fail_i2c_operation = 0;
    sensor_api_call = 0;
    fail_sensor_api_call = 0;
    output_register = UINT8_MAX;
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
    puts("STM32 ToF initialization fault injection passed");
    return 0;
}
