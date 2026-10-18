#ifndef TEST_VL53L5CX_API_H
#define TEST_VL53L5CX_API_H

#include <stdint.h>

#define VL53L5CX_DEFAULT_I2C_ADDRESS ((uint16_t)0x52)
#define VL53L5CX_RESOLUTION_8X8 ((uint8_t)64)
#define VL53L5CX_TARGET_ORDER_CLOSEST ((uint8_t)1)
#define VL53L5CX_RANGING_MODE_CONTINUOUS ((uint8_t)1)
#define VL53L5CX_STATUS_OK ((uint8_t)0)

typedef struct {
    uint16_t platform;
} VL53L5CX_Configuration;

typedef struct {
    int16_t distance_mm[64];
    uint8_t nb_target_detected[64];
    uint8_t target_status[64];
    uint16_t range_sigma_mm[64];
    uint32_t signal_per_spad[64];
} VL53L5CX_ResultsData;

uint8_t vl53l5cx_init(VL53L5CX_Configuration *device);
uint8_t vl53l5cx_set_i2c_address(VL53L5CX_Configuration *device,
                                 uint16_t address);
uint8_t vl53l5cx_set_resolution(VL53L5CX_Configuration *device,
                                uint8_t resolution);
uint8_t vl53l5cx_set_ranging_frequency_hz(VL53L5CX_Configuration *device,
                                           uint8_t frequency);
uint8_t vl53l5cx_set_target_order(VL53L5CX_Configuration *device,
                                  uint8_t order);
uint8_t vl53l5cx_set_ranging_mode(VL53L5CX_Configuration *device,
                                  uint8_t mode);
uint8_t vl53l5cx_start_ranging(VL53L5CX_Configuration *device);
uint8_t vl53l5cx_check_data_ready(VL53L5CX_Configuration *device,
                                  uint8_t *ready);
uint8_t vl53l5cx_get_ranging_data(VL53L5CX_Configuration *device,
                                  VL53L5CX_ResultsData *results);

#endif
