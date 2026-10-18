#ifndef TEST_I2C_EXPANDER_H
#define TEST_I2C_EXPANDER_H

#include <stdbool.h>
#include <stdint.h>

#define OUTPUT_REG_ADDRESS ((uint8_t)0x01)

bool I2C_expander_set_pin(uint8_t pin_number, uint8_t pin_value);
bool I2C_expander_initialize(void);

#endif
