#ifndef TEST_I2CDEV_H
#define TEST_I2CDEV_H

#include <stdbool.h>
#include <stdint.h>

#define I2C1_DEV 1
bool i2cdevReadByte(int device, uint8_t address, uint8_t reg, uint8_t *value);
bool i2cdevWriteByte(int device, uint8_t address, uint8_t reg, uint8_t value);

#endif
