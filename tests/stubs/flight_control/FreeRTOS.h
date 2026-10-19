#ifndef TEST_FLIGHT_FREERTOS_H
#define TEST_FLIGHT_FREERTOS_H

#include <stdint.h>

typedef uint32_t TickType_t;

#define M2T(milliseconds) ((TickType_t)(milliseconds))

#endif
