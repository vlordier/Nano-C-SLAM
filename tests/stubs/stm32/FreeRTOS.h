#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H

#include <stdint.h>

typedef uint32_t TickType_t;
#define M2T(milliseconds) ((TickType_t)(milliseconds))

TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *previous_wake_time, TickType_t increment);

#endif
