#ifndef TEST_FLIGHT_TASK_H
#define TEST_FLIGHT_TASK_H

#include "FreeRTOS.h"

void vTaskDelay(TickType_t ticks);
void vTaskDelete(void *task);

#endif
