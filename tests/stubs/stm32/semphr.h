#ifndef TEST_SEMPHR_H
#define TEST_SEMPHR_H

typedef void *SemaphoreHandle_t;
int xSemaphoreTake(SemaphoreHandle_t semaphore, unsigned int ticks);
int xSemaphoreGive(SemaphoreHandle_t semaphore);

#endif
