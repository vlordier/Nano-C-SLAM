#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "tof_daq.h"
#include "tof_matrix.h"

SemaphoreHandle_t tofSemaphore0;
SemaphoreHandle_t tofSemaphore1;

static jmp_buf task_exit;
static unsigned int init_attempts;
static unsigned int read_attempts;
static unsigned int delay_count;
static TickType_t delays[16];
static int recover_after_attempts;
static int read_unavailable;
static unsigned int diagnostic_count;

void test_debug_print(const char *format, ...) {
    (void)format;
    diagnostic_count++;
}

tof_err_code_e tof_matrix_init(void) {
    init_attempts++;
    assert(!tof_is_init());
    if (recover_after_attempts >= 0 &&
        init_attempts > (unsigned int)recover_after_attempts)
        return TOF_SUCCESS;
    return TOF_CONFIG_ERROR;
}

tof_err_code_e get_tof_matrix(uint8_t sensor_id, int16_t matrix[][8]) {
    assert(!tof_is_init());
    assert(sensor_id < 4);
    read_attempts++;
    memset(matrix, 0, 8 * 8 * sizeof(int16_t));
    return read_unavailable ? TOF_NOT_READY : TOF_SUCCESS;
}

int xSemaphoreTake(SemaphoreHandle_t semaphore, unsigned int ticks) {
    (void)semaphore;
    (void)ticks;
    return 1;
}

int xSemaphoreGive(SemaphoreHandle_t semaphore) {
    (void)semaphore;
    return 1;
}

TickType_t xTaskGetTickCount(void) {
    return 0;
}

void vTaskDelay(TickType_t ticks) {
    if (delay_count < sizeof(delays) / sizeof(delays[0]))
        delays[delay_count] = ticks;
    delay_count++;
    if (recover_after_attempts < 0 && delay_count == 7)
        longjmp(task_exit, 1);
    if (read_unavailable && read_attempts >= 101)
        longjmp(task_exit, 1);
}

void vTaskDelayUntil(TickType_t *previous_wake_time, TickType_t increment) {
    (void)previous_wake_time;
    (void)increment;
    longjmp(task_exit, 1);
}

static void run_persistent_failure(void) {
    recover_after_attempts = -1;
    if (setjmp(task_exit) == 0)
        tof_task(NULL);

    const TickType_t expected[] = {100, 200, 400, 800, 1600, 3200, 5000};
    assert(delay_count == sizeof(expected) / sizeof(expected[0]));
    assert(memcmp(delays, expected, sizeof(expected)) == 0);
    assert(init_attempts == 7);
    assert(!tof_is_init());
    assert(read_attempts == 0);
}

static void run_recovery(void) {
    recover_after_attempts = 2;
    if (setjmp(task_exit) == 0)
        tof_task(NULL);

    assert(init_attempts == 3);
    assert(delay_count == 6);
    assert(delays[0] == 100 && delays[1] == 200);
    assert(tof_is_init());
    assert(read_attempts == 4);
}

static void run_unavailable_first_data(void) {
    recover_after_attempts = 0;
    read_unavailable = 1;
    if (setjmp(task_exit) == 0)
        tof_task(NULL);

    assert(init_attempts == 1);
    assert(read_attempts == 101);
    assert(!tof_is_init());
    assert(diagnostic_count > 0);
}

int main(int argc, char **argv) {
    assert(argc == 2);
    if (strcmp(argv[1], "persistent") == 0)
        run_persistent_failure();
    else if (strcmp(argv[1], "recovery") == 0)
        run_recovery();
    else if (strcmp(argv[1], "unavailable-data") == 0)
        run_unavailable_first_data();
    else
        assert(0 && "unknown test case");
    puts("STM32 ToF DAQ retry contract passed");
    return 0;
}
