#include "spi_wait.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t ticks;
    int level;
    uint32_t delay_count;
    uint32_t ready_after_delays;
} fake_clock_t;

static int read_level(void *context) {
    return ((fake_clock_t *)context)->level;
}

static uint32_t get_ticks(void *context) {
    return ((fake_clock_t *)context)->ticks;
}

static void delay_ticks(void *context, uint32_t ticks) {
    fake_clock_t *clock = context;
    clock->ticks += ticks;
    clock->delay_count++;
    if (clock->ready_after_delays != 0 &&
        clock->delay_count == clock->ready_after_delays)
        clock->level = 1;
}

static void test_wait_for_level(void) {
    fake_clock_t clock = {.ticks = 10, .level = 0, .ready_after_delays = 2};
    spi_wait_ops_t ops = {&clock, read_level, get_ticks, delay_ticks};
    assert(spi_wait_for_level(&ops, 1, 5, 1));
    assert(clock.delay_count == 2);
}

static void test_timeout_wraps_ticks(void) {
    fake_clock_t clock = {.ticks = UINT32_MAX - 1, .level = 0};
    spi_wait_ops_t ops = {&clock, read_level, get_ticks, delay_ticks};
    assert(!spi_wait_for_level(&ops, 1, 3, 1));
    assert(clock.ticks == 1);
}

static void test_invalid_ops_fail(void) {
    fake_clock_t clock = {0};
    spi_wait_ops_t ops = {&clock, read_level, get_ticks, delay_ticks};
    assert(!spi_wait_for_level(NULL, 0, 1, 1));
    assert(!spi_wait_for_level(&ops, 0, 0, 1));
    assert(!spi_wait_for_level(&ops, 0, 1, 0));
}

int main(void) {
    test_wait_for_level();
    test_timeout_wraps_ticks();
    test_invalid_ops_fail();
    return 0;
}
