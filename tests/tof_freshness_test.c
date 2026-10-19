#include "tof_freshness.h"

#include <assert.h>
#include <stdint.h>

static void test_requires_a_successful_frame_from_each_sensor(void) {
    tof_freshness_t state;
    tof_freshness_reset(&state);

    assert(!tof_freshness_all_fresh(&state, 100, 500));
    for (uint8_t sensor = 0; sensor < TOF_FRESHNESS_SENSOR_COUNT - 1; sensor++)
        assert(tof_freshness_mark_success(&state, sensor, 100));
    assert(!tof_freshness_all_fresh(&state, 100, 500));
    assert(!tof_freshness_mark_success(&state, TOF_FRESHNESS_SENSOR_COUNT, 100));
    assert(tof_freshness_mark_success(&state, 3, 100));
    assert(tof_freshness_all_fresh(&state, 600, 500));
    assert(!tof_freshness_all_fresh(&state, 601, 500));
}

static void test_tick_wraparound_uses_unsigned_elapsed_time(void) {
    tof_freshness_t state;
    const TickType_t last_tick = UINT32_MAX - 4u;
    tof_freshness_reset(&state);
    for (uint8_t sensor = 0; sensor < TOF_FRESHNESS_SENSOR_COUNT; sensor++)
        assert(tof_freshness_mark_success(&state, sensor, last_tick));

    assert(tof_freshness_all_fresh(&state, 4, 9));
    assert(!tof_freshness_all_fresh(&state, 5, 9));
}

int main(void) {
    test_requires_a_successful_frame_from_each_sensor();
    test_tick_wraparound_uses_unsigned_elapsed_time();
    return 0;
}
