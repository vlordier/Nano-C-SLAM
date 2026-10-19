#include "tof_freshness.h"

#include <stddef.h>

void tof_freshness_reset(volatile tof_freshness_t *state) {
    if (state == NULL)
        return;

    state->seen_mask = 0;
    for (uint8_t i = 0; i < TOF_FRESHNESS_SENSOR_COUNT; i++)
        state->last_success[i] = 0;
}

bool tof_freshness_mark_success(volatile tof_freshness_t *state,
                                uint8_t sensor_id, TickType_t now) {
    if (state == NULL || sensor_id >= TOF_FRESHNESS_SENSOR_COUNT)
        return false;

    state->last_success[sensor_id] = now;
    state->seen_mask |= (uint8_t)(1u << sensor_id);
    return true;
}

bool tof_freshness_all_fresh(const volatile tof_freshness_t *state,
                             TickType_t now, TickType_t max_age) {
    const uint8_t all_sensors_mask =
        (uint8_t)((1u << TOF_FRESHNESS_SENSOR_COUNT) - 1u);
    if (state == NULL || state->seen_mask != all_sensors_mask)
        return false;

    for (uint8_t i = 0; i < TOF_FRESHNESS_SENSOR_COUNT; i++) {
        if ((TickType_t)(now - state->last_success[i]) > max_age)
            return false;
    }
    return true;
}
