#include "mission_timeout.h"

#include <stddef.h>

static uint8_t flight_has_failed(flight_status_t status) {
    return status == FLIGHT_STATUS_TAKEOFF_FAILED ||
           status == FLIGHT_STATUS_LAND_FAILED ||
           status == FLIGHT_STATUS_ABORTED_BEFORE_TAKEOFF;
}

mission_timeout_action_t mission_timeout_update(mission_timeout_t *state,
                                                TickType_t now,
                                                TickType_t mission_duration,
                                                TickType_t landing_grace,
                                                flight_status_t flight_status) {
    if (state == NULL)
        return MISSION_TIMEOUT_FAILED;
    if (flight_status == FLIGHT_STATUS_COMPLETE)
        return MISSION_TIMEOUT_LANDED;
    if (flight_has_failed(flight_status))
        return MISSION_TIMEOUT_FAILED;

    if (!state->landing_requested &&
        (TickType_t)(now - state->started_at) >= mission_duration) {
        state->landing_requested = 1;
        state->landing_requested_at = now;
        return MISSION_TIMEOUT_REQUEST_LAND;
    }

    if (!state->landing_requested)
        return MISSION_TIMEOUT_CONTINUE;
    if ((TickType_t)(now - state->landing_requested_at) >= landing_grace)
        return MISSION_TIMEOUT_FAILED;

    return MISSION_TIMEOUT_CONTINUE;
}
