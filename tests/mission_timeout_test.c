#include "mission_timeout.h"

#include <assert.h>
#include <stdint.h>

static void test_deadline_requests_landing_once_and_observes_completion(void) {
    mission_timeout_t state = {.started_at = 100};
    assert(mission_timeout_update(&state, 1099, 1000, 5000, FLIGHT_STATUS_ACTIVE) ==
           MISSION_TIMEOUT_CONTINUE);
    assert(!state.landing_requested);
    assert(mission_timeout_update(&state, 1100, 1000, 5000, FLIGHT_STATUS_ACTIVE) ==
           MISSION_TIMEOUT_REQUEST_LAND);
    assert(state.landing_requested);
    assert(state.landing_requested_at == 1100);
    assert(mission_timeout_update(&state, 1200, 1000, 5000, FLIGHT_STATUS_LANDING) ==
           MISSION_TIMEOUT_CONTINUE);
    assert(mission_timeout_update(&state, 1300, 1000, 5000, FLIGHT_STATUS_COMPLETE) ==
           MISSION_TIMEOUT_LANDED);
}

static void test_failed_flight_and_expired_landing_grace_are_terminal(void) {
    mission_timeout_t rejected = {.started_at = 0};
    assert(mission_timeout_update(&rejected, 10, 10, 5000, FLIGHT_STATUS_ACTIVE) ==
           MISSION_TIMEOUT_REQUEST_LAND);
    assert(mission_timeout_update(&rejected, 11, 10, 5000, FLIGHT_STATUS_LAND_FAILED) ==
           MISSION_TIMEOUT_FAILED);

    mission_timeout_t stalled = {.started_at = 0};
    assert(mission_timeout_update(&stalled, 10, 10, 5000, FLIGHT_STATUS_ACTIVE) ==
           MISSION_TIMEOUT_REQUEST_LAND);
    assert(mission_timeout_update(&stalled, 5009, 10, 5000, FLIGHT_STATUS_ACTIVE) ==
           MISSION_TIMEOUT_CONTINUE);
    assert(mission_timeout_update(&stalled, 5010, 10, 5000, FLIGHT_STATUS_ACTIVE) ==
           MISSION_TIMEOUT_FAILED);
}

static void test_deadline_survives_tick_wraparound(void) {
    mission_timeout_t state = {.started_at = UINT32_MAX - 4u};
    assert(mission_timeout_update(&state, 4, 9, 10, FLIGHT_STATUS_ACTIVE) ==
           MISSION_TIMEOUT_REQUEST_LAND);
}

int main(void) {
    test_deadline_requests_landing_once_and_observes_completion();
    test_failed_flight_and_expired_landing_grace_are_terminal();
    test_deadline_survives_tick_wraparound();
    return 0;
}
