#include "FreeRTOS.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "flight_commander.h"
#include "task.h"
#include "tof_daq.h"

#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <string.h>

enum event {
    EVENT_TAKEOFF = 1,
    EVENT_SETPOINT,
    EVENT_RELAX_PRIORITY,
    EVENT_LAND,
    EVENT_DELETE,
};

static jmp_buf task_exit;
static int events[32];
static int event_count;
static int takeoff_result;
static int land_result;
static int land_calls;
static int setpoint_calls;
static bool arm_land_after_first_command;
static bool reset_land_after_first_failed_landing;
static bool enable_after_two_waits;
static bool stale_during_takeoff;
static bool stale_after_first_command;
static uint8_t tof_fresh = 1;
static int ready_waits;
static int takeoff_waits;
volatile uint8_t land;
int16_t tof_data_buf[4][8][8];

static void record_event(enum event event) {
    assert(event_count < (int)(sizeof(events) / sizeof(events[0])));
    events[event_count++] = event;
}

int crtpCommanderHighLevelTakeoffWithVelocity(float height_m, float velocity_m_s,
                                              bool relative) {
    assert(height_m == 0.7f);
    assert(velocity_m_s == 0.3f);
    assert(!relative);
    record_event(EVENT_TAKEOFF);
    return takeoff_result;
}

int crtpCommanderHighLevelLand(float height_m, float duration_s) {
    assert(height_m == 0.0f);
    assert(duration_s == 2.0f);
    land_calls++;
    record_event(EVENT_LAND);
    if (reset_land_after_first_failed_landing && land_calls == 1)
        return -1;
    return land_result;
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority) {
    assert(priority == COMMANDER_PRIORITY_EXTRX);
    assert(setpoint->mode.x == modeVelocity);
    assert(setpoint->mode.y == modeVelocity);
    assert(setpoint->mode.z == modeAbs);
    assert(setpoint->mode.yaw == modeVelocity);
    assert(setpoint->position.z == 0.7f);
    if (setpoint_calls == 0) {
        assert(fabsf(setpoint->velocity.x - (0.5f / 15.0f)) < 1e-6f);
        assert(setpoint->velocity.y == -0.08f);
        assert(fabsf(setpoint->attitudeRate.yaw - 85.95f) < 1e-4f);
    } else if (setpoint_calls == 1 && land_calls == 1 &&
               reset_land_after_first_failed_landing) {
        assert(setpoint->velocity.x == 0.0f);
        assert(setpoint->velocity.y == 0.0f);
        assert(setpoint->attitudeRate.yaw == 0.0f);
    } else {
        float expected_x = setpoint_calls == 2 ? (1.0f / 15.0f) : (0.5f / 15.0f);
        assert(fabsf(setpoint->velocity.x - expected_x) < 1e-6f);
        assert(setpoint->velocity.y == -0.08f);
        assert(fabsf(setpoint->attitudeRate.yaw - 85.95f) < 1e-4f);
    }
    setpoint_calls++;
    record_event(EVENT_SETPOINT);
}

void commanderRelaxPriority(void) {
    record_event(EVENT_RELAX_PRIORITY);
}

void vTaskDelay(TickType_t ticks) {
    if (stale_during_takeoff && ticks == 20 && ++takeoff_waits == 3)
        tof_fresh = 0;
    if (stale_after_first_command && ticks == 20 && setpoint_calls == 1)
        tof_fresh = 0;
    if (enable_after_two_waits && ticks == 100) {
        assert(event_count == 0);
        if (++ready_waits == 2) {
            land = 1;
            enable_commander();
        }
    }
    if (arm_land_after_first_command && ticks == 20 &&
        (setpoint_calls == 1 ||
         (reset_land_after_first_failed_landing && setpoint_calls == 3)))
        land = 1;
    if (reset_land_after_first_failed_landing && ticks == 100 && land_calls == 1)
        land = 0;
}

void vTaskDelete(void *task) {
    assert(task == NULL);
    record_event(EVENT_DELETE);
    longjmp(task_exit, 1);
}

int logGetVarId(const char *group, const char *name) {
    (void)group;
    (void)name;
    return 0;
}

float logGetFloat(int id) {
    (void)id;
    return 0.0f;
}

float line_detector(float *line, int length) {
    (void)line;
    (void)length;
    return 0.0f;
}

float get_min_dtd_dist(void) {
    return 2.0f;
}

uint8_t tof_is_fresh(void) {
    return tof_fresh;
}

void explore(uint16_t tof_data[64], float lines[3], float sides[2], float distance_back,
             float fly_commands[3], float dtd_distance) {
    (void)tof_data;
    (void)lines;
    (void)sides;
    (void)distance_back;
    (void)dtd_distance;
    fly_commands[0] = 0.12f;
    fly_commands[1] = -0.08f;
    fly_commands[2] = 1.5f;
}

static void test_abort_before_takeoff(void) {
    land = 1;
    enable_commander();
    if (setjmp(task_exit) == 0)
        flight_task(NULL);
    assert(flight_status == FLIGHT_STATUS_ABORTED_BEFORE_TAKEOFF);
    assert(event_count == 1 && events[0] == EVENT_DELETE);
    assert(land_calls == 0 && setpoint_calls == 0);
}

static void test_stale_tof_before_takeoff_aborts(void) {
    tof_fresh = 0;
    enable_commander();
    if (setjmp(task_exit) == 0)
        flight_task(NULL);
    assert(flight_status == FLIGHT_STATUS_ABORTED_BEFORE_TAKEOFF);
    assert(event_count == 1 && events[0] == EVENT_DELETE);
    assert(land_calls == 0 && setpoint_calls == 0);
}

static void test_stale_tof_during_takeoff_requests_landing(void) {
    stale_during_takeoff = true;
    enable_commander();
    if (setjmp(task_exit) == 0)
        flight_task(NULL);
    assert(flight_status == FLIGHT_STATUS_COMPLETE);
    assert(land_calls == 1 && setpoint_calls == 0);
    assert(event_count == 4);
    assert(events[0] == EVENT_TAKEOFF);
    assert(events[1] == EVENT_RELAX_PRIORITY);
    assert(events[2] == EVENT_LAND);
    assert(events[3] == EVENT_DELETE);
}

static void test_stale_tof_stops_active_movement(void) {
    stale_after_first_command = true;
    enable_commander();
    if (setjmp(task_exit) == 0)
        flight_task(NULL);
    assert(flight_status == FLIGHT_STATUS_COMPLETE);
    assert(land_calls == 1 && setpoint_calls == 1);
    assert(event_count == 5);
    assert(events[0] == EVENT_TAKEOFF);
    assert(events[1] == EVENT_SETPOINT);
    assert(events[2] == EVENT_RELAX_PRIORITY);
    assert(events[3] == EVENT_LAND);
    assert(events[4] == EVENT_DELETE);
}

static void test_ready_gate_prevents_early_commands(void) {
    enable_after_two_waits = true;
    if (setjmp(task_exit) == 0)
        flight_task(NULL);
    assert(ready_waits == 2);
    assert(flight_status == FLIGHT_STATUS_ABORTED_BEFORE_TAKEOFF);
    assert(event_count == 1 && events[0] == EVENT_DELETE);
    assert(land_calls == 0 && setpoint_calls == 0);
}

static void test_takeoff_failure_stops_before_setpoints(void) {
    takeoff_result = -1;
    enable_commander();
    if (setjmp(task_exit) == 0)
        flight_task(NULL);
    assert(flight_status == FLIGHT_STATUS_TAKEOFF_FAILED);
    assert(event_count == 2);
    assert(events[0] == EVENT_TAKEOFF && events[1] == EVENT_DELETE);
    assert(land_calls == 0 && setpoint_calls == 0);
}

static void test_takeoff_setpoint_priority_handoff_and_landing(void) {
    arm_land_after_first_command = true;
    enable_commander();
    if (setjmp(task_exit) == 0)
        flight_task(NULL);
    assert(flight_status == FLIGHT_STATUS_COMPLETE);
    assert(land_calls == 1 && setpoint_calls == 1);
    assert(event_count == 5);
    assert(events[0] == EVENT_TAKEOFF);
    assert(events[1] == EVENT_SETPOINT);
    assert(events[2] == EVENT_RELAX_PRIORITY);
    assert(events[3] == EVENT_LAND);
    assert(events[4] == EVENT_DELETE);
}

static void test_rejected_land_holds_position_until_retry(void) {
    arm_land_after_first_command = true;
    reset_land_after_first_failed_landing = true;
    enable_commander();
    if (setjmp(task_exit) == 0)
        flight_task(NULL);
    assert(flight_status == FLIGHT_STATUS_COMPLETE);
    assert(land_calls == 2);
    assert(setpoint_calls == 3);
    assert(events[event_count - 1] == EVENT_DELETE);
}

int main(int argc, char **argv) {
    assert(argc == 2);
    if (strcmp(argv[1], "abort-before-takeoff") == 0) {
        test_abort_before_takeoff();
    } else if (strcmp(argv[1], "ready-gate") == 0) {
        test_ready_gate_prevents_early_commands();
    } else if (strcmp(argv[1], "stale-before-takeoff") == 0) {
        test_stale_tof_before_takeoff_aborts();
    } else if (strcmp(argv[1], "stale-during-takeoff") == 0) {
        test_stale_tof_during_takeoff_requests_landing();
    } else if (strcmp(argv[1], "stale-active") == 0) {
        test_stale_tof_stops_active_movement();
    } else if (strcmp(argv[1], "takeoff-failure") == 0) {
        test_takeoff_failure_stops_before_setpoints();
    } else if (strcmp(argv[1], "land-handoff") == 0) {
        test_takeoff_setpoint_priority_handoff_and_landing();
    } else if (strcmp(argv[1], "land-retry") == 0) {
        test_rejected_land_holds_position_until_retry();
    } else {
        assert(!"unknown flight-control test case");
    }
    return 0;
}
