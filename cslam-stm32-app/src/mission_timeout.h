#ifndef MISSION_TIMEOUT_H
#define MISSION_TIMEOUT_H

#include "FreeRTOS.h"
#include "flight_commander.h"

#include <stdint.h>

typedef struct {
    TickType_t started_at;
    TickType_t landing_requested_at;
    uint8_t landing_requested;
} mission_timeout_t;

typedef enum {
    MISSION_TIMEOUT_CONTINUE = 0,
    MISSION_TIMEOUT_REQUEST_LAND = 1,
    MISSION_TIMEOUT_LANDED = 2,
    MISSION_TIMEOUT_FAILED = 3,
} mission_timeout_action_t;

mission_timeout_action_t mission_timeout_update(mission_timeout_t *state,
                                                TickType_t now,
                                                TickType_t mission_duration,
                                                TickType_t landing_grace,
                                                flight_status_t flight_status);

#endif
