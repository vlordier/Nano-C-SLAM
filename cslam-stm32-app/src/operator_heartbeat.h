#ifndef OPERATOR_HEARTBEAT_H
#define OPERATOR_HEARTBEAT_H

#include "FreeRTOS.h"

#include <stdbool.h>
#include <stdint.h>

#define OPERATOR_HEARTBEAT_TIMEOUT_MS 1500u

typedef struct {
    uint32_t last_value;
    TickType_t last_change_tick;
    uint8_t initialized;
    uint8_t has_update;
} operator_heartbeat_watch_t;

extern volatile uint32_t heartbeat_counter;

void operator_heartbeat_watch_init(operator_heartbeat_watch_t *watch,
                                   uint32_t initial_value, TickType_t now);
bool operator_heartbeat_watch_is_fresh(operator_heartbeat_watch_t *watch,
                                       uint32_t value, TickType_t now,
                                       TickType_t timeout);

#endif
