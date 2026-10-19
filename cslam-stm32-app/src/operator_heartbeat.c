#include "operator_heartbeat.h"

#include <stddef.h>

void operator_heartbeat_watch_init(operator_heartbeat_watch_t *watch,
                                   uint32_t initial_value, TickType_t now) {
    if (watch == NULL)
        return;

    watch->last_value = initial_value;
    watch->last_change_tick = now;
    watch->initialized = 1;
    watch->has_update = 0;
}

bool operator_heartbeat_watch_is_fresh(operator_heartbeat_watch_t *watch,
                                       uint32_t value, TickType_t now,
                                       TickType_t timeout) {
    if (watch == NULL || !watch->initialized)
        return false;

    if (value != watch->last_value) {
        watch->last_value = value;
        watch->last_change_tick = now;
        watch->has_update = 1;
    }

    return watch->has_update &&
           (TickType_t)(now - watch->last_change_tick) <= timeout;
}
