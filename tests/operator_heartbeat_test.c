#include "operator_heartbeat.h"

#include <assert.h>
#include <stdint.h>

volatile uint32_t heartbeat_counter;

static void test_requires_a_changed_value_then_expires_at_deadline(void) {
    operator_heartbeat_watch_t watch = {0};
    operator_heartbeat_watch_init(&watch, 10, 100);
    assert(!operator_heartbeat_watch_is_fresh(&watch, 10, 100, 1500));
    assert(operator_heartbeat_watch_is_fresh(&watch, 11, 101, 1500));
    assert(operator_heartbeat_watch_is_fresh(&watch, 11, 1601, 1500));
    assert(!operator_heartbeat_watch_is_fresh(&watch, 11, 1602, 1500));
    assert(operator_heartbeat_watch_is_fresh(&watch, 12, 1602, 1500));
}

static void test_tick_and_counter_wraparound_are_supported(void) {
    operator_heartbeat_watch_t watch = {0};
    operator_heartbeat_watch_init(&watch, UINT32_MAX, UINT32_MAX - 2u);
    assert(operator_heartbeat_watch_is_fresh(&watch, 0, 0, 2));
    assert(!operator_heartbeat_watch_is_fresh(&watch, 0, 3, 2));
}

int main(void) {
    test_requires_a_changed_value_then_expires_at_deadline();
    test_tick_and_counter_wraparound_are_supported();
    return 0;
}
