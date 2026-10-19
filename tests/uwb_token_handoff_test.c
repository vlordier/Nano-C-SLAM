#include "uwb_token_handoff.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t successful_target;
    uint8_t targets[8];
    uint8_t optimize_flags[8];
    uint8_t attempts;
} attempt_log_t;

static bool try_target(uint8_t target_id, uint8_t send_optimization, void *context) {
    attempt_log_t *log = context;
    assert(log->attempts < sizeof(log->targets));
    log->targets[log->attempts] = target_id;
    log->optimize_flags[log->attempts] = send_optimization;
    log->attempts++;
    return target_id == log->successful_target;
}

static void test_rotation_skips_self_and_stops_after_success(void) {
    attempt_log_t log = {.successful_target = 2};
    uint8_t passed_to = UINT8_MAX;
    assert(uwb_token_try_handoff(0, 3, 1, try_target, &log, &passed_to));
    assert(log.attempts == 2);
    assert(log.targets[0] == 1 && log.targets[1] == 2);
    assert(log.optimize_flags[0] == 1 && log.optimize_flags[1] == 1);
    assert(passed_to == 2);
}

static void test_exhaustion_is_bounded_and_preserves_peer_rotation(void) {
    attempt_log_t log = {.successful_target = UINT8_MAX};
    uint8_t passed_to = UINT8_MAX;
    assert(!uwb_token_try_handoff(2, 3, 1, try_target, &log, &passed_to));
    assert(log.attempts == 2);
    assert(log.targets[0] == 0 && log.targets[1] == 1);
    assert(log.optimize_flags[0] == 0 && log.optimize_flags[1] == 0);
    assert(passed_to == UINT8_MAX);
}

static void test_degenerate_topologies_do_not_call_the_sender(void) {
    attempt_log_t log = {.successful_target = 0};
    assert(!uwb_token_try_handoff(0, 1, 0, try_target, &log, NULL));
    assert(!uwb_token_try_handoff(3, 3, 0, try_target, &log, NULL));
    assert(!uwb_token_try_handoff(0, 3, 0, NULL, &log, NULL));
    assert(log.attempts == 0);
}

int main(void) {
    test_rotation_skips_self_and_stops_after_success();
    test_exhaustion_is_bounded_and_preserves_peer_rotation();
    test_degenerate_topologies_do_not_call_the_sender();
    return 0;
}
