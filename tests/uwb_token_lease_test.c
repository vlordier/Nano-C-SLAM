#include "uwb_token_handoff.h"
#include "uwb_token_lease.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

static void test_lease_expires_at_the_configured_boundary(void) {
    uwb_token_lease_t lease = {0};
    uwb_token_lease_arm(&lease, 100);

    assert(!uwb_token_lease_expired(&lease, 109, 10));
    assert(uwb_token_lease_expired(&lease, 110, 10));
}

static void test_lease_deadline_is_wrap_safe(void) {
    uwb_token_lease_t lease = {0};
    uwb_token_lease_arm(&lease, UINT32_MAX - 5U);

    assert(!uwb_token_lease_expired(&lease, 3, 10));
    assert(uwb_token_lease_expired(&lease, 4, 10));
}

static void test_valid_token_refreshes_deadline(void) {
    uwb_token_lease_t lease = {0};
    uwb_token_lease_arm(&lease, 100);
    uwb_token_lease_arm(&lease, 108);

    assert(!uwb_token_lease_expired(&lease, 117, 10));
    assert(uwb_token_lease_expired(&lease, 118, 10));
}

static void test_disarmed_lease_cannot_expire_or_acquire_ownership(void) {
    uwb_token_lease_t lease = {0};
    uwb_token_lease_arm(&lease, 100);
    uwb_token_lease_disarm(&lease);

    assert(!uwb_token_lease_expired(&lease, 1000, 10));
}

static void test_zero_timeout_expires_an_armed_lease_immediately(void) {
    uwb_token_lease_t lease = {0};
    uwb_token_lease_arm(&lease, 100);

    assert(uwb_token_lease_expired(&lease, 100, 0));
}

static void test_null_lease_is_ignored(void) {
    uwb_token_lease_arm(NULL, 0);
    uwb_token_lease_disarm(NULL);
    assert(!uwb_token_lease_expired(NULL, 0, 0));
}

static void test_token_payload_accepts_only_exact_protocol_forms(void) {
    static const uint8_t token[] = {'T', 'O', 'K', 'E', 'N'};
    static const uint8_t token_opt[] = {'T', 'O', 'K', 'E', 'N', 'O', 'P', 'T'};
    bool requests_optimization = true;

    assert(uwb_token_payload_valid(token, sizeof(token), &requests_optimization));
    assert(!requests_optimization);
    assert(
        uwb_token_payload_valid(token_opt, sizeof(token_opt), &requests_optimization));
    assert(requests_optimization);
    assert(!uwb_token_payload_valid(token_opt, 7, &requests_optimization));
    assert(!requests_optimization);
}

static void test_malformed_or_null_token_payload_is_rejected(void) {
    static const uint8_t wrong_suffix[] = {'T', 'O', 'K', 'E', 'N', 'B', 'A', 'D'};
    static const uint8_t wrong_prefix[] = {'X', 'O', 'K', 'E', 'N'};

    assert(!uwb_token_payload_valid(wrong_suffix, sizeof(wrong_suffix), NULL));
    assert(!uwb_token_payload_valid(wrong_prefix, sizeof(wrong_prefix), NULL));
    assert(!uwb_token_payload_valid(NULL, 5, NULL));
}

static void test_token_message_requires_valid_ring_envelope(void) {
    static const uint8_t token[] = {'T', 'O', 'K', 'E', 'N'};
    bool requests_optimization = true;

    assert(uwb_token_message_valid(7, 7, 0, 1, 1, 2, token, sizeof(token),
                                   &requests_optimization));
    assert(!requests_optimization);
    assert(!uwb_token_message_valid(8, 7, 0, 1, 1, 2, token, sizeof(token), NULL));
    assert(!uwb_token_message_valid(7, 7, 0, 0, 1, 2, token, sizeof(token), NULL));
    assert(!uwb_token_message_valid(7, 7, 2, 1, 1, 2, token, sizeof(token), NULL));
    assert(!uwb_token_message_valid(7, 7, 1, 1, 1, 2, token, sizeof(token), NULL));
    assert(!uwb_token_message_valid(7, 7, 0, 1, 2, 2, token, sizeof(token), NULL));
}

static void test_delayed_valid_token_refreshes_before_lease_expiry(void) {
    static const uint8_t token[] = {'T', 'O', 'K', 'E', 'N'};
    uwb_token_lease_t lease = {0};
    uint8_t has_token = 0;
    bool requests_optimization = true;
    uwb_token_lease_arm(&lease, 100);

    assert(!uwb_token_lease_expired(&lease, 109, 10));
    assert(uwb_token_lease_receive(&lease, 109, 7, 7, 0, 1, 1, 2, token, sizeof(token),
                                   &requests_optimization));
    assert(lease.since_tick == 109 && !requests_optimization);
    assert(!uwb_token_lease_expired(&lease, 118, 10));
    uwb_token_lease_take(&lease, &has_token);
    assert(!lease.armed && has_token);
}

static void test_non_token_activity_does_not_refresh_waiting_lease(void) {
    static const uint8_t pose_payload[] = {'T', 'O', 'K', 'E', 'N'};
    uwb_token_lease_t lease = {0};
    uwb_token_lease_arm(&lease, 100);

    assert(!uwb_token_lease_receive(&lease, 109, 8, 7, 0, 1, 1, 2, pose_payload,
                                    sizeof(pose_payload), NULL));
    assert(lease.since_tick == 100);
    assert(uwb_token_lease_request_land_if_expired(&lease, 110, 10, NULL, NULL));
}

static void test_expiry_requests_land_without_claiming_token(void) {
    uwb_token_lease_t lease = {0};
    uint8_t has_token = 0;
    volatile uint8_t land = 0;
    uwb_token_lease_arm(&lease, 100);

    assert(uwb_token_lease_request_land_if_expired(&lease, 109, 10, &has_token,
                                                   &land) == false);
    assert(!land && !has_token);
    assert(uwb_token_lease_request_land_if_expired(&lease, 110, 10, &has_token, &land));
    assert(land && !has_token && !lease.armed);

    has_token = 1;
    land = 0;
    uwb_token_lease_arm(&lease, 200);
    assert(uwb_token_lease_request_land_if_expired(&lease, 210, 10, &has_token, &land));
    assert(land && !has_token && !lease.armed);
}

typedef struct failed_handoff {
    uint8_t attempts;
} failed_handoff_t;

static bool reject_handoff(uint8_t target_id, uint8_t optimize, void *context) {
    (void)target_id;
    (void)optimize;
    failed_handoff_t *state = context;
    state->attempts++;
    return false;
}

static void test_failed_outgoing_handoff_lands_and_clears_ownership(void) {
    uwb_token_lease_t lease = {0};
    uint8_t has_token = 1;
    volatile uint8_t land = 0;
    failed_handoff_t state = {0};

    assert(!uwb_token_try_handoff(0, 2, 0, reject_handoff, &state, NULL));
    assert(state.attempts == 1);
    uwb_token_request_land(&lease, &has_token, &land);
    assert(land && !has_token && !lease.armed);
}

static void test_successful_pass_starts_waiting_lease(void) {
    uwb_token_lease_t lease = {0};
    uint8_t has_token = 1;

    uwb_token_lease_passed(&lease, 42, &has_token);
    assert(lease.armed && lease.since_tick == 42 && !has_token);
}

int main(void) {
    test_lease_expires_at_the_configured_boundary();
    test_lease_deadline_is_wrap_safe();
    test_valid_token_refreshes_deadline();
    test_disarmed_lease_cannot_expire_or_acquire_ownership();
    test_zero_timeout_expires_an_armed_lease_immediately();
    test_null_lease_is_ignored();
    test_token_payload_accepts_only_exact_protocol_forms();
    test_malformed_or_null_token_payload_is_rejected();
    test_token_message_requires_valid_ring_envelope();
    test_delayed_valid_token_refreshes_before_lease_expiry();
    test_non_token_activity_does_not_refresh_waiting_lease();
    test_expiry_requests_land_without_claiming_token();
    test_failed_outgoing_handoff_lands_and_clears_ownership();
    test_successful_pass_starts_waiting_lease();
    return 0;
}
