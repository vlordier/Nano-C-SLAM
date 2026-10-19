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
    return 0;
}
