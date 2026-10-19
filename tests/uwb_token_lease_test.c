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

int main(void) {
    test_lease_expires_at_the_configured_boundary();
    test_lease_deadline_is_wrap_safe();
    test_valid_token_refreshes_deadline();
    test_disarmed_lease_cannot_expire_or_acquire_ownership();
    test_zero_timeout_expires_an_armed_lease_immediately();
    test_null_lease_is_ignored();
    return 0;
}
