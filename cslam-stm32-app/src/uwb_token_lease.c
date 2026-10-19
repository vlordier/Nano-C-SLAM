#include "uwb_token_lease.h"

#include <stddef.h>

void uwb_token_lease_arm(uwb_token_lease_t *lease, uint32_t now_tick) {
    if (lease == NULL)
        return;
    lease->since_tick = now_tick;
    lease->armed = true;
}

void uwb_token_lease_disarm(uwb_token_lease_t *lease) {
    if (lease == NULL)
        return;
    lease->armed = false;
}

bool uwb_token_lease_expired(const uwb_token_lease_t *lease, uint32_t now_tick,
                             uint32_t timeout_ticks) {
    if (lease == NULL || !lease->armed)
        return false;
    return (uint32_t)(now_tick - lease->since_tick) >= timeout_ticks;
}
