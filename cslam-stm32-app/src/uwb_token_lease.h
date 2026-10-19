#ifndef UWB_TOKEN_LEASE_H
#define UWB_TOKEN_LEASE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t since_tick;
    bool armed;
} uwb_token_lease_t;

/** Begin or restart the deadline while this node is waiting for a token. */
void uwb_token_lease_arm(uwb_token_lease_t *lease, uint32_t now_tick);

/** Stop monitoring while this node owns the token or has stopped the ring. */
void uwb_token_lease_disarm(uwb_token_lease_t *lease);

/** Return true once the unsigned tick age reaches timeout_ticks. */
bool uwb_token_lease_expired(const uwb_token_lease_t *lease,
                            uint32_t now_tick, uint32_t timeout_ticks);

#endif
