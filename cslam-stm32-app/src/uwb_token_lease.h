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
bool uwb_token_lease_expired(const uwb_token_lease_t *lease, uint32_t now_tick,
                             uint32_t timeout_ticks);

/** Validate the exact TOKEN or TOKENOPT payload used by the ring protocol. */
bool uwb_token_payload_valid(const uint8_t *data, uint8_t data_len,
                             bool *requests_optimization);

/** Validate token envelope, ring source, destination, and exact payload. */
bool uwb_token_message_valid(uint8_t message_code, uint8_t token_code,
                             uint8_t source, uint8_t destination,
                             uint8_t node_id, uint8_t node_count,
                             const uint8_t *data, uint8_t data_len,
                             bool *requests_optimization);

#endif
