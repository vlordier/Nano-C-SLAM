#ifndef UWB_TOKEN_LEASE_H
#define UWB_TOKEN_LEASE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t since_tick;
    bool armed;
} uwb_token_lease_t;

typedef void (*uwb_token_queue_clear_fn)(void *context);

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

/** Refresh the waiting lease only for a valid token addressed to this node. */
bool uwb_token_lease_receive(uwb_token_lease_t *lease, uint32_t now_tick,
                             uint8_t message_code, uint8_t token_code,
                             uint8_t source, uint8_t destination,
                             uint8_t node_id, uint8_t node_count,
                             const uint8_t *data, uint8_t data_len,
                             bool *requests_optimization);

/** Clear local ownership and arm the receive deadline after a passed token. */
void uwb_token_lease_passed(uwb_token_lease_t *lease, uint32_t now_tick,
                            uint8_t *agent_has_token);

/** Clear stale inbound messages and arm listening only after a successful pass.
 */
bool uwb_token_lease_finish_handoff(bool passed, uwb_token_lease_t *lease,
                                    uint32_t now_tick, uint8_t *agent_has_token,
                                    volatile uint8_t *land,
                                    uwb_token_queue_clear_fn clear_queue,
                                    void *context);

/** Disarm the lease and mark local token ownership after a valid handoff. */
void uwb_token_lease_take(uwb_token_lease_t *lease, uint8_t *agent_has_token);

/** Fail closed: disarm, clear ownership, and request landing. */
void uwb_token_request_land(uwb_token_lease_t *lease, uint8_t *agent_has_token,
                            volatile uint8_t *land);

/** Request landing and clear ownership once the waiting lease expires. */
bool uwb_token_lease_request_land_if_expired(uwb_token_lease_t *lease,
                                             uint32_t now_tick,
                                             uint32_t timeout_ticks,
                                             uint8_t *agent_has_token,
                                             volatile uint8_t *land);

#endif
