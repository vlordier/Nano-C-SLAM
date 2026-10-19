#include "uwb_token_lease.h"

#include <stddef.h>
#include <string.h>

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

bool uwb_token_payload_valid(const uint8_t *data, uint8_t data_len,
                             bool *requests_optimization) {
    static const uint8_t token[] = {'T', 'O', 'K', 'E', 'N'};
    static const uint8_t optimize[] = {'O', 'P', 'T'};
    if (requests_optimization != NULL)
        *requests_optimization = false;
    if (data == NULL ||
        (data_len != sizeof(token) &&
         data_len != sizeof(token) + sizeof(optimize)) ||
        memcmp(data, token, sizeof(token)) != 0) {
        return false;
    }
    if (data_len == sizeof(token) + sizeof(optimize)) {
        if (memcmp(data + sizeof(token), optimize, sizeof(optimize)) != 0)
            return false;
        if (requests_optimization != NULL)
            *requests_optimization = true;
    }
    return true;
}

bool uwb_token_message_valid(uint8_t message_code, uint8_t token_code,
                             uint8_t source, uint8_t destination,
                             uint8_t node_id, uint8_t node_count,
                             const uint8_t *data, uint8_t data_len,
                             bool *requests_optimization) {
    if (requests_optimization != NULL)
        *requests_optimization = false;
    return message_code == token_code && node_count > 1 &&
           node_id < node_count && source < node_count && source != node_id &&
           destination == node_id &&
           uwb_token_payload_valid(data, data_len, requests_optimization);
}
