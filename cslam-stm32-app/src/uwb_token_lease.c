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

bool uwb_token_lease_receive(uwb_token_lease_t *lease, uint32_t now_tick,
                             uint8_t message_code, uint8_t token_code,
                             uint8_t source, uint8_t destination,
                             uint8_t node_id, uint8_t node_count,
                             const uint8_t *data, uint8_t data_len,
                             bool *requests_optimization) {
    if (requests_optimization != NULL)
        *requests_optimization = false;
    if (lease == NULL ||
        !uwb_token_message_valid(message_code, token_code, source, destination,
                                 node_id, node_count, data, data_len,
                                 requests_optimization)) {
        return false;
    }
    uwb_token_lease_arm(lease, now_tick);
    return true;
}

void uwb_token_lease_passed(uwb_token_lease_t *lease, uint32_t now_tick,
                            uint8_t *agent_has_token) {
    if (agent_has_token != NULL)
        *agent_has_token = 0;
    uwb_token_lease_arm(lease, now_tick);
}

void uwb_token_lease_take(uwb_token_lease_t *lease, uint8_t *agent_has_token) {
    uwb_token_lease_disarm(lease);
    if (agent_has_token != NULL)
        *agent_has_token = 1;
}

void uwb_token_request_land(uwb_token_lease_t *lease, uint8_t *agent_has_token,
                            volatile uint8_t *land) {
    uwb_token_lease_disarm(lease);
    if (agent_has_token != NULL)
        *agent_has_token = 0;
    if (land != NULL)
        *land = 1;
}

bool uwb_token_lease_request_land_if_expired(uwb_token_lease_t *lease,
                                             uint32_t now_tick,
                                             uint32_t timeout_ticks,
                                             uint8_t *agent_has_token,
                                             volatile uint8_t *land) {
    if (!uwb_token_lease_expired(lease, now_tick, timeout_ticks))
        return false;
    uwb_token_request_land(lease, agent_has_token, land);
    return true;
}
