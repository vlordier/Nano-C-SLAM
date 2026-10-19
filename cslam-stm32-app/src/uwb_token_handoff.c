#include "uwb_token_handoff.h"

#include <stddef.h>

bool uwb_token_try_handoff(uint8_t current_id, uint8_t node_count,
                           uint8_t send_token_with_opt,
                           uwb_token_attempt_fn attempt, void *context,
                           uint8_t *passed_to) {
    if (node_count < 2 || current_id >= node_count || attempt == NULL)
        return false;

    for (uint16_t offset = 1; offset < node_count; offset++) {
        const uint8_t target_id = (uint8_t)((current_id + offset) % node_count);
        const uint8_t optimize =
            (uint8_t)(send_token_with_opt && target_id > current_id);
        if (attempt(target_id, optimize, context)) {
            if (passed_to != NULL)
                *passed_to = target_id;
            return true;
        }
    }

    return false;
}
