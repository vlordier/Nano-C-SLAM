#ifndef UWB_TOKEN_HANDOFF_H
#define UWB_TOKEN_HANDOFF_H

#include <stdbool.h>
#include <stdint.h>

typedef bool (*uwb_token_attempt_fn)(uint8_t target_id,
                                     uint8_t send_optimization, void *context);

bool uwb_token_try_handoff(uint8_t current_id, uint8_t node_count,
                           uint8_t send_token_with_opt,
                           uwb_token_attempt_fn attempt, void *context,
                           uint8_t *passed_to);

#endif
