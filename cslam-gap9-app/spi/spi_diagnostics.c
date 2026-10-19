#include "spi_diagnostics.h"

#include <stddef.h>

void spi_diagnostics_record_frame(spi_diagnostics_t *diagnostics, int command) {
    if (diagnostics == NULL) return;
    diagnostics->received_frames++;
    if (command < 0) {
        diagnostics->malformed_frames++;
    } else if (command >= SPI_DIAGNOSTIC_COMMAND_COUNT) {
        diagnostics->unknown_commands++;
    } else {
        diagnostics->command_frames[command]++;
    }
}

void spi_diagnostics_record_decode(spi_diagnostics_t *diagnostics, int command,
                                   int succeeded) {
    if (diagnostics == NULL || command < 0 ||
        command >= SPI_DIAGNOSTIC_COMMAND_COUNT)
        return;
    if (succeeded) {
        diagnostics->decode_success[command]++;
    } else {
        diagnostics->decode_failure[command]++;
    }
}

void spi_diagnostics_record_pose_id(spi_diagnostics_t *diagnostics,
                                    int16_t pose_id) {
    if (diagnostics == NULL) return;
    if (diagnostics->has_last_pose_id) {
        int32_t delta = (int32_t)pose_id - diagnostics->last_pose_id;
        if (delta > INT16_MAX) delta -= INT16_MAX + 1L + INT16_MAX + 1L;
        if (delta < INT16_MIN) delta += INT16_MAX + 1L + INT16_MAX + 1L;
        if (delta > 1) {
            diagnostics->missing_pose_ids += (uint32_t)(delta - 1);
        } else if (delta <= 0) {
            diagnostics->nonadvancing_pose_ids++;
        }
    }
    diagnostics->last_pose_id = pose_id;
    diagnostics->has_last_pose_id = 1;
}

void spi_diagnostics_record_optimize_response(spi_diagnostics_t *diagnostics) {
    if (diagnostics != NULL) diagnostics->completed_optimize_responses++;
}
