#ifndef NANO_C_SLAM_SPI_DIAGNOSTICS_H
#define NANO_C_SLAM_SPI_DIAGNOSTICS_H

#include <stdint.h>

#define SPI_DIAGNOSTIC_COMMAND_COUNT 7

typedef struct {
    uint32_t received_frames;
    uint32_t malformed_frames;
    uint32_t unknown_commands;
    uint32_t command_frames[SPI_DIAGNOSTIC_COMMAND_COUNT];
    uint32_t decode_success[SPI_DIAGNOSTIC_COMMAND_COUNT];
    uint32_t decode_failure[SPI_DIAGNOSTIC_COMMAND_COUNT];
    uint32_t missing_pose_ids;
    uint32_t nonadvancing_pose_ids;
    uint32_t completed_optimize_responses;
    int16_t last_pose_id;
    uint8_t has_last_pose_id;
} spi_diagnostics_t;

void spi_diagnostics_record_frame(spi_diagnostics_t *diagnostics, int command);
void spi_diagnostics_record_decode(spi_diagnostics_t *diagnostics, int command,
                                   int succeeded);
void spi_diagnostics_record_pose_id(spi_diagnostics_t *diagnostics,
                                    int16_t pose_id);
void spi_diagnostics_record_optimize_response(spi_diagnostics_t *diagnostics);

#endif
