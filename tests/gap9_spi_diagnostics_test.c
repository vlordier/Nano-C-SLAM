#include "spi_diagnostics.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

int main(void) {
    spi_diagnostics_t diagnostics = {0};

    spi_diagnostics_record_frame(&diagnostics, -1);
    spi_diagnostics_record_frame(&diagnostics, SPI_DIAGNOSTIC_COMMAND_COUNT);
    spi_diagnostics_record_frame(&diagnostics, 0);
    spi_diagnostics_record_decode(&diagnostics, 0, 0);
    spi_diagnostics_record_decode(&diagnostics, 0, 1);
    spi_diagnostics_record_decode(&diagnostics, -1, 1);
    spi_diagnostics_record_pose_id(&diagnostics, 7);
    spi_diagnostics_record_pose_id(&diagnostics, 8);
    spi_diagnostics_record_pose_id(&diagnostics, 11);
    spi_diagnostics_record_pose_id(&diagnostics, 11);
    spi_diagnostics_record_pose_id(&diagnostics, 10);
    spi_diagnostics_record_optimize_response(&diagnostics);

    assert(diagnostics.received_frames == 3);
    assert(diagnostics.malformed_frames == 1);
    assert(diagnostics.unknown_commands == 1);
    assert(diagnostics.command_frames[0] == 1);
    assert(diagnostics.decode_success[0] == 1);
    assert(diagnostics.decode_failure[0] == 1);
    assert(diagnostics.missing_pose_ids == 2);
    assert(diagnostics.nonadvancing_pose_ids == 2);
    assert(diagnostics.completed_optimize_responses == 1);
    assert(diagnostics.last_pose_id == 10);

    memset(&diagnostics, 0, sizeof(diagnostics));
    spi_diagnostics_record_frame(&diagnostics, 1);
    spi_diagnostics_record_decode(&diagnostics, 1, 1);
    spi_diagnostics_record_pose_id(&diagnostics, INT16_MAX);
    spi_diagnostics_record_pose_id(&diagnostics, INT16_MIN);
    assert(diagnostics.missing_pose_ids == 0);
    assert(diagnostics.nonadvancing_pose_ids == 0);

    diagnostics.received_frames = UINT32_MAX;
    diagnostics.command_frames[1] = UINT32_MAX;
    spi_diagnostics_record_frame(&diagnostics, 1);
    assert(diagnostics.received_frames == 0);
    assert(diagnostics.command_frames[1] == 0);

    spi_diagnostics_record_frame(NULL, 1);
    spi_diagnostics_record_decode(NULL, 1, 1);
    spi_diagnostics_record_pose_id(NULL, 1);
    spi_diagnostics_record_optimize_response(NULL);
    return 0;
}
