#ifndef GAP9_GVSOC_SPI_TRACE_TRANSPORT_H
#define GAP9_GVSOC_SPI_TRACE_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "point_cloud.h"

bool gap9_gvsoc_spi_trace_has_packet(void);
uint16_t gap9_gvsoc_spi_trace_packet_count(void);
size_t gap9_gvsoc_spi_trace_response_size(void);
int gap9_gvsoc_spi_trace_validate(int16_t pose_packets_decoded,
                                  int16_t poses_inserted,
                                  int16_t bad_pose_checksums,
                                  int16_t scan_packets_decoded,
                                  int16_t scans_registered,
                                  int16_t optimize_packets,
                                  int32_t accepted_constraints,
                                  int32_t optimizer_edges,
                                  const slam_job_t *job);

#endif
