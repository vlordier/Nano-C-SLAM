# Owned Nano translation units selected for source-level CrazySim SITL.
# Physical ToF/I2C implementations are replaced by tof_matrix_sitl.c.
set(NCS_NANO_APP_SOURCES
  src/bounded_batch.c
  src/exploration/exploration.c
  src/exploration/limit_actuation.c
  src/exploration/line_detector.c
  src/flight_commander.c
  src/mission_timeout.c
  src/operator_heartbeat.c
  src/radio_comm.c
  src/scan.c
  src/scan_manager.c
  src/spi_comm/spi_comm.c
  src/spi_comm/spi_wait.c
  src/tof_daq.c
  src/tof_freshness.c
  src/uwb_collision_det.c
  src/uwb_ranging.c
  src/uwb_token_handoff.c
  src/uwb_token_lease.c
)
