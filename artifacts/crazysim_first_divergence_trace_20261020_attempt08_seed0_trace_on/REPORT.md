# Trace diagnostic

Date: 2026-10-20. Classification: seed 0; full-rate trace on; firmware crash. This is the upstream Crazyflie SITL reference firmware; it does not execute Nano-C-SLAM firmware or validate mapping.

Seeded pair uses Gazebo world retained from attempt02, Gazebo Harmonic on macOS, Lima Ubuntu firmware and the same TAKEOFF_2 (2.4 m / 8 s), 10-second wait, LAND_2 (2.015 m / 8 s) mission.

Observed: 12034 odometry messages; z range 2.014–2.503 m, final z 2.015 m; max tilt 180.000°. World statistics: 490 samples, mean RTF 0.894, median 0.948. Stabilizer warning: 450 Hz. Crash marker: True; landing-timeout marker: False. Plugin motor trace rows: 29503; trace mode enabled: True. Firmware trace header: NCS_TRACE_HEADER schema=1 records=32768 capacity=32768 sampling_hz=500 record_bytes=120 memory_bytes=3932160.

This is a fixed-seed A/B diagnostic. Trace-off returned to the pad but logged a landing timeout and rate warning; trace-on crashed with a 450-Hz warning. The full-rate observer is materially intrusive or otherwise confounded; this pair establishes neither a causal overhead estimate nor an accepted flight.
