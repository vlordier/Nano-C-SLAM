# Trace diagnostic

Date: 2026-10-20. Classification: unseeded; full-rate trace on; diagnostic gate failed. This is the upstream Crazyflie SITL reference firmware; it does not execute Nano-C-SLAM firmware or validate mapping.

Seeded pair uses Gazebo world retained from attempt02, Gazebo Harmonic on macOS, Lima Ubuntu firmware and the same TAKEOFF_2 (2.4 m / 8 s), 10-second wait, LAND_2 (2.015 m / 8 s) mission.

Observed: 13000 odometry messages; z range 2.014–2.506 m, final z 2.015 m; max tilt 180.000°. World statistics: 497 samples, mean RTF 0.971, median 0.979. Stabilizer warning: 467 Hz. Crash marker: True; landing-timeout marker: False. Plugin motor trace rows: 28188; trace mode enabled: True. Firmware trace header: NCS_TRACE_HEADER schema=1 records=32768 capacity=32768 sampling_hz=500 record_bytes=120 memory_bytes=3932160.

This run has no fixed seed and its RTF differs substantially from its paired trace-off run. It cannot isolate observer overhead.
