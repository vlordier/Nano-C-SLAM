# Trace diagnostic

Date: 2026-10-20. Classification: unseeded; trace off; invalid for overhead attribution. This is the upstream Crazyflie SITL reference firmware; it does not execute Nano-C-SLAM firmware or validate mapping.

Seeded pair uses Gazebo world retained from attempt02, Gazebo Harmonic on macOS, Lima Ubuntu firmware and the same TAKEOFF_2 (2.4 m / 8 s), 10-second wait, LAND_2 (2.015 m / 8 s) mission.

Observed: 12786 odometry messages; z range 2.015–2.506 m, final z 2.015 m; max tilt 13.255°. World statistics: 512 samples, mean RTF 0.876, median 0.942. Stabilizer warning: 495 Hz. Crash marker: False; landing-timeout marker: True. Plugin motor trace rows: 0; trace mode enabled: False. Firmware trace header: disabled.

This run has no fixed seed and its RTF differs substantially from its paired trace-off run. It cannot isolate observer overhead.
