# Trace diagnostic

Date: 2026-10-20. Classification: seed 0; trace off; baseline diagnostic, strict gate not accepted. This is the upstream Crazyflie SITL reference firmware; it does not execute Nano-C-SLAM firmware or validate mapping.

Seeded pair uses Gazebo world retained from attempt02, Gazebo Harmonic on macOS, Lima Ubuntu firmware and the same TAKEOFF_2 (2.4 m / 8 s), 10-second wait, LAND_2 (2.015 m / 8 s) mission.

Observed: 12053 odometry messages; z range 2.015–2.516 m, final z 2.015 m; max tilt 1.120°. World statistics: 482 samples, mean RTF 0.905, median 0.962. Stabilizer warning: 502 Hz. Crash marker: False; landing-timeout marker: True. Plugin motor trace rows: 0; trace mode enabled: False. Firmware trace header: disabled.

This is a fixed-seed A/B diagnostic. Trace-off returned to the pad but logged a landing timeout and rate warning; trace-on crashed with a 450-Hz warning. The full-rate observer is materially intrusive or otherwise confounded; this pair establishes neither a causal overhead estimate nor an accepted flight.
