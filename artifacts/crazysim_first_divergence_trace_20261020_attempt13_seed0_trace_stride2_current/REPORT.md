# Current same-seed trace on run: crazysim_first_divergence_trace_20261020_attempt13_seed0_trace_stride2_current

Date: 2026-10-20. Both sequential runs used Gazebo seed 0, the same SDF (SHA-256 `e2172e5ff88e174f5f63781acef079c35ebb2a8a999db087a58e45cac3383ab9`), same 2.4-m takeoff / 8-s hold command, 10-s wait, and 2.015-m landing / 8-s command. Firmware binaries were built from the same fork source commit `62abf6d4`; only `NCS_SITL_TRACE_RING` differs. Plugin receive tracing remained enabled in both.

## Result

- Firmware binary SHA-256: `0cc18fdf26bd89853941bd89baf39d0ab6537132e4c610ac0030b3cc62c1de3e`.
- Takeoff and land ACKs: result 0 for both.
- Gazebo: 12951 odometry samples; z 2.015–2.507 m, final 2.015 m; maximum tilt 108.19 degrees.
- Host sampler: 496 world-stat samples; mean RTF 0.955, median 0.971.
- CrazySim plugin: 29727 motor packets (9143 nonzero), zero trace drops.
- Firmware stabilizer warning: 494 Hz; landing timeout marker: True; crash marker: False.
- Firmware ring: NCS_TRACE_HEADER schema=1 records=17257 skipped=17257 stride=2 capacity=32768 sampling_hz=250 record_bytes=120 memory_bytes=3932160.
- Captured firmware wrapper exit: 0; Gazebo exit: 0.

The diagnostic checker was run against the retained disconnected actuator control (attempt 04). Its result is in `CHECKER_OUTPUT.json`. The pair is not accepted flight: firmware and Gazebo wrapper exits were captured as zero; the checker still rejects estimator/rate diagnostics.

The trace-on run avoided a crash but still reached a 108-degree tilt, logged a landing timeout, and warned at 494 Hz. The `Landing timeout, disarming` marker is the expected post-landing reset transition after three seconds, per the pinned supervisor state machine; it is not itself a landing failure. The unchanged flight gate still fails on the 494-Hz stabilizer warning.
