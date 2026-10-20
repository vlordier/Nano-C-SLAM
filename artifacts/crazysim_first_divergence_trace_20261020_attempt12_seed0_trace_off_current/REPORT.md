# Current same-seed trace off run: crazysim_first_divergence_trace_20261020_attempt12_seed0_trace_off_current

Date: 2026-10-20. Both sequential runs used Gazebo seed 0, the same SDF (SHA-256 `e2172e5ff88e174f5f63781acef079c35ebb2a8a999db087a58e45cac3383ab9`), same 2.4-m takeoff / 8-s hold command, 10-s wait, and 2.015-m landing / 8-s command. Firmware binaries were built from the same fork source commit `62abf6d4`; only `NCS_SITL_TRACE_RING` differs. Plugin receive tracing remained enabled in both.

## Result

- Firmware binary SHA-256: `5686120e1300b9958aa11510a7c4b7a28fbd53cb0c9627cef555dfaf7e99338a`.
- Takeoff and land ACKs: result 0 for both.
- Gazebo: 10883 odometry samples; z 2.014–2.428 m, final 2.015 m; maximum tilt 180.00 degrees.
- Host sampler: 479 world-stat samples; mean RTF 0.812, median 0.910.
- CrazySim plugin: 26045 motor packets (5699 nonzero), zero trace drops.
- Firmware stabilizer warning: 456 Hz; landing timeout marker: False; crash marker: True.
- Firmware ring: disabled.
- Captured firmware wrapper exit: MISSING; Gazebo exit: 0.

The diagnostic checker was run against the retained disconnected actuator control (attempt 04). Its result is in `CHECKER_OUTPUT.json`. The pair is not accepted flight: firmware wrapper status was not retained because the zsh wrapper assigned its reserved `status` variable; its process exit acceptance is unverified.

The trace-off run crashed and fell below the RTF gate. Because host load/RTF differed markedly from the adjacent trace-on run, this pair does not quantify recorder overhead.
