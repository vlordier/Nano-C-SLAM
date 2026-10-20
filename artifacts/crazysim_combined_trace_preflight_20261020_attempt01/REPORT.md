# Combined provenance/motor trace Gazebo preflight

Date: 2026-10-20. This is a Gazebo/plugin load and transport preflight only. No `cf2` firmware process was connected and no flight command was sent.

- CrazySim source: `codex/sitl-combined-provenance-motor-clean` commit `02429e8` (parent `1eadb3e`).
- Exact plugin loaded by Gazebo: `/tmp/ncs-cf-combined-clean-build/libgz_crazysim_plugin.dylib`, SHA-256 `2e70696d11235be4053cb72b65992b6ef2a01af30fd089c300ba2809d67e55e5`.
- Build options: `CRAZYSIM_IMU_PROVENANCE=ON`, `CRAZYSIM_MOTOR_TRACE=ON`; CTest passed 2/2 (provenance and legacy IMU wire layouts).
- Gazebo Harmonic ran native on macOS, server/sampler used `GZ_IP=127.0.0.1` and partition `ncs-combined-trace-preflight-20261020-01`.
- Typed sampler discovered all five topics and retained 634 odometry, 596 IMU, 1,191 motor-command, 10 world-stat, and 594 joint-state messages in 4 seconds.
- The plugin logged motor tracing enabled and zero captured motor packets at shutdown. IMU send CSV contains only its header. Both empty trace results are expected because no firmware connected.

This proves the combined dylib loads and the Gazebo transport path is live. It does not yet prove IMU metadata reaches firmware, motor packets are emitted, the reference firmware flies, or Nano-C-SLAM firmware/mapping works. Next: run the same plugin with matching provenance-enabled `cf2`, then retain strict positive/disconnected pair evidence.
