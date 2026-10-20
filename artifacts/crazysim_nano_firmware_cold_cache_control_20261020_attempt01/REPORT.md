# Cold-cache cflib run and disconnected-actuator control, 2026-10-20

This run exercises the handshake-retry runner from commit `dbb6d33` on a fresh
native Gazebo server and fresh Nano `cf2` process. Before startup, the existing
host `cache/` directory was moved aside, leaving cflib with an empty TOC cache.
The current runner completed parameter download, set the initial pose/readiness,
and streamed operator heartbeats. It produced 365 Nano pose packets. The retry
implementation allows at most two 10-second initial-handshake attempts with a
250 ms pause; this run proves that a cold-cache invocation completed, but the
runner does not yet log which attempt succeeded.

The firmware PTY capture records UWB and ToF initialization, fresh ToF data,
task readiness, and `Take-off`. The interactive session later emitted
`Mission ended after landing`; that final console message was not persisted in
the PTY file. The firmware also logged `UWB token handoff exhausted` because
the broker had only node 0 while this firmware build expects two nodes.

The Gazebo plugin started with `NCS_CRAZYSIM_DISABLE_ACTUATORS=1`. Its source
trace recorded 2,237 nonzero firmware motor packets (maximum requested speed
2,774.98 rad/s). Across the active, post-stop, and after-landing captures,
Gazebo reported 45,932 motor messages, all zero; 45,932 rotor joint states
remained within 2.0e-9 rad/s of zero. Odometry stayed at z=0.0149991 m. This is
a firmware-command negative control, not a physical flight.

Inputs and raw data are preserved here: compressed Gazebo streams in
`active/`, `post-stop/`, and `after-landing/`; the plugin motor source trace;
firmware PTY output; Nano pose packets; world SDF; `cf2`; and the exact plugin
binary. `evidence.json` contains machine-computed counts and envelopes.
`SHA256SUMS` verifies the archive.

The separate positive flight remains in
[`../crazysim_nano_firmware_flight_20261020_attempt03/REPORT.md`](../crazysim_nano_firmware_flight_20261020_attempt03/REPORT.md).
This cold-cache capture does not validate UWB token circulation with a peer,
GAP9 execution, map quality, or multi-drone operation. Stabilizer/estimator
rate warnings remain, and repeated cold-start reliability still needs a
multi-run acceptance check.
