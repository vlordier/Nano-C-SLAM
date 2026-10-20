# CrazySim first-divergence trace: positive run 01

Date: 2026-10-20. This is diagnostic evidence from the CrazySim reference SITL
firmware. It does not run the Nano-C-SLAM STM32 application.

## Setup

- Firmware checkout: `crazyflie-firmware`, branch
  `codex/sitl-first-divergence-trace`, commit `4a6094ad99e80cf3`.
- Firmware binary: built for `cf2`, 500 Hz main loop, diagnostics and a
  32,768-record trace ring enabled. SHA-256:
  `126e07b3ded883aebcf087077235061f3f66b0c33bff287b4dfd54d36674cd70`.
- Ring record: 120 bytes; capacity 3,932,160 bytes. Reported process BSS was
  4,004,344 bytes. It retained all 28,978 records (57.96 seconds at the
  configured 500 Hz); it did not wrap.
- The Gazebo world is `world.sdf`. It adds a 500 Hz `JointStatePublisher` for
  the four rotor joints. The capture contains Gazebo odometry, IMU, motor
  command topic, world statistics, and those joint states. JSONL files and
  the firmware log are gzip-compressed without changing their contents.
- The host sampler build has SHA-256
  `2a43b73662ca2ba385b0b9597a17b59fc7bfdf22a59c1f08511d996db794b4e3`.
- Bridge commands were takeoff to 2.315 m over 8 s, then land to 2.015 m over
  4 s. Both returned result code 0.

## Measurements

| Stream | Samples | Observation |
| --- | ---: | --- |
| Firmware stabilizer ring | 28,978 | 500 Hz requested; log reports a 490 Hz loop-rate warning |
| Gazebo odometry | 8,680 | 200 Hz publisher; z ranged 2.014–2.566 m and ended at 2.015 m |
| Gazebo IMU | 21,240 | Orientation tilt reached 1.659 degrees; max angular velocity component 0.781 rad/s |
| Motor command topic | 42,479 | Maximum command 2,764.52 rad/s; final command was zero |
| Rotor joint state | 21,240 | Captured all four physical joint velocities; SDF slowdown factor is 120 |
| World statistics | 348 | RTF ranged 0.046–2.134, median 0.908 |

Gazebo odometry first exceeded 1 degree of tilt at sim time 31.465 s, at
z=2.387 m. The IMU crossed 1 degree at 31.466 s. Firmware estimated attitude
reached 1.331 degrees somewhere in its trace. The trace records RTOS tick and
step, but `sensorData.interruptTimestamp` was always zero in this SITL port.
There is no proven conversion from that tick to Gazebo sim time, so the exact
firmware row at the first Gazebo divergence cannot yet be identified.

The motor ring stores four `motorPwm` ratios and reconstructs the outgoing
9-byte CRTP motor packet from those same values. Gazebo command and joint
streams are captured separately. This run did not enable the plugin's packet
decoder trace, and the firmware ring has no shared clock with the host capture;
therefore this is not yet a verified packet-to-plugin-to-joint per-sample
correlation. The joint-state capture verifies that actual rotor motion is
observable, not that each packet was decoded correctly at the first
divergence.

## Result and remaining acceptance

This is a useful positive diagnostic run, not a flight acceptance run. It
shows a commanded climb with nonzero motor commands and rotor joint motion,
but not a stable takeoff/hover/land envelope. It also has no paired
actuator-disconnected negative run, no same-world tracing-off baseline, no
measured trace overhead, and no strict paired checker result. RTF varied
widely, so the run is not suitable for controller tuning. Bead
`ncs-fea.5.8.2.7.1.3.3` remains open.

Next experiment: add a monotonic timestamp and decoded packet trace at the
plugin receive boundary (bounded or changed-value-only), run tracing-on and
tracing-off positive cases plus an actuator-disconnected negative case in
fresh isolated partitions, then correlate the first 1-degree event across
firmware, IMU, odometry, motor command, and joint-state streams. Preserve the
same world, binary, command sequence, capture duration, and sampler settings
for all three cases.
