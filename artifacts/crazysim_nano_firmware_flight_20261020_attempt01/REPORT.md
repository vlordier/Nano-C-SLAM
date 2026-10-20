# Nano firmware flight in Gazebo — attempt 01

This is a positive, single-drone behavioral capture of the Nano-C-SLAM STM32
application running in Crazyflie SITL, connected to native Gazebo Harmonic by
the CrazySim UDP plugin. It demonstrates firmware-driven motor commands,
takeoff, sustained altitude control, horizontal mission motion, and a later
return to the ground. It does not pass the full paired-flight acceptance gate.

## Setup and command

- Gazebo world: `world.sdf` (two models configured; firmware ran only for
  `cf_0`). Gazebo was native macOS Gazebo 8.14.0, headless, partition
  `ncs-nano-uwb-e2600d5`, `GZ_IP=127.0.0.1`.
- Gazebo CrazySim plugin: `gz_crazysim_plugin.dylib`, SHA-256
  `607357023cd6ec7b11c2229106e92ae659e49ece6afe8b32eb55be09695dfc6d`.
- Firmware: Lima Ubuntu 24.04 ARM64 process
  `/home/vincent.guest/ncs-cf-build-nano-app/cf2 21950 192.168.5.2`, built
  with `CRAZYSIM_NANO_APP=ON`, SHA-256
  `18016ddd59923c6f4a392989d22e755babcd190df8298a64b281b0affc4b1849`.
- Operator: `PYTHONUNBUFFERED=1 timeout --signal=INT --kill-after=8 30s uv run
  --locked python cslam-stm32-app/run_demo.py 1 --uri
  udp://127.0.0.1:21850 --log-root
  /tmp/ncs-nano-run-demo-attempt05-20261020/logs`.
- Capture: `GZ_PARTITION=ncs-nano-uwb-e2600d5 GZ_IP=127.0.0.1
  /tmp/ncs-crazysim-flight-sampler 48
  /tmp/ncs-nano-run-demo-attempt05-20261020`.

The runner downloaded the parameter TOC and logged `Parameters downloaded`
before writing the configured initial pose and ready flag. The timeout stopped
the operator heartbeat; the firmware's heartbeat failsafe requested landing.
The flight capture started and ended while Gazebo remained running. The
sampler's final sample was during descent; a later direct Gazebo query showed
`cf_0` at z=0.01499996 m and `/cf_0/command/motor_speed` at `[0, 0, 0, 0]`.

## Measurements

- 6,730 odometry samples, 15,840 IMU messages, 31,675 motor-command messages,
  and 360 world-statistics samples.
- Altitude: 0.0150 m at start, 0.8956 m maximum, 0.4807 m at the sampler's
  final sample; the later direct query confirmed return to 0.0150 m.
- Stable hover in the registered 0.70 ± 0.12 m altitude window, within 15°
  tilt and 0.5 rad/s angular-rate limits: 13.835 s continuous.
- Motor command maximum: 2,774.984 rad/s; 83,320 of 126,700 scalar motor
  values were nonzero.
- Horizontal position moved from (2.946, -0.132) m to a maximum displacement
  of about 7.91 m, consistent with the Nano mission issuing a trajectory.
- Maximum odometry gap 5 ms; maximum IMU gap 2 ms. Aggregate Gazebo RTF was
  0.719, below the 0.95 acceptance minimum.

## What remains unproven

The flight process proves a meaningful Nano firmware → CRTP motor command →
Gazebo response loop, but this attempt is not a strict acceptance run. The
plugin's one-shot firmware-source trace was disabled, physical rotor joint
velocities were not captured, and no actuator-disconnected negative control
was run. Gazebo RTF was below the registered envelope. The capture's final
sample occurred before the later ground-state query, and the run did not retain
firmware console/startup markers or child exit receipts. Reconnection after
closing cflib remains intermittent. The second drone was not run, and neither
GAP9 mapping nor map-to-world geometry was validated.

## Capture integrity

The four compressed JSONL streams and exact world file are retained beside
this report. `SHA256SUMS` covers each retained file and the sampler binary.
This report deliberately records the RTF and evidence gaps rather than treating
the positive motion as a passed paired-flight gate. The retained firmware ELF,
plugin library, and sampler binary hashes bind the reported measurements to the
tested executable artifacts.
