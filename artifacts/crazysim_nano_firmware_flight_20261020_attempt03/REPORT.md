# Nano STM32 firmware flight in CrazySim/Gazebo — attempt 03

This archive records a live single-drone flight with the Nano STM32 SITL
application in a Lima Ubuntu 24.04 guest and native Gazebo Sim 8.14.0 on
macOS. Gazebo supplied IMU and four 8×8 ToF scans; the firmware produced the
motor packets consumed by the CrazySim model. `world.sdf`, firmware, plugin,
sampler, captures, and the per-run motor packet trace are retained here.

## Positive flight evidence

The cflib operator connected to `udp://127.0.0.1:21850`, downloaded firmware
parameters, set the initial pose, and enabled the mission. The firmware log
shows successful UWB and ToF provider initialization, fresh ToF data, app task
startup, and `Take-off`. The Gazebo plugin's per-run trace contains 3,961
validated firmware motor packets in the flight interval; 3,692 have nonzero
motor ratios. The Gazebo motor topic contains 88,837 messages in the flight
sample, with a maximum motor speed of 2,126.71 rad/s.

The 92.07-second flight telemetry sample contains 18,157 odometry, 44,419 IMU,
88,837 motor, and 719 world-stat messages. Altitude rose from 0.015 m to
0.894 m and stayed near the 0.70 m target while the vehicle moved 31.8 m in
the world X direction. It stayed continuously within 0.58–0.82 m for 65.99 s,
with tilt under 15 degrees and angular speed under 0.5 rad/s during that
interval. Aggregate Gazebo real-time factor over the capture was 0.986. A
separate 19.9-second post-mission sample contains 4,359 odometry and
19,903 motor messages; all odometry remained at 0.015 m and all four motor
speeds were zero.

The operator stopped its heartbeat after the flight sample. The operator
reported `Mission ended after landing`, and the later Gazebo sample confirms
the landed state and zero motor output. The gap between the two sampler
intervals was 30.97 seconds, so these files do not establish a continuous
takeoff-to-landing telemetry capture. The operator output was not retained as
a raw process log; `operator-observed-events.txt` records the console events
observed during the run.

## Limits and remaining acceptance work

This is positive evidence that the Nano STM32 application can take off, fly
under its own motor outputs, and land in a synthetic CrazySim world. It is not
a passing paired acceptance run. The firmware emitted estimator and
stabilizer-rate warnings during operation, the continuous flight sample ended
before landing, and the new disconnected-actuator attempt did not arm: cflib
timed out while opening `udp://127.0.0.1:21850`. The older attempt 02 has a
disconnected-actuator ground capture, but its console and per-run plugin trace
are incomplete, so it does not close the paired gate either.

The isolated flight does not validate a second drone, GAP9 firmware execution,
SLAM map quality, or map-to-Gazebo geometry. The reported 31.8 m X displacement
is evidence of movement, not evidence that the synthetic range returns or
mapping geometry are physically calibrated. Do not describe this as flight
hardware validation.

The ToF bridge used host UDP port 21951 while the firmware receiver stayed on
19951. Lima reserves host UDP 19951 for forwarding to the guest. The UWB broker
used `--pose-max-skew-ms 250`; its default remains 50 ms.

## Checks

Python gates passed after the source fixes: Ruff check, Ruff format check, Astral
ty, and all 567 unittest tests. The focused ToF/UWB tests passed 20/20. The
strict paired flight checker was not passed with this archive.
