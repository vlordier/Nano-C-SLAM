# Nano STM32 firmware flight in CrazySim/Gazebo — attempt 02

This archive contains a continuous, firmware-driven one-drone flight capture
and a separate disconnected-actuator control. The Nano STM32 SITL application
ran in the Linux Lima guest; native Gazebo Sim 8.14.0 ran on macOS with the
CrazySim UDP plugin. Both runs used the same firmware image and SDF world. The
negative run additionally set `NCS_CRAZYSIM_DISABLE_ACTUATORS=1` in Gazebo.

## Positive flight

The odometry, IMU, motor and world-stat samplers started before the mission
operator and continued through the heartbeat-triggered landing. The cflib
operator connected to `udp://127.0.0.1:21850`, downloaded the parameter TOC,
set the initial pose and ready flag, and sent its heartbeat while active.

The capture contains 31,841 odometry samples, 78,639 IMU samples, 157,278
four-motor messages, and 1,311 world-stat samples. Altitude started at 0.015 m,
reached 0.886 m, and ended at 0.015 m. The vehicle remained within 0.58–0.82 m
for 127.135 s while tilt stayed below 15 degrees and angular rate below
0.5 rad/s. It traveled horizontally during the Nano mission. Of 632,420 motor
components, 540,376 were nonzero; the maximum commanded speed was 2,278.20
rad/s. The final motor message was zero. Aggregate Gazebo real-time factor was
0.9881, and the largest odometry gap was 5 ms.

The CrazySim plugin's one-shot `TRACE_MOTOR_NONZERO ... source=firmware` line
was observed earlier in the same long-lived Gazebo server and is retained in
`positive/plugin-attribution-observation.txt`. It identifies the plugin's
incoming motor command as firmware-originated. Because the plugin emits this
line only once per loaded model, it is not a per-message trace for this
sampler interval.

## Disconnected-actuator control

With `NCS_CRAZYSIM_DISABLE_ACTUATORS=1`, the mission operator still downloaded
parameters and wrote the arm/ready parameters. The plugin logged
`actuator_bridge=disconnected` and observed a nonzero firmware motor command.
Gazebo published 42,431 motor messages and 42,431 model joint-state messages;
all four published motor speeds and all rotor joint velocities remained zero.
All 8,877 odometry samples stayed at z=0.014999 m. This confirms the model did
not rise when the actuator bridge was disabled.

## Evidence limits

The positive flight and disconnected control demonstrate the single-drone
Nano firmware → CrazySim motor-output path and its actuator-disconnected
negative control. The physical rotor joint response was independently captured
in the earlier `attempt01` instrumentation run; the positive capture here
omits the high-rate joint-state subscriber to preserve real-time factor.

This archive does **not** claim that the registered automated paired-flight
checker passed. The SITL process's block-buffered stdout was not retained as a
raw file for these captures, so startup, calibration and mission receipts are
not bound into the checker inputs. The plugin's positive source-attribution
line is also a one-shot observation from the same long-lived server rather
than a record attached to each captured motor message. Multi-drone flight,
GAP9 execution, SLAM mapping, and map-to-Gazebo geometry validation remain
unproven.

The firmware image, plugin library, sampler, world, and compressed captures
are covered by `SHA256SUMS`; `evidence.json` records their primary hashes and
capture provenance.
