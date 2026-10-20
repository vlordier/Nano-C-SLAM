# Nano firmware disconnected-actuator control, 2026-10-20

This is a live negative control in native Gazebo, not a firmware flight. The
Nano STM32 SITL firmware received the operator's ready/heartbeat path and
emitted nonzero motor-ratio packets. Gazebo's actuator bridge was launched
with `NCS_CRAZYSIM_DISABLE_ACTUATORS=1`; the model remained on the ground and
the Gazebo motor-command topic stayed at zero during both captures.

The paired positive flight is archived separately in
[`../crazysim_nano_firmware_flight_20261020_attempt03/REPORT.md`](../crazysim_nano_firmware_flight_20261020_attempt03/REPORT.md).
This negative result uses the same `cf2` binary hash as that positive archive.
Both copies of `gz_crazysim_plugin.dylib` also have the same hash.

## Run and observations

The world ran in native Gazebo 8.14.0 under partition
`ncs-nano-negative-fixed`, with host `GZ_IP=127.0.0.1`. The CrazySim firmware
radio endpoint was `udp://127.0.0.1:21850`; the Nano process ran in Lima with
`CF2_SIM_MODEL=gz_crazysim`, `NCS_SITL_TOF_BRIDGE_PORT=21951`, and firmware
receive ports 19951 (ToF) and 19953 (UWB). The UWB broker used UDP 19952 and a
250 ms pose/clock skew allowance. The host ToF bridge used UDP 21951 because
Lima reserves host UDP 19951 for forwarding.

`run_demo.py` downloaded parameters, set the initial pose/readiness, and
streamed heartbeats. The firmware PTY log records UWB and ToF readiness,
fresh ToF frames, task readiness, and `Take-off`. It also reports scheduler
and estimator rate warnings, so this is not a timing-quality pass.

The plugin trace records 9,357 incoming motor packets, 4,358 with nonzero
firmware ratios. The first nonzero packet requests about 2,339 rad/s on the
four motors. During the 25-second active capture, Gazebo recorded 4,739
odometry, 10,877 IMU, and 21,752 motor messages. Odometry stayed at
z=0.0149992 m and x≈0; every motor command was `[0, 0, 0, 0]`. After stopping
the operator heartbeat, a separate 18-second capture recorded 3,380 odometry
and 15,225 motor messages, again at ground level with all motor commands zero.
The firmware terminal emitted `Mission ended after landing`; that final line
was not flushed into the archived PTY typescript, so the post-stop Gazebo
capture is the retained landing evidence.

The captures are in `active/` and `landed/`; `plugin-motors.jsonl` preserves
the firmware packet trace. `evidence.json` contains counts and computed
envelopes. `world.sdf`, `cf2`, and the plugin binary bind the evidence to the
run inputs; `SHA256SUMS` verifies all archived files.

## Limits

This proves that the firmware can issue motor commands while Gazebo action is
disconnected and that the model stays grounded. It does not pass the strict
paired-flight checker, validate mapping, exercise GAP9, or test multiple
drones. A first operator attempt in this run timed out while the parameter
TOC was cold; retrying on the same live instance succeeded. A later reconnect
attempt timed out at the link handshake. The runner now has separate bounded
link, TOC, and parameter-value phases, but that latest TOC-wait change has not
yet been revalidated in a fresh live Gazebo instance. The cold-cache and
reconnect behavior remains an open acceptance item.
