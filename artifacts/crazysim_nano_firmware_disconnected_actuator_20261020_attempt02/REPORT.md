# Fresh-instance Nano cflib and disconnected-actuator check, 2026-10-20

This capture exercises the committed `run_demo.py` connection path against a
fresh native Gazebo/CrazySim server and a fresh Nano `cf2` process. Gazebo ran
with `NCS_CRAZYSIM_DISABLE_ACTUATORS=1`, so this is deliberately a negative
actuator control, not a firmware flight.

## Result

On the successful operator attempt, `run_demo.py` printed `Parameters
downloaded`, set the configured initial pose, and began its heartbeat. It
received 379 Nano pose packets. The plugin motor trace contains 7,810 sampled
firmware packets, 2,207 with nonzero motor ratios, and a maximum requested
speed of 2,774.98 rad/s. During the 20-second active capture, Gazebo recorded
4,414 odometry, 10,015 IMU, and 20,030 motor samples. The model stayed at
z=0.0149991 m; every Gazebo motor command was zero; all measured rotor
velocities were within 2.0e-9 rad/s of zero. After the operator stopped, the
18-second capture recorded 3,817 odometry and 18,004 zero motor messages, with
the same ground pose and near-zero rotor velocities.

The Gazebo process reported `actuator_bridge=disconnected` at startup. The
firmware PTY capture proves UWB/ToF initialization, fresh ToF data, and task
readiness. The interactive firmware session also printed `Start`, `Init`,
`Take-off`, and later `Mission ended after landing`; those final console lines
were visible in the live session but were not flushed into the saved PTY file.
The retained actuator evidence is therefore the exact plugin source trace and
paired Gazebo motor/joint/odometry streams, not a claim based on the saved
console text.

## Reproduction inputs

- Gazebo 8.14.0, partition `ncs-nano-fresh-toc2-20261020`, `GZ_IP=127.0.0.1`
- Engine selection: `--physics-engine gz-physics-bullet-featherstone-plugin`
- Operator command: `uv run --locked python cslam-stm32-app/run_demo.py 1 --uri udp://127.0.0.1:21850 --log-root <run-log-dir>`
- Firmware: `CF2_SIM_MODEL=gz_crazysim ./cf2 21950 192.168.5.2`, with
  `NCS_SITL_TOF_BRIDGE_PORT=21951`
- Host ToF bridge: UDP 21951; firmware ToF receiver: UDP 19951
- UWB broker: UDP 19952, one configured broker node and 250 ms pose/clock
  skew allowance
- Host cflib TOC cache was restored from the pre-run workspace for the
  successful attempt. The first attempt with that cache moved aside timed out
  at the 10-second link-handshake phase. A direct diagnostic then observed
  `link_established`, but its temporary packet-print callback raised an error
  and stopped that diagnostic's incoming thread. The subsequent `run_demo`
  attempt succeeded. Thus this run proves live setup with the committed
  three-phase runner, but does not prove reliable first connection with an
  empty TOC cache.

`active/` and `post-stop/` hold compressed raw Gazebo captures. `plugin-motors.jsonl`
holds the source-side firmware motor packets, and `firmware-pty-capture.log`
holds the persisted firmware output. `world.sdf`, `cf2`, and the plugin binary
bind the capture to its inputs; `evidence.json` contains computed metrics and
`SHA256SUMS` verifies every archived file.

## Remaining limits

The firmware reported that UWB token handoff exhausted because no peer drone
was present. Its stabilizer and estimator also emitted rate warnings. This
capture does not establish a clean solo mission or flight. The separate
positive flight evidence remains in
[`../crazysim_nano_firmware_flight_20261020_attempt03/REPORT.md`](../crazysim_nano_firmware_flight_20261020_attempt03/REPORT.md).
Mapping quality, GAP9 execution, multi-drone operation, and strict paired
acceptance remain unvalidated.
