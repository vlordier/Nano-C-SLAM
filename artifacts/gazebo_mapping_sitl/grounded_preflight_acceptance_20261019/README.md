# Grounded odometry preflight acceptance (2026-10-19)

This directory retains evidence for the coordinator's grounded-odometry gate.
Both tests used native macOS Gazebo and the locally built ROS overlay; neither
used flight hardware.

## Passing mapping run

`nano-c-slam-preflight-final42-20261019.json` and its provenance sidecar are a
fresh run on ROS domain 42, Gazebo partition
`nano_c_slam_preflight_final42_20261019`, and world-session UUID
`ab97b10b-13cd-47c1-9391-cf0ccddb5d38`. The launch recorded Gazebo server PID
90759. Before it started either mission or collector, the coordinator saved
three distinct odometry callbacks at simulation times 10.740, 10.763, and
10.781 seconds. All three source stamps were increasing, callback gaps were
below 0.024 seconds, and altitude was approximately -0.00000085 m.

The mission and collector both exited 0. The host mapping replay produced 60
finite poses, 3 scan windows, 199 projected map points, 3 stored graph
constraints, 3 successful solver calls, and all 9 configured solver
iterations. This remains synthetic, single-drone Gazebo host-SITL evidence; it
does not validate target firmware, GAP9/GVSOC, sensor fidelity, mapping
accuracy, multi-drone behavior, or physical flight.

Verify the report and retained child streams from the repository root:

```sh
uv run --locked python -m scripts.gazebo_run_manifest \
  artifacts/gazebo_mapping_sitl/grounded_preflight_acceptance_20261019/nano-c-slam-preflight-final42-20261019.json
```

## Live negative controls

`nano-c-slam-preflight-duplicate-final42-20261019.evidence.json` records a
second coordinator process (PID 93961) against the same live session. It exited
1 with the existing-session-claim diagnostic, created no report, sidecar,
collector log directory, or capture artifact, and left Gazebo server PID
90759 alive. Its output and the session manifest are retained.

`nano-c-slam-preflight-no-odom-20261019.evidence.json` records a separate fresh
world session (session UUID `8a782542-ee55-4a2b-b90e-e3887504ed4a`, Gazebo
server PID 98754). Gazebo and bridges ran on ROS domain 42 while the coordinator
was deliberately isolated on domain 41. With a two-second wall timeout, the
coordinator failed with `odometry preflight timed out: no grounded samples
arrived`; it created no report, sidecar, child logs, capture artifact, or
mission/collector process. This tests missing-odometry rejection, not paused
physics. Pure tests cover the wall-time timeout and repeated-stamp/paused-clock
case.

The attempt on domain 43 was discarded: Fast DDS blocked the sensor bridge
during participant creation, so that world never published the mapping range
topic and the mission was not started. The passing and missing-odometry
controls above used domain 42.

## Direct `SimMission` no-odometry control

The rebuilt isolated overlay at `build/gazebo-ros-overlay.Dvetqa` contains the
exact tested `sim_mission.py` source (SHA-256 is in
`direct-sim-mission-no-odom-20261019.json`). Its colcon build receipt and
before-build source snapshot are retained alongside the harness output. The
test ran only the installed `sim_mission` entry point on ROS domain 48, with
`use_sim_time=True`, a two-second steady-time startup timeout, and no
`/clock` or `/cf1/odom` publishers.

The actual mission process published WAITING then FAILED at simulation time
zero, logged `odometry_callbacks=0`, exited 1, never published enable, and
published one all-zero cmd_vel at watchdog teardown. This is the direct
mission-level fail-closed control; the separate coordinator no-odom control
above verifies rejection before either child starts.

## Independent fresh-session `SimMission` success trial

`trial_b/trial-b-success.json` records a separately launched near-field world
on ROS domain 48 and Gazebo partition `nano_c_slam_trial_b_20261019`. Its
session UUID (`1acabff6-5d16-4b7e-988c-787947f8f5b0`) differs from trial A's
(`ab97b10b-13cd-47c1-9391-cf0ccddb5d38`); the domain and partition differ too.
The pre-launch process audit found no existing process using that domain or
partition. Three consecutive samples at x/y = -0.5/-0.5 m and z approximately
−0.00000085 m passed the grounded gate before `SimMission` started. The ROS
clock advanced, the installed source hash matched the tested source, and the
mission logged 195 odometry callbacks before exiting 0 after ordered
WAITING/TAKEOFF/HOVER/LAND/COMPLETE statuses. All trial-owned launch, Gazebo,
bridge, and mission PIDs were absent after teardown. Raw mission logs,
pre/post process audits, and the world manifest are retained in `trial_b/`.

This trial only checks the installed mission's odometry and flight-state path;
it makes no mapping or SLAM claim.
