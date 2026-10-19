# Fresh Gazebo acceptance attempts (2026-10-19)

These runs are failure evidence only. None produced a passing capture or a
mapping result. The retained `.capture.ready` files mean only that the host
collector reached its ROS subscription setup; they do not prove telemetry was
received.

## What was exercised

| Attempt | ROS domain / Gazebo partition | Result |
| --- | --- | --- |
| Initial isolated-overlay launch | 46 / `nano_c_slam_acceptance_20261019_7f41` | Launch aborted because the Crazyswarm install had dangling ament-index links for `crazyflie_ros2_multiranger_wall_following` and related packages. No mission was started. |
| Run `cc4e710e-5fe0-4290-8fc5-b68044e6859d` | 47 / `nano_c_slam_acceptance_20261019_c883` | Coordinator rejected `/nano_c_slam/sim/cf1/ranges_mm`: it expected `StampedPairwiseRanges`, while the actual sensor publisher uses `StampedRanges`. No mission was started. |
| Run `21dddd62-6207-4356-b95e-ef461c72a12e` | 48 / `nano_c_slam_acceptance_20261019_93af` | Coordinator waited for two `/cf1/mission_status` subscribers before starting the mission. Only its own subscriber exists at that stage. This preflight deadlock was interrupted; no mission was started. |
| Run `49d91bb3-8d95-4dea-944b-8a059d800280` | 49 / `nano_c_slam_acceptance_20261019_e815` | Mission started and reported `WAITING`, but received zero odometry callbacks and failed its 10-second startup watchdog. The Gazebo server had aborted with `SIGABRT`; macOS crash report identifies `Bad file descriptor (src/tcp_connecter.cpp:263)` in ZeroMQ, and the GUI subsequently reported `Broken pipe`. |

## Changes made and verified

- `scripts/build_gazebo_ros_overlay.sh` now includes available Crazyswarm
  multiranger packages as explicit overlay targets. This recreates valid
  package-index entries in the isolated overlay without modifying the shared
  Crazyswarm installation.
- The mapping coordinator now expects `StampedRanges` for per-drone lidar and
  starts the mission when its one mission-status subscriber and telemetry
  endpoints are discovered.
- Host tests cover the exact topic type and one-subscriber startup condition.
- The standard isolated overlay builder completed, ShellCheck passed, and the
  focused Python tests, Ruff, format check, and `ty` passed.

## Boundary

Gazebo did not deliver `/cf1/odom`; the last run therefore proves only that the
mission's startup watchdog emits `FAILED` under missing odometry. No flight,
mapping, SLAM, firmware, ToF/UWB fidelity, or hardware claim is supported. The
next blocker is the reproducible macOS Gazebo/ZeroMQ server abort; the next
acceptance attempt must use a fresh launch after that is resolved.

Raw coordinator logs and collector-ready markers are retained in this
directory by run ID. The macOS crash report is retained outside the repository
at `~/Library/Logs/DiagnosticReports/ruby-2026-10-19-070444.ips`.
