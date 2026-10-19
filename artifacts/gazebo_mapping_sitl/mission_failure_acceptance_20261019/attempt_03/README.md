# Failed stale-odometry attempt: incompatible installed DART plugin

This clean-source trial did not reach HOVER and is not acceptance evidence. The installed Gazebo DART engine plugin requested `libdart-collision-bullet.6.19.dylib`, while Homebrew had updated `dartsim` to 6.20.1. `/clock` and odometry stamps advanced, but cf1 remained at 0.05 m; the mission exited with `takeoff phase timed out`. Probe acceptance was false, including `failure_triggered_after_hover`, `stale_odometry_failure_reason`, and `truth_odometry_continued_after_failure`.

The incompatibility was resolved for a rerun by rebuilding the local gz-physics 7.8.0 DART wrapper against installed DART 6.20.1 into ignored `build/`, without modifying the `~/Work/gz-macOS` checkout or Homebrew. The build receipt and plugin hash are retained here. This attempt remains a recorded environment failure, not a successful stale-odom test.
