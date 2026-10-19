# Stale odometry failure acceptance passed

This isolated Gazebo Harmonic / ROS 2 Jazzy trial exercised the mission's stale-odometry failure response in the 1.4 m near-field mapping world. The DART plugin was rebuilt against Homebrew DART 6.20.1 from the existing gz-physics 7.8.0 source in `~/Work/gz-macOS`; the custom-name copy under the repository's ignored `build/` directory forced Gazebo to load that compatible plugin rather than its built-in plugin compiled against DART 6.19. No external checkout or Homebrew files were modified by the rebuild.

The probe passed every acceptance check: odometry was stopped after HOVER; the mission published FAILED with the exact stale-odometry reason and exited 1; it did not publish `enable=True` after FAILED; it published at least two zero-velocity commands; mission odometry stopped while the independent truth-odometry bridge continued for at least five seconds. The probe observed 250 post-exit samples over 5.002 seconds and altitude remained between 0.580655 m and 0.581244 m with maximum absolute vertical velocity 0.000231 m/s.

The truth stream establishes that the simulator continued and the vehicle remained near a steady altitude after mission failure. This is a failure-path acceptance test, not evidence of successful SLAM mapping or real-hardware behavior.

The Gazebo server exited cleanly on shutdown. The GUI did not respond to SIGINT or SIGTERM and launch escalated to SIGKILL; ROS bridge processes also emitted Fast DDS shared-memory mutex errors during shutdown. No Nano-C-SLAM launch processes remained afterward. Two unrelated Durandal simulation processes (PIDs 38302 and 39029) remained running and were not touched. This shutdown rough edge is retained as a separate follow-up, not hidden by the passing mission assertions.
