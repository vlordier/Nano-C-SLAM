# CrazySim Kalman startup diagnostic, attempt 01

Classification: diagnostic only; not a flight acceptance run and not Nano-C-SLAM firmware.

The Crazyflie SITL firmware was built in Lima from the diagnostics/provenance worktree with the one-shot bounds diagnostic from commit `5192377bbc8b879cfaa3ca0315b77ca189206553`. Gazebo Harmonic 8.14 loaded the provenance-enabled CrazySim plugin and corrected odometry publisher. Firmware destination UDP port `19950` matched the plugin's SDF listener port. The handshake completed, gyro calibration completed, the stabilizer started, and the firmware reached `Ready to fly`.

This was an 8-second no-command startup capture. Gazebo had been running before the firmware was launched; the first trace row reports source simulation time `51.576 s` and sequence-gap counter `21722`. The trace contains 1,749 records. It reports no Kalman bounds reset and no Kalman prediction-rate warning during this interval; the stabilizer emitted one `503 Hz` warning. The firmware trace also shows a transient position estimate (`z_m=-4.83657`) before it settles near the Gazebo spawn altitude, but this field alone does not establish the internal Kalman state or a bound violation.

The run does not reproduce the 24 `State out of bounds, resetting` warnings in attempt14. Because simulation time was already 51.6 s when the firmware started, this result only demonstrates that the reset is not inevitable on every firmware boot. It does not identify the source of the attempt14 reset. `timeout` returned 124 after requesting orderly shutdown; the firmware log ends with `SITL: Scheduler stopped cleanly`. No motor command was sent, and no strict paired flight checker was run.

The next diagnostic must launch a fresh Gazebo process from sim time zero and start the same firmware immediately after the plugin binds UDP 19950. It should retain the one-shot Kalman state diagnostic, plugin IMU send trace, and world time so that any reset can be tied to actual source samples.
