# CrazySim cold-start Kalman guard mission attempt 02

Date: 2026-10-20. This uses the upstream Crazyflie SITL firmware in Gazebo, not Nano-C-SLAM's STM32 application. It does not validate mapping.

## Setup and evidence

Gazebo Harmonic 8.14.0 ran natively on macOS; the SITL `cf2` executable ran in Lima. The firmware process started before the Gazebo server in a unique `GZ_PARTITION`, so its first recorded IMU sequence began at source time 2.048 s with zero sequence gaps. The run used the Kalman startup guard later committed and pushed as firmware commit `a0e6faba`. The exact executable hash for this run was not retained; the post-run binary was rebuilt after diagnostics-only formatting changes.

The firmware reached `Ready to fly`. TAKEOFF_2 to 2.4 m and LAND_2 to 2.015 m each received firmware ACK result 0. Its bounded trace contains 4,485 records and 3,016 nonzero motor-ratio rows. The highest firmware-estimated altitude was 2.48222 m and the highest altitude setpoint was 2.4 m. The run contains no Kalman bounds reset or supervisor crash. The Gazebo plugin logged a nonzero firmware-originated motor ratio `(37673, 37673, 37673, 37673)`.

The unchanged host sampler failed discovery: it saw topic names but reported zero matching publisher endpoints for odometry, IMU, motor commands, world stats, and joint state, then exited without telemetry. Manual `gz topic -e` / `-f` probes also yielded no messages. Therefore there is no Gazebo odometry, world RTF, motor-topic callback, stable-hover interval, or physical landing evidence, and the strict flight checker was not run. Firmware ACKs, estimated altitude, and a nonzero plugin motor log do not prove that Gazebo physics followed the command.

Firmware rate health still fails: six Kalman prediction warnings were 96–98 Hz against 100 Hz; one stabilizer warning reported 506 Hz against 500 Hz. The `timeout` wrapper recorded exit code 124 after requesting its 30-second deadline; the firmware log confirms the scheduler stopped cleanly. Gazebo's shutdown did not complete and its exact exit code was not retained.

## Interpretation

The cold-start reset trigger is addressed at the Kalman prediction boundary: when either acceleration or gyro has no accumulated sample, the estimator advances its prediction timestamp without integrating the zero-filled interval. The prior reproduction measured a 2,065 ms prediction with zero acceleration samples and a vertical velocity crossing below −20 m/s. The new firmware unit regression covers both the skipped interval and the unguarded free-fall result. This mission attempt provides runtime evidence of no bounds reset with zero IMU sequence gaps, but it is not a strict flight pass.

Next work is tracked in bead `ncs-fea.5.8.2.7.1.3.3.2.1`: make the host telemetry subscriber discover and record live Gazebo publishers before attempting another positive/negative flight pair. Cadence warnings and mapping validation remain open.
