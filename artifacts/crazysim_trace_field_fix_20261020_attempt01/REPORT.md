# Crazyflie reference SITL trace-field correction — attempt 01

Date: 2026-10-20. Classification: diagnostic trace/schema validation only; not an accepted flight and not Nano-C-SLAM firmware.

## Purpose and configuration

This run verifies that the schema-3 firmware trace prints the Kalman prediction counter in the correct field after Crazyflie firmware fork commit `662a7916` (`fix(sitl): align Kalman trace fields`). The Linux AArch64 `cf2` binary was rebuilt in Lima from the existing provenance/trace-enabled source configuration; its SHA-256 is `d5b1127507500e93e1ca731aaf357a2e8a6ade294fa26b255ab80a07b4ab094d`. The native Gazebo 8.14 CrazySim plugin SHA-256 is `fb57bcafa1ec2651f180a5116cbe369f7c0fb69d5fe47c54f10040dcab200640`. The SDF is retained here. Gazebo ran with seed 0, Featherstone physics, `GZ_PARTITION=ncs-kalman-field-fix`, and `GZ_IP=127.0.0.1`. Firmware connected as `cf2 19950 192.168.5.2` and reached `SUP: Ready to fly` after calibration.

## Result

The trace header reported schema 3 and 98 records. Every sampled line placed `kalman_predictions` before `imu_seq`, with nonzero monotonic fields. First and last retained rows were:

- First: tick 1258, step 32, source time 8.348 s, predictions 7, IMU sequence 4174.
- Last: tick 8720, step 3136, source time 14.556 s, predictions 746, IMU sequence 7278.

Across those trace endpoints, the firmware tick advanced 7.462 s and the prediction counter advanced 739, giving 99.0 predictions/s against the 100-Hz target. The stabilizer step counter advanced 3,104, or 416 steps/s against 500 Hz. The corresponding firmware log emitted warnings at 97–98 Hz Kalman and 419 Hz stabilizer. This is consistent with the counters and confirms the corrected fields are distinguishable; the runtime cadence still fails the existing guard.

The Gazebo IMU sequence advanced 3,104 over 6.208 s of source simulation time, consistent with 500 Hz and with no sequence discontinuity within the retained firmware trace interval. The sampler recorded 2,610 IMU callbacks, 1,242 odometry callbacks, 5,222 motor components and 78 world-stat callbacks. World-stat simulation time advanced 5.173 s while real time advanced about 9.933 s (RTF about 0.52), so this is a loaded diagnostic run and is not suitable for a controlled rate or flight claim. The captured model remained stationary on its platform; all motor ratios were zero. No mission command was sent.

The firmware was ended by the bounded guest `timeout`; the log contains `SITL: Scheduler stopped cleanly`. The host capture process returned 124 because that guest timeout expired, not because the bounded stdout capture lost data (46,642 of 46,642 bytes were recorded).

## Claim boundary and next step

The prior schema-3 stationary capture's reported Kalman/sequence fields must be disregarded because its `printf` argument ordering was wrong. This run verifies the corrected formatting and confirms the rate warning with independent counters, but RTF≈0.52 and active unrelated Gazebo processes prevent attributing the cadence shortfall. Repeat under controlled host load with one owned Gazebo instance, capture simulator and firmware rates, then run the unchanged positive and disconnected-actuator flight gates. This remains upstream Crazyflie reference SITL: Nano-C-SLAM firmware execution, STM32 motor control, ToF/UWB integration and mapping accuracy remain unvalidated.
