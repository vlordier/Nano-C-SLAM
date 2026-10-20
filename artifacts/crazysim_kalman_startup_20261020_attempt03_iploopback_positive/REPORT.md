# CrazySim cold-start mission with matched Gazebo transport address

Date: 2026-10-20. This run exercised the upstream Crazyflie reference SITL firmware in Gazebo Harmonic. It did not run Nano-C-SLAM's STM32 application and did not validate mapping.

## Captured evidence

The Gazebo server and native typed sampler shared `GZ_PARTITION=ncs-kalman-positive-iploopback-01` and `GZ_IP=127.0.0.1`. The sampler discovered live, type-correct publishers and recorded 5,558 odometry messages, 13,025 IMU messages, 26,050 four-channel motor messages, 13,025 joint-state messages and 211 world-stat messages. The corresponding compressed JSONL captures and firmware console log are retained here.

Gazebo odometry started at 2.01457 m, reached 2.48238 m, included a 7.45 s interval within the checker's stable-hover band around 2.4 m, and ended at 2.01500 m. There were 63,208 nonzero motor components. The firmware log contains successful transport reception markers for TAKEOFF_2 (`data=07`) and LAND_2 (`data=08`), one CRTP receive-start marker, no crash or Kalman-bounds marker, and a clean scheduler-shutdown marker.

## Acceptance status

This is a meaningful reference-firmware rise-hover-land observation, but it is not a strict acceptance run. The mean reported world RTF was 0.9052 (required 0.95–1.05). The firmware logged one 477-Hz stabilizer warning (nominal 500 Hz) and three 98-Hz Kalman prediction warnings (nominal 100 Hz). The outer timeout wrapper returned 124 even though the firmware reported a clean scheduler stop. The world SDF, Gazebo process exit code, plugin log, exact executable hash, firmware process count during the run, and machine-readable command ACK files were not retained, so the paired strict checker cannot be applied to this run. The SDF copied into this evidence directory is from the preceding attempt and is not asserted to be the exact run input.

The next diagnostic must retain the exact world, plugin binary/hash and logs, firmware binary/hash, positive and disconnected-actuator captures, command ACKs, live process counts and actual child exit codes. It must preserve the existing strict checker thresholds. Even a passing Crazyflie reference pair would not establish Nano firmware flight or mapping correctness.
