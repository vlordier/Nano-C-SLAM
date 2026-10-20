# CrazySim disconnected-actuator negative control — attempt 06

This is the upstream Crazyflie SITL reference firmware. It does not execute Nano-C-SLAM STM32 tasks or validate mapping.

## Configuration identity

The run uses the same sources, firmware/plugin/sampler binaries, physics engine and 500-Hz SITL configuration as positive attempt05; see `../crazysim_provenance_flight_20261020_attempt05/REPORT.md`. Its actuator bridge was explicitly disabled in the captured `world.sdf`. The generated firmware binary SHA-256 is `6a3c3463571e24da5a27024d07c1fb4458ad5eb43c232fda0799ff657b9653cb`.

## Result

TAKEOFF_2 and LAND_2 both returned firmware result 0. The firmware produced nonzero motor-ratio packets, and the plugin trace retained 7,918 packets with no reported drops. Gazebo's motor topic captured 32,829 messages (131,316 components), all zero. The model stayed at 2.014999 m with no measurable altitude rise. The plugin explicitly logged `actuator_bridge=disconnected`. Captured 6,576 odometry, 16,440 IMU, and 269 world-stat messages; world-stat coverage was 99.14%. Firmware, sampler and Gazebo exited 0, with one firmware process.

This is evidence that commanded firmware output alone does not move the model when the actuator bridge is disconnected. The firmware cadence warning remains (453-Hz stabilizer window; 97–98-Hz Kalman windows), so this does not satisfy the strict paired gate. The unchanged checker was run on positive attempt05 and this run was assessed as its negative control from retained topic and packet data; attempt06 was not separately passed through the CLI as a positive run.

## Artifacts

The captured `world.sdf` retains the negative bridge setting. Logs, captures and plugin packet trace are stored with deterministic gzip where large. `manifest.json` includes source/executable identities, compressed and decompressed hashes, and claim boundary.
