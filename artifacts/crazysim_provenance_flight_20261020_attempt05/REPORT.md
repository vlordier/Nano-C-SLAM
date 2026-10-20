# CrazySim reference flight diagnostic — attempt 05

This is the upstream Crazyflie SITL reference firmware. It does not execute Nano-C-SLAM STM32 tasks or validate mapping.

## Configuration identity

- Firmware source: `a0e6faba30d3d73bb2838b9034aeee1b756ca3e5`; isolated Lima source snapshot also included CrazySim plugin `02429e8e3385d88ef62a7954df491681795000bd`.
- Firmware binary SHA-256: `6a3c3463571e24da5a27024d07c1fb4458ad5eb43c232fda0799ff657b9653cb`.
- SITL loop: 500 Hz. Trace: capacity 8192, stride 32, IMU provenance enabled. The guest-only snapshot changed `sitl_make/CMakeLists.txt` optimization from `-O0` to `-O2`; that edit was not committed upstream.
- CrazySim plugin SHA-256: `2e70696d11235be4053cb72b65992b6ef2a01af30fd089c300ba2809d67e55e5`.
- Host sampler SHA-256: `7a302dff29bb617d2c3899dcdb97cc6191559bcf4dea6cac906c316f96a8eaf8`.
- Native Gazebo Sim 8.14.0 Harmonic with rebuilt DART 6.20.1; Lima Ubuntu 24.04 AArch64 runs `cf2`. Unique `GZ_PARTITION`, `GZ_IP=127.0.0.1`.

## Result

TAKEOFF_2 to 2.4 m and LAND_2 to the spawn platform both returned firmware result 0. Captured 7,030 odometry, 17,576 IMU, and 35,062 motor messages (140,248 motor components). Altitude started at 2.014990 m, peaked at 2.508163 m, and ended at 2.014999 m. Stable hover interval: 7.99 s. Maximum observed motor speed: 2104.30 rad/s. World statistics covered 99.62% of the odometry interval; aggregate RTF was 0.999873. Firmware, sampler and Gazebo exited 0; one `cf2` process was recorded.

The unchanged strict gate **fails**, with exactly one error: `firmware reported estimator/rate/crash failure`. Firmware reports one 503-Hz stabilizer window against 500 Hz and Kalman windows at 96–98 Hz against 100 Hz. The gate correctly rejects this run despite the useful physical response.

## Reproduction and artifacts

The exact strict-gate JSON result is retained in `strict-check.json`.

The captured `world.sdf` is the launch input. Artifacts include source logs, command ACKs, firmware and Gazebo exit receipts, raw captures and bounded plugin packet traces. Large logs are stored as deterministic gzip (`gzip -n`); manifest entries record both compressed and decompressed SHA-256 and sizes. `manifest.json` records executable/source identities and the explicit claim boundary. The official checker reads `.gz` inputs directly.
