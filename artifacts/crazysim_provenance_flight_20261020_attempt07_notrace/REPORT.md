# CrazySim no-ring scheduler diagnostic — attempt 07

This is the upstream Crazyflie SITL reference firmware. No Nano-C-SLAM task or mapping code ran. This is a failed harness/host-load diagnostic, not a flight or strict-gate trial.

## Build identity

- Guest source snapshot commit: `c86ca621d6adaa859179044821f5c6178bc0dbfb`, based on firmware `a0e6faba30d3d73bb2838b9034aeee1b756ca3e5` and CrazySim plugin commit `02429e8e3385d88ef62a7954df491681795000bd`.
- The guest-only CMake source edit changed `-O0` to `-O2`. However, `CMAKE_BUILD_TYPE=Release` appended `-O3 -DNDEBUG` later in `C_FLAGS`; GCC uses the last optimization flag. Effective optimization was **-O3**, not -O2.
- Firmware SHA-256: `c363793058228731bcdf1e293ad06ad362bdacc415f87c8ec42e49101239273c`.
- 500-Hz stabilizer build, lightweight readiness diagnostics on, verbose diagnostics off, trace ring/trace diagnostics/IMU provenance off.
- Same native Gazebo world, physics engine, plugin, and sampler as attempt05. Plugin SHA-256 `2e70696d11235be4053cb72b65992b6ef2a01af30fd089c300ba2809d67e55e5`; sampler SHA-256 `7a302dff29bb617d2c3899dcdb97cc6191559bcf4dea6cac906c316f96a8eaf8`.

## Observation

Gazebo accepted the firmware handshake and the firmware eventually reached `SUP: Ready to fly`. No takeoff or landing command was sent. Captures contain 4,601 odometry, 11,504 IMU, 91,824 motor-component and 260 world-stat messages. Gazebo ran at aggregate RTF 0.6553. The firmware logged a 406-Hz stabilizer window and Kalman windows from 94 to 98 Hz.

At capture time three unrelated Gazebo processes were active on the same host (PIDs 38302, 39029, 76433; see `host-load.txt`). This run therefore cannot isolate trace overhead or scheduler behavior from host contention. It is not comparable to attempt05's RTF 0.9999 and does not pass the strict checker: no mission was commanded, and the RTF requirement is missed.

The first harness waited for readiness by reading redirected firmware stdout, but the guest process used fully buffered stdio. Readiness became visible only after direct SIGTERM flushed the log. The firmware log ends with `Scheduler stopped cleanly`; its wrapper exit receipt is missing, so its process exit code is unverified. Gazebo and the host sampler each exited 0. The corrected harness must line-buffer firmware output and track the actual `cf2` child PID before repeating the comparison.

All large captures use deterministic gzip. `manifest.json` and `SHA256SUMS` retain compressed and decompressed hashes. Do not use this run to claim a flight, a trace-overhead result, or a scheduler fix.
