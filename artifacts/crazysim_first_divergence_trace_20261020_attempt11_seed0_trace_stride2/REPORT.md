# CrazySim trace-stride flight diagnostic: attempt 11

Date: 2026-10-20. This is an upstream Crazyflie SITL reference flight; it does not run Nano-C-SLAM's STM32 application or validate mapping.

## Setup and observed behavior

The run used Gazebo Harmonic 8.14 on macOS with seed 0, one `cf2` process in the Lima Ubuntu 24.04 VM, the CrazySim bridge fork at `8f30dc1`, and the same takeoff/land command values as seeded attempts 07–08. Firmware trace capture was configured at stride 2 (nominal 250 Hz), capacity 32,768 records, 120 bytes per record. Both TAKEOFF_2 to 2.4 m and LAND_2 to 2.015 m were acknowledged with result 0.

Gazebo captured 11,862 odometry, 28,963 IMU, 57,922 motor-command, 28,962 joint-state, and 472 world-stat records. The model rose from 2.015 m to 2.508 m and returned to 2.015 m. Maximum odometry tilt was 0.0675 degrees. The plugin retained 28,634 firmware UDP records with zero trace drops; 9,802 contained at least one nonzero motor ratio. This confirms commanded ascent, return, packet reception and nonzero motor commands in this diagnostic run.

The firmware logged `stabilizer loop rate is off (421)`. It also logged `Landing timeout, disarming`; source inspection shows this is the normal supervisor transition from Landed to Reset after the configured three-second post-landing timer, not a failed-landing diagnostic. Its ring retained 16,061 records and counted 16,062 skipped stabilizer steps. Aggregate RTF was 0.882 (median 0.954), outside the registered 0.95–1.05 gate. Therefore this is not an accepted stable-flight result, and the observer has not been shown to be overhead-neutral. Attempt 08's full-rate trace crashed at 450 Hz; attempt 11's stride-2 trace did not crash, but measured only 421 Hz. The difference does not prove the stride caused or fixed the control behavior because simulation RTF and run scheduling differ.

## Interpretation and remaining work

Stride 2 reduced record writes and retained enough actuator packets to show nonzero firmware output, but it did not establish the registered loop-rate, RTF, landing-predicate, or strict paired flight conditions. Three other Gazebo processes from separate workspaces were active on the host during this run. They may have competed for CPU, but no per-process resource profile was retained, so they cannot be identified as the cause of the low RTF. The capture also has no matched disconnected-actuator run using this exact binary and configuration. Keep bead `ncs-fea.5.8.2.7.1.3.3` open.

Next, make the recorder genuinely low-cost (e.g. a fixed-size compact record with producer-side sampling and a bounded, separately scheduled drain; measure drops and loop timing with trace off/on), then run paired seed-0 positive and disconnected-control captures under the unchanged strict checker. Only after that should the trace be used to assert a precise first-divergence cause. Nano firmware execution and mapping remain unvalidated.

See `manifest.json` and `SHA256SUMS` for capture hashes and machine-readable measurements. JSONL and firmware logs are gzip-compressed without altering their contents.

The unchanged checker was invoked against attempt 11 and disconnected
control attempt 04. The negative-control physics/bridge conditions passed. The
positive diagnostics returned exactly two errors: aggregate Gazebo RTF 0.890 (required 0.95–1.05)
and firmware estimator/rate/crash diagnostics. Full machine output and the checker process exit status are in
`CHECKER_OUTPUT.json` and `CHECKER_EXIT_CODE.txt`. However, the checker was
supplied firmware/Gazebo exit-code arguments of 0 without retained wrapper
return codes, so process-exit acceptance remains unverified; see the limitation
above.

The checker invocation set both firmware and Gazebo exit-code arguments to 0,
but the shell process return codes were not retained as raw evidence (the
firmware log contains the clean scheduler-stop marker; the Gazebo session
reported its plugin destructor summary). Consequently, treat the two listed
checker errors as the verified physics/firmware findings, while the strict
process-exit criterion remains unverified for this attempt and for the paired
attempt04 metadata. A repeat must record each wrapper's actual exit status.
