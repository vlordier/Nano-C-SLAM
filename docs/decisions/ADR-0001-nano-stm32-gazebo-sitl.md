# ADR-0001: Run Nano-C-SLAM application code in a firmware-driven Gazebo SITL

- Status: Accepted for the first one-drone SITL implementation
- Date: 2026-10-20
- Scope: source-level execution of the owned Nano STM32 application and flight
  control in Gazebo; this does not emulate the Cortex-M4 instruction set or
  prove timing on an STM32 board.

## Decision

Extend the pinned CrazySim Linux `cf2` FreeRTOS/POSIX target to compile and run
the owned `cslam-stm32-app` task graph. Keep Gazebo native on macOS and use the
CrazySim UDP sensor and actuator boundary between the macOS Gazebo process and
the Linux firmware process in Lima. Preserve the Nano application task and
control sources; introduce named SITL implementations only at hardware
boundaries. Do not route flight commands through ROS `/cmd_vel`.

The reference baseline is CrazySim `gtfactslab/CrazySim`
`3ec8b55da4bff887da542a9f314da825460e65be`, with its documented Linux
FreeRTOS/POSIX `cf2` and Gazebo UDP plugin architecture. The working fork used
for implementation is `vlordier/crazyflie-firmware`, branch
`codex/sitl-first-divergence-trace`, commit `662a7916bbd4454bad117a39aa324000bdb0c004`,
whose `tools/crazyflie-simulation` submodule is pinned to
`30a6177ff5c14f0782a72a50c27a90a955d1b7b8`. The paired plugin build is native
Gazebo Sim 8.14.0 (Harmonic) from `~/Work/gz-macOS/ws/install`; the Linux
firmware `cf2` target builds under Lima Ubuntu 24.04 AArch64. The most recent
trace-validation binary SHA-256 is
`d5b1127507500e93e1ca731aaf357a2e8a6ade294fa26b255ab80a07b4ab094d`.

Nano SITL integration is carried by the pushed fork branch
[`codex/nano-app-sitl-integration`](https://github.com/vlordier/crazyflie-firmware/tree/codex/nano-app-sitl-integration),
currently at commit [`e8e26f4f`](https://github.com/vlordier/crazyflie-firmware/commit/e8e26f4f),
based on the reference baseline above. It adds the opt-in app source manifest,
enables `CONFIG_APP_ENABLE`, supplies Nano's bounded Kalman pose-init entry
point, and accepts an explicit SITL node ID.

Build the firmware with `sitl_make` target `cf2`, then add the Nano app sources
through an explicit `NCS_NANO_APP_DIR` CMake input. The SITL build must include
and start `appMain()` from a statically allocated FreeRTOS task. Source
selection and the exact source-file manifest must be generated from the app's
Kbuild files or checked against them; do not maintain a second hand-copied list
of application logic.

## Device contracts

| Boundary | SITL behavior | Required proof |
| --- | --- | --- |
| Flight outputs | Keep Crazyflie's actual `stabilizerTask`, controller and mixer. Its `power_distribution_sitl.c::motorsSetRatio` output is sent by the pinned plugin to Gazebo motor models. Nano `flight_task` may request setpoints through the same commander API. | Match firmware motor packet sequence and four ratios to Gazebo motor commands and joint velocities; disconnect the actuator bridge and show the vehicle stays put. |
| IMU and pose | Keep CrazySim's timestamped simulator IMU packet into `sensors_sitl`; retain source sequence/time, firmware receive tick, units and drop counters. Use Gazebo odometry as independent evaluation truth, not as hidden state injected into the estimator. | Compare source and firmware accepted samples; no unreported clock conversion or zero placeholder timestamps. |
| ToF matrices | Add a SITL provider implementing the existing `tof_matrix_init` / `get_tof_matrix` contract for four 8×8 `int16_t` matrices, same sensor ordering, millimetres, invalid-value semantics and freshness. It must derive cells from Gazebo ray geometry. The existing four-ray multiranger may seed tests but cannot stand in for all 256 matrix samples in mapping acceptance. | Deterministic wall/obstacle fixtures prove each sensor transform, cell order, range clipping and stale-data failure; a live run compares matrix rays with the Gazebo geometry. |
| GAP9 SPI | Connect the existing bounded SPI protocol to the repository's GVSOC GAP9 testbench. Keep packet bytes, size and timeout behavior identical to the hardware interface. A mock is permitted only for isolated startup tests and must be explicitly labelled. | Live GVSOC receives Nano pose/scan packets and returns optimizer results in the same run; exercise timeout and malformed replies. |
| UWB and radio | Preserve the owned Nano packet/state logic. Supply simulated node identity and timestamped ranging through a transport adapter; use real FreeRTOS queues and token scheduling. Use CRTP only for operator commands/telemetry supported by the existing SITL bridge. | Single-node and multi-node protocol tests cover identity, loss, stale packets, token lease expiry and safe landing. No silent always-success ranging stub. |
| Safety and lifecycle | Keep heartbeat, ToF freshness, mission deadline, land/fault and task-shutdown logic. Each adapter must report initialization, freshness and failure. | Missing/stale input and disconnected devices fail closed, yield nonzero run result, and retain a reasoned terminal marker. |

## Alternatives considered

| Candidate | Evidence and compatibility | Decision |
| --- | --- | --- |
| `gtfactslab/CrazySim` and the current `vlordier` fork | The fork's Linux `cf2` target builds in Lima; the matching native Gazebo 8.15 plugin loads, handshakes and exchanges IMU/odometry with firmware. The Nano app now links and reaches `appMain()`, then exits at its intentionally unconfigured UWB provider before mission tasks; its trace has zero motor ratios. Flight acceptance remains open. Gazebo Garden is upstream's documented baseline; the fork plugin also builds and runs on the local Harmonic stack. | Choose as the smallest tested firmware-driven path while retaining native macOS Gazebo. |
| CrazyflieTHI `sim_cf2` | Linux SITL/FreeRTOS is a relevant reference, but this machine has no reproduced checkout/build/flight for it. The documented ROS 2 Humble / Gazebo 11 / Ubuntu 22.04 stack differs from the installed native Jazzy / Harmonic stack. Switching stacks would discard the working local Gazebo plugin boundary without evidence of a benefit for Nano task integration. | Keep as a fallback/reference only; do not migrate the simulator now. |
| `prakash-aryan/crazyswarm2_gazebo` at local checkout `5f2e4cc8e9687fb4c05e9e4be40a20370a022f9e` | Local README states Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic, and describes a Python-bound firmware backend plus PD conversion to `/cmd_vel`. Its demonstrated flight path is not Nano app execution and motor-ratio actuation. The checkout also contains unrelated dirty submodule/untracked changes and is left untouched. | Retain for synthetic geometry, sensor and mapping experiments; reject as the firmware flight path. |
| Run the normal ARM `cf2` hardware image directly | `cslam-stm32-app` and the hardware `cf2` image build on macOS, but they target STM32F405 and embedded peripherals. Gazebo cannot execute that ARM image as a host process. Instruction-accurate execution would require a Cortex-M emulator and peripheral model that has not been reproduced here. | Do not claim binary-equivalent STM32 execution. The initial SITL validates the same owned C app sources under Linux FreeRTOS plus the Crazyflie SITL flight stack. |

## Consequences and acceptance boundary

This choice minimizes new simulator work and keeps the tested native Gazebo
install. It also creates a maintained fork seam: the SITL firmware is not the
repository's pinned hardware firmware submodule, so both revisions and the
app-source manifest must be recorded on every run. The POSIX FreeRTOS port
does not establish MCU timing, interrupt behavior, memory fit, or hardware
peripheral behavior.

The first implementation milestone is not “the firmware flies.” The opt-in
and default `cf2` builds pass, and a live boot proves handshake, self-test,
`appInit()`/`appMain()` and explicit fail-closed behavior at missing UWB. The
mission, ToF and UWB task graph is not yet reached; current boot evidence is in
[`nano_app_boot_sitl_20261020_attempt01`](../../artifacts/nano_app_boot_sitl_20261020_attempt01/REPORT.md).
Flight acceptance requires firmware-originated motor output,
Gazebo sensor input, a successful takeoff/hover/land positive run and a
disconnected-actuator negative control. Mapping acceptance additionally
requires live GVSOC SPI processing and an independent geometry comparison.
None of these gates is closed by the current Crazyflie reference runs.

## Reconsider this decision if

- the selected fork cannot compile the owned app without rewriting task or
  control logic;
- the native Gazebo/UDP path cannot sustain the registered timing envelope
  after controlled-load testing;
- GVSOC and simulator clocks cannot be correlated across the host/guest
  boundary; or
- the requirement is changed to execute the exact STM32 binary, in which case
  a Cortex-M emulator and faithful STM32 peripheral model must be evaluated.
