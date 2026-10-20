# STM32 application inventory for firmware SITL

## What counts as firmware flight

The current Gazebo mission is not this firmware: sim_mission.py publishes ROS
Twist messages to /<drone>/cmd_vel, and the Gazebo velocity controller moves
the model. The GAP9 GVSOC probe runs mapping firmware, but the Crazyflie STM32
app and its flight controller are absent. A valid firmware flight must run
appMain() and the Nano tasks, then demonstrate that firmware-produced motor
outputs drive Gazebo while firmware receives simulator sensor feedback. A ROS
command that bypasses those outputs fails this claim.

## Runtime tasks created by appMain

Task names and creation mechanisms below are guarded by
tests/test_stm32_sitl_inventory.py; adding or removing an app task requires
updating this inventory and its test expectation.

| Task symbol | Entry source | Creation | Purpose and blocking dependencies |
| --- | --- | --- | --- |
| mission_task | cslam-stm32-app/app_main.c | Static task, priority 2, stack allocation 2000 | Waits for ToF readiness, fresh sensor frames, and operator heartbeat; initializes the estimator; gates takeoff; forwards pose and scan records over CRTP; enforces mission timeout and landing. Depends on ToF status, FreeRTOS semaphores/queue, estimator, radio/CRTP, and GAP9 SPI. |
| tof_task | cslam-stm32-app/src/tof_daq.c | Static task, priority 3, stack allocation 500 | Initializes and polls four 8x8 ToF matrices sequentially, tracks per-sensor freshness, and signals acquisition semaphores. Current implementation depends on VL53L5CX, I2C, sensor power/address selection, and the deck platform. VL53L9CX is a planned additional source with a separate interface qualification; see [VL53L9CX integration](VL53L9CX_INTEGRATION.md). |
| flight_task | cslam-stm32-app/src/flight_commander.c | Static task, priority 2, stack allocation 1000 | Runs the Nano exploration controller, requests high-level takeoff/landing, and submits low-level velocity setpoints. Depends on Crazyflie estimator/logging/commander APIs, ToF freshness and matrix data, and upstream stabilization and motor-mixing tasks. |
| uwb_ranging_task | cslam-stm32-app/src/uwb_ranging.c | Dynamic xTaskCreate, priority 4, stack 5 * configMINIMAL_STACK_SIZE | Owns token scheduling, UWB ranging, scan exchange, pose updates, and GAP9 optimization exchange. Depends on UWB transceiver/API, radio-address-based node identity, FreeRTOS queues, estimator, and GAP9 SPI. |

show_optimized_poses() is a helper, not a task. appMain() also allocates two
ToF semaphores and a 100-element aug_pose_t queue, initializes the scan queue
and UWB node, then remains alive in a one-second delay loop.

## Owned modules selected by the app Kbuild files

This list is derived from object entries in the app Kbuild files and checked
against them by tests/test_stm32_sitl_inventory.py.

- app_main.c
- src/bounded_batch.c
- src/exploration/exploration.c
- src/exploration/limit_actuation.c
- src/exploration/line_detector.c
- src/flight_commander.c
- src/mission_timeout.c
- src/operator_heartbeat.c
- src/radio_comm.c
- src/scan.c
- src/scan_manager.c
- src/spi_comm/spi_comm.c
- src/spi_comm/spi_wait.c
- src/tof-matrix/drivers/i2c_expander.c
- src/tof-matrix/drivers/vl53l5cx_api.c
- src/tof-matrix/i2c_platform.c
- src/tof-matrix/tof_matrix.c
- src/tof_daq.c
- src/tof_freshness.c
- src/uwb_collision_det.c
- src/uwb_ranging.c
- src/uwb_token_handoff.c
- src/uwb_token_lease.c

The app Kbuild also links ../uwb-software-library/, a separate submodule. Its
Decawave driver and radio protocol are not owned app modules and cannot be
treated as a simulator implementation without an explicit device adapter.

### Source readiness matrix for the Linux `cf2` target

All 23 translation units below are selected by the recursive app Kbuilds for
the normal STM32 `cf2` OOT build.
`SITL-linked` means the owned translation unit was linked into the opt-in
CrazySim `cf2` target against its FreeRTOS POSIX port. This proves compilation
and linkage, not that its device calls have a functioning simulator peer.
`ARM-build only` marks physical I²C/VL53 units excluded from SITL.

| Owned source | Current evidence | Linux SITL dependency or boundary |
| --- | --- | --- |
| `app_main.c` | SITL-linked | Linked with CrazySim `app_handler.c`; UWB init is checked and currently fails closed through the unconfigured SITL provider. |
| `src/bounded_batch.c` | SITL-linked | FreeRTOS-independent bounded batch behavior is included unchanged. |
| `src/exploration/exploration.c` | SITL-linked | Nano exploration code links against the real CrazySim estimator/commander; no valid matrix is available yet. |
| `src/exploration/limit_actuation.c` | SITL-linked | Included unchanged before commander handoff; existing finite-value and physical-bound checks remain. |
| `src/exploration/line_detector.c` | SITL-linked | Compiled unchanged; no Gazebo matrix provider feeds its orientation/order contract yet. |
| `src/flight_commander.c` | SITL-linked | Uses CrazySim FreeRTOS, estimator and high-level commander; physical flight remains unverified. |
| `src/mission_timeout.c` | SITL-linked | Uses the SITL FreeRTOS tick; host fixture tests cover wrap-safe expiry. |
| `src/operator_heartbeat.c` | SITL-linked | Compiled, but no operator CRTP input bridge is configured. |
| `src/radio_comm.c` | SITL-linked | Compiled, but no operator command bridge is configured. |
| `src/scan.c` | SITL-linked | Existing matrix shape and scan serialization compile unchanged. |
| `src/scan_manager.c` | SITL-linked | Queue and timeout logic compile; actual GAP9/GVSOC SPI result exchange is absent. |
| `src/spi_comm/spi_comm.c` | SITL-linked | Packet encoding is unchanged; SITL calls a fail-closed GAP9 transport seam until a socket/GVSOC bridge exists. |
| `src/spi_comm/spi_wait.c` | SITL-linked | Bounded wait logic compiles; no live host transport state exists yet. |
| `src/tof-matrix/drivers/i2c_expander.c` | ARM-build only | Hardware-only GPIO/I²C address-selection driver; excluded from SITL build. |
| `src/tof-matrix/drivers/vl53l5cx_api.c` | ARM-build only | Physical sensor API; excluded from SITL build; simulator must preserve public matrix units/range/invalid-cell semantics. |
| `src/tof-matrix/i2c_platform.c` | ARM-build only | Hardware-only I²C implementation; excluded from SITL build. |
| `src/tof-matrix/tof_matrix.c` | ARM-build only | Physical sensor orchestrator; replaced by fail-closed SITL provider until Gazebo matrix transport exists. |
| `src/tof-matrix/tof_matrix.h` | ARM-build only | Shared public API/data contract; do not fork struct layout between targets. |
| `src/tof_daq.c` | SITL-linked | Acquisition/freshness task compiles against the same public API and remains blocked until matrix init/data succeed. |
| `src/tof_freshness.c` | SITL-linked | Freshness logic compiles unchanged and retains timeout/fault behavior. |
| `src/uwb_collision_det.c` | SITL-linked | Nano policy compiles; timestamped simulated neighbor/range input is absent. |
| `src/uwb_ranging.c` | SITL-linked | Token/pose logic compiles against fail-closed UWB/SPI providers; no simulated ranging occurs. |
| `src/uwb_token_handoff.c` | SITL-linked | Compiles under the real FreeRTOS queue/task API; runtime task behavior is not yet observed. |
| `src/uwb_token_lease.c` | SITL-linked | Compiles unchanged; runtime stale/lost-token behavior remains unvalidated. |

The classifier is guarded by
[`tests/test_stm32_sitl_inventory.py`](../tests/test_stm32_sitl_inventory.py).
The opt-in CrazySim `cf2` target has successfully linked this source manifest
against the pinned POSIX FreeRTOS port. The binary waits for the Gazebo socket
connection before system startup; a four-second isolated boot probe timed out
at that wait, so it did not observe `appInit()` or `appMain()`. The target
consumes the checked-in
[`nano_sources.cmake`](../cslam-stm32-app/sitl/nano_sources.cmake) manifest.
The Python test expands nested Kbuild directory entries and asserts the
manifest equals the complete app source set minus `app_main.c` (listed
separately in the CMake target) and the four physical ToF/I²C units replaced
by `tof_matrix_sitl.c`.

## Host-portability boundary

## Hardware-image build baseline (2026-10-19)

The owned STM32 app and pinned Crazyflie firmware still build together for the
normal `cf2` target on macOS with the installed Arm GNU Toolchain 15.3.1:

```sh
cd cslam-stm32-app
make -j8
```

The earlier baseline reports Flash `363,556 / 1,032,192` bytes (35%), RAM
`115,988 / 131,072` bytes (88%, 15,084 free), and CCM `47,740 / 65,536` bytes
(73%, 17,796 free). The build emits existing upstream/toolchain warnings,
including unused UWB library variables and newlib stubs for `_read`, `_write`,
and `_lseek`; it exits successfully. This proves the regular embedded image
still compiles, not that it was flashed, flown, or ported to Linux SITL.

After fail-closed UWB initialization was moved ahead of task creation, the
2026-10-20 rebuild reports Flash `363,716 / 1,032,192` bytes (35%), unchanged
RAM `115,988 / 131,072` bytes (88%) and CCM `47,740 / 65,536` bytes (73%).
The opt-in CrazySim app target and Nano-disabled default target both compile
and link in the Lima Ubuntu guest. The app target is source-level POSIX
FreeRTOS execution, and its ToF, UWB and GAP9 SPI providers deliberately fail
closed. The boot probe stopped at CrazySim's Gazebo socket wait; it did not
reach Nano app startup. No Nano-controlled flight or mapping result is claimed.

The host C suite currently compiles and exercises pure timeout, heartbeat,
token-handoff/lease, bounded-batch, ToF freshness, packet encoding, SPI wait,
actuation-limit, and selected flight-task behavior using test stubs. These
tests validate those units only; they do not start appMain() or provide an
RTOS scheduler, radio, estimator, sensor, UWB transceiver, SPI peripheral, or
motor mixer.

The exact owned translation units currently compiled by the host suite are
`src/flight_commander.c`, `src/operator_heartbeat.c`,
`src/exploration/limit_actuation.c`, `src/tof_freshness.c`,
`src/mission_timeout.c`, `src/uwb_token_handoff.c`,
`src/uwb_token_lease.c`, `src/bounded_batch.c`, `src/spi_comm/spi_wait.c`,
and `src/spi_comm/spi_comm.c`. The suite uses purpose-built test stubs, not
the FreeRTOS POSIX port. No complete Nano app translation unit has yet been
verified in a host FreeRTOS SITL build; portability of these selected units
must not be confused with proof that `appMain()` or its task graph runs there.

The likely portable control path is existing Nano logic in flight_commander.c,
exploration/, line_detector.c, limit_actuation.c, and the pure policy modules
above. Board seams are concentrated in tof_matrix.c/i2c_platform.c,
spi_comm.c/deck GPIO, uwb_ranging.c plus the UWB submodule,
radio_comm.c/CRTP, and the Crazyflie estimator/commander/motor pipeline. A
Linux-port integration needs simulator-backed implementations for each seam
it executes; every stub must be named in the run report.

## Execution and flight evidence markers

The existing app emits a ToF-ready message from `mission_task()` and `Take-off:`
from `flight_commander.c`. Those strings
are useful debugging clues, but they are not a sufficient SITL contract: no
existing marker identifies each task, timestamps simulator input, or proves
that motor output reached Gazebo. A SITL adapter must add structured markers
with at least these fields before flight is claimed:

- `NCS_SITL_TASK_READY task=<name>` for each required Nano task, including
  `mission_task`, `tof_task`, `flight_task`, and `uwb_ranging_task`.
- `NCS_SITL_SENSOR source=<adapter> seq=<n> sim_ns=<n> fw_tick=<n>` when a
  fresh simulator sample is accepted by the corresponding owned task.
- `NCS_SITL_MOTOR seq=<n> sim_ns=<n> fw_tick=<n> m1=<n> m2=<n> m3=<n> m4=<n>`
  at the firmware-to-simulator actuator boundary, after range and finite-value
  validation.
- `NCS_SITL_MISSION phase=<takeoff|hover|land|done|fault>` with simulator
  time and the reason for terminal phases.

The run evidence must correlate those markers with Gazebo pose/velocity,
include a disconnected-actuator negative control, and show nonzero exit status
for stale sensor or malformed motor input. Markers alone are not proof: the
negative control and output-to-state causal link are required.

## Evidence-backed SITL direction

Architecture decision: [ADR-0001](decisions/ADR-0001-nano-stm32-gazebo-sitl.md)
selects CrazySim's Linux `cf2` / FreeRTOS-POSIX target with the native macOS
Gazebo 8.14 plugin. The corrected trace-enabled firmware build and plugin
exchange simulated IMU data and firmware motor packets, but those runs use
upstream Crazyflie firmware only. They do not include this repository's app
tasks or prove Nano firmware flight.

The next port is source-level: compile and start the owned `cslam-stm32-app`
task graph in the pinned CrazySim `cf2` process. Preserve the app logic and
name every simulator-backed device adapter. The normal `cf2` ARM image remains
the hardware build; Linux SITL is not an instruction-equivalent execution of
the STM32 binary. The selected target and comparisons with CrazyflieTHI
`sim_cf2` and `crazyswarm2_gazebo` are recorded in ADR-0001. Do not call the
existing ROS-controlled flight path Nano firmware validation.

## Additional ToF source

The existing VL53L5CX matrix implementation is not a generic ToF driver. The
requested VL53L9CX has a distinct data shape and output interfaces; its
electrical route, data rate, official driver, and map integration are tracked
in `ncs-fea.5.9` and its children. The first firmware flight can validate the
flight loop with a simulated source matching the existing VL53L5CX contract,
but that does not close VL53L9CX integration or dual-sensor mapping.
