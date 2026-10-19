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
| tof_task | cslam-stm32-app/src/tof_daq.c | Static task, priority 3, stack allocation 500 | Initializes and polls four 8x8 ToF matrices sequentially, tracks per-sensor freshness, and signals acquisition semaphores. Depends on VL53L5CX, I2C, sensor power/address selection, and the deck platform. |
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

## Host-portability boundary

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

The local crazyflie-firmware checkout is pinned by this repository; it
contains no sim_cf2 platform or SITL launch scripts. The current Crazyswarm2
Gazebo overlay is therefore not a firmware SITL. The upstream
CrazyflieTHI/sim_cf2 reference runs modified Crazyflie firmware with the
FreeRTOS Linux port and documents ROS 2 Humble/Gazebo 11 on Ubuntu 22.04.
gtfactslab/CrazySim also runs a modified Linux SITL firmware with Gazebo
Garden or MuJoCo, but its README warns that its firmware fork is for SITL, not
hardware. Neither reference is a drop-in for the installed native macOS ROS 2
Jazzy/Gazebo Harmonic stack. Treat them as architecture references until their
source revisions, build, motor/sensor interface, and compatibility are
reproduced.

The next experiment should reproduce the smallest one-vehicle firmware loop
from a pinned reference on its documented Linux toolchain, then compare its
Gazebo-version and host-OS requirements with the local native stack. Only
after that should the project choose between adapting that firmware interface,
migrating the simulator environment, or implementing a narrower native
FreeRTOS/device port. Do not call the existing ROS-controlled flight firmware
validation.
