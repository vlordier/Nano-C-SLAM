# CrazySim SITL reference reproduction status

## Pinned source examined

The candidate is `gtfactslab/CrazySim` at `3ec8b55da4bff887da542a9f314da825460e65be`.
Its recursive submodule pins are:

- modified `llanesc/crazyflie-firmware` at
  `aa6571dc465f06f7d1f9aaf7b0b861fbcd1b3d67`;
- `llanesc/crazyflie-simulation` at
  `89d8cf79fb722bf4bd7e363ade7d90d6c45d6d5d`;
- Gazebo MuJoCo drone models at
  `512c864ffa55081ef4e80fadde2e861f22f45943`.

This is a reference implementation, not the firmware in this repository. Its
README warns that its modified firmware is for SITL and must not be used on
hardware. Its `sitl_make/CMakeLists.txt` builds a modified Crazyflie image with
the FreeRTOS POSIX port and a `power_distribution_sitl.c` backend. The Gazebo
plugin handles UDP motor and sensor/state exchange. The README documents
Gazebo Garden and Ubuntu; the plugin CMake file also explicitly recognizes
Gazebo 8/Harmonic. The firmware is not a drop-in build of this repository's
`crazyflie-firmware` submodule or Nano app.

## macOS checks performed

The host is macOS 26.6.2 arm64, Apple Clang 21.0.0, Gazebo Sim 8.15.0
(Harmonic), ROS 2 Jazzy, CMake 4.4.4. No Lima instance exists; `multipass list`
cannot connect to its service. These checks used a clean shallow clone plus
recursive submodules under `/tmp/ncs-crazysim-reference`; no reference files
were changed.

Firmware configure succeeds:

```sh
cmake -S crazyflie-firmware/sitl_make -B crazyflie-firmware/sitl_make/build \
  -DCMAKE_BUILD_TYPE=Release
```

The first build attempt stopped because the upstream CMake custom command calls
`python`, which is absent from the default PATH. Exposing the repository's
locked uv interpreter allowed the build to proceed:

```sh
UVPY=$(uv run --locked --project /Users/vincent/Work/Nano-C-SLAM which python)
PATH="$(dirname "$UVPY"):$PATH" \
  cmake --build crazyflie-firmware/sitl_make/build --target cf2 -j 8
```

It then exited 2. The first compiler errors are in
`crazyflie-firmware/src/hal/src/radiolink_sitl.c:35` and `pm_sitl.c:9`:
`section` attributes such as `.log.radio` are rejected because Mach-O requires
a segment and section name. The candidate's linker flags also assume GNU ELF
(`--gc-sections` and `log_param_linker.ld`). This is a real host-format/OS
boundary, not a missing Python dependency. No firmware executable was
produced, so no CrazySim runtime, sensor exchange, actuator exchange, or flight
was reproduced.

The Gazebo plugin builds natively after passing the Gazebo and Qt Homebrew
prefixes and putting the matching Abseil headers ahead of the stale global
Homebrew include path:

```sh
cmake -S crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim \
  -B /tmp/ncs-crazysim-gz-build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH='/opt/homebrew/opt/gz-plugin2;/opt/homebrew/opt/gz-sim8;/opt/homebrew/opt/gz-msgs10;/opt/homebrew/opt/gz-transport13;/opt/homebrew/opt/gz-cmake3;/opt/homebrew/opt/gz-common5;/opt/homebrew/opt/gz-math7;/opt/homebrew/opt/gz-utils2;/opt/homebrew/opt/qt@5' \
  -DCMAKE_CXX_FLAGS='-I/opt/homebrew/opt/abseil/include'
cmake --build /tmp/ncs-crazysim-gz-build --clean-first -j 8
```

The result is an arm64 Mach-O `libgz_crazysim_plugin.dylib`. Without the
Abseil include override, the build reaches link and fails with an undefined
protobuf symbol using Abseil namespace `lts_20250512`; this Homebrew setup has
new protobuf/Abseil libraries plus a stale global Abseil header symlink. Adding
the ROS Jazzy protobuf include path is the wrong fix and causes Gazebo message
headers to fail compilation.

The plugin was loaded by Gazebo Sim 8.15.0 in the pinned CrazySim world after
rendering the model SDF from its template. The model was accepted by
`/world/crazysim_default/create`, and live `/cf_0/imu` and `/cf_0/odom` messages
were observed. This validates plugin loading and sensor-topic production only;
no firmware executable was running and no actuator-driven flight occurred.
Gazebo's default DART plugin has a stale Homebrew dependency on
`libdart-collision-bullet.6.19.dylib`, while 6.20.1 is installed. Launching
with the installed `gz-physics-bullet-plugin` avoided that DART load error and
allowed the model to fall and contact the ground, proving a physics backend
was active for the later firmware probe.

The reference plugin also has a shutdown/reconnect defect. Its destructor
joins threads whose `recvfrom()` calls can block indefinitely, but does not
close or shut down their sockets to wake them. One Ctrl-C test server exited
only after a one-byte `0xF3` datagram was sent to port 19950; a later run had
to be force-killed. Separately, after `socketInit` becomes true the receiver
stores one firmware remote address and never accepts a new handshake until
the plugin process restarts. Add bounded receives or explicit socket wakeup,
reconnect handling, and regression tests before relying on repeated runs.

## What the source proves, and what it does not

The reference's `power_distribution_sitl.c::motorsSetRatio()` receives four
firmware motor ratios and sends the packed motor command over CRTP/UDP. The
CrazySim Gazebo plugin decodes that command and publishes Gazebo actuator
velocities. Its sensor path returns simulated state/IMU data to the SITL
firmware. This is the right causal architecture to test against the current
Gazebo Sim vehicle; a ROS `/cmd_vel` controller is not equivalent.

The source-level interface is promising for keeping Gazebo Harmonic on macOS
and building the SITL firmware in a Linux environment. The plugin build and
load are now proven locally, while cross-host UDP, physics backend health,
sensor units/clocks, actuator causality, and the one-drone closed loop remain
untested. More importantly, the referenced firmware's app is upstream
Crazyflie SITL firmware, not Nano-C-SLAM. Nano's own task graph and hardware
seams still need to be ported behind explicit simulated-device adapters.

## Reproduction state

**Partially reproduced; reference flight failed.** A Lima Ubuntu 24.04.5
ARM64 VM was created with 4 CPUs, 6 GiB RAM, and a 25 GiB disk using
`--containerd=none`; only the firmware build/runtime uses Linux. Gazebo remains
native on macOS. The pinned reference firmware was built there with:

```sh
cmake -S /tmp/ncs-crazysim/crazyflie-firmware/sitl_make \
  -B /tmp/ncs-crazysim/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/ncs-crazysim/build --target cf2 -j 4
```

This produced an ELF AArch64 `cf2` binary. The build exits successfully but
emits upstream warnings, including implicit EEPROM declarations, a
`CONTROLLER` macro redefinition, and `getMotorRatio()` reaching the end of a
non-void function.

With the Gazebo plugin and model running, the firmware logged `Connection
established with gazebo`, `Software-in-the-Loop Simulator is up and running!`,
and `SYS: Self test passed!` after the plugin answered its UDP handshake. That
proves a guest-to-host request and host-to-guest reply. A high-level takeoff
CRTP packet sent through the plugin did not produce flight: all four
`/cf_0/command/motor_speed` values remained zero and odometry showed the model
fall from its 0.5 m spawn to about 0.015 m on the ground. This is a failed
reference flight, not a partial success.

Bead `ncs-fea.5.2` remains open until the reference itself flies and lands,
with logs and hashes archived. First determine why it remains at
`Wait for sensor calibration` and emits no motor output despite a successful
handshake and live Gazebo sensor topics. Then port Nano's owned tasks and
device seams; the reference app is upstream Crazyflie firmware, not
Nano-C-SLAM.

## Follow-up packet-boundary trace (2026-10-19)

The exact pinned CrazySim repository and its firmware submodule were re-cloned
into the Lima guest and `cf2` rebuilt. A bounded host UDP probe completed the
handshake, and a guest-side packet capture saw approximately 2,500
14-byte IMU-shaped datagrams arrive. This proves host-to-guest UDP delivery,
not consumption by the sensor task. Temporary instrumentation in the guest
clone showed `poll()` returning `EINTR`; the observed loop must retry that
condition rather than treat it as readiness. Changing task priority to make
the socket task run allowed startup to continue but did not prove calibration
or actuator output. A 500-Hz injected stream filled the 2,000-entry socket
queue and hit its assertion at `socketlink.c:104`.

The trace exposed a confound in the first probe: the plugin handshake occurs
before system self-tests release `systemWaitStart()`, while the sensor task
blocks there. Injecting a high-rate stationary IMU stream during that interval
can fill the queue before sensor consumption begins. The host probe now
supports `--ready-log <firmware-log>` to wait for `Self test passed!` before
injecting its synthetic IMU stream. That mode is a transport/calibration
diagnostic only. A separate run with native Gazebo IMU callbacks must show
sensor-task receive counts, calibration, firmware motor packets and resulting
Gazebo motion before the reference flight can pass. No such run has passed.

The ready log is mandatory so a new probe run cannot silently repeat the
pre-self-test injection. Example once firmware stdout is being captured:

```sh
uv run --locked python scripts/crazysim_udp_probe.py \
  --ready-log /tmp/cf2-firmware.log --imu-hz 100 --imu-seconds 5
```

The 2026-10-19 ready-gated retest sent 2,994 stationary IMU datagrams at
500 Hz after the `Self test passed!` marker, then sent the high-level takeoff
packet. The firmware still logged `Wait for sensor calibration`; the probe
received zero motor packets. Raising the socket task from
`USBLINK_TASK_PRI-1` to `USBLINK_TASK_PRI` and yielding one tick per poll did
not change that result. Guest packet capture proves datagrams reached the VM,
but temporary first-packet logging in both `socketlinkTask` and `sensorsTask`
did not appear in captured firmware output. Therefore this run does not prove
that the socket task consumed the packets. The full CrazySim aggregate build
also attempts to build the Gazebo plugin in the Linux guest and fails because
`gz-plugin2` is unavailable there; this does not invalidate the separate
`cf2` target build.

## Sensor routing and actuator-output retest (2026-10-19)

Further instrumentation isolated two scheduling boundaries. With the default
priorities, `socketlinkTask` did not consume UDP datagrams. Raising that task
alone made it receive valid 14-byte packets (`header=0x9c`, IMU type `0`), but
the CRTP delivery queue filled while its priority-2 router and priority-4
sensor task were starved. A diagnostic VM-only build that raised the socket,
CRTP RX, and sensor tasks to priority 5 showed the complete path: the CRTP
router received port-9 packets, the sensor task created its queue and consumed
them, `GYro Bias found` appeared, and the stabilizer loop started. These
priority edits and trace prints were confined to `/tmp/ncs-crazysim`; they are
not a proposed production configuration.

The UDP probe had a separate test defect: it stopped sending IMU immediately
before takeoff. It now keeps sensor input active through the takeoff and
post-flight interval, and treats motor output saturated at `65535` as a failed
probe. Under the diagnostic priority configuration and constant synthetic
1-g IMU, one run received 5,417 motor packets, but the maximum ratio was
`65535`; firmware logs also reported estimator state reset and off-rate
warnings. This proves actuator packets can be emitted, not a valid flight.
The input does not include Gazebo's changing IMU, gravity orientation, or
vehicle dynamics. No actuator-driven Gazebo rise/hover/landing has passed, and
Nano-C-SLAM's STM32 application still has not run in this reference firmware.
The earlier ineffective socket patch was removed rather than retained as a
claimed fix.

The native macOS model was then isolated from the firmware. Gazebo's own
upstream `quadcopter.sdf` was launched with the same source-built Gazebo 8
stack. `gz topic -i` reported four `gz.msgs.Actuators` subscribers on
`/X3/gazebo/command/motor_speed`; a published `[700, 700, 700, 700]` command
was echoed on that topic and the rotor link poses changed, confirming the
motor command reached the motor systems. However, `/world/quadcopter/pose/info`
kept the X3 root at `z=0.055 m` throughout the capture. The same no-rise result
occurred with the Crazyflie model under direct `[3052, 3052, 3052, 3052]`
commands (root `z` remained `0.015 m`). A separate run of the upstream X3
world under default DART also stayed at `z=0.055 m`, although rotor response
was not observed in that run. These results make the failure independent of
firmware scheduling and point to the Gazebo motor-force/physics/model path.
They do not establish that Gazebo upstream is broken: the source-built plugin,
selected engine, and model-force application still need a controlled
calibration test. Context7 documentation
lookup was unavailable because its monthly quota was exhausted; local source
inspection and the upstream Gazebo example were used for this diagnosis.

Follow-up source inspection identified the backend capability mismatch. In the
local `gz-sim` `Physics.cc`, an `ExternalWorldWrenchCmd` is skipped with the
diagnostic `Wrench will be ignored` when the selected backend lacks
`AddLinkExternalForceTorque`. The local `gz-physics` Bullet Featherstone
backend implements that feature; the regular Bullet backend does not. Repeating
the official X3 test with `gz-physics-bullet-featherstone-plugin` and the same
`[700, 700, 700, 700]` command raised the root from `z=0.055 m` to `z=2.3558 m`
in roughly three seconds. This confirms thrust works in the local Gazebo stack
with Featherstone. It does not validate the Crazyflie model or firmware flight;
those are the next tests. The integrated Gazebo launcher now selects this
force-capable backend by default, while `NANO_C_SLAM_GZ_PHYSICS_ENGINE` can
override it.

## Controlled CrazySim command-path probe (2026-10-19)

A temporary raised platform was added to the pinned CrazySim world so the
firmware would remain upright during gyro calibration. With a diagnostic
`cf2` build and the Gazebo sensor/plugin path active, the plugin logged
stationary IMU input (`acc=(0,0,9.80665)`, `gyro=(0,0,0)`). A CRTP high-level
takeoff datagram was observed by the plugin and diagnostic firmware. Decoding
the captured bytes
`8c 07 00 33 33 13 40 00 00 00 00 01 00 00 40 40` with the CrazySim
`<BBBff?f` layout gives `TAKEOFF_2`, target altitude `2.3 m`, current-yaw mode,
and duration `3.0 s`. This target is above the platform's starting pose
(approximately `2.04 m`). The plugin's `TRACE_MOTOR_NONZERO` is emitted only
inside `handleMotorsMessage()`, which is called for a firmware UDP datagram
whose CRTP port is 9 and whose payload is the four-motor response. Therefore
the observed `36000` values are firmware-originated actuator packets, not a
direct-motor command from the cflib socket. The separate one-byte `ff` packet
is a cflib ping echo and does not call the motor handler.

This establishes an initial firmware-commanded rise in the diagnostic
CrazySim build, but does **not** pass the flight acceptance criteria. The
captured trajectory lacks a stable finite hover; earlier attempts reported
estimator resets, stabilizer-rate warnings, non-finite angular odometry, and
a crash during landing. Thus the reference flight is **partial and unstable**,
not a successful takeoff/hover/land. The platform, custom instrumentation,
and firmware priority changes exist only under `/tmp`; no vendor/submodule
changes are maintained.

The next valid test must start from a clean server with a unique Gazebo
partition, record synchronized `/cf_1/odom`, `/cf_1/imu`, plugin motor topic,
firmware controller/setpoint output, and packet source; reject any non-finite
odometry or motor value; and demonstrate stable hold plus bounded landing.
This still qualifies only upstream CrazySim firmware. Nano-owned STM32 tasks
and mapping remain unexecuted.

The temporary debug edits live only in `/tmp/ncs-crazysim` in the Lima guest;
they are not a maintained patch and were not copied into the submodule. Bead
`ncs-fea.5.8.1` now isolates readiness gating and bounded-queue diagnosis.
