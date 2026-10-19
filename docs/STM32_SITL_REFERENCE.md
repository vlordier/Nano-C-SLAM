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

**Current status: diagnostic upstream CrazySim firmware can rise under Gazebo
motor output, but it does not yet pass a stable flight.** Older attempts below
that reported no motor output are retained as historical failures; the
single-process live retest near the end of this document supersedes them.

A Lima Ubuntu 24.04.5
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

In the first run, with the Gazebo plugin and model running, the firmware logged `Connection
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

The first captured run was later found to have two `cf2` processes connected
to one plugin. Do not use its trajectory as single-firmware evidence. A fresh
single-process repetition and priority experiments are recorded below.

## Single-process firmware-driven rise and stability failure (2026-10-19)

A clean native Gazebo server on Featherstone, a fresh `crazyflie_1` model on
the raised calibration platform, and exactly one Lima `cf2` process were used.
The firmware logged a successful self-test, received live Gazebo IMU over the
socket/CRTP/sensor tasks, found gyro bias, started the stabilizer, and accepted
the captured TAKEOFF_2 command. Captured 200-Hz odometry has 2,391 pose
samples: z begins at `2.03999 m`, rises to `2.54461 m`, then returns to
`2.03999 m`. The plugin motor topic has 47,824 components, 12,250 nonzero,
with maximum `2774.98 rad/s`. This proves an initial firmware-commanded rise.

The same run is a flight failure: angular twist first becomes non-finite at
about `2.855 s`, firmware reports stabilizer rate `823` vs expected `1000`,
Kalman prediction rate `98` vs expected `100`, and supervisor reports
`Crashed, recovery required`. The aircraft returns to the platform before the
scheduled land command. It does not demonstrate stable hover or commanded
landing. This result belongs to a diagnostic build whose temporary task
priorities and trace edits are only in `/tmp/ncs-crazysim`.

Priority tests isolate a remaining scheduling problem but are not fixes. Giving
socket, CRTP RX and sensor tasks maximum priority permits sensor calibration
but starves the stabilizer. Lowering all three to priority 4 (below the
stabilizer's priority 5) allows handshake but stalls sensor delivery before
calibration. Raising them to priority 5 restores calibration, yet the
stabilizer still reports about `840 Hz`; one command capture had no accepted
high-level packet and produced no motor output. `gz topic -f` measured odometry
near 200 Hz and Gazebo stats near 10 Hz; these measurements do not establish
the simulation's 1-kHz sensor scheduling or explain the missing stabilizer
ticks. Keep `ncs-fea.5.8.2` open to identify a valid task/clock configuration.

The passing subset is therefore limited to hardware-free upstream diagnostic
firmware readiness, sensor ingestion, high-level command reception, firmware
motor packet emission, and initial Gazebo rise. Stable flight, finite state,
hover, land, disconnected-actuator control, Nano STM32 task execution, and
Nano mapping remain unproven.

The temporary debug edits live only in `/tmp/ncs-crazysim` in the Lima guest;
they are not a maintained patch and were not copied into the submodule. Bead
`ncs-fea.5.8.1` now isolates readiness gating and bounded-queue diagnosis.

## Isolated 500-Hz retest and invalidated duplicate-process run (2026-10-19)

A later 500-Hz experiment was first treated as a single-firmware run, but a
process audit found two simultaneous guest processes (`cf2` PIDs 25248 and
26477) using the same UDP endpoint. Its trajectory is invalid evidence and
must not be combined with the earlier single-process run. The flight gate now
requires an explicit process count for both positive and negative captures,
and exactly one `TRACE_CRTP_RX_STARTED` marker in each combined firmware log.
The process-count argument is necessary because multiple `cf2` processes may
write to separate outputs.

The test was restarted with a fresh Gazebo plugin/world and exactly one
instrumented `cf2`. The model was loaded from the world SDF because the
cross-process `/world/.../create` request timed out; the `/cf_1/imu`,
`/cf_1/odom`, and `/cf_1/command/motor_speed` topics were then observed. The
firmware reported successful self-test, live sensor queue delivery, gyro bias
calibration, stabilizer start, and `SUP: Ready to fly`. A TAKEOFF_2 and a
LAND_2 command were sent through the plugin's cflib UDP port. Gazebo captured
5,750 odometry samples over 28.89 simulation seconds, 14,428 IMU samples, and
115,576 motor velocity values. Motor output was nonzero and finite, with a
maximum of 2,774.98 rad/s. The model rose from 2.040 m but reached 10.099 m,
and ended at 9.352 m. Finite angular speed peaked at 606.94 rad/s and was
31.39 rad/s at the final sample. Only 5,135 of 5,750 odometry samples were
fully finite; the first non-finite angular velocity appeared at 26.17 s. The
longest interval meeting the configured
2.40-m/upright/low-angular-rate hover envelope was 0.69 s. This is a failed
flight, not a takeoff/hover/landing result. The bounded run used `timeout`, so
its process exit status also fails the gate.

The same build changed `RATE_MAIN_LOOP` to 500 Hz, but the upstream
`stabilizer.c` rate supervisor still checks a fixed 997–1003 events per
second. The log reported 542 events; that warning cannot be interpreted as a
500-Hz pass/fail until the check is derived from the compiled loop rate and
the measurement clock is aligned. Kalman prediction counts of 86–98 against
the configured 100 Hz target also remain unexplained. Do not clear these
warnings by simply widening limits. Bead `ncs-fea.5.8.2.1` tracks the explicit
rate-measurement fix and its negative test. The 500-Hz main-loop override,
task-priority, filter, and trace edits remain in the Lima guest diagnostic
clone; no submodule pointer in the Nano checkout changed.

## POSIX tick-width correction and repeat flight (2026-10-19)

The 500-Hz reference firmware build also warned that `vTaskDelayUntil()` was
passed a `uint32_t *` where the POSIX FreeRTOS port expects `TickType_t *`
(64-bit `unsigned long`). `stabilizerTask()` kept `lastWakeTime` as `uint32_t`,
so the call could write eight bytes into a four-byte stack variable while
waiting for sensor calibration. The local CrazySim firmware branch
`codex/sitl-rate-supervisor` now has focused commits `78f4021e` (derive and
report the stabilizer supervisor frequency from `RATE_MAIN_LOOP`) and
`7db68dbd` (use `TickType_t` for `lastWakeTime`). The `cf2` target rebuilt;
the incompatible-pointer warning disappeared. These commits exist only in the
Lima guest firmware worktree and have not been pushed or added to Nano's
submodule.

A fresh single-process run with those two fixes still failed. It logged
`ESTKALMAN: State out of bounds, resetting`, accepted TAKEOFF_2 and LAND_2,
and emitted finite, nonzero motor commands. Captures contain 4,841 odometry
samples over 24.32 simulation seconds, 12,140 IMU samples, and 97,300 motor
velocity values. Altitude ranged from 2.040 m to 6.202 m and ended at 3.813 m;
the first non-finite angular velocity was at 14.695 s. Of 4,841 odometry
samples, 3,395 were fully finite; finite angular speed peaked at 612.70 rad/s,
and the stable-hover envelope lasted only 0.195 s. The patched supervisor now
reports `547 Hz; expected 500 Hz`; Kalman prediction remained 80–98 Hz against
100 Hz. The run was timeout-terminated. This removes one concrete stack-width
hazard and makes the rate diagnostic accurate, but it does not fix the
remaining scheduler/estimator/flight failure. Bead `ncs-fea.5.8.2.2` tracks
the state/attitude trace. No stable flight or Nano firmware execution is
claimed.

An idle diagnostic trace narrowed the startup reset further. Temporary
instrumentation in the firmware localization handler observed live external
Gazebo poses over CRTP (`z=2.040 m`, identity quaternion). At the first
Kalman bounds failure, the internal state was approximately `z=-20.293 m`,
`vz=-19.954 m/s`, while the latest accelerometer value was `(0,0,1.024) g`
and the gravity-frame term was `Rzz=1.000`. This happened before any takeoff
command, so motor dynamics cannot explain that initial divergence. The
temporary trace edits were reverted and `cf2` rebuilt successfully. This
localizes the next investigation to estimator initialization/prediction and
measurement timing/conversion; it does not yet identify which is faulty.

## Empty-subsample startup prediction fix (2026-10-19)

Instrumentation in `kalmanCorePredict()` identified the immediate cause of the
pre-takeoff reset. Its first call used `nowMs=1981`, `lastPredictionMs=12`
(`dt=1.969 s`), zero acceleration and gyro subsamples, and `quadIsFlying=false`.
The following predictions used gravity-consistent acceleration near
`(0, 0, 9.8052) m/s^2`. `estimator_kalman.c` had its existing guard for nonempty
acceleration and gyro subsamplers commented out, so the zero-initialized
subsamples were integrated over the sensor calibration delay.

The local Crazyflie firmware branch now restores the guard and explains why
predictions must wait for both sensor streams. Commit `70eab0ba` rebuilt the
`cf2` target successfully. A fresh single-process run reached `Ready to fly`
without an estimator bounds reset; TAKEOFF_2 and LAND_2 reached firmware, and
Gazebo recorded 85,000 motor values, 24,960 nonzero, with maximum
`2056.69 rad/s`. Odometry recorded 4,250 poses and rose from `2.040 m` to
`2.685 m`, confirming a firmware-driven initial climb. This first retest ended
at `2.659 m` after the landing command and did not prove landing.

A subsequent bounded positive-control run used one `cf2` process, two identical
TAKEOFF_2 packets, and one LAND_2 packet. It recorded 3,324 odometry poses from
`2.040 m` to `2.685 m` and back to `2.040 m`, 215,552 finite motor values
(84,244 nonzero; maximum `2,426.68 rad/s`), and no Kalman out-of-bounds reset.
The longest interval in the configured upright/low-angular-rate hover envelope
was `15.495 s`. After LAND_2, odometry returned to `2.119 m` (within 0.10 m of
the platform). This demonstrates reference-firmware
takeoff, sustained hover, and physical return to the platform, but it is not a
passing flight-gate capture: the firmware emitted 47 rate warnings, reported
`Landing timeout, disarming`, and the run did not include the disconnected-
actuator negative control. The duplicate takeoff packet also means this was a
manual positive-control run, not the checker fixture. Captures are
`/tmp/ncs-kalman-cycle-{odom,motors}.log` on the host and
`/tmp/ncs-kalman-cycle-fw.log` in the Lima guest.

The two focused firmware fixes are published on
[`vlordier/crazyflie-firmware` branch `codex/sitl-rate-supervisor`](https://github.com/vlordier/crazyflie-firmware/tree/codex/sitl-rate-supervisor):
`516ee762` derives the stabilizer supervisor range from `RATE_MAIN_LOOP` and
uses `TickType_t` for `lastWakeTime`; `c0df4f62` skips Kalman prediction until
both sensor streams have supplied samples. The Lima diagnostic clone has
additional local CrazySim integration changes and its `70eab0ba` commit; those
were not pushed. Neither branch changes Nano's submodule pointer.
`ncs-fea.5.8.2.1` remains open for clock-aligned rate validation and
`ncs-fea.5.8.2.2` remains open for a clean checker-passing flight. No Nano
application or mapping validation is claimed.

## Clock observation and lower-overhead retest (2026-10-19)

Two additional one-process reference runs tested the rate-measurement question.
Both used the Lima-built diagnostic `cf2` from firmware commit
`70eab0ba1520a4cf0fe679ed8c391e05fac12e5f`, with local SITL changes in
`config.h`, `sensors_sitl.c`, `socketlink.c`, and CRTP trace points. The binary
SHA-256 was
`116573d136471c417c2266c88445bdf8cbbd22e4f3ce3cc2c50f5153ad9c96f8`. These
are temporary diagnostic builds, not the Nano STM32 application or a
reproducible checked-in flight package.

The first run subscribed to odometry, IMU, motors, and `/stats` for the same
capture. It recorded 12,755 odometry samples over 64.09 simulation seconds,
31,954 IMU samples (2 ms median timestamp spacing, 4 ms maximum gap), and
256,388 motor components. The model rose from 2.040 m to 3.044 m and returned
to 2.040 m, but spent only 0.26 s in the 2.64 m hover envelope. The firmware
console showed a 464 Hz stabilizer interval and Kalman prediction warnings
between 80 and 98 Hz. The `/stats` echo itself received 64,004 messages and
created a 10.8 MB text log; its reported real-time factor varied from 0.011 to
8.219, with a median of 1.000. Simulation advanced 64.07 s while wall time
advanced about 92.44 s. Because the high-rate stats and IMU text subscribers
were active during this run, this RTF is observer-influenced and cannot be
used to attribute the firmware warnings.

The second run removed `/stats` and IMU text echo, retaining only odometry and
motor capture. It recorded 18,515 odometry samples from 2.225 s to 95.260 s;
17,619 were finite. Altitude ranged from 2.039 m to 2.915 m and ended at
2.040 m. The configured 2.40 m stable-hover envelope lasted 0.32 s. All
372,148 motor components were finite, 87,876 were nonzero, and the maximum
was 2,774.984 rad/s. The firmware file contained a 536 Hz stabilizer warning
against 500 Hz and 50 Kalman-rate warnings (80–98 Hz), with no captured
out-of-bounds reset or landing-timeout marker. TAKEOFF_2 and LAND_2 were sent
by the host runner, but the firmware log ended at exactly 4 KiB because stdout
was fully buffered and the process was terminated; the command-receipt markers
were therefore not preserved. This attempt fails the flight gate on finite
telemetry, hover duration, and complete command evidence. Its capture files
are temporary files under `/tmp/ncs-livepass{1,2}-*` on the development host;
they are not committed evidence artifacts.

The lower-overhead run still reports loop-rate warnings, so the first run's
monitoring overhead is not a complete explanation. The captures do not
timestamp firmware loop events against the Gazebo clock on one shared clock,
and the second run did not record `/stats`; the reason for the remaining
rate mismatch is therefore unresolved. The next run must line-buffer the
firmware output (for example, `stdbuf -oL`), record process exit status and
count, and use low-overhead plugin-side counters for simulation time, IMU
callbacks, and motor packets instead of high-rate text echo. Do not change
rate tolerances based on these two failed runs. The CrazySim plugin still
blocks during shutdown in this setup, so the run procedure must also make its
cleanup explicit and record any forced termination.

## IMU-trigger correction and flight retest (2026-10-19)

The CrazySim sensor adapter gave the `dataReady` semaphore after every packet
on `CRTP_PORT_SETPOINT_SIM`. The stabilizer waits on that semaphore and uses
the most recent gyro/accelerometer values, so a simulated 50-Hz barometer
packet could trigger another controller iteration with stale IMU data. The
reference model publishes IMU at 500 Hz and barometer at 50 Hz. Commit
[`743ebe99`](https://github.com/vlordier/crazyflie-firmware/commit/743ebe99)
on branch `codex/sitl-imu-wakeup` now gives the semaphore only after
`SENSOR_GYRO_ACC_SIM`.

A paired before/after diagnostic used the same single-drone raised-platform
world, Featherstone physics, 500-Hz IMU, 200-Hz odometry, and one `cf2`
process. Before the change, the firmware reported `535 Hz; expected 500 Hz`;
the diagnostic `cf2` SHA-256 was
`116573d136471c417c2266c88445bdf8cbbd22e4f3ce3cc2c50f5153ad9c96f8`, and
Gazebo's aggregate RTF was `0.996`. The capture had 4,768 odometry samples,
only 4,473 fully finite samples, a 2.979-m peak from a 2.040-m spawn, and only
0.355 s within the 2.40-m hover envelope. Both TAKEOFF_2 and LAND_2 reached
the firmware, but it logged `Landing timeout, disarming`.

With the IMU-only wakeup, the 500-Hz stabilizer warning disappeared in the
first retest; the diagnostic `cf2` SHA-256 was
`1934c3dbd9a05ef544b41b0a0c82848420ee2918ce274d064ab474b22370ae0c` and
aggregate Gazebo RTF was `0.994`. The trajectory still failed:
4,853 odometry samples had only 4,542 fully finite values; altitude peaked at
2.922 m and ended at 2.040 m; the longest 2.40-m hover interval was 0.380 s;
and `Landing timeout, disarming` remained. The first nonfinite odometry
angular velocity occurred after the commanded flight. Kalman rate warnings
continued between 83 and 98 predictions per second against the configured
100-Hz rate. The firmware binary also contained temporary SITL-only task,
filter, and tracing edits, so this is behavioral evidence for the semaphore
fix, not a clean acceptance run of commit `743ebe99` by itself.

A follow-up changed the diagnostic Kalman task priority from 2 to 4 but did
not isolate a scheduler fix: one capture had aggregate RTF `0.902`, and a
later one had `0.948`, below the flight gate's minimum. It still showed
Kalman-rate warnings, nonfinite angular odometry, and less than 0.5 s in the
hover envelope. Treat both as failed diagnostic runs; do not use them to
justify widening rate thresholds.

The pushed branch was then cloned cleanly and its `cf2` target compiled with
the Python 3.12.15 interpreter installed by `uv`; the clean binary SHA-256 was
`7207929f6b4c8e1041c895b5bcfba3a579531a68d4e84af51787e10e1211d6d7`. A
separate, uncommitted diagnostic build raised the socket, CRTP RX, sensor, and
Kalman tasks to priority 5, selected the 500-Hz stabilizer loop, and used
500-Hz SITL filter settings. Its binary SHA-256 was
`164ebf5b15b8e351112a386e9a739d735f70c42364759474cb4546dda93cb330`. It
reached `Ready to fly`; one `cf2` process drove finite nonzero motor messages
and Gazebo RTF aggregated to `0.998`. However, the stabilizer reported
`464 Hz; expected 500 Hz`, Kalman remained at 95–98 Hz, and the 4,889-pose
capture had only 4,478 fully finite samples. Altitude peaked at 2.944 m from
2.040 m and returned to 2.040 m, but it stayed in the 2.40-m hover envelope
for only 0.325 s (0.505 s around 2.64 m). Angular odometry became nonfinite
after the flight, and the firmware logged `Landing timeout, disarming`.
Because aggregate RTF was within the gate, this run rules out low average RTF
as the sole cause of the Kalman and stabilizer warnings. The diagnostic
priority/configuration edits were not committed or pushed.

This closes one concrete cause of excess stabilizer wakeups only. The
reference still needs a synchronized, clean positive capture with finite
estimator/odometry state, at least five seconds of stable hover, and a
successful commanded landing, plus a paired disconnected-actuator negative
control. This CrazySim adapter fix does not run Nano-C-SLAM's STM32
application and does not validate mapping.

## Reproducible flight-capture gate

`scripts/check_crazysim_flight.py` checks paired positive-flight and
disconnected-actuator captures. It requires timestamped odometry at 50 Hz or
faster, IMU at 20 Hz or faster, and Gazebo `/stats` covering at least 95% of
the odometry interval. It computes aggregate RTF from the first and last
sim-time and wall-time stamps and requires 0.95–1.05; the 5% bound keeps the
firmware's wall-clock scheduler and Gazebo's simulation clock close enough for
rate comparisons. It also requires finite telemetry and actuator values, a spawn
height within 0.10 m of the raised platform, at least 0.30 m of climb, five
continuous seconds within 0.12 m of the requested hover altitude while upright
and below 0.5 rad/s angular speed, and a landing within 0.10 m of platform
height. It also requires the firmware readiness markers, captured TAKEOFF_2
and LAND_2 commands, exactly one `TRACE_CRTP_RX_STARTED` marker, a plugin trace
attributing motor output to the firmware, no estimator/rate/crash messages,
and successful process exit codes. The
negative capture must explicitly identify a disconnected actuator bridge,
stay within 0.10 m of its starting height, and contain only zero motor output.

Run it against saved `gz topic -e` captures and complete firmware/plugin logs.
Do not echo `/stats` continuously in Gazebo 8: it publishes at the physics
update rate and the text subscriber can perturb simulation timing. Build the
small typed sampler instead; Gazebo Transport throttles the subscription to at
most one callback per second. Install the matching Gazebo `pkg-config` files
first (set `PKG_CONFIG_PATH` if they are outside the default search path):

```sh
clang++ -std=c++17 -O2 scripts/crazysim_world_stats_sampler.cc \
  -o /tmp/crazysim_world_stats_sampler \
  $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
```

Start it with the same `GZ_PARTITION` and network environment as Gazebo, for
the duration of each run. It writes typed protobuf text at 1 Hz and exits
nonzero if it captured fewer than two samples:

```sh
GZ_PARTITION=ncs-flight /tmp/crazysim_world_stats_sampler 120 \
  > flight-world-stats.txt
```

The gate rejects partial clock coverage rather than pairing separate idle
timing data. While the firmware is running, count the processes whose
executable is `cf2` and whose first argument is the firmware UDP port (19951
in this setup):

```sh
ssh -F ~/.lima/ncs-sitl/ssh.config lima-ncs-sitl \
  'ps -eo args= | awk '\''$1 ~ /(^|\/)cf2$/ && $2 == "19951" { n++ } END { print n+0 }'\'''
```

Record that count for each run and pass it to the gate:

```sh
uv run --locked python scripts/check_crazysim_flight.py \
  --odometry flight-odometry.txt --imu flight-imu.txt \
  --world-stats flight-world-stats.txt \
  --motors flight-motors.txt --firmware-log flight-firmware.log \
  --plugin-log flight-gazebo.log \
  --negative-control-odometry disconnected-odometry.txt \
  --negative-control-motors disconnected-motors.txt \
  --negative-control-log disconnected-run.log \
  --firmware-process-count 1 \
  --negative-control-firmware-process-count 1
```

The checker is tested with synthetic fixtures, not represented as a passed
flight capture. The current single-process evidence above fails the hover,
finite-state, rate, crash, and landing criteria. Passing this upstream
CrazySim gate would still not prove execution of Nano-C-SLAM's STM32 tasks or
validate mapping; those require their own firmware-integrated capture and
geometry comparison.

## Rate-supervisor configuration fix (2026-10-19)

The published Crazyflie fork branch `codex/sitl-imu-wakeup` now also contains
commit `0f577c99` (`fix(sitl): derive stabilizer rate checks from loop config`).
`sitl_make` exposes `CRAZYSIM_RATE_MAIN_LOOP` as a validated 500/1000-Hz CMake
setting and passes it into the firmware compile. The stabilizer header accepts
that override, and its rate supervisor now derives the bounds from the compiled
loop frequency while preserving the previous 997–1003 events/s bounds at
1 kHz (498–501 at 500 Hz). This fixes the prior diagnostic mismatch where a
500-Hz loop was compared against 997–1003 events/s. Unity tests cover nominal
500-Hz and 1000-Hz counts and reject a 460-Hz stream; all 11 rate-supervisor
tests pass with Clang and the pinned Unity source.

A Linux `cf2` target build completed at 500 Hz during implementation, before
the final tolerance expression was tightened to preserve the original
per-mille bound. A clean build of the exact pushed commit could not be repeated:
the Lima submodule fetch failed with an `fsync` I/O error after the guest had
reported an I/O error, and the follow-up Gazebo run did not produce a valid
firmware capture. Therefore this commit fixes the measurement configuration;
it does not explain or fix the observed Kalman shortfall (95–98/100 Hz),
non-finite angular odometry, short hover, or landing timeout. The firmware
flight bead remains open, and no Nano-C-SLAM STM32 application or mapping
validation is claimed.
