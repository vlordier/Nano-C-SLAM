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
`libdart-collision-bullet.6.19.dylib`, while 6.20.1 is installed. The integrated
Nano launcher already selects Bullet Featherstone. Direct CrazySim invocations
must also explicitly select an engine; the SDF `type="ode"` field does not
select Gazebo Sim's engine plugin. A manual invocation that omitted
`--physics-engine` hit the DART load error and left firmware motor commands with
no observed physical response. CrazySim flight should use the force-capable
Featherstone plugin rather than regular Bullet.

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

**Current status: upstream CrazySim SITL firmware produces firmware-attributed
motor output and acknowledged takeoff/landing, but stable flight has not
passed.** The latest full-rate paired capture is attempt07: Gazebo altitude
rose from 2.014 m to 2.908 m and returned to 2.015 m, but hover lasted only
0.27 s. Older attempts below that reported no motor output remain historical
failures; attempt07 is the latest flight and cadence evidence.

### Model-consistent motor mapping and strict paired run (2026-10-20)

The CrazySim plugin previously converted firmware PWM using the thrust-upgrade
model's constants (`2.3375e-8`, `2797 rad/s`) even when the selected standard
Crazyflie model used `1.7965e-8`, `3052 rad/s`. The fork now passes the chosen
model's `motorConstant` and `maxRotVelocity` from the SDF interface plugin,
matching all four rotor plugins. The implementation is in fork commit
[`36594bc`](https://github.com/vlordier/crazyflie-simulation/commit/36594bc335102a435d2290621586d2ed5ea6ea43)
on `codex/cf2-motor-thrust-coefficients`. Its C++ motor conversion test and
seven rendered-SDF tests pass. This repairs a real actuator-model inconsistency;
it does not by itself prove a stable flight.

The paired captures in `artifacts/crazysim_motor_mapping_20261020_attempt02`
and `attempt03` use the upstream Crazyflie SITL binary, not Nano firmware.
Attempt01 is explicitly invalid because two firmware processes were alive.
In clean positive attempt02, one firmware acknowledged takeoff and landing,
Gazebo RTF was 0.998875, and altitude rose from 2.015 m to 2.887 m before
returning to the platform. But the longest stable interval near the 2.615 m
target was only 0.64 s. The exact strict checker result is preserved in
`attempt02/paired-check.json`: it fails due to a 0.290 s odometry gap, a
0.092 s IMU gap, world statistics covering only 85.2% of odometry time,
firmware rate/estimator warnings, nonzero shutdown statuses (143/130), and
0.64 s stable hover against the unchanged 5.0 s requirement.

The negative control in attempt03 passes its behavioral checks: the plugin
reported a disconnected actuator bridge, all 2,248 motor components were zero,
and altitude stayed exactly at the 2.015 m spawn height while firmware commands
were acknowledged. The combined flight gate remains open. The run manifests
also call out that the exact Gazebo launch command was not persisted, so this
pair is diagnostic evidence rather than a fully reproducible acceptance run.
No Nano firmware execution, Nano mapping result, multi-drone flight, or hardware
validation is established by these runs.

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

Run it against saved `gz topic -e` captures in protobuf text or
`--json-output` format and complete firmware/plugin logs.
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
  --negative-control-firmware-log disconnected-firmware.log \
  --negative-control-plugin-log disconnected-plugin.log \
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

## Kalman prediction phase correction (2026-10-19)

Follow-up inspection found that `estimator_kalman.c` advanced each prediction
deadline from the actual task wake time (`now + 10 ms`). Any late wake therefore
shifted every future prediction, accumulating scheduler jitter as rate loss.
Crazyflie commit `fadd9ede` replaces this with a wrap-safe periodic deadline
that advances from the previous scheduled deadline and skips missed periods
without issuing catch-up bursts. Seven focused Unity tests pass for exact and
late wakeups, missed periods, `uint32_t` wraparound, future deadlines, and a zero
interval.

This is a code-level phase-drift correction, not yet a demonstrated flight
fix. The exact commit has not completed a `cf2` build or Gazebo retest: Lima
returned guest I/O errors, and fetching the pinned FreeRTOS submodule failed
with an `fsync` I/O error. The previous measured 95–98/100-Hz Kalman rate,
non-finite angular odometry, short hover, and landing timeout remain the
acceptance baseline until a clean build and paired capture are available.

## FreeRTOS POSIX stabilizer tick-width correction (2026-10-19)

Compiling the exact `stabilizer.c` translation unit with Clang against the
CrazySim FreeRTOS POSIX headers exposed a pointer-width mismatch:
`lastWakeTime` was `uint32_t`, while `vTaskDelayUntil` takes a `TickType_t *`
and the 64-bit POSIX port defines `TickType_t` as 64-bit. Passing the narrower
object lets the delay API write beyond it on this host port. Crazyflie commit
`17cdadfe` changes `lastWakeTime` to `TickType_t`; targeted compilation then
completed without the incompatible-pointer diagnostic. The exact Kalman
translation unit also compiles with Clang using the local SITL include paths.

This is compile-level evidence only. It is not a linked `cf2` build, and the
latest Gazebo run does not include these exact fixes. Lima's guest shell
continues to fail with I/O errors, so the previous invalid-estimator state,
short hover, non-finite odometry, and landing timeout remain the latest runtime
evidence. Stable flight and Nano firmware execution are still unvalidated.

## CRTP receive-queue backpressure and retest (2026-10-19)

After restoring the Lima guest and cloning the pushed Crazyflie branch with its
submodules, the exact `17cdadfe` default `cf2` runtime reproducibly aborted in
`socketlink.c`: a zero-timeout `xQueueSend()` assertion fired when the CRTP
delivery queue filled. Crazyflie commit `3e848afb` changes this send to wait
for queue space (`portMAX_DELAY`) instead of crashing the UDP reader. The
change is pushed to `vlordier/crazyflie-firmware`, branch
`codex/sitl-imu-wakeup`. A clean default `cf2` build of that exact commit
completed; its binary SHA-256 is
`3e80e806e4f048105fd55ca98017db4496b8f1153d2e0f73f0c283db25550536`.

A separate diagnostic binary used temporary task-priority overrides and a
500-Hz loop/filter configuration (binary SHA-256
`df6cb48ebdb1ecf4675d9c2a83cb1b7a581bfbed20f0ed57ab4d2728bbc4673c`). It
started the sensor and stabilizer path, logged `GYro Bias found` and `SUP:
Ready to fly`, accepted TAKEOFF_2/LAND_2, and emitted finite nonzero motor
commands. Captured Gazebo odometry rose from the 2.040-m platform to 2.970 m
and returned to 2.015 m after LAND_2. This demonstrates a live reference
firmware-to-Gazebo actuator path for this diagnostic setup.

It is explicitly a **failed flight-gate run**, not flight validation. Only
4,674 of 5,823 odometry samples were fully finite; angular odometry first
became nonfinite at simulation time 28.225 s. The longest interval in the
requested 2.40-m hover envelope was 0.325 s, versus the required five seconds.
The firmware still reported stabilizer rate around 485/500 Hz and Kalman
prediction around 97–98/100 Hz. This capture omitted synchronized IMU and world
statistics, the plugin's motor-attribution trace, and the disconnected-actuator
negative control, so the strict checker cannot pass it. Temporary scheduling
and rate overrides were diagnostic only and are not the clean default build.

The next concrete blocker is to locate the first nonfinite estimator/control
value under timestamped raw IMU and setpoint tracing while retaining the
discovery that queue overflow can crash the exact firmware. Fix the measured
source cause, then produce paired strict-gate captures using one identified
firmware binary: stable finite hover and commanded landing, plus a disconnected
actuator run that stays on the platform with zero motor output. Even those
reference-SITL results would not prove that Nano-C-SLAM's STM32 application or
GAP9 mapping runs in Gazebo.

## Sampled estimator diagnostics and matched-rate retest (2026-10-19)

Crazyflie fork commits `da4ba7de` and `783219ac` add `CRAZYSIM_DIAGNOSTICS`,
disabled by default. When enabled, the firmware prints sampled raw/converted
IMU values and, at 20-Hz intervals, estimator position/velocity, attitude,
setpoint altitude, and controller thrust. Exact fork source builds passed both
with diagnostics on (`cf2` SHA-256
`5325a25fd8b057052aef8177053dd68826ca864b8dea6af6b683c956e0aecbbf`) and off
(`dd4359b93d566833c0694336c43521f61fc6e8bd8806fdc3c176c38c24820e9a`). The
option is pushed to `vlordier/crazyflie-firmware`, branch
`codex/sitl-imu-wakeup`.

The first matched-rate diagnostic capture accidentally omitted the engine flag,
so Gazebo attempted its broken default DART load and odometry stayed at 2.040 m
despite saturated thrust. That was a launcher error, not evidence of a broken
motor conversion. Re-running with `gz-physics-bullet-featherstone-plugin`
selected explicitly produced a firmware-driven rise and landing: odometry
moved from 2.015 m to 2.591 m and returned to 2.015 m; motor topics reached
2,501 rad/s. This resolves the previously unexplained “motor commands but no
climb” result for the direct test setup.

The corrected-engine run still fails the flight gate. Of 1,483 sampled poses,
1,438 were finite; the first nonfinite state appeared at 14.15 s. The sampled
altitude range was 2.015–2.591 m, but the capture did not hold the requested
2.40-m envelope for five seconds. Firmware reported a 473/500-Hz stabilizer
rate, Kalman rates between 67 and 98/100 Hz, and `Landing timeout, disarming`.
Aggregate RTF was 0.728, odometry gaps reached 0.130 s, and IMU gaps reached
0.236 s. The capture therefore fails timing and finite-state checks and cannot
be used to infer clean steady-state controller performance. The strict
positive/negative capture gate remains open.

For every direct reproduction, match firmware and Gazebo IMU rates and select
Featherstone explicitly:

```sh
export GZ_SIM_PHYSICS_ENGINE_PATH="$GZ_INSTALL/lib/gz-physics-7/engine-plugins"
gz sim --physics-engine gz-physics-bullet-featherstone-plugin -r -s world.sdf
```

Gazebo 8's documentation says DART is the default and that the engine plugin
is selected separately from SDF physics parameters. The same CLI flag is
already built into Nano-C-SLAM's `physics_engine_model.py` and macOS launcher;
the omission was in the manual CrazySim reproduction. An earlier attempt also
paired 500-Hz firmware with a 1-kHz IMU and flushed every sampler record; its
1,053-Hz loop count and 0.806 RTF are configuration/instrumentation failures.
The corrected run's sampler still missed strict sensor-rate coverage, and
neither run passes the checker. Stable reference flight, Nano STM32 firmware,
and mapping validation are still unproven.

## Rebuilt-fork command and telemetry retest (2026-10-19)

The Lima checkout was stale at `cb1f716d`; it had not fetched the newer
published `codex/sitl-imu-wakeup` tip. It is now checked out at
`fc30709529ee60dc02b386c49ee004855375bb15`, with the CrazySim submodule at
`7e2d25bcb448349290b65fc0f86a98cdb46bb1d9`. A clean-first default `cf2`
build at 500 Hz completed with diagnostics disabled (SHA-256
`8557d0d18ce57bff1f73e88b7209acddf540089935e5aefe4a02b10cb92d34b1`). A
separate diagnostics-enabled binary also built from a fresh build directory
(SHA-256
`7d824ef815c56627ca7117d15c0ef06cb753c49bea09bb5425d750f3269d25b5`). The
build still emits warnings in upstream SITL code; they are not treated as
warnings-free evidence.

A fresh native Gazebo Sim 8.15.0 world used the Featherstone engine, the
raised platform at 2.015 m, and a single `cf_0`. The diagnostics-enabled
firmware connected, calibrated, logged `SUP: Ready to fly`, accepted the
bounded CRTP client's TAKEOFF_2 and LAND_2 commands with result byte zero, and
sent motor commands that moved the model. In a 24-second subscription window,
3,724/3,724 odometry poses and 74,500 motor velocity values were finite;
odometry altitude ranged 2.0150–2.6565 m. Against the requested 2.615 m
takeoff target, the longest continuous interval within 0.12 m was only 1.840
seconds. The capture contains 9,314 IMU messages and 228 world-stat samples;
the arithmetic mean of sampled instantaneous RTF values was 0.795, with a
0.024–2.160 range. Because the capture used four verbose topic subscriptions
and sampled firmware diagnostics, this RTF is a loaded diagnostic measurement,
not an idle or production benchmark.

Firmware logged `STAB: WARNING: stabilizer loop rate is off (502)` against
the configured 500 Hz and later `SUP: Landing timeout, disarming`. A follow-up
8-second capture showed 1,365 poses at the 2.015 m platform and zero motor
commands: the vehicle did return to the platform and disarm, but only after
the firmware reported the landing timeout. No `State out of bounds` or
`SUP: Crashed` message appeared in this run. The capture still fails the
flight gate: less than five seconds in the required hover envelope, loop-rate
warning, landing timeout, and degraded capture-time RTF. The standalone
checker was not run because this run did not include a synchronized plugin
motor-attribution trace or disconnected-actuator negative-control bundle.

## Timestamped firmware console capture

The previous console capture ended at 4 KiB because the SITL firmware's stdout
was block-buffered and the process was terminated. The host utility
`scripts/capture_timestamped_process.py` now records stdout and stderr as exact
raw byte streams, emits each received line to `events.jsonl` with host realtime
and monotonic arrival timestamps, preserves a final unterminated line, records
the child exit status and per-stream byte counts (including any difference
between raw bytes and timestamped event bytes), and terminates the child process
group at its deadline.
It also samples host and Lima guest clocks before launch and records the
lowest-round-trip offset interval in `clock-offset.json`. That interval bounds
the measured clock offset for the probe; it does not synchronize Gazebo or
replace simulation timestamps in the flight checker.

For a diagnostic run, launch the firmware through Lima with an explicit
deadline and unique output directory, for example:

```sh
uv run --locked python scripts/capture_timestamped_process.py \
  --output-dir /tmp/ncs-firmware-console-run01 \
  --timeout-s 120 \
  --lima-instance ncs-sitl \
  -- limactl shell ncs-sitl -- bash -lc \
  'cd /path/to/crazyflie-firmware/build && exec timeout --signal=TERM --kill-after=2s 115s stdbuf -oL ./cf2 19950 192.168.5.2'
```

Replace the build path with the pinned diagnostic firmware checkout used by the
world, and start the matching Gazebo plugin/world before starting `cf2`. The
collector records the host-side Lima client output. The `process/` directory
contains `stdout.raw`, `stderr.raw`, `events.jsonl`, and `process.json`; the
parent directory contains the clock probe. A bounded collector timeout sends
TERM and then KILL to the Lima client process group; the guest-side `timeout`
independently terminates `cf2` before the collector deadline. `stdbuf -oL`
forces the guest's C stdout into line-buffered mode; without it Lima delivered
bursts of up to 49 diagnostic lines under one host timestamp, too coarse for a
50-ms controller comparison. Confirm the guest process has exited before
reusing its UDP port. This captures line arrival and exit evidence only; it
does not make the current reference flight pass or validate Nano-C-SLAM
firmware.

The new `scripts/crazysim_bridge_command.py` sends these bounded commands to
the plugin's cflib UDP port (20850). Its initial default was corrected after
an ACK test exposed that 19850 was the wrong port for this world. Unit tests
cover the wire encoding, ACK filtering, invalid values, and the port default.
This client makes command reproduction easier; it does not change the flight
gate or turn the diagnostic upstream CrazySim firmware into Nano-C-SLAM.

Next, repeat the same pinned build with a single synchronized capture and
reduced observer overhead, then trace the 502-Hz supervisor measurement and
landing-timeout predicate against simulation timestamps. Keep the existing
five-second hover, finite-state, no-warning, commanded-landing, and
disconnected-actuator checks unchanged. Nano STM32 task execution, ToF/UWB
adapters, and map-to-Gazebo geometry comparison remain separate, unpassed
gates.

## Correctly rendered direct-world retest (2026-10-19)

A second direct-world retest isolated two setup errors that had made the
earlier manual run inconclusive. Rendering `model.sdf.jinja` without
`cf_name="cf"` and `cf_id=0` left `<cfPrefix>` empty; the plugin then
subscribed to `/_0/imu` instead of `/cf_0/imu`. The firmware therefore
completed its UDP handshake but received no IMU and stayed in sensor
calibration. Rendering with `imu_rate=500`, `cffirm_udp_port=19950`,
`cflib_udp_port=20850`, `cf_name="cf"`, and `cf_id=0` exposed the expected
`/cf_0/imu` and `/cf_0/command/motor_speed` topics. The Linux firmware also
needs the Lima host gateway as its second argument: `cf2 19950 192.168.5.2`.
Passing only `19950` leaves its destination at `INADDR_ANY` and does not
establish the cross-VM link.

With those settings, firmware build `fc30709529ee` (diagnostic binary SHA-256
`8d0cc0505f72282eff5f2d57df4e5671cd92cd31354d71d79efc094a4d38dc9c`) logged
`SUP: Ready to fly`, received TAKEOFF_2 and LAND_2, and returned result byte
zero for both bounded commands. A 10.13-second Gazebo odometry capture
contained 2,027 records. Altitude ranged from 2.0146 to 2.8317 m and ended at
2.0150 m after commands for 2.615 m takeoff and 2.015 m landing. The longest
continuous interval within 0.12 m of the takeoff target was 0.25 seconds.
Firmware logs contained one 484-Hz stabilizer-rate warning and one landing
timeout, with no `SUP: Crashed` message. This proves the corrected setup can
run the upstream reference firmware and produce motor-driven Gazebo motion;
it does not pass the stable-flight gate and is not Nano-C-SLAM firmware.

The matched Gazebo 8.14 CrazySim bridge was rebuilt against the local workspace
ABI. However, `lsof` showed that this capture still loaded the installed stock
odometry publisher, not the patched binary from
`/tmp/ncs-gz-sim-odometry-build-qt`; the SDF plugin resolver did not prefer
that build directory. The capture retained 161 records with NaN angular
velocity. The focused publisher regression and 10 integration cases pass,
but runtime verification of the patch remains open. Preserve this capture as
a failed publisher-runtime check, not as evidence that the patch regressed or
that angular telemetry is now finite. The world, full firmware log, odometry
capture, commands, and machine-readable measurements are archived in
[`artifacts/crazysim_rate_match_20261019_attempt03`](../artifacts/crazysim_rate_match_20261019_attempt03/manifest.json).

## Rate-window boundary fix and retest (2026-10-19)

The prior 502/500 stabilizer warning prompted a trace of
`rateSupervisorValidate()`. It incremented the event count, then evaluated
only when `osTimeMs` was strictly greater than the window deadline. If the
supervisor starts with an already-queued sensor event, an exact 500-Hz stream
can therefore include its event at the start and the first event after the
1,000-ms deadline, producing 502 counts for just over one second. Crazyflie
fork commit `8bf5167f` changes this comparison to `>=` and updates the
existing rate tests to cover an immediate first sample, both consecutive
500-Hz windows (501 then 500), 1,000-Hz events, and rejected 460-Hz input. All
11 `test_rateSupervisor` Unity tests pass with GCC and the pinned Unity source.
The full `cf2` default and diagnostics-enabled builds pass after this change;
their hashes are recorded in bead `ncs-fea.5.8.2.1`.

A new run with that firmware still failed the rate/flight checks. Two verbose
Gazebo subscriptions plus sampled firmware diagnostics produced 3,206
odometry samples over 15.998 simulation seconds and 19.481 wall seconds
(aggregate RTF 0.821); altitude ranged from 2.0150 to 2.9995 m for the 2.615-m
takeoff target. Firmware now reported `485/500` rather than the previous
502/500, and later logged `Landing timeout, disarming`. Its estimator state
returned to the 2.015-m platform with thrust at zero; this does not meet the
five-second hover or no-timeout criteria. The changed comparison addresses
the exact-boundary counting error, while the new 485 count shows a remaining
runtime cadence failure under this loaded run. No warning threshold was
expanded. This capture still lacks the synchronized plugin motor-attribution
trace and disconnected-actuator negative control, so the strict checker was
not run and the rate/flight bead remains open.

Next isolate runtime cadence with a low-overhead shared-clock capture, then
measure the relationship between Gazebo RTF, IMU delivery timestamps,
stabilizer iterations and the rate supervisor. The altitude response also
needs a bounded planner/controller trace from setpoint through motor output;
the current 0.38-m overshoot and landing timeout are independent failure
signals. Keep the registered acceptance limits unchanged.

## Low-overhead sampler and firmware-output attribution (2026-10-19)

`scripts/crazysim_flight_sampler.cc` provides a bounded observer for the
existing strict flight gate. For the current 200-Hz odometry and 500-Hz IMU
world, it subscribes without callback throttling to `/cf_0/odom`, `/cf_0/imu`,
and `/cf_0/command/motor_speed`, while world statistics remain capped at 10 Hz.
Earlier sampler versions throttled state streams, which could alias transport
scheduling and create artificial timestamp gaps; retain source callback cadence
for rate and freshness diagnosis. The sampler records compact protobuf JSONL
in four files and refuses
to overwrite an existing capture. It waits up to 15 seconds for correctly
typed publishers, so the sampler can start before Gazebo without losing the
world's initial statistics. The checker accepts the protobuf JSON camelCase
field names emitted by Gazebo. Build the sampler against the same Gazebo
Transport and Messages libraries as the installed Gazebo:

```sh
clang++ -std=c++17 -O2 scripts/crazysim_flight_sampler.cc \
  -o /tmp/ncs-crazysim-flight-sampler \
  $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
GZ_PARTITION=ncs-flight GZ_IP=127.0.0.1 \
  /tmp/ncs-crazysim-flight-sampler 12 \
  /tmp/ncs-flight-capture
```

Set `GZ_IP` to the same value used by the Gazebo server. Matching only
`GZ_PARTITION` is insufficient on this macOS setup: Transport discovery
returned no publishers when the server used `GZ_IP=127.0.0.1` and the sampler
left `GZ_IP` unset. The sampler now prints its effective partition, `GZ_IP`,
each subscription result, publisher message type and address, and a per-topic
zero-callback diagnostic.

Each JSONL record now also carries `capture_unix_time_ns` and
`capture_monotonic_time_ns`, sampled on the host when the Gazebo transport
callback receives the message. Gazebo header timestamps remain unchanged and
continue to drive flight acceptance. The host timestamps let a bounded
firmware console capture be aligned with the sensor and motor streams; use
monotonic time for event order within this sampler and Unix time only for
cross-process correlation after checking host/guest wall-clock offset. These
fields are observer metadata and do not convert sensor arrival time into
simulation time.

The compact JSONL capture retained full timestamps without disturbing
simulation speed: RTF was 0.998 and the largest IMU gap was 22 ms. Starting
the sampler before Gazebo aligned the odometry and stats intervals (99.3%
coverage). The paired capture is archived in
[`artifacts/crazysim_motor_mapping_20261020_attempt07`](../artifacts/crazysim_motor_mapping_20261020_attempt07/manifest.json).
It still fails flight acceptance: maximum odometry gap 50 ms versus the
registered 20-ms limit, stabilizer warning at 502/500 Hz, Kalman warning at
98/100 Hz, and only 0.27 seconds of stable hover. The motor-disconnected
negative control stays grounded with zero motors and reports a disconnected
bridge, though both deliberately signalled shutdown statuses still fail the
strict process-exit check. This separates the remaining control/odometry
problem from the old sampler's deliberate 50-Hz IMU cap; it does not establish
stable flight or Nano firmware execution.

The firmware's `NCS_STAB` trace uses RTOS ticks and does not yet include a
Gazebo timestamp. Its estimated altitude therefore cannot be compared directly
with the odometry clock; the next controller investigation must first correlate
those clocks, then align command, setpoint, estimated state, thrust, PWM and
rotor speed. That work is tracked in bead `ncs-fea.5.8.2.7`.

A synchronized run using the diagnostic `cf2` build and trace-enabled
CrazySim plugin received TAKEOFF_2 and LAND_2 (both acknowledged with result
0). Exactly one firmware process was connected. Firmware logs included one
`TRACE_CRTP_RX_STARTED` marker and both high-level command traces; the plugin
reported `TRACE_MOTOR_NONZERO` with all four firmware ratios at 35999 and
resulting rotor commands at 2056.69 rad/s. This directly attributes a
nonzero Gazebo actuator command to the firmware path.

The 12-second low-overhead capture contained 857 timestamped odometry samples,
536 IMU samples, 2,228 motor components and 24 world-stat samples. Aggregate
RTF was 0.875. Altitude ranged from 2.0150 to 2.8337 m, but the longest stable
interval around the 2.615-m target was only 0.23 s. Firmware reported
stabilizer rate 495 Hz against 500 Hz and `Landing timeout, disarming`. This
proves firmware-to-rotor actuation and an initial rise; the strict flight gate
still fails for hover duration, rate and landing.

A paired disconnected-actuator capture then used one `cf2` process, the same
world and sampler, and a plugin built from CrazySim commit `051d36e`. Both
TAKEOFF_2 and LAND_2 were acknowledged; firmware logged exactly one startup
marker and both command traces. The plugin log explicitly said
`NCS_SITL actuator_bridge=disconnected` and also traced nonzero firmware motor
ratios (36,000 each, 2,056.72 rad/s requested). Gazebo's captured actuator
topic nevertheless contained 1,512 components, all exactly zero. Across 597
odometry samples the model stayed at z=2.014994 m (zero measured drift); 376
IMU samples and 16 world-stat samples were captured, with aggregate RTF 0.984.
This confirms the negative-control mechanism: firmware motor output cannot
move the model when the actuator bridge is disconnected. That firmware log
still contains an estimator reset, a loop-rate warning and a landing timeout;
these are not masked by the negative test. The strict paired checker reports
no negative-control failure, while the positive capture fails on odometry and
IMU gaps, RTF 0.875, estimator/rate diagnostics, 0.24 s hover and failure to
land within 0.10 m. Process exit status was not part of this live capture's
evidence, so do not call the combined gate passed.

These captures use the upstream Crazyflie SITL reference application; they do
not execute Nano-C-SLAM's STM32 app and do not validate mapping.

## Automatic firmware-to-Gazebo IMU rate matching (2026-10-19)

Earlier matched-rate experiments required editing a generated world by hand,
so future runs could silently return to a 1-kHz Gazebo IMU with a 500-Hz
firmware loop. CrazySim fork commit `1e6d1262` adds a validated `--imu-rate`
generator option to both Crazyflie model templates. The three Gazebo launchers
now read `CRAZYSIM_RATE_MAIN_LOOP` from the firmware build's `CMakeCache.txt`
and pass it to the generator; `NCS_CRAZYSIM_IMU_RATE` is an explicit override.
The existing 1000-Hz default remains for builds without the CMake setting.
The corresponding SITL firmware fork commit `f42b9036` pins that CrazySim
revision on `codex/sitl-imu-wakeup`.

Six focused tests pass for the two generated models at 500 and 1000 Hz, invalid
rate rejection, CMake-cache selection, override precedence and fallback. In
the Lima guest, with the actual 500-Hz diagnostic build cache and no override,
the launch resolver selected `500`; the forked Jinja generator emitted valid
SDF whose IMU `update_rate` parsed as `500`. The test suite, Bash syntax checks,
ShellCheck on the new resolver, Ruff check/format for the test, and
`git diff --check` passed. This removes a configuration footgun. It does not
show that Gazebo delivered 500 IMU callbacks per wall-clock second, that the
firmware supervisor met its timing bound, or that flight is stable. The latest
flight evidence below remains a failed gate; future runs should use the new
pinned launcher path and synchronized low-overhead capture.

## Matched 500-Hz flight retest (2026-10-19)

A fresh 20-second low-overhead capture used one diagnostic `cf2` process
(binary SHA-256
`8d0cc0505f72282eff5f2d57df4e5671cd92cd31354d71d79efc094a4d38dc9c`), a
500-Hz firmware build, and the manually matched 500-Hz IMU world. TAKEOFF_2
and LAND_2 both received firmware ACK result 0. The firmware reported one
startup marker and `SUP: Ready to fly`, then warned that the stabilizer rate
was 502 Hz, the Kalman prediction rate was 97–98 Hz, and landing timed out.

The sampler captured 1,458 odometry samples, 909 IMU samples, 3,760 motor
components and 39 world-stat samples over 18.2 simulation seconds. Only
1,253 odometry samples were finite; the first non-finite state occurred at
37.925 s. Maximum odometry and IMU gaps were 0.065 s and 0.122 s. Altitude
rose from 2.014994 m to 2.923774 m and ended at 2.037885 m, but the longest
continuous interval within 0.12 m of the 2.615-m target was 0.27 s. Aggregate
RTF was 0.911. These fail the registered timing, finite-state, hover and
landing criteria. The estimator and cadence problem is not explained solely
by pairing a 500-Hz loop with a 1-kHz IMU.

The raw data, firmware log, world SDF and SHA-256 manifest are preserved in
[`artifacts/crazysim_rate_match_20261019_attempt01`](../artifacts/crazysim_rate_match_20261019_attempt01/manifest.json).
This run did not capture a synchronized plugin log or record process exit
statuses, so the strict paired checker was not run. It also used the previous
manually matched world rather than spawning through the new launcher; the
separate CMake-cache-to-generated-SDF test proves the launcher configuration,
not a new flight. Continue by capturing plugin attribution and improving
observer RTF, then trace the first non-finite estimator state and 502-Hz
measurement from raw sensor arrival through stabilizer scheduling. Do not
relax the registered gate.

A second run reused the same manually matched world with the 500-Hz
diagnostics-disabled firmware (binary SHA-256
`95ab9a2f778063fb757273d88a1fe04f1eab566ce42130d21e313670344b6651`). The
two commands again received ACK result 0 and one `cf2` process was observed.
The parsed capture contained 1,434 odometry samples, of which 1,211 were
finite, 881 IMU samples, 3,648 motor components and 38 world-stat samples.
Altitude rose from 2.014994 m to 2.935993 m and returned to 2.014994 m; the
longest target-envelope interval was 0.315 s. Aggregate RTF was 0.945, just
below the required 0.95 floor. Maximum odometry gap was 0.040 s; IMU gap was
0.048 s. The first non-finite odometry sample was at simulation time 78.62 s.

The RTF difference from the diagnostics-enabled attempt01 (0.911) is
consistent with trace volume affecting simulation throughput, but these are
separate runs and do not isolate that cause. The diagnostics-disabled run's
firmware output remained buffered and no synchronized plugin log was captured;
therefore it cannot establish warning absence or pass the paired checker. Its
raw topic data and manifest are in
[`artifacts/crazysim_rate_match_20261019_attempt02`](../artifacts/crazysim_rate_match_20261019_attempt02/manifest.json).
Both matched-rate attempts still fail stable-flight acceptance. The next
capture needs line-buffered firmware logs, plugin motor attribution, exact
process/exit evidence, and lower observer overhead while retaining the same
finite-state, timing, hover and landing thresholds.

## Gazebo odometry quaternion roundoff regression (2026-10-19)

Inspection of matched-rate attempt01 showed that all 205 non-finite records
were limited to `twist.angular`; position, orientation, and linear velocity
remained finite. The first affected message was at simulation time 37.925 s.
Its quaternion and the preceding orientation produce a relative quaternion
with `w` slightly above 1 due to floating-point roundoff. Gazebo Harmonic's
`OdometryPublisher::CalculateAngularVelocity` called `Quaternion::AxisAngle`,
whose `acos(w)` then returned NaN. This attributes the captured angular NaNs
to the Gazebo odometry publisher; it does not establish that firmware state was
non-finite. CrazySim forwards pose to the firmware localization packet and
does not forward the odometry angular velocity there.

An isolated worktree of `gz-sim` has a focused change that normalizes the
relative quaternion and computes its angle with `atan2`, including a zero
rotation / non-positive timestep guard. Its regression uses the actual
consecutive orientations from attempt01: the previous `AxisAngle` result is
non-finite, while the replacement result is finite and near zero. The
odometry-publisher integration target builds and passes all ten cases on the
local Gazebo workspace when configured with that workspace's install prefix.

On 2026-10-20, a copied build plugin was loaded in a fresh CrazySim reference
run. The first load attempt exposed a duplicate protobuf descriptor because
the copied dylib retained its build-tree rpath; a temporary runtime copy with
that rpath removed and Gazebo dependencies pinned to the workspace install
loaded without changing the shared installation. `lsof` confirmed both the
patched odometry dylib and the CrazySim bridge from their intended paths. The
3,200-message JSON odometry capture has 15.995 s of strictly timestamped data,
all pose/twist values finite, and no angular-velocity NaNs. This closes the
publisher's runtime finite-output check.

The integration target also passes when its test server and system plugins use
the same build tree. An earlier CTest abort mixed the test binary's build-tree
`libgz-sim` with system plugins from the shared install; protobuf then
registered `gz/sim/private_msgs/peer_control.proto` twice. An isolated build
prefix and explicit plugin path avoid that split:

```sh
cmake -S /tmp/ncs-gz-sim-odometry -B /tmp/ncs-gz-sim-odometry-build-qt \
  -DCMAKE_INSTALL_PREFIX=/tmp/ncs-gz-sim-odometry-build-qt/install
cmake --build /tmp/ncs-gz-sim-odometry-build-qt --target \
  INTEGRATION_odometry_publisher gz-sim8-odometry-publisher-system \
  gz-sim8-physics-system gz-sim8-user-commands-system \
  gz-sim8-scene-broadcaster-system -j4
GZ_SIM_SYSTEM_PLUGIN_PATH=/tmp/ncs-gz-sim-odometry-build-qt/lib \
  ctest --test-dir /tmp/ncs-gz-sim-odometry-build-qt \
  -R '^INTEGRATION_odometry_publisher$' --output-on-failure
```

That run passed all ten GoogleTest cases (CTest 1/1) without installing over
or changing the shared Gazebo workspace. The integration test and matched-rate
runtime capture now agree on finite angular output; flight control remains a
separate failed gate.

The firmware still does not pass flight acceptance: altitude was 2.0149 to
3.0964 m and ended at 2.6360 m, the longest target-hover interval was 0.205 s,
and peak angular speed was 6.149 rad/s. The firmware received TAKEOFF_2 and
LAND_2 (ACK result 0 for both), but logged one 502-Hz stabilizer warning, 18
Kalman cadence warnings, and one landing timeout; it did not log a supervisor
crash. The strict paired checker was not run because this attempt lacks
synchronized IMU, motor, world-stat, disconnected-actuator, and process-exit
captures. Raw files and hashes are in
[`artifacts/crazysim_patched_odometry_20261020_attempt02`](../artifacts/crazysim_patched_odometry_20261020_attempt02/manifest.json).
This fixes the odometry NaN symptom only; it does not resolve controller
stability, firmware cadence, Nano firmware execution, or mapping validation.

A follow-up 20-second run used the repository's typed C++ sampler. The patched
odometry dylib and CrazySim bridge loaded, and the plugin traced four nonzero
motor ratios originating from firmware. The sampler nevertheless received
zero callbacks. The cause was an incomplete client environment: Gazebo had
`GZ_IP=127.0.0.1`, while the sampler inherited only `GZ_PARTITION`. Repeating
the test against one isolated server showed four publishers at the expected
`gz.msgs.*` types and a shared `tcp://127.0.0.1:<port>` address when both
variables matched; the sampler captured odometry, IMU, motor and world-stat
messages and exited zero. With `GZ_IP` unset and the same partition, discovery
returned zero publishers for all four topics, callbacks stayed at zero, and
the sampler exited 4 with a diagnostic naming the required environment
check. A five-second capture with the matched environment received 506
odometry, 235 IMU, 242 motor and 10 world-stat messages. The diagnostic run is
archived in
[`artifacts/crazysim_patched_odometry_20261020_attempt03`](../artifacts/crazysim_patched_odometry_20261020_attempt03/manifest.json);
the corrected positive firmware capture is in
[`artifacts/crazysim_patched_odometry_20261020_attempt04`](../artifacts/crazysim_patched_odometry_20261020_attempt04/manifest.json).

The corrected 14-second firmware capture received acknowledged TAKEOFF_2 and
LAND_2 commands and shows the model rise from 2.015 m to 3.070 m before
returning to 2.015 m. It contains 1,193 timestamped odometry samples, 648 IMU samples,
2,680 motor components and 27 world-stat samples. The longest interval inside
the registered hover band was 0.47 s, below the 5-second requirement; peak
motor speed was 2,775 rad/s. This is an instrumented but failed flight. The
strict paired checker remains unpassed because this run did not persist the
firmware/plugin logs or a paired disconnected-actuator capture with process
exit evidence. Fix hover stability and retain a complete positive/negative
bundle before claiming firmware flight validation. Parsing also exposed and
fixed a checker bug: Protobuf text omits zero-valued `nsec`, so five valid
odometry messages at integer simulation seconds had previously been dropped.

## Fresh-plugin flight with uncapped telemetry (2026-10-20)

I rebuilt the CrazySim plugin from fork commit
`da1e3d8b0c2c7ad3879a69176c7e67da909aca6e` and verified the loaded library by
its hash. With the same upstream Crazyflie SITL binary and world, the firmware
completed its self-test and calibration, received acknowledged TAKEOFF_2 and
LAND_2 commands, and produced motor output explicitly attributed to firmware
by the plugin. Gazebo odometry measured a rise from 2.014 m to 3.015 m and a
return to 2.015 m. This is direct evidence that a firmware process drives the
Gazebo rotors and the model can take off and land; it remains a failed stable
flight, and it is not the Nano-C-SLAM application.

The sampler no longer throttles IMU or motor callbacks. The 25-second capture
contains 4,994 odometry samples at 200 Hz (maximum gap 5 ms), 12,448 IMU samples
at 500 Hz (maximum gap 2 ms), 24,896 motor messages, and 187 world-stat samples.
The strict paired gate reports only 0.38 s in the hover envelope rather than
the required 5 s, and rejects stabilizer/Kalman rate warnings (485/500 Hz and
98/100 Hz). Firmware shutdown was SIGTERM (143); the reused attempt03 negative
control has SIGTERM/130 exits. The checker therefore exits 1. Raw captures,
logs, the unchanged-gate result, hashes, model, and run configuration are in
[`artifacts/crazysim_motor_mapping_20261020_attempt08`](../artifacts/crazysim_motor_mapping_20261020_attempt08/manifest.json).
The same flight interval exposes another sensor question: Gazebo IMU
sensor-frame Z acceleration stayed within 9.80608–9.80665 m/s² while
acceleration finite-differenced from odometry's vertical velocity ranged from
-2.017 to 9.379 m/s². This is evidence to inspect the model sensor and
coordinate conversion; it does not yet establish which side is wrong. The next
firmware investigation should address the measured 15/500-Hz stabilizer loop
deficit, the position-controller oscillation, and this IMU/odometry disagreement before
another acceptance run. Mapping, swarm, and Nano application validation remain
open.

## Hover and actuator-isolation retest (2026-10-20)

The next matched pair used SITL firmware commit `55a8f414`, CrazySim commit
`aeafa64`, and the patched odometry publisher. The positive run started at
z=2.015 m, acknowledged TAKEOFF_2 to 2.315 m and LAND_2 back to the platform,
and kept all 5,975 odometry samples finite. The capture includes 14,938 IMU
samples, 29,777 motor messages and 232 world-stat samples. It remained inside
the registered 0.12-m altitude, 15-degree tilt and 0.5-rad/s angular-rate
envelope for 5.255 continuous seconds. It landed at z=2.015 m; aggregate RTF
was 0.992 and the largest odometry gap was 5 ms.

The paired negative run used the same build and world with the actuator bridge
disabled before spawn. Across 3,013 poses, altitude remained at 2.015 m within
4 micrometers, and all 15,014 captured motor components were zero. The plugin
logged `actuator_bridge=disconnected`.

This is the first captured 5-second hover interval with a same-configuration
actuator-isolation control. It still fails the strict flight gate: the
firmware warned at 502 Hz for the 500-Hz stabilizer and at 95–98 Hz for the
100-Hz Kalman prediction task; it also logged `Landing timeout, disarming`.
Both firmware sessions were interrupted after bounded captures (exit 255).
The first checker invocation passed only the negative plugin log for its
startup-marker check, although the separate firmware log contains the marker.
After separating those inputs, the checker still exits 1 for the rate warning
and two nonzero process exits. Full captures, logs, SDFs,
commands and hashes are in
[`artifacts/crazysim_motor_mapping_20261020_attempt09`](../artifacts/crazysim_motor_mapping_20261020_attempt09/manifest.json).

The five orphaned `cf2` processes left by the test sessions were stopped after
their PID, command, working directory and sockets were checked. The next run
should use a session supervisor that captures PID and shutdown status for each
firmware and Gazebo process, and wait for all sessions to exit before starting
the next UDP partition. Do not loosen the rate gate based on aggregate RTF:
these captures show aggregate real-time performance near 1.0 alongside
per-window firmware rate warnings. Firmware-driven movement and the
disconnected-actuator negative control are demonstrated, while clean strict
flight acceptance, Nano-C-SLAM firmware, mapping, and swarm validation remain
open.

## SITL shutdown and rejected Kalman-priority experiment (2026-10-20)

The reference SITL process previously ran an unconditional loop after starting
FreeRTOS and had no signal handling, so bounded Gazebo captures could only
terminate it forcibly. Firmware fork commit `e06abd11` adds SIGINT/SIGTERM
handling through a low-priority monitor task. A live probe sent SIGTERM before
Gazebo connected; the firmware printed `SITL: Scheduler stopped cleanly` and
returned exit code 0. The full-flight attempts below also exited zero after
SIGTERM. This makes process shutdown measurable by the flight gate.

I tested raising the SITL Kalman task from priority 2 to the stabilizer's
priority 5, on the theory that the Kalman task was missing its binary-semaphore
wakeups. The paired positive/negative captures use the same experimental
binary. The positive run accepted TAKEOFF_2 and LAND_2 and ran at aggregate RTF
0.987, but the stabilizer warned at 481/500 Hz and Kalman at 97–98/100 Hz.
The supervisor reported `Crashed, recovery required`; the model flipped to a
180-degree attitude, produced 1,536 non-finite odometry records, and remained
within the registered hover envelope for only 0.315 s. This experiment is
rejected, and the checked-out firmware config has priority 2 restored.

With its actuator bridge disabled, the same priority-5 binary kept the model
at z=2.014990–2.014994 m and published zero nonzero motor components across
17.895 simulation seconds. The negative control passes its isolation behavior,
but it does not rescue the positive flight. The strict paired checker exits 1
for non-finite positive telemetry, missing positive plugin attribution,
firmware cadence/crash warnings and insufficient hover. Raw captures, both
logs, the candidate binary, configuration patch, hashes and gate output are in
[`artifacts/crazysim_motor_mapping_20261020_attempt10`](../artifacts/crazysim_motor_mapping_20261020_attempt10/manifest.json).

The experiment rules out task priority as the cadence fix. Next trace the
FreeRTOS POSIX port's tick and semaphore timing against Gazebo simulation time,
and instrument the first non-finite odometry timestamp alongside the
firmware's estimator state. The shutdown fix is independent of flight
behavior; Nano-C-SLAM execution and mapping validation remain open.

## Matched-rate instability retest and rejected 510-Hz experiment (2026-10-20)

I tried 510-Hz IMU output to compensate for wall-clock scheduling jitter. That
was an invalid experiment: the local Gazebo Harmonic `gz-sensors` checkout
converts `1 / update_rate` to integer milliseconds in
[`Sensor.cc`](../../gz-macOS/gz-sensors/src/Sensor.cc#L521). At 510 Hz,
1.96 ms truncates to 1 ms, and the captured topic ran at 1 kHz. The CrazySim
generator only supports 500 or 1,000 Hz; use those exact values. The sampled
510-Hz world and 1-kHz result are in `/tmp/ncs-imu510-*` and were not counted as
a flight retest.

I then reran with a 500-Hz firmware build, a 500-Hz IMU, the `aeafa64` PWM to
thrust conversion, and the `b256950` odometry publisher patch. Firmware was
started before Gazebo. TAKEOFF_2 to 2.315 m and LAND_2 to 2.015 m both received
ACK result 0. Gazebo delivered 500.03 IMU samples/s with 2-ms maximum gaps and
200-Hz odometry with 5-ms maximum gaps. The vehicle climbed to 3.497 m, flipped,
and fell off the platform to 0.015 m. It remained in the registered 2.4-m hover
envelope for 0.175 s. Firmware reported a 483/500-Hz stabilizer rate, 95–98-Hz
Kalman predictions, estimator resets, and `SUP: Crashed, recovery required`.
Aggregate RTF was 0.9513, narrowly inside the configured [0.95, 1.05] band;
world-stat coverage was 94.1%, below its 95% threshold. Firmware shutdown
returned 0; Gazebo was interrupted and returned 1. This positive-only run did
not receive a strict paired-gate result because there was no fresh negative
capture. Details and hashes are in
[`artifacts/crazysim_motor_mapping_20261020_attempt11`](../artifacts/crazysim_motor_mapping_20261020_attempt11/manifest.json).

The firmware-control source differs from attempt09 only by the signal-shutdown
change, yet attempt09 held within the 2.315-m band for 5.255 s while attempt11
flipped before completing its hold. The reference flight is therefore not
repeatable from the archived configuration alone. Compare time-correlated
IMU arrival, estimator reset, attitude, thrust, per-motor ratios, and rotor
speeds around the first attitude divergence before changing gains or relaxing
the flight gate. A single successful hover interval is not repeatable-flight
acceptance. Even repeatable CrazySim reference flight would not run the
Nano-C-SLAM STM32 application or validate its map.

## Timestamp-aligned attitude correlation and checker correction (2026-10-20)

A bounded 45-second upstream CrazySim reference run is archived at
[`artifacts/crazysim_attitude_correlation_20261020_attempt03`](../artifacts/crazysim_attitude_correlation_20261020_attempt03/README.md).
Its matching firmware binary returned guest timeout code 124 after 60 seconds;
the timestamp collector completed normally and captured 275,300 stdout bytes
with zero missing event bytes. The best Lima clock probe measured 37.49 ms RTT
and guest-minus-host offset +72.88 ±18.74 ms. Use the recorded arrival times as
interval evidence, not as exact simulation-time timestamps. `stdbuf -oL` is
required for useful line timing: earlier captures without it grouped up to 49
firmware lines under one host arrival timestamp.

In attempt03, Gazebo observed both takeoff and land ACKs, nonzero motor output,
and altitude from 2.0144 m to 2.4682 m. The model nevertheless tilted to
167.11 degrees and the supervisor reported a crash. The strict paired checker
was not run because this capture lacks a matching disconnected-actuator
negative control. Firmware/Gazebo gyro cross-correlation peaks within roughly
6 ms, so this capture weighs against a large sensor queue delay; it does not
identify the cause of instability. Preserve the motor order/sign/scaling and
estimator/controller questions as open until first-divergence traces establish
them.

This capture also exposed a checker defect: the prior `2*acos(abs(q.w))`
metric treated heading yaw as body tilt. The checker now measures body-Z
alignment with world vertical, so yaw-only rotation does not fail the tilt
limit. Focused tests cover 90-degree yaw and 20-degree roll/pitch while keeping
the existing 15-degree threshold. Reanalysis still finds only about 0.35 s in
the target hover envelope and a 167.11-degree maximum tilt; the correction does
not turn this failed flight into a pass.

The next hardware-independent investigation is the four-channel motor mapping
from firmware ratio through SITL packet, plugin actuator and SDF rotor to
measured body torque, followed by aligned controller/estimator traces if that
mapping passes. Nano-C-SLAM firmware execution, mapping validation, and swarm
flight remain unproven.

### Static motor-channel audit

The exact Linux reference firmware in Lima (`/home/vincent.guest/ncs-cf-fw`,
commit `e06abd1154ff29e1b226b6d0921c8992b306953e`) and the captured Gazebo
plugin/world have a consistent **static** four-channel mapping:

| Firmware field / legacy mixer | CRTP payload field | Plugin conversion / published actuator | SDF rotor | Rotor position (m) | Spin |
| --- | --- | --- | --- | --- | --- |
| `m1`; legacy `-roll + pitch + yaw` | packed `m1` | `m1` → `m_motor_command_.m1` → index 0 | `m1_joint`, `actuator_number=0` | (+0.031, -0.031) | CCW |
| `m2`; legacy `-roll - pitch - yaw` | packed `m2` | `m2` → `m_motor_command_.m2` → index 1 | `m2_joint`, `actuator_number=1` | (-0.031, -0.031) | CW |
| `m3`; legacy `+roll - pitch + yaw` | packed `m3` | `m3` → `m_motor_command_.m3` → index 2 | `m3_joint`, `actuator_number=2` | (-0.031, +0.031) | CCW |
| `m4`; legacy `+roll + pitch - yaw` | packed `m4` | `m4` → `m_motor_command_.m4` → index 3 | `m4_joint`, `actuator_number=3` | (+0.031, +0.031) | CW |

The SITL firmware stores four contiguous `uint16_t` values in `motorPower`,
assigns `m1` through `m4` from the mixer output in that order, and copies all
four fields into the CRTP packet. The plugin reads each same-named field,
converts it to angular velocity, and publishes the four values in that order.
The SDF assigns the corresponding motor model to actuator indices 0 through 3.
For the X-layout, the legacy roll/pitch mix is consistent with rotor position;
the force/torque mix's yaw signs are also consistent with the SDF's alternating
CCW/CW directions under the Crazyflie body convention.

Source locations for reproduction: the firmware packed layout is
`src/modules/src/power_distribution_sitl.c:55-60`, legacy mixer
`104-112`, force/torque mixer `115-137`, and packet serialization
`207-217`. The archived plugin snapshot has packet decoding/conversion at
[`crazysim_plugin.cpp:402-429`](../artifacts/crazysim_motor_mapping_20261020_attempt11/source/crazysim_plugin.cpp#L402)
and actuator publication at `441-449`; the archived SDF has rotor positions at
`119-204` and motor-model configuration at `287-356`. Its exact hashes are
recorded in
[`attempt11/SHA256SUMS`](../artifacts/crazysim_motor_mapping_20261020_attempt11/SHA256SUMS).

This source audit rules out an obvious channel permutation between these
specific captured sources. It does not measure Gazebo's resulting single-rotor
body torque, prove the firmware binary was built from precisely this working
tree beyond its recorded identity, or rule out PWM-to-thrust calibration,
actuator dynamics, estimator timing, or controller instability. The restrained
single-channel torque-sign test and first-divergence motor trace remain open in
bead `ncs-fea.5.8.2.7.1.3`; no speculative sign or gain change is justified.

### Isolated single-channel attitude response (2026-10-20)

Four fresh Gazebo runs published one nonzero value at a time to
`/cf_0/command/motor_speed`; all other entries remained zero. The vehicle used
the same captured CrazySim SDF and Bullet Featherstone motor models as the
reference flight. The world was deliberately gravity-free so a pulse could not
be mistaken for a takeoff. M1, M2 and M4 used 500 rad/s; M3 was repeated at
100 rad/s after the 500 rad/s run produced invalid angular-velocity fields.
Each pulse lasted about 0.20 simulation seconds. The sampler recorded the
command vectors, world time, odometry and process exit status in
[`attempt04`](../artifacts/crazysim_motor_pulse_20261020_attempt04),
[`attempt05/m1`](../artifacts/crazysim_motor_pulse_20261020_attempt05/m1), and
[`attempt11_m3_100`](../artifacts/crazysim_motor_pulse_20261020_attempt11_m3_100).
All four selected runs returned zero from both publisher and Gazebo.

The odometry `twist.angular` fields were intermittently NaN, so the sign check
uses the finite orientation quaternions instead. For each pulse, the values
below are the change in roll, pitch and yaw between the means of the first and
last 20% of finite samples inside the pulse. Angles are in radians.

| Channel | Active command vector (rad/s) | Finite pose samples | Δ roll | Δ pitch | Δ yaw | Expected / observed signs |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| M1 | `[500, 0, 0, 0]` | 35 | -0.0641 | -0.0627 | -0.00776 | `---` / `---` |
| M2 | `[0, 500, 0, 0]` | 51 | -0.1490 | +0.1466 | +0.00944 | `-++` / `-++` |
| M3 | `[0, 0, 100, 0]` | 31 | +0.00213 | +0.00206 | -0.000708 | `++-` / `++-` |
| M4 | `[0, 0, 0, 500]` | 29 | +0.0455 | -0.0444 | +0.00631 | `+-+` / `+-+` |

The signs agree with the static X-layout mixer/SDF convention for all four
channels. The control vectors before and after each pulse were `[0, 0, 0, 0]`.
This is useful evidence against a simple channel-order or rotation-sign error,
but it is not a direct torque measurement: the capture does not include the
four motor-model speed outputs or a force/torque sensor, and the angular-rate
fields were not reliable. M1, M2 and M4 also use a higher command speed than
M3, so their magnitudes are not comparable. The zero-gravity free-body setup
does not validate thrust, hover, landing, firmware motor output, or flight.
The disconnected-bridge negative control and restrained, repeatable test
harness remain outstanding; bead `ncs-fea.5.8.2.7.1.3.2` therefore stays open.
The synchronized firmware-to-Gazebo first-divergence trace in
`ncs-fea.5.8.2.7.1.3.3` is still required before changing controller gains or
claiming a validated motor mapping.
