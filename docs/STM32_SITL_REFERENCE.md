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

The Gazebo plugin source recognizes Harmonic, but a native plugin build did
not complete with the installed Homebrew dependency set. Initial CMake package
discovery did not find `gz-plugin2` until Homebrew prefixes were supplied. With
Gazebo 8, `gz-plugin`, `gz-sim`, `gz-msgs`, `gz-transport`, `gz-cmake`, Qt 5,
and their Homebrew prefixes provided, the plugin C++ source compiled, then
linking failed with an undefined protobuf symbol using Abseil namespace
`lts_20250512`. Adding the ROS Jazzy protobuf include path made the generated
Gazebo message headers incompatible at compile time. Do not mask this ABI
mismatch by allowing unresolved symbols; the plugin has not been loaded into
Gazebo.

## What the source proves, and what it does not

The reference's `power_distribution_sitl.c::motorsSetRatio()` receives four
firmware motor ratios and sends the packed motor command over CRTP/UDP. The
CrazySim Gazebo plugin decodes that command and publishes Gazebo actuator
velocities. Its sensor path returns simulated state/IMU data to the SITL
firmware. This is the right causal architecture to test against the current
Gazebo Sim vehicle; a ROS `/cmd_vel` controller is not equivalent.

The source-level interface is promising for keeping Gazebo Harmonic on macOS
and building the SITL firmware in a Linux environment, but cross-host UDP,
plugin ABI compatibility, sensor units/clocks, and the one-drone closed loop
remain untested. More importantly, the referenced firmware's app is upstream
Crazyflie SITL firmware, not Nano-C-SLAM. Nano's own task graph and hardware
seams still need to be ported behind explicit simulated-device adapters.

## Reproduction state

**Not reproduced.** This record is an adversarial compatibility probe, not
completion of bead `ncs-fea.5.2`. That bead remains open until a clean supported
Linux build produces the SITL executable, the Gazebo plugin loads, a one-drone
sensor/motor loop flies and lands, and logs/hash receipts are archived. The
next bounded experiment is to provision or select a Linux build host without
changing the native Gazebo installation, then use the pinned reference before
porting any Nano app source.
