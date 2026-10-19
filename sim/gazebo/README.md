# Gazebo SITL

This integration uses the Gazebo Harmonic Crazyflie world from [`crazyswarm2_gazebo`](https://github.com/prakash-aryan/crazyswarm2_gazebo), with ROS 2 Jazzy. The world provides Crazyflie physics, velocity control, odometry, and a four-ray multiranger scan. The native macOS mission uses measured odometry and publishes velocity commands directly to the existing `/cf1/cmd_vel` Gazebo bridge; this avoids the upstream `control_services` relay's internal flight-state cutoff. The launch keeps the world's default physics engine. We tested the Crazyswarm2 `hello_world` service path as well, but its calls were acknowledged while Gazebo odometry showed no altitude change.

The `nano_c_slam_gazebo` ROS package currently publishes two telemetry topics:

- `/nano_c_slam/sim/<drone>/ranges_mm`: `StampedRanges` with source stamp, drone identity, sequence, and four distances in `front,back,left,right` order; `-1` marks invalid readings.
- `/nano_c_slam/sim/uwb_pairwise_range_m`: `StampedPairwiseRanges` with a publication stamp, bridge sequence, stable drone and pair order, exact source odometry stamps, and ideal 3D distances computed from Gazebo odometry.

See [RANGE_TELEMETRY.md](RANGE_TELEMETRY.md) for the timestamp, sequence, clock, pairing, and evidence contract. The bridge and collector now pair source-stamped ranges with odometry under the documented age and skew bounds.

These are simulation adapters. The Gazebo lidar has four rays, while the hardware uses four 8x8 ToF matrices; the pairwise ranges use ground-truth simulator odometry, with no UWB noise, packet loss, or ranging protocol. The simulation does not run the GAP9 SLAM task or STM32 firmware. The existing `cslam-stm32-app/run_demo.py` still uses `cflib` and a Crazyradio, so it cannot be run unchanged against this ROS 2 simulator.

## Build

On Ubuntu 24.04, install ROS 2 Jazzy, Gazebo Harmonic, `ros-jazzy-ros-gz`, and `uv`. Clone the upstream project with its pinned submodules and nested dependencies:

```sh
git clone --recursive https://github.com/prakash-aryan/crazyswarm2_gazebo.git ~/Work/crazyswarm2_gazebo
cd ~/Work/crazyswarm2_gazebo/src/crazyswarm2
git submodule update --init --recursive
cd ~/Work/crazyswarm2_gazebo
```

Build the upstream workspace and this package together. The package is passed as an extra colcon source path, so it does not need to be copied into the upstream checkout:

```sh
source /opt/ros/jazzy/setup.bash
uv run --no-project --python /usr/bin/python3 --with colcon-common-extensions \
  colcon build --base-paths src "$HOME/Work/Nano-C-SLAM/sim/gazebo/nano_c_slam_interfaces" \
    "$HOME/Work/Nano-C-SLAM/sim/gazebo/nano_c_slam_gazebo" \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

The upstream project separately requires Crazyflie firmware Python bindings on `PYTHONPATH`; follow its build instructions in the linked README if they are not already available.

## Run

In one terminal, launch the upstream Gazebo/Crazyswarm2 backend:

```sh
cd ~/Work/crazyswarm2_gazebo
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$HOME/Work/crazyflie-firmware/build:$PYTHONPATH"
uv run --no-project --python /usr/bin/python3 \
  /opt/ros/jazzy/bin/ros2 launch ros_gz_crazyflie_bringup gazebo_crazyswarm2.launch.py
```

In a second terminal, launch the bridge and check sensor messages:

```sh
cd ~/Work/crazyswarm2_gazebo
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
uv run --no-project --python /usr/bin/python3 \
  /opt/ros/jazzy/bin/ros2 launch nano_c_slam_gazebo bridge.launch.py
```

In a third terminal, run the repository's measured-odometry takeoff, hover, and
landing mission. It publishes velocity commands directly to the Gazebo bridge:

```sh
source /opt/ros/jazzy/setup.bash
source ~/Work/crazyswarm2_gazebo/install/local_setup.bash
export PYTHONPATH="$HOME/Work/crazyflie-firmware/build:$PYTHONPATH"
uv run --no-project --python /usr/bin/python3 \
  /opt/ros/jazzy/bin/ros2 run nano_c_slam_gazebo sim_mission --ros-args \
  -p use_sim_time:=True
```

The mission takes off to 0.5 m, hovers for three seconds, and lands using fresh
odometry. To fly all three vehicles together, run this in place of `sim_mission`:

Each `sim_mission` process generates a UUID run ID (or accepts an orchestrator's
UUID through the `run_id` ROS parameter) and publishes versioned JSON phase
events on `/<drone_name>/mission_status` (`std_msgs/String`). Events include the
run and drone identity, simulation-clock time, measured altitude, requested and
completed mapping-path state, and landing-height limit. A `COMPLETE` event is
emitted after odometry altitude remains at or below the configured threshold
for at least 0.50 s of simulation time, with no accepted sample gap above
0.20 s, and, when requested, the bounded path is complete. This establishes
stable altitude below the threshold only; it does not establish contact or
touchdown. `FAILED` exits with status 1;
interrupt exits with status 130. The transient-local topic retains the phase
history for a collector that joins after mission startup. This status stream is evidence for later
single-run orchestration; launching the mission separately still does not bind
it to a telemetry capture.

A mapping-path mission refuses its first odometry sample if the measured
altitude exceeds `mapping_start_max_altitude_m` (default 0.15 m). It publishes
`FAILED` and zero velocity without publishing `enable`; ordinary non-mapping
missions retain their existing already-airborne behavior. This vertical bound
does not validate x/y start position or replace a fresh-world check.

Before the first odometry message arrives, the mission uses a 10-second wall
clock watchdog (`odometry_startup_timeout_s`). If odometry does not arrive, it
publishes a `FAILED` status, sends only a zero velocity command, and exits
nonzero. This watchdog continues to run if simulation time is paused; it does
not by itself diagnose why the ROS odometry path failed. Override the bound
with `-p odometry_startup_timeout_s:=<seconds>` when starting `sim_mission`.

Mission odometry stamps and `now` are compared in the ROS clock domain (the
simulation clock when `use_sim_time:=True`). A sample may be at most 100 ms
ahead of that clock; equal stamps are allowed, while regression or data older
than `odometry_timeout_s` fails the mission before it emits a flight command.
While simulation time is paused, equal-time samples remain valid and age does
not advance. This is a software contract covered by model tests; it does not
substitute for live ROS failure-injection evidence.

```sh
uv run --no-project --python /usr/bin/python3 \
  /opt/ros/jazzy/bin/ros2 launch nano_c_slam_gazebo swarm_mission.launch.py
```

Then inspect telemetry with `ros2 topic echo /nano_c_slam/sim/cf1/ranges_mm` and
`ros2 topic echo /nano_c_slam/sim/uwb_pairwise_range_m`. This exercises the
repository's velocity-command mission and sensor adapters; it does not validate
Nano-C-SLAM mapping or the embedded firmware.

The pure sensor conversion and flight-sequence tests run without ROS or flight hardware:

```sh
uv run --locked python -m unittest tests.test_gazebo_sensor_model tests.test_gazebo_mission
```

## Native macOS (Homebrew ROS 2 Jazzy)

The Linux instructions above are the supported upstream path. On macOS, use the Gazebo Harmonic build already installed at `~/Work/gz-macOS/ws/install`; it provides the working native Gazebo server and desktop GUI. ROS 2 Jazzy still comes from the Homebrew tap and needs Python 3.11. The helper temporarily exposes the older Abseil ABI required by the prebuilt ROS bridge, then restores Homebrew when Gazebo exits. Do not run Homebrew upgrades while the helper is running.

If Gazebo reports that its DART engine cannot load a versioned DART library
after Homebrew updates `dartsim`, rebuild only the Gazebo DART wrapper against
the installed DART version in this repository's ignored `build/` directory:

```sh
cd ~/Work/Nano-C-SLAM
source "$HOME/Work/gz-macOS/ws/install/setup.zsh"
./scripts/build_gz_physics_dartsim_plugin.sh
```

The script reads `~/Work/gz-macOS/gz-physics` and writes no files there. It
prints an engine path containing the rebuilt plugin and the existing Gazebo
plugins. Export the printed `NANO_C_SLAM_GZ_PHYSICS_ENGINE_PATH` and
`NANO_C_SLAM_GZ_PHYSICS_ENGINE` values before running
`scripts/run_gazebo_macos.sh`. The plugin is copied under a unique name because
Gazebo otherwise prefers its built-in DART plugin even when an alternate path
is configured. The default engine path and selection remain the installed
Gazebo plugins. Retain the build receipt with any test evidence that uses the
rebuilt wrapper, since its hash and DART version identify that runtime.

Only one `run_gazebo_macos.sh` launch may be active at a time because the
Abseil compatibility link is shared through Homebrew's global `opt` directory.
The launcher rejects a second invocation while that link points into another
Nano-C-SLAM launch's temporary directory; wait for the active launch to exit.
For this same-machine GUI/server setup, the launcher advertises Gazebo
Transport on `127.0.0.1` by default to avoid ambiguous macOS network interfaces.
Set `GZ_IP` to another local interface only when connecting a remote Gazebo
client; the selected address is recorded in run provenance.

Install Python 3.11.9 from [python.org](https://www.python.org/downloads/release/python-3119/) (the final 3.11 macOS installer), then install the ROS 2 Jazzy formula and its dependencies as described by [idesign0/homebrew-ros2](https://github.com/idesign0/homebrew-ros2). The Homebrew formula's Python is not compatible with the prebuilt ROS bindings; use the python.org interpreter below. Install the remaining libraries and the historical Abseil bottle:

```sh
brew install libusb
brew tap-new vlordier/ros-compat
brew extract --version=20260107.0 abseil vlordier/ros-compat
brew install vlordier/ros-compat/abseil@20260107.0
uv pip install --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  'empy==3.3.4' 'lark==1.1.1' catkin_pkg netifaces osrf-pycommon PyYAML packaging \
  psutil typeguard jinja2 lxml numpy argcomplete colcon-common-extensions rowan \
  nicegui cflib transforms3d
```

Clone the Crazyswarm2 Gazebo backend and build the Crazyflie firmware bindings (the latter provide the Python module expected by Crazyswarm2):

```sh
git clone --recursive https://github.com/prakash-aryan/crazyswarm2_gazebo.git \
  ~/Work/crazyswarm2_gazebo
cd ~/Work/crazyswarm2_gazebo/src/crazyswarm2
git submodule update --init --recursive
cd ~/Work/Nano-C-SLAM/crazyflie-firmware
make cf2_defconfig
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  make bindings_python PYTHON=/Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11
```

Build the ROS workspace and this repository's bridge. The native build excludes `motion_capture_tracking` (not needed by the Gazebo backend). Homebrew libusb does not ship CMake metadata, so this repository supplies a tiny `libusbConfig.cmake`; the C++ driver still builds against Homebrew's libusb. Eigen's Homebrew include path is passed explicitly:

```sh
cd ~/Work/crazyswarm2_gazebo
export PATH=/Library/Frameworks/Python.framework/Versions/3.11/bin:$PATH
export CXXFLAGS='-I/opt/homebrew/opt/eigen/include/eigen3'
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  colcon build --base-paths src "$HOME/Work/Nano-C-SLAM/sim/gazebo/nano_c_slam_interfaces" \
    "$HOME/Work/Nano-C-SLAM/sim/gazebo/nano_c_slam_gazebo" \
  --packages-skip motion_capture_tracking \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  -DCMAKE_CXX_FLAGS=-I/opt/homebrew/opt/eigen/include/eigen3 \
  -Dlibusb_DIR="$HOME/Work/Nano-C-SLAM/scripts/macos" \
  '-DCMAKE_EXE_LINKER_FLAGS=-framework CoreFoundation -framework Security -framework IOKit'
```

Launch the integrated Gazebo world, ROS-Gazebo bridge, Nano-C-SLAM sensor adapter, and the separate Gazebo desktop GUI from the repository root. The helper sets ROS domain 42 to keep this graph separate from other local ROS projects. The GUI process joins the running server, following the `gz-macOS` setup's documented `gz sim -s` / `gz sim -g` split. Server-side sensor rendering uses the same Metal headless-rendering flags as the working `~/Work` macOS Gazebo launch. macOS may still show a separate blank `OgreWindow(0)_0`: it is OGRE's auxiliary 1×1 render window for sensors, not the Gazebo viewport. Use the **Gazebo Sim** window for the scene. This helper window is associated with an upstream macOS Cocoa threading limitation in the sensor renderer:

```sh
cd ~/Work/Nano-C-SLAM
./scripts/run_gazebo_macos.sh
```

To try a local colcon build without replacing packages in the Crazyswarm2
workspace, set `NANO_C_SLAM_OVERLAY_SETUP` to that build's `local_setup.bash`.
The helper sources the overlay after ROS, Crazyswarm2, and Gazebo.

For repeatable development builds, first source ROS Jazzy, Gazebo, and the
Crazyswarm2 workspace, then run:

```sh
cd ~/Work/Nano-C-SLAM
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source "$HOME/Work/gz-macOS/ws/install/setup.zsh"
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
./scripts/build_gazebo_ros_overlay.sh
```

The script creates a unique build/install/log tree under ignored `build/`, so
colcon never cleans the external Crazyswarm install. When the two
`ros_gz_crazyflie` package sources are present, it builds them into the same
overlay as the Nano-C-SLAM packages. This repairs missing package setup files
without rewriting `~/Work/crazyswarm2_gazebo/install`. A before/after source
snapshot records the Crazyswarm workspace revision, submodule revisions,
dirty/untracked paths, and source file hashes; the build fails if those inputs
change while colcon runs. The script prints the resulting `local_setup.bash`
path; use it for `NANO_C_SLAM_OVERLAY_SETUP` when launching:

```sh
NANO_C_SLAM_OVERLAY_SETUP="$HOME/Work/Nano-C-SLAM/build/gazebo-ros-overlay.<id>/install/local_setup.bash" \
  ./scripts/run_gazebo_macos.sh
```

Keep an existing Gazebo launch separate from a fresh acceptance run. The
launcher and build script do not stop or reuse existing simulators.

For the firmware-range host mapping run, generate a temporary near-field variant
of the upstream world so the arena walls and mapping obstacles return points
inside the unchanged 1.4 m gate:

```sh
cd ~/Work/Nano-C-SLAM
NANO_C_SLAM_NEARFIELD_MAPPING=1 ./scripts/run_gazebo_macos.sh
```

The launcher derives the world from the installed Crazyswarm2 SDF and keeps the
generated file in its temporary directory. The upstream checkout is not edited.

The independent geometry scenario uses different wall and obstacle positions
and a five-waypoint asymmetric path. Build the Nano-C-SLAM ROS overlay first,
then start a separate world with a dedicated partition and ROS domain:

```sh
cd ~/Work/Nano-C-SLAM
export NANO_C_SLAM_OVERLAY_SETUP="$PWD/build/gazebo-ros-overlay.<id>/install/local_setup.bash"
export NANO_C_SLAM_NEARFIELD_MAPPING=1
export NANO_C_SLAM_NEARFIELD_VARIANT=heldout
export NANO_C_SLAM_GZ_PARTITION=nano_c_slam_mapping_heldout
export NANO_C_SLAM_ROS_DOMAIN_ID=69
export GZ_DISCOVERY_MSG_PORT=45327
export GZ_DISCOVERY_SRV_PORT=45328
./scripts/run_gazebo_macos.sh
```

Once the server is ready, archive the exact SDF and active world session before
running the capture. In a second terminal, source the same ROS, Crazyswarm2,
Gazebo, and Nano-C-SLAM overlay setup files, then use the matching partition and
domain:

```sh
cd ~/Work/Nano-C-SLAM
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
source ~/Work/gz-macOS/ws/install/local_setup.zsh
source "$PWD/build/gazebo-ros-overlay.<id>/install/local_setup.zsh"
export GZ_PARTITION=nano_c_slam_mapping_heldout
export ROS_DOMAIN_ID=69
artifact_dir="$PWD/artifacts/gap9_gvsoc_evidence/heldout-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$artifact_dir"
uv run --locked python -m scripts.archive_gazebo_world_evidence \
  --partition "$GZ_PARTITION" --output-dir "$artifact_dir/world"
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  -m scripts.gazebo_gvsoc_live --mapping-route heldout-asymmetric \
  --output "$artifact_dir/gazebo-run.json"
```

The archive command fails if the launched SDF no longer matches the session
hash, if the session does not identify exactly one live Gazebo server, or if
the output directory already exists. Use the archived `world/world.sdf`,
`world/world-session.json`, and `world/world-input-attestation.json` with the
geometry analyzer and pass `--expected-route heldout-asymmetric`; the analyzer
checks the route recorded in the mission argv. A successful capture still
produces measurements only. Acceptance thresholds remain unset until the
baseline repeatability work in bead `ncs-fea.4.8` is complete.

In a second terminal, run the single-drone example mission (takeoff to 0.5 m, hover for 3 seconds, then land). The mission continuously enables the simulated Crazyflie, sends vertical velocity commands through the Gazebo bridge, and requires fresh measured odometry. It holds a zero vertical command and reports failure if odometry becomes stale or inconsistent, or if takeoff or landing exceeds its timeout:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
source ~/Work/gz-macOS/ws/install/setup.zsh
overlay_install="$(ls -td build/gazebo-ros-overlay.*/install | head -1)"
source "$overlay_install/local_setup.zsh"
export GZ_PARTITION="nano_c_slam_$(id -u)"
export ROS_DOMAIN_ID=42
export GZ_IP=127.0.0.1
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 run nano_c_slam_gazebo sim_mission --ros-args -p use_sim_time:=True
```

To fly all three Crazyflies in the world at once, run the swarm launch instead:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
export ROS_DOMAIN_ID=42
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 launch nano_c_slam_gazebo swarm_mission.launch.py
```

The swarm launch keeps spatial paths disabled by default. To opt all three
vehicles into their bounded square paths, pass `mapping_path:=true`:

```sh
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 launch nano_c_slam_gazebo swarm_mission.launch.py mapping_path:=true
```

For a future combined swarm evidence bundle, pass one canonical UUID as
`run_id:=<uuid>` so all three mission status streams share the same run
identity. If omitted, each mission generates its own UUID; that remains valid
for an uncoordinated demonstration but cannot bind the three status streams as
one run. The standalone mission supervisor below assigns a shared ID and
records process outcomes; synchronized telemetry and spacing evidence remain a
separate step.

To retain a standalone mission-phase report for three concurrent processes,
source ROS/Crazyswarm and use the same ROS domain and Gazebo partition as the
running world, then run the supervisor against the installed world SDF:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
export ROS_DOMAIN_ID=42
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  -m scripts.gazebo_swarm_mission_run \
  --world ~/Work/crazyswarm2_gazebo/install/ros_gz_crazyflie_gazebo/share/ros_gz_crazyflie_gazebo/worlds/crazyflie_swarm_world.sdf \
  --output /tmp/nano-c-slam-swarm-mission.json
```

The supervisor checks the planned routes, launches one `sim_mission` process
per vehicle with a shared UUID, retains separate stdout/stderr logs and exit
codes, and writes a report even when terminal evidence fails validation. A
passing report establishes mission status only; it does not contain synchronized
spacing telemetry or prove collision-free flight. Do not run this alongside
another mission using the same ROS domain and world.

Before creating any mission node, the swarm launch reads `cf1`, `cf3`, and
`cf4` spawn XY positions from the installed
`ros_gz_crazyflie_gazebo/share/ros_gz_crazyflie_gazebo/worlds/crazyflie_swarm_world.sdf`
and checks the continuous projected paths defined by the owned
`MappingPath`. It allows each vehicle to be at any point on its path
independently, so the check does not assume synchronized takeoff, waypoint
progress, or landing. Current pairwise minimum horizontal center distances are
0.75 m (`cf1/cf3`), 1.061 m (`cf1/cf4`), and 0.75 m (`cf3/cf4`). All exceed
the nominal 0.50 m planned-separation target. The SDF body collision is a
0.10 m square, so that target is five body widths. Horizontal distance is a
lower bound on 3D center distance during takeoff, hover, path, and landing.

This is a deterministic planned-route check, not a physical safety margin. It
does not model rotor extent, attitude, controller tracking error, localization
error, command/time skew, or collision dynamics. It must not be used to claim
collision-free physical flight.

Separation policy for the current Gazebo mission is **offline verification**.
The passive trajectory checker must fail the run evidence if any synchronized
pairwise center distance falls below 0.50 m, but it does not send stop commands
or protect vehicles while the mission is running. Report a passing run as
“spacing remained above 0.50 m in this Gazebo run,” never as runtime
separation protection or collision-free flight. Adding a reactive swarm
monitor requires a separate design and failure-injection validation.

On successful completion the mission process publishes one final zero-velocity
command and exits. On phase failure it publishes a zero-velocity command,
publishes `FAILED`, and exits nonzero. The node does not currently prove that a
zero command remains active after process exit, and this simulator behavior has
not been failure-injection validated; do not interpret it as a verified hold,
landing, or hardware safety response. See beads `ncs-ju6.6.8` and
`ncs-ju6.6.10` for the decision and live validation work.

Check sensor output while the first terminal is running:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
export ROS_DOMAIN_ID=42
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 topic echo /nano_c_slam/sim/cf1/ranges_mm
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 topic echo /nano_c_slam/sim/uwb_pairwise_range_m
```

This confirms native ROS 2, the `~/Work/gz-macOS` Gazebo build and desktop GUI, simulated flight commands, and synthetic bridge messages. The four-ray lidar and ideal odometry-derived ranges do not emulate the hardware ToF matrices or UWB ranging protocol. The mapping-core replay below exercises the owned C SLAM implementation on the host; neither procedure runs the GAP9 firmware entry point or validates mapping on hardware.

To verify that the native ROS graph and all telemetry streams are live without
starting another mission, source the same ROS and workspace setup files and run:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
export ROS_DOMAIN_ID=42
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  -m scripts.verify_gazebo_swarm_ros --timeout 20
```

This passive check validates advancing odometry and correctly namespaced,
fresh four-ray ranges from cf1, cf3 and cf4, then checks timestamped pairwise
ranges against the exact source odometry samples. Add
`--require-mission-commands` when mission nodes are already running to verify
`/<drone>/cmd_vel` graph endpoints. It never launches a mission or publishes
flight commands. Run it from the Nano-C-SLAM repository root.

To isolate ROS 2/DDS discovery from Gazebo, Crazyswarm2, and the Nano overlay,
open a separate terminal and source only the ROS Jazzy underlay. This bounded
canary selects an unused domain (or accepts an explicitly unused one), starts
its own `std_msgs/msg/String` subscriber and publisher, and verifies the nonce,
topic type, and both nodes with the ROS CLI:

```sh
cd ~/Work/Nano-C-SLAM
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
uv run --no-project \
  --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  -m scripts.probe_ros2_graph --timeout 30
```

The command writes JSON evidence under `artifacts/ros2_discovery_probe_*.json`.
Use `--domain-id <0..232>` to select a particular unused domain and
`--compare-subnet` to repeat a successful `LOCALHOST` trial with only
`ROS_AUTOMATIC_DISCOVERY_RANGE` changed to `SUBNET`. A successful result proves
that this ROS installation can exchange a standard message and expose its
graph on the tested domain; it does not identify the cause of historical
domain-43 hangs or validate Gazebo, the Nano bridge, mapping, or firmware.

To exercise the same ROS subscriptions and validators without Gazebo, run:

```sh
uv run --no-project \\
  --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \\
  -m scripts.verify_gazebo_swarm_ros --self-test
```

The fixture publishes all three namespaced streams and pairwise snapshots on
an isolated local ROS domain; it does not publish velocity commands.

To test the mission's native ROS callback contract without Gazebo, source only
ROS Jazzy and run the isolated probe. It starts the owned mission node plus
controlled `/clock` and `/cf1/odom` publishers/subscribers in a fresh ROS
domain. The first odometry sample is exactly 100 ms ahead of simulation time
and must reach TAKEOFF; a later sample 101 ms ahead must publish FAILED with
`odometry timestamp is ahead of simulation time`, publish zero vertical
velocity, and exit with status 1:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
ROS_PYTHON="$(head -n 1 "$(command -v ros2)" | sed 's/^#!//')"
uv run --no-project --python "$ROS_PYTHON" \
  scripts/verify_ros_mission_future_odom.py
```

The ROS formula's Python ABI differs from this repository's locked Python
environment, so this integration probe uses the interpreter named by the
installed `ros2` executable through `uv`; it does not install Python packages.
The result is native ROS node contract evidence only. It does not start
Gazebo, validate simulated flight, or establish mapping behavior.

### Replay Gazebo data through the Nano-C-SLAM mapping core

The collector's ROS callback, timer, freshness rejection, timeout diagnostics,
and no-success-artifact behavior can also be exercised without starting
Gazebo. After sourcing ROS and a workspace containing the Nano-C-SLAM
interfaces and bridge packages, run the controlled publisher fixture:

```sh
cd ~/Work/Nano-C-SLAM
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
uv run --no-project \
  --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  python scripts/verify_gazebo_collector_ros.py \
  --ros-python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11
```

The fixture uses an isolated local ROS domain. It captures 60 fresh pairs, then
verifies that stale, future-dated, wrong-drone, and out-of-order streams fail
with named rejection counters and no passing report. The pure pairer tests
cover inclusive age/skew boundaries and over-limit skew. This is a ROS collector
contract test, not a Gazebo freshness measurement.

With the near-field Gazebo instance and sensor bridge running, use the
single-run coordinator to start the collector before the mission, wait for
matching ROS subscriptions, capture only after that run begins, and replay the
telemetry through the owned C point-cloud, ICP, and pose-graph code:

```sh
cd ~/Work/Nano-C-SLAM
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
export ROS_DOMAIN_ID=42
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  -m scripts.gazebo_mapping_run --drone cf1 --samples 60 \
  --scene nearfield-mapping --timeout 120 --preflight-timeout 10 \
  --output /tmp/nano-c-slam-gazebo-run.json
```

Before it starts either child process, the coordinator requires three distinct
`/cf1/odom` callbacks from the live simulation clock. Each source stamp must be
fresh and increasing, callback and source-stamp gaps must stay at or below
0.2 seconds, and every altitude must be finite and at or below 0.15 m. The
wall-clock `--preflight-timeout` bounds startup when the world is paused, the
clock is unavailable, or odometry is absent. The exact accepted stamps,
altitudes, and simulation/receipt times are stored in `odometry_preflight` in
the combined report; the offline bundle verifier rechecks them.

The command generates one UUID for both processes, requires a completed bounded
square path and stable odometry altitude at or below 0.1 m, and writes the combined
report only after both processes and host replay pass. Child logs remain beside
the report in a hidden run-scoped directory. The collector requires at least
60 fresh samples (three 20-pose scan windows) and reports success only when it
obtains finite map points, at least two
unique edges submitted to `ls_slam`, a finite three-node graph, successful
solver calls, and complete fixed iteration counts. The report names poses
`finite_pose_count` because it records finite reconstructed output; it does
not claim convergence or objective improvement. See
[`CONSTRAINT_ACCOUNTING.md`](../../cslam-gap9-app/point-cloud/CONSTRAINT_ACCOUNTING.md)
for edge accounting and
[`SCAN_OBSERVABILITY.md`](../../cslam-gap9-app/point-cloud/SCAN_OBSERVABILITY.md)
for the fixture-derived per-window spread check. The retained run
had at least three directions within the cutoff in 19–20 poses per window, and
each of the four directions had multiple in-range values over the capture; it
does not claim every beam is valid in every pose. The JSON report contains raw
sample records, per-direction range accounting, finite pose estimates, exact
constraint endpoint accounting, and projected map points. Current coordinator
reports use schema v2 and include SHA-256 for the owned capture/mission/mapping
inputs, Python/ROS 2/Gazebo executables, host replay binary, and installed
Nano-C-SLAM interface/mission and Crazyswarm2 Gazebo packages. The coordinator
compares these identities before and after capture and refuses to publish a
passing report if any change during the run. Earlier schema-v1 reports remain
useful run evidence but are historical and lack complete tool/overlay binding.
On Linux, use the ROS and workspace setup paths from the Linux
section and replace the Python path with `/usr/bin/python3`.

Treat this as one mapping run per freshly launched world. The launch holds an
exclusive lock on its Gazebo partition and writes a UUID session manifest.
Starting a second Nano-C-SLAM launch on the same partition fails before Gazebo
starts. Before starting the collector or mission, the coordinator atomically
claims the session; a second mapping invocation fails before takeoff and asks
you to restart the Gazebo launch. A repeated-world run previously collected 60
samples and failed the per-window observability gate, so the coordinator wrote
no passing report.
There is no validated world-frame safe-start envelope and no Gazebo reset in
this workflow. The session guard prevents repeated runs through this
coordinator; it cannot detect a drone moved manually or by another controller.

When diagnosing missing mission odometry, use a dedicated ROS domain and
Gazebo partition, and build a fresh owned-package overlay first. Set
`NANO_C_SLAM_ROS_DOMAIN_ID` and `NANO_C_SLAM_GZ_PARTITION` before starting
`run_gazebo_macos.sh`, then set `NANO_C_SLAM_OVERLAY_SETUP` to the resulting
overlay's `install/local_setup.bash`. Confirm the launch PID's environment and
the `ros2 pkg prefix nano_c_slam_gazebo` path before running a mission. The
mission logs `odometry_callbacks` and the last odometry source stamp when it
leaves `WAITING`; topic visibility in a separate `ros2 topic echo` process is
not proof that this callback ran. The 10-second startup watchdog fails closed
without odometry. Do not launch a second native Gazebo server beside another
active test instance just to gather this evidence.

To take a passive, bounded snapshot of the current ROS domain without starting
a mission, source the same ROS, Crazyswarm2, and Gazebo workspaces as the active
launch, set its `ROS_DOMAIN_ID`, and run:

```sh
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  scripts/diagnose_gazebo_odom.py --drone cf1 --duration 5 \
  --output artifacts/gazebo_mapping_sitl/odom-diagnostic.json
```

The JSON records the probe's received `/cf1/odom` callback count and source
stamp span, `/clock` span, ROS node names, odometry publisher QoS, and domain
metadata. A positive probe count proves delivery to this diagnostic subscriber
only; mission callback evidence still comes from the mission log.

For a bounded spatial mapping pass, enable the optional square path. It moves
0.25 m along each side at no more than 0.25 m/s, holds at the target altitude,
then performs the configured hover and landing. The square is relative to the
measured takeoff position and each waypoint has an eight-second timeout:

```sh
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 run nano_c_slam_gazebo sim_mission --ros-args \
  -p use_sim_time:=True -p mapping_path:=True
```

The collector records paired source timestamps, skew, range samples, x/y bounds,
cumulative horizontal travel, and accepted/rejected readings at the cutoff. It
does not infer mission success from a command-line flag; automated mission
outcome evidence is tracked separately.
Build the host replay before flight and pass
`--reuse-binary` to avoid compiling while Gazebo is running:

```sh
scripts/build_host_mapping_sitl.sh
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  -m scripts.gazebo_mapping_sitl --drone cf1 --samples 60 \
  --scene nearfield-mapping --reuse-binary \
  --output /tmp/nano-c-slam-gazebo-map.json
```

Run the collector concurrently to capture the spatial telemetry and replay it
through the host mapping core. The default upstream arena is too large for the
1.4 m gate and can correctly produce no points; the near-field option changes
only scene geometry, not range values or the firmware cutoff. The older
`cf1_native_gazebo_mapping_path.json` is a broad-range historical baseline and
must not be treated as firmware-range evidence.

This is host execution of the repository's GAP9 point-cloud, ICP, and graph
optimization C modules. PMSIS cluster calls run through serial CPU stubs. A
successful near-field run demonstrates pipeline execution with the 1.4 m
range gate; it does not measure map accuracy, execute GAP9 `main.c` or SPI,
model the hardware 8x8 ToF matrices/UWB, or validate flight firmware. Each
report distinguishes accepted ICP results from constraints actually stored and
passed to graph optimization. The retained first near-field capture is
[`artifacts/gazebo_mapping_sitl/cf1_nearfield_firmware_cutoff.json`](../../artifacts/gazebo_mapping_sitl/cf1_nearfield_firmware_cutoff.json).
