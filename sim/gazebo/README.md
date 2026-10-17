# Gazebo SITL

This integration uses the Gazebo Harmonic Crazyflie world from [`crazyswarm2_gazebo`](https://github.com/prakash-aryan/crazyswarm2_gazebo), with ROS 2 Jazzy. The world provides Crazyflie physics, velocity control, odometry, and a four-ray multiranger scan. The native macOS mission uses measured odometry and publishes velocity commands directly to the existing `/cf1/cmd_vel` Gazebo bridge; this avoids the upstream `control_services` relay's internal flight-state cutoff. The launch keeps the world's default physics engine. We tested the Crazyswarm2 `hello_world` service path as well, but its calls were acknowledged while Gazebo odometry showed no altitude change.

The `nano_c_slam_gazebo` ROS package currently publishes two telemetry topics:

- `/nano_c_slam/sim/<drone>/ranges_mm`: `StampedRanges` with source stamp, drone identity, sequence, and four distances in `front,back,left,right` order; `-1` marks invalid readings.
- `/nano_c_slam/sim/uwb_pairwise_range_m`: ideal 3D distances computed from Gazebo odometry. The `Float32MultiArray` dimension label identifies each pair.

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

For the firmware-range host mapping run, generate a temporary near-field variant
of the upstream world so the arena walls and mapping obstacles return points
inside the unchanged 1.4 m gate:

```sh
cd ~/Work/Nano-C-SLAM
NANO_C_SLAM_NEARFIELD_MAPPING=1 ./scripts/run_gazebo_macos.sh
```

The launcher derives the world from the installed Crazyswarm2 SDF and keeps the
generated file in its temporary directory. The upstream checkout is not edited.

In a second terminal, run the single-drone example mission (takeoff to 0.5 m, hover for 3 seconds, then land). The mission continuously enables the simulated Crazyflie, sends vertical velocity commands through the Gazebo bridge, and requires fresh measured odometry. It holds a zero vertical command and reports failure if odometry becomes stale or inconsistent, or if takeoff or landing exceeds its timeout:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
export ROS_DOMAIN_ID=42
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

The mission process exits after it sends its final zero-velocity command on a
successful landing. It remains alive after a phase timeout so the vehicle stays
under a zero-velocity hold command until you stop it.

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
  python scripts/check_gazebo_macos.py --timeout 20 --mission
```

The check validates `/cf1/odom`, the four lidar values, and pairwise ranges
before and after a complete takeoff-hover-land cycle. It verifies phase order,
near-ground completion, and advancing odometry timestamps. It exits nonzero
with a diagnostic if flight or telemetry fails. Run it from the Nano-C-SLAM
repository root.

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

With the near-field Gazebo instance and sensor bridge running, start the
collector in one terminal before launching the mission in another. It samples
fresh odometry and the four-ray range topic at 10 Hz, then builds and runs the
owned C point-cloud, ICP, and pose-graph code on the captured data. The mapper
discards returns above the firmware loop-closure limit of 1.4 m:

```sh
cd ~/Work/Nano-C-SLAM
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
export ROS_DOMAIN_ID=42
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  python scripts/gazebo_mapping_sitl.py --drone cf1 --samples 60 \
  --scene nearfield-mapping \
  --output /tmp/nano-c-slam-gazebo-map.json
```

In a second terminal, run the single-drone mission from the previous section.
The collector requires at least 60 fresh samples (three 20-pose scan windows)
and reports success only when it obtains finite map points, at least two
unique edges submitted to `ls_slam`, a finite three-node graph, successful
solver calls, and complete fixed iteration counts. The report names poses
`finite_pose_count` because it records finite reconstructed output; it does
not claim convergence or objective improvement. See
[`CONSTRAINT_ACCOUNTING.md`](../../cslam-gap9-app/point-cloud/CONSTRAINT_ACCOUNTING.md)
for the edge accounting contract. The retained run
had at least three directions within the cutoff in 19–20 poses per window, and
each of the four directions had multiple in-range values over the capture; it
does not claim every beam is valid in every pose. The JSON report contains raw
sample records, per-direction range accounting, finite pose estimates, exact
constraint endpoint accounting, and projected map points. On Linux, use the ROS and workspace setup paths from the Linux
section and replace the Python path with `/usr/bin/python3`.

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
  python scripts/gazebo_mapping_sitl.py --drone cf1 --samples 60 \
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
