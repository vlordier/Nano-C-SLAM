# Gazebo SITL

This integration uses the Gazebo Harmonic Crazyflie world from [`crazyswarm2_gazebo`](https://github.com/prakash-aryan/crazyswarm2_gazebo), with ROS 2 Jazzy. The world provides Crazyflie physics, velocity control, odometry, and a four-ray multiranger scan. On macOS the launcher uses that workspace's `control_services` velocity controller and keeps the world's default physics engine. We tested the Crazyswarm2 `hello_world` service path as well, but its calls were acknowledged while Gazebo odometry showed no altitude change, so the native macOS demo uses measured-altitude velocity control instead.

The `nano_c_slam_gazebo` ROS package adds two telemetry topics:

- `/nano_c_slam/sim/<drone>/ranges_mm`: four distances in `front,back,left,right` order, with `-1` for invalid readings.
- `/nano_c_slam/sim/uwb_pairwise_range_m`: ideal 3D distances computed from Gazebo odometry. The `Float32MultiArray` dimension label identifies each pair.

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
  colcon build --base-paths src "$HOME/Work/Nano-C-SLAM/sim/gazebo/nano_c_slam_gazebo" \
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

In a third terminal, exercise the simulated flight-command path and bridge topics:

```sh
source /opt/ros/jazzy/setup.bash
source ~/Work/crazyswarm2_gazebo/install/local_setup.bash
export PYTHONPATH="$HOME/Work/crazyflie-firmware/build:$PYTHONPATH"
uv run --no-project --python /usr/bin/python3 \
  /opt/ros/jazzy/bin/ros2 run crazyflie_examples hello_world --ros-args -p use_sim_time:=True
```

Then inspect telemetry with `ros2 topic echo /nano_c_slam/sim/cf1/ranges_mm` and `ros2 topic echo /nano_c_slam/sim/uwb_pairwise_range_m`. This verifies the upstream simulated flight-command path and this bridge's sensor topics; it does not validate Nano-C-SLAM mapping or the embedded firmware.

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
  colcon build --base-paths src "$HOME/Work/Nano-C-SLAM/sim/gazebo/nano_c_slam_gazebo" \
  --packages-skip motion_capture_tracking \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  -DCMAKE_CXX_FLAGS=-I/opt/homebrew/opt/eigen/include/eigen3 \
  -Dlibusb_DIR="$HOME/Work/Nano-C-SLAM/scripts/macos" \
  '-DCMAKE_EXE_LINKER_FLAGS=-framework CoreFoundation -framework Security -framework IOKit'
```

Launch the integrated Gazebo world, ROS-Gazebo bridge, Crazyflie velocity controller, Nano-C-SLAM sensor adapter, and the separate Gazebo desktop GUI from the repository root. The GUI process joins the running server, following the `gz-macOS` setup's documented `gz sim -s` / `gz sim -g` split. Server-side sensor rendering uses the same Metal headless-rendering flags as the working `~/Work` macOS Gazebo launch. macOS may still show a separate blank `OgreWindow(0)_0`: it is OGRE's auxiliary 1×1 render window for sensors, not the Gazebo viewport. Use the **Gazebo Sim** window for the scene. This helper window is associated with an upstream macOS Cocoa threading limitation in the sensor renderer:

```sh
cd ~/Work/Nano-C-SLAM
./scripts/run_gazebo_macos.sh
```

In a second terminal, run the single-drone example mission (takeoff to 0.5 m, hover for 3 seconds, then land). The mission continuously enables the Gazebo controller, waits for measured odometry at each phase, and stops with an error if takeoff or landing exceeds its timeout:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 run nano_c_slam_gazebo sim_mission --ros-args -p use_sim_time:=True
```

Check sensor output while the first terminal is running:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 topic echo /nano_c_slam/sim/cf1/ranges_mm
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  ros2 topic echo /nano_c_slam/sim/uwb_pairwise_range_m
```

This confirms native ROS 2, the `~/Work/gz-macOS` Gazebo build and desktop GUI, simulated flight commands, and synthetic bridge messages. The four-ray lidar and ideal odometry-derived ranges do not emulate the hardware ToF matrices or UWB ranging protocol, and this does not run GAP9 SLAM or validate mapping on hardware.

To verify that the native ROS graph and all telemetry streams are live without
starting another mission, source the same ROS and workspace setup files and run:

```sh
source /opt/homebrew/opt/ros2-jazzy/setup.zsh
source ~/Work/crazyswarm2_gazebo/install/local_setup.zsh
export GZ_PARTITION="nano_c_slam_$(id -u)"
uv run --no-project --python /Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11 \
  python scripts/check_gazebo_macos.py --timeout 20
```

The check waits for `/cf1/odom`, validates the four lidar values and the
pairwise ranges, and exits nonzero with a diagnostic if the graph is not
publishing usable messages. Run it from the Nano-C-SLAM repository root.
