# Crazyswarm2 and Gazebo SITL

This integration uses the [Crazyswarm2 Gazebo backend](https://github.com/prakash-aryan/crazyswarm2_gazebo), which runs on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. The upstream backend provides the Crazyflie physics, `cmd_vel` control path, odometry, and a four-ray multiranger scan.

The `nano_c_slam_gazebo` ROS package adds two telemetry topics:

- `/nano_c_slam/sim/<drone>/ranges_mm`: four distances in `front,back,left,right` order, with `-1` for invalid readings.
- `/nano_c_slam/sim/uwb_pairwise_range_m`: ideal 3D distances computed from Gazebo odometry. The `Float32MultiArray` dimension label identifies each pair.

These are simulation adapters. The Gazebo lidar has four rays, while the hardware uses four 8x8 ToF matrices; the pairwise ranges use ground-truth simulator odometry, with no UWB noise, packet loss, or ranging protocol. The bridge does not run the GAP9 SLAM task or the STM32 firmware. The existing `cslam-stm32-app/run_demo.py` still uses `cflib` and a Crazyradio, so it cannot be run unchanged against Crazyswarm2's ROS 2 simulator.

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

The pure conversion tests run without ROS or flight hardware:

```sh
uv run --locked python -m unittest tests.test_gazebo_sensor_model
```
