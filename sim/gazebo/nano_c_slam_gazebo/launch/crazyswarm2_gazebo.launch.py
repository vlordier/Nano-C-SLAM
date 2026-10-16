"""Launch Crazyswarm2 SITL, Gazebo, and Nano-C-SLAM telemetry on macOS."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

DRONE_NAMES = ["cf1", "cf3", "cf4"]


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("ros_gz_crazyflie_bringup"))
    gazebo_share = Path(get_package_share_directory("ros_gz_crazyflie_gazebo"))
    sim_share = Path(get_package_share_directory("ros_gz_sim"))
    crazyflie_share = Path(get_package_share_directory("crazyflie"))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [
                    str(gazebo_share),
                    "worlds/crazyflie_swarm_world.sdf -r -s "
                    "--physics-engine gz-physics-bullet-featherstone-plugin",
                ]
            )
        }.items(),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": str(
                    bringup_share / "config" / "ros_gz_crazyflie_swarm_bridge.yaml"
                ),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    odom_tf = Node(
        package="ros_gz_crazyflie_control",
        executable="odom_tf_publisher",
        name="odom_tf_publisher",
        parameters=[{"drone_names": DRONE_NAMES, "use_sim_time": True}],
        output="screen",
    )

    crazyflies = yaml.safe_load(
        (crazyflie_share / "config/crazyflies.yaml").read_text()
    )
    server_params = yaml.safe_load(
        (crazyflie_share / "config/server.yaml").read_text()
    )["/crazyflie_server"]["ros__parameters"]
    server_params["robot_description"] = (
        crazyflie_share / "urdf/crazyflie_description.urdf"
    ).read_text()

    crazyflie_server = Node(
        package="crazyflie_sim",
        executable="crazyflie_server",
        name="crazyflie_server",
        parameters=[crazyflies, server_params, {"use_sim_time": True}],
        output="screen",
        emulate_tty=True,
    )
    sensor_bridge = Node(
        package="nano_c_slam_gazebo",
        executable="sensor_bridge",
        name="nano_c_slam_sensor_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [gazebo, gz_bridge, odom_tf, crazyflie_server, sensor_bridge]
    )
