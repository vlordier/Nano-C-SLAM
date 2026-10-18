"""Run the bounded flight mission on every Crazyflie in the Gazebo world."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nano_c_slam_gazebo.mission_model import SUPPORTED_DRONE_NAMES
from nano_c_slam_gazebo.swarm_geometry import (
    read_spawn_positions,
    validate_default_swarm_routes,
)


def generate_launch_description() -> LaunchDescription:
    world_path = (
        Path(get_package_share_directory("ros_gz_crazyflie_gazebo"))
        / "worlds/crazyflie_swarm_world.sdf"
    )
    validate_default_swarm_routes(read_spawn_positions(world_path))
    nodes = [
        Node(
            package="nano_c_slam_gazebo",
            executable="sim_mission",
            name=f"{drone_name}_sim_mission",
            output="screen",
            parameters=[
                {"drone_name": drone_name, "use_sim_time": True},
                {
                    "mapping_path": ParameterValue(
                        LaunchConfiguration("mapping_path"), value_type=bool
                    )
                },
            ],
        )
        for drone_name in SUPPORTED_DRONE_NAMES
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping_path",
                default_value="false",
                description=(
                    "Enable each drone's bounded spatial square path during the "
                    "mapping mission"
                ),
            ),
            *nodes,
        ]
    )
