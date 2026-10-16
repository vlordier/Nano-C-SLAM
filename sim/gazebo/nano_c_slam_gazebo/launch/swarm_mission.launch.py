"""Run the bounded flight mission on every Crazyflie in the Gazebo world."""

from launch import LaunchDescription
from launch_ros.actions import Node
from nano_c_slam_gazebo.mission_model import SUPPORTED_DRONE_NAMES


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="nano_c_slam_gazebo",
                executable="sim_mission",
                name=f"{drone_name}_sim_mission",
                output="screen",
                parameters=[{"drone_name": drone_name, "use_sim_time": True}],
            )
            for drone_name in SUPPORTED_DRONE_NAMES
        ]
    )
