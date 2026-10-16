from launch import LaunchDescription
from launch_ros.actions import Node
from nano_c_slam_gazebo.mission_model import SUPPORTED_DRONE_NAMES


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="nano_c_slam_gazebo",
                executable="sensor_bridge",
                name="nano_c_slam_sensor_bridge",
                output="screen",
                parameters=[
                    {"drone_names": list(SUPPORTED_DRONE_NAMES), "use_sim_time": True}
                ],
            )
        ]
    )
