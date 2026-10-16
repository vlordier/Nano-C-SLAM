from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="nano_c_slam_gazebo",
                executable="sensor_bridge",
                name="nano_c_slam_sensor_bridge",
                output="screen",
                parameters=[{"drone_names": ["cf1", "cf3", "cf4"]}],
            )
        ]
    )
