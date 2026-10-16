"""Launch the Gazebo Crazyflie SITL world and Nano-C-SLAM telemetry on macOS."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from nano_c_slam_gazebo.mission_model import SUPPORTED_DRONE_NAMES


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("ros_gz_crazyflie_bringup")
    gazebo_share = Path(get_package_share_directory("ros_gz_crazyflie_gazebo"))
    sim_share = get_package_share_directory("ros_gz_sim")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [
                    gazebo_share,
                    "worlds/crazyflie_swarm_world.sdf -r -s --headless-rendering "
                    "--render-engine-server=ogre2 "
                    "--render-engine-server-api-backend=metal",
                ]
            )
        }.items(),
    )
    gazebo_gui = ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen")
    gazebo_gui_after_server = TimerAction(period=8.0, actions=[gazebo_gui])

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [bringup_share, "config", "ros_gz_crazyflie_swarm_bridge.yaml"]
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
        parameters=[{"drone_names": list(SUPPORTED_DRONE_NAMES), "use_sim_time": True}],
        output="screen",
    )

    sensor_bridge = Node(
        package="nano_c_slam_gazebo",
        executable="sensor_bridge",
        name="nano_c_slam_sensor_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            gazebo_gui_after_server,
            gz_bridge,
            odom_tf,
            sensor_bridge,
        ]
    )
