"""Launch the Gazebo Crazyflie SITL world and Nano-C-SLAM telemetry on macOS."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from nano_c_slam_gazebo.world_session_model import (
    acquire_world_session_lock,
    publish_world_session,
    record_world_process_event,
    session_file_for_partition,
)

_WORLD_SESSION_LOCK = None


def _publish_session(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Bind one single-run token to this launch before Gazebo starts."""
    world = LaunchConfiguration("world").perform(context)
    scene = (
        "nearfield-mapping"
        if Path(world).name == "nano_c_slam_nearfield.sdf"
        else "default"
    )
    partition = os.environ.get("GZ_PARTITION", "default")
    session_path = session_file_for_partition(partition)
    global _WORLD_SESSION_LOCK
    _WORLD_SESSION_LOCK = acquire_world_session_lock(session_path)
    publish_world_session(session_path, scene=scene)
    return []


def _record_started_process(
    event: ProcessStarted, context: LaunchContext
) -> list[LaunchDescriptionEntity]:
    """Record only processes started by this launch with known runtime roles."""
    del context
    partition = os.environ.get("GZ_PARTITION", "default")
    record_world_process_event(
        session_file_for_partition(partition),
        pid=event.pid,
        argv=event.cmd,
    )
    return []


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("ros_gz_crazyflie_bringup")
    gazebo_share = Path(get_package_share_directory("ros_gz_crazyflie_gazebo"))
    sim_share = get_package_share_directory("ros_gz_sim")
    default_world = str(gazebo_share / "worlds/crazyflie_swarm_world.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                TextSubstitution(
                    text=(
                        " -r -s --headless-rendering --render-engine-server=ogre2 "
                        "--render-engine-server-api-backend=metal"
                    )
                ),
            ]
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
    sensor_bridge = Node(
        package="nano_c_slam_gazebo",
        executable="sensor_bridge",
        name="nano_c_slam_sensor_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            OpaqueFunction(function=_publish_session),
            RegisterEventHandler(OnProcessStart(on_start=_record_started_process)),
            gazebo,
            gazebo_gui_after_server,
            gz_bridge,
            sensor_bridge,
        ]
    )
