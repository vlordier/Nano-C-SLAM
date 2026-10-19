"""Launch the Gazebo Crazyflie SITL world and Nano-C-SLAM telemetry on macOS."""

import os
import tempfile
from pathlib import Path

import yaml
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
from launch.event_handlers import OnProcessStart, OnShutdown
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
from nano_c_slam_gazebo.bridge_config_model import split_cf1_odometry_bridge
from nano_c_slam_gazebo.physics_engine_model import gazebo_server_arguments
from nano_c_slam_gazebo.world_session_model import (
    acquire_world_session_lock,
    publish_world_session,
    record_world_process_event,
    session_file_for_partition,
)

_WORLD_SESSION_LOCK = None
_BRIDGE_CONFIG_DIRECTORY: Path | None = None


def _write_bridge_config(directory: Path, filename: str, entries: object) -> Path:
    if not isinstance(entries, list):
        raise RuntimeError("Crazyswarm2 bridge configuration must be a YAML list")
    path = directory / filename
    path.write_text(yaml.safe_dump(entries, sort_keys=False), encoding="utf-8")
    return path


def _bridge_actions(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Split cf1 odometry so a test can stop it without stopping the world."""
    global _BRIDGE_CONFIG_DIRECTORY
    bringup_share = get_package_share_directory("ros_gz_crazyflie_bringup")
    source_path = Path(bringup_share) / "config/ros_gz_crazyflie_swarm_bridge.yaml"
    try:
        source_entries = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(
            "cannot load Crazyswarm2 Gazebo bridge configuration"
        ) from error
    if not isinstance(source_entries, list) or any(
        not isinstance(entry, dict) for entry in source_entries
    ):
        raise RuntimeError("Crazyswarm2 bridge configuration is malformed")

    main_entries, odometry_entry, truth_entry = split_cf1_odometry_bridge(
        source_entries
    )
    _BRIDGE_CONFIG_DIRECTORY = Path(tempfile.mkdtemp(prefix="nano-c-slam-bridges-"))
    main_config = _write_bridge_config(
        _BRIDGE_CONFIG_DIRECTORY, "world.yaml", main_entries
    )
    odometry_config = _write_bridge_config(
        _BRIDGE_CONFIG_DIRECTORY, "cf1-odometry.yaml", [odometry_entry]
    )
    bridge_nodes = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="nano_c_slam_world_bridge",
            parameters=[{"config_file": str(main_config), "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="nano_c_slam_cf1_odom_bridge",
            parameters=[{"config_file": str(odometry_config), "use_sim_time": True}],
            output="screen",
        ),
    ]
    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[OpaqueFunction(function=_cleanup_bridge_config_directory)]
        )
    )
    actions: list[LaunchDescriptionEntity] = [cleanup, *bridge_nodes]
    if LaunchConfiguration("failure_truth_odometry").perform(context).lower() in {
        "1",
        "true",
    }:
        truth_config = _write_bridge_config(
            _BRIDGE_CONFIG_DIRECTORY, "cf1-truth-odometry.yaml", [truth_entry]
        )
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="nano_c_slam_cf1_truth_odom_bridge",
                parameters=[{"config_file": str(truth_config), "use_sim_time": True}],
                output="screen",
            )
        )
    return actions


def _cleanup_bridge_config_directory(
    context: LaunchContext,
) -> list[LaunchDescriptionEntity]:
    del context
    global _BRIDGE_CONFIG_DIRECTORY
    if _BRIDGE_CONFIG_DIRECTORY is not None:
        for path in _BRIDGE_CONFIG_DIRECTORY.iterdir():
            path.unlink(missing_ok=True)
        _BRIDGE_CONFIG_DIRECTORY.rmdir()
        _BRIDGE_CONFIG_DIRECTORY = None
    return []


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
    publish_world_session(session_path, scene=scene, world_path=Path(world))
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
    gazebo_share = Path(get_package_share_directory("ros_gz_crazyflie_gazebo"))
    sim_share = get_package_share_directory("ros_gz_sim")
    default_world = str(gazebo_share / "worlds/crazyflie_swarm_world.sdf")
    server_arguments = gazebo_server_arguments(
        os.environ.get("NANO_C_SLAM_GZ_PHYSICS_ENGINE")
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                TextSubstitution(text=server_arguments),
            ]
        }.items(),
    )
    gazebo_gui = ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen")
    gazebo_gui_after_server = TimerAction(period=8.0, actions=[gazebo_gui])

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
            DeclareLaunchArgument(
                "failure_truth_odometry",
                default_value="false",
                description=(
                    "Also bridge read-only cf1 Gazebo odometry for failure probes"
                ),
            ),
            OpaqueFunction(function=_publish_session),
            RegisterEventHandler(OnProcessStart(on_start=_record_started_process)),
            gazebo,
            gazebo_gui_after_server,
            OpaqueFunction(function=_bridge_actions),
            sensor_bridge,
        ]
    )
