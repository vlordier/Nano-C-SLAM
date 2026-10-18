"""Check the swarm launch forwards the opt-in path flag to every vehicle."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import cast
from unittest.mock import patch

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = (
    REPOSITORY_ROOT / "sim/gazebo/nano_c_slam_gazebo/launch/swarm_mission.launch.py"
)


class FakeLaunchDescription:
    def __init__(self, entities: list[object]) -> None:
        self.entities = entities


class FakeDeclareLaunchArgument:
    def __init__(self, name: str, *, default_value: str, description: str) -> None:
        self.name = name
        self.default_value = default_value
        self.description = description


class FakeLaunchConfiguration:
    def __init__(self, name: str) -> None:
        self.name = name


class FakeParameterValue:
    def __init__(self, value: object, *, value_type: type[object]) -> None:
        self.value = value
        self.value_type = value_type


class FakeNode:
    def __init__(self, **kwargs: object) -> None:
        self.kwargs: dict[str, object] = kwargs


class SwarmMissionLaunchTests(unittest.TestCase):
    def test_mapping_path_defaults_off_and_is_typed_for_every_drone(self) -> None:
        launch = types.ModuleType("launch")
        _set_module_attribute(launch, "LaunchDescription", FakeLaunchDescription)
        launch_actions = types.ModuleType("launch.actions")
        _set_module_attribute(
            launch_actions, "DeclareLaunchArgument", FakeDeclareLaunchArgument
        )
        launch_substitutions = types.ModuleType("launch.substitutions")
        _set_module_attribute(
            launch_substitutions, "LaunchConfiguration", FakeLaunchConfiguration
        )

        launch_ros = types.ModuleType("launch_ros")
        launch_ros_actions = types.ModuleType("launch_ros.actions")
        _set_module_attribute(launch_ros_actions, "Node", FakeNode)
        launch_ros_parameters = types.ModuleType("launch_ros.parameter_descriptions")
        _set_module_attribute(
            launch_ros_parameters, "ParameterValue", FakeParameterValue
        )

        ament = types.ModuleType("ament_index_python")
        ament_packages = types.ModuleType("ament_index_python.packages")
        _set_module_attribute(
            ament_packages, "get_package_share_directory", lambda package: "/fake/share"
        )

        mission_model = types.ModuleType("nano_c_slam_gazebo.mission_model")
        _set_module_attribute(
            mission_model, "SUPPORTED_DRONE_NAMES", ("cf1", "cf3", "cf4")
        )
        swarm_geometry = types.ModuleType("nano_c_slam_gazebo.swarm_geometry")
        _set_module_attribute(swarm_geometry, "read_spawn_positions", lambda path: {})
        _set_module_attribute(
            swarm_geometry, "validate_default_swarm_routes", lambda positions: {}
        )

        fake_modules = {
            "launch": launch,
            "launch.actions": launch_actions,
            "launch.substitutions": launch_substitutions,
            "launch_ros": launch_ros,
            "launch_ros.actions": launch_ros_actions,
            "launch_ros.parameter_descriptions": launch_ros_parameters,
            "ament_index_python": ament,
            "ament_index_python.packages": ament_packages,
            "nano_c_slam_gazebo.mission_model": mission_model,
            "nano_c_slam_gazebo.swarm_geometry": swarm_geometry,
        }

        with tempfile.TemporaryDirectory() as temporary:
            fake_world = Path(temporary) / "worlds/crazyflie_swarm_world.sdf"
            fake_world.parent.mkdir(parents=True)
            fake_world.touch()
            _set_module_attribute(
                ament_packages,
                "get_package_share_directory",
                lambda package: str(Path(temporary)),
            )
            spec = importlib.util.spec_from_file_location(
                "swarm_mission_launch_under_test", LAUNCH_FILE
            )
            self.assertIsNotNone(spec)
            assert spec is not None and spec.loader is not None
            module = importlib.util.module_from_spec(spec)
            with patch.dict(sys.modules, fake_modules):
                spec.loader.exec_module(module)
                description = module.generate_launch_description()

        self.assertIsInstance(description, FakeLaunchDescription)
        launch_description = cast(FakeLaunchDescription, description)
        argument = launch_description.entities[0]
        self.assertIsInstance(argument, FakeDeclareLaunchArgument)
        launch_argument = cast(FakeDeclareLaunchArgument, argument)
        self.assertEqual(launch_argument.name, "mapping_path")
        self.assertEqual(launch_argument.default_value, "false")
        nodes = launch_description.entities[1:]
        self.assertEqual(len(nodes), 3)
        node_actions = cast(list[FakeNode], nodes)
        self.assertEqual(
            [
                cast(list[dict[str, object]], node.kwargs["parameters"])[0][
                    "drone_name"
                ]
                for node in node_actions
            ],
            ["cf1", "cf3", "cf4"],
        )
        for node in node_actions:
            parameters = cast(list[dict[str, object]], node.kwargs["parameters"])
            mapping_parameter = parameters[1]["mapping_path"]
            self.assertIsInstance(mapping_parameter, FakeParameterValue)
            typed_parameter = cast(FakeParameterValue, mapping_parameter)
            self.assertEqual(typed_parameter.value_type, bool)
            self.assertEqual(
                cast(FakeLaunchConfiguration, typed_parameter.value).name,
                "mapping_path",
            )


def _set_module_attribute(module: types.ModuleType, name: str, value: object) -> None:
    setattr(module, name, value)


if __name__ == "__main__":
    unittest.main()
