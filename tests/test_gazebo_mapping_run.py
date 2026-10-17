"""Hardware-free identity and command tests for a controlled mapping run."""

from __future__ import annotations

import sys
import tempfile
import unittest
import uuid
from pathlib import Path

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_run_model import (
    MappingRunConfig,
)


class MappingRunConfigTests(unittest.TestCase):
    def test_generates_canonical_identity_and_binds_it_to_both_commands(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            config = MappingRunConfig(
                output_path=directory / "mapping-run.json",
                binary_path=directory / "mapping_replay",
                drone_name="cf3",
                sample_count=80,
                timeout_s=75.0,
            )

            commands = config.build_commands(
                python_executable=sys.executable,
                ros2_executable="ros2",
                collector_script=Path("scripts/gazebo_mapping_sitl.py"),
            )

        self.assertEqual(str(uuid.UUID(config.run_id)), config.run_id)
        self.assertIn(config.run_id, commands.collector)
        self.assertIn(f"run_id:={config.run_id}", commands.mission)
        self.assertIn("drone_name:=cf3", commands.mission)
        self.assertIn("--drone", commands.collector)
        self.assertIn("cf3", commands.collector)
        self.assertIn("mapping_path:=True", commands.mission)
        self.assertIn("--reuse-binary", commands.collector)

    def test_rejects_invalid_drone_bounds_timeout_and_run_id(self) -> None:
        invalid_configs = (
            lambda path, binary: MappingRunConfig(
                output_path=path, binary_path=binary, drone_name="cf2"
            ),
            lambda path, binary: MappingRunConfig(
                output_path=path, binary_path=binary, sample_count=59
            ),
            lambda path, binary: MappingRunConfig(
                output_path=path, binary_path=binary, sample_count=171
            ),
            lambda path, binary: MappingRunConfig(
                output_path=path, binary_path=binary, timeout_s=0.0
            ),
            lambda path, binary: MappingRunConfig(
                output_path=path, binary_path=binary, timeout_s=float("nan")
            ),
            lambda path, binary: MappingRunConfig(
                output_path=path, binary_path=binary, run_id=""
            ),
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            for invalid_config in invalid_configs:
                with (
                    self.subTest(invalid_config=invalid_config),
                    self.assertRaises(ValueError),
                ):
                    invalid_config(
                        directory / "mapping-run.json", directory / "mapping_replay"
                    )

    def test_rejects_existing_final_or_intermediate_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            output_path = directory / "mapping-run.json"
            output_path.touch()
            with self.assertRaisesRegex(FileExistsError, "already exists"):
                MappingRunConfig(
                    output_path=output_path,
                    binary_path=directory / "mapping_replay",
                )

    def test_fails_preflight_when_ros_or_host_replay_is_missing(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            config = MappingRunConfig(
                output_path=directory / "mapping-run.json",
                binary_path=directory / "mapping_replay",
            )
            with self.assertRaisesRegex(FileNotFoundError, "ROS 2 executable"):
                config.validate_runtime(
                    python_executable=sys.executable,
                    ros2_executable="ros2-command-that-does-not-exist",
                )


if __name__ == "__main__":
    unittest.main()
