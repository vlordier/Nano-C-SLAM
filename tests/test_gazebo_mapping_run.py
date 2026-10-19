"""Hardware-free identity and command tests for a controlled mapping run."""

from __future__ import annotations

import sys
import tempfile
import unittest
import uuid
from pathlib import Path

from scripts.gazebo_mapping_run import _mapping_topics_ready
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_run_model import (
    MappingRunConfig,
    MissionCaptureGate,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    MissionPhase,
    MissionStatusEvent,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.odometry_preflight import (
    GroundedOdometryPreflight,
)


class _TopicGraph:
    def __init__(self, *, mission_status_subscribers: int = 1) -> None:
        self.events: list[MissionStatusEvent] = []
        self.error: str | None = None
        self.preflight = GroundedOdometryPreflight()
        self._subscribers = {
            "/cf1/odom": 1,
            "/nano_c_slam/sim/cf1/ranges_mm": 1,
            "/cf1/mission_status": mission_status_subscribers,
        }
        self._publishers = {
            "/cf1/odom": 1,
            "/nano_c_slam/sim/cf1/ranges_mm": 1,
        }

    def count_subscribers(self, topic_name: str) -> int:
        return self._subscribers.get(topic_name, 0)

    def count_publishers(self, topic_name: str) -> int:
        return self._publishers.get(topic_name, 0)

    def get_topic_names_and_types(self) -> list[tuple[str, list[str]]]:
        return [
            ("/cf1/odom", ["nav_msgs/msg/Odometry"]),
            (
                "/nano_c_slam/sim/cf1/ranges_mm",
                ["nano_c_slam_interfaces/msg/StampedRanges"],
            ),
            ("/cf1/mission_status", ["std_msgs/msg/String"]),
        ]

    def destroy_node(self) -> None:
        pass


class MappingTopicReadinessTests(unittest.TestCase):
    def test_starts_mission_with_one_status_subscriber(self) -> None:
        self.assertTrue(_mapping_topics_ready(_TopicGraph(), "cf1"))

    def test_waits_if_status_listener_is_not_discovered(self) -> None:
        self.assertFalse(
            _mapping_topics_ready(_TopicGraph(mission_status_subscribers=0), "cf1")
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
            )

        self.assertEqual(str(uuid.UUID(config.run_id)), config.run_id)
        self.assertIn(config.run_id, commands.collector)
        self.assertIn(f"run_id:={config.run_id}", commands.mission)
        self.assertIn("drone_name:=cf3", commands.mission)
        self.assertIn("--drone", commands.collector)
        self.assertEqual(commands.collector[1:3], ("-m", "scripts.gazebo_mapping_sitl"))
        self.assertIn("cf3", commands.collector)
        self.assertIn("mapping_path:=True", commands.mission)
        self.assertIn("--reuse-binary", commands.collector)
        self.assertIn("--mission-gated", commands.collector)
        self.assertIn("--ready-file", commands.collector)
        self.assertIn(str(config.readiness_path.resolve()), commands.collector)

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


class MissionCaptureGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.run_id = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"

    def event(
        self,
        run_id: str,
        sequence: int,
        phase: MissionPhase,
        drone_name: str = "cf1",
    ) -> str:
        event = MissionStatusEvent(
            run_id=run_id,
            drone_name=drone_name,
            sequence=sequence,
            sim_time_s=float(sequence),
            phase=phase,
            altitude_m=0.0 if phase is MissionPhase.TAKEOFF else None,
            mapping_path_requested=True,
            mapping_path_complete=False,
            landing_height_m=0.1,
        )
        return event.to_json()

    def test_waits_for_expected_run_identity_to_enter_active_flight(self) -> None:
        gate = MissionCaptureGate(self.run_id, "cf1")

        self.assertFalse(gate.observe(self.event(self.run_id, 0, MissionPhase.WAITING)))
        self.assertFalse(
            gate.observe(
                self.event(
                    "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb", 0, MissionPhase.WAITING
                )
            )
        )
        self.assertFalse(gate.observe(self.event(self.run_id, 1, MissionPhase.TAKEOFF)))
        self.assertTrue(gate.observe(self.event(self.run_id, 2, MissionPhase.HOVER)))
        self.assertEqual(gate.rejected_status_count, 1)
        self.assertEqual(gate.started_at_s, 2.0)

    def test_non_mapping_mission_can_start_collection_at_takeoff(self) -> None:
        gate = MissionCaptureGate(self.run_id, "cf1")
        waiting = MissionStatusEvent(
            run_id=self.run_id,
            drone_name="cf1",
            sequence=0,
            sim_time_s=0.0,
            phase=MissionPhase.WAITING,
            altitude_m=None,
            mapping_path_requested=False,
            mapping_path_complete=False,
            landing_height_m=0.1,
        )
        takeoff = MissionStatusEvent(
            run_id=self.run_id,
            drone_name="cf1",
            sequence=1,
            sim_time_s=1.0,
            phase=MissionPhase.TAKEOFF,
            altitude_m=0.0,
            mapping_path_requested=False,
            mapping_path_complete=False,
            landing_height_m=0.1,
        )

        self.assertFalse(gate.observe(waiting.to_json()))
        self.assertTrue(gate.observe(takeoff.to_json()))
        self.assertEqual(gate.started_at_s, 1.0)

    def test_fails_closed_on_malformed_reordered_or_failed_status(self) -> None:
        malformed = MissionCaptureGate(self.run_id, "cf1")
        self.assertFalse(malformed.observe("{"))
        self.assertIn("invalid mission status", malformed.failure_reason or "")

        reordered = MissionCaptureGate(self.run_id, "cf1")
        self.assertFalse(
            reordered.observe(self.event(self.run_id, 0, MissionPhase.WAITING))
        )
        self.assertFalse(
            reordered.observe(self.event(self.run_id, 2, MissionPhase.TAKEOFF))
        )
        self.assertIn("sequence", reordered.failure_reason or "")

        failed = MissionCaptureGate(self.run_id, "cf1")
        failed_event = MissionStatusEvent(
            run_id=self.run_id,
            drone_name="cf1",
            sequence=0,
            sim_time_s=0.0,
            phase=MissionPhase.FAILED,
            altitude_m=None,
            mapping_path_requested=True,
            mapping_path_complete=False,
            landing_height_m=0.1,
            failure_reason="odometry unavailable",
        )
        self.assertFalse(failed.observe(failed_event.to_json()))
        self.assertIn("mission failed", failed.failure_reason or "")

        completed = MissionCaptureGate(self.run_id, "cf1")
        complete_event = MissionStatusEvent(
            run_id=self.run_id,
            drone_name="cf1",
            sequence=0,
            sim_time_s=0.5,
            phase=MissionPhase.COMPLETE,
            altitude_m=0.05,
            mapping_path_requested=True,
            mapping_path_complete=True,
            landing_height_m=0.1,
            landing_stable_since_s=0.0,
        )
        self.assertFalse(completed.observe(complete_event.to_json()))
        self.assertIn("before telemetry", completed.failure_reason or "")


if __name__ == "__main__":
    unittest.main()
