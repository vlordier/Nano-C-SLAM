"""Pure CLI and atomic-publication checks for the one-run coordinator."""

from __future__ import annotations

import json
import tempfile
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

from scripts.gazebo_mapping_run import (
    _events_from_mission_log,
    _parse_args,
    _publish_report_atomically,
    _run,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_run_processes import (
    supervise_mapping_processes,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    MissionPhase,
    MissionStatusEvent,
)


class MappingRunCliTests(unittest.TestCase):
    def test_missing_world_session_fails_before_child_process_start(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            _, config = _parse_args(["--output", str(directory / "run.json")])
            with (
                patch.object(type(config), "validate_runtime"),
                patch(
                    "scripts.gazebo_mapping_run.session_file_for_partition",
                    return_value=directory / "missing-session.json",
                ),
                patch(
                    "scripts.gazebo_mapping_run.supervise_mapping_processes",
                    wraps=supervise_mapping_processes,
                ) as supervise,
                self.assertRaisesRegex(RuntimeError, "start the world"),
            ):
                _run(config, "ros2")

            supervise.assert_not_called()

    def test_mission_log_recovers_terminal_event_and_checks_ros_prefix(self) -> None:
        phases = (
            MissionPhase.WAITING,
            MissionPhase.TAKEOFF,
            MissionPhase.HOVER,
            MissionPhase.LAND,
            MissionPhase.COMPLETE,
        )
        events = [
            MissionStatusEvent(
                run_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
                drone_name="cf1",
                sequence=index,
                sim_time_s=float(index),
                phase=phase,
                altitude_m=(None if phase is MissionPhase.WAITING else 0.05),
                mapping_path_requested=True,
                mapping_path_complete=phase
                in (MissionPhase.LAND, MissionPhase.COMPLETE),
                landing_height_m=0.1,
            )
            for index, phase in enumerate(phases)
        ]
        mission_log = "\n".join(f"mission_status={event.to_json()}" for event in events)

        recovered = _events_from_mission_log(mission_log, events[:-1])

        self.assertEqual(recovered, events)
        with self.assertRaisesRegex(RuntimeError, "disagree"):
            _events_from_mission_log(mission_log, [events[1]])
        with self.assertRaisesRegex(RuntimeError, "no structured"):
            _events_from_mission_log("mission exited 0", [])

    def test_cli_builds_a_run_config_without_ros_imports(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            options, config = _parse_args(
                [
                    "--drone",
                    "cf3",
                    "--output",
                    str(directory / "run.json"),
                    "--binary",
                    str(directory / "mapping_replay"),
                ]
            )

        self.assertEqual(options.drone, "cf3")
        self.assertEqual(str(uuid.UUID(config.run_id)), config.run_id)

    def test_rejects_invalid_cli_drone_before_starting_children(self) -> None:
        with (
            tempfile.TemporaryDirectory() as temporary_directory,
            self.assertRaises(SystemExit),
        ):
            _parse_args(
                [
                    "--drone",
                    "cf2",
                    "--output",
                    str(Path(temporary_directory) / "run.json"),
                ]
            )

    def test_atomic_report_publication_does_not_replace_existing_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output_path = Path(temporary_directory) / "run.json"
            run_id = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
            report: dict[str, object] = {
                "run_id": run_id,
                "mission_terminal": {"phase": "COMPLETE"},
            }

            _publish_report_atomically(output_path, report)
            original = output_path.read_text(encoding="utf-8")
            self.assertEqual(json.loads(original), report)
            with self.assertRaises(FileExistsError):
                _publish_report_atomically(output_path, {"run_id": run_id})

            self.assertEqual(output_path.read_text(encoding="utf-8"), original)
            self.assertEqual(list(Path(temporary_directory).glob("*.tmp")), [])


if __name__ == "__main__":
    unittest.main()
