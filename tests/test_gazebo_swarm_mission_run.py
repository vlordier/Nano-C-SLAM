"""Pure report and command checks for retained three-drone mission runs."""

from __future__ import annotations

import json
import tempfile
import unittest
import uuid
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from scripts import gazebo_swarm_mission_run
from scripts.gazebo_swarm_mission_run import (
    build_mission_commands,
    build_run_report,
    publish_report_atomically,
    read_mission_status_events,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
    MissionPhase,
    MissionStatusEvent,
    create_run_id,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.swarm_mission_processes import (
    SwarmMissionProcessResult,
)


def _complete_stream(run_id: str, drone: str) -> list[MissionStatusEvent]:
    phases = (
        (MissionPhase.WAITING, 0.0, None, None, None, False),
        (MissionPhase.TAKEOFF, 0.1, 0.0, 0.1, None, False),
        (MissionPhase.HOVER, 0.2, 0.5, 0.2, None, True),
        (MissionPhase.LAND, 1.0, 0.05, 1.0, 1.0, True),
        (MissionPhase.LAND, 1.2, 0.05, 1.2, 1.0, True),
        (MissionPhase.LAND, 1.4, 0.05, 1.4, 1.0, True),
        (MissionPhase.COMPLETE, 1.5, 0.05, 1.5, 1.0, True),
    )
    return [
        MissionStatusEvent(
            run_id=run_id,
            drone_name=drone,
            sequence=index,
            phase=phase,
            sim_time_s=sim_time,
            altitude_m=altitude,
            odometry_stamp_s=odom_stamp,
            landing_stable_since_s=stable_since,
            mapping_path_requested=True,
            mapping_path_complete=path_complete,
            landing_height_m=0.1,
        )
        for index, (
            phase,
            sim_time,
            altitude,
            odom_stamp,
            stable_since,
            path_complete,
        ) in enumerate(phases)
    ]


class GazeboSwarmMissionRunTests(unittest.TestCase):
    def test_builds_explicit_shared_run_commands_for_each_supported_drone(self) -> None:
        run_id = create_run_id()

        commands = build_mission_commands(
            run_id=run_id,
            ros2_executable="ros2",
            mapping_path=True,
        )

        self.assertEqual(tuple(commands), SUPPORTED_DRONE_NAMES)
        for drone, command in commands.items():
            self.assertIn(f"drone_name:={drone}", command)
            self.assertIn(f"run_id:={run_id}", command)
            self.assertIn("mapping_path:=True", command)
            self.assertIn("use_sim_time:=True", command)

    def test_reads_and_orders_structured_events_from_separate_raw_logs(self) -> None:
        run_id = create_run_id()
        stream = _complete_stream(run_id, "cf1")
        with tempfile.TemporaryDirectory() as temporary:
            stdout = Path(temporary) / "stdout.log"
            stderr = Path(temporary) / "stderr.log"
            stdout.write_text(
                "INFO mission_status=" + stream[0].to_json() + "\n"
                "INFO mission_status=" + stream[2].to_json() + "\n",
                encoding="utf-8",
            )
            stderr.write_text(
                "INFO mission_status=" + stream[1].to_json() + "\n",
                encoding="utf-8",
            )

            events, errors = read_mission_status_events(stdout, stderr)

        self.assertEqual([event.sequence for event in events], [0, 1, 2])
        self.assertEqual(errors, [])

    def test_report_preserves_all_success_streams_and_can_only_pass_all_three(
        self,
    ) -> None:
        run_id = create_run_id()
        commands = build_mission_commands(
            run_id=run_id, ros2_executable="ros2", mapping_path=True
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logs = root / "logs"
            logs.mkdir()
            stdout_paths = {}
            stderr_paths = {}
            for drone in SUPPORTED_DRONE_NAMES:
                stdout = logs / f"{drone}.stdout.log"
                stderr = logs / f"{drone}.stderr.log"
                stdout.write_text(
                    "\n".join(
                        f"INFO mission_status={event.to_json()}"
                        for event in _complete_stream(run_id, drone)
                    ),
                    encoding="utf-8",
                )
                stderr.write_text("", encoding="utf-8")
                stdout_paths[drone] = stdout
                stderr_paths[drone] = stderr
            process_result = SwarmMissionProcessResult(
                exit_codes=dict.fromkeys(SUPPORTED_DRONE_NAMES, 0),
                stdout_paths=stdout_paths,
                stderr_paths=stderr_paths,
                error=None,
            )

            report = build_run_report(
                run_id=run_id,
                world_path=root / "world.sdf",
                commands=commands,
                process_result=process_result,
                mapping_path=True,
            )

        self.assertTrue(report["overall_success"])
        self.assertEqual(
            {drone: len(events) for drone, events in report["mission_events"].items()},
            dict.fromkeys(SUPPORTED_DRONE_NAMES, 7),
        )
        self.assertEqual(set(report["mission_outcomes"]), set(SUPPORTED_DRONE_NAMES))

    def test_failure_report_retains_timeout_reason_and_never_claims_success(
        self,
    ) -> None:
        run_id = create_run_id()
        commands = build_mission_commands(
            run_id=run_id, ros2_executable="ros2", mapping_path=True
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logs = root / "logs"
            logs.mkdir()
            stdout_paths = {}
            stderr_paths = {}
            for drone in SUPPORTED_DRONE_NAMES:
                stdout = logs / f"{drone}.stdout.log"
                stderr = logs / f"{drone}.stderr.log"
                events = _complete_stream(run_id, drone)
                if drone == "cf3":
                    failure = replace(
                        events[1],
                        sequence=2,
                        sim_time_s=2.0,
                        phase=MissionPhase.FAILED,
                        altitude_m=0.4,
                        mapping_path_complete=False,
                        failure_reason="mapping path timed out",
                    )
                    events = [*events[:2], failure]
                stdout.write_text(
                    "\n".join(
                        f"INFO mission_status={event.to_json()}" for event in events
                    ),
                    encoding="utf-8",
                )
                stderr.write_text("", encoding="utf-8")
                stdout_paths[drone] = stdout
                stderr_paths[drone] = stderr
            process_result = SwarmMissionProcessResult(
                exit_codes={"cf1": 0, "cf3": 1, "cf4": 0},
                stdout_paths=stdout_paths,
                stderr_paths=stderr_paths,
                error=None,
            )

            report = build_run_report(
                run_id=run_id,
                world_path=root / "world.sdf",
                commands=commands,
                process_result=process_result,
                mapping_path=True,
            )

        self.assertFalse(report["overall_success"])
        failed = report["mission_outcomes"]["cf3"]
        self.assertEqual(failed["terminal_phase"], "FAILED")
        self.assertEqual(failed["failure_reason"], "mapping path timed out")
        self.assertEqual(failed["process_exit_code"], 1)

    def test_missing_status_keeps_other_drone_outcomes_in_failure_report(self) -> None:
        run_id = create_run_id()
        commands = build_mission_commands(
            run_id=run_id, ros2_executable="ros2", mapping_path=True
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logs = root / "logs"
            logs.mkdir()
            stdout_paths = {}
            stderr_paths = {}
            for drone in SUPPORTED_DRONE_NAMES:
                stdout = logs / f"{drone}.stdout.log"
                stderr = logs / f"{drone}.stderr.log"
                events = [] if drone == "cf3" else _complete_stream(run_id, drone)
                stdout.write_text(
                    "\n".join(
                        f"INFO mission_status={event.to_json()}" for event in events
                    ),
                    encoding="utf-8",
                )
                stderr.write_text("", encoding="utf-8")
                stdout_paths[drone] = stdout
                stderr_paths[drone] = stderr
            process_result = SwarmMissionProcessResult(
                exit_codes={"cf1": 0, "cf3": 2, "cf4": 0},
                stdout_paths=stdout_paths,
                stderr_paths=stderr_paths,
                error=None,
            )

            report = build_run_report(
                run_id=run_id,
                world_path=root / "world.sdf",
                commands=commands,
                process_result=process_result,
                mapping_path=True,
            )

        self.assertFalse(report["overall_success"])
        outcomes = report["mission_outcomes"]
        self.assertEqual(outcomes["cf1"]["classification"], "success")
        self.assertEqual(outcomes["cf3"]["classification"], "invalid_or_missing_status")
        self.assertEqual(outcomes["cf4"]["classification"], "success")

    def test_report_is_atomically_immutable(self) -> None:
        run_id = str(uuid.uuid4())
        report: dict[str, object] = {"run_id": run_id, "overall_success": False}
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "report.json"
            publish_report_atomically(output, report)
            self.assertEqual(json.loads(output.read_text()), report)
            with self.assertRaises(FileExistsError):
                publish_report_atomically(output, report)

    def _fake_supervisor(self, *, fail_drone: str | None = None) -> object:
        def supervise(
            commands: dict[str, tuple[str, ...]],
            *,
            log_directory: Path,
            timeout_s: float,
        ) -> SwarmMissionProcessResult:
            self.assertGreater(timeout_s, 0)
            log_directory.mkdir(parents=True)
            run_id = next(
                part.removeprefix("run_id:=")
                for part in commands["cf1"]
                if part.startswith("run_id:=")
            )
            stdout_paths = {}
            stderr_paths = {}
            exit_codes = dict.fromkeys(SUPPORTED_DRONE_NAMES, 0)
            for drone in SUPPORTED_DRONE_NAMES:
                events = _complete_stream(run_id, drone)
                if drone == fail_drone:
                    events = [
                        *events[:2],
                        replace(
                            events[1],
                            sequence=2,
                            sim_time_s=2.0,
                            phase=MissionPhase.FAILED,
                            altitude_m=0.4,
                            mapping_path_complete=False,
                            failure_reason="injected timeout",
                        ),
                    ]
                    exit_codes[drone] = 1
                stdout = log_directory / f"{drone}.stdout.log"
                stderr = log_directory / f"{drone}.stderr.log"
                stdout.write_text(
                    "\n".join(
                        f"INFO mission_status={event.to_json()}" for event in events
                    ),
                    encoding="utf-8",
                )
                stderr.write_text("", encoding="utf-8")
                stdout_paths[drone] = stdout
                stderr_paths[drone] = stderr
            return SwarmMissionProcessResult(
                exit_codes=exit_codes,
                stdout_paths=stdout_paths,
                stderr_paths=stderr_paths,
                error=None,
            )

        return supervise

    def test_run_preflights_world_and_publishes_full_success_bundle(self) -> None:
        world = Path(__file__).parent / "fixtures/swarm_world_spawn_fixture.sdf"
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "swarm-run.json"
            with patch.object(
                gazebo_swarm_mission_run,
                "supervise_swarm_mission_processes",
                side_effect=self._fake_supervisor(),
            ):
                report = gazebo_swarm_mission_run.run(
                    world_path=world,
                    output_path=output,
                    ros2_executable="ros2",
                    timeout_s=5.0,
                    mapping_path=True,
                )

            self.assertTrue(report["overall_success"])
            self.assertEqual(json.loads(output.read_text()), report)
            self.assertTrue(
                all(
                    Path(paths["stdout"]).is_file() and Path(paths["stderr"]).is_file()
                    for paths in report["raw_logs"].values()
                )
            )

    def test_run_publishes_failure_bundle_with_injected_drone_timeout(self) -> None:
        world = Path(__file__).parent / "fixtures/swarm_world_spawn_fixture.sdf"
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "swarm-run.json"
            with patch.object(
                gazebo_swarm_mission_run,
                "supervise_swarm_mission_processes",
                side_effect=self._fake_supervisor(fail_drone="cf3"),
            ):
                report = gazebo_swarm_mission_run.run(
                    world_path=world,
                    output_path=output,
                    ros2_executable="ros2",
                    timeout_s=5.0,
                    mapping_path=True,
                )

            self.assertFalse(report["overall_success"])
            persisted = json.loads(output.read_text())
            self.assertFalse(persisted["overall_success"])
            self.assertEqual(
                persisted["mission_outcomes"]["cf3"]["failure_reason"],
                "injected timeout",
            )


if __name__ == "__main__":
    unittest.main()
