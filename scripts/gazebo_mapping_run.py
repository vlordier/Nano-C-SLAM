#!/usr/bin/env python3
"""Run one UUID-bound Gazebo mission, telemetry capture, and host replay."""

from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
import tempfile
import time
from pathlib import Path
from subprocess import SubprocessError
from typing import Protocol, cast

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_model import (
    validate_capture_report,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_run_model import (
    MappingRunConfig,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_run_processes import (
    supervise_mapping_processes,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    MissionPhase,
    MissionStatusEvent,
    validate_mission_status_events,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance import (
    collect_run_provenance,
    require_provenance_unchanged,
    validate_combined_run_report,
    validate_discovered_topic_types,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model import (
    claim_world_session,
    session_file_for_partition,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class _StringMessage(Protocol):
    data: str


class _StatusListener(Protocol):
    events: list[MissionStatusEvent]
    error: str | None

    def count_subscribers(self, topic_name: str) -> int: ...

    def count_publishers(self, topic_name: str) -> int: ...

    def get_topic_names_and_types(self) -> list[tuple[str, list[str]]]: ...

    def destroy_node(self) -> None: ...


def _events_from_mission_log(
    mission_log: str, observed_events: list[MissionStatusEvent]
) -> list[MissionStatusEvent]:
    """Recover the publisher's complete event stream and cross-check ROS delivery."""
    marker = "mission_status="
    logged_events = [
        MissionStatusEvent.from_json(line.split(marker, 1)[1])
        for line in mission_log.splitlines()
        if marker in line
    ]
    if not logged_events:
        raise RuntimeError("mission log contains no structured status events")
    if len(observed_events) > len(logged_events) or any(
        observed.to_json() != logged.to_json()
        for observed, logged in zip(observed_events, logged_events, strict=False)
    ):
        raise RuntimeError("ROS status events disagree with the mission publisher log")
    if not observed_events:
        raise RuntimeError("ROS listener received no mission status events")
    return logged_events


def _parse_args(
    args: list[str] | None = None,
) -> tuple[argparse.Namespace, MappingRunConfig]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", default="cf1")
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument(
        "--scene",
        choices=("default", "nearfield-mapping"),
        default="nearfield-mapping",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(tempfile.gettempdir()) / "nano-c-slam-gazebo-run.json",
    )
    parser.add_argument(
        "--binary",
        type=Path,
        default=REPOSITORY_ROOT / "cslam-gap9-app/BUILD-host/mapping_replay",
    )
    parser.add_argument("--ros2", default="ros2", help="ROS 2 CLI executable")
    options = parser.parse_args(args)
    try:
        config = MappingRunConfig(
            output_path=options.output,
            binary_path=options.binary,
            drone_name=options.drone,
            scene=options.scene,
            sample_count=options.samples,
            timeout_s=options.timeout,
        )
    except (FileExistsError, FileNotFoundError, ValueError) as error:
        parser.error(str(error))
    return options, config


def _create_status_listener(run_id: str, drone_name: str) -> _StatusListener:
    """Create a ROS status node without importing ROS in hardware-free tests."""
    node_base = importlib.import_module("rclpy.node").Node
    string_type = importlib.import_module("std_msgs.msg").String
    qos_module = importlib.import_module("rclpy.qos")

    class MissionStatusNode(node_base):  # type: ignore[valid-type,misc]
        def __init__(self) -> None:
            super().__init__("nano_c_slam_run_status")
            self.events: list[MissionStatusEvent] = []
            self.error: str | None = None
            self._run_id = run_id
            self._drone_name = drone_name
            status_qos = qos_module.QoSProfile(
                depth=10,
                durability=qos_module.DurabilityPolicy.TRANSIENT_LOCAL,
                history=qos_module.HistoryPolicy.KEEP_LAST,
            )
            self._subscription = self.create_subscription(
                string_type,
                f"/{drone_name}/mission_status",
                self._on_status,
                status_qos,
            )

        def _on_status(self, message: _StringMessage) -> None:
            try:
                event = MissionStatusEvent.from_json(message.data)
            except ValueError as error:
                self.error = f"invalid mission status event: {error}"
                return
            if event.run_id == self._run_id and event.drone_name == self._drone_name:
                self.events.append(event)

    return cast(_StatusListener, MissionStatusNode())


def _publish_report_atomically(output_path: Path, report: dict[str, object]) -> None:
    """Publish a complete JSON report without replacing an existing artifact."""
    run_id = report.get("run_id")
    if not isinstance(run_id, str):
        raise ValueError("run report is missing its run ID")
    temporary_path = output_path.with_name(f".{output_path.name}.{run_id}.tmp")
    content = json.dumps(report, indent=2, allow_nan=False) + "\n"
    try:
        with temporary_path.open("x", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary_path, output_path)
    finally:
        temporary_path.unlink(missing_ok=True)


def _run(config: MappingRunConfig, ros2_executable: str) -> Path:
    python_executable = sys.executable
    config.validate_runtime(
        python_executable=python_executable,
        ros2_executable=ros2_executable,
    )
    commands = config.build_commands(
        python_executable=python_executable,
        ros2_executable=ros2_executable,
    )
    provenance_before = collect_run_provenance(
        REPOSITORY_ROOT,
        ros2_executable=ros2_executable,
        mapping_binary=config.binary_path,
    )
    world_session_id = claim_world_session(
        session_file_for_partition(os.environ.get("GZ_PARTITION", "default")),
        scene=config.scene,
        run_id=config.run_id,
    )
    rclpy = importlib.import_module("rclpy")
    rclpy.init(args=[])
    node: _StatusListener | None = None
    try:
        node = _create_status_listener(config.run_id, config.drone_name)

        def pump_status() -> None:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.error is not None:
                raise RuntimeError(node.error)

        def subscriptions_ready() -> bool:
            validate_discovered_topic_types(
                node.get_topic_names_and_types(), config.drone_name
            )
            topic_counts = (
                node.count_subscribers(f"/{config.drone_name}/odom"),
                node.count_subscribers(
                    f"/nano_c_slam/sim/{config.drone_name}/ranges_mm"
                ),
                node.count_subscribers(f"/{config.drone_name}/mission_status"),
            )
            publisher_counts = (
                node.count_publishers(f"/{config.drone_name}/odom"),
                node.count_publishers(
                    f"/nano_c_slam/sim/{config.drone_name}/ranges_mm"
                ),
            )
            return (
                topic_counts[0] >= 1
                and topic_counts[1] >= 1
                and topic_counts[2] >= 2
                and all(count >= 1 for count in publisher_counts)
            )

        process_result = supervise_mapping_processes(
            commands,
            readiness_file=config.readiness_path.resolve(),
            run_id=config.run_id,
            log_directory=config.log_directory.resolve(),
            timeout_s=config.timeout_s,
            pump_status=pump_status,
            subscriptions_ready=subscriptions_ready,
        )
        # Child exit and DDS delivery are independent events. Give the
        # transient-local terminal event a bounded grace period after both
        # processes exit rather than assuming it arrived in the same spin.
        terminal_deadline = time.monotonic() + min(5.0, config.timeout_s)
        while (
            not node.events
            or node.events[-1].phase not in (MissionPhase.COMPLETE, MissionPhase.FAILED)
        ) and time.monotonic() < terminal_deadline:
            pump_status()
        mission_log = process_result.mission_log.read_text(encoding="utf-8")
        events = _events_from_mission_log(mission_log, node.events)
        if node.error is not None:
            raise RuntimeError(node.error)
        terminal = validate_mission_status_events(
            events,
            config.run_id,
            config.drone_name,
            require_mapping_path=True,
        )
        if terminal.phase is not MissionPhase.COMPLETE:
            raise RuntimeError("mission status did not reach COMPLETE")

        capture_data = json.loads(config.capture_path.read_text(encoding="utf-8"))
        capture = validate_capture_report(capture_data)
        if (
            capture.get("run_id") != config.run_id
            or capture.get("drone") != config.drone_name
            or capture.get("mission_binding") != "matching-status-run"
        ):
            raise RuntimeError("capture report identity does not match mission run")

        first_active = next(
            (
                event
                for event in events
                if event.phase in (MissionPhase.TAKEOFF, MissionPhase.HOVER)
            ),
            None,
        )
        capture_time = capture.get("input_sim_time_ms")
        if first_active is None or not isinstance(capture_time, dict):
            raise RuntimeError("capture report is missing run-bounded simulation times")
        capture_start_ms = capture_time.get("start")
        capture_end_ms = capture_time.get("end")
        if (
            isinstance(capture_start_ms, bool)
            or not isinstance(capture_start_ms, int)
            or isinstance(capture_end_ms, bool)
            or not isinstance(capture_end_ms, int)
            or capture_start_ms < int(first_active.sim_time_s * 1000) - 1
            or capture_end_ms > int(terminal.sim_time_s * 1000) + 1
        ):
            raise RuntimeError("telemetry timestamps fall outside the mission interval")

        collector_log = process_result.collector_log.read_text(encoding="utf-8")
        provenance_after = collect_run_provenance(
            REPOSITORY_ROOT,
            ros2_executable=ros2_executable,
            mapping_binary=config.binary_path,
        )
        stable_provenance = require_provenance_unchanged(
            provenance_before, provenance_after
        )
        report: dict[str, object] = {
            "schema_version": 2,
            "run_id": config.run_id,
            "drone": config.drone_name,
            "scene": config.scene,
            "gazebo_world_session_id": world_session_id,
            "run_provenance": stable_provenance,
            "child_commands": {
                "collector": list(commands.collector),
                "mission": list(commands.mission),
            },
            "process_exit_codes": {
                "collector": process_result.collector_returncode,
                "mission": process_result.mission_returncode,
            },
            "mission_events": [json.loads(event.to_json()) for event in events],
            "mission_terminal": json.loads(terminal.to_json()),
            "capture_report": capture,
            "raw_logs": {
                "collector": collector_log,
                "mission": mission_log,
                "retained_at": str(config.log_directory.resolve()),
            },
        }
        validate_combined_run_report(report)
        _publish_report_atomically(config.output_path.resolve(), report)
        for temporary_artifact in (config.capture_path, config.readiness_path):
            try:
                temporary_artifact.unlink()
            except OSError as error:
                print(
                    f"Could not remove temporary run artifact "
                    f"{temporary_artifact}: {error}",
                    file=sys.stderr,
                )
        return config.output_path.resolve()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: list[str] | None = None) -> int:
    options, config = _parse_args(args)
    try:
        output_path = _run(config, options.ros2)
    except KeyboardInterrupt:
        print("Gazebo mapping run interrupted; raw logs are retained", file=sys.stderr)
        return 130
    except (
        FileExistsError,
        FileNotFoundError,
        ImportError,
        OSError,
        RuntimeError,
        SubprocessError,
        TimeoutError,
        ValueError,
    ) as error:
        print(f"Gazebo mapping run failed: {error}", file=sys.stderr)
        return 1
    print(f"Gazebo mission and Nano-C-SLAM host replay passed: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
