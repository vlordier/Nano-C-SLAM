#!/usr/bin/env python3
"""Run one UUID-bound Gazebo mission, telemetry capture, and host replay."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import os
import sys
import tempfile
import time
from datetime import UTC, datetime
from pathlib import Path
from subprocess import SubprocessError
from typing import Protocol, cast

from scripts.gazebo_run_manifest import (
    create_gazebo_run_manifest,
    validate_gazebo_mapping_bundle,
)
from scripts.source_provenance import (
    collect_source_identity,
    source_identity_to_mapping,
    verify_source_identity,
)
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
    create_run_id,
    validate_mission_status_events,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.odometry_preflight import (
    GroundedOdometryPreflight,
    OdometrySample,
    wait_for_grounded_preflight,
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


class _OdometryMessage(Protocol):
    header: _OdometryHeader
    pose: _OdometryPose


class _OdometryHeader(Protocol):
    stamp: _OdometryStamp


class _OdometryStamp(Protocol):
    sec: int
    nanosec: int


class _OdometryPose(Protocol):
    pose: _OdometryPoseValue


class _OdometryPoseValue(Protocol):
    position: _OdometryPosition


class _OdometryPosition(Protocol):
    x: float
    y: float
    z: float


class _TopicListener(Protocol):
    def count_subscribers(self, topic_name: str) -> int: ...

    def count_publishers(self, topic_name: str) -> int: ...

    def get_topic_names_and_types(self) -> list[tuple[str, list[str]]]: ...

    def destroy_node(self) -> None: ...


class _StatusListener(_TopicListener, Protocol):
    events: list[MissionStatusEvent]
    error: str | None
    preflight: GroundedOdometryPreflight


def _mapping_topics_ready(node: _TopicListener, drone_name: str) -> bool:
    """Wait until telemetry publishers and the collector subscriptions exist."""
    validate_discovered_topic_types(node.get_topic_names_and_types(), drone_name)
    subscriptions = (
        node.count_subscribers(f"/{drone_name}/odom"),
        node.count_subscribers(f"/nano_c_slam/sim/{drone_name}/ranges_mm"),
        node.count_subscribers(f"/{drone_name}/mission_status"),
    )
    publishers = (
        node.count_publishers(f"/{drone_name}/odom"),
        node.count_publishers(f"/nano_c_slam/sim/{drone_name}/ranges_mm"),
    )
    return all(count >= 1 for count in subscriptions + publishers)


def _capture_control_publisher_preflight(
    node: _TopicListener, drone_name: str
) -> dict[str, object]:
    """Require an idle ROS command graph before launching the mission process."""
    publisher_counts = {
        f"/{drone_name}/cmd_vel": node.count_publishers(f"/{drone_name}/cmd_vel"),
        f"/{drone_name}/enable": node.count_publishers(f"/{drone_name}/enable"),
    }
    active = {topic: count for topic, count in publisher_counts.items() if count != 0}
    if active:
        raise RuntimeError(
            f"grounded preflight refused existing command publishers: {active}"
        )
    return {
        "checked_before_mission_process": True,
        "publisher_counts": publisher_counts,
        "no_command_publishers": True,
    }


def _events_from_mission_log(
    mission_log: str, observed_events: list[MissionStatusEvent]
) -> list[MissionStatusEvent]:
    """Recover log events and validate the ordered ROS-delivered subset."""
    marker = "mission_status="
    logged_events = [
        MissionStatusEvent.from_json(line.split(marker, 1)[1])
        for line in mission_log.splitlines()
        if marker in line
    ]
    if not logged_events:
        raise RuntimeError("mission log contains no structured status events")
    if not observed_events:
        raise RuntimeError("ROS listener received no mission status events")
    next_log_index = 0
    for observed in observed_events:
        matching_index = next(
            (
                index
                for index in range(next_log_index, len(logged_events))
                if observed.to_json() == logged_events[index].to_json()
            ),
            None,
        )
        if matching_index is None:
            raise RuntimeError(
                "ROS status events disagree with the mission publisher log"
            )
        next_log_index = matching_index + 1
    if observed_events[-1].to_json() != logged_events[-1].to_json():
        raise RuntimeError("ROS listener did not receive the mission terminal status")
    return logged_events


def _parse_args(
    args: list[str] | None = None,
) -> tuple[argparse.Namespace, MappingRunConfig]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", default="cf1")
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--preflight-timeout", type=float, default=10.0)
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
    parser.add_argument("--run-id", help="explicit run ID shared with GVSOC")
    parser.add_argument(
        "--stream-socket",
        type=Path,
        help="stream accepted telemetry to the live GVSOC SPI receiver",
    )
    parser.add_argument("--stream-timeout", type=float, default=1.0)
    options = parser.parse_args(args)
    if not math.isfinite(options.preflight_timeout) or options.preflight_timeout <= 0:
        parser.error("preflight timeout must be finite and positive")
    if not math.isfinite(options.stream_timeout) or options.stream_timeout <= 0:
        parser.error("stream timeout must be finite and positive")
    try:
        config = MappingRunConfig(
            output_path=options.output,
            binary_path=options.binary,
            drone_name=options.drone,
            scene=options.scene,
            sample_count=options.samples,
            timeout_s=options.timeout,
            run_id=options.run_id or create_run_id(),
            stream_socket_path=options.stream_socket,
            stream_timeout_s=options.stream_timeout,
        )
    except (FileExistsError, FileNotFoundError, ValueError) as error:
        parser.error(str(error))
    return options, config


def _create_status_listener(run_id: str, drone_name: str) -> _StatusListener:
    """Create a ROS status node without importing ROS in hardware-free tests."""
    node_base = importlib.import_module("rclpy.node").Node
    string_type = importlib.import_module("std_msgs.msg").String
    odometry_type = importlib.import_module("nav_msgs.msg").Odometry
    qos_module = importlib.import_module("rclpy.qos")
    parameter_type = importlib.import_module("rclpy.parameter").Parameter

    class MissionStatusNode(node_base):  # type: ignore[valid-type,misc]
        def __init__(self) -> None:
            super().__init__(
                "nano_c_slam_run_status",
                parameter_overrides=[parameter_type("use_sim_time", value=True)],
            )
            self.events: list[MissionStatusEvent] = []
            self.error: str | None = None
            self.preflight = GroundedOdometryPreflight()
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
            self._odometry_subscription = self.create_subscription(
                odometry_type,
                f"/{drone_name}/odom",
                self._on_odometry,
                qos_module.qos_profile_sensor_data,
            )

        def _on_status(self, message: _StringMessage) -> None:
            try:
                event = MissionStatusEvent.from_json(message.data)
            except ValueError as error:
                self.error = f"invalid mission status event: {error}"
                return
            if event.run_id == self._run_id and event.drone_name == self._drone_name:
                self.events.append(event)

        def _on_odometry(self, message: _OdometryMessage) -> None:
            header = message.header
            pose = message.pose
            stamp = header.stamp
            position = pose.pose.position
            stamp_s = float(stamp.sec) + float(stamp.nanosec) / 1e9
            sim_time_s = self.get_clock().now().nanoseconds / 1e9
            self.preflight.observe(
                OdometrySample(
                    stamp_s=stamp_s,
                    x_m=float(position.x),
                    y_m=float(position.y),
                    z_m=float(position.z),
                    sim_time_s=sim_time_s,
                    received_monotonic_s=time.monotonic(),
                )
            )

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


def _publish_report_bundle_atomically(
    output_path: Path,
    report: dict[str, object],
    manifest: bytes,
    *,
    artifacts: dict[Path, bytes] | None = None,
) -> None:
    """Publish a manifest first and report last as the bundle commit marker."""
    manifest_path = output_path.with_suffix(".provenance.json")
    if (
        output_path.exists()
        or manifest_path.exists()
        or any(path.exists() for path in (artifacts or {}))
    ):
        raise FileExistsError("Gazebo report or provenance manifest already exists")
    run_id = report.get("run_id")
    if not isinstance(run_id, str):
        raise ValueError("run report is missing its run ID")
    report["provenance_manifest"] = {
        "path": manifest_path.name,
        "sha256": hashlib.sha256(manifest).hexdigest(),
    }
    report_temporary = output_path.with_name(f".{output_path.name}.{run_id}.tmp")
    manifest_temporary = manifest_path.with_name(f".{manifest_path.name}.{run_id}.tmp")
    artifact_temporaries = {
        path: path.with_name(f".{path.name}.{run_id}.tmp") for path in (artifacts or {})
    }
    if any(path.exists() for path in artifact_temporaries):
        raise FileExistsError("Gazebo process stream artifact already exists")
    report_bytes = (
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode()
    published: list[Path] = []
    try:
        temporary_contents = [
            *(
                (artifact_temporaries[path], contents)
                for path, contents in (artifacts or {}).items()
            ),
            (manifest_temporary, manifest),
            (report_temporary, report_bytes),
        ]
        for temporary, contents in temporary_contents:
            with temporary.open("xb") as stream:
                stream.write(contents)
                stream.flush()
                os.fsync(stream.fileno())
        for path, temporary in artifact_temporaries.items():
            os.link(temporary, path)
            published.append(path)
        os.link(manifest_temporary, manifest_path)
        published.append(manifest_path)
        os.link(report_temporary, output_path)
        published.append(output_path)
    except OSError:
        for path in reversed(published):
            path.unlink(missing_ok=True)
        raise
    finally:
        report_temporary.unlink(missing_ok=True)
        manifest_temporary.unlink(missing_ok=True)
        for temporary in artifact_temporaries.values():
            temporary.unlink(missing_ok=True)


def _run(
    config: MappingRunConfig, ros2_executable: str, *, preflight_timeout_s: float = 10.0
) -> Path:
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
    source_before = collect_source_identity(REPOSITORY_ROOT)
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
            return _mapping_topics_ready(node, config.drone_name)

        odometry_preflight = wait_for_grounded_preflight(
            node.preflight,
            pump_status,
            timeout_s=preflight_timeout_s,
            monotonic=time.monotonic,
        )
        control_preflight = _capture_control_publisher_preflight(
            node, config.drone_name
        )

        execution_started = datetime.now(UTC).isoformat().replace("+00:00", "Z")
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
        verify_source_identity(REPOSITORY_ROOT, source_before)
        stream_paths = {
            "collector.stdout": process_result.collector_stdout,
            "collector.stderr": process_result.collector_stderr,
            "mission.stdout": process_result.mission_stdout,
            "mission.stderr": process_result.mission_stderr,
        }
        stream_contents = {
            name: path.read_bytes() for name, path in stream_paths.items()
        }
        process_streams = {
            name: {
                "path": path.relative_to(
                    config.output_path.resolve().parent
                ).as_posix(),
                "sha256": hashlib.sha256(stream_contents[name]).hexdigest(),
                "bytes": len(stream_contents[name]),
            }
            for name, path in stream_paths.items()
        }
        report: dict[str, object] = {
            "schema_version": 2,
            "run_id": config.run_id,
            "drone": config.drone_name,
            "scene": config.scene,
            "gazebo_world_session_id": world_session_id,
            "run_provenance": stable_provenance,
            "source_identity": source_identity_to_mapping(source_before),
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
            "odometry_preflight": odometry_preflight,
            "control_preflight": control_preflight,
            "capture_report": capture,
            "raw_logs": {
                "collector": collector_log,
                "mission": mission_log,
                "retained_at": str(config.log_directory.resolve()),
            },
            "process_streams": process_streams,
        }
        validate_combined_run_report(report)
        manifest = create_gazebo_run_manifest(
            report,
            source=source_before,
            stream_contents=stream_contents,
            started_at_utc=execution_started,
            finished_at_utc=datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        )
        _publish_report_bundle_atomically(
            config.output_path.resolve(), report, manifest
        )
        validate_gazebo_mapping_bundle(config.output_path.resolve())
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
        output_path = _run(
            config,
            options.ros2,
            preflight_timeout_s=options.preflight_timeout,
        )
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
