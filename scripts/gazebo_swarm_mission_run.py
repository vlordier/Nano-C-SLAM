#!/usr/bin/env python3
"""Run and retain one three-process Gazebo swarm mission evidence bundle."""

from __future__ import annotations

import argparse
import itertools
import json
import os
import shutil
import sys
import tempfile
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
    MissionStatusEvent,
    create_run_id,
    summarize_mission_process_outcome,
    validate_run_id,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.swarm_geometry import (
    read_spawn_positions,
    validate_default_swarm_routes,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.swarm_mission_processes import (
    SwarmMissionProcessResult,
    supervise_swarm_mission_processes,
)

STATUS_MARKER = "mission_status="


def build_mission_commands(
    *, run_id: str, ros2_executable: str, mapping_path: bool
) -> dict[str, tuple[str, ...]]:
    """Build shell-free, run-bound mission commands for the exact swarm."""
    validate_run_id(run_id)
    if not isinstance(ros2_executable, str) or not ros2_executable:
        raise ValueError("ros2_executable must be a nonempty string")
    if not isinstance(mapping_path, bool):
        raise ValueError("mapping_path must be a boolean")
    ros2_path = shutil.which(ros2_executable) or ros2_executable
    return {
        drone: (
            ros2_path,
            "run",
            "nano_c_slam_gazebo",
            "sim_mission",
            "--ros-args",
            "-r",
            f"__node:={drone}_sim_mission",
            "-p",
            "use_sim_time:=True",
            "-p",
            f"drone_name:={drone}",
            "-p",
            f"run_id:={run_id}",
            "-p",
            f"mapping_path:={'True' if mapping_path else 'False'}",
        )
        for drone in SUPPORTED_DRONE_NAMES
    }


def read_mission_status_events(
    stdout_path: Path, stderr_path: Path
) -> tuple[list[MissionStatusEvent], list[str]]:
    """Read structured status events and retain malformed-line diagnostics."""
    events: list[MissionStatusEvent] = []
    errors: list[str] = []
    for path in (stdout_path, stderr_path):
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as error:
            errors.append(f"could not read process log {path.name}: {error}")
            continue
        channel_events: list[MissionStatusEvent] = []
        for line_number, line in enumerate(lines, start=1):
            if STATUS_MARKER not in line:
                continue
            payload = line.split(STATUS_MARKER, 1)[1]
            try:
                event = MissionStatusEvent.from_json(payload)
            except ValueError as error:
                errors.append(
                    f"{path.name}:{line_number}: invalid mission status: {error}"
                )
            else:
                channel_events.append(event)
        if any(
            following.sequence <= previous.sequence
            for previous, following in itertools.pairwise(channel_events)
        ):
            errors.append(f"{path.name}: mission status sequence is not increasing")
        events.extend(channel_events)
    events.sort(key=lambda event: event.sequence)
    return events, errors


def build_run_report(
    *,
    run_id: str,
    world_path: Path,
    commands: dict[str, tuple[str, ...]],
    process_result: SwarmMissionProcessResult,
    mapping_path: bool,
) -> dict[str, object]:
    """Build a fail-closed report while preserving every parseable status."""
    validate_run_id(run_id)
    events_by_drone: dict[str, list[MissionStatusEvent]] = {}
    parse_errors: dict[str, list[str]] = {}
    for drone in SUPPORTED_DRONE_NAMES:
        events, errors = read_mission_status_events(
            process_result.stdout_paths[drone], process_result.stderr_paths[drone]
        )
        events_by_drone[drone] = events
        parse_errors[drone] = errors

    errors = [error for drone_errors in parse_errors.values() for error in drone_errors]
    if process_result.error is not None:
        errors.append(process_result.error)
    outcomes: dict[str, object] = {}
    for drone in SUPPORTED_DRONE_NAMES:
        exit_code = process_result.exit_codes[drone]
        terminal_phase: str | None = None
        failure_reason: str | None = None
        if exit_code is None:
            classification = "process_not_started_or_reaped"
            failure_reason = process_result.error or "process exit status is missing"
        elif parse_errors[drone]:
            classification = "invalid_status_log"
            failure_reason = "; ".join(parse_errors[drone])
        else:
            try:
                outcome = summarize_mission_process_outcome(
                    events_by_drone[drone],
                    run_id,
                    drone,
                    exit_code,
                    require_mapping_path=mapping_path,
                )
            except ValueError as error:
                classification = "invalid_or_missing_status"
                failure_reason = str(error)
                if events_by_drone[drone]:
                    terminal_phase = events_by_drone[drone][-1].phase.name
            else:
                classification = outcome.classification
                terminal_phase = outcome.terminal.phase.name
                failure_reason = outcome.terminal.failure_reason
        outcomes[drone] = {
            "terminal_phase": terminal_phase,
            "failure_reason": failure_reason,
            "process_exit_code": exit_code,
            "classification": classification,
        }

    overall_success = (
        process_result.error is None
        and not errors
        and all(item["classification"] == "success" for item in outcomes.values())
    )
    if not overall_success and not errors:
        errors.append("one or more drone missions failed or mismatched")

    return {
        "schema_version": 1,
        "run_id": run_id,
        "created_at_utc": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "world_path": str(world_path.resolve()),
        "mapping_path_requested": mapping_path,
        "overall_success": overall_success,
        "process_exit_codes": process_result.exit_codes,
        "process_error": process_result.error,
        "mission_events": {
            drone: [json.loads(event.to_json()) for event in events]
            for drone, events in events_by_drone.items()
        },
        "mission_outcomes": outcomes,
        "commands": {drone: list(command) for drone, command in commands.items()},
        "raw_logs": {
            drone: {
                "stdout": str(process_result.stdout_paths[drone].resolve()),
                "stderr": str(process_result.stderr_paths[drone].resolve()),
            }
            for drone in SUPPORTED_DRONE_NAMES
        },
        "validation_errors": errors,
    }


def publish_report_atomically(output_path: Path, report: dict[str, object]) -> None:
    """Publish one immutable JSON report without replacing prior evidence."""
    if output_path.exists():
        raise FileExistsError(f"run report already exists: {output_path}")
    run_id = report.get("run_id")
    if not isinstance(run_id, str):
        raise ValueError("run report is missing its run ID")
    content = (
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode()
    with tempfile.NamedTemporaryFile(
        mode="wb",
        prefix=f".{output_path.name}.{run_id}.",
        suffix=".tmp",
        dir=output_path.parent,
        delete=False,
    ) as temporary:
        temporary_path = Path(temporary.name)
        temporary.write(content)
        temporary.flush()
        os.fsync(temporary.fileno())
    try:
        os.link(temporary_path, output_path)
    finally:
        temporary_path.unlink(missing_ok=True)


def run(
    *,
    world_path: Path,
    output_path: Path,
    ros2_executable: str,
    timeout_s: float,
    mapping_path: bool,
    run_id: str | None = None,
) -> dict[str, object]:
    """Preflight routes, run three missions, and publish success or failure."""
    selected_run_id = create_run_id() if run_id is None else validate_run_id(run_id)
    world = world_path.resolve(strict=True)
    if not world.is_file() or output_path.exists() or not output_path.parent.is_dir():
        raise ValueError(
            "world must be a file; output must be new in an existing directory"
        )
    validate_default_swarm_routes(read_spawn_positions(world))
    commands = build_mission_commands(
        run_id=selected_run_id,
        ros2_executable=ros2_executable,
        mapping_path=mapping_path,
    )
    log_directory = output_path.with_name(f"{output_path.stem}.{selected_run_id}.logs")
    result = supervise_swarm_mission_processes(
        commands,
        log_directory=log_directory.resolve(),
        timeout_s=timeout_s,
    )
    report = build_run_report(
        run_id=selected_run_id,
        world_path=world,
        commands=commands,
        process_result=result,
        mapping_path=mapping_path,
    )
    publish_report_atomically(output_path, report)
    return report


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ros2", default="ros2")
    parser.add_argument("--run-id")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--no-mapping-path", action="store_true")
    options = parser.parse_args(args)
    try:
        report = run(
            world_path=options.world,
            output_path=options.output,
            ros2_executable=options.ros2,
            timeout_s=options.timeout,
            mapping_path=not options.no_mapping_path,
            run_id=options.run_id,
        )
    except (OSError, RuntimeError, ValueError) as error:
        print(
            f"Gazebo swarm mission run failed before report publication: {error}",
            file=sys.stderr,
        )
        return 1
    print(json.dumps(report, sort_keys=True))
    return 0 if report["overall_success"] is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
