"""Measure target GAP9 poses and map points against a bound Gazebo run."""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import cast

from scripts.capture_gap9_gvsoc_spi_main_probe_evidence import (
    SCAN_BUSY_PATTERN,
    SPI_DIAGNOSTICS_PATTERN,
    TARGET_MAP_CAPACITY,
    _decode_slam_job,
    _parse_target_map_points,
    _parse_target_poses,
    validate_config,
)
from scripts.evidence_json import load_json_file
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_model import (
    MappingSample,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_packets import (
    SCAN_CMD,
    encode_mapping_samples,
    samples_from_run_bundle,
)

EXPECTED_MODELS = frozenset(
    {"wall1", "wall2", "wall3", "wall4", "obstacle1", "obstacle2", "obstacle3"}
)
MAP_TOLERANCE_M = 0.05


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mapping(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(k, str) for k in value):
        raise ValueError(f"{label} must be a JSON object")
    return cast(dict[str, object], value)


def _mapping_route(run: dict[str, object], expected_route: str | None) -> str:
    """Read and optionally enforce the route recorded in the mission argv."""
    commands = _mapping(run.get("child_commands"), "run child commands")
    mission = commands.get("mission")
    if not isinstance(mission, list) or not all(
        isinstance(argument, str) for argument in mission
    ):
        raise ValueError("run bundle has no valid mission command")
    route_arguments = [
        argument.partition(":=")[2]
        for argument in mission
        if argument.startswith("mapping_route:=")
    ]
    if len(route_arguments) > 1:
        raise ValueError("mission command contains duplicate mapping routes")
    route = route_arguments[0] if route_arguments else "square"
    if route not in ("square", "heldout-asymmetric"):
        raise ValueError("mission command contains an unsupported mapping route")
    if expected_route is not None and route != expected_route:
        raise ValueError(f"run mapping route is {route!r}, expected {expected_route!r}")
    return route


def _point_to_segment_distance(
    point: tuple[float, float],
    segment: tuple[tuple[float, float], tuple[float, float]],
) -> float:
    (ax, ay), (bx, by) = segment
    dx, dy = bx - ax, by - ay
    denominator = dx * dx + dy * dy
    if denominator <= 0 or not math.isfinite(denominator):
        raise ValueError("SDF geometry contains a degenerate wall segment")
    fraction = max(
        0.0,
        min(1.0, ((point[0] - ax) * dx + (point[1] - ay) * dy) / denominator),
    )
    return math.hypot(point[0] - ax - fraction * dx, point[1] - ay - fraction * dy)


def _axis_aligned_bounds(
    points: list[tuple[float, float]],
) -> dict[str, float]:
    if not points:
        raise ValueError("cannot measure bounds of an empty point set")
    xs, ys = zip(*points, strict=True)
    return {
        "min_x_m": min(xs),
        "max_x_m": max(xs),
        "min_y_m": min(ys),
        "max_y_m": max(ys),
        "span_x_m": max(xs) - min(xs),
        "span_y_m": max(ys) - min(ys),
    }


def _sdf_segments(path: Path) -> dict[str, tuple[tuple[float, float], ...]]:
    raw = path.read_bytes()
    if len(raw) > 5_000_000 or b"<!DOCTYPE" in raw.upper():
        raise ValueError("SDF is oversized or contains a document type")
    try:
        world = ET.fromstring(raw).find("world")  # noqa: S314 - bounded local SDF
    except ET.ParseError as error:
        raise ValueError("SDF is malformed") from error
    if world is None:
        raise ValueError("SDF has no world")
    segments: dict[str, tuple[tuple[float, float], ...]] = {}
    for model in world.findall("model"):
        name = model.get("name")
        if name not in EXPECTED_MODELS:
            continue
        if name in segments:
            raise ValueError(f"SDF contains duplicate surface model {name}")
        if model.findtext("static") != "true":
            raise ValueError(f"SDF surface model {name} must be static")
        pose = model.findtext("pose")
        box_sizes = model.findall("link/collision/geometry/box/size")
        if pose is None or len(box_sizes) != 1:
            raise ValueError(
                f"SDF model {name} must have one model pose and one collision box"
            )
        for relative_pose in (
            model.find("link/pose"),
            model.find("link/collision/pose"),
        ):
            if relative_pose is not None:
                try:
                    relative_values = tuple(
                        float(v) for v in relative_pose.text.split()
                    )
                except (AttributeError, ValueError) as error:
                    raise ValueError(
                        f"SDF model {name} has an invalid link or collision pose"
                    ) from error
                if len(relative_values) != 6 or any(
                    value != 0 for value in relative_values
                ):
                    raise ValueError(
                        f"SDF model {name} uses unsupported relative link geometry"
                    )
        if model.find("pose").get("relative_to") not in (None, "world"):
            raise ValueError(f"SDF model {name} pose is not in the world frame")
        size = box_sizes[0].text
        try:
            x, y, _z, _roll, _pitch, yaw = (float(v) for v in pose.split())
            width, height, _depth = (float(v) for v in size.split())
        except (ValueError, TypeError) as error:
            raise ValueError(
                f"SDF model {name} has invalid pose or box size"
            ) from error
        if (
            not all(map(math.isfinite, (x, y, yaw, width, height)))
            or min(width, height) <= 0
        ):
            raise ValueError(f"SDF model {name} has nonfinite or invalid geometry")
        cosine, sine = math.cos(yaw), math.sin(yaw)
        corners = tuple(
            (
                x + cosine * local_x - sine * local_y,
                y + sine * local_x + cosine * local_y,
            )
            for local_x, local_y in (
                (-width / 2, -height / 2),
                (width / 2, -height / 2),
                (width / 2, height / 2),
                (-width / 2, height / 2),
            )
        )
        segments[name] = corners
    if set(segments) != EXPECTED_MODELS:
        missing = sorted(EXPECTED_MODELS - set(segments))
        raise ValueError(f"SDF is missing required walls or obstacles: {missing}")
    return segments


def _verify_world_binding(
    run: dict[str, object],
    session: dict[str, object],
    session_path: Path,
    sdf_path: Path,
    world_attestation: dict[str, object],
) -> None:
    run_id = run.get("run_id")
    if (
        run.get("scene") != "nearfield-mapping"
        or session.get("scene") != "nearfield-mapping"
        or session.get("session_id") != run.get("gazebo_world_session_id")
    ):
        raise ValueError("run and world session identities or scenes do not match")
    processes = session.get("processes")
    if not isinstance(processes, list):
        raise ValueError("world session has no process records")
    server_commands = [
        item.get("argv")
        for item in processes
        if isinstance(item, dict) and item.get("role") == "gazebo-server"
    ]
    if len(server_commands) != 1 or not isinstance(server_commands[0], list):
        raise ValueError("world session must identify exactly one Gazebo server")
    launched_sdf_paths = [
        Path(argument)
        for argument in server_commands[0]
        if isinstance(argument, str) and argument.endswith(".sdf")
    ]
    if len(launched_sdf_paths) != 1:
        raise ValueError("world session does not identify exactly one SDF input")
    expected_sdf_hash = _sha256(sdf_path)
    session_sdf_hash = session.get("world_sdf_sha256")
    session_sdf_path = session.get("world_sdf_path")
    session_sdf_bytes = session.get("world_sdf_bytes")
    session_has_sdf = any(
        value is not None
        for value in (session_sdf_hash, session_sdf_path, session_sdf_bytes)
    )
    if session_has_sdf and (
        session_sdf_hash is None
        or session_sdf_path is None
        or session_sdf_bytes is None
    ):
        raise ValueError("world session contains an incomplete SDF identity")
    if session_sdf_hash is not None and (
        session_sdf_hash != expected_sdf_hash
        or not isinstance(session_sdf_path, str)
        or Path(session_sdf_path).resolve() != launched_sdf_paths[0].resolve()
        or session_sdf_bytes != sdf_path.stat().st_size
    ):
        raise ValueError("world session SDF hash does not match the archived input")
    if (
        world_attestation.get("schema_version") != 1
        or world_attestation.get("session_id") != session.get("session_id")
        or world_attestation.get("session_sha256") != _sha256(session_path)
        or world_attestation.get("gazebo_server_sdf_argument")
        != str(launched_sdf_paths[0])
        or world_attestation.get("world_sdf_sha256") != expected_sdf_hash
        or world_attestation.get("archived_sdf_sha256") != expected_sdf_hash
        or world_attestation.get("verified_while_server_active") is not True
    ):
        raise ValueError("world input attestation does not match the archived run")
    if (
        launched_sdf_paths[0].is_file()
        and _sha256(launched_sdf_paths[0]) != expected_sdf_hash
    ):
        raise ValueError("archived SDF bytes differ from the Gazebo launch input")
    if not isinstance(run_id, str) or not run_id:
        raise ValueError("run bundle has no run ID")
    if session.get("schema_version") != 1:
        raise ValueError("unsupported world session schema")


def _verify_target_capture(
    target: dict[str, object],
    host_transfer: dict[str, object],
    target_log: str,
    run: dict[str, object],
    samples: tuple[MappingSample, ...],
) -> None:
    """Cross-check target health fields against its raw log and source packets."""
    packets = encode_mapping_samples(samples)
    expected_frames = [packet.frame.hex() for packet in packets]
    expected_scans = [
        packet.pose_id for packet in packets if packet.command == SCAN_CMD
    ]
    if (
        target.get("evidence_class")
        != "gvsoc-firmware-ipc-stream-of-inflight-gazebo-telemetry"
        or target.get("frame_count") != len(packets)
        or target.get("frames") != expected_frames
    ):
        raise ValueError("target evidence frames differ from the paired Gazebo run")
    expected_host_frames = [
        {
            "sequence": packet.sequence,
            "command": packet.command,
            "pose_id": packet.pose_id,
            "source_stamp_ns": packet.source_stamp_ns,
            "frame_tx_hex": packet.frame.hex(),
            "length": len(packet.frame),
        }
        for packet in packets
    ]
    expected_transfer_lengths = [
        {"master_tx_bytes": len(packet.frame), "master_rx_bytes": len(packet.frame)}
        for packet in packets
    ]
    response_hex = target.get("optimizer_response_expected_hex")
    if (
        host_transfer.get("sequence") != "gazebo-live"
        or host_transfer.get("run_id") != run.get("run_id")
        or host_transfer.get("drone_id") != run.get("drone")
        or host_transfer.get("pre_cs_dummy_clock_cycles") != 8
        or host_transfer.get("frames") != expected_host_frames
        or host_transfer.get("command_transfer_lengths") != expected_transfer_lengths
        or host_transfer.get("control_phase") != "optimizer_response_captured"
        or host_transfer.get("response_rx_hex") != response_hex
    ):
        raise ValueError("host SPI transfer differs from the paired Gazebo run")

    marker = target.get("target_marker")
    marker_pattern = re.compile(
        r"NANO_C_SLAM_GVSOC_SPI_MAIN:gazebo_capture_ok "
        r"packets=(\d+) pose_decoded=(\d+) pose_inserted=(\d+) "
        r"scans_decoded=(\d+) scans_registered=(\d+) constraints=(\d+) "
        r"optimize=(\d+) poses=(\d+) graph=(\d+) solver_calls=(\d+) "
        r"has_nan=(\d+) completed=(\d+) response_expected_hex=([0-9a-f]{36})"
    )
    if (
        not isinstance(marker, str)
        or target_log.count(marker) != 1
        or sum(
            line.startswith("NANO_C_SLAM_GVSOC_SPI_MAIN:gazebo_capture_ok ")
            for line in target_log.splitlines()
        )
        != 1
        or "NANO_C_SLAM_GVSOC_SPI_MAIN:run_fail" in target_log
    ):
        raise ValueError("target success marker is absent or duplicated in its log")
    match = marker_pattern.fullmatch(marker)
    if match is None:
        raise ValueError("target success marker has an unsupported format")
    (
        packet_count,
        decoded_poses,
        inserted_poses,
        decoded_scans,
        registered_scans,
        constraint_count,
        optimize_count,
        optimized_poses,
        graph_size,
        solver_calls,
        has_nan,
        completed,
    ) = (int(value) for value in match.groups()[:12])
    response_hex = match.group(13)
    sample_count = len(samples)
    if (
        packet_count != len(packets)
        or decoded_poses != sample_count
        or inserted_poses != sample_count
        or decoded_scans != len(expected_scans)
        or registered_scans != len(expected_scans)
        or constraint_count != 2
        or optimize_count != 1
        or optimized_poses != sample_count
        or graph_size != 3
        or solver_calls < 1
        or has_nan != 0
        or completed != 1
    ):
        raise ValueError("target success marker reports invalid optimizer health")

    expected_job = _decode_slam_job(response_hex)
    if target.get("optimizer_response_expected_hex") != response_hex:
        raise ValueError("target expected response bytes differ from its marker")
    if target.get("slam_job_fields") != expected_job:
        raise ValueError("target optimizer fields differ from its response bytes")
    if (
        expected_job["number_of_poses"] != sample_count
        or expected_job["main_graph_size"] != graph_size
        or expected_job["has_nan"] != 0
        or expected_job["main_solver_returned"] != 1
        or expected_job["successful_solver_call_count"] < 1
        or expected_job["completed_solver_iteration_count"]
        != 3 * expected_job["successful_solver_call_count"]
        or expected_job["optimization_completed"] != 1
    ):
        raise ValueError("target slam_job_t does not prove a completed finite solve")
    if (
        target.get("optimizer_response_rx_hex") != response_hex
        or target.get("optimizer_response_status")
        != "captured and matched target slam_job_t"
        or target.get("mapping_outcome")
        != "gazebo_telemetry_streamed_over_ipc; optimizer_completed"
    ):
        raise ValueError("target optimizer response capture is incomplete")

    source = _mapping(target.get("gazebo_source"), "target Gazebo source")
    run_exit_codes = run.get("process_exit_codes")
    if (
        source.get("drone") != run.get("drone")
        or source.get("sample_count") != sample_count
        or source.get("process_exit_codes") != run_exit_codes
        or source.get("mission_terminal_phase") != "COMPLETE"
    ):
        raise ValueError("target source summary differs from the completed Gazebo run")

    diagnostics = _mapping(target.get("spi_diagnostics"), "target SPI diagnostics")
    expected_commands = {
        "pose": sample_count,
        "scan": len(expected_scans),
        "optimize": 1,
        "pose_request": 0,
        "check": 0,
        "external_pose": 0,
        "pose_update": 0,
    }
    expected_success = {**expected_commands, "optimize": 0}
    expected_failure = dict.fromkeys(expected_commands, 0)
    if (
        diagnostics.get("received_frames") != len(packets)
        or diagnostics.get("malformed_frames") != 0
        or diagnostics.get("unknown_commands") != 0
        or diagnostics.get("command_frames") != expected_commands
        or diagnostics.get("decode_success") != expected_success
        or diagnostics.get("decode_failure") != expected_failure
        or diagnostics.get("missing_pose_ids") != 0
        or diagnostics.get("nonadvancing_pose_ids") != 0
        or diagnostics.get("completed_optimize_responses") != 1
    ):
        raise ValueError("target SPI diagnostics differ from the expected source trace")
    raw_diagnostics = list(SPI_DIAGNOSTICS_PATTERN.finditer(target_log))
    raw_diagnostics_lines = [
        line
        for line in target_log.splitlines()
        if line.startswith("NANO_C_SLAM_SPI:stats ")
    ]
    if len(raw_diagnostics) != 1 or len(raw_diagnostics_lines) != 1:
        raise ValueError("target log must contain one raw SPI diagnostics record")
    (
        received_frames,
        malformed_frames,
        unknown_commands,
        command_csv,
        success_csv,
        failure_csv,
        missing_pose_ids,
        nonadvancing_pose_ids,
        optimize_responses,
    ) = raw_diagnostics[0].groups()
    raw_command_counts = [int(value) for value in command_csv.split(",")]
    raw_success_counts = [int(value) for value in success_csv.split(",")]
    raw_failure_counts = [int(value) for value in failure_csv.split(",")]
    if (
        int(received_frames) != len(packets)
        or int(malformed_frames) != 0
        or int(unknown_commands) != 0
        or raw_command_counts != list(expected_commands.values())
        or raw_success_counts != list(expected_success.values())
        or raw_failure_counts != list(expected_failure.values())
        or int(missing_pose_ids) != 0
        or int(nonadvancing_pose_ids) != 0
        or int(optimize_responses) != 1
    ):
        raise ValueError("raw target SPI diagnostics differ from the source trace")
    scan_durations = target.get("scan_busy_durations_us")
    if not isinstance(scan_durations, list) or len(scan_durations) != len(
        expected_scans
    ):
        raise ValueError("target log lacks one SCAN timing per source scan")
    raw_scan_matches = list(SCAN_BUSY_PATTERN.finditer(target_log))
    raw_scan_lines = [
        line
        for line in target_log.splitlines()
        if line.startswith("NANO_C_SLAM_GVSOC_SPI_MAIN:scan_busy ")
    ]
    if len(raw_scan_matches) != len(expected_scans) or len(raw_scan_lines) != len(
        expected_scans
    ):
        raise ValueError("target log must contain one raw SCAN timing per source scan")
    for index, (record, pose_id) in enumerate(
        zip(scan_durations, expected_scans, strict=True)
    ):
        item = _mapping(record, f"target scan timing {index}")
        duration = item.get("duration_us")
        raw_index, raw_pose_id, raw_duration = raw_scan_matches[index].groups()
        if (
            item.get("index") != index
            or item.get("pose_id") != pose_id
            or isinstance(duration, bool)
            or not isinstance(duration, int)
            or duration < 0
            or (int(raw_index), int(raw_pose_id), int(raw_duration))
            != (index, pose_id, duration)
        ):
            raise ValueError("target scan timing differs from log or source trace")


def analyze_mapping(
    *,
    target_report_path: Path,
    target_log_path: Path,
    target_elf_path: Path,
    gvsoc_config_path: Path,
    host_transfer_path: Path,
    run_bundle_path: Path,
    session_path: Path,
    world_attestation_path: Path,
    sdf_path: Path,
    expected_route: str | None = None,
) -> dict[str, object]:
    """Produce diagnostic geometry metrics without silently choosing a pass bar."""
    target = load_json_file(target_report_path, label="target report")
    target_log = target_log_path.read_text(encoding="utf-8")
    host_transfer = load_json_file(host_transfer_path, label="host transfer")
    run = load_json_file(run_bundle_path, label="run bundle")
    session = load_json_file(session_path, label="world session")
    world_attestation = load_json_file(
        world_attestation_path, label="world input attestation"
    )
    target = _mapping(target, "target report")
    host_transfer = _mapping(host_transfer, "host transfer")
    run = _mapping(run, "run bundle")
    session = _mapping(session, "world session")
    world_attestation = _mapping(world_attestation, "world input attestation")
    mapping_route = _mapping_route(run, expected_route)
    _verify_world_binding(run, session, session_path, sdf_path, world_attestation)
    if target.get("status") != "passed" or target.get("sequence") != "gazebo-live":
        raise ValueError("target report must prove a live Gazebo-to-GAP9 run")
    gazebo_source = _mapping(target.get("gazebo_source"), "target Gazebo source")
    capture = _mapping(run.get("capture_report"), "Gazebo capture report")
    run_terminal = _mapping(run.get("mission_terminal"), "run mission terminal")
    run_exit_codes = _mapping(run.get("process_exit_codes"), "run exit codes")
    target_hashes = _mapping(target.get("sha256"), "target evidence hashes")
    source_hashes = _mapping(target_hashes.get("sources"), "target source hashes")
    if (
        target_hashes.get("elf") != _sha256(target_elf_path)
        or target_hashes.get("gvsoc_config") != _sha256(gvsoc_config_path)
        or target_hashes.get("host_transfer") != _sha256(host_transfer_path)
        or not source_hashes
        or "cslam-gap9-app/main.c" not in source_hashes
        or any(
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
            for digest in source_hashes.values()
        )
        or validate_config(gvsoc_config_path)
    ):
        raise ValueError("target ELF, config, transfer, or source hashes are invalid")
    if (
        gazebo_source.get("run_id") != run.get("run_id")
        or capture.get("run_id") != run.get("run_id")
        or run_terminal.get("run_id") != run.get("run_id")
        or run_terminal.get("phase") != "COMPLETE"
        or run_exit_codes != {"collector": 0, "mission": 0}
        or gazebo_source.get("mission_terminal_phase") != "COMPLETE"
        or gazebo_source.get("process_exit_codes") != {"collector": 0, "mission": 0}
        or gazebo_source.get("run_bundle_sha256") != _sha256(run_bundle_path)
        or target_hashes.get("gvsoc_log") != _sha256(target_log_path)
        or run.get("scene") != "nearfield-mapping"
    ):
        raise ValueError("run bundle is incomplete or does not match target evidence")
    run_id = run.get("run_id")
    if not isinstance(run_id, str):
        raise ValueError("run bundle has no run ID")
    _, mapping_samples = samples_from_run_bundle(run, expected_run_id=run_id)
    source_poses = [
        (sample.x_m, sample.y_m, sample.yaw_rad) for sample in mapping_samples
    ]
    if len(source_poses) < 2:
        raise ValueError("at least two matched poses are required")
    poses = _parse_target_poses(target_log, expected_count=len(source_poses))
    points = _parse_target_map_points(target_log)
    if target.get("target_optimized_poses") != poses:
        raise ValueError("target pose dump differs from the validated target report")
    if target.get("target_projected_map_points") != points:
        raise ValueError("target point dump differs from the validated target report")
    _verify_target_capture(target, host_transfer, target_log, run, mapping_samples)
    if len(points) >= TARGET_MAP_CAPACITY:
        raise ValueError(
            "target map reached SIZE_POINTS capacity; completeness is unknown"
        )

    pose_errors = [
        (
            math.hypot(pose["x_m"] - source[0], pose["y_m"] - source[1]),
            abs((pose["z_m"] - source[2] + math.pi) % (2 * math.pi) - math.pi),
        )
        for pose, source in zip(poses, source_poses, strict=True)
    ]
    distances_xy = [error[0] for error in pose_errors]
    yaw_errors = [error[1] for error in pose_errors]
    trajectory = [(float(p["x_m"]), float(p["y_m"])) for p in poses]
    source_path = [(pose[0], pose[1]) for pose in source_poses]
    per_pose_errors = [
        {
            "pose_id": index,
            "xy_error_m": xy_error,
            "yaw_error_rad": yaw_error,
        }
        for index, (xy_error, yaw_error) in enumerate(pose_errors)
    ]

    def path_length(path: list[tuple[float, float]]) -> float:
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in itertools.pairwise(path)
        )

    source_length, target_length = path_length(source_path), path_length(trajectory)
    if source_length <= 1e-6 or target_length <= 1e-6:
        raise ValueError("source and target trajectories must both be nondegenerate")
    polygons = _sdf_segments(sdf_path)
    all_segments = [
        (corners[i], corners[(i + 1) % len(corners)])
        for corners in polygons.values()
        for i in range(len(corners))
    ]
    map_points = [(float(p["x_m"]), float(p["y_m"])) for p in points]
    map_distances = [
        min(_point_to_segment_distance(point, segment) for segment in all_segments)
        for point in map_points
    ]
    sorted_distances = sorted(map_distances)

    def percentile(values: list[float], fraction: float) -> float:
        return values[min(len(values) - 1, math.ceil(fraction * len(values)) - 1)]

    return {
        "schema_version": 1,
        "status": "measured-no-acceptance-threshold",
        "validation_claims": {
            "pose_consistency": "diagnostic-no-acceptance-threshold",
            "map_vs_world_geometry": "diagnostic-no-acceptance-threshold",
            "target_optimizer_health": "validated",
            "sensor_model_fidelity": "not-validated",
            "stm32_flight_firmware": "not-validated",
            "target_build_reproducibility": "artifact-hashes-verified-not-rebuilt",
        },
        "run_id": run["run_id"],
        "mapping_route": mapping_route,
        "world_session_id": session["session_id"],
        "coordinate_convention": (
            "Gazebo world-frame XY meters and yaw radians; GAP9 z field is planar yaw. "
            "Trajectory errors compare same-frame values without fitted alignment; "
            "the live bridge initializes this frame from Gazebo odometry."
        ),
        "target_pose_count": len(poses),
        "pose_error": {
            "xy_rmse_m": math.sqrt(sum(v * v for v in distances_xy) / len(poses)),
            "xy_max_m": max(distances_xy),
            "yaw_rmse_rad": math.sqrt(sum(v * v for v in yaw_errors) / len(poses)),
            "yaw_max_rad": max(yaw_errors),
            "per_pose": per_pose_errors,
            "source_xy_bounds": _axis_aligned_bounds(source_path),
            "target_xy_bounds": _axis_aligned_bounds(trajectory),
            "source_path_length_m": source_length,
            "target_path_length_m": target_length,
            "target_to_source_path_length_ratio": target_length / source_length
            if source_length > 0
            else None,
        },
        "target_map": {
            "point_count": len(map_points),
            "xy_bounds": _axis_aligned_bounds(map_points),
            "capacity_points": TARGET_MAP_CAPACITY,
            "capacity_reached": False,
            "sdf_surface_distance_m": {
                "median": percentile(sorted_distances, 0.5),
                "p95": percentile(sorted_distances, 0.95),
                "max": max(sorted_distances),
                "within_0_05m_fraction": sum(
                    distance <= MAP_TOLERANCE_M for distance in map_distances
                )
                / len(map_distances),
            },
            "point_fraction_within_0_05m_of_model_surface": {
                name: sum(
                    min(
                        _point_to_segment_distance(
                            point,
                            (corners[i], corners[(i + 1) % len(corners)]),
                        )
                        for i in range(len(corners))
                    )
                    <= MAP_TOLERANCE_M
                    for point in map_points
                )
                / len(map_points)
                for name, corners in sorted(polygons.items())
            },
        },
        "geometry": {
            "source_sdf_sha256": _sha256(sdf_path),
            "source_sdf_bytes": sdf_path.stat().st_size,
            "surface_models": sorted(polygons),
            "point_distance_tolerance_m": MAP_TOLERANCE_M,
            "tolerance_note": (
                "Diagnostic bucket only; not an acceptance threshold. "
                "The sensor bridge uses idealized simulator-derived four-ray ranges."
            ),
        },
        "sha256": {
            "target_report": _sha256(target_report_path),
            "target_elf": _sha256(target_elf_path),
            "gvsoc_config": _sha256(gvsoc_config_path),
            "host_transfer": _sha256(host_transfer_path),
            "target_sources": source_hashes,
            "run_bundle": _sha256(run_bundle_path),
            "world_session": _sha256(session_path),
            "world_input_attestation": _sha256(world_attestation_path),
            "source_sdf": _sha256(sdf_path),
            "target_log": _sha256(target_log_path),
        },
        "limitations": [
            "Metrics characterize one synthetic near-field run; no acceptance "
            "thresholds are declared.",
            "The report cross-checks target frames, optimizer response, and SPI "
            "diagnostics against the same-run log and source packets, but does "
            "not independently rebuild the target firmware. ELF, configuration, "
            "and transfer hashes are checked; source hashes are recorded from "
            "capture because source snapshots are not archived.",
            "The Gazebo sensor bridge uses idealized four-ray ranges from the "
            "simulator, not physical ToF behavior.",
            "This does not validate UWB effects or Crazyflie STM32 flight "
            "firmware in Gazebo.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-report", type=Path, required=True)
    parser.add_argument("--target-log", type=Path, required=True)
    parser.add_argument("--target-elf", type=Path, required=True)
    parser.add_argument("--gvsoc-config", type=Path, required=True)
    parser.add_argument("--host-transfer", type=Path, required=True)
    parser.add_argument("--run-bundle", type=Path, required=True)
    parser.add_argument("--world-session", type=Path, required=True)
    parser.add_argument("--world-attestation", type=Path, required=True)
    parser.add_argument("--world-sdf", type=Path, required=True)
    parser.add_argument("--expected-route", choices=("square", "heldout-asymmetric"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = analyze_mapping(
            target_report_path=args.target_report,
            target_log_path=args.target_log,
            target_elf_path=args.target_elf,
            gvsoc_config_path=args.gvsoc_config,
            host_transfer_path=args.host_transfer,
            run_bundle_path=args.run_bundle,
            session_path=args.world_session,
            world_attestation_path=args.world_attestation,
            sdf_path=args.world_sdf,
            expected_route=args.expected_route,
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Target GAP9 map geometry metrics captured: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
