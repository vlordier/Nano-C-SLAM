"""Measure target GAP9 poses and map points against a bound Gazebo run."""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import cast

from scripts.capture_gap9_gvsoc_spi_main_probe_evidence import (
    TARGET_MAP_CAPACITY,
    _parse_target_map_points,
    _parse_target_poses,
)
from scripts.evidence_json import load_json_file

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
        pose = model.findtext("pose")
        size = model.findtext("link/collision/geometry/box/size")
        if pose is None or size is None:
            raise ValueError(f"SDF model {name} lacks a pose or collision box")
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
        or session_sdf_path != str(launched_sdf_paths[0])
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


def analyze_mapping(
    *,
    target_report_path: Path,
    target_log_path: Path,
    run_bundle_path: Path,
    session_path: Path,
    world_attestation_path: Path,
    sdf_path: Path,
) -> dict[str, object]:
    """Produce diagnostic geometry metrics without silently choosing a pass bar."""
    target = load_json_file(target_report_path, label="target report")
    target_log = target_log_path.read_text(encoding="utf-8")
    run = load_json_file(run_bundle_path, label="run bundle")
    session = load_json_file(session_path, label="world session")
    world_attestation = load_json_file(
        world_attestation_path, label="world input attestation"
    )
    target = _mapping(target, "target report")
    run = _mapping(run, "run bundle")
    session = _mapping(session, "world session")
    world_attestation = _mapping(world_attestation, "world input attestation")
    _verify_world_binding(run, session, session_path, sdf_path, world_attestation)
    if target.get("status") != "passed" or target.get("sequence") != "gazebo-live":
        raise ValueError("target report must prove a live Gazebo-to-GAP9 run")
    gazebo_source = _mapping(target.get("gazebo_source"), "target Gazebo source")
    capture = _mapping(run.get("capture_report"), "Gazebo capture report")
    run_terminal = _mapping(run.get("mission_terminal"), "run mission terminal")
    run_exit_codes = _mapping(run.get("process_exit_codes"), "run exit codes")
    target_hashes = _mapping(target.get("sha256"), "target evidence hashes")
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
    sample_strings = capture.get("input_samples")
    if not isinstance(sample_strings, list) or not sample_strings:
        raise ValueError("run bundle has no source pose samples")
    source_poses: list[tuple[float, float, float]] = []
    for index, sample in enumerate(sample_strings):
        if not isinstance(sample, str):
            raise ValueError("source pose sample must use the text record format")
        fields = sample.split()
        if len(fields) != 9 or fields[0] != str(index):
            raise ValueError("source pose IDs must be contiguous and complete")
        try:
            pose = (float(fields[2]), float(fields[3]), float(fields[4]))
        except ValueError as error:
            raise ValueError("source pose sample has malformed coordinates") from error
        if not all(map(math.isfinite, pose)):
            raise ValueError("source pose sample has nonfinite coordinates")
        source_poses.append(pose)
    if len(source_poses) < 2:
        raise ValueError("at least two matched poses are required")
    poses = _parse_target_poses(target_log, expected_count=len(source_poses))
    points = _parse_target_map_points(target_log)
    if target.get("target_optimized_poses") != poses:
        raise ValueError("target pose dump differs from the validated target report")
    if target.get("target_projected_map_points") != points:
        raise ValueError("target point dump differs from the validated target report")
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
        "run_id": run["run_id"],
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
            "run_bundle": _sha256(run_bundle_path),
            "world_session": _sha256(session_path),
            "world_input_attestation": _sha256(world_attestation_path),
            "source_sdf": _sha256(sdf_path),
            "target_log": _sha256(target_log_path),
        },
        "limitations": [
            "Metrics characterize one synthetic near-field run; no acceptance "
            "thresholds are declared.",
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
    parser.add_argument("--run-bundle", type=Path, required=True)
    parser.add_argument("--world-session", type=Path, required=True)
    parser.add_argument("--world-attestation", type=Path, required=True)
    parser.add_argument("--world-sdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = analyze_mapping(
            target_report_path=args.target_report,
            target_log_path=args.target_log,
            run_bundle_path=args.run_bundle,
            session_path=args.world_session,
            world_attestation_path=args.world_attestation,
            sdf_path=args.world_sdf,
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Target GAP9 map geometry metrics captured: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
