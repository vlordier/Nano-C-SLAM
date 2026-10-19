"""Validate and summarize independent live GAP9 geometry baseline artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
from datetime import UTC, datetime
from pathlib import Path

EXPECTED_RESPONSE_HEX = "3c0003001500830000000100030009000100"
EXPECTED_SDF_SHA256 = "5d2d9cc1a54bcfc7e978225c44aff96b6c1de9e2f3a99431aadb5174681815a2"
BASELINE_COUNT = 5
METRIC_FIELDS = {
    "pose_xy_rmse_m": ("pose_error", "xy_rmse_m"),
    "pose_xy_max_m": ("pose_error", "xy_max_m"),
    "pose_yaw_rmse_rad": ("pose_error", "yaw_rmse_rad"),
    "pose_yaw_max_rad": ("pose_error", "yaw_max_rad"),
    "map_surface_median_m": ("target_map", "sdf_surface_distance_m", "median"),
    "map_surface_p95_m": ("target_map", "sdf_surface_distance_m", "p95"),
    "map_surface_max_m": ("target_map", "sdf_surface_distance_m", "max"),
    "map_fraction_within_0_05m": (
        "target_map",
        "sdf_surface_distance_m",
        "within_0_05m_fraction",
    ),
}


def _mapping(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _read_json(path: Path, label: str) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {label}: {path}") from error
    return _mapping(value, label)


def _sha256(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as error:
        raise ValueError(f"cannot hash artifact: {path}") from error


def _nested_number(
    value: dict[str, object], path: tuple[str, ...], label: str
) -> float:
    current: object = value
    for component in path:
        current = _mapping(current, label).get(component)
    if isinstance(current, bool) or not isinstance(current, (int, float)):
        raise ValueError(f"{label} must contain numeric {'/'.join(path)}")
    number = float(current)
    if not math.isfinite(number):
        raise ValueError(f"{label} contains non-finite {'/'.join(path)}")
    return number


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    number = float(value)
    if not math.isfinite(number) or not 0.0 <= number <= 1.0:
        raise ValueError(f"{label} must be a finite fraction in [0, 1]")
    return number


def _load_baseline(artifact_dir: Path) -> dict[str, object]:
    required = {
        "geometry metrics": artifact_dir / "geometry-metrics.json",
        "Gazebo run": artifact_dir / "gazebo-run.json",
        "target evidence": artifact_dir / "target-evidence.json",
        "host transfer": artifact_dir / "host-transfer.json",
        "target log": artifact_dir / "gvsoc.log",
        "target ELF": artifact_dir / "nano_c_slam_gvsoc_spi_main_probe",
        "GVSOC config": artifact_dir / "gvsoc-config.json",
        "world session": artifact_dir / "world-session.json",
        "world attestation": artifact_dir / "world-input-attestation.json",
        "source SDF": artifact_dir / "world.sdf",
    }
    for label, path in required.items():
        if not path.is_file():
            raise ValueError(f"{artifact_dir} is missing {label}: {path.name}")

    metrics = _read_json(required["geometry metrics"], "geometry metrics")
    run = _read_json(required["Gazebo run"], "Gazebo run")
    target = _read_json(required["target evidence"], "target evidence")
    transfer = _read_json(required["host transfer"], "host transfer")
    session = _read_json(required["world session"], "world session")
    attestation = _read_json(required["world attestation"], "world attestation")

    run_id = run.get("run_id")
    session_id = session.get("session_id")
    if not isinstance(run_id, str) or not isinstance(session_id, str):
        raise ValueError(f"{artifact_dir} has missing run or world session identity")
    if (
        run.get("scene") != "nearfield-mapping"
        or metrics.get("run_id") != run_id
        or metrics.get("world_session_id") != session_id
        or run.get("gazebo_world_session_id") != session_id
    ):
        raise ValueError(f"{artifact_dir} mixes run or world identities")
    route = metrics.get("mapping_route")
    if route is None:
        # Attempt 12 predates the explicit route field. Its archived mission log
        # and README record the route; validate both before accepting the legacy
        # schema rather than guessing from metric values.
        mission_log = str(_mapping(run.get("raw_logs"), "raw logs").get("mission", ""))
        readme = (artifact_dir / "README.md").read_text(encoding="utf-8")
        if (
            "Starting bounded square mapping path" not in mission_log
            or "default `square` route" not in readme
        ):
            raise ValueError(f"{artifact_dir} has no evidence for the square route")
        route = "square"
    if route != "square":
        raise ValueError(f"{artifact_dir} uses unexpected mapping route: {route}")

    terminal = _mapping(run.get("mission_terminal"), "mission terminal")
    capture = _mapping(run.get("capture_report"), "capture report")
    exit_codes = _mapping(run.get("process_exit_codes"), "process exit codes")
    if terminal.get("phase") != "COMPLETE" or terminal.get("run_id") != run_id:
        raise ValueError(f"{artifact_dir} mission did not complete for its run ID")
    if capture.get("input_sample_count") != 60:
        raise ValueError(f"{artifact_dir} does not contain 60 host samples")
    if exit_codes.get("collector") != 0 or exit_codes.get("mission") != 0:
        raise ValueError(f"{artifact_dir} has a failed Gazebo mapping child")
    if target.get("frame_count") != 64:
        raise ValueError(f"{artifact_dir} target evidence does not contain 64 frames")
    target_source = _mapping(target.get("gazebo_source"), "target Gazebo source")
    if (
        target_source.get("run_id") != run_id
        or target_source.get("mission_terminal_phase") != "COMPLETE"
        or target_source.get("run_bundle_sha256") != _sha256(required["Gazebo run"])
    ):
        raise ValueError(f"{artifact_dir} target evidence is not bound to its run")
    if (
        transfer.get("run_id") != run_id
        or transfer.get("control_phase") != "optimizer_response_captured"
        or transfer.get("response_rx_hex") != EXPECTED_RESPONSE_HEX
        or not isinstance(transfer.get("frames"), list)
        or len(transfer["frames"]) != 64
    ):
        raise ValueError(f"{artifact_dir} has no complete live optimizer response")
    log = required["target log"].read_text(encoding="utf-8")
    if "NANO_C_SLAM_GVSOC_SPI_MAIN:gazebo_capture_ok packets=64" not in log:
        raise ValueError(f"{artifact_dir} lacks the target mapping completion marker")

    sdf_hash = _sha256(required["source SDF"])
    metric_geometry = _mapping(metrics.get("geometry"), "geometry metrics")
    if (
        sdf_hash != EXPECTED_SDF_SHA256
        or metric_geometry.get("source_sdf_sha256") != sdf_hash
    ):
        raise ValueError(f"{artifact_dir} uses a different baseline SDF")
    archived_session_hash = session.get("world_sdf_sha256")
    if archived_session_hash is None:
        # Legacy attempt 12 records this digest in its verified active-server
        # attestation and geometry report rather than the session object.
        archived_session_hash = attestation.get("archived_sdf_sha256")
    session_bytes = session.get("world_sdf_bytes")
    if (
        session_bytes is not None
        and session_bytes != required["source SDF"].stat().st_size
    ):
        raise ValueError(f"{artifact_dir} world session has an unexpected SDF size")
    if (
        archived_session_hash != sdf_hash
        or attestation.get("session_id") != session_id
        or attestation.get("world_sdf_sha256") != sdf_hash
        or attestation.get("archived_sdf_sha256") != sdf_hash
        or attestation.get("verified_while_server_active") is not True
    ):
        raise ValueError(f"{artifact_dir} world session or attestation is inconsistent")

    hashes = _mapping(metrics.get("sha256"), "geometry hashes")
    file_hash_keys = {
        "target_report": required["target evidence"],
        "target_elf": required["target ELF"],
        "gvsoc_config": required["GVSOC config"],
        "host_transfer": required["host transfer"],
        "run_bundle": required["Gazebo run"],
        "world_session": required["world session"],
        "world_input_attestation": required["world attestation"],
        "source_sdf": required["source SDF"],
        "target_log": required["target log"],
    }
    if any(hashes.get(key) != _sha256(path) for key, path in file_hash_keys.items()):
        raise ValueError(f"{artifact_dir} geometry report has stale artifact hashes")

    row: dict[str, object] = {
        "artifact": artifact_dir.as_posix(),
        "run_id": run_id,
        "world_session_id": session_id,
        "sdf_sha256": sdf_hash,
        "host_sample_count": capture["input_sample_count"],
        "target_pose_count": metrics.get("target_pose_count"),
        "target_map_point_count": _nested_number(
            metrics, ("target_map", "point_count"), "geometry metrics"
        ),
        "mission_phase": terminal["phase"],
        "mapping_route": route,
        "response_rx_hex": transfer["response_rx_hex"],
    }
    for name, path in METRIC_FIELDS.items():
        row[name] = _nested_number(metrics, path, "geometry metrics")
    coverage = _mapping(
        _mapping(metrics.get("target_map"), "target map").get(
            "point_fraction_within_0_05m_of_model_surface"
        ),
        "per-model coverage",
    )
    row["model_coverage_within_0_05m"] = {
        name: _finite_number(value, f"coverage for {name}")
        for name, value in coverage.items()
    }
    if row["target_pose_count"] != 60:
        raise ValueError(f"{artifact_dir} does not contain 60 target poses")
    return row


def summarize_baselines(artifact_dirs: list[Path]) -> dict[str, object]:
    """Validate exactly five same-world completed runs and calculate spread."""
    if len(artifact_dirs) != BASELINE_COUNT:
        raise ValueError(f"exactly {BASELINE_COUNT} baseline artifacts are required")
    rows = [_load_baseline(path.resolve()) for path in artifact_dirs]
    if len({row["run_id"] for row in rows}) != BASELINE_COUNT:
        raise ValueError("baseline run IDs must be unique")
    if len({row["world_session_id"] for row in rows}) != BASELINE_COUNT:
        raise ValueError("baseline world session IDs must be unique")
    if len({row["sdf_sha256"] for row in rows}) != 1:
        raise ValueError("baseline runs must use the exact same SDF")

    aggregate: dict[str, dict[str, float]] = {}
    for name in (*METRIC_FIELDS, "target_map_point_count"):
        values = [float(row[name]) for row in rows]
        aggregate[name] = {
            "min": min(values),
            "max": max(values),
            "mean": statistics.mean(values),
            "sample_sd": statistics.stdev(values),
        }
    model_names = sorted(rows[0]["model_coverage_within_0_05m"])
    if any(sorted(row["model_coverage_within_0_05m"]) != model_names for row in rows):
        raise ValueError("baseline runs report different wall/obstacle model sets")
    aggregate["model_coverage_within_0_05m"] = {
        model: {
            "min": min(
                float(row["model_coverage_within_0_05m"][model]) for row in rows
            ),
            "max": max(
                float(row["model_coverage_within_0_05m"][model]) for row in rows
            ),
            "mean": statistics.mean(
                float(row["model_coverage_within_0_05m"][model]) for row in rows
            ),
            "sample_sd": statistics.stdev(
                float(row["model_coverage_within_0_05m"][model]) for row in rows
            ),
        }
        for model in model_names
    }
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "baseline_count": BASELINE_COUNT,
        "common_sdf_sha256": rows[0]["sdf_sha256"],
        "route": "square",
        "claim_boundary": (
            "Five runs characterize one idealized synthetic near-field scene; "
            "they do not validate physical ToF/UWB behavior, STM32 flight "
            "firmware, or real-world mapping accuracy."
        ),
        "runs": rows,
        "aggregate": aggregate,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--artifact",
        type=Path,
        action="append",
        required=True,
        help="baseline artifact directory (supply exactly five)",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        summary = summarize_baselines(args.artifact)
        content = json.dumps(summary, indent=2, allow_nan=False) + "\n"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding="utf-8")
    except (OSError, ValueError) as error:
        print(f"GAP9 geometry baseline summary failed: {error}", file=sys.stderr)
        return 1
    print(f"GAP9 geometry baseline summary written: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
