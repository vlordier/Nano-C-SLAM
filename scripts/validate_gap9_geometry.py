"""Apply the frozen synthetic GAP9 geometry acceptance profile."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path


def _object(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _json(path: Path, label: str) -> dict[str, object]:
    try:
        return _object(json.loads(path.read_text(encoding="utf-8")), label)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {label}: {path}") from error


def validate_geometry(
    metrics: dict[str, object],
    profile: dict[str, object],
    *,
    independent_scenario: bool = False,
    run_bundle: dict[str, object] | None = None,
) -> list[str]:
    """Return acceptance failures; malformed or non-finite metrics fail closed."""
    metrics = _normalize_metrics(metrics, run_bundle)
    limits = _object(profile.get("limits"), "profile limits")
    failures: list[str] = []
    for key, rule in limits.items():
        rule_obj = _object(rule, f"limit {key}")
        direction = rule_obj.get("direction")
        threshold = rule_obj.get("value")
        value = metrics.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            failures.append(f"{key}: missing or non-finite numeric metric")
            continue
        if (
            isinstance(threshold, bool)
            or not isinstance(threshold, (int, float))
            or not math.isfinite(float(threshold))
        ):
            raise ValueError(f"limit {key} has an invalid threshold")
        if direction == "max" and float(value) > float(threshold):
            failures.append(f"{key}: {value} exceeds {threshold}")
        elif direction == "min" and float(value) < float(threshold):
            failures.append(f"{key}: {value} is below {threshold}")
        elif direction not in {"min", "max"}:
            raise ValueError(f"limit {key} has invalid direction {direction!r}")

    if not _matches_count(
        metrics.get("target_pose_count"), profile.get("required_target_pose_count")
    ):
        failures.append("target_pose_count: does not match the required count")
    if not _matches_count(
        metrics.get("host_sample_count"), profile.get("required_host_sample_count")
    ):
        failures.append("host_sample_count: does not match the required count")
    if metrics.get("mission_phase") != "COMPLETE":
        failures.append("mission_phase: mission is not COMPLETE")
    if independent_scenario:
        if run_bundle is None:
            raise ValueError("independent-scenario validation requires --run-bundle")
        if metrics.get("mapping_route") != "heldout-asymmetric":
            failures.append("mapping_route: expected heldout-asymmetric route")
        digest = metrics.get("sdf_sha256")
        if not _valid_sha256(digest) or digest == profile.get("sdf_sha256"):
            failures.append("sdf_sha256: held-out world is not independent of baseline")
    else:
        if metrics.get("mapping_route") != profile.get("route"):
            failures.append("mapping_route: does not match the frozen route")
        if metrics.get("sdf_sha256") != profile.get("sdf_sha256"):
            failures.append("sdf_sha256: does not match the frozen world")
    return failures


def _matches_count(value: object, expected: object) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and isinstance(expected, int)
        and not isinstance(expected, bool)
        and value == expected
    )


def _valid_sha256(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _normalize_metrics(
    metrics: dict[str, object], run_bundle: dict[str, object] | None
) -> dict[str, object]:
    """Accept the analyzer's nested report or the baseline summary's flat row."""
    if "pose_xy_rmse_m" in metrics:
        normalized = dict(metrics)
    else:
        geometry = _object(metrics.get("geometry"), "geometry metrics")
        pose = _object(metrics.get("pose_error"), "pose error metrics")
        target_map = _object(metrics.get("target_map"), "target map metrics")
        surface = _object(
            target_map.get("sdf_surface_distance_m"), "surface distance metrics"
        )
        hashes = _object(metrics.get("sha256"), "geometry hashes")
        sdf_hash = geometry.get("source_sdf_sha256")
        if sdf_hash != hashes.get("source_sdf"):
            raise ValueError("geometry report SDF hashes disagree")
        normalized = {
            "target_map_point_count": target_map.get("point_count"),
            "pose_xy_rmse_m": pose.get("xy_rmse_m"),
            "pose_xy_max_m": pose.get("xy_max_m"),
            "pose_yaw_rmse_rad": pose.get("yaw_rmse_rad"),
            "pose_yaw_max_rad": pose.get("yaw_max_rad"),
            "map_surface_median_m": surface.get("median"),
            "map_surface_p95_m": surface.get("p95"),
            "map_surface_max_m": surface.get("max"),
            "map_fraction_within_0_05m": surface.get("within_0_05m_fraction"),
            "target_pose_count": metrics.get("target_pose_count"),
            "mapping_route": metrics.get("mapping_route"),
            "sdf_sha256": sdf_hash,
            "run_id": metrics.get("run_id"),
            "world_session_id": metrics.get("world_session_id"),
        }
    if run_bundle is not None:
        run_id = run_bundle.get("run_id")
        capture = _object(run_bundle.get("capture_report"), "run capture report")
        terminal = _object(run_bundle.get("mission_terminal"), "run mission terminal")
        exit_codes = _object(run_bundle.get("process_exit_codes"), "run exit codes")
        session_id = run_bundle.get("gazebo_world_session_id")
        if (
            not isinstance(run_id, str)
            or normalized.get("run_id") != run_id
            or not isinstance(session_id, str)
            or normalized.get("world_session_id") != session_id
            or terminal.get("run_id") != run_id
            or terminal.get("phase") != "COMPLETE"
            or capture.get("run_id") != run_id
            or capture.get("input_sample_count") != 60
            or exit_codes != {"collector": 0, "mission": 0}
        ):
            raise ValueError("geometry metrics are not bound to one complete run")
        normalized.update(
            {
                "host_sample_count": capture.get("input_sample_count"),
                "mission_phase": terminal.get("phase"),
                "mapping_route": _route_from_run(run_bundle),
            }
        )
    return normalized


def _route_from_run(run_bundle: dict[str, object]) -> object:
    commands = _object(run_bundle.get("child_commands"), "run child commands")
    mission = commands.get("mission")
    if not isinstance(mission, list) or not all(
        isinstance(argument, str) for argument in mission
    ):
        raise ValueError("run bundle has no valid mission command")
    routes = [
        argument.partition(":=")[2]
        for argument in mission
        if argument.startswith("mapping_route:=")
    ]
    if len(routes) > 1:
        raise ValueError("run mission command contains duplicate routes")
    return routes[0] if routes else "square"


def gate_report(
    metrics: dict[str, object],
    profile: dict[str, object],
    *,
    independent_scenario: bool = False,
    run_bundle: dict[str, object] | None = None,
    profile_sha256: str | None = None,
) -> dict[str, object]:
    """Describe the frozen profile and each observed comparison for auditing."""
    normalized = _normalize_metrics(metrics, run_bundle)
    limits = _object(profile.get("limits"), "profile limits")
    failures = validate_geometry(
        metrics,
        profile,
        independent_scenario=independent_scenario,
        run_bundle=run_bundle,
    )
    gates: dict[str, dict[str, object]] = {}
    for key, rule in limits.items():
        rule_obj = _object(rule, f"limit {key}")
        direction = rule_obj.get("direction")
        threshold = rule_obj.get("value")
        observed = normalized.get(key)
        valid_number = (
            not isinstance(observed, bool)
            and isinstance(observed, (int, float))
            and math.isfinite(float(observed))
        )
        passed = valid_number and (
            (direction == "max" and float(observed) <= float(threshold))
            or (direction == "min" and float(observed) >= float(threshold))
        )
        gates[key] = {
            "observed": observed,
            "direction": direction,
            "threshold": threshold,
            "status": "PASS" if passed else "FAIL",
        }
    report: dict[str, object] = {
        "profile_id": profile.get("profile_id"),
        "profile_sha256": profile_sha256,
        "status": "FAIL" if failures else "PASS",
        "scenario": {
            "independent": independent_scenario,
            "route": normalized.get("mapping_route"),
            "sdf_sha256": normalized.get("sdf_sha256"),
            "run_id": normalized.get("run_id"),
        },
        "gates": gates,
        "failures": failures,
    }
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--run-bundle", type=Path)
    parser.add_argument(
        "--independent-scenario",
        action="store_true",
        help="apply the frozen numeric gates to a distinct held-out route and SDF",
    )
    args = parser.parse_args()
    try:
        metrics = _json(args.metrics, "geometry metrics")
        profile = _json(args.profile, "geometry profile")
        run_bundle = (
            _json(args.run_bundle, "run bundle")
            if args.run_bundle is not None
            else None
        )
        profile_sha256 = hashlib.sha256(args.profile.read_bytes()).hexdigest()
        report = gate_report(
            metrics,
            profile,
            independent_scenario=args.independent_scenario,
            run_bundle=run_bundle,
            profile_sha256=profile_sha256,
        )
    except (OSError, ValueError) as error:
        print(f"GAP9 geometry validation failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, allow_nan=False))
    if report["failures"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
