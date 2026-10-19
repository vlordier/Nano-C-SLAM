"""Apply the frozen synthetic GAP9 geometry acceptance profile."""

from __future__ import annotations

import argparse
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
    metrics: dict[str, object], profile: dict[str, object]
) -> list[str]:
    """Return acceptance failures; malformed or non-finite metrics fail closed."""
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

    if metrics.get("target_pose_count") != profile.get("required_target_pose_count"):
        failures.append("target_pose_count: does not match the required count")
    if metrics.get("host_sample_count") != profile.get("required_host_sample_count"):
        failures.append("host_sample_count: does not match the required count")
    if metrics.get("mission_phase") != "COMPLETE":
        failures.append("mission_phase: mission is not COMPLETE")
    if metrics.get("mapping_route") != profile.get("route"):
        failures.append("mapping_route: does not match the frozen route")
    if metrics.get("sdf_sha256") != profile.get("sdf_sha256"):
        failures.append("sdf_sha256: does not match the frozen world")
    return failures


def gate_report(
    metrics: dict[str, object], profile: dict[str, object]
) -> dict[str, object]:
    """Describe the frozen profile and each observed comparison for auditing."""
    limits = _object(profile.get("limits"), "profile limits")
    gates: dict[str, dict[str, object]] = {}
    for key, rule in limits.items():
        rule_obj = _object(rule, f"limit {key}")
        gates[key] = {
            "observed": metrics.get(key),
            "direction": rule_obj.get("direction"),
            "threshold": rule_obj.get("value"),
        }
    failures = validate_geometry(metrics, profile)
    return {
        "profile_id": profile.get("profile_id"),
        "status": "FAIL" if failures else "PASS",
        "gates": gates,
        "failures": failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    try:
        metrics = _json(args.metrics, "geometry metrics")
        profile = _json(args.profile, "geometry profile")
        report = gate_report(metrics, profile)
    except (OSError, ValueError) as error:
        print(f"GAP9 geometry validation failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, allow_nan=False))
    if report["failures"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
