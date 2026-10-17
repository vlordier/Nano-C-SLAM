"""Pure sample conversion and validation for host-side mapping replay."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise
from statistics import median


@dataclass(frozen=True)
class MappingSample:
    """One timestamped pose and the four synthetic Gazebo range readings."""

    sample_id: int
    timestamp_ms: int
    x_m: float
    y_m: float
    yaw_rad: float
    ranges_mm: tuple[int, int, int, int]

    def to_record(self) -> str:
        """Serialize the sample for the native Nano-C-SLAM replay executable."""
        if not 0 <= self.sample_id <= 32767:
            raise ValueError("sample_id must fit a nonnegative int16")
        if not 0 <= self.timestamp_ms <= 2147483647:
            raise ValueError("timestamp_ms must fit a nonnegative int32")
        if len(self.ranges_mm) != 4:
            raise ValueError("a Gazebo sample must contain four range values")
        if not all(
            math.isfinite(value) for value in (self.x_m, self.y_m, self.yaw_rad)
        ):
            raise ValueError("pose coordinates must be finite")
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in self.ranges_mm
        ):
            raise ValueError("range values must be integer millimeters")
        if any(value < -1 or value > 32767 for value in self.ranges_mm):
            raise ValueError("range values must fit the Nano-C-SLAM int16 format")
        fields = (
            str(self.sample_id),
            str(self.timestamp_ms),
            format(self.x_m, ".9g"),
            format(self.y_m, ".9g"),
            format(self.yaw_rad, ".9g"),
            *(str(value) for value in self.ranges_mm),
        )
        return " ".join(fields)


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a finite, nonzero ROS quaternion."""
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError("odometry quaternion must be finite")
    if math.hypot(math.hypot(x, y), math.hypot(z, w)) < 1e-12:
        raise ValueError("odometry quaternion must be nonzero")
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def summarize_sample_path(samples: Sequence[MappingSample]) -> dict[str, object]:
    """Summarize raw horizontal odometry to make spatial excitation auditable."""
    if not samples:
        raise ValueError("at least one mapping sample is required")
    for sample in samples:
        sample.to_record()
    path_length_m = sum(
        math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m)
        for previous, current in pairwise(samples)
    )
    return {
        "x_bounds_m": [
            min(sample.x_m for sample in samples),
            max(sample.x_m for sample in samples),
        ],
        "y_bounds_m": [
            min(sample.y_m for sample in samples),
            max(sample.y_m for sample in samples),
        ],
        "horizontal_path_length_m": path_length_m,
    }


def summarize_range_accounting(
    samples: Sequence[MappingSample], *, range_limit_mm: int = 1400
) -> dict[str, object]:
    """Reconcile raw, sensor-valid, and mapping-gate range counts."""
    if (
        isinstance(range_limit_mm, bool)
        or not isinstance(range_limit_mm, int)
        or range_limit_mm < 0
    ):
        raise ValueError("range limit must be a nonnegative integer millimeter value")
    if not samples:
        raise ValueError("at least one sample is required for range accounting")
    for sample in samples:
        sample.to_record()

    directions = ("front", "back", "left", "right")
    readings = [value for sample in samples for value in sample.ranges_mm]
    valid = [value for value in readings if value >= 0]
    if not valid:
        raise ValueError("range accounting requires at least one valid reading")
    accepted = [value for value in valid if value <= range_limit_mm]
    by_direction = {
        direction: {
            "valid": sum(sample.ranges_mm[index] >= 0 for sample in samples),
            "accepted_at_or_below_limit": sum(
                0 <= sample.ranges_mm[index] <= range_limit_mm for sample in samples
            ),
            "distinct_accepted_values": len(
                {
                    sample.ranges_mm[index]
                    for sample in samples
                    if 0 <= sample.ranges_mm[index] <= range_limit_mm
                }
            ),
        }
        for index, direction in enumerate(directions)
    }
    return {
        "input_valid_range_bounds_mm": {
            "minimum": min(valid),
            "maximum": max(valid),
        },
        "range_accounting": {
            "readings_total": len(readings),
            "invalid": len(readings) - len(valid),
            "valid": len(valid),
            "accepted_at_or_below_limit": len(accepted),
            "rejected_above_limit": len(valid) - len(accepted),
            "by_direction": by_direction,
        },
    }


def validate_mapping_result(
    result: object, *, minimum_poses: int = 60
) -> dict[str, object]:
    """Require three scans, stored constraints, and a finite optimized graph."""
    if not isinstance(result, dict):
        raise ValueError("mapping output must be a JSON object")
    normalized_result: dict[str, object] = {}
    for key, value in result.items():
        if not isinstance(key, str):
            raise ValueError("mapping result keys must be strings")
        normalized_result[key] = value
    required_counts = (
        "pose_count",
        "scan_count",
        "map_point_count",
        "icp_pair_attempt_count",
        "icp_constraint_add_attempt_count",
        "icp_constraint_accepted_insertion_count",
        "stored_constraint_count",
        "optimizer_input_constraint_count",
        "optimizer_input_unique_edge_count",
        "main_graph_size",
    )
    counts: dict[str, int] = {}
    for name in required_counts:
        value = normalized_result.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"mapping result {name} must be a nonnegative integer")
        counts[name] = value
    for obsolete_name in ("optimized_constraint_count", "icp_constraint_count"):
        if obsolete_name in normalized_result:
            raise ValueError(f"mapping result uses unsupported {obsolete_name} claim")
    if counts["pose_count"] < minimum_poses:
        raise ValueError("mapping result contains too few poses for three scan windows")
    if (
        counts["scan_count"] < 3
        or counts["map_point_count"] == 0
        or counts["optimizer_input_unique_edge_count"] < 2
    ):
        raise ValueError(
            "mapping result needs three scans, two optimizer edges, and map points"
        )
    if (
        counts["icp_constraint_accepted_insertion_count"]
        > counts["icp_constraint_add_attempt_count"]
        or counts["icp_constraint_accepted_insertion_count"]
        > counts["stored_constraint_count"]
    ):
        raise ValueError("mapping constraint insertion counts are inconsistent")
    if "optimized_pose_count" in normalized_result:
        raise ValueError("mapping result uses unsupported optimized_pose_count claim")
    legacy_solver_evidence = normalized_result.get("solver_execution_evidence")
    if legacy_solver_evidence is not None and legacy_solver_evidence != (
        "not-recorded-in-legacy-capture"
    ):
        raise ValueError("legacy solver evidence must identify unrecorded fields")
    has_nan = normalized_result.get("has_nan")
    if (
        counts["main_graph_size"] < 3
        or isinstance(has_nan, bool)
        or not isinstance(has_nan, int)
        or has_nan != 0
    ):
        raise ValueError("pose graph did not produce finite output")
    legacy_claims = normalized_result.get("legacy_solver_claims")
    if legacy_claims is not None and (
        not isinstance(legacy_claims, dict)
        or legacy_claims.get("solver_return_iterations_and_completion")
        != "not_recorded_in_original_capture; do not infer"
    ):
        raise ValueError("legacy solver evidence must state unrecorded fields")
    if counts["main_graph_size"] > counts["scan_count"]:
        raise ValueError("pose graph cannot contain more nodes than captured scans")
    if (
        counts["optimizer_input_constraint_count"] < counts["stored_constraint_count"]
        or counts["optimizer_input_unique_edge_count"]
        != counts["optimizer_input_constraint_count"]
    ):
        raise ValueError("optimizer input does not reconcile to unique stored edges")
    pair_fields = (
        ("icp_pair_attempt_count", "icp_pair_attempt_pairs"),
        (
            "icp_constraint_accepted_insertion_count",
            "icp_constraint_accepted_endpoint_pairs",
        ),
        ("stored_constraint_count", "stored_unique_endpoint_pairs"),
        (
            "optimizer_input_unique_edge_count",
            "optimizer_input_unique_edge_pairs",
        ),
    )
    normalized_pairs: dict[str, list[tuple[int, int]]] = {}
    for count_field, pairs_field in pair_fields:
        pair_values = normalized_result.get(pairs_field)
        if not isinstance(pair_values, list) or len(pair_values) != counts[count_field]:
            raise ValueError(f"mapping result {pairs_field} does not match its count")
        pairs: list[tuple[int, int]] = []
        for pair in pair_values:
            if (
                not isinstance(pair, list)
                or len(pair) != 2
                or any(
                    isinstance(index, bool) or not isinstance(index, int) or index < 0
                    for index in pair
                )
                or pair[0] >= pair[1]
            ):
                raise ValueError(f"mapping result {pairs_field} has invalid endpoints")
            pairs.append((pair[0], pair[1]))
        if len(set(pairs)) != len(pairs):
            raise ValueError(f"mapping result {pairs_field} contains duplicate edges")
        normalized_pairs[pairs_field] = pairs

    attempts = normalized_pairs["icp_pair_attempt_pairs"]
    accepted = normalized_pairs["icp_constraint_accepted_endpoint_pairs"]
    stored = normalized_pairs["stored_unique_endpoint_pairs"]
    optimizer_edges = normalized_pairs["optimizer_input_unique_edge_pairs"]
    if (
        counts["icp_constraint_add_attempt_count"] > len(attempts)
        or counts["icp_constraint_accepted_insertion_count"]
        > counts["icp_constraint_add_attempt_count"]
        or counts["icp_constraint_accepted_insertion_count"] > len(stored)
        or not set(accepted).issubset(attempts)
        or not set(accepted).issubset(stored)
        or any(second >= counts["pose_count"] for _, second in stored)
        or any(second >= counts["main_graph_size"] for _, second in optimizer_edges)
    ):
        raise ValueError("mapping constraint endpoint accounting is inconsistent")
    if not set(normalized_pairs["icp_constraint_accepted_endpoint_pairs"]).issubset(
        normalized_pairs["stored_unique_endpoint_pairs"]
    ):
        raise ValueError("accepted ICP endpoints are missing from stored constraints")
    arrays: dict[str, list[list[float | int]]] = {}
    for name in ("poses", "points"):
        values = normalized_result.get(name)
        if not isinstance(values, list) or not values:
            raise ValueError(f"mapping result {name} must be a nonempty array")
        vectors: list[list[float | int]] = []
        for vector in values:
            if not isinstance(vector, list):
                raise ValueError(f"mapping result {name} contains invalid coordinates")
            expected_width = 3 if name == "poses" else 2
            if len(vector) != expected_width:
                raise ValueError(f"mapping result {name} contains invalid coordinates")
            coordinates: list[float | int] = []
            for value in vector:
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (float, int))
                    or not math.isfinite(value)
                ):
                    raise ValueError(
                        f"mapping result {name} contains invalid coordinates"
                    )
                coordinates.append(value)
            vectors.append(coordinates)
        arrays[name] = vectors
    if len(arrays["poses"]) != counts["pose_count"]:
        raise ValueError("mapping pose array does not match pose_count")
    if len(arrays["points"]) != counts["map_point_count"]:
        raise ValueError("mapping point array does not match map_point_count")
    return normalized_result


def validate_capture_report(report: object) -> dict[str, object]:
    """Validate cross-field invariants for a live stamped Gazebo capture.

    This deliberately does not accept synthetic replay fixtures: those validate
    through ``validate_mapping_result`` and carry no live capture provenance.
    """
    if not isinstance(report, dict):
        raise ValueError("capture report must be a JSON object")
    normalized_report: dict[str, object] = {}
    for key, value in report.items():
        if not isinstance(key, str):
            raise ValueError("capture report keys must be strings")
        normalized_report[key] = value
    report = normalized_report
    schema_version = report.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 1
    ):
        raise ValueError("capture report has an unsupported schema version")
    if report.get("evidence_class") != "host-execution-of-owned-gap9-mapping-core":
        raise ValueError("capture report has an unsupported evidence class")
    if report.get("mapping_claim") != (
        "host mapping pipeline execution only; map accuracy and completeness "
        "are not measured"
    ):
        raise ValueError("capture report must state the bounded mapping claim")

    drone = report.get("drone")
    provenance = report.get("source_provenance")
    if not isinstance(drone, str) or not drone:
        raise ValueError("capture report is missing drone identity")
    if (
        not isinstance(provenance, dict)
        or not isinstance(provenance.get("revision"), str)
        or not provenance["revision"]
        or not isinstance(provenance.get("worktree_dirty"), bool)
    ):
        raise ValueError("capture report is missing source provenance")

    samples = report.get("input_samples")
    pairs = report.get("input_pair_provenance")
    count = report.get("input_sample_count")
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("capture report sample count must be positive")
    if not isinstance(samples, list) or len(samples) != count:
        raise ValueError("capture samples do not match input_sample_count")
    if not isinstance(pairs, list) or len(pairs) != count:
        raise ValueError("pair provenance does not match input_sample_count")
    limits = report.get("pairing_limits")
    if not isinstance(limits, dict):
        raise ValueError("capture report is missing pairing limits")
    max_age = limits.get("maximum_age_ns")
    max_skew = limits.get("maximum_skew_ns")
    if any(
        isinstance(v, bool) or not isinstance(v, int) or v < 0
        for v in (max_age, max_skew)
    ):
        raise ValueError("capture pairing limits must be nonnegative integers")

    previous_stamp = -1
    previous_sequence = -1
    valid_ranges: list[int] = []
    direction_valid = [0, 0, 0, 0]
    direction_accepted = [0, 0, 0, 0]
    for index, (record, pair) in enumerate(zip(samples, pairs, strict=True)):
        if not isinstance(record, str) or not isinstance(pair, dict):
            raise ValueError("capture sample or pair provenance has invalid format")
        fields = record.split()
        if len(fields) != 9:
            raise ValueError("capture sample must contain pose and four range fields")
        try:
            sample_id = int(fields[0])
            timestamp_ms = int(fields[1])
            pose = tuple(float(value) for value in fields[2:5])
            ranges = tuple(int(value) for value in fields[5:])
        except ValueError as error:
            raise ValueError("capture sample contains non-integer fields") from error
        if (
            sample_id != index
            or timestamp_ms < 0
            or len(pose) != 3
            or not all(math.isfinite(value) for value in pose)
            or len(ranges) != 4
        ):
            raise ValueError("capture sample identity or timestamp is inconsistent")
        if pair.get("drone_id") != drone:
            raise ValueError("capture pair belongs to a different drone")
        range_stamp = pair.get("range_stamp_ns")
        odom_stamp = pair.get("odometry_stamp_ns")
        skew = pair.get("pair_skew_ns")
        range_age = pair.get("range_age_ns")
        odometry_age = pair.get("odometry_age_ns")
        pair_sequence = pair.get("range_sequence")
        ints = (range_stamp, odom_stamp, skew, pair_sequence)
        if any(isinstance(v, bool) or not isinstance(v, int) for v in ints):
            raise ValueError("capture pair timestamps and sequence must be integers")
        if range_stamp < previous_stamp or range_stamp < 0 or odom_stamp < 0:
            raise ValueError("capture source timestamps are missing or out of order")
        if pair_sequence <= previous_sequence:
            raise ValueError("capture range sequences are not strictly increasing")
        previous_stamp = range_stamp
        previous_sequence = pair_sequence
        if abs(range_stamp - odom_stamp) != skew or skew > max_skew:
            raise ValueError("capture pair skew is inconsistent or exceeds its limit")
        if range_age is not None or odometry_age is not None:
            if any(
                isinstance(age, bool) or not isinstance(age, int)
                for age in (range_age, odometry_age)
            ):
                raise ValueError("capture pair ages must be recorded together")
            if any(age < 0 or age > max_age for age in (range_age, odometry_age)):
                raise ValueError(
                    "capture pair age is inconsistent or exceeds its limit"
                )
            if range_age - odometry_age != odom_stamp - range_stamp:
                raise ValueError("capture pair ages do not match source timestamps")
        if timestamp_ms != odom_stamp // 1_000_000:
            raise ValueError(
                "sample timestamp does not match source odometry timestamp"
            )
        for direction, value in enumerate(ranges):
            if value < -1 or value > 32767:
                raise ValueError("capture range is outside the packet's int16 domain")
            if value >= 0:
                valid_ranges.append(value)
                direction_valid[direction] += 1
                if value <= 1400:
                    direction_accepted[direction] += 1

    range_limit = report.get("mapping_range_limit_mm")
    if (
        isinstance(range_limit, bool)
        or not isinstance(range_limit, int)
        or range_limit != 1400
    ):
        raise ValueError("capture report does not use the exact 1400 mm cutoff")
    accounting = report.get("range_accounting")
    if not isinstance(accounting, dict):
        raise ValueError("capture report is missing range accounting")
    expected_total = count * 4
    expected_valid = len(valid_ranges)
    expected_accepted = sum(value <= 1400 for value in valid_ranges)
    expected_counts = {
        "readings_total": expected_total,
        "invalid": expected_total - expected_valid,
        "valid": expected_valid,
        "accepted_at_or_below_limit": expected_accepted,
        "rejected_above_limit": expected_valid - expected_accepted,
    }
    if any(
        isinstance(accounting.get(key), bool)
        or not isinstance(accounting.get(key), int)
        or accounting.get(key) != value
        for key, value in expected_counts.items()
    ):
        raise ValueError("capture range accounting does not reconcile with samples")
    by_direction = accounting.get("by_direction")
    if not isinstance(by_direction, dict):
        raise ValueError("capture report is missing per-direction range accounting")
    for index, (direction, valid, accepted) in enumerate(
        zip(
            ("front", "back", "left", "right"),
            direction_valid,
            direction_accepted,
            strict=True,
        )
    ):
        values = by_direction.get(direction)
        if (
            not isinstance(values, dict)
            or isinstance(values.get("valid"), bool)
            or not isinstance(values.get("valid"), int)
            or isinstance(values.get("accepted_at_or_below_limit"), bool)
            or not isinstance(values.get("accepted_at_or_below_limit"), int)
            or isinstance(values.get("distinct_accepted_values"), bool)
            or not isinstance(values.get("distinct_accepted_values"), int)
            or values.get("valid") != valid
            or values.get("accepted_at_or_below_limit") != accepted
        ):
            raise ValueError("per-direction range accounting does not reconcile")
        distinct = len(
            {
                int(record.split()[5 + index])
                for record in samples
                if 0 <= int(record.split()[5 + index]) <= 1400
            }
        )
        if values.get("distinct_accepted_values") != distinct:
            raise ValueError(
                "per-direction distinct range accounting does not reconcile"
            )

    bounds = report.get("input_valid_range_bounds_mm")
    if bounds != {"minimum": min(valid_ranges), "maximum": max(valid_ranges)}:
        raise ValueError("reported range bounds do not match captured input")
    sim_time = report.get("input_sim_time_ms")
    if sim_time != {
        "start": int(samples[0].split()[1]),
        "end": int(samples[-1].split()[1]),
    }:
        raise ValueError("reported simulation time does not match captured input")
    range_stamps = [int(pair["range_stamp_ns"]) for pair in pairs]
    intervals = [
        current - previous
        for previous, current in pairwise(range_stamps)
        if current > previous
    ]
    measured_frequency = 1_000_000_000 / median(intervals) if intervals else 0.0
    reported_frequency = report.get("median_input_frequency_hz")
    if (
        isinstance(reported_frequency, bool)
        or not isinstance(reported_frequency, (int, float))
        or not math.isfinite(reported_frequency)
        or not math.isclose(reported_frequency, measured_frequency, rel_tol=1e-9)
    ):
        raise ValueError("reported input frequency does not match source timestamps")

    mapping = validate_mapping_result(report.get("mapping"), minimum_poses=1)
    map_point_count = mapping["map_point_count"]
    if not isinstance(map_point_count, int):
        raise ValueError("mapping point count must be an integer")
    if map_point_count > expected_accepted:
        raise ValueError("mapping contains more points than accepted input rays")
    window_counts = report.get("scan_window_in_range_sample_counts")
    if not isinstance(window_counts, list):
        raise ValueError("capture report is missing scan-window accounting")
    derived_windows = []
    for start in range(0, count, 20):
        window_samples = samples[start : start + 20]
        derived_windows.append(
            sum(
                any(0 <= int(value) <= 1400 for value in record.split()[5:])
                for record in window_samples
            )
        )
    if (
        not all(
            isinstance(value, int) and not isinstance(value, bool)
            for value in window_counts
        )
        or window_counts != derived_windows
        or mapping["scan_count"] != len(derived_windows)
    ):
        raise ValueError(
            "scan count or window accounting does not match captured input"
        )
    three_direction_counts = report.get("scan_window_three_direction_sample_counts")
    derived_three_direction = [
        sum(
            sum(0 <= int(value) <= 1400 for value in record.split()[5:]) >= 3
            for record in samples[start : start + 20]
        )
        for start in range(0, count, 20)
    ]
    if (
        not isinstance(three_direction_counts, list)
        or not all(
            isinstance(value, int) and not isinstance(value, bool)
            for value in three_direction_counts
        )
        or three_direction_counts != derived_three_direction
    ):
        raise ValueError("three-direction scan-window accounting does not reconcile")
    return report
