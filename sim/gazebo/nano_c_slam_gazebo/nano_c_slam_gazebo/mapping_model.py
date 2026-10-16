"""Pure sample conversion and validation for host-side mapping replay."""

from __future__ import annotations

import math
from dataclasses import dataclass


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


def validate_mapping_result(
    result: object, *, minimum_poses: int = 40
) -> dict[str, object]:
    """Require a nonempty, finite map with at least one optimized graph."""
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
        "icp_constraint_count",
        "optimized_pose_count",
        "main_graph_size",
    )
    counts: dict[str, int] = {}
    for name in required_counts:
        value = normalized_result.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"mapping result {name} must be a nonnegative integer")
        counts[name] = value
    if counts["pose_count"] < minimum_poses:
        raise ValueError("mapping result contains too few poses for two scan windows")
    if (
        counts["scan_count"] < 2
        or counts["map_point_count"] == 0
        or counts["icp_constraint_count"] == 0
    ):
        raise ValueError(
            "mapping result needs two scans, an ICP constraint, and map points"
        )
    if counts["optimized_pose_count"] != counts["pose_count"]:
        raise ValueError("pose graph did not optimize all captured poses")
    if counts["main_graph_size"] < 2 or normalized_result.get("has_nan") != 0:
        raise ValueError("pose graph optimization did not produce a finite graph")
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
