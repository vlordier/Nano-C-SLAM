"""Pure conversions for the simplified Crazyswarm2 Gazebo sensors."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import combinations

BEAM_ORDER = (2, 0, 3, 1)  # front, back, left, right for the upstream SDF scan
EXPECTED_SCAN_ANGLE_MIN_RAD = -3.14159
EXPECTED_SCAN_ANGLE_INCREMENT_RAD = (1.5708 - EXPECTED_SCAN_ANGLE_MIN_RAD) / 3
SCAN_ANGLE_TOLERANCE_RAD = 0.01
FLOAT32_MAX = 3.4028234663852886e38


class OdometrySynchronizationError(ValueError):
    """A valid sample set cannot currently produce synchronized swarm ranges."""


@dataclass(frozen=True)
class OdometryCacheUpdate:
    """Validated candidate swarm cache and publishable pairwise ranges."""

    positions: dict[str, tuple[float, float, float]]
    timestamps_s: dict[str, float]
    distances_m: tuple[tuple[str, str, float], ...] | None


def odometry_stamp_seconds(sec: object, nanosec: object) -> float:
    """Convert normalized ROS time fields without accepting malformed values."""
    if (
        isinstance(sec, bool)
        or not isinstance(sec, int)
        or isinstance(nanosec, bool)
        or not isinstance(nanosec, int)
        or not 0 <= nanosec < 1_000_000_000
    ):
        raise ValueError("odometry timestamp fields are malformed")
    try:
        stamp_s = float(sec) + float(nanosec) / 1e9
    except OverflowError as error:
        raise ValueError("odometry timestamp must be finite") from error
    if (
        isinstance(stamp_s, bool)
        or not isinstance(stamp_s, (int, float))
        or not math.isfinite(stamp_s)
    ):
        raise ValueError("odometry timestamp must be finite")
    return stamp_s


def update_odometry_cache(
    positions: Mapping[str, Sequence[float]],
    timestamps_s: Mapping[str, float],
    *,
    drone: str,
    position: Sequence[float],
    stamp_s: float,
    drone_names: Sequence[str],
    now_s: float,
    max_age_s: float = 0.5,
    max_skew_s: float = 0.1,
) -> OdometryCacheUpdate:
    """Validate one sample, then return a new cache and eligible pair ranges.

    Invalid individual input raises before either input mapping is copied or
    changed. Valid samples update a candidate cache even when another drone's
    sample is currently stale or unsynchronized; no ranges are returned until
    the complete candidate swarm snapshot satisfies the synchronization rules.
    """
    if drone not in drone_names:
        raise ValueError(f"odometry received from unknown drone {drone!r}")
    if len(position) != 3:
        raise ValueError(f"position for {drone} must have three coordinates")
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in position
    ):
        raise ValueError(f"position for {drone} must be numeric")
    try:
        coordinates = tuple(float(value) for value in position)
    except OverflowError as error:
        raise ValueError(f"position for {drone} must be finite") from error
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError(f"position for {drone} must be finite")
    if (
        isinstance(stamp_s, bool)
        or not isinstance(stamp_s, (int, float))
        or not math.isfinite(stamp_s)
    ):
        raise ValueError("odometry timestamp must be finite")
    if not set(positions).issubset(drone_names) or not set(timestamps_s).issubset(
        drone_names
    ):
        raise ValueError("odometry cache contains an unknown drone")
    validate_odometry_timestamps(
        {drone: stamp_s},
        now_s=now_s,
        max_age_s=max_age_s,
        max_skew_s=max_skew_s,
    )
    previous_stamp_s = timestamps_s.get(drone)
    if previous_stamp_s is not None and stamp_s < previous_stamp_s:
        raise ValueError(f"odometry timestamp moved backwards for {drone}")

    candidate_positions = dict(positions)
    candidate_timestamps_s = dict(timestamps_s)
    candidate_positions[drone] = coordinates  # type: ignore[assignment]
    candidate_timestamps_s[drone] = stamp_s
    if set(candidate_positions) != set(drone_names):
        return OdometryCacheUpdate(candidate_positions, candidate_timestamps_s, None)

    try:
        validate_odometry_timestamps(
            candidate_timestamps_s,
            now_s=now_s,
            max_age_s=max_age_s,
            max_skew_s=max_skew_s,
        )
    except OdometrySynchronizationError:
        return OdometryCacheUpdate(candidate_positions, candidate_timestamps_s, None)
    # Dict insertion order follows simulator callback timing. Build the snapshot
    # in the configured drone order so published pair labels remain stable.
    ordered_positions = {name: candidate_positions[name] for name in drone_names}
    distances = pairwise_ranges_m(ordered_positions)
    if any(
        not math.isfinite(distance) or distance > FLOAT32_MAX
        for _, _, distance in distances
    ):
        raise ValueError("pairwise range exceeds finite Float32 representation")
    return OdometryCacheUpdate(candidate_positions, candidate_timestamps_s, distances)


def laser_scan_ranges_mm(
    ranges_m: Sequence[float],
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
) -> tuple[int, int, int, int]:
    """Convert the upstream four-beam scan to front/back/left/right millimeters.

    The Crazyswarm2 Gazebo model has four horizontal rays. Invalid or out-of-range
    readings become -1, matching the STM32 exploration code's invalid sentinel.
    This does not model the real 8x8 VL53L5CX depth matrices.
    """
    if len(ranges_m) != len(BEAM_ORDER):
        raise ValueError(f"expected {len(BEAM_ORDER)} lidar beams, got {len(ranges_m)}")
    if not math.isfinite(angle_min_rad) or not math.isfinite(angle_increment_rad):
        raise ValueError("lidar scan angles must be finite")
    if angle_increment_rad <= 0:
        raise ValueError("lidar scan angle increment must be positive")
    if (
        abs(angle_min_rad - EXPECTED_SCAN_ANGLE_MIN_RAD) > SCAN_ANGLE_TOLERANCE_RAD
        or abs(angle_increment_rad - EXPECTED_SCAN_ANGLE_INCREMENT_RAD)
        > SCAN_ANGLE_TOLERANCE_RAD
    ):
        raise ValueError("lidar scan angles do not match the four-ray SDF convention")
    if not math.isfinite(range_min_m) or not math.isfinite(range_max_m):
        raise ValueError("lidar range bounds must be finite")
    if range_min_m < 0 or range_max_m <= range_min_m:
        raise ValueError("lidar range bounds are invalid")

    result: list[int] = []
    for index in BEAM_ORDER:
        distance_m = ranges_m[index]
        if (
            not math.isfinite(distance_m)
            or distance_m < range_min_m
            or distance_m > range_max_m
        ):
            result.append(-1)
        else:
            result.append(round(distance_m * 1000))
    return tuple(result)  # type: ignore[return-value]


def pairwise_ranges_m(
    positions: Mapping[str, Sequence[float]],
) -> tuple[tuple[str, str, float], ...]:
    """Compute ideal pairwise 3D distances from Gazebo odometry positions."""
    normalized: dict[str, tuple[float, float, float]] = {}
    for name, position in positions.items():
        if len(position) != 3:
            raise ValueError(f"position for {name} must have three coordinates")
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in position
        ):
            raise ValueError(f"position for {name} must be numeric")
        try:
            coordinates = tuple(float(value) for value in position)
        except OverflowError as error:
            raise ValueError(f"position for {name} must be finite") from error
        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError(f"position for {name} must be finite")
        normalized[name] = coordinates  # type: ignore[assignment]

    return tuple(
        (
            first,
            second,
            math.dist(normalized[first], normalized[second]),
        )
        for first, second in combinations(normalized, 2)
    )


def validate_odometry_timestamps(
    timestamps_s: Mapping[str, float],
    *,
    now_s: float,
    max_age_s: float = 0.5,
    max_skew_s: float = 0.1,
) -> None:
    """Reject missing, stale, or poorly synchronized swarm odometry samples."""
    if not timestamps_s:
        raise ValueError("no odometry timestamps are available")
    if (
        isinstance(now_s, bool)
        or not isinstance(now_s, (int, float))
        or not math.isfinite(now_s)
    ):
        raise ValueError("current simulation time must be finite")
    if not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError("maximum odometry age must be finite and positive")
    if not math.isfinite(max_skew_s) or max_skew_s <= 0:
        raise ValueError("maximum odometry skew must be finite and positive")

    if any(
        isinstance(stamp, bool) or not isinstance(stamp, (int, float))
        for stamp in timestamps_s.values()
    ):
        raise ValueError("odometry timestamps must be finite")
    try:
        stamps = tuple(float(stamp) for stamp in timestamps_s.values())
    except OverflowError as error:
        raise ValueError("odometry timestamps must be finite") from error
    if not all(math.isfinite(stamp) for stamp in stamps):
        raise ValueError("odometry timestamps must be finite")
    oldest_stamp_s = min(stamps)
    newest_stamp_s = max(stamps)
    if newest_stamp_s - now_s > 1e-6:
        raise OdometrySynchronizationError(
            "odometry timestamp is ahead of simulation time"
        )
    if now_s - oldest_stamp_s > max_age_s:
        raise OdometrySynchronizationError("odometry data is stale")
    if newest_stamp_s - oldest_stamp_s > max_skew_s:
        raise OdometrySynchronizationError("odometry samples are not synchronized")
