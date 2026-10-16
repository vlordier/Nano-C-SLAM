"""Pure conversions for the simplified Crazyswarm2 Gazebo sensors."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from itertools import combinations

BEAM_ORDER = (2, 0, 3, 1)  # front, back, left, right for the upstream SDF scan


def laser_scan_ranges_mm(
    ranges_m: Sequence[float], *, range_min_m: float, range_max_m: float
) -> tuple[int, int, int, int]:
    """Convert the upstream four-beam scan to front/back/left/right millimeters.

    The Crazyswarm2 Gazebo model has four horizontal rays. Invalid or out-of-range
    readings become -1, matching the STM32 exploration code's invalid sentinel.
    This does not model the real 8x8 VL53L5CX depth matrices.
    """
    if len(ranges_m) != len(BEAM_ORDER):
        raise ValueError(f"expected {len(BEAM_ORDER)} lidar beams, got {len(ranges_m)}")
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
        coordinates = tuple(float(value) for value in position)
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
