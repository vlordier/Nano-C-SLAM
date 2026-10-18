"""Conservative planned-route separation checks for the default Gazebo swarm."""

from __future__ import annotations

import math
from itertools import combinations, pairwise
from pathlib import Path
from xml.etree import ElementTree

from .mapping_path import DEFAULT_WAYPOINT_OFFSETS_M

MINIMUM_HORIZONTAL_SEPARATION_M = 0.5
DEFAULT_SPAWN_XY_M = {
    "cf1": (-0.5, -0.5),
    "cf3": (-0.5, 0.5),
    "cf4": (0.5, 0.5),
}

Point2D = tuple[float, float]
Segment2D = tuple[Point2D, Point2D]


def minimum_segment_distance(first: Segment2D, second: Segment2D) -> float:
    """Return the exact minimum Euclidean distance between two planar segments."""
    if _segments_intersect(first, second):
        return 0.0
    return min(
        _point_segment_distance(point, segment)
        for point, segment in (
            (first[0], second),
            (first[1], second),
            (second[0], first),
            (second[1], first),
        )
    )


def default_route_points(spawn_xy_m: Point2D) -> tuple[Point2D, ...]:
    """Project takeoff/landing anchors and the bounded path into the XY plane."""
    origin_x, origin_y = spawn_xy_m
    offsets = ((0.0, 0.0), *DEFAULT_WAYPOINT_OFFSETS_M)
    return tuple((origin_x + x, origin_y + y) for x, y in offsets)


def read_spawn_positions(sdf_path: Path) -> dict[str, Point2D]:
    """Read the configured swarm vehicle XY origins from the selected world SDF."""
    try:
        root = ElementTree.parse(sdf_path).getroot()  # noqa: S314 - trusted local installed SDF
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"could not read swarm world SDF: {sdf_path}") from error
    worlds = root.findall("world")
    if len(worlds) != 1:
        raise ValueError("swarm world SDF must contain exactly one top-level world")

    positions: dict[str, Point2D] = {}
    for model in worlds[0].findall("model"):
        name = model.get("name")
        if name not in DEFAULT_SPAWN_XY_M:
            continue
        pose = model.findtext("pose")
        try:
            coordinates = tuple(float(value) for value in (pose or "").split())
        except ValueError as error:
            raise ValueError(f"swarm model {name} has a malformed SDF pose") from error
        if len(coordinates) < 3 or not all(
            math.isfinite(value) for value in coordinates[:3]
        ):
            raise ValueError(f"swarm model {name} has a non-finite or incomplete pose")
        positions[name] = (coordinates[0], coordinates[1])

    if set(positions) != set(DEFAULT_SPAWN_XY_M):
        missing = sorted(set(DEFAULT_SPAWN_XY_M) - set(positions))
        names = ", ".join(missing)
        raise ValueError(f"swarm world SDF is missing model poses: {names}")
    return {name: positions[name] for name in DEFAULT_SPAWN_XY_M}


def minimum_swarm_route_separations_m(
    spawn_xy_m: dict[str, Point2D] | None = None,
) -> dict[str, float]:
    """Compute continuous route minima, allowing independent vehicle progress."""
    spawns = DEFAULT_SPAWN_XY_M if spawn_xy_m is None else spawn_xy_m
    if set(spawns) != set(DEFAULT_SPAWN_XY_M):
        raise ValueError("swarm spawn positions must define cf1, cf3, and cf4")
    routes = {
        name: _segments(default_route_points(spawn)) for name, spawn in spawns.items()
    }
    minima: dict[str, float] = {}
    for (first_name, first_route), (second_name, second_route) in combinations(
        routes.items(), 2
    ):
        minima[f"{first_name}/{second_name}"] = min(
            minimum_segment_distance(first_segment, second_segment)
            for first_segment in first_route
            for second_segment in second_route
        )
    return minima


def validate_default_swarm_routes(
    spawn_xy_m: dict[str, Point2D] | None = None,
) -> dict[str, float]:
    """Fail closed unless every planned route pair clears the nominal target."""
    minima = minimum_swarm_route_separations_m(spawn_xy_m)
    violations = {
        pair: distance
        for pair, distance in minima.items()
        if distance + 1e-9 < MINIMUM_HORIZONTAL_SEPARATION_M
    }
    if violations:
        details = ", ".join(
            f"{pair}={distance:.3f} m" for pair, distance in violations.items()
        )
        raise ValueError(
            "default swarm routes violate the 0.50 m horizontal separation "
            f"target: {details}"
        )
    return minima


def _segments(points: tuple[Point2D, ...]) -> tuple[Segment2D, ...]:
    return tuple(pairwise(points))


def _point_segment_distance(point: Point2D, segment: Segment2D) -> float:
    start, end = segment
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, start)
    projection = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared
    projection = min(1.0, max(0.0, projection))
    closest = (start[0] + projection * dx, start[1] + projection * dy)
    return math.dist(point, closest)


def _segments_intersect(first: Segment2D, second: Segment2D) -> bool:
    a, b = first
    c, d = second
    ab_c = _cross(a, b, c)
    ab_d = _cross(a, b, d)
    cd_a = _cross(c, d, a)
    cd_b = _cross(c, d, b)
    epsilon = 1e-12

    first_straddles = (ab_c > epsilon and ab_d < -epsilon) or (
        ab_c < -epsilon and ab_d > epsilon
    )
    second_straddles = (cd_a > epsilon and cd_b < -epsilon) or (
        cd_a < -epsilon and cd_b > epsilon
    )
    if first_straddles and second_straddles:
        return True
    collinear_endpoints = (
        (ab_c, c, a, b),
        (ab_d, d, a, b),
        (cd_a, a, c, d),
        (cd_b, b, c, d),
    )
    return any(
        abs(orientation) <= epsilon and _within_bounds(point, start, end, epsilon)
        for orientation, point, start, end in collinear_endpoints
    )


def _cross(start: Point2D, end: Point2D, point: Point2D) -> float:
    return (end[0] - start[0]) * (point[1] - start[1]) - (end[1] - start[1]) * (
        point[0] - start[0]
    )


def _within_bounds(
    point: Point2D, start: Point2D, end: Point2D, epsilon: float
) -> bool:
    return (
        min(start[0], end[0]) - epsilon <= point[0] <= max(start[0], end[0]) + epsilon
        and min(start[1], end[1]) - epsilon
        <= point[1]
        <= max(start[1], end[1]) + epsilon
    )
