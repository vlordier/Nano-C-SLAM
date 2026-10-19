"""Bounded world-frame waypoint path for exercising mapping in Gazebo SITL."""

from __future__ import annotations

import math

DEFAULT_WAYPOINT_OFFSETS_M = (
    (0.25, 0.0),
    (0.25, 0.25),
    (0.0, 0.25),
    (0.0, 0.0),
)
HELDOUT_ASYMMETRIC_WAYPOINT_OFFSETS_M = (
    (0.18, 0.02),
    (0.32, 0.22),
    (0.05, 0.38),
    (-0.18, 0.12),
    (0.0, 0.0),
)


def waypoint_offsets_for_route(route: str) -> tuple[tuple[float, float], ...]:
    """Return one predeclared path profile for deterministic sim validation."""
    if route == "square":
        return DEFAULT_WAYPOINT_OFFSETS_M
    if route == "heldout-asymmetric":
        return HELDOUT_ASYMMETRIC_WAYPOINT_OFFSETS_M
    raise ValueError("mapping route must be square or heldout-asymmetric")


class MappingPath:
    """Track a small square path with bounded velocity commands."""

    def __init__(
        self,
        origin_x_m: float,
        origin_y_m: float,
        start_time_s: float,
        *,
        offsets_m: tuple[tuple[float, float], ...] = DEFAULT_WAYPOINT_OFFSETS_M,
        max_speed_m_s: float = 0.25,
        tolerance_m: float = 0.08,
        waypoint_timeout_s: float = 8.0,
    ) -> None:
        values = (
            origin_x_m,
            origin_y_m,
            start_time_s,
            max_speed_m_s,
            tolerance_m,
            waypoint_timeout_s,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("mapping path configuration must be finite")
        if max_speed_m_s <= 0 or tolerance_m <= 0 or waypoint_timeout_s <= 0:
            raise ValueError("mapping path limits must be positive")
        if not isinstance(offsets_m, tuple) or not offsets_m:
            raise ValueError("mapping path needs at least one waypoint")
        if any(
            not isinstance(offset, tuple)
            or len(offset) != 2
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                for value in offset
            )
            for offset in offsets_m
        ):
            raise ValueError("mapping path waypoints must be finite 2D offsets")
        if any(any(abs(value) > 0.5 for value in offset) for offset in offsets_m):
            raise ValueError("mapping path waypoints must stay within 0.5 m of origin")

        self._origin = (origin_x_m, origin_y_m)
        self._offsets = offsets_m
        self._max_speed_m_s = max_speed_m_s
        self._tolerance_m = tolerance_m
        self._waypoint_timeout_s = waypoint_timeout_s
        self._waypoint_started_s = start_time_s
        self._last_now_s = start_time_s
        self._waypoint_index = 0
        self.failure_reason: str | None = None

    @property
    def complete(self) -> bool:
        return self._waypoint_index >= len(self._offsets)

    @property
    def waypoint_index(self) -> int:
        return self._waypoint_index

    def step(self, now_s: float, x_m: float, y_m: float) -> tuple[float, float]:
        """Return bounded world-frame x/y velocity, or zero after completion."""
        if self.complete or self.failure_reason is not None:
            return (0.0, 0.0)
        if any(not math.isfinite(value) for value in (now_s, x_m, y_m)):
            self.failure_reason = "mapping path received non-finite state"
            return (0.0, 0.0)
        if now_s < self._last_now_s:
            self.failure_reason = "mapping path time moved backwards"
            return (0.0, 0.0)
        self._last_now_s = now_s

        target_offset = self._offsets[self._waypoint_index]
        target_x = self._origin[0] + target_offset[0]
        target_y = self._origin[1] + target_offset[1]
        delta_x = target_x - x_m
        delta_y = target_y - y_m
        distance = math.hypot(delta_x, delta_y)
        if distance <= self._tolerance_m:
            self._waypoint_index += 1
            self._waypoint_started_s = now_s
            return (0.0, 0.0)
        if now_s - self._waypoint_started_s >= self._waypoint_timeout_s:
            self.failure_reason = (
                f"mapping waypoint {self._waypoint_index + 1} timed out"
            )
            return (0.0, 0.0)

        speed = min(self._max_speed_m_s, 0.8 * distance)
        return (speed * delta_x / distance, speed * delta_y / distance)
