"""Hardware-free flight-mission state transitions for the Gazebo demo."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto


class MissionPhase(Enum):
    WAITING = auto()
    TAKEOFF = auto()
    HOVER = auto()
    LAND = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass(frozen=True)
class MissionConfig:
    hover_height_m: float = 0.5
    # Treat 10 cm above ground as landed to avoid settling/contact oscillation.
    landing_height_m: float = 0.1
    hover_duration_s: float = 3.0
    vertical_speed_m_s: float = 0.5
    phase_timeout_s: float = 15.0
    odometry_timeout_s: float = 0.5

    def __post_init__(self) -> None:
        if not math.isfinite(self.hover_height_m) or self.hover_height_m <= 0:
            raise ValueError("hover_height_m must be finite and positive")
        if not math.isfinite(self.landing_height_m) or self.landing_height_m < 0:
            raise ValueError("landing_height_m must be finite and nonnegative")
        if self.landing_height_m >= self.hover_height_m:
            raise ValueError("landing_height_m must be below hover_height_m")
        if not math.isfinite(self.hover_duration_s) or self.hover_duration_s < 0:
            raise ValueError("hover_duration_s must be finite and nonnegative")
        if not math.isfinite(self.vertical_speed_m_s) or self.vertical_speed_m_s <= 0:
            raise ValueError("vertical_speed_m_s must be finite and positive")
        if not math.isfinite(self.phase_timeout_s) or self.phase_timeout_s <= 0:
            raise ValueError("phase_timeout_s must be finite and positive")
        if not math.isfinite(self.odometry_timeout_s) or self.odometry_timeout_s <= 0:
            raise ValueError("odometry_timeout_s must be finite and positive")


class MissionSequence:
    """Convert measured altitude and simulation time into vertical commands."""

    def __init__(self, config: MissionConfig | None = None) -> None:
        self.config = config or MissionConfig()
        self.phase = MissionPhase.WAITING
        self._hover_started_s: float | None = None
        self._phase_started_s: float | None = None
        self._last_now_s: float | None = None
        self._last_odometry_stamp_s: float | None = None
        self.failure_reason: str | None = None

    def step(
        self,
        now_s: float,
        altitude_m: float | None,
        odometry_stamp_s: float | None = None,
    ) -> float:
        if self.phase in (MissionPhase.COMPLETE, MissionPhase.FAILED):
            return 0.0
        if not math.isfinite(now_s):
            return self._fail("simulation time is not finite")
        if self._last_now_s is not None and now_s < self._last_now_s:
            return self._fail("simulation time moved backwards")
        self._last_now_s = now_s
        if altitude_m is not None and not math.isfinite(altitude_m):
            return self._fail("odometry altitude is not finite")
        if odometry_stamp_s is not None:
            if not math.isfinite(odometry_stamp_s):
                return self._fail("odometry timestamp is not finite")
            if (
                self._last_odometry_stamp_s is not None
                and odometry_stamp_s < self._last_odometry_stamp_s
            ):
                return self._fail("odometry timestamp moved backwards")
            self._last_odometry_stamp_s = odometry_stamp_s
        if self.phase is MissionPhase.WAITING and altitude_m is None:
            return 0.0
        if altitude_m is None:
            return self._fail("odometry altitude is unavailable")
        if odometry_stamp_s is None:
            return self._fail("odometry timestamp is unavailable")
        if now_s - odometry_stamp_s > self.config.odometry_timeout_s:
            return self._fail("odometry data is stale")
        if self.phase is MissionPhase.WAITING:
            self.phase = MissionPhase.TAKEOFF
            self._phase_started_s = now_s

        if self.phase is MissionPhase.TAKEOFF:
            if altitude_m is not None and altitude_m >= self.config.hover_height_m:
                self.phase = MissionPhase.HOVER
                self._hover_started_s = now_s
                self._phase_started_s = now_s
                return 0.0
            if self._timed_out(now_s):
                return self._fail("takeoff phase timed out")
            return self.config.vertical_speed_m_s

        if self.phase is MissionPhase.HOVER:
            hover_started_s = self._hover_started_s
            if hover_started_s is None:
                raise RuntimeError("hover phase started without an entry timestamp")
            if now_s - hover_started_s >= self.config.hover_duration_s:
                self.phase = MissionPhase.LAND
                self._phase_started_s = now_s
            else:
                return 0.0

        if self.phase is MissionPhase.LAND:
            if altitude_m is not None and altitude_m <= self.config.landing_height_m:
                self.phase = MissionPhase.COMPLETE
                return 0.0
            if self._timed_out(now_s):
                return self._fail("landing phase timed out")
            return -self.config.vertical_speed_m_s

        return 0.0

    def _fail(self, reason: str) -> float:
        self.phase = MissionPhase.FAILED
        self.failure_reason = reason
        return 0.0

    def _timed_out(self, now_s: float) -> bool:
        phase_started_s = self._phase_started_s
        return (
            phase_started_s is not None
            and now_s - phase_started_s >= self.config.phase_timeout_s
        )
