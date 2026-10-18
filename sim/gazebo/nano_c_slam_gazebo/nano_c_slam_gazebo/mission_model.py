"""Hardware-free flight-mission state transitions for the Gazebo demo."""

from __future__ import annotations

import json
import math
import uuid
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any

SUPPORTED_DRONE_NAMES = ("cf1", "cf3", "cf4")
MAX_ODOMETRY_FUTURE_SKEW_S = 0.1


class MissionPhase(Enum):
    WAITING = auto()
    TAKEOFF = auto()
    HOVER = auto()
    LAND = auto()
    COMPLETE = auto()
    FAILED = auto()


MISSION_STATUS_SCHEMA_VERSION = 2
LANDING_STABLE_DWELL_S = 0.5
LANDING_MAX_SAMPLE_GAP_S = 0.2


def create_run_id() -> str:
    """Create a canonical run identifier suitable for binding simulator evidence."""
    return str(uuid.uuid4())


def mission_exit_code(phase: MissionPhase, *, interrupted: bool = False) -> int:
    """Map actual terminal state to a process status, failing closed otherwise."""
    if interrupted:
        return 130
    return 0 if phase is MissionPhase.COMPLETE else 1


@dataclass(frozen=True)
class MissionStatusEvent:
    """One machine-readable mission state observation, independent of ROS."""

    run_id: str
    drone_name: str
    sequence: int
    sim_time_s: float
    phase: MissionPhase
    altitude_m: float | None
    mapping_path_requested: bool
    mapping_path_complete: bool
    landing_height_m: float
    odometry_stamp_s: float | None = None
    landing_stable_since_s: float | None = None
    failure_reason: str | None = None
    schema_version: int = MISSION_STATUS_SCHEMA_VERSION

    def __post_init__(self) -> None:
        _validate_run_id(self.run_id, "run_id")
        validate_drone_name(self.drone_name)
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or self.sequence < 0
        ):
            raise ValueError("sequence must be a nonnegative integer")
        _require_finite(self.sim_time_s, "sim_time_s")
        _require_finite(self.landing_height_m, "landing_height_m")
        if self.landing_height_m < 0:
            raise ValueError("landing_height_m must be nonnegative")
        if self.altitude_m is not None:
            _require_finite(self.altitude_m, "altitude_m")
        if self.landing_stable_since_s is not None:
            _require_finite(self.landing_stable_since_s, "landing_stable_since_s")
        if self.odometry_stamp_s is not None:
            _require_finite(self.odometry_stamp_s, "odometry_stamp_s")
        if (
            isinstance(self.schema_version, bool)
            or not isinstance(self.schema_version, int)
            or self.schema_version != MISSION_STATUS_SCHEMA_VERSION
        ):
            raise ValueError("unsupported mission status schema_version")
        if not isinstance(self.phase, MissionPhase):
            raise ValueError("phase must be a MissionPhase")
        if not isinstance(self.mapping_path_requested, bool) or not isinstance(
            self.mapping_path_complete, bool
        ):
            raise ValueError("mapping path status fields must be booleans")
        if self.mapping_path_complete and not self.mapping_path_requested:
            raise ValueError("unrequested mapping path cannot be complete")
        if self.failure_reason is not None and not isinstance(self.failure_reason, str):
            raise ValueError("failure_reason must be text or null")
        if self.phase is MissionPhase.FAILED and not self.failure_reason:
            raise ValueError("FAILED status requires failure_reason")
        if self.phase is not MissionPhase.FAILED and self.failure_reason is not None:
            raise ValueError("failure_reason is only valid for FAILED status")
        if self.phase is MissionPhase.COMPLETE:
            self.validate_success()

    def validate_success(self) -> None:
        """Reject terminal success unless measured mission evidence supports it."""
        if self.phase is not MissionPhase.COMPLETE:
            raise ValueError("mission did not reach COMPLETE")
        if self.mapping_path_requested and not self.mapping_path_complete:
            raise ValueError("requested mapping path did not complete")
        if self.altitude_m is None:
            raise ValueError("terminal success requires measured landing altitude")
        if self.altitude_m > self.landing_height_m:
            raise ValueError("landing altitude exceeds configured limit")
        if self.landing_stable_since_s is None:
            raise ValueError("terminal success requires stable-altitude start time")
        if self.landing_stable_since_s > self.sim_time_s:
            raise ValueError("stable-altitude start time is after terminal status")
        stable_duration_s = self.sim_time_s - self.landing_stable_since_s
        if stable_duration_s + 1e-9 < LANDING_STABLE_DWELL_S:
            raise ValueError("terminal success lacks required stable-altitude dwell")

    def to_json(self) -> str:
        """Serialize with nonstandard NaN/Infinity JSON values forbidden."""
        payload = {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "drone_name": self.drone_name,
            "sequence": self.sequence,
            "sim_time_s": self.sim_time_s,
            "phase": self.phase.name,
            "altitude_m": self.altitude_m,
            "mapping_path_requested": self.mapping_path_requested,
            "mapping_path_complete": self.mapping_path_complete,
            "landing_height_m": self.landing_height_m,
            "odometry_stamp_s": self.odometry_stamp_s,
            "landing_stable_since_s": self.landing_stable_since_s,
            "failure_reason": self.failure_reason,
        }
        return json.dumps(payload, allow_nan=False, sort_keys=True)

    @classmethod
    def from_json(cls, payload: str) -> MissionStatusEvent:
        """Parse and strictly validate one serialized status event."""
        try:
            data: Any = json.loads(payload, object_pairs_hook=_unique_json_object)
        except (json.JSONDecodeError, TypeError) as error:
            raise ValueError("mission status is not valid JSON") from error
        expected = {
            "schema_version",
            "run_id",
            "drone_name",
            "sequence",
            "sim_time_s",
            "phase",
            "altitude_m",
            "mapping_path_requested",
            "mapping_path_complete",
            "landing_height_m",
            "odometry_stamp_s",
            "landing_stable_since_s",
            "failure_reason",
        }
        if not isinstance(data, dict) or set(data) != expected:
            raise ValueError("mission status fields do not match schema")
        if not isinstance(data["mapping_path_requested"], bool) or not isinstance(
            data["mapping_path_complete"], bool
        ):
            raise ValueError("mapping path status fields must be booleans")
        try:
            phase = MissionPhase[data["phase"]]
        except (KeyError, TypeError) as error:
            raise ValueError("mission status phase is invalid") from error
        return cls(
            schema_version=data["schema_version"],
            run_id=data["run_id"],
            drone_name=data["drone_name"],
            sequence=data["sequence"],
            sim_time_s=data["sim_time_s"],
            phase=phase,
            altitude_m=data["altitude_m"],
            mapping_path_requested=data["mapping_path_requested"],
            mapping_path_complete=data["mapping_path_complete"],
            landing_height_m=data["landing_height_m"],
            odometry_stamp_s=data["odometry_stamp_s"],
            landing_stable_since_s=data["landing_stable_since_s"],
            failure_reason=data["failure_reason"],
        )


def validate_mission_status_events(
    events: list[MissionStatusEvent],
    expected_run_id: str,
    expected_drone_name: str,
    *,
    require_mapping_path: bool = False,
) -> MissionStatusEvent:
    """Validate a complete, ordered event stream and return terminal success."""
    if not events:
        raise ValueError("mission status stream is empty")
    _validate_run_id(expected_run_id, "expected_run_id")
    validate_drone_name(expected_drone_name)
    previous_time = -math.inf
    previous_phase: MissionPhase | None = None
    landing_stable_since_s: float | None = None
    previous_odometry_stamp_s: float | None = None
    path_requested: bool | None = None
    path_complete = False
    allowed_transitions = {
        MissionPhase.WAITING: {
            MissionPhase.TAKEOFF,
            MissionPhase.HOVER,
            MissionPhase.FAILED,
        },
        MissionPhase.TAKEOFF: {MissionPhase.HOVER, MissionPhase.FAILED},
        MissionPhase.HOVER: {MissionPhase.LAND, MissionPhase.FAILED},
        MissionPhase.LAND: {
            MissionPhase.LAND,
            MissionPhase.COMPLETE,
            MissionPhase.FAILED,
        },
    }
    for index, event in enumerate(events):
        if event.run_id != expected_run_id:
            raise ValueError("mission status run_id does not match capture")
        if event.drone_name != expected_drone_name:
            raise ValueError("mission status drone_name does not match capture")
        if event.sequence != index:
            raise ValueError(
                "mission status sequence is missing, duplicate, or reordered"
            )
        if event.sim_time_s < previous_time:
            raise ValueError("mission status simulation time moved backwards")
        if index == 0 and event.phase is not MissionPhase.WAITING:
            raise ValueError("mission status stream must start at WAITING")
        if previous_phase is not None and event.phase not in allowed_transitions.get(
            previous_phase, set()
        ):
            raise ValueError("mission status phase transition is invalid")
        previous_phase = event.phase
        if event.phase is MissionPhase.LAND:
            if (
                event.altitude_m is None
                or event.altitude_m > event.landing_height_m
                or event.odometry_stamp_s is None
            ):
                landing_stable_since_s = None
            elif landing_stable_since_s is None or (
                previous_odometry_stamp_s is not None
                and event.odometry_stamp_s - previous_odometry_stamp_s
                > LANDING_MAX_SAMPLE_GAP_S + 1e-9
            ):
                landing_stable_since_s = event.odometry_stamp_s
            if event.landing_stable_since_s != landing_stable_since_s:
                raise ValueError("LAND event stable-altitude evidence does not match")
            if event.odometry_stamp_s is not None:
                previous_odometry_stamp_s = event.odometry_stamp_s
        elif event.phase is MissionPhase.COMPLETE:
            if landing_stable_since_s is None:
                raise ValueError("terminal status is missing stable-altitude samples")
            if (
                event.odometry_stamp_s is None
                or previous_odometry_stamp_s is None
                or event.odometry_stamp_s - previous_odometry_stamp_s
                > LANDING_MAX_SAMPLE_GAP_S + 1e-9
            ):
                raise ValueError("terminal status is missing recent altitude evidence")
            if landing_stable_since_s != event.landing_stable_since_s:
                raise ValueError(
                    "terminal stable-altitude evidence does not match stream"
                )
            if (
                previous_odometry_stamp_s is None
                or event.odometry_stamp_s - previous_odometry_stamp_s
                > LANDING_MAX_SAMPLE_GAP_S + 1e-9
            ):
                raise ValueError("terminal status is missing recent altitude evidence")
        previous_time = event.sim_time_s
        if path_requested is None:
            path_requested = event.mapping_path_requested
        elif event.mapping_path_requested != path_requested:
            raise ValueError("mapping path request changed during mission")
        if path_complete and not event.mapping_path_complete:
            raise ValueError("mapping path completion regressed")
        path_complete = event.mapping_path_complete
        if (
            event.phase in (MissionPhase.COMPLETE, MissionPhase.FAILED)
            and index != len(events) - 1
        ):
            raise ValueError("terminal mission event must be the final event")
    terminal = events[-1]
    if terminal.phase is MissionPhase.FAILED:
        raise ValueError(f"mission failed: {terminal.failure_reason}")
    terminal.validate_success()
    if require_mapping_path and not terminal.mapping_path_requested:
        raise ValueError("mapping run completed without requesting a bounded path")
    return terminal


def _require_finite(value: object, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be finite numeric data")
    try:
        finite = math.isfinite(value)
    except OverflowError as error:
        raise ValueError(f"{field_name} must be finite numeric data") from error
    if not finite:
        raise ValueError(f"{field_name} must be finite numeric data")


def _validate_run_id(value: object, field_name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field_name} must be a canonical UUID")
    try:
        canonical = str(uuid.UUID(value))
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError(f"{field_name} must be a canonical UUID") from error
    if canonical != value:
        raise ValueError(f"{field_name} must be a canonical UUID")
    return value


def validate_run_id(value: object, field_name: str = "run_id") -> str:
    """Validate and return a canonical UUID for evidence identity fields."""
    return _validate_run_id(value, field_name)


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"mission status contains duplicate field {key!r}")
        result[key] = value
    return result


@dataclass(frozen=True)
class MissionConfig:
    hover_height_m: float = 0.5
    # Use an altitude threshold without claiming simulator contact or touchdown.
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


def validate_drone_name(value: object) -> str:
    """Accept only Crazyflie names present in the Gazebo world."""
    if not isinstance(value, str) or value not in SUPPORTED_DRONE_NAMES:
        choices = ", ".join(SUPPORTED_DRONE_NAMES)
        raise ValueError(f"drone_name must be one of: {choices}")
    return value


def validate_drone_names(value: object) -> tuple[str, ...]:
    """Validate the sensor bridge's configured set of world drones."""
    if not isinstance(value, (list, tuple)) or not value:
        raise ValueError("drone_names must be a nonempty list of Crazyflie names")
    names = tuple(validate_drone_name(name) for name in value)
    if len(names) != len(set(names)):
        raise ValueError("drone_names must not contain duplicates")
    return names


def create_mission_config(
    hover_height_m: object, hover_duration_s: object
) -> MissionConfig:
    """Validate untrusted ROS parameter values and build mission settings."""
    if isinstance(hover_height_m, bool) or not isinstance(hover_height_m, (int, float)):
        raise ValueError("mission height and duration parameters must be numeric")
    if isinstance(hover_duration_s, bool) or not isinstance(
        hover_duration_s, (int, float)
    ):
        raise ValueError("mission height and duration parameters must be numeric")
    return MissionConfig(
        hover_height_m=float(hover_height_m),
        hover_duration_s=float(hover_duration_s),
    )


def create_mapping_start_altitude_limit(value: object) -> float:
    """Validate the maximum initial altitude for a spatial mapping mission."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("mapping_start_max_altitude_m must be numeric")
    altitude_m = float(value)
    if not math.isfinite(altitude_m) or altitude_m < 0:
        raise ValueError("mapping_start_max_altitude_m must be finite and nonnegative")
    return altitude_m


def validate_mission_odometry(
    position: tuple[float, float, float],
    stamp_s: float,
    *,
    now_s: float,
    previous_stamp_s: float | None,
    max_age_s: float = 0.5,
    max_future_skew_s: float = MAX_ODOMETRY_FUTURE_SKEW_S,
) -> None:
    """Reject malformed or temporally invalid odometry before mission readiness.

    ROS time is used for all timestamp comparisons so paused simulation time does
    not make otherwise valid samples appear stale. Equal timestamps are allowed
    because some bridges publish multiple state updates per simulation tick.
    """
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        raise ValueError("odometry position is not finite")
    if not math.isfinite(now_s) or not math.isfinite(stamp_s):
        raise ValueError("odometry timestamp is not finite")
    if previous_stamp_s is not None and stamp_s < previous_stamp_s:
        raise ValueError("odometry timestamp moved backwards")
    if stamp_s - now_s > max_future_skew_s + 1e-9:
        raise ValueError("odometry timestamp is ahead of simulation time")
    if now_s - stamp_s > max_age_s:
        raise ValueError("odometry data is stale")


class MissionSequence:
    """Convert measured altitude and simulation time into vertical commands."""

    def __init__(self, config: MissionConfig | None = None) -> None:
        self.config = config or MissionConfig()
        self.phase = MissionPhase.WAITING
        self._hover_started_s: float | None = None
        self._phase_started_s: float | None = None
        self._last_now_s: float | None = None
        self._last_odometry_stamp_s: float | None = None
        self._landing_stable_since_s: float | None = None
        self.failure_reason: str | None = None

    def step(
        self,
        now_s: float,
        altitude_m: float | None,
        odometry_stamp_s: float | None = None,
        *,
        hold_hover: bool = False,
    ) -> float:
        if self.phase in (MissionPhase.COMPLETE, MissionPhase.FAILED):
            return 0.0
        if not math.isfinite(now_s):
            return self.fail("simulation time is not finite")
        if self._last_now_s is not None and now_s < self._last_now_s:
            return self.fail("simulation time moved backwards")
        self._last_now_s = now_s
        previous_odometry_stamp_s = self._last_odometry_stamp_s
        odometry_delta_s = (
            None
            if previous_odometry_stamp_s is None or odometry_stamp_s is None
            else odometry_stamp_s - previous_odometry_stamp_s
        )
        if altitude_m is not None and not math.isfinite(altitude_m):
            return self.fail("odometry altitude is not finite")
        if odometry_stamp_s is not None:
            if not math.isfinite(odometry_stamp_s):
                return self.fail("odometry timestamp is not finite")
            if (
                self._last_odometry_stamp_s is not None
                and odometry_stamp_s < self._last_odometry_stamp_s
            ):
                return self.fail("odometry timestamp moved backwards")
            self._last_odometry_stamp_s = odometry_stamp_s
        if self.phase is MissionPhase.WAITING and altitude_m is None:
            return 0.0
        if altitude_m is None:
            return self.fail("odometry altitude is unavailable")
        if odometry_stamp_s is None:
            return self.fail("odometry timestamp is unavailable")
        if odometry_stamp_s - now_s > MAX_ODOMETRY_FUTURE_SKEW_S + 1e-9:
            return self.fail("odometry timestamp is ahead of simulation time")
        if now_s - odometry_stamp_s > self.config.odometry_timeout_s:
            return self.fail("odometry data is stale")
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
                return self.fail("takeoff phase timed out")
            return self.config.vertical_speed_m_s

        if self.phase is MissionPhase.HOVER:
            if hold_hover:
                self._hover_started_s = now_s
                return 0.0
            hover_started_s = self._hover_started_s
            if hover_started_s is None:
                raise RuntimeError("hover phase started without an entry timestamp")
            if now_s - hover_started_s >= self.config.hover_duration_s:
                self.phase = MissionPhase.LAND
                self._phase_started_s = now_s
                return -self.config.vertical_speed_m_s
            else:
                return 0.0

        if self.phase is MissionPhase.LAND:
            if (
                self._landing_stable_since_s is not None
                and odometry_stamp_s is not None
                and previous_odometry_stamp_s is not None
                and odometry_delta_s is not None
                and odometry_delta_s > LANDING_MAX_SAMPLE_GAP_S + 1e-9
            ):
                self._landing_stable_since_s = None
            if altitude_m <= self.config.landing_height_m:
                if self._landing_stable_since_s is None or (
                    odometry_delta_s is not None
                    and odometry_delta_s > LANDING_MAX_SAMPLE_GAP_S + 1e-9
                ):
                    self._landing_stable_since_s = odometry_stamp_s
                if (
                    odometry_stamp_s - self._landing_stable_since_s
                    >= LANDING_STABLE_DWELL_S - 1e-9
                ):
                    self.phase = MissionPhase.COMPLETE
                    return 0.0
            else:
                self._landing_stable_since_s = None
            if self._timed_out(now_s):
                return self.fail("landing phase timed out")
            return -self.config.vertical_speed_m_s

        return 0.0

    def fail(self, reason: str) -> float:
        """Stop the mission safely and retain a diagnostic failure reason."""
        self.phase = MissionPhase.FAILED
        self.failure_reason = reason
        self._landing_stable_since_s = None
        return 0.0

    def _timed_out(self, now_s: float) -> bool:
        phase_started_s = self._phase_started_s
        return (
            phase_started_s is not None
            and now_s - phase_started_s >= self.config.phase_timeout_s
        )

    @property
    def landing_stable_since_s(self) -> float | None:
        """Odometry time when the current uninterrupted low-altitude dwell began."""
        return self._landing_stable_since_s
