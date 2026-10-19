"""Pure evidence gate for starting a simulated mapping mission on the ground."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

REQUIRED_GROUNDED_SAMPLES = 3
MAX_GROUNDED_ALTITUDE_M = 0.15
MAX_SAMPLE_AGE_S = 0.20
MAX_SAMPLE_GAP_S = 0.20
MAX_FUTURE_SKEW_S = 0.10


@dataclass(frozen=True)
class OdometrySample:
    """One odometry callback compared with the live ROS simulation clock."""

    stamp_s: float
    x_m: float | None
    y_m: float | None
    z_m: float
    sim_time_s: float
    received_monotonic_s: float

    @property
    def altitude_m(self) -> float:
        """The world-frame vertical coordinate used by the grounded gate."""
        return self.z_m


class GroundedOdometryPreflight:
    """Accept only fresh, ordered odometry proving the vehicle is grounded."""

    def __init__(self, *, require_position: bool = True) -> None:
        self.samples: list[OdometrySample] = []
        self.failure_reason: str | None = None
        self.require_position = require_position

    @property
    def passed(self) -> bool:
        return (
            len(self.samples) == REQUIRED_GROUNDED_SAMPLES
            and self.failure_reason is None
        )

    def observe(self, sample: OdometrySample) -> bool:
        """Add one sample, permanently failing closed on invalid evidence."""
        if self.failure_reason is not None or self.passed:
            return self.passed
        values = [
            sample.stamp_s,
            sample.z_m,
            sample.sim_time_s,
            sample.received_monotonic_s,
        ]
        if sample.x_m is not None:
            values.append(sample.x_m)
        if sample.y_m is not None:
            values.append(sample.y_m)
        if self.require_position and (sample.x_m is None or sample.y_m is None):
            self.failure_reason = "odometry preflight is missing x/y position"
            return False
        if not all(math.isfinite(value) for value in values):
            self.failure_reason = "odometry preflight received a non-finite value"
            return False
        if sample.stamp_s <= 0 or sample.sim_time_s <= 0:
            self.failure_reason = "simulation clock or odometry stamp has not started"
            return False
        age_s = sample.sim_time_s - sample.stamp_s
        if age_s < -MAX_FUTURE_SKEW_S or age_s > MAX_SAMPLE_AGE_S:
            self.failure_reason = f"odometry sample age is invalid ({age_s:.3f} s)"
            return False
        if sample.z_m > MAX_GROUNDED_ALTITUDE_M:
            self.failure_reason = (
                f"mapping run refused airborne start ({sample.z_m:.3f} m > "
                f"{MAX_GROUNDED_ALTITUDE_M:.3f} m)"
            )
            return False
        if self.samples:
            previous = self.samples[-1]
            stamp_gap = sample.stamp_s - previous.stamp_s
            receive_gap = sample.received_monotonic_s - previous.received_monotonic_s
            if stamp_gap <= 0 or receive_gap <= 0:
                self.failure_reason = (
                    "odometry preflight stamps or callbacks are reordered"
                )
                return False
            if stamp_gap > MAX_SAMPLE_GAP_S or receive_gap > MAX_SAMPLE_GAP_S:
                self.failure_reason = (
                    "odometry preflight samples are not contiguous and fresh"
                )
                return False
        self.samples.append(sample)
        return self.passed

    def to_report(self) -> dict[str, object]:
        """Serialize gate thresholds and the exact accepted sample evidence."""
        return {
            "schema_version": 2,
            "passed": self.passed,
            "failure_reason": self.failure_reason,
            "required_samples": REQUIRED_GROUNDED_SAMPLES,
            "max_grounded_altitude_m": MAX_GROUNDED_ALTITUDE_M,
            "max_sample_age_s": MAX_SAMPLE_AGE_S,
            "max_sample_gap_s": MAX_SAMPLE_GAP_S,
            "max_future_skew_s": MAX_FUTURE_SKEW_S,
            "samples": [
                {
                    "stamp_s": sample.stamp_s,
                    "x_m": sample.x_m,
                    "y_m": sample.y_m,
                    "z_m": sample.z_m,
                    "sim_time_s": sample.sim_time_s,
                    "received_monotonic_s": sample.received_monotonic_s,
                }
                for sample in self.samples
            ],
        }


def wait_for_grounded_preflight(
    gate: GroundedOdometryPreflight,
    pump_once: Callable[[], None],
    *,
    timeout_s: float,
    monotonic: Callable[[], float],
) -> dict[str, object]:
    """Pump callbacks until grounded evidence passes or wall time expires."""
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("odometry preflight timeout must be finite and positive")
    deadline = monotonic() + timeout_s
    while not gate.passed and monotonic() < deadline:
        if gate.failure_reason is not None:
            raise RuntimeError(gate.failure_reason)
        pump_once()
    if gate.failure_reason is not None:
        raise RuntimeError(gate.failure_reason)
    if not gate.passed:
        raise TimeoutError("odometry preflight timed out: no grounded samples arrived")
    return validate_odometry_preflight(gate.to_report())


def validate_odometry_preflight(value: object) -> dict[str, object]:
    """Validate successful preflight evidence in a persisted run bundle."""
    schema_version = value.get("schema_version") if isinstance(value, dict) else None
    if (
        not isinstance(value, dict)
        or isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version not in (1, 2)
    ):
        raise ValueError("odometry preflight report has unsupported schema")
    if value.get("passed") is not True or value.get("failure_reason") is not None:
        raise ValueError("odometry preflight did not pass")
    expected = {
        "required_samples": REQUIRED_GROUNDED_SAMPLES,
        "max_grounded_altitude_m": MAX_GROUNDED_ALTITUDE_M,
        "max_sample_age_s": MAX_SAMPLE_AGE_S,
        "max_sample_gap_s": MAX_SAMPLE_GAP_S,
        "max_future_skew_s": MAX_FUTURE_SKEW_S,
    }
    if any(
        isinstance(value.get(key), bool) or value.get(key) != threshold
        for key, threshold in expected.items()
    ):
        raise ValueError("odometry preflight thresholds differ from the owned gate")
    raw_samples = value.get("samples")
    if (
        not isinstance(raw_samples, list)
        or len(raw_samples) != REQUIRED_GROUNDED_SAMPLES
    ):
        raise ValueError("odometry preflight must contain exactly three samples")
    gate = GroundedOdometryPreflight(require_position=schema_version >= 2)
    for raw in raw_samples:
        if not isinstance(raw, dict):
            raise ValueError("odometry preflight sample is invalid")
        coordinate_keys = ("x_m", "y_m", "z_m") if schema_version >= 2 else ()
        raw_values = tuple(raw.get(key) for key in coordinate_keys) + tuple(
            raw.get(key)
            for key in (
                "stamp_s",
                "sim_time_s",
                "received_monotonic_s",
            )
        )
        if schema_version == 1:
            raw_values = (raw.get("altitude_m"), *raw_values)
        if any(
            isinstance(item, bool) or not isinstance(item, (int, float))
            for item in raw_values
        ):
            raise ValueError("odometry preflight sample contains non-numeric values")
        try:
            if schema_version == 1:
                sample = OdometrySample(
                    stamp_s=float(raw["stamp_s"]),
                    x_m=None,
                    y_m=None,
                    z_m=float(raw["altitude_m"]),
                    sim_time_s=float(raw["sim_time_s"]),
                    received_monotonic_s=float(raw["received_monotonic_s"]),
                )
            else:
                sample = OdometrySample(
                    stamp_s=float(raw["stamp_s"]),
                    x_m=float(raw["x_m"]),
                    y_m=float(raw["y_m"]),
                    z_m=float(raw["z_m"]),
                    sim_time_s=float(raw["sim_time_s"]),
                    received_monotonic_s=float(raw["received_monotonic_s"]),
                )
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError("odometry preflight sample is incomplete") from error
        gate.observe(sample)
        if gate.failure_reason is not None:
            raise ValueError(gate.failure_reason)
    if not gate.passed:
        raise ValueError("odometry preflight did not collect three valid samples")
    return value
