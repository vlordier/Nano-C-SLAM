"""Pure, bounded timestamp pairing for simulated mapping telemetry."""

from __future__ import annotations

import math
from collections import Counter, deque
from dataclasses import dataclass

RANGE_COUNT = 4
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1
UINT64_MAX = 2**64 - 1


@dataclass(frozen=True)
class StampedRangeSample:
    """One four-ray range message with its Gazebo source timestamp."""

    drone_id: str
    stamp_ns: int
    sequence: int
    ranges_mm: tuple[int, int, int, int]

    def __post_init__(self) -> None:
        if not self.drone_id:
            raise ValueError("range sample drone_id must not be empty")
        if isinstance(self.stamp_ns, bool) or not isinstance(self.stamp_ns, int):
            raise ValueError("range sample timestamp must be an integer")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise ValueError("range sample sequence must be an integer")
        if not 0 <= self.sequence <= UINT64_MAX:
            raise ValueError("range sample sequence must fit uint64")
        if len(self.ranges_mm) != RANGE_COUNT:
            raise ValueError("range sample must contain exactly four readings")
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or not INT32_MIN <= value <= INT32_MAX
            for value in self.ranges_mm
        ):
            raise ValueError("range readings must fit int32")
        if any(value < -1 or value > 32767 for value in self.ranges_mm):
            raise ValueError("range readings must fit the Nano-C-SLAM int16 format")


@dataclass(frozen=True)
class StampedOdometrySample:
    """The planar pose needed by the mapping replay, with source time."""

    drone_id: str
    stamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        if not self.drone_id:
            raise ValueError("odometry sample drone_id must not be empty")
        if isinstance(self.stamp_ns, bool) or not isinstance(self.stamp_ns, int):
            raise ValueError("odometry timestamp must be an integer")
        if not all(
            math.isfinite(value) for value in (self.x_m, self.y_m, self.yaw_rad)
        ):
            raise ValueError("odometry pose must be finite")


@dataclass(frozen=True)
class PairedTelemetry:
    """A one-use same-drone range/odometry pair and its measured skew."""

    ranges: StampedRangeSample
    odometry: StampedOdometrySample
    skew_ns: int


@dataclass(frozen=True)
class PairingLimits:
    """Source-time bounds; simulation time never mixes with host wall time."""

    max_age_ns: int = 500_000_000
    max_skew_ns: int = 100_000_000
    buffer_size: int = 32

    def __post_init__(self) -> None:
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in (self.max_age_ns, self.max_skew_ns, self.buffer_size)
        ):
            raise ValueError("telemetry pairing limits must be integers")
        if self.max_age_ns <= 0:
            raise ValueError("maximum telemetry age must be positive")
        if self.max_skew_ns <= 0:
            raise ValueError("maximum telemetry skew must be positive")
        if self.buffer_size <= 0:
            raise ValueError("telemetry buffer size must be positive")


class TelemetryPairer:
    """Pair fresh samples once, using bounded queues and explicit rejections.

    The caller supplies the current ROS simulation-clock timestamp to
    :meth:`pop_pair`. Host-side timeout handling belongs to the caller and
    should use a monotonic clock.
    """

    def __init__(self, drone_id: str, limits: PairingLimits | None = None) -> None:
        if not drone_id:
            raise ValueError("pairer drone_id must not be empty")
        self._drone_id = drone_id
        self._limits = limits if limits is not None else PairingLimits()
        self._ranges: deque[StampedRangeSample] = deque(maxlen=self._limits.buffer_size)
        self._odometry: deque[StampedOdometrySample] = deque(
            maxlen=self._limits.buffer_size
        )
        self._last_range_stamp_ns: int | None = None
        self._last_range_sequence: int | None = None
        self._last_odometry_stamp_ns: int | None = None
        self._last_now_ns: int | None = None
        self.rejections: Counter[str] = Counter()

    @property
    def buffered_sample_count(self) -> int:
        """Return the number of queued range and odometry samples."""
        return len(self._ranges) + len(self._odometry)

    def add_range(self, sample: StampedRangeSample) -> bool:
        """Queue a valid, increasing range sample or record its rejection."""
        if sample.drone_id != self._drone_id:
            self.rejections["wrong_drone"] += 1
            return False
        if sample.stamp_ns <= 0:
            self.rejections["uninitialized_stamp"] += 1
            return False
        if self._last_range_sequence is not None:
            if sample.sequence == self._last_range_sequence:
                self.rejections["duplicate_range_sequence"] += 1
                return False
            if sample.sequence < self._last_range_sequence:
                self.rejections["out_of_order_range_sequence"] += 1
                return False
        if self._last_range_stamp_ns is not None:
            if sample.stamp_ns == self._last_range_stamp_ns:
                self.rejections["duplicate_range_stamp"] += 1
                return False
            if sample.stamp_ns < self._last_range_stamp_ns:
                self.rejections["out_of_order_range_stamp"] += 1
                return False
        if len(self._ranges) == self._limits.buffer_size:
            self.rejections["range_buffer_overflow"] += 1
        self._ranges.append(sample)
        self._last_range_stamp_ns = sample.stamp_ns
        self._last_range_sequence = sample.sequence
        return True

    def add_odometry(self, sample: StampedOdometrySample) -> bool:
        """Queue a valid, increasing odometry sample or record its rejection."""
        if sample.drone_id != self._drone_id:
            self.rejections["wrong_drone"] += 1
            return False
        if sample.stamp_ns <= 0:
            self.rejections["uninitialized_stamp"] += 1
            return False
        if self._last_odometry_stamp_ns is not None:
            if sample.stamp_ns == self._last_odometry_stamp_ns:
                self.rejections["duplicate_odometry_stamp"] += 1
                return False
            if sample.stamp_ns < self._last_odometry_stamp_ns:
                self.rejections["out_of_order_odometry_stamp"] += 1
                return False
        if len(self._odometry) == self._limits.buffer_size:
            self.rejections["odometry_buffer_overflow"] += 1
        self._odometry.append(sample)
        self._last_odometry_stamp_ns = sample.stamp_ns
        return True

    def pop_pair(self, now_ns: int) -> PairedTelemetry | None:
        """Return the closest valid pair, consuming each sample at most once."""
        if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns <= 0:
            self.rejections["uninitialized_clock"] += 1
            return None
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            self._ranges.clear()
            self._odometry.clear()
            self._last_range_stamp_ns = None
            self._last_odometry_stamp_ns = None
            # Sequence is generated by the still-running bridge, so preserve it
            # across a simulator clock reset and only reset its source-time fence.
            self.rejections["clock_reset"] += 1
        self._last_now_ns = now_ns

        valid_ranges = self._fresh_ranges(now_ns)
        valid_odometry = self._fresh_odometry(now_ns)
        candidates = [
            (abs(ranges.stamp_ns - odometry.stamp_ns), ranges, odometry)
            for ranges in valid_ranges
            for odometry in valid_odometry
            if abs(ranges.stamp_ns - odometry.stamp_ns) <= self._limits.max_skew_ns
        ]
        if not candidates:
            self._reject_unpairable_skew()
            return None

        skew_ns, ranges, odometry = min(
            candidates,
            key=lambda candidate: (
                candidate[0],
                -candidate[1].stamp_ns,
                -candidate[2].stamp_ns,
            ),
        )
        self._consume_through(self._ranges, ranges, "range")
        self._consume_through(self._odometry, odometry, "odometry")
        return PairedTelemetry(ranges, odometry, skew_ns)

    def _reject_unpairable_skew(self) -> None:
        """Discard queued samples that monotonic peers can no longer match."""
        while self._ranges and self._odometry:
            ranges = self._ranges[0]
            odometry = self._odometry[0]
            if ranges.stamp_ns + self._limits.max_skew_ns < odometry.stamp_ns:
                self._ranges.popleft()
                self.rejections["range_skew_exceeded"] += 1
            elif odometry.stamp_ns + self._limits.max_skew_ns < ranges.stamp_ns:
                self._odometry.popleft()
                self.rejections["odometry_skew_exceeded"] += 1
            else:
                return

    def _fresh_ranges(self, now_ns: int) -> tuple[StampedRangeSample, ...]:
        fresh: list[StampedRangeSample] = []
        for sample in self._ranges:
            age_ns = now_ns - sample.stamp_ns
            if age_ns < 0:
                self.rejections["future_range"] += 1
            elif age_ns > self._limits.max_age_ns:
                self.rejections["stale_range"] += 1
            else:
                fresh.append(sample)
        self._ranges = deque(fresh, maxlen=self._limits.buffer_size)
        return tuple(fresh)

    def _fresh_odometry(self, now_ns: int) -> tuple[StampedOdometrySample, ...]:
        fresh: list[StampedOdometrySample] = []
        for sample in self._odometry:
            age_ns = now_ns - sample.stamp_ns
            if age_ns < 0:
                self.rejections["future_odometry"] += 1
            elif age_ns > self._limits.max_age_ns:
                self.rejections["stale_odometry"] += 1
            else:
                fresh.append(sample)
        self._odometry = deque(fresh, maxlen=self._limits.buffer_size)
        return tuple(fresh)

    def _consume_through(
        self,
        queue: deque[StampedRangeSample] | deque[StampedOdometrySample],
        selected: StampedRangeSample | StampedOdometrySample,
        stream_name: str,
    ) -> None:
        while queue:
            sample = queue.popleft()
            if sample is selected:
                return
            self.rejections[f"superseded_{stream_name}"] += 1
