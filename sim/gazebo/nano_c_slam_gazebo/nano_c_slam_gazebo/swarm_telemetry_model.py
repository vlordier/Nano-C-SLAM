"""Pure validation for stamped three-drone Gazebo telemetry."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from numbers import Integral

from .mission_model import SUPPORTED_DRONE_NAMES
from .sensor_model import odometry_stamp_seconds

PAIR_LABELS = tuple(
    f"{first}:{second}"
    for index, first in enumerate(SUPPORTED_DRONE_NAMES)
    for second in SUPPORTED_DRONE_NAMES[index + 1 :]
)
MAX_SOURCE_AGE_NS = 500_000_000
MAX_SOURCE_SKEW_NS = 100_000_000
MAX_FUTURE_NS = 100_000_000


@dataclass(frozen=True)
class PairwiseSnapshotEvidence:
    sequence: int
    publication_stamp_ns: int
    source_stamps_ns: tuple[int, ...]
    source_age_ns: tuple[int, ...]
    source_skew_ns: int
    drone_ids: tuple[str, ...]
    pair_labels: tuple[str, ...]
    published_ranges_m: tuple[float, ...]
    computed_ranges_m: tuple[float, ...]
    residuals_m: tuple[float, ...]


@dataclass
class StampSpan:
    """Bounded count and first/last evidence for a strictly advancing stamp stream."""

    count: int = 0
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None

    def observe(self, stamp_ns_value: object) -> None:
        if (
            isinstance(stamp_ns_value, bool)
            or not isinstance(stamp_ns_value, int)
            or stamp_ns_value <= 0
        ):
            raise ValueError("timestamp must be a positive integer")
        if self.last_stamp_ns is not None and stamp_ns_value <= self.last_stamp_ns:
            raise ValueError("timestamp did not advance")
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns_value
        self.last_stamp_ns = stamp_ns_value
        self.count += 1

    def require_at_least(self, minimum_count: int, stream_name: str) -> None:
        if self.count < minimum_count:
            raise ValueError(
                f"{stream_name} advanced {self.count} times; requires {minimum_count}"
            )

    def summary(self) -> dict[str, int | None]:
        return {
            "count": self.count,
            "first_stamp_ns": self.first_stamp_ns,
            "last_stamp_ns": self.last_stamp_ns,
            "span_ns": (self.last_stamp_ns or 0) - (self.first_stamp_ns or 0),
        }


def stamp_ns(stamp: object) -> int:
    """Convert a ROS time-shaped value to integer nanoseconds."""
    seconds = odometry_stamp_seconds(
        getattr(stamp, "sec", None), getattr(stamp, "nanosec", None)
    )
    return round(seconds * 1_000_000_000)


def validate_stamped_ranges(
    message: object,
    *,
    expected_drone: str,
    now_ns: int,
) -> tuple[int, int, tuple[int, int, int, int]]:
    """Check range identity, source age, payload, and sequence-domain fields."""
    if expected_drone not in SUPPORTED_DRONE_NAMES:
        raise ValueError(f"unsupported drone {expected_drone!r}")
    if getattr(message, "drone_id", None) != expected_drone:
        raise ValueError(f"range message identity does not match {expected_drone}")
    header = getattr(message, "header", None)
    source_ns = stamp_ns(getattr(header, "stamp", None))
    age_ns = now_ns - source_ns
    if age_ns > MAX_SOURCE_AGE_NS:
        raise ValueError(f"{expected_drone} range sample is stale")
    if age_ns < -MAX_FUTURE_NS:
        raise ValueError(f"{expected_drone} range sample is too far in the future")
    sequence = getattr(message, "sequence", None)
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError(f"{expected_drone} range sequence is invalid")
    values = tuple(getattr(message, "ranges_mm", ()))
    if len(values) != 4:
        raise ValueError(f"{expected_drone} ranges must contain exactly four rays")
    if any(
        isinstance(value, bool)
        or not isinstance(value, Integral)
        or value < -1
        or value > 32767
        for value in values
    ):
        raise ValueError(
            f"{expected_drone} ranges contain invalid millimeter values: {values!r}"
        )
    normalized_values = tuple(int(value) for value in values)
    return source_ns, sequence, normalized_values  # type: ignore[return-value]


def validate_pairwise_snapshot(
    message: object,
    *,
    now_ns: int,
    odometry: Mapping[str, Mapping[int, Sequence[float]]],
) -> tuple[int, tuple[float, ...]]:
    """Validate pair labels and distances against exact retained odometry."""
    evidence = analyze_pairwise_snapshot(message, now_ns=now_ns, odometry=odometry)
    return evidence.sequence, evidence.published_ranges_m


def analyze_pairwise_snapshot(
    message: object,
    *,
    now_ns: int,
    odometry: Mapping[str, Mapping[int, Sequence[float]]],
) -> PairwiseSnapshotEvidence:
    """Validate a pairwise message and retain its timestamp/distance evidence."""
    header = getattr(message, "header", None)
    publication_ns = stamp_ns(getattr(header, "stamp", None))
    if publication_ns - now_ns > MAX_FUTURE_NS:
        raise ValueError("pairwise publication stamp is too far in the future")
    if now_ns - publication_ns > MAX_SOURCE_AGE_NS:
        raise ValueError("pairwise publication is stale")
    if tuple(getattr(message, "drone_ids", ())) != SUPPORTED_DRONE_NAMES:
        raise ValueError("pairwise drone IDs do not match stable cf1/cf3/cf4 order")
    source_stamps = tuple(stamp_ns(value) for value in message.source_stamps)
    if len(source_stamps) != len(SUPPORTED_DRONE_NAMES):
        raise ValueError("pairwise source stamp count does not match drone IDs")
    if max(source_stamps) - min(source_stamps) > MAX_SOURCE_SKEW_NS:
        raise ValueError("pairwise source odometry exceeds allowed inter-drone skew")
    for drone, source_ns in zip(SUPPORTED_DRONE_NAMES, source_stamps, strict=True):
        age_ns = now_ns - source_ns
        if age_ns > MAX_SOURCE_AGE_NS or age_ns < -MAX_FUTURE_NS:
            raise ValueError(f"{drone} pairwise source odometry is stale or future")
        if source_ns not in odometry.get(drone, {}):
            raise ValueError(f"{drone} exact pairwise source odometry is unavailable")
    if tuple(message.pair_labels) != PAIR_LABELS:
        raise ValueError("pairwise labels do not match stable pair order")
    ranges = tuple(float(value) for value in message.ranges_m)
    if len(ranges) != len(PAIR_LABELS) or any(
        not math.isfinite(value) or value < 0 for value in ranges
    ):
        raise ValueError("pairwise ranges must contain three finite nonnegative values")
    expected: list[float] = []
    for first_index, first in enumerate(SUPPORTED_DRONE_NAMES):
        first_position = odometry[first][source_stamps[first_index]]
        if len(first_position) != 3 or not all(map(math.isfinite, first_position)):
            raise ValueError(f"{first} source odometry position is malformed")
        for second_index in range(first_index + 1, len(SUPPORTED_DRONE_NAMES)):
            second = SUPPORTED_DRONE_NAMES[second_index]
            second_position = odometry[second][source_stamps[second_index]]
            distance = math.dist(first_position, second_position)
            expected.append(distance)
    if any(
        abs(actual - wanted) > 1e-4
        for actual, wanted in zip(ranges, expected, strict=True)
    ):
        raise ValueError("pairwise ranges do not match exact source odometry positions")
    sequence = getattr(message, "sequence", None)
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("pairwise sequence is invalid")
    return PairwiseSnapshotEvidence(
        sequence=sequence,
        publication_stamp_ns=publication_ns,
        source_stamps_ns=source_stamps,
        source_age_ns=tuple(now_ns - stamp for stamp in source_stamps),
        source_skew_ns=max(source_stamps) - min(source_stamps),
        drone_ids=tuple(message.drone_ids),
        pair_labels=tuple(message.pair_labels),
        published_ranges_m=ranges,
        computed_ranges_m=tuple(expected),
        residuals_m=tuple(
            abs(actual - wanted)
            for actual, wanted in zip(ranges, expected, strict=True)
        ),
    )
