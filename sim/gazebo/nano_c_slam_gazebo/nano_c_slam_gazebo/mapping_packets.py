"""Encode paired Gazebo telemetry as the production GAP9 SPI wire protocol."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
from typing import TypeGuard

from .firmware_protocol import (
    OPTIMIZE_CMD,
    POSE_CMD,
    SCAN_CMD,
    make_optimize_frame,
    make_pose_frame,
    make_scan_frame,
)
from .mapping_model import MappingSample
from .telemetry_model import PairedTelemetry

MAX_FIRMWARE_POSES = 260
DEFAULT_SCAN_WINDOW_SIZE = 20
GAZEBO_MAPPING_EVIDENCE_CLASS = "host-execution-of-owned-gap9-mapping-core"


@dataclass(frozen=True)
class MappingPacket:
    """One firmware command and its relationship to captured Gazebo input."""

    sequence: int
    command: int
    pose_id: int | None
    source_stamp_ns: int | None
    frame: bytes


def _is_mapping(value: object) -> TypeGuard[dict[str, object]]:
    return isinstance(value, dict) and all(isinstance(key, str) for key in value)


def _is_integer(value: object) -> TypeGuard[int]:
    return isinstance(value, int) and not isinstance(value, bool)


def samples_from_run_bundle(
    bundle: object, *, expected_run_id: str | None = None
) -> tuple[str, tuple[MappingSample, ...]]:
    """Read bounded samples only from a complete, identity-matched Gazebo run."""
    if not _is_mapping(bundle):
        raise ValueError("run bundle must be a JSON object")
    run_id = bundle.get("run_id")
    capture = bundle.get("capture_report")
    terminal = bundle.get("mission_terminal")
    drone_id = bundle.get("drone")
    if not isinstance(run_id, str) or not run_id:
        raise ValueError("run bundle must have a nonempty run_id")
    if expected_run_id is not None and run_id != expected_run_id:
        raise ValueError("run bundle does not match the requested run_id")
    if not _is_mapping(capture) or not _is_mapping(terminal):
        raise ValueError("run bundle must include capture and terminal reports")
    if (
        not isinstance(drone_id, str)
        or not drone_id
        or capture.get("drone") != drone_id
        or terminal.get("drone_name") != drone_id
        or capture.get("run_id") != run_id
        or terminal.get("run_id") != run_id
        or capture.get("evidence_class") != GAZEBO_MAPPING_EVIDENCE_CLASS
        or terminal.get("phase") != "COMPLETE"
        or terminal.get("mapping_path_complete") is not True
    ):
        raise ValueError("run, capture, and completed mission identities must match")
    raw_samples = capture.get("input_samples")
    declared_count = capture.get("input_sample_count")
    if (
        not isinstance(raw_samples, list)
        or not 1 <= len(raw_samples) <= MAX_FIRMWARE_POSES
    ):
        raise ValueError("capture must contain between 1 and 260 mapping samples")
    if (
        isinstance(declared_count, bool)
        or not isinstance(declared_count, int)
        or declared_count != len(raw_samples)
    ):
        raise ValueError("capture sample count does not match its sample list")
    limits = capture.get("pairing_limits")
    provenance = capture.get("input_pair_provenance")
    if (
        not _is_mapping(limits)
        or not isinstance(provenance, list)
        or len(provenance) != declared_count
    ):
        raise ValueError("capture must include source provenance for every sample")
    maximum_age_ns = limits.get("maximum_age_ns")
    maximum_skew_ns = limits.get("maximum_skew_ns")
    if (
        not _is_integer(maximum_age_ns)
        or maximum_age_ns < 0
        or not _is_integer(maximum_skew_ns)
        or maximum_skew_ns < 0
    ):
        raise ValueError("capture pairing limits must be nonnegative integers")

    samples: list[MappingSample] = []
    previous_range_sequence = -1
    for expected_id, raw_sample in enumerate(raw_samples):
        if not isinstance(raw_sample, str):
            raise ValueError(
                "capture samples must use the documented text record format"
            )
        fields = raw_sample.split()
        if len(fields) != 9:
            raise ValueError("each capture sample must contain nine fields")
        try:
            sample = MappingSample(
                sample_id=int(fields[0]),
                timestamp_ms=int(fields[1]),
                x_m=float(fields[2]),
                y_m=float(fields[3]),
                yaw_rad=float(fields[4]),
                ranges_mm=(
                    int(fields[5]),
                    int(fields[6]),
                    int(fields[7]),
                    int(fields[8]),
                ),
            )
        except (ValueError, OverflowError) as error:
            raise ValueError(f"capture sample {expected_id} is malformed") from error
        if sample.sample_id != expected_id:
            raise ValueError("capture pose IDs must be contiguous and start at zero")
        sample.to_record()
        if samples and sample.timestamp_ms <= samples[-1].timestamp_ms:
            raise ValueError("capture timestamps must increase strictly")
        pair = provenance[expected_id]
        if not _is_mapping(pair):
            raise ValueError(f"capture provenance {expected_id} must be an object")
        range_age_ns = pair.get("range_age_ns")
        odometry_age_ns = pair.get("odometry_age_ns")
        pair_skew_ns = pair.get("pair_skew_ns")
        odometry_stamp_ns = pair.get("odometry_stamp_ns")
        range_stamp_ns = pair.get("range_stamp_ns")
        range_sequence = pair.get("range_sequence")
        if (
            pair.get("drone_id") != drone_id
            or not _is_integer(range_age_ns)
            or not _is_integer(odometry_age_ns)
            or not _is_integer(pair_skew_ns)
            or not _is_integer(odometry_stamp_ns)
            or not _is_integer(range_stamp_ns)
            or not _is_integer(range_sequence)
            or range_age_ns < 0
            or odometry_age_ns < 0
            or odometry_stamp_ns < 0
            or range_stamp_ns < 0
            or range_age_ns > maximum_age_ns
            or odometry_age_ns > maximum_age_ns
            or abs(pair_skew_ns) > maximum_skew_ns
            or abs(odometry_stamp_ns - range_stamp_ns) != abs(pair_skew_ns)
            or range_sequence <= previous_range_sequence
            or odometry_stamp_ns // 1_000_000 != sample.timestamp_ms
        ):
            raise ValueError(f"capture provenance {expected_id} is stale or mismatched")
        previous_range_sequence = range_sequence
        sample = replace(sample, source_stamp_ns=odometry_stamp_ns)
        sample.to_record()
        samples.append(sample)
    return run_id, tuple(samples)


def mapping_sample_from_pair(
    pair: PairedTelemetry,
    *,
    pose_id: int,
    max_skew_ns: int = 100_000_000,
) -> MappingSample:
    """Convert one validated same-drone Gazebo telemetry pair to firmware pose."""
    if pair.ranges.drone_id != pair.odometry.drone_id:
        raise ValueError("range and odometry samples belong to different drones")
    if (
        isinstance(pair.skew_ns, bool)
        or not isinstance(pair.skew_ns, int)
        or isinstance(max_skew_ns, bool)
        or not isinstance(max_skew_ns, int)
        or max_skew_ns < 0
        or abs(pair.skew_ns) > max_skew_ns
    ):
        raise ValueError("range and odometry source timestamps exceed the skew limit")
    timestamp_ms = pair.odometry.stamp_ns // 1_000_000
    sample = MappingSample(
        sample_id=pose_id,
        timestamp_ms=timestamp_ms,
        x_m=pair.odometry.x_m,
        y_m=pair.odometry.y_m,
        yaw_rad=pair.odometry.yaw_rad,
        ranges_mm=pair.ranges.ranges_mm,
        source_stamp_ns=pair.odometry.stamp_ns,
    )
    sample.to_record()
    return sample


def encode_mapping_samples(
    samples: Sequence[MappingSample],
    *,
    scan_window_size: int = DEFAULT_SCAN_WINDOW_SIZE,
) -> tuple[MappingPacket, ...]:
    """Build ordered POSE/SCAN/OPTIMIZE packets from one bounded capture.

    POSE frames retain the four raw Gazebo ranges in the matching four ToF
    directions (column 3); other matrix cells use the firmware's -1 sentinel.
    The firmware applies its own scan range cutoff when projecting points.
    """
    encoder = MappingPacketEncoder(scan_window_size=scan_window_size)
    packets = [packet for sample in samples for packet in encoder.push(sample)]
    packets.extend(encoder.finish())
    return tuple(packets)


class MappingPacketEncoder:
    """Incrementally encode accepted telemetry without buffering a full mission."""

    def __init__(self, *, scan_window_size: int = DEFAULT_SCAN_WINDOW_SIZE) -> None:
        if (
            isinstance(scan_window_size, bool)
            or not isinstance(scan_window_size, int)
            or not 1 <= scan_window_size <= MAX_FIRMWARE_POSES
        ):
            raise ValueError("scan window size must be between 1 and 260")
        self._scan_window_size = scan_window_size
        self._next_pose_id = 0
        self._next_sequence = 0
        self._previous_timestamp_ms = -1
        self._finished = False

    def push(self, sample: MappingSample) -> tuple[MappingPacket, ...]:
        """Encode one next pose and its scan when a complete window closes."""
        if self._finished:
            raise ValueError("cannot append a pose after packet stream completion")
        if self._next_pose_id >= MAX_FIRMWARE_POSES:
            raise ValueError("mapping capture must contain 1..260 poses")
        sample.to_record()
        if sample.sample_id != self._next_pose_id:
            raise ValueError("pose IDs must be contiguous and start at zero")
        if sample.timestamp_ms <= self._previous_timestamp_ms:
            raise ValueError("pose timestamps must increase strictly")
        self._previous_timestamp_ms = sample.timestamp_ms
        stamp_ns = (
            sample.source_stamp_ns
            if sample.source_stamp_ns is not None
            else sample.timestamp_ms * 1_000_000
        )
        packets = [
            self._packet(
                POSE_CMD,
                sample.sample_id,
                stamp_ns,
                make_pose_frame(
                    sample.sample_id,
                    timestamp=sample.timestamp_ms,
                    position=(sample.x_m, sample.y_m, sample.yaw_rad),
                    tof_ranges=sample.ranges_mm,
                ),
            )
        ]
        completed_poses = self._next_pose_id + 1
        self._next_pose_id = completed_poses
        if completed_poses % self._scan_window_size == 0:
            start_id = completed_poses - self._scan_window_size
            packets.append(
                self._packet(SCAN_CMD, start_id, stamp_ns, make_scan_frame(start_id))
            )
        return tuple(packets)

    def finish(self) -> tuple[MappingPacket, ...]:
        """Emit the sole OPTIMIZE command and make further input invalid."""
        if self._finished:
            raise ValueError("packet stream is already complete")
        if self._next_pose_id == 0:
            raise ValueError("mapping capture must contain at least one pose")
        self._finished = True
        return (self._packet(OPTIMIZE_CMD, None, None, make_optimize_frame()),)

    def _packet(
        self,
        command: int,
        pose_id: int | None,
        source_stamp_ns: int | None,
        frame: bytes,
    ) -> MappingPacket:
        packet = MappingPacket(
            sequence=self._next_sequence,
            command=command,
            pose_id=pose_id,
            source_stamp_ns=source_stamp_ns,
            frame=frame,
        )
        self._next_sequence += 1
        return packet
