"""Bounded run-bound newline protocol for local Gazebo-to-GVSOC frame IPC."""

from __future__ import annotations

import json
import math
import socket
from collections.abc import Iterator
from dataclasses import dataclass
from typing import TypeGuard

from .firmware_protocol import (
    FRAME_SIZE,
    OPTIMIZE_CMD,
    POSE_CMD,
    POSE_PAYLOAD,
    SCAN_CMD,
)
from .mapping_packets import (
    MappingPacket,
    MappingPacketEncoder,
    mapping_sample_from_pair,
)
from .mission_model import validate_drone_name, validate_run_id
from .telemetry_model import PairedTelemetry

STREAM_SCHEMA_VERSION = 1
MAX_RECORD_BYTES = 512
MAX_RECEIVE_CHUNK_BYTES = 8192
SOCKET_READ_BYTES = 4096
_RECORD_KEYS = frozenset(
    {
        "schema_version",
        "run_id",
        "drone_id",
        "sequence",
        "command",
        "pose_id",
        "source_stamp_ns",
        "frame_hex",
    }
)


@dataclass(frozen=True)
class MappingStreamFrame:
    """One bounded SPI frame with source and mission identity attached."""

    run_id: str
    drone_id: str
    sequence: int
    command: int
    pose_id: int | None
    source_stamp_ns: int | None
    frame: bytes


def _is_mapping(value: object) -> TypeGuard[dict[str, object]]:
    return isinstance(value, dict) and all(isinstance(key, str) for key in value)


def _is_integer(value: object) -> TypeGuard[int]:
    return isinstance(value, int) and not isinstance(value, bool)


def _validate_stream_frame(record: MappingStreamFrame) -> MappingStreamFrame:
    validate_run_id(record.run_id)
    validate_drone_name(record.drone_id)
    if not _is_integer(record.sequence) or record.sequence < 0:
        raise ValueError("stream sequence must be a nonnegative integer")
    if not _is_integer(record.command) or record.command not in {
        POSE_CMD,
        SCAN_CMD,
        OPTIMIZE_CMD,
    }:
        raise ValueError("stream command must be POSE, SCAN, or OPTIMIZE")
    if len(record.frame) != FRAME_SIZE:
        raise ValueError("stream frame must contain exactly 97 bytes")
    if record.frame[0] != ord("S") or record.frame[-1] != ord("E"):
        raise ValueError("stream frame has invalid start or end guards")
    if record.frame[1] != record.command:
        raise ValueError("stream command differs from encoded frame command")

    if record.command == POSE_CMD:
        if (
            not _is_integer(record.pose_id)
            or record.pose_id < 0
            or not _is_integer(record.source_stamp_ns)
            or record.source_stamp_ns < 0
        ):
            raise ValueError("POSE requires integer pose and source timestamp metadata")
        pose_values = POSE_PAYLOAD.unpack_from(record.frame, 2)
        pose_id, timestamp_ms = pose_values[:2]
        coordinates = pose_values[2:5]
        ranges = pose_values[5:]
        checksum = int.from_bytes(record.frame[86:90], "little")
        if (
            pose_id != record.pose_id
            or timestamp_ms != record.source_stamp_ns // 1_000_000
        ):
            raise ValueError("POSE metadata differs from encoded payload")
        if checksum != sum(record.frame[2 : 2 + POSE_PAYLOAD.size]):
            raise ValueError("POSE checksum does not match its payload")
        if record.frame[4:6] != b"\0\0" or any(record.frame[90:96]):
            raise ValueError("POSE reserved frame bytes must be zero")
        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError("POSE coordinates must be finite")
        if any(value < -1 for value in ranges):
            raise ValueError("POSE ranges must use -1 or nonnegative millimeters")
    elif record.command == SCAN_CMD:
        if (
            not _is_integer(record.pose_id)
            or record.pose_id < 0
            or not _is_integer(record.source_stamp_ns)
            or record.source_stamp_ns < 0
        ):
            raise ValueError("SCAN requires integer pose and source timestamp metadata")
        pose_id = int.from_bytes(record.frame[2:4], "little", signed=True)
        checksum = int.from_bytes(record.frame[4:8], "little")
        if pose_id != record.pose_id:
            raise ValueError("SCAN metadata differs from encoded pose ID")
        if checksum != sum(record.frame[2:4]):
            raise ValueError("SCAN checksum does not match its pose ID")
        if any(record.frame[8:96]):
            raise ValueError("SCAN reserved frame bytes must be zero")
    elif record.pose_id is not None or record.source_stamp_ns is not None:
        raise ValueError("OPTIMIZE cannot carry pose or source timestamp metadata")
    elif any(record.frame[2:-1]):
        raise ValueError("OPTIMIZE reserved payload bytes must be zero")
    return record


def encode_mapping_stream_record(
    run_id: str, drone_id: str, packet: MappingPacket
) -> bytes:
    """Encode one frame as one bounded newline-delimited JSON record."""
    record = _validate_stream_frame(
        MappingStreamFrame(
            run_id=run_id,
            drone_id=drone_id,
            sequence=packet.sequence,
            command=packet.command,
            pose_id=packet.pose_id,
            source_stamp_ns=packet.source_stamp_ns,
            frame=packet.frame,
        )
    )
    payload = {
        "schema_version": STREAM_SCHEMA_VERSION,
        "run_id": record.run_id,
        "drone_id": record.drone_id,
        "sequence": record.sequence,
        "command": record.command,
        "pose_id": record.pose_id,
        "source_stamp_ns": record.source_stamp_ns,
        "frame_hex": record.frame.hex(),
    }
    encoded = (
        json.dumps(payload, separators=(",", ":"), sort_keys=True, allow_nan=False)
        + "\n"
    ).encode("ascii")
    if len(encoded) > MAX_RECORD_BYTES:
        raise ValueError("encoded stream record exceeds the 512-byte limit")
    return encoded


def _decode_record(
    line: bytes,
    *,
    expected_run_id: str,
    expected_drone_id: str,
    expected_sequence: int,
) -> MappingStreamFrame:
    if not line or len(line) > MAX_RECORD_BYTES:
        raise ValueError("stream record must contain 1..512 bytes")
    try:
        value = json.loads(line)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("stream record is not valid JSON") from error
    if not _is_mapping(value) or set(value) != _RECORD_KEYS:
        raise ValueError("stream record has missing or unknown fields")
    schema_version = value.get("schema_version")
    if not _is_integer(schema_version) or schema_version != STREAM_SCHEMA_VERSION:
        raise ValueError("unsupported mapping stream schema version")
    if value.get("run_id") != expected_run_id:
        raise ValueError("stream record belongs to a different run")
    if value.get("drone_id") != expected_drone_id:
        raise ValueError("stream record belongs to a different drone")
    sequence = value.get("sequence")
    if not _is_integer(sequence) or sequence != expected_sequence:
        raise ValueError("stream sequence is duplicate, missing, or out of order")
    command = value.get("command")
    if not _is_integer(command):
        raise ValueError("stream command must be an integer")
    pose_id = value.get("pose_id")
    source_stamp_ns = value.get("source_stamp_ns")
    frame_hex = value.get("frame_hex")
    if not isinstance(frame_hex, str) or len(frame_hex) != FRAME_SIZE * 2:
        raise ValueError("frame_hex must encode exactly 97 bytes")
    try:
        frame = bytes.fromhex(frame_hex)
    except ValueError as error:
        raise ValueError("frame_hex contains invalid hexadecimal data") from error
    if len(frame) != FRAME_SIZE:
        raise ValueError("frame_hex must encode exactly 97 bytes")
    if pose_id is not None and not _is_integer(pose_id):
        raise ValueError("pose_id must be an integer or null")
    if source_stamp_ns is not None and not _is_integer(source_stamp_ns):
        raise ValueError("source_stamp_ns must be an integer or null")
    record = MappingStreamFrame(
        run_id=expected_run_id,
        drone_id=expected_drone_id,
        sequence=sequence,
        command=command,
        pose_id=pose_id,
        source_stamp_ns=source_stamp_ns,
        frame=frame,
    )
    return _validate_stream_frame(record)


class MappingStreamDecoder:
    """Decode bounded fragmented stream data and enforce a run-local sequence."""

    def __init__(self, *, run_id: str, drone_id: str, first_sequence: int = 0) -> None:
        self.run_id = validate_run_id(run_id)
        self.drone_id = validate_drone_name(drone_id)
        if not _is_integer(first_sequence) or first_sequence < 0:
            raise ValueError("first_sequence must be a nonnegative integer")
        self._next_sequence = first_sequence
        self._buffer = bytearray()

    def feed(self, data: bytes) -> tuple[MappingStreamFrame, ...]:
        """Return decoded records from one bounded socket read."""
        if not isinstance(data, bytes):
            raise TypeError("stream input must be bytes")
        if len(data) > MAX_RECEIVE_CHUNK_BYTES:
            raise ValueError("stream receive chunk exceeds the 8192-byte limit")
        records: list[MappingStreamFrame] = []
        remaining = data
        while remaining:
            delimiter = remaining.find(b"\n")
            if delimiter < 0:
                if len(self._buffer) + len(remaining) > MAX_RECORD_BYTES:
                    raise ValueError("partial stream record exceeds the 512-byte limit")
                self._buffer.extend(remaining)
                break
            if len(self._buffer) + delimiter > MAX_RECORD_BYTES:
                raise ValueError("stream record exceeds the 512-byte limit")
            line = bytes(self._buffer) + remaining[:delimiter]
            self._buffer.clear()
            record = _decode_record(
                line,
                expected_run_id=self.run_id,
                expected_drone_id=self.drone_id,
                expected_sequence=self._next_sequence,
            )
            records.append(record)
            self._next_sequence += 1
            remaining = remaining[delimiter + 1 :]
        return tuple(records)

    def finish(self) -> None:
        """Reject EOF in the middle of a record."""
        if self._buffer:
            raise ValueError("stream ended with a truncated mapping record")


def send_mapping_stream_record(
    connection: socket.socket,
    run_id: str,
    drone_id: str,
    packet: MappingPacket,
    *,
    timeout_s: float,
) -> None:
    """Send one complete record with a finite blocking deadline."""
    _validate_timeout(timeout_s)
    encoded = encode_mapping_stream_record(run_id, drone_id, packet)
    original_timeout = connection.gettimeout()
    try:
        connection.settimeout(timeout_s)
        connection.sendall(encoded)
    finally:
        connection.settimeout(original_timeout)


class MappingStreamWriter:
    """Turn accepted live pairs into bounded socket writes, failing closed."""

    def __init__(
        self,
        connection: socket.socket,
        *,
        run_id: str,
        drone_id: str,
        timeout_s: float,
    ) -> None:
        _validate_timeout(timeout_s)
        self._connection = connection
        self._run_id = validate_run_id(run_id)
        self._drone_id = validate_drone_name(drone_id)
        self._timeout_s = timeout_s
        self._encoder = MappingPacketEncoder()

    def push_pair(self, pair: PairedTelemetry, *, pose_id: int) -> None:
        """Encode and synchronously send one validated telemetry pair."""
        sample = mapping_sample_from_pair(pair, pose_id=pose_id)
        for packet in self._encoder.push(sample):
            send_mapping_stream_record(
                self._connection,
                self._run_id,
                self._drone_id,
                packet,
                timeout_s=self._timeout_s,
            )

    def finish(self) -> None:
        """Send OPTIMIZE after all poses; send failures propagate to caller."""
        for packet in self._encoder.finish():
            send_mapping_stream_record(
                self._connection,
                self._run_id,
                self._drone_id,
                packet,
                timeout_s=self._timeout_s,
            )


def iter_mapping_stream_records(
    connection: socket.socket,
    decoder: MappingStreamDecoder,
    *,
    timeout_s: float,
) -> Iterator[MappingStreamFrame]:
    """Yield frames as received and fail on read timeout or truncated EOF."""
    _validate_timeout(timeout_s)
    original_timeout = connection.gettimeout()
    try:
        connection.settimeout(timeout_s)
        while chunk := connection.recv(SOCKET_READ_BYTES):
            yield from decoder.feed(chunk)
        decoder.finish()
    finally:
        connection.settimeout(original_timeout)


def _validate_timeout(timeout_s: float) -> None:
    if (
        isinstance(timeout_s, bool)
        or not isinstance(timeout_s, (int, float))
        or not math.isfinite(timeout_s)
        or timeout_s <= 0
    ):
        raise ValueError("socket timeout must be finite and positive")
