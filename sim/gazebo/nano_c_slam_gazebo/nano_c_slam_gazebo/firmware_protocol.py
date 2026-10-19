"""Wire-compatible Nano-C-SLAM STM32-to-GAP9 command frames."""

from __future__ import annotations

import struct

FRAME_SIZE = 97
POSE_PAYLOAD = struct.Struct("<h2xi3f32h")
POSE_CMD = 0
SCAN_CMD = 1
OPTIMIZE_CMD = 2
POSE_REQ_CMD = 3


def make_pose_frame(
    pose_id: int = 0,
    *,
    timestamp: int | None = None,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    tof_ranges: tuple[int, int, int, int] = (1000, 1200, 900, 1100),
) -> bytes:
    """Build a POSE frame with four ranges in the symmetric central bins."""
    if not 0 <= pose_id <= 0x7FFF:
        raise ValueError("pose_id must fit the decoder's nonnegative int16 range")
    if len(position) != 3 or len(tof_ranges) != 4:
        raise ValueError("POSE requires 3 coordinates and 4 directional ranges")
    tof = [-1] * 32
    for direction, distance in enumerate(tof_ranges):
        tof[direction * 8 + 3] = distance
        tof[direction * 8 + 4] = distance
    pose_timestamp = pose_id * 100 if timestamp is None else timestamp
    payload = POSE_PAYLOAD.pack(pose_id, pose_timestamp, *position, *tof)
    return _guarded_frame(POSE_CMD, payload, checksum_width=4)


def make_scan_frame(pose_id: int = 0) -> bytes:
    """Build a zero-padded SCAN frame with its signed-int16 byte-sum checksum."""
    if not 0 <= pose_id <= 0x7FFF:
        raise ValueError("pose_id must fit the decoder's nonnegative int16 range")
    payload = bytearray(FRAME_SIZE - 3)
    payload[:2] = pose_id.to_bytes(2, byteorder="little", signed=True)
    checksum = sum(payload[:2]).to_bytes(2, byteorder="little", signed=True)
    payload[2:4] = checksum
    return _guarded_frame(SCAN_CMD, bytes(payload), checksum_width=0)


def make_optimize_frame() -> bytes:
    """Build an empty production OPTIMIZE command frame."""
    return _guarded_frame(OPTIMIZE_CMD, b"", checksum_width=0)


def make_pose_request_frame(from_id: int = 0, count: int = 2) -> bytes:
    """Build a POSE_REQ frame with the firmware's signed-int16 checksum."""
    if not -0x8000 <= from_id <= 0x7FFF:
        raise ValueError("from_id must fit signed int16")
    if not -0x8000 <= count <= 0x7FFF:
        raise ValueError("count must fit signed int16")
    payload = bytearray(FRAME_SIZE - 3)
    payload[:2] = from_id.to_bytes(2, "little", signed=True)
    payload[2:4] = count.to_bytes(2, "little", signed=True)
    checksum = sum(payload[:4])
    payload[4:6] = checksum.to_bytes(2, "little", signed=True)
    return _guarded_frame(POSE_REQ_CMD, bytes(payload), checksum_width=0)


def _guarded_frame(command: int, payload: bytes, *, checksum_width: int) -> bytes:
    frame = bytearray(FRAME_SIZE)
    frame[0] = ord("S")
    frame[1] = command
    frame[2 : 2 + len(payload)] = payload
    if checksum_width:
        checksum = sum(payload).to_bytes(checksum_width, byteorder="little")
        checksum_offset = 2 + len(payload)
        frame[checksum_offset : checksum_offset + checksum_width] = checksum
    frame[-1] = ord("E")
    return bytes(frame)
