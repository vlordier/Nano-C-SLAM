"""Pure protocol helpers for the isolated GAP9 GVSOC SPI probe."""

from __future__ import annotations

import math
import struct

FRAME_SIZE = 97
POSE_PAYLOAD = struct.Struct("<h2xi3f32h")
POSE_CMD = 0
SCAN_CMD = 1
OPTIMIZE_CMD = 2


def _float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def make_pose_frame(
    pose_id: int = 0,
    *,
    timestamp: int | None = None,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    tof_ranges: tuple[int, int, int, int] = (1000, 1200, 900, 1100),
) -> bytes:
    """Build one production-layout POSE frame with valid ToF samples."""
    if not 0 <= pose_id <= 0x7FFF:
        raise ValueError("pose_id must fit the decoder's nonnegative int16 range")
    if len(position) != 3 or len(tof_ranges) != 4:
        raise ValueError("POSE requires 3 coordinates and 4 directional ranges")

    tof = [-1] * 32
    for direction, distance in enumerate(tof_ranges):
        tof[direction * 8 + 3] = distance
    pose_timestamp = pose_id * 100 if timestamp is None else timestamp
    payload = POSE_PAYLOAD.pack(pose_id, pose_timestamp, *position, *tof)
    frame = bytearray(FRAME_SIZE)
    frame[0] = ord("S")
    frame[1] = 0
    frame[2 : 2 + len(payload)] = payload
    checksum = sum(payload).to_bytes(4, byteorder="little")
    frame[2 + len(payload) : 6 + len(payload)] = checksum
    frame[-1] = ord("E")
    return bytes(frame)


def make_mapping_trace() -> tuple[bytes, ...]:
    """Reproduce the 66-frame owned C trace for real-SPI mapping coverage."""
    invalid_pose = bytearray(make_pose_frame())
    invalid_pose[POSE_PAYLOAD.size + 2] ^= 1
    frames = [bytes(invalid_pose)]

    for pose_id in range(60):
        angle = _float32(_float32(pose_id) * _float32(0.02))
        x = _float32(_float32(0.15) * _float32(math.cos(angle)))
        y = _float32(_float32(0.15) * _float32(math.sin(angle)))
        yaw = _float32(angle + _float32(math.pi / 2))
        frames.append(
            make_pose_frame(
                pose_id,
                timestamp=pose_id * 100,
                position=(x, y, yaw),
            )
        )
        if (pose_id + 1) % 20 == 0:
            frames.append(make_scan_frame(pose_id - 19))

    frames.extend((make_scan_frame(40), _make_command_frame(OPTIMIZE_CMD)))
    return tuple(bytes(frame) for frame in frames)


def _make_command_frame(command: int) -> bytes:
    frame = bytearray(FRAME_SIZE)
    frame[0] = ord("S")
    frame[1] = command
    frame[-1] = ord("E")
    return bytes(frame)


def make_scan_frame(pose_id: int = 0) -> bytes:
    """Build a zero-padded SCAN frame with its signed-int16 byte-sum checksum."""
    if not 0 <= pose_id <= 0x7FFF:
        raise ValueError("pose_id must fit the decoder's nonnegative int16 range")

    frame = bytearray(FRAME_SIZE)
    frame[0] = ord("S")
    frame[1] = 1
    frame[2] = pose_id & 0xFF
    frame[3] = pose_id >> 8
    frame[4:6] = (frame[2] + frame[3]).to_bytes(2, byteorder="little")
    frame[-1] = ord("E")
    return bytes(frame)
