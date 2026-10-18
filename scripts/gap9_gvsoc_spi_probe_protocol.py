"""Pure protocol helpers for the isolated GAP9 GVSOC SPI probe."""

from __future__ import annotations

import struct

FRAME_SIZE = 97
POSE_PAYLOAD = struct.Struct("<h2xi3f32h")


def make_pose_frame(pose_id: int = 0) -> bytes:
    """Build one production-layout POSE frame with four valid ToF samples."""
    if not 0 <= pose_id <= 0x7FFF:
        raise ValueError("pose_id must fit the decoder's nonnegative int16 range")

    tof = [-1] * 32
    for direction, distance in enumerate((1000, 1200, 900, 1100)):
        tof[direction * 8 + 3] = distance
    payload = POSE_PAYLOAD.pack(pose_id, pose_id * 100, 0.0, 0.0, 0.0, *tof)
    frame = bytearray(FRAME_SIZE)
    frame[0] = ord("S")
    frame[1] = 0
    frame[2 : 2 + len(payload)] = payload
    checksum = sum(payload).to_bytes(4, byteorder="little")
    frame[2 + len(payload) : 6 + len(payload)] = checksum
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
