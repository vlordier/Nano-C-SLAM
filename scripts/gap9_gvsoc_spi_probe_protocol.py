"""Pure protocol helpers for the isolated GAP9 GVSOC SPI probe."""

from __future__ import annotations

FRAME_SIZE = 97


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
