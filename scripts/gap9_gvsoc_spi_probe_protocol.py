"""Pure protocol helpers for the isolated GAP9 GVSOC SPI probe."""

from __future__ import annotations

import math
import struct

FRAME_SIZE = 97
POSE_PAYLOAD = struct.Struct("<h2xi3f32h")
POSE_CMD = 0
SCAN_CMD = 1
OPTIMIZE_CMD = 2
POSE_REQ_CMD = 3


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


def make_mapping_trace(
    pose_count: int = 60,
    scan_interval: int = 20,
    *,
    stationary: bool = False,
    periodic: bool = False,
    scans_after_poses: bool = False,
) -> tuple[bytes, ...]:
    """Build a bounded production-layout trace with periodic scan anchors.

    ``stationary`` repeats an identical pose and sensor observation to stress
    loop-closure and optimizer capacity independently of trajectory accuracy.
    ``periodic`` repeats a non-degenerate two-pose pattern. With even scan
    intervals, each 20-pose ICP window is identical at each scan anchor.
    ``scans_after_poses`` queues all anchors after pose ingress so every
    forward 20-pose ICP window exists when the firmware evaluates it.
    """
    if not 1 <= pose_count <= 260:
        raise ValueError("pose_count must fit the GAP9 graph capacity (1..260)")
    if not 1 <= scan_interval <= pose_count:
        raise ValueError("scan_interval must be between 1 and pose_count")
    if stationary and periodic:
        raise ValueError("stationary and periodic trace modes are mutually exclusive")
    invalid_pose = bytearray(make_pose_frame())
    invalid_pose[POSE_PAYLOAD.size + 2] ^= 1
    frames = [bytes(invalid_pose)]
    deferred_scan_frames: list[bytes] = []

    for pose_id in range(pose_count):
        if periodic:
            phase = pose_id % 2
            angle = _float32(phase * 0.35)
            x = _float32(phase * 0.10)
            y = _float32(phase * 0.05)
            yaw = angle
        else:
            angle = 0.0 if stationary else _float32(_float32(pose_id) * _float32(0.02))
            x = (
                0.0
                if stationary
                else _float32(_float32(0.15) * _float32(math.cos(angle)))
            )
            y = (
                0.0
                if stationary
                else _float32(_float32(0.15) * _float32(math.sin(angle)))
            )
            yaw = 0.0 if stationary else _float32(angle + _float32(math.pi / 2))
        frames.append(
            make_pose_frame(
                pose_id,
                timestamp=pose_id * 100,
                position=(x, y, yaw),
            )
        )
        if (pose_id + 1) % scan_interval == 0:
            scan_frame = make_scan_frame(pose_id - scan_interval + 1)
            (deferred_scan_frames if scans_after_poses else frames).append(scan_frame)

    if pose_count % scan_interval:
        scan_frame = make_scan_frame(pose_count - (pose_count % scan_interval))
        (deferred_scan_frames if scans_after_poses else frames).append(scan_frame)
    if pose_count >= 2 * scan_interval:
        scan_frame = make_scan_frame(
            ((pose_count - 1) // scan_interval) * scan_interval
        )
        (deferred_scan_frames if scans_after_poses else frames).append(scan_frame)
    frames.extend(deferred_scan_frames)
    frames.append(_make_command_frame(OPTIMIZE_CMD))
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


def make_pose_request_frame(from_id: int = 0, count: int = 2) -> bytes:
    """Build a POSE_REQ frame with the firmware's signed-int16 checksum."""
    if not -0x8000 <= from_id <= 0x7FFF:
        raise ValueError("from_id must fit signed int16")
    if not -0x8000 <= count <= 0x7FFF:
        raise ValueError("count must fit signed int16")
    frame = bytearray(FRAME_SIZE)
    frame[0] = ord("S")
    frame[1] = POSE_REQ_CMD
    frame[2:4] = from_id.to_bytes(2, "little", signed=True)
    frame[4:6] = count.to_bytes(2, "little", signed=True)
    checksum = sum(frame[2:6])
    frame[6:8] = checksum.to_bytes(2, "little", signed=True)
    frame[-1] = ord("E")
    return bytes(frame)
