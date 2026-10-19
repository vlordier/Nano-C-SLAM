"""Pure protocol helpers for the isolated GAP9 GVSOC SPI probe."""

from __future__ import annotations

import math
import struct

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.firmware_protocol import (
    FRAME_SIZE,
    OPTIMIZE_CMD,
    POSE_CMD,
    POSE_PAYLOAD,
    POSE_REQ_CMD,
    SCAN_CMD,
    make_optimize_frame,
    make_pose_frame,
    make_pose_request_frame,
    make_scan_frame,
)

__all__ = [
    "FRAME_SIZE",
    "OPTIMIZE_CMD",
    "POSE_CMD",
    "POSE_PAYLOAD",
    "POSE_REQ_CMD",
    "SCAN_CMD",
    "make_mapping_trace",
    "make_pose_frame",
    "make_pose_request_frame",
    "make_scan_frame",
]


def _float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


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
    if command == OPTIMIZE_CMD:
        return make_optimize_frame()
    return bytes((ord("S"), command)) + bytes(FRAME_SIZE - 3) + b"E"
