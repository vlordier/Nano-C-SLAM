"""Exercise the pinned CrazySim firmware UDP boundary without Gazebo physics."""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import time
from collections.abc import Sequence
from pathlib import Path
from typing import cast

CRTP_SETPOINT_SIM = 0x9C
CRTP_SETPOINT_HL = 0x8C
CRTP_PORT_MASK = 0xF0
CRTP_PORT_SHIFT = 4
MOTOR_PACKET_BYTES = 9
MOTOR_STRUCT = struct.Struct("<4H")
IMU_STRUCT = struct.Struct("<BB6h")
TAKEOFF_STRUCT = struct.Struct("<BBBff?f")
FIRMWARE_READY_MARKER = "Self test passed!"


def _read_ready_marker(path: Path, offset: int) -> tuple[bool, int]:
    """Read only newly appended firmware output and return the next offset."""
    try:
        with path.open(encoding="utf-8", errors="replace") as log_file:
            log_file.seek(offset)
            new_log = log_file.read()
            next_offset = log_file.tell()
    except FileNotFoundError:
        return False, offset
    return FIRMWARE_READY_MARKER in new_log, next_offset


def encode_imu_packet(
    acceleration_lsb: Sequence[int], gyro_lsb: Sequence[int]
) -> bytes:
    """Encode one CrazySim gyro/accelerometer CRTP datagram."""
    if len(acceleration_lsb) != 3 or len(gyro_lsb) != 3:
        raise ValueError("acceleration and gyro must each have three axes")
    values = (*acceleration_lsb, *gyro_lsb)
    if any(type(value) is not int or not -32768 <= value <= 32767 for value in values):
        raise ValueError("IMU values must be signed 16-bit integers")
    return IMU_STRUCT.pack(CRTP_SETPOINT_SIM, 0, *values)


def encode_takeoff_packet(height_m: float, duration_s: float) -> bytes:
    """Encode Crazyflie's TAKEOFF_2 command for all commander groups."""
    if not math.isfinite(height_m) or not math.isfinite(duration_s):
        raise ValueError("takeoff values must be finite")
    if height_m <= 0 or duration_s <= 0:
        raise ValueError("height and duration must be positive")
    return TAKEOFF_STRUCT.pack(
        CRTP_SETPOINT_HL,
        7,  # COMMAND_TAKEOFF_2
        0,  # ALL_GROUPS
        height_m,
        0.0,  # yaw
        True,  # use current yaw
        duration_s,
    )


def decode_motor_packet(packet: bytes) -> tuple[int, int, int, int] | None:
    """Decode the firmware's four uint16 motor values; ignore other traffic."""
    if len(packet) != MOTOR_PACKET_BYTES:
        return None
    if packet[0] & CRTP_PORT_MASK != CRTP_SETPOINT_SIM & CRTP_PORT_MASK:
        return None
    motors = cast(tuple[int, int, int, int], MOTOR_STRUCT.unpack_from(packet, 1))
    return motors


def _should_send_imu(elapsed: float, imu_start: float, run_end: float) -> bool:
    """Keep sensor feedback active through takeoff and the post-flight window."""
    return imu_start <= elapsed < run_end


def _has_unsaturated_motor_output(
    motor_packets: int, max_motor_value: int, saturated_values: int
) -> bool:
    """Reject empty output and output that only demonstrates actuator saturation."""
    return motor_packets > 0 and max_motor_value > 0 and saturated_values == 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    # Lima's user-mode network reaches this host through its non-loopback path.
    parser.add_argument("--bind", default="0.0.0.0")  # noqa: S104
    parser.add_argument("--port", type=int, default=19950)
    parser.add_argument("--handshake-timeout", type=float, default=20.0)
    parser.add_argument(
        "--ready-log",
        type=Path,
        required=True,
        help=(
            "wait for 'Self test passed!' in this live firmware log "
            "before IMU injection"
        ),
    )
    parser.add_argument("--ready-timeout", type=float, default=60.0)
    parser.add_argument("--imu-hz", type=float, default=500.0)
    parser.add_argument("--imu-start-delay", type=float, default=3.0)
    parser.add_argument("--imu-seconds", type=float, default=5.0)
    parser.add_argument("--takeoff-height", type=float, default=0.6)
    parser.add_argument("--takeoff-duration", type=float, default=3.0)
    return parser.parse_args()


def run_probe(args: argparse.Namespace) -> int:
    """Answer firmware handshake, inject stationary IMU samples, and observe motors."""
    if args.port < 1 or args.port > 65535:
        raise ValueError("port must be in [1, 65535]")
    if not math.isfinite(args.imu_hz) or args.imu_hz <= 0:
        raise ValueError("imu-hz must be finite and positive")
    if not math.isfinite(args.imu_seconds) or args.imu_seconds <= 0:
        raise ValueError("imu-seconds must be finite and positive")
    if not math.isfinite(args.imu_start_delay) or args.imu_start_delay < 0:
        raise ValueError("imu-start-delay must be finite and non-negative")
    if not math.isfinite(args.ready_timeout) or args.ready_timeout <= 0:
        raise ValueError("ready-timeout must be finite and positive")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    sock.settimeout(min(0.02, 0.5 / args.imu_hz))
    peer: tuple[str, int] | None = None
    handshake_deadline = time.monotonic() + args.handshake_timeout
    imu_packet = encode_imu_packet((0, 0, 2048), (0, 0, 0))
    imu_interval = 1.0 / args.imu_hz
    next_imu = 0.0
    imu_count = 0
    motor_count = 0
    max_motor = 0
    saturated_motor_values = 0
    takeoff_sent = False
    started_at = 0.0
    firmware_ready = False
    log_position = 0
    readiness_deadline = time.monotonic() + args.ready_timeout

    print(
        json.dumps({"event": "listening", "bind": args.bind, "port": args.port}),
        flush=True,
    )
    try:
        while True:
            now = time.monotonic()
            if args.ready_log is not None and not firmware_ready:
                ready_found, log_position = _read_ready_marker(
                    args.ready_log, log_position
                )
                if ready_found:
                    firmware_ready = True
                    started_at = now
                    next_imu = now
                    print(json.dumps({"event": "firmware_ready"}), flush=True)
                elif now >= readiness_deadline:
                    print(json.dumps({"event": "firmware_ready_timeout"}), flush=True)
                    return 3

            try:
                packet, source = sock.recvfrom(256)
            except TimeoutError:
                packet = b""
                source = ("", 0)

            if packet == b"\xf3":
                peer = source
                started_at = now
                next_imu = now
                sock.sendto(b"\xf3", peer)
                print(json.dumps({"event": "handshake", "peer": peer}), flush=True)
            elif peer is not None:
                motor_values = decode_motor_packet(packet)
                if motor_values is not None:
                    motor_count += 1
                    max_motor = max(max_motor, *motor_values)
                    saturated_motor_values += sum(
                        value == 0xFFFF for value in motor_values
                    )

            if peer is None:
                if now >= handshake_deadline:
                    print(json.dumps({"event": "handshake_timeout"}), flush=True)
                    return 2
                continue

            if not firmware_ready:
                continue

            elapsed = now - started_at
            imu_start = args.imu_start_delay
            takeoff_at = imu_start + args.imu_seconds
            run_end = takeoff_at + args.takeoff_duration + 2.0
            if _should_send_imu(elapsed, imu_start, run_end) and now >= next_imu:
                sock.sendto(imu_packet, peer)
                imu_count += 1
                next_imu += imu_interval
                if now - next_imu > imu_interval * 4:
                    next_imu = now + imu_interval

            if not takeoff_sent and elapsed >= takeoff_at:
                sock.sendto(
                    encode_takeoff_packet(args.takeoff_height, args.takeoff_duration),
                    peer,
                )
                takeoff_sent = True
                print(
                    json.dumps({"event": "takeoff_sent", "elapsed_s": elapsed}),
                    flush=True,
                )

            if elapsed >= run_end:
                break
    finally:
        sock.close()

    result = {
        "event": "result",
        "imu_packets_sent": imu_count,
        "motor_packets_received": motor_count,
        "max_motor_value": max_motor,
        "saturated_motor_values": saturated_motor_values,
        "firmware_peer": peer,
    }
    print(json.dumps(result, sort_keys=True), flush=True)
    return int(
        not _has_unsaturated_motor_output(
            motor_count, max_motor, saturated_motor_values
        )
    )


def main() -> int:
    """Run the bounded UDP sensor/motor probe."""
    return run_probe(_parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
