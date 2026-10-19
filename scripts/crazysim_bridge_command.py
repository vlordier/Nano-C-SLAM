"""Send an absolute-altitude high-level command through the CrazySim bridge."""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import time
from typing import Literal

CRTP_HIGH_LEVEL_HEADER = 0x8C
COMMAND_IDS = {"takeoff": 7, "land": 8}
COMMAND_STRUCT = struct.Struct("<BBBff?f")

type HighLevelCommand = Literal["takeoff", "land"]


def encode_command(
    command: HighLevelCommand, target_height_m: float, duration_s: float
) -> bytes:
    """Encode TAKEOFF_2 or LAND_2 with an absolute world-frame altitude."""
    if command not in COMMAND_IDS:
        raise ValueError(f"unsupported high-level command: {command}")
    if not math.isfinite(target_height_m) or target_height_m <= 0:
        raise ValueError("target height must be finite and positive")
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("duration must be finite and positive")
    return COMMAND_STRUCT.pack(
        CRTP_HIGH_LEVEL_HEADER,
        COMMAND_IDS[command],
        0,  # ALL_GROUPS
        target_height_m,
        0.0,  # yaw (ignored when useCurrentYaw is true)
        True,
        duration_s,
    )


def await_command_ack(
    sock: socket.socket, command: HighLevelCommand, timeout_s: float
) -> bytes:
    """Return the matching high-level commander response before the deadline."""
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("response timeout must be finite and positive")
    command_id = COMMAND_IDS[command]
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"no acknowledgement for {command}")
        sock.settimeout(remaining)
        try:
            packet, _ = sock.recvfrom(64)
        except TimeoutError as error:
            raise TimeoutError(f"no acknowledgement for {command}") from error
        if (
            len(packet) >= 5
            and packet[0] == CRTP_HIGH_LEVEL_HEADER
            and packet[1] == command_id
        ):
            return packet


def run_command(args: argparse.Namespace) -> int:
    """Send one command to the plugin's cflib UDP port and print its response."""
    if not 1 <= args.bridge_port <= 65535:
        raise ValueError("bridge port must be in [1, 65535]")
    packet = encode_command(args.command, args.target_height_m, args.duration_s)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.bind, 0))
        sock.sendto(packet, (args.bridge_host, args.bridge_port))
        acknowledgement = await_command_ack(sock, args.command, args.timeout)
    print(
        json.dumps(
            {
                "event": "firmware_ack",
                "command": args.command,
                "target_height_m": args.target_height_m,
                "duration_s": args.duration_s,
                "firmware_result": acknowledgement[4],
                "packet_hex": acknowledgement.hex(),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=tuple(COMMAND_IDS))
    parser.add_argument(
        "--target-height-m",
        type=float,
        required=True,
        help="absolute Gazebo world-frame altitude in metres",
    )
    parser.add_argument("--duration-s", type=float, required=True)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=20850)
    parser.add_argument("--bind", default="0.0.0.0")  # noqa: S104
    parser.add_argument("--timeout", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    """Run the bounded CrazySim bridge command client."""
    return run_command(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
