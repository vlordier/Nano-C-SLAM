"""Protocol tests for sending flight commands through the CrazySim bridge."""

from __future__ import annotations

import socket
import struct
import sys
import unittest
from unittest.mock import Mock, patch

from scripts.crazysim_bridge_command import (
    await_command_ack,
    encode_command,
    parse_args,
)


class CrazySimBridgeCommandTests(unittest.TestCase):
    def test_cli_defaults_to_crazysim_cflib_port(self) -> None:
        with patch.object(
            sys,
            "argv",
            [
                "crazysim_bridge_command",
                "takeoff",
                "--target-height-m",
                "2.4",
                "--duration-s",
                "2",
            ],
        ):
            args = parse_args()

        self.assertEqual(args.bridge_port, 20850)

    def test_takeoff_encodes_absolute_height_and_duration(self) -> None:
        packet = encode_command("takeoff", 2.615, 3.0)

        self.assertEqual(
            packet,
            struct.pack("<BBBff?f", 0x8C, 7, 0, 2.615, 0.0, True, 3.0),
        )

    def test_land_uses_high_level_land_command_id(self) -> None:
        packet = encode_command("land", 2.015, 3.0)

        self.assertEqual(packet[0:3], bytes((0x8C, 8, 0)))
        self.assertEqual(packet[3:], struct.pack("<ff?f", 2.015, 0.0, True, 3.0))

    def test_encode_rejects_nonfinite_or_nonpositive_values(self) -> None:
        for target, duration in ((float("nan"), 3.0), (2.0, float("inf")), (0.0, 1.0)):
            with (
                self.subTest(target=target, duration=duration),
                self.assertRaises(ValueError),
            ):
                encode_command("takeoff", target, duration)

    def test_ack_wait_ignores_sensor_datagrams_until_matching_command_reply(
        self,
    ) -> None:
        sock = Mock(spec=socket.socket)
        sock.recvfrom.side_effect = [
            (bytes((0x9C, 0, 0)), ("127.0.0.1", 19850)),
            (bytes((0x8C, 7, 0, 0x29, 1)), ("127.0.0.1", 19850)),
        ]

        acknowledgement = await_command_ack(sock, "takeoff", 1.0)

        self.assertEqual(acknowledgement, bytes((0x8C, 7, 0, 0x29, 1)))
        self.assertEqual(sock.recvfrom.call_count, 2)


if __name__ == "__main__":
    unittest.main()
