"""Unit tests for the bounded CrazySim firmware transport probe."""

from __future__ import annotations

import math
import struct
import tempfile
import unittest
from pathlib import Path

from scripts.crazysim_udp_probe import (
    CRTP_SETPOINT_HL,
    CRTP_SETPOINT_SIM,
    _read_ready_marker,
    decode_motor_packet,
    encode_imu_packet,
    encode_takeoff_packet,
)


class CrazySimUdpProbeTests(unittest.TestCase):
    def test_ready_marker_reader_observes_only_appended_log_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            log_path = Path(temporary_directory) / "firmware.log"
            log_path.write_text("booting\n", encoding="utf-8")
            ready, offset = _read_ready_marker(log_path, 0)
            self.assertFalse(ready)
            self.assertEqual(offset, len("booting\n"))

            with log_path.open("a", encoding="utf-8") as log_file:
                log_file.write("SYS: Self test passed!\n")
            ready, next_offset = _read_ready_marker(log_path, offset)
            self.assertTrue(ready)
            self.assertEqual(next_offset, log_path.stat().st_size)

    def test_ready_marker_reader_tolerates_log_not_created_yet(self) -> None:
        missing_path = Path("/path/that/does/not/exist/firmware.log")

        self.assertEqual(_read_ready_marker(missing_path, 17), (False, 17))

    def test_imu_packet_has_crtp_header_type_and_six_signed_axes(self) -> None:
        packet = encode_imu_packet((1, -2, 2048), (3, -4, 5))

        self.assertEqual(
            packet,
            struct.pack("<BB6h", CRTP_SETPOINT_SIM, 0, 1, -2, 2048, 3, -4, 5),
        )

    def test_imu_encoder_rejects_wrong_shape_and_overflow(self) -> None:
        with self.assertRaises(ValueError):
            encode_imu_packet((1, 2), (0, 0, 0))
        with self.assertRaises(ValueError):
            encode_imu_packet((0, 0, 32768), (0, 0, 0))

    def test_takeoff_packet_preserves_packed_high_level_command_layout(self) -> None:
        packet = encode_takeoff_packet(0.6, 3.0)

        self.assertEqual(len(packet), 16)
        self.assertEqual(packet[0], CRTP_SETPOINT_HL)
        self.assertEqual(packet[1:3], bytes((7, 0)))
        height, yaw, use_current_yaw, duration = struct.unpack_from("<ff?f", packet, 3)
        self.assertAlmostEqual(height, 0.6)
        self.assertAlmostEqual(yaw, 0.0)
        self.assertTrue(use_current_yaw)
        self.assertAlmostEqual(duration, 3.0)

    def test_takeoff_encoder_rejects_non_finite_or_non_positive_values(self) -> None:
        for values in ((math.nan, 1.0), (1.0, math.inf), (0.0, 1.0), (1.0, 0.0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                encode_takeoff_packet(*values)

    def test_motor_decoder_accepts_only_full_setpoint_sim_packet(self) -> None:
        packet = bytes((CRTP_SETPOINT_SIM,)) + struct.pack("<4H", 100, 200, 300, 400)

        self.assertEqual(decode_motor_packet(packet), (100, 200, 300, 400))
        self.assertIsNone(decode_motor_packet(packet[:-1]))
        self.assertIsNone(decode_motor_packet(bytes((0x8C,)) + packet[1:]))


if __name__ == "__main__":
    unittest.main()
