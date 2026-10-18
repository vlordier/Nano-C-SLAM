"""Tests for the pure GVSOC SPI probe frame builder."""

from __future__ import annotations

import unittest

from scripts.gap9_gvsoc_spi_probe_protocol import (
    FRAME_SIZE,
    POSE_PAYLOAD,
    make_pose_frame,
    make_scan_frame,
)


class Gap9GvsocSpiProbeProtocolTests(unittest.TestCase):
    def test_builds_stm32_scan_frame_for_pose_zero(self) -> None:
        frame = make_scan_frame()

        self.assertEqual(len(frame), FRAME_SIZE)
        self.assertEqual(frame[:8], b"S\x01\x00\x00\x00\x00\x00\x00")
        self.assertEqual(frame[8:-1], bytes(FRAME_SIZE - 9))
        self.assertEqual(frame[-1:], b"E")

    def test_encodes_id_and_unsigned_byte_sum_little_endian(self) -> None:
        frame = make_scan_frame(0x0102)

        self.assertEqual(frame[2:6], b"\x02\x01\x03\x00")

    def test_rejects_ids_outside_decoder_range(self) -> None:
        for pose_id in (-1, 0x8000):
            with self.subTest(pose_id=pose_id), self.assertRaises(ValueError):
                make_scan_frame(pose_id)

    def test_builds_production_pose_layout_with_checksum_and_valid_ranges(self) -> None:
        frame = make_pose_frame()
        payload = frame[2 : 2 + POSE_PAYLOAD.size]

        self.assertEqual(len(frame), FRAME_SIZE)
        self.assertEqual(frame[:2], b"S\x00")
        self.assertEqual(POSE_PAYLOAD.size, 84)
        self.assertEqual(int.from_bytes(frame[86:90], "little"), sum(payload))
        values = POSE_PAYLOAD.unpack(payload)
        self.assertEqual(values[:5], (0, 0, 0.0, 0.0, 0.0))
        self.assertEqual(
            tuple(values[5 + index * 8 + 3] for index in range(4)),
            (1000, 1200, 900, 1100),
        )
        self.assertEqual(frame[-1:], b"E")


if __name__ == "__main__":
    unittest.main()
