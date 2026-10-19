"""Tests for the pure GVSOC SPI probe frame builder."""

from __future__ import annotations

import unittest

from scripts.gap9_gvsoc_spi_probe_protocol import (
    FRAME_SIZE,
    OPTIMIZE_CMD,
    POSE_PAYLOAD,
    SCAN_CMD,
    make_mapping_trace,
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

    def test_mapping_trace_matches_firmware_sequence_and_checksums(self) -> None:
        frames = make_mapping_trace()

        self.assertEqual(len(frames), 66)
        self.assertEqual([frame[1] for frame in frames].count(0), 61)
        self.assertEqual([frame[1] for frame in frames].count(SCAN_CMD), 4)
        self.assertEqual(frames[-1][1], OPTIMIZE_CMD)
        scans = [frame for frame in frames if frame[1] == SCAN_CMD]
        self.assertEqual(
            [int.from_bytes(frame[2:4], "little") for frame in scans], [0, 20, 40, 40]
        )

        invalid_pose = frames[0]
        valid_poses = [frame for frame in frames[1:] if frame[1] == 0]
        self.assertEqual(invalid_pose[:2], b"S\x00")
        invalid_payload = invalid_pose[2 : 2 + POSE_PAYLOAD.size]
        invalid_checksum = int.from_bytes(invalid_pose[86:90], "little")
        self.assertNotEqual(invalid_checksum, sum(invalid_payload))
        for pose_id, frame in enumerate(valid_poses):
            payload = frame[2 : 2 + POSE_PAYLOAD.size]
            self.assertEqual(POSE_PAYLOAD.unpack(payload)[0], pose_id)
            self.assertEqual(int.from_bytes(frame[86:90], "little"), sum(payload))
            self.assertEqual(frame[-1:], b"E")

        for frame in scans:
            self.assertEqual(int.from_bytes(frame[4:6], "little"), frame[2] + frame[3])

    def test_max_capacity_trace_stops_at_firmware_pose_limit(self) -> None:
        frames = make_mapping_trace(260)

        self.assertEqual(len(frames), 276)
        self.assertEqual([frame[1] for frame in frames].count(0), 261)
        self.assertEqual([frame[1] for frame in frames].count(SCAN_CMD), 14)
        self.assertEqual(frames[-1][1], OPTIMIZE_CMD)
        poses = [frame for frame in frames if frame[1] == 0][1:]
        self.assertEqual(
            [
                POSE_PAYLOAD.unpack(frame[2 : 2 + POSE_PAYLOAD.size])[0]
                for frame in poses
            ],
            list(range(260)),
        )
        self.assertEqual(
            [
                int.from_bytes(frame[2:4], "little")
                for frame in frames
                if frame[1] == SCAN_CMD
            ],
            [*range(0, 260, 20), 240],
        )

    def test_capacity_trace_exercises_scan_anchor_storage_limit(self) -> None:
        frames = make_mapping_trace(260, scan_interval=2, stationary=True)
        scans = [frame for frame in frames if frame[1] == SCAN_CMD]
        poses = [frame for frame in frames if frame[1] == 0][1:]

        self.assertEqual(len(frames), 393)
        self.assertEqual(len(scans), 131)
        self.assertEqual(
            [int.from_bytes(frame[2:4], "little") for frame in scans],
            [*range(0, 260, 2), 258],
        )
        observations = [
            POSE_PAYLOAD.unpack(frame[2 : 2 + POSE_PAYLOAD.size])[2:] for frame in poses
        ]
        self.assertTrue(
            all(observation == observations[0] for observation in observations)
        )

    def test_mapping_trace_rejects_pose_count_outside_firmware_capacity(self) -> None:
        for pose_count in (0, 261):
            with self.subTest(pose_count=pose_count), self.assertRaises(ValueError):
                make_mapping_trace(pose_count)


if __name__ == "__main__":
    unittest.main()
