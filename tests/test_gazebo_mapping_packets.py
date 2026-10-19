"""Tests for Gazebo telemetry packetization into the GAP9 production protocol."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.firmware_protocol import (
    OPTIMIZE_CMD,
    POSE_CMD,
    POSE_PAYLOAD,
    SCAN_CMD,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_model import (
    MappingSample,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_packets import (
    MappingPacketEncoder,
    encode_mapping_samples,
    mapping_sample_from_pair,
    samples_from_run_bundle,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.telemetry_model import (
    PairedTelemetry,
    StampedOdometrySample,
    StampedRangeSample,
)


def _sample(sample_id: int, *, timestamp_ms: int | None = None) -> MappingSample:
    return MappingSample(
        sample_id=sample_id,
        timestamp_ms=(sample_id + 1) * 100 if timestamp_ms is None else timestamp_ms,
        x_m=0.02 * sample_id,
        y_m=-0.01 * sample_id,
        yaw_rad=0.1 * sample_id,
        ranges_mm=(700, -1, 1300, 1800),
    )


class MappingPacketTests(unittest.TestCase):
    def test_acceptance_run_bundle_becomes_bounded_firmware_stream(self) -> None:
        run_path = (
            Path(__file__).resolve().parents[1] / "artifacts/gazebo_mapping_sitl/"
            "strict_host_mapping_acceptance_20261019/attempt_01/run.json"
        )
        bundle = json.loads(run_path.read_text(encoding="utf-8"))

        run_id, samples = samples_from_run_bundle(bundle)
        packets = encode_mapping_samples(samples)

        self.assertEqual(run_id, "15a06f08-657e-4012-922e-4ca35f7dda22")
        self.assertEqual(len(samples), 60)
        self.assertEqual(len(packets), 64)
        self.assertEqual(
            [packet.pose_id for packet in packets if packet.command == SCAN_CMD],
            [0, 20, 40],
        )
        self.assertEqual(packets[-1].command, OPTIMIZE_CMD)
        golden_root = (
            Path(__file__).resolve().parent / "fixtures/gazebo_firmware_packets"
        )
        self.assertEqual(
            packets[0].frame,
            bytes.fromhex((golden_root / "pose_cf1_sample0.hex").read_text()),
        )
        first_scan = next(packet for packet in packets if packet.command == SCAN_CMD)
        self.assertEqual(
            first_scan.frame,
            bytes.fromhex((golden_root / "scan_cf1_pose0.hex").read_text()),
        )

    def test_rejects_unbound_or_incomplete_capture_bundles(self) -> None:
        sample_record = "0 100 0.0 0.0 0.0 700 800 900 1000"
        run = {
            "run_id": "run-a",
            "drone": "cf1",
            "mission_terminal": {
                "run_id": "run-a",
                "drone_name": "cf1",
                "phase": "COMPLETE",
                "mapping_path_complete": True,
            },
            "capture_report": {
                "run_id": "run-a",
                "drone": "cf1",
                "evidence_class": "host-execution-of-owned-gap9-mapping-core",
                "input_sample_count": 1,
                "input_samples": [sample_record],
                "pairing_limits": {
                    "maximum_age_ns": 500_000_000,
                    "maximum_skew_ns": 100_000_000,
                },
                "input_pair_provenance": [
                    {
                        "drone_id": "cf1",
                        "range_age_ns": 100_000_000,
                        "odometry_age_ns": 100_000_000,
                        "pair_skew_ns": 0,
                        "odometry_stamp_ns": 100_000_000,
                        "range_stamp_ns": 100_000_000,
                        "range_sequence": 10,
                    }
                ],
            },
        }
        samples_from_run_bundle(run, expected_run_id="run-a")

        invalid_bundles = (
            ({**run, "run_id": "run-b"}, "identities must match"),
            (
                {
                    **run,
                    "mission_terminal": {**run["mission_terminal"], "phase": "FAILED"},
                },
                "identities must match",
            ),
            (
                {
                    **run,
                    "capture_report": {
                        **run["capture_report"],
                        "input_sample_count": 2,
                    },
                },
                "count does not match",
            ),
            (
                {
                    **run,
                    "capture_report": {
                        **run["capture_report"],
                        "input_samples": ["bad"],
                    },
                },
                "nine fields",
            ),
            (
                {
                    **run,
                    "capture_report": {
                        **run["capture_report"],
                        "input_pair_provenance": [
                            {
                                **run["capture_report"]["input_pair_provenance"][0],
                                "odometry_age_ns": 600_000_000,
                            }
                        ],
                    },
                },
                "stale or mismatched",
            ),
            (
                {
                    **run,
                    "capture_report": {
                        **run["capture_report"],
                        "input_pair_provenance": [
                            {
                                **run["capture_report"]["input_pair_provenance"][0],
                                "range_stamp_ns": 99_000_000,
                            }
                        ],
                    },
                },
                "stale or mismatched",
            ),
            (
                {
                    **run,
                    "capture_report": {
                        **run["capture_report"],
                        "input_pair_provenance": [
                            {
                                **run["capture_report"]["input_pair_provenance"][0],
                                "drone_id": "cf2",
                            }
                        ],
                    },
                },
                "stale or mismatched",
            ),
        )
        for invalid_bundle, message in invalid_bundles:
            with (
                self.subTest(message=message),
                self.assertRaisesRegex(ValueError, message),
            ):
                samples_from_run_bundle(invalid_bundle)
        with self.assertRaisesRegex(ValueError, "requested run_id"):
            samples_from_run_bundle(run, expected_run_id="run-b")

    def test_converts_source_stamped_pair_to_firmware_pose_fields(self) -> None:
        pair = PairedTelemetry(
            ranges=StampedRangeSample("cf1", 1_200_000_000, 12, (700, -1, 1300, 1800)),
            odometry=StampedOdometrySample("cf1", 1_250_000_000, 1.25, -0.5, 0.75),
            skew_ns=50_000_000,
        )

        sample = mapping_sample_from_pair(pair, pose_id=4)

        self.assertEqual(sample.sample_id, 4)
        self.assertEqual(sample.timestamp_ms, 1250)
        self.assertEqual(sample.source_stamp_ns, 1_250_000_000)
        self.assertEqual((sample.x_m, sample.y_m, sample.yaw_rad), (1.25, -0.5, 0.75))
        self.assertEqual(sample.ranges_mm, (700, -1, 1300, 1800))

    def test_rejects_cross_drone_or_excessively_skewed_pairs(self) -> None:
        ranges = StampedRangeSample("cf1", 1_000_000_000, 1, (1, 2, 3, 4))
        odometry = StampedOdometrySample("cf2", 1_000_000_000, 0.0, 0.0, 0.0)

        with self.assertRaisesRegex(ValueError, "different drones"):
            mapping_sample_from_pair(PairedTelemetry(ranges, odometry, 0), pose_id=0)

        odometry = StampedOdometrySample("cf1", 1_200_000_000, 0.0, 0.0, 0.0)
        with self.assertRaisesRegex(ValueError, "skew limit"):
            mapping_sample_from_pair(
                PairedTelemetry(ranges, odometry, 200_000_000), pose_id=0
            )

    def test_encodes_pose_samples_and_full_scan_windows_as_guarded_frames(self) -> None:
        samples = [_sample(sample_id) for sample_id in range(60)]

        packets = encode_mapping_samples(samples)

        self.assertEqual(len(packets), 64)
        self.assertEqual([packet.sequence for packet in packets], list(range(64)))
        self.assertEqual([packet.command for packet in packets].count(POSE_CMD), 60)
        scan_packets = [packet for packet in packets if packet.command == SCAN_CMD]
        self.assertEqual([packet.pose_id for packet in scan_packets], [0, 20, 40])
        self.assertEqual(packets[-1].command, OPTIMIZE_CMD)
        for packet in packets:
            self.assertEqual(len(packet.frame), 97)
            self.assertEqual(packet.frame[0], ord("S"))
            self.assertEqual(packet.frame[-1], ord("E"))

        first_pose = packets[0]
        pose_payload = first_pose.frame[2 : 2 + POSE_PAYLOAD.size]
        expected_tof = [-1] * 32
        expected_tof[3] = 700
        expected_tof[19] = 1300
        expected_tof[27] = 1800
        self.assertEqual(
            POSE_PAYLOAD.unpack(pose_payload),
            (0, 100, 0.0, 0.0, 0.0, *expected_tof),
        )
        self.assertEqual(
            int.from_bytes(first_pose.frame[86:90], "little"), sum(pose_payload)
        )

    def test_incremental_encoder_matches_bounded_batch_encoding(self) -> None:
        samples = [_sample(sample_id) for sample_id in range(60)]
        encoder = MappingPacketEncoder()
        streamed = [packet for sample in samples for packet in encoder.push(sample)]
        streamed.extend(encoder.finish())

        self.assertEqual(tuple(streamed), encode_mapping_samples(samples))
        with self.assertRaisesRegex(ValueError, "already complete"):
            encoder.finish()
        with self.assertRaisesRegex(ValueError, "after packet stream completion"):
            encoder.push(_sample(60))

    def test_incremental_encoder_rejects_gaps_and_nonmonotonic_timestamps(self) -> None:
        encoder = MappingPacketEncoder()
        with self.assertRaisesRegex(ValueError, "contiguous"):
            encoder.push(_sample(1))
        encoder.push(_sample(0, timestamp_ms=100))
        with self.assertRaisesRegex(ValueError, "increase strictly"):
            encoder.push(_sample(1, timestamp_ms=100))

    def test_rejects_invalid_sample_order_timestamps_and_unbounded_inputs(self) -> None:
        cases = (
            ([_sample(1)], "contiguous"),
            ([_sample(0), _sample(1, timestamp_ms=100)], "increase strictly"),
            ([_sample(sample_id) for sample_id in range(261)], "1..260"),
        )
        for samples, message in cases:
            with (
                self.subTest(message=message),
                self.assertRaisesRegex(ValueError, message),
            ):
                encode_mapping_samples(samples)

        invalid_pose = MappingSample(0, 100, float("nan"), 0, 0, (1, 2, 3, 4))
        with self.assertRaisesRegex(ValueError, "finite"):
            encode_mapping_samples([invalid_pose])

    def test_rejects_invalid_scan_window_size(self) -> None:
        for window_size in (0, 261, True):
            with self.subTest(window_size=window_size), self.assertRaises(ValueError):
                encode_mapping_samples([_sample(0)], scan_window_size=window_size)


if __name__ == "__main__":
    unittest.main()
