import unittest
from typing import cast

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.telemetry_model import (
    PairingLimits,
    StampedOdometrySample,
    StampedRangeSample,
    TelemetryPairer,
)


def _range(stamp_ns: int, sequence: int, drone_id: str = "cf1") -> StampedRangeSample:
    return StampedRangeSample(drone_id, stamp_ns, sequence, (1000, 1100, 1200, -1))


def _odometry(stamp_ns: int, drone_id: str = "cf1") -> StampedOdometrySample:
    return StampedOdometrySample(drone_id, stamp_ns, 1.0, 2.0, 0.25)


class TelemetryPairingTests(unittest.TestCase):
    def test_pairs_closest_samples_once_with_inclusive_age_and_skew(self) -> None:
        pairer = TelemetryPairer(
            "cf1", PairingLimits(max_age_ns=100, max_skew_ns=10, buffer_size=4)
        )
        pairer.add_range(_range(1000, 1))
        pairer.add_odometry(_odometry(1010))

        pair = pairer.pop_pair(1100)

        self.assertIsNotNone(pair)
        assert pair is not None
        self.assertEqual(pair.skew_ns, 10)
        self.assertEqual(pair.ranges.stamp_ns, 1000)
        self.assertEqual(pair.odometry.stamp_ns, 1010)
        self.assertIsNone(pairer.pop_pair(1100))

    def test_rejects_skew_just_over_limit_and_accepts_exact_age(self) -> None:
        pairer = TelemetryPairer("cf1", PairingLimits(max_age_ns=100, max_skew_ns=10))
        pairer.add_range(_range(1000, 1))
        pairer.add_odometry(_odometry(1011))
        self.assertIsNone(pairer.pop_pair(1100))
        self.assertEqual(pairer.rejections["range_skew_exceeded"], 1)

        pairer.add_range(_range(1100, 2))
        pairer.add_odometry(_odometry(1100))
        pair = pairer.pop_pair(1200)
        self.assertIsNotNone(pair)
        assert pair is not None
        self.assertEqual(pair.skew_ns, 0)

    def test_rejects_wrong_drone_uninitialized_and_future_samples(self) -> None:
        pairer = TelemetryPairer("cf1")
        self.assertFalse(pairer.add_range(_range(1000, 1, "cf2")))
        self.assertFalse(pairer.add_range(_range(0, 2)))
        self.assertTrue(pairer.add_range(_range(1000, 3)))
        self.assertTrue(pairer.add_odometry(_odometry(1000)))
        self.assertIsNone(pairer.pop_pair(0))
        self.assertIsNone(pairer.pop_pair(900))

        self.assertEqual(pairer.rejections["wrong_drone"], 1)
        self.assertEqual(pairer.rejections["uninitialized_stamp"], 1)
        self.assertEqual(pairer.rejections["uninitialized_clock"], 1)
        self.assertEqual(pairer.rejections["future_range"], 1)
        self.assertEqual(pairer.rejections["future_odometry"], 1)

    def test_rejects_duplicate_and_out_of_order_stream_records(self) -> None:
        pairer = TelemetryPairer("cf1")
        self.assertTrue(pairer.add_range(_range(1000, 5)))
        self.assertFalse(pairer.add_range(_range(1000, 5)))
        self.assertFalse(pairer.add_range(_range(1100, 4)))
        self.assertFalse(pairer.add_range(_range(900, 6)))
        self.assertTrue(pairer.add_odometry(_odometry(1000)))
        self.assertFalse(pairer.add_odometry(_odometry(1000)))
        self.assertFalse(pairer.add_odometry(_odometry(900)))

        self.assertEqual(pairer.rejections["duplicate_range_sequence"], 1)
        self.assertEqual(pairer.rejections["out_of_order_range_sequence"], 1)
        self.assertEqual(pairer.rejections["out_of_order_range_stamp"], 1)
        self.assertEqual(pairer.rejections["duplicate_odometry_stamp"], 1)
        self.assertEqual(pairer.rejections["out_of_order_odometry_stamp"], 1)

    def test_prunes_stale_inputs_and_keeps_buffer_bounded(self) -> None:
        pairer = TelemetryPairer(
            "cf1", PairingLimits(max_age_ns=100, max_skew_ns=20, buffer_size=2)
        )
        for index in range(3):
            pairer.add_range(_range(1000 + index, index))
            pairer.add_odometry(_odometry(1000 + index))
        self.assertEqual(pairer.buffered_sample_count, 4)
        self.assertEqual(pairer.rejections["range_buffer_overflow"], 1)
        self.assertEqual(pairer.rejections["odometry_buffer_overflow"], 1)
        self.assertIsNone(pairer.pop_pair(2000))
        self.assertEqual(pairer.buffered_sample_count, 0)
        self.assertEqual(pairer.rejections["stale_range"], 2)
        self.assertEqual(pairer.rejections["stale_odometry"], 2)

    def test_simulation_pause_does_not_advance_source_age(self) -> None:
        pairer = TelemetryPairer("cf1")
        pairer.add_range(_range(1000, 1))
        self.assertIsNone(pairer.pop_pair(1050))
        pairer.add_odometry(_odometry(1000))

        pair = pairer.pop_pair(1050)

        self.assertIsNotNone(pair)
        self.assertEqual(pairer.rejections["stale_range"], 0)

    def test_backward_sim_clock_resets_stamp_fence_and_queues(self) -> None:
        pairer = TelemetryPairer("cf1")
        pairer.add_range(_range(10_000, 1))
        pairer.add_odometry(_odometry(10_000))
        pairer.pop_pair(10_000)

        pairer.add_range(_range(1_000, 2))  # rejected before clock reset observed
        self.assertEqual(pairer.rejections["out_of_order_range_stamp"], 1)
        self.assertIsNone(pairer.pop_pair(1_000))
        self.assertEqual(pairer.rejections["clock_reset"], 1)

        self.assertTrue(pairer.add_range(_range(1_000, 3)))
        self.assertTrue(pairer.add_odometry(_odometry(1_000)))
        self.assertIsNotNone(pairer.pop_pair(1_000))

    def test_rejects_nonfinite_pose_and_malformed_ranges(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            StampedOdometrySample("cf1", 10, float("inf"), 0.0, 0.0)
        with self.assertRaisesRegex(ValueError, "exactly four"):
            StampedRangeSample(
                "cf1",
                10,
                1,
                cast(tuple[int, int, int, int], (1000, 1100, 1200)),
            )
        with self.assertRaisesRegex(ValueError, "int16 format"):
            StampedRangeSample("cf1", 10, 1, (-2, 1100, 1200, 1300))

    def test_rejects_invalid_pairing_limits(self) -> None:
        for limits in (
            (0, 100, 2),
            (100, 0, 2),
            (100, 10, 0),
            (True, 10, 2),
        ):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                PairingLimits(*limits)


if __name__ == "__main__":
    unittest.main()
