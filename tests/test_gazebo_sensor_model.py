import math
import unittest

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.sensor_model import (
    laser_scan_ranges_mm,
    odometry_stamp_seconds,
    pairwise_ranges_m,
    update_odometry_cache,
    validate_odometry_timestamps,
)


class LaserScanConversionTests(unittest.TestCase):
    def test_maps_four_gazebo_rays_to_front_back_left_right(self) -> None:
        ranges = laser_scan_ranges_mm(
            [1.0, 2.0, 3.0, 4.0],
            angle_min_rad=-3.14159,
            angle_increment_rad=(1.5708 + 3.14159) / 3,
            range_min_m=0.08,
            range_max_m=3.49,
        )

        self.assertEqual(ranges, (3000, 1000, -1, 2000))

    def test_marks_nonfinite_and_out_of_range_readings_invalid(self) -> None:
        ranges = laser_scan_ranges_mm(
            [math.inf, 0.07, math.nan, 3.49],
            angle_min_rad=-3.14159,
            angle_increment_rad=(1.5708 + 3.14159) / 3,
            range_min_m=0.08,
            range_max_m=3.49,
        )

        self.assertEqual(ranges, (-1, -1, 3490, -1))

    def test_rejects_unexpected_scan_shape_and_bounds(self) -> None:
        with self.assertRaises(ValueError):
            laser_scan_ranges_mm(
                [1.0] * 3,
                angle_min_rad=-3.14159,
                angle_increment_rad=(1.5708 + 3.14159) / 3,
                range_min_m=0.08,
                range_max_m=3.49,
            )
        with self.assertRaises(ValueError):
            laser_scan_ranges_mm(
                [1.0] * 4,
                angle_min_rad=-3.14159,
                angle_increment_rad=(1.5708 + 3.14159) / 3,
                range_min_m=1.0,
                range_max_m=1.0,
            )
        with self.assertRaisesRegex(ValueError, "angles do not match"):
            laser_scan_ranges_mm(
                [1.0] * 4,
                angle_min_rad=-1.57,
                angle_increment_rad=1.57,
                range_min_m=0.08,
                range_max_m=3.49,
            )


class PairwiseRangeTests(unittest.TestCase):
    def test_odometry_stamp_requires_normalized_ros_time_fields(self) -> None:
        self.assertEqual(odometry_stamp_seconds(1, 500_000_000), 1.5)
        for sec, nanosec in ((True, 0), (1, True), (1, -1), (1, 1_000_000_000)):
            with (
                self.subTest(sec=sec, nanosec=nanosec),
                self.assertRaisesRegex(ValueError, "malformed"),
            ):
                odometry_stamp_seconds(sec, nanosec)

    def test_invalid_odometry_does_not_mutate_cache_and_valid_sample_recovers(
        self,
    ) -> None:
        positions = {"cf1": (0.0, 0.0, 0.0)}
        stamps = {"cf1": 10.0}

        with self.assertRaisesRegex(ValueError, "finite"):
            update_odometry_cache(
                positions,
                stamps,
                drone="cf3",
                position=(math.nan, 1.0, 0.0),
                stamp_s=10.0,
                drone_names=("cf1", "cf3"),
                now_s=10.0,
            )
        self.assertEqual(positions, {"cf1": (0.0, 0.0, 0.0)})
        self.assertEqual(stamps, {"cf1": 10.0})

        update = update_odometry_cache(
            positions,
            stamps,
            drone="cf3",
            position=(3.0, 4.0, 0.0),
            stamp_s=10.0,
            drone_names=("cf1", "cf3"),
            now_s=10.0,
        )
        self.assertEqual(update.distances_m, (("cf1", "cf3", 5.0),))
        self.assertEqual(positions, {"cf1": (0.0, 0.0, 0.0)})

    def test_cache_rejects_regression_stale_future_unknown_and_float32_overflow(
        self,
    ) -> None:
        positions = {"cf1": (0.0, 0.0, 0.0)}
        stamps = {"cf1": 10.0}
        cases = (
            ("cf1", (0.0, 0.0, 0.0), 9.9, 10.0, "backwards"),
            ("cf3", (1.0, 0.0, 0.0), 9.0, 10.0, "stale"),
            ("cf3", (1.0, 0.0, 0.0), 10.2, 10.0, "ahead"),
            ("cf2", (1.0, 0.0, 0.0), 10.0, 10.0, "unknown"),
            ("cf3", (3.5e38, 0.0, 0.0), 10.0, 10.0, "Float32"),
        )
        for drone, position, stamp_s, now_s, reason in cases:
            with (
                self.subTest(reason=reason),
                self.assertRaisesRegex(ValueError, reason),
            ):
                update_odometry_cache(
                    positions,
                    stamps,
                    drone=drone,
                    position=position,
                    stamp_s=stamp_s,
                    drone_names=("cf1", "cf3"),
                    now_s=now_s,
                )
        self.assertEqual(positions, {"cf1": (0.0, 0.0, 0.0)})
        self.assertEqual(stamps, {"cf1": 10.0})

    def test_cache_withholds_ranges_until_samples_are_synchronized(self) -> None:
        first = update_odometry_cache(
            {},
            {},
            drone="cf1",
            position=(0.0, 0.0, 0.0),
            stamp_s=10.0,
            drone_names=("cf1", "cf3"),
            now_s=10.0,
        )
        second = update_odometry_cache(
            first.positions,
            first.timestamps_s,
            drone="cf3",
            position=(1.0, 0.0, 0.0),
            stamp_s=10.100001,
            drone_names=("cf1", "cf3"),
            now_s=10.100001,
        )
        self.assertIsNone(second.distances_m)
        recovered = update_odometry_cache(
            second.positions,
            second.timestamps_s,
            drone="cf1",
            position=(0.0, 0.0, 0.0),
            stamp_s=10.100001,
            drone_names=("cf1", "cf3"),
            now_s=10.100001,
        )
        self.assertEqual(recovered.distances_m, (("cf1", "cf3", 1.0),))

    def test_cache_accepts_inclusive_age_and_skew_boundaries(self) -> None:
        age_boundary = update_odometry_cache(
            {"cf1": (0.0, 0.0, 0.0)},
            {"cf1": 9.5},
            drone="cf3",
            position=(1.0, 0.0, 0.0),
            stamp_s=9.5,
            drone_names=("cf1", "cf3"),
            now_s=10.0,
        )
        skew_boundary = update_odometry_cache(
            {"cf1": (0.0, 0.0, 0.0)},
            {"cf1": 9.9},
            drone="cf3",
            position=(1.0, 0.0, 0.0),
            stamp_s=10.0,
            drone_names=("cf1", "cf3"),
            now_s=10.0,
        )
        self.assertEqual(age_boundary.distances_m, (("cf1", "cf3", 1.0),))
        self.assertEqual(skew_boundary.distances_m, (("cf1", "cf3", 1.0),))

    def test_returns_stable_three_dimensional_pairwise_ranges(self) -> None:
        distances = pairwise_ranges_m(
            {"cf1": (0.0, 0.0, 0.0), "cf3": (3.0, 4.0, 0.0), "cf4": (0, 0, 12)}
        )

        self.assertEqual(
            distances,
            (("cf1", "cf3", 5.0), ("cf1", "cf4", 12.0), ("cf3", "cf4", 13.0)),
        )

    def test_pairwise_cache_order_does_not_depend_on_callback_arrival(self) -> None:
        update = update_odometry_cache(
            {"cf4": (0.0, 0.0, 12.0), "cf1": (0.0, 0.0, 0.0)},
            {"cf4": 10.0, "cf1": 10.0},
            drone="cf3",
            position=(3.0, 4.0, 0.0),
            stamp_s=10.0,
            drone_names=("cf1", "cf3", "cf4"),
            now_s=10.0,
        )

        self.assertEqual(
            update.distances_m,
            (("cf1", "cf3", 5.0), ("cf1", "cf4", 12.0), ("cf3", "cf4", 13.0)),
        )

    def test_rejects_invalid_odometry_positions(self) -> None:
        with self.assertRaises(ValueError):
            pairwise_ranges_m({"cf1": (0.0, 1.0)})
        with self.assertRaises(ValueError):
            pairwise_ranges_m({"cf1": (0.0, math.inf, 0.0)})

    def test_accepts_recent_synchronized_odometry(self) -> None:
        validate_odometry_timestamps(
            {"cf1": 9.95, "cf3": 10.0, "cf4": 9.98}, now_s=10.0
        )

    def test_rejects_stale_skewed_or_future_odometry(self) -> None:
        cases = (
            ({"cf1": 9.4, "cf3": 10.0}, 10.0, "stale"),
            ({"cf1": 9.8, "cf3": 10.0}, 10.0, "synchronized"),
            ({"cf1": 10.0, "cf3": 10.05}, 10.0, "ahead"),
            ({"cf1": 10.0, "cf3": 10.2}, 10.0, "ahead"),
            ({"cf1": 9.9, "cf3": math.nan}, 10.0, "finite"),
        )
        for stamps, now_s, reason in cases:
            with (
                self.subTest(stamps=stamps),
                self.assertRaisesRegex(ValueError, reason),
            ):
                validate_odometry_timestamps(stamps, now_s=now_s)

    def test_rejects_empty_timestamp_set_and_invalid_tolerances(self) -> None:
        with self.assertRaisesRegex(ValueError, "no odometry"):
            validate_odometry_timestamps({}, now_s=0.0)
        with self.assertRaisesRegex(ValueError, "maximum odometry age"):
            validate_odometry_timestamps({"cf1": 1.0}, now_s=1.0, max_age_s=0.0)


if __name__ == "__main__":
    unittest.main()
