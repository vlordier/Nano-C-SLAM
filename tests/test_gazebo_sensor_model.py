import math
import unittest

from sim.gazebo.nano_c_slam_gazebo.sensor_model import (
    laser_scan_ranges_mm,
    pairwise_ranges_m,
)


class LaserScanConversionTests(unittest.TestCase):
    def test_maps_four_gazebo_rays_to_front_back_left_right(self) -> None:
        ranges = laser_scan_ranges_mm(
            [1.0, 2.0, 3.0, 4.0], range_min_m=0.08, range_max_m=3.49
        )

        self.assertEqual(ranges, (3000, 1000, -1, 2000))

    def test_marks_nonfinite_and_out_of_range_readings_invalid(self) -> None:
        ranges = laser_scan_ranges_mm(
            [math.inf, 0.07, math.nan, 3.49],
            range_min_m=0.08,
            range_max_m=3.49,
        )

        self.assertEqual(ranges, (-1, -1, 3490, -1))

    def test_rejects_unexpected_scan_shape_and_bounds(self) -> None:
        with self.assertRaises(ValueError):
            laser_scan_ranges_mm([1.0] * 3, range_min_m=0.08, range_max_m=3.49)
        with self.assertRaises(ValueError):
            laser_scan_ranges_mm([1.0] * 4, range_min_m=1.0, range_max_m=1.0)


class PairwiseRangeTests(unittest.TestCase):
    def test_returns_stable_three_dimensional_pairwise_ranges(self) -> None:
        distances = pairwise_ranges_m(
            {"cf1": (0.0, 0.0, 0.0), "cf3": (3.0, 4.0, 0.0), "cf4": (0, 0, 12)}
        )

        self.assertEqual(
            distances,
            (("cf1", "cf3", 5.0), ("cf1", "cf4", 12.0), ("cf3", "cf4", 13.0)),
        )

    def test_rejects_invalid_odometry_positions(self) -> None:
        with self.assertRaises(ValueError):
            pairwise_ranges_m({"cf1": (0.0, 1.0)})
        with self.assertRaises(ValueError):
            pairwise_ranges_m({"cf1": (0.0, math.inf, 0.0)})


if __name__ == "__main__":
    unittest.main()
