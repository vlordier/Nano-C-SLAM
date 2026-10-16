import unittest

from scripts.check_gazebo_macos import (
    _array_values,
    _mission_landing_altitude,
    _odometry_timestamp_ns,
    _require_odometry_progress,
)


class GazeboSmokeCheckTests(unittest.TestCase):
    def test_parses_ros_cli_yaml_sequence(self) -> None:
        message = """layout: {}\ndata:\n- 3401.0\n- -1.0\n"""
        self.assertEqual(_array_values(message), [3401.0, -1.0])

    def test_parses_inline_sequence(self) -> None:
        self.assertEqual(_array_values("data: [1.5, 2.5]\n"), [1.5, 2.5])

    def test_rejects_missing_or_invalid_data(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "did not contain"):
            _array_values("layout: {}\n")
        with self.assertRaisesRegex(RuntimeError, "invalid numeric"):
            _array_values("data:\n- not-a-number\n")

    def test_parses_odometry_timestamp_and_requires_progress(self) -> None:
        before = "header:\n  stamp:\n    sec: 12\n    nanosec: 400000000\n"
        after = "header:\n  stamp:\n    sec: 13\n    nanosec: 100000000\n"
        before_ns = _odometry_timestamp_ns(before)
        after_ns = _odometry_timestamp_ns(after)
        self.assertEqual(before_ns, 12_400_000_000)
        _require_odometry_progress(before_ns, after_ns)
        with self.assertRaisesRegex(RuntimeError, "did not advance"):
            _require_odometry_progress(after_ns, before_ns)

    def test_rejects_odometry_without_header_timestamp(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "header timestamp"):
            _odometry_timestamp_ns("position:\n  z: 0.2\n")

    def test_accepts_only_a_complete_mission_that_lands_near_ground(self) -> None:
        output = """cf1 mission phase: takeoff at altitude 0.1 m
cf1 mission phase: hover at altitude 0.5 m
cf1 mission phase: land at altitude 0.5 m
cf1 completed takeoff, hover, and landing at altitude 0.06 m
"""
        self.assertEqual(_mission_landing_altitude(0, output), 0.06)

    def test_rejects_incomplete_or_high_landing(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "before phases: land"):
            _mission_landing_altitude(1, "mission phase: takeoff\nmission phase: hover")
        with self.assertRaisesRegex(RuntimeError, "above the ground"):
            _mission_landing_altitude(
                0,
                "mission phase: takeoff\nmission phase: hover\n"
                "mission phase: land\n"
                "completed takeoff, hover, and landing at altitude 0.5 m",
            )


if __name__ == "__main__":
    unittest.main()
