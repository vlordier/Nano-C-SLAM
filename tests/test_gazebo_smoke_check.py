import unittest

from scripts.check_gazebo_macos import (
    _array_values,
    _mission_command,
    _mission_landing_altitude,
    _odometry_timestamp_ns,
    _require_odometry_progress,
    _valid_drone_name,
)


class GazeboSmokeCheckTests(unittest.TestCase):
    def test_targets_mission_at_selected_drone(self) -> None:
        command = _mission_command("/opt/ros/bin/ros2", "cf3")
        self.assertEqual(command[-2:], ["-p", "drone_name:=cf3"])
        self.assertTrue(_valid_drone_name("cf3"))
        for invalid_name in ("cf", "cf1/foo", "/cf1", "cf1;shutdown"):
            with self.subTest(invalid_name=invalid_name):
                self.assertFalse(_valid_drone_name(invalid_name))

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
        with self.assertRaisesRegex(RuntimeError, "expected.*takeoff.*hover.*land"):
            _mission_landing_altitude(1, "mission phase: takeoff\nmission phase: hover")
        with self.assertRaisesRegex(RuntimeError, "outside the ground"):
            _mission_landing_altitude(
                0,
                "mission phase: takeoff\nmission phase: hover\n"
                "mission phase: land\n"
                "completed takeoff, hover, and landing at altitude 0.5 m",
            )

    def test_accepts_safe_landing_when_drone_starts_airborne(self) -> None:
        output = """mission phase: hover at altitude 0.6 m
mission phase: land at altitude 0.6 m
completed takeoff, hover, and landing at altitude 0.08 m
"""
        self.assertEqual(_mission_landing_altitude(0, output), 0.08)

    def test_rejects_misordered_phases_and_below_ground_completion(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "expected "):
            _mission_landing_altitude(
                0,
                "mission phase: hover\nmission phase: takeoff\n"
                "mission phase: land\n"
                "completed takeoff, hover, and landing at altitude 0.05 m",
            )
        with self.assertRaisesRegex(RuntimeError, "outside the ground threshold"):
            _mission_landing_altitude(
                0,
                "mission phase: takeoff\nmission phase: hover\n"
                "mission phase: land\n"
                "completed takeoff, hover, and landing at altitude -0.2 m",
            )


if __name__ == "__main__":
    unittest.main()
