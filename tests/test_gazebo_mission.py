"""Tests for the hardware-free Gazebo flight sequence."""

import unittest

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_path import (
    MappingPath,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
    MissionConfig,
    MissionPhase,
    MissionSequence,
    create_mission_config,
    validate_drone_name,
    validate_drone_names,
)


class MissionSequenceTests(unittest.TestCase):
    def test_validates_drone_name_before_constructing_topics(self) -> None:
        self.assertEqual(validate_drone_name("cf3"), "cf3")
        self.assertEqual(
            validate_drone_names(list(SUPPORTED_DRONE_NAMES)), SUPPORTED_DRONE_NAMES
        )
        for invalid_name in ("cf", "cf2", "cf1/topic", "/cf1", "cf1;stop", 1):
            with (
                self.subTest(invalid_name=invalid_name),
                self.assertRaisesRegex(ValueError, "drone_name"),
            ):
                validate_drone_name(invalid_name)

    def test_rejects_invalid_or_duplicate_sensor_bridge_drones(self) -> None:
        for invalid_names in ([], ["cf1", "cf1"], ["cf1", "cf2"], "cf1"):
            with (
                self.subTest(invalid_names=invalid_names),
                self.assertRaises(ValueError),
            ):
                validate_drone_names(invalid_names)

    def test_builds_config_from_numeric_ros_parameters(self) -> None:
        self.assertEqual(
            create_mission_config(0.7, 2),
            MissionConfig(hover_height_m=0.7, hover_duration_s=2.0),
        )
        for invalid_values in (("0.5", 2.0), (0.5, True), (float("nan"), 2.0)):
            with (
                self.subTest(invalid_values=invalid_values),
                self.assertRaises(ValueError),
            ):
                create_mission_config(*invalid_values)

    def test_takeoff_hover_and_land_follow_measured_altitude(self) -> None:
        mission = MissionSequence(MissionConfig(hover_duration_s=2.0))

        self.assertEqual(mission.step(0.0, None), 0.0)
        self.assertIs(mission.phase, MissionPhase.WAITING)
        self.assertEqual(mission.step(0.1, 0.0, 0.1), 0.5)
        self.assertIs(mission.phase, MissionPhase.TAKEOFF)
        self.assertEqual(mission.step(1.0, 0.5, 1.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.HOVER)
        self.assertEqual(mission.step(2.9, 0.5, 2.9), 0.0)
        self.assertEqual(mission.step(3.0, 0.5, 3.0), -0.5)
        self.assertIs(mission.phase, MissionPhase.LAND)
        self.assertEqual(mission.step(3.1, 0.1, 3.1), 0.0)
        self.assertIs(mission.phase, MissionPhase.COMPLETE)
        self.assertEqual(mission.step(3.2, 0.1, 3.2), 0.0)

    def test_config_rejects_unreachable_landing_height(self) -> None:
        with self.assertRaisesRegex(ValueError, "below hover_height_m"):
            MissionConfig(hover_height_m=0.5, landing_height_m=0.5)

    def test_non_finite_altitude_fails_safe_with_zero_command(self) -> None:
        mission = MissionSequence()
        self.assertEqual(mission.step(0.0, float("nan"), 0.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "odometry altitude is not finite")

    def test_backward_simulation_time_fails_safe(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.step(2.0, 0.0, 2.0), 0.5)
        self.assertEqual(mission.step(1.9, 0.0, 1.9), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "simulation time moved backwards")

    def test_non_finite_simulation_time_fails_safe(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.step(float("nan"), 0.0, 0.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "simulation time is not finite")

    def test_stale_odometry_fails_safe(self) -> None:
        mission = MissionSequence(MissionConfig(odometry_timeout_s=0.2))

        self.assertEqual(mission.step(0.0, 0.0, 0.0), 0.5)
        self.assertEqual(mission.step(0.1, 0.0, 0.0), 0.5)
        self.assertEqual(mission.step(0.21, 0.0, 0.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "odometry data is stale")

    def test_odometry_timestamp_regression_fails_safe(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.step(0.1, 0.0, 0.1), 0.5)
        self.assertEqual(mission.step(0.2, 0.0, 0.05), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "odometry timestamp moved backwards")

    def test_active_mission_fails_if_altitude_disappears(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.step(0.0, 0.0, 0.0), 0.5)
        self.assertEqual(mission.step(0.1, None), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "odometry altitude is unavailable")

    def test_takeoff_timeout_stops_with_zero_command(self) -> None:
        mission = MissionSequence(MissionConfig(phase_timeout_s=1.0))

        self.assertEqual(mission.step(0.0, 0.0, 0.0), 0.5)
        self.assertEqual(mission.step(1.0, 0.0, 1.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "takeoff phase timed out")

    def test_external_path_failure_is_terminal_and_zero_command(self) -> None:
        mission = MissionSequence()

        self.assertEqual(mission.fail("mapping waypoint timed out"), 0.0)

        self.assertIs(mission.phase, MissionPhase.FAILED)
        self.assertEqual(mission.failure_reason, "mapping waypoint timed out")
        self.assertEqual(mission.step(1.0, 0.0, 1.0), 0.0)

    def test_already_airborne_mission_moves_to_hover_and_landing(self) -> None:
        mission = MissionSequence(MissionConfig(hover_duration_s=0.5))

        self.assertEqual(mission.step(2.0, 0.6, 2.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.HOVER)
        self.assertEqual(mission.step(2.5, 0.6, 2.5), -0.5)
        self.assertIs(mission.phase, MissionPhase.LAND)

    def test_mapping_path_holds_hover_timer_until_route_finishes(self) -> None:
        mission = MissionSequence(MissionConfig(hover_duration_s=2.0))
        mission.step(0.0, 0.0, 0.0)
        mission.step(0.1, 0.5, 0.1)

        self.assertEqual(mission.step(1.0, 0.5, 1.0, hold_hover=True), 0.0)
        self.assertEqual(mission.step(5.0, 0.5, 5.0, hold_hover=True), 0.0)

        self.assertEqual(mission.step(6.9, 0.5, 6.9), 0.0)
        self.assertIs(mission.phase, MissionPhase.HOVER)
        self.assertEqual(mission.step(7.0, 0.5, 7.0), -0.5)


class MappingPathTests(unittest.TestCase):
    def test_tracks_square_with_bounded_world_frame_commands(self) -> None:
        path = MappingPath(2.0, -1.0, 0.0)

        command = path.step(0.1, 2.0, -1.0)
        self.assertAlmostEqual(command[0], 0.2)
        self.assertAlmostEqual(command[1], 0.0)

        self.assertEqual(path.step(1.0, 2.25, -1.0), (0.0, 0.0))
        self.assertEqual(path.waypoint_index, 1)
        command = path.step(1.1, 2.25, -1.0)
        self.assertAlmostEqual(command[0], 0.0)
        self.assertAlmostEqual(command[1], 0.2)

    def test_completes_only_after_all_waypoints_are_reached(self) -> None:
        offsets = ((0.1, 0.0),)
        path = MappingPath(0.0, 0.0, 0.0, offsets_m=offsets)

        path.step(0.1, 0.0, 0.0)
        self.assertFalse(path.complete)
        self.assertEqual(path.step(0.2, 0.1, 0.0), (0.0, 0.0))
        self.assertTrue(path.complete)
        self.assertEqual(path.step(0.3, 10.0, 10.0), (0.0, 0.0))

    def test_fails_safe_on_timeout_regression_and_non_finite_odometry(self) -> None:
        timed_out = MappingPath(0.0, 0.0, 0.0, waypoint_timeout_s=1.0)
        self.assertEqual(timed_out.step(1.0, 0.0, 0.0), (0.0, 0.0))
        self.assertIn("timed out", timed_out.failure_reason or "")

        regressed = MappingPath(0.0, 0.0, 0.0)
        regressed.step(1.0, 0.0, 0.0)
        self.assertEqual(regressed.step(0.5, 0.0, 0.0), (0.0, 0.0))
        self.assertIn("backwards", regressed.failure_reason or "")

        invalid = MappingPath(0.0, 0.0, 0.0)
        self.assertEqual(invalid.step(0.1, float("nan"), 0.0), (0.0, 0.0))
        self.assertIn("non-finite", invalid.failure_reason or "")

    def test_rejects_unbounded_or_invalid_paths(self) -> None:
        for options in (
            {"offsets_m": ()},
            {"offsets_m": ((0.6, 0.0),)},
            {"waypoint_timeout_s": float("inf")},
        ):
            with self.subTest(options=options), self.assertRaises(ValueError):
                MappingPath(0.0, 0.0, 0.0, **options)


if __name__ == "__main__":
    unittest.main()
