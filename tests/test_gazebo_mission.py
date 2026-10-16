"""Tests for the hardware-free Gazebo flight sequence."""

import unittest

from sim.gazebo.nano_c_slam_gazebo.mission_model import (
    MissionConfig,
    MissionPhase,
    MissionSequence,
    create_mission_config,
    validate_drone_name,
)


class MissionSequenceTests(unittest.TestCase):
    def test_validates_drone_name_before_constructing_topics(self) -> None:
        self.assertEqual(validate_drone_name("cf12"), "cf12")
        for invalid_name in ("cf", "cf1/topic", "/cf1", "cf1;stop", 1):
            with (
                self.subTest(invalid_name=invalid_name),
                self.assertRaisesRegex(ValueError, "drone_name"),
            ):
                validate_drone_name(invalid_name)

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

    def test_already_airborne_mission_moves_to_hover_and_landing(self) -> None:
        mission = MissionSequence(MissionConfig(hover_duration_s=0.5))

        self.assertEqual(mission.step(2.0, 0.6, 2.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.HOVER)
        self.assertEqual(mission.step(2.5, 0.6, 2.5), -0.5)
        self.assertIs(mission.phase, MissionPhase.LAND)


if __name__ == "__main__":
    unittest.main()
