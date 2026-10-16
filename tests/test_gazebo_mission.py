"""Tests for the hardware-free Gazebo flight sequence."""

import unittest

from sim.gazebo.nano_c_slam_gazebo.mission_model import (
    MissionConfig,
    MissionPhase,
    MissionSequence,
)


class MissionSequenceTests(unittest.TestCase):
    def test_takeoff_hover_and_land_follow_measured_altitude(self) -> None:
        mission = MissionSequence(MissionConfig(hover_duration_s=2.0))

        self.assertEqual(mission.step(0.0, None), 0.0)
        self.assertIs(mission.phase, MissionPhase.WAITING)
        self.assertEqual(mission.step(0.1, 0.0), 0.5)
        self.assertIs(mission.phase, MissionPhase.TAKEOFF)
        self.assertEqual(mission.step(1.0, 0.5), 0.0)
        self.assertIs(mission.phase, MissionPhase.HOVER)
        self.assertEqual(mission.step(2.9, 0.5), 0.0)
        self.assertEqual(mission.step(3.0, 0.5), -0.5)
        self.assertIs(mission.phase, MissionPhase.LAND)
        self.assertEqual(mission.step(3.1, 0.08), 0.0)
        self.assertIs(mission.phase, MissionPhase.COMPLETE)
        self.assertEqual(mission.step(3.2, 0.08), 0.0)

    def test_config_rejects_unreachable_landing_height(self) -> None:
        with self.assertRaisesRegex(ValueError, "below hover_height_m"):
            MissionConfig(hover_height_m=0.5, landing_height_m=0.5)

    def test_non_finite_altitude_is_rejected(self) -> None:
        mission = MissionSequence()
        with self.assertRaisesRegex(ValueError, "altitude_m must be finite"):
            mission.step(0.0, float("nan"))

    def test_takeoff_timeout_stops_with_zero_command(self) -> None:
        mission = MissionSequence(MissionConfig(phase_timeout_s=1.0))

        self.assertEqual(mission.step(0.0, 0.0), 0.5)
        self.assertEqual(mission.step(1.0, 0.0), 0.0)
        self.assertIs(mission.phase, MissionPhase.FAILED)


if __name__ == "__main__":
    unittest.main()
