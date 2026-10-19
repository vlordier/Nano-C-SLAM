from __future__ import annotations

import unittest
from dataclasses import replace

from scripts.check_crazysim_flight import validate_capture
from test_support.crazysim_flight_fixtures import passing_capture


class CrazySimFlightCheckerTests(unittest.TestCase):
    def _validate(self, **overrides: object) -> list[str]:
        capture = replace(passing_capture(), **overrides)
        _, errors = validate_capture(
            odometry_text=capture.odometry,
            imu_text=capture.imu,
            motor_text=capture.motors,
            firmware_log=capture.firmware_log,
            plugin_log=capture.plugin_log,
            negative_control_odometry_text=capture.negative_odometry,
            negative_control_motor_text=capture.negative_motors,
            negative_control_log=capture.negative_log,
        )
        return errors

    def test_realistic_takeoff_hover_land_and_disconnected_control_pass(self) -> None:
        capture = passing_capture()
        summary, errors = validate_capture(
            odometry_text=capture.odometry,
            imu_text=capture.imu,
            motor_text=capture.motors,
            firmware_log=capture.firmware_log,
            plugin_log=capture.plugin_log,
            negative_control_odometry_text=capture.negative_odometry,
            negative_control_motor_text=capture.negative_motors,
            negative_control_log=capture.negative_log,
        )

        self.assertEqual(errors, [])
        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertAlmostEqual(summary.max_altitude_m, 2.4)
        self.assertGreaterEqual(summary.hover_duration_s, 5.0)
        self.assertEqual(summary.negative_control_drift_m, 0.0)

    def test_nonfinite_odometry_is_rejected(self) -> None:
        capture = passing_capture()
        errors = self._validate(
            odometry=capture.odometry.replace("z: 2.040000", "z: nan", 1)
        )

        self.assertIn(
            "positive run contains NaN or infinite state/actuator values", errors
        )

    def test_rate_warning_crash_and_nonzero_exit_are_rejected(self) -> None:
        capture = passing_capture()
        errors = self._validate(
            firmware_log=capture.firmware_log
            + "\nSTAB: WARNING: stabilizer loop rate is off (823)\n"
            + "SUP: Crashed, recovery required",
        )
        _, exit_errors = validate_capture(
            odometry_text=capture.odometry,
            imu_text=capture.imu,
            motor_text=capture.motors,
            firmware_log=capture.firmware_log,
            plugin_log=capture.plugin_log,
            negative_control_odometry_text=capture.negative_odometry,
            negative_control_motor_text=capture.negative_motors,
            negative_control_log=capture.negative_log,
            firmware_exit_code=1,
        )

        self.assertIn("firmware reported estimator/rate/crash failure", errors)
        self.assertIn("firmware exited with status 1", exit_errors)

    def test_stale_imu_and_missing_landing_command_are_rejected(self) -> None:
        capture = passing_capture()
        stale_imu = "\n\n".join(
            line
            for index, line in enumerate(capture.imu.split("\n\n"))
            if index % 10 == 0
        )
        errors = self._validate(
            imu=stale_imu,
            firmware_log=capture.firmware_log.replace(
                "TRACE_HL_RX header=8c size=15 data=08 00 00 00", ""
            ),
        )

        self.assertTrue(any("IMU: stale sample gap" in error for error in errors))
        self.assertIn("firmware did not receive LAND_2", errors)

    def test_negative_control_must_be_grounded_and_explicitly_disconnected(
        self,
    ) -> None:
        capture = passing_capture()
        first_record, remainder = capture.negative_odometry.split("\n\n", maxsplit=1)
        raised_control = (
            first_record + "\n\n" + remainder.replace("z: 2.040000", "z: 2.240000", 1)
        )
        errors = self._validate(
            negative_odometry=raised_control,
            negative_log="",
        )

        self.assertIn("disconnected-actuator control rose more than 0.10 m", errors)
        self.assertIn(
            "negative control lacks explicit disconnected-actuator marker", errors
        )

    def test_zero_motor_flight_is_rejected(self) -> None:
        capture = passing_capture()
        errors = self._validate(
            motors=capture.motors.replace("velocity: 1800", "velocity: 0")
        )

        self.assertIn("positive run has no nonzero actuator command", errors)


if __name__ == "__main__":
    unittest.main()
