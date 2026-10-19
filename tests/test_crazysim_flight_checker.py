from __future__ import annotations

import json
import unittest
from dataclasses import replace

from scripts.check_crazysim_flight import (
    parse_imu_timestamps,
    parse_imu_vectors,
    parse_motor_components,
    parse_odometry,
    parse_world_stats,
    validate_capture,
)
from test_support.crazysim_flight_fixtures import passing_capture


class CrazySimFlightCheckerTests(unittest.TestCase):
    def _validate(self, **overrides: object) -> list[str]:
        capture = replace(passing_capture(), **overrides)
        _, errors = validate_capture(
            odometry_text=capture.odometry,
            imu_text=capture.imu,
            world_stats_text=capture.world_stats,
            motor_text=capture.motors,
            firmware_log=capture.firmware_log,
            plugin_log=capture.plugin_log,
            negative_control_odometry_text=capture.negative_odometry,
            negative_control_motor_text=capture.negative_motors,
            negative_control_log=capture.negative_log,
            firmware_process_count=capture.firmware_process_count,
            negative_control_firmware_process_count=(
                capture.negative_firmware_process_count
            ),
        )
        return errors

    def test_gazebo_json_topic_captures_parse_like_protobuf_text(self) -> None:
        odometry = json.dumps(
            {
                "header": {"stamp": {"sec": "12", "nsec": 500_000_000}},
                "pose": {
                    "position": {"x": 1, "y": 2, "z": 2.4},
                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                },
                "twist": {
                    "linear": {"x": 0.1, "y": 0.2, "z": 0.3},
                    "angular": {"x": 0.01, "y": 0.02, "z": 0.03},
                },
            }
        )
        imu = json.dumps(
            {
                "header": {"stamp": {"sec": "12", "nsec": 750_000_000}},
                "linearAcceleration": {"x": 0, "y": 0, "z": 9.81},
                "angularVelocity": {"x": 0.1, "y": 0.2, "z": 0.3},
            }
        )
        motors = json.dumps({"velocity": [1000, 1100, 1200, 1300]})
        world_stats = json.dumps(
            {
                "simTime": {"sec": "12", "nsec": 500_000_000},
                "realTime": {"sec": "12", "nsec": 250_000_000},
                "realTimeFactor": 1.1,
            }
        )

        pose_sample = parse_odometry(odometry)[0]
        self.assertEqual(pose_sample.time_s, 12.5)
        self.assertEqual(pose_sample.position, (1.0, 2.0, 2.4))
        self.assertEqual(pose_sample.angular_velocity, (0.01, 0.02, 0.03))
        self.assertEqual(parse_imu_timestamps(imu), [12.75])
        self.assertEqual(parse_imu_vectors(imu), [((0.0, 0.0, 9.81), (0.1, 0.2, 0.3))])
        self.assertEqual(
            parse_motor_components(motors), [1000.0, 1100.0, 1200.0, 1300.0]
        )
        self.assertEqual(
            parse_world_stats(world_stats)[0].real_time_factor,
            1.1,
        )
        subsecond_odometry = json.dumps(
            {
                **json.loads(odometry),
                "header": {"stamp": {"nsec": "5000000"}},
            }
        )
        self.assertEqual(parse_odometry(subsecond_odometry)[0].time_s, 0.005)

    def test_protobuf_text_timestamps_default_omitted_nanoseconds_to_zero(self) -> None:
        odometry = """header {
  stamp {
    sec: 42
  }
}
pose {
  position {
    z: 2.5
  }
}
"""
        imu = """header {
  stamp {
    sec: 42
  }
}
linear_acceleration {
  z: 9.81
}
"""
        world_stats = """sim_time {
  sec: 42
}
real_time {
  sec: 42
}
iterations: 42000
real_time_factor: 1.0
"""

        self.assertEqual(parse_odometry(odometry)[0].time_s, 42.0)
        self.assertEqual(parse_imu_timestamps(imu), [42.0])
        self.assertEqual(parse_world_stats(world_stats)[0].sim_time_s, 42.0)
        self.assertEqual(parse_world_stats(world_stats)[0].real_time_s, 42.0)

    def test_realistic_takeoff_hover_land_and_disconnected_control_pass(self) -> None:
        capture = passing_capture()
        summary, errors = validate_capture(
            odometry_text=capture.odometry,
            imu_text=capture.imu,
            world_stats_text=capture.world_stats,
            motor_text=capture.motors,
            firmware_log=capture.firmware_log,
            plugin_log=capture.plugin_log,
            negative_control_odometry_text=capture.negative_odometry,
            negative_control_motor_text=capture.negative_motors,
            negative_control_log=capture.negative_log,
            firmware_process_count=capture.firmware_process_count,
            negative_control_firmware_process_count=(
                capture.negative_firmware_process_count
            ),
        )

        self.assertEqual(errors, [])
        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertAlmostEqual(summary.max_altitude_m, 2.4)
        self.assertGreaterEqual(summary.hover_duration_s, 5.0)
        self.assertEqual(summary.negative_control_drift_m, 0.0)
        self.assertAlmostEqual(summary.aggregate_real_time_factor, 1.0)

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
            world_stats_text=capture.world_stats,
            motor_text=capture.motors,
            firmware_log=capture.firmware_log,
            plugin_log=capture.plugin_log,
            negative_control_odometry_text=capture.negative_odometry,
            negative_control_motor_text=capture.negative_motors,
            negative_control_log=capture.negative_log,
            firmware_exit_code=1,
            firmware_process_count=capture.firmware_process_count,
            negative_control_firmware_process_count=(
                capture.negative_firmware_process_count
            ),
        )

        self.assertIn("firmware reported estimator/rate/crash failure", errors)
        self.assertIn("firmware exited with status 1", exit_errors)

    def test_slow_and_incompletely_observed_simulation_are_rejected(self) -> None:
        slow_stats = "".join(
            f"sim_time {{ sec: {second} nsec: 0 }}\n"
            f"real_time {{ sec: {second * 2} nsec: 0 }}\n"
            f"iterations: {second * 1000}\nreal_time_factor: 0.5\n\n"
            for second in range(10)
        )
        slow_errors = self._validate(world_stats=slow_stats)
        self.assertTrue(
            any(
                "aggregate Gazebo real-time factor is 0.500" in error
                for error in slow_errors
            )
        )

        short_stats = "".join(
            f"sim_time {{ sec: {second} nsec: 0 }}\n"
            f"real_time {{ sec: {second} nsec: 0 }}\n"
            f"iterations: {second * 1000}\nreal_time_factor: 1.0\n\n"
            for second in range(1, 9)
        )
        coverage_errors = self._validate(world_stats=short_stats)
        self.assertTrue(any("cover only" in error for error in coverage_errors))

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

    def test_stale_odometry_is_rejected_at_registered_20ms_limit(self) -> None:
        capture = passing_capture()
        samples = capture.odometry.split("\n\n")
        # Remove four consecutive nominal 200-Hz messages: the resulting
        # 25-ms interval must fail the strict odometry freshness requirement.
        stale_odometry = "\n\n".join(samples[:1] + samples[6:])
        errors = self._validate(odometry=stale_odometry)

        self.assertTrue(any("odometry: stale sample gap" in error for error in errors))

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

    def test_zero_or_multiple_firmware_instances_are_rejected(self) -> None:
        capture = passing_capture()
        errors_without_firmware = self._validate(
            firmware_log=capture.firmware_log.replace(
                "TRACE_CRTP_RX_STARTED link=0x1234\n", ""
            )
        )
        errors_with_duplicate_firmware = self._validate(
            firmware_log=capture.firmware_log + "\nTRACE_CRTP_RX_STARTED link=0x5678"
        )

        self.assertIn(
            "positive run log must contain exactly one firmware startup marker; "
            "found 0",
            errors_without_firmware,
        )
        self.assertIn(
            "positive run log must contain exactly one firmware startup marker; "
            "found 2",
            errors_with_duplicate_firmware,
        )

    def test_duplicate_firmware_processes_are_rejected_for_both_runs(self) -> None:
        positive_errors = self._validate(firmware_process_count=2)
        negative_errors = self._validate(negative_firmware_process_count=2)

        self.assertIn(
            "positive run must have exactly one firmware process; observed 2",
            positive_errors,
        )
        self.assertIn(
            "negative control must have exactly one firmware process; observed 2",
            negative_errors,
        )


if __name__ == "__main__":
    unittest.main()
