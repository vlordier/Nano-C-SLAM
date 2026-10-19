"""Focused behavior tests for the grounded odometry start gate."""

from __future__ import annotations

import copy
import unittest

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.odometry_preflight import (
    GroundedOdometryPreflight,
    OdometrySample,
    validate_odometry_preflight,
    wait_for_grounded_preflight,
)


def _sample(index: int, *, altitude_m: float = 0.05) -> OdometrySample:
    stamp = 10.0 + index * 0.05
    return OdometrySample(stamp, altitude_m, stamp + 0.01, 100.0 + index * 0.05)


class GroundedOdometryPreflightTests(unittest.TestCase):
    def test_requires_three_fresh_ordered_grounded_samples(self) -> None:
        gate = GroundedOdometryPreflight()
        self.assertFalse(gate.observe(_sample(0)))
        self.assertFalse(gate.observe(_sample(1)))
        self.assertTrue(gate.observe(_sample(2)))
        self.assertTrue(validate_odometry_preflight(gate.to_report())["passed"])

    def test_rejects_airborne_nonfinite_stale_future_and_reordered_samples(
        self,
    ) -> None:
        invalid_samples = (
            _sample(0, altitude_m=0.151),
            OdometrySample(10.0, float("nan"), 10.01, 100.0),
            OdometrySample(10.0, float("inf"), 10.01, 100.0),
            OdometrySample(10.0, float("-inf"), 10.01, 100.0),
            OdometrySample(10.0, 0.05, 10.21, 100.0),
            OdometrySample(10.0, 0.05, 9.89, 100.0),
        )
        for sample in invalid_samples:
            with self.subTest(sample=sample):
                gate = GroundedOdometryPreflight()
                self.assertFalse(gate.observe(sample))
                self.assertIsNotNone(gate.failure_reason)

        gate = GroundedOdometryPreflight()
        gate.observe(_sample(1))
        gate.observe(_sample(0))
        self.assertIn("reordered", gate.failure_reason or "")

        gate = GroundedOdometryPreflight()
        gate.observe(_sample(0))
        gate.observe(OdometrySample(10.25, 0.05, 10.26, 100.25))
        self.assertIn("contiguous", gate.failure_reason or "")

    def test_wall_timeout_and_paused_sim_clock_fail_closed(self) -> None:
        gate = GroundedOdometryPreflight()
        clock_values = iter((100.0, 100.3))
        with self.assertRaisesRegex(TimeoutError, "timed out"):
            wait_for_grounded_preflight(
                gate,
                lambda: self.fail("must not pump after the deadline"),
                timeout_s=0.2,
                monotonic=lambda: next(clock_values),
            )

        gate = GroundedOdometryPreflight()
        gate.observe(_sample(0))
        with self.assertRaisesRegex(RuntimeError, "reordered"):
            wait_for_grounded_preflight(
                gate,
                lambda: gate.observe(_sample(0)),
                timeout_s=1.0,
                monotonic=lambda: 100.0,
            )

    def test_report_rejects_mutated_threshold_or_sample(self) -> None:
        gate = GroundedOdometryPreflight()
        for index in range(3):
            gate.observe(_sample(index))
        report = gate.to_report()
        invalid = copy.deepcopy(report)
        invalid["max_grounded_altitude_m"] = 1.0
        with self.assertRaisesRegex(ValueError, "thresholds"):
            validate_odometry_preflight(invalid)

        invalid = copy.deepcopy(report)
        samples = invalid["samples"]
        assert isinstance(samples, list) and isinstance(samples[1], dict)
        samples[1]["altitude_m"] = 0.5
        with self.assertRaisesRegex(ValueError, "airborne"):
            validate_odometry_preflight(invalid)

        invalid = copy.deepcopy(report)
        samples = invalid["samples"]
        assert isinstance(samples, list) and isinstance(samples[0], dict)
        samples[0]["altitude_m"] = True
        with self.assertRaisesRegex(ValueError, "non-numeric"):
            validate_odometry_preflight(invalid)


if __name__ == "__main__":
    unittest.main()
