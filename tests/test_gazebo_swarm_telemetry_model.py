"""Adversarial fixtures for the read-only three-drone telemetry contract."""

from __future__ import annotations

import types
import unittest

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.swarm_telemetry_model import (
    validate_pairwise_snapshot,
    validate_stamped_ranges,
)


def _stamp(nanoseconds: int) -> types.SimpleNamespace:
    seconds, nanosec = divmod(nanoseconds, 1_000_000_000)
    return types.SimpleNamespace(sec=seconds, nanosec=nanosec)


class SwarmTelemetryModelTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now_ns = 10_100_000_000
        self.source_ns = self.now_ns - 100_000_000
        self.positions = {
            "cf1": {self.source_ns: (0.0, 0.0, 0.0)},
            "cf3": {self.source_ns: (1.0, 0.0, 0.0)},
            "cf4": {self.source_ns: (0.0, 2.0, 0.0)},
        }
        self.pairwise = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=_stamp(self.now_ns)),
            sequence=4,
            drone_ids=["cf1", "cf3", "cf4"],
            source_stamps=[_stamp(self.source_ns) for _ in range(3)],
            pair_labels=["cf1:cf3", "cf1:cf4", "cf3:cf4"],
            ranges_m=[1.0, 2.0, 5**0.5],
        )

    def test_accepts_ranges_from_correct_namespace(self) -> None:
        message = types.SimpleNamespace(
            drone_id="cf3",
            header=types.SimpleNamespace(stamp=_stamp(self.source_ns)),
            sequence=9,
            ranges_mm=[100, -1, 230, 400],
        )
        self.assertEqual(
            validate_stamped_ranges(message, expected_drone="cf3", now_ns=self.now_ns),
            (self.source_ns, 9, (100, -1, 230, 400)),
        )

    def test_rejects_wrong_identity_bad_sequence_and_malformed_ray_values(self) -> None:
        valid = types.SimpleNamespace(
            drone_id="cf1",
            header=types.SimpleNamespace(stamp=_stamp(self.source_ns)),
            sequence=1,
            ranges_mm=[1, 2, 3, 4],
        )
        for field, value, expected_error in (
            ("drone_id", "cf3", "identity"),
            ("sequence", -1, "sequence"),
            ("ranges_mm", [1, 2, 3], "exactly four"),
            ("ranges_mm", [1, 2, 3, 32768], "invalid millimeter"),
        ):
            with self.subTest(field=field, value=value):
                message = types.SimpleNamespace(**vars(valid))
                setattr(message, field, value)
                with self.assertRaisesRegex(ValueError, expected_error):
                    validate_stamped_ranges(
                        message, expected_drone="cf1", now_ns=self.now_ns
                    )

    def test_rejects_stale_and_far_future_ranges(self) -> None:
        message = types.SimpleNamespace(
            drone_id="cf1",
            header=types.SimpleNamespace(stamp=_stamp(self.now_ns - 500_000_001)),
            sequence=1,
            ranges_mm=[1, 2, 3, 4],
        )
        with self.assertRaisesRegex(ValueError, "stale"):
            validate_stamped_ranges(message, expected_drone="cf1", now_ns=self.now_ns)
        message.header.stamp = _stamp(self.now_ns + 100_000_001)
        with self.assertRaisesRegex(ValueError, "future"):
            validate_stamped_ranges(message, expected_drone="cf1", now_ns=self.now_ns)

    def test_accepts_pairwise_ranges_only_for_exact_source_poses(self) -> None:
        self.assertEqual(
            validate_pairwise_snapshot(
                self.pairwise, now_ns=self.now_ns, odometry=self.positions
            ),
            (4, (1.0, 2.0, 5**0.5)),
        )

    def test_rejects_pairwise_identity_labels_and_distance_cross_assignment(
        self,
    ) -> None:
        for field, value, expected_error in (
            ("drone_ids", ["cf3", "cf1", "cf4"], "drone IDs"),
            ("pair_labels", ["cf1:cf4", "cf1:cf3", "cf3:cf4"], "labels"),
            ("ranges_m", [2.0, 1.0, 5**0.5], "do not match"),
        ):
            with self.subTest(field=field):
                message = types.SimpleNamespace(**vars(self.pairwise))
                setattr(message, field, value)
                with self.assertRaisesRegex(ValueError, expected_error):
                    validate_pairwise_snapshot(
                        message, now_ns=self.now_ns, odometry=self.positions
                    )

    def test_rejects_future_pairwise_publication_and_invalid_sequence(self) -> None:
        future = types.SimpleNamespace(**vars(self.pairwise))
        future.header = types.SimpleNamespace(stamp=_stamp(self.now_ns + 100_000_001))
        with self.assertRaisesRegex(ValueError, "too far in the future"):
            validate_pairwise_snapshot(
                future, now_ns=self.now_ns, odometry=self.positions
            )
        malformed = types.SimpleNamespace(**vars(self.pairwise))
        malformed.sequence = -1
        with self.assertRaisesRegex(ValueError, "sequence"):
            validate_pairwise_snapshot(
                malformed, now_ns=self.now_ns, odometry=self.positions
            )

    def test_rejects_missing_stale_skewed_and_unavailable_source_samples(self) -> None:
        cases = (
            (
                {"cf1": {}, "cf3": self.positions["cf3"], "cf4": self.positions["cf4"]},
                "exact",
                self.pairwise.source_stamps,
            ),
            (
                {
                    "cf1": {self.source_ns - 500_000_001: (0.0, 0.0, 0.0)},
                    "cf3": {self.source_ns - 500_000_001: (1.0, 0.0, 0.0)},
                    "cf4": {self.source_ns - 500_000_001: (0.0, 2.0, 0.0)},
                },
                "stale or future",
                [_stamp(self.source_ns - 500_000_001) for _ in range(3)],
            ),
        )
        for odometry, reason, stamps in cases:
            self.pairwise.source_stamps = stamps
            with (
                self.subTest(reason=reason),
                self.assertRaisesRegex(ValueError, reason),
            ):
                validate_pairwise_snapshot(
                    self.pairwise, now_ns=self.now_ns, odometry=odometry
                )
        self.pairwise.source_stamps = [_stamp(self.source_ns) for _ in range(3)]
        self.pairwise.source_stamps[2] = _stamp(self.source_ns - 100_000_001)
        with self.assertRaisesRegex(ValueError, "inter-drone skew"):
            validate_pairwise_snapshot(
                self.pairwise, now_ns=self.now_ns, odometry=self.positions
            )


if __name__ == "__main__":
    unittest.main()
