"""Hardware-free tests for post-failure Gazebo motion classification."""

from __future__ import annotations

import unittest

from scripts.gazebo_failure_probe import (
    _classify_post_exit_motion,
    _matching_hashes,
    _velocity_controller_plugin,
)


class PostExitMotionClassificationTests(unittest.TestCase):
    def test_fewer_than_three_samples_is_unknown(self) -> None:
        self.assertEqual(
            _classify_post_exit_motion([0.5, 0.5], [0.0, 0.0]),
            "unknown: fewer than three post-exit odometry samples",
        )

    def test_ground_height_is_reported_as_landed_or_grounded(self) -> None:
        self.assertEqual(
            _classify_post_exit_motion([0.1, 0.08, 0.02], [0.0, 0.01, 0.0]),
            "landed or remained grounded",
        )

    def test_stable_hover_height_is_reported_as_held(self) -> None:
        self.assertEqual(
            _classify_post_exit_motion(
                [0.50, 0.51, 0.50, 0.49], [0.01, 0.02, 0.0, 0.01]
            ),
            "held near a steady altitude during the observed window",
        )

    def test_vertical_motion_is_reported_as_drift(self) -> None:
        self.assertEqual(
            _classify_post_exit_motion([0.5, 0.45, 0.3], [0.01, 0.08, 0.2]),
            "drift or continued motion",
        )

    def test_ambiguous_motion_is_not_overclassified(self) -> None:
        self.assertEqual(
            _classify_post_exit_motion([0.5, 0.47, 0.44], [0.02, 0.04, 0.06]),
            "unknown: observed motion is between hold and drift thresholds",
        )


class InstalledSourceIdentityTests(unittest.TestCase):
    def test_matching_nonempty_hashes_pass(self) -> None:
        self.assertTrue(_matching_hashes("abc", "abc"))

    def test_missing_or_different_hashes_fail_closed(self) -> None:
        for source_hash, installed_hash in (
            (None, None),
            ("", ""),
            ("abc", None),
            ("abc", "def"),
        ):
            with self.subTest(source_hash=source_hash, installed_hash=installed_hash):
                self.assertFalse(_matching_hashes(source_hash, installed_hash))

    def test_velocity_controller_plugin_requires_a_hashed_loaded_library(self) -> None:
        evidence = {
            "libraries": [
                {"path": "/opt/lib/libother.dylib", "sha256": "other"},
                {
                    "path": "/opt/lib/libgz-sim-multicopter-control-system.dylib",
                    "sha256": "plugin-hash",
                },
            ]
        }
        self.assertEqual(
            _velocity_controller_plugin(evidence), evidence["libraries"][1]
        )
        self.assertIsNone(_velocity_controller_plugin({"libraries": []}))
        self.assertIsNone(
            _velocity_controller_plugin(
                {
                    "libraries": [
                        {
                            "path": (
                                "/opt/lib/libgz-sim-multicopter-control-system.dylib"
                            ),
                            "sha256": None,
                        }
                    ]
                }
            )
        )
