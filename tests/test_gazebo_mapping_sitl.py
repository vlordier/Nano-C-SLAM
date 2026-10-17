"""Tests for converting Gazebo SITL data and running the owned mapping core."""

from __future__ import annotations

import json
import math
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from scripts.gazebo_mapping_sitl import _parse_args
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_model import (
    MappingSample,
    quaternion_yaw,
    summarize_sample_path,
    validate_mapping_result,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def _deterministic_samples(count: int) -> list[MappingSample]:
    samples = []
    for sample_id in range(count):
        angle = sample_id * 0.02
        samples.append(
            MappingSample(
                sample_id=sample_id,
                timestamp_ms=sample_id * 100,
                x_m=0.15 * math.cos(angle),
                y_m=0.15 * math.sin(angle),
                yaw_rad=angle + math.pi / 2,
                ranges_mm=(1000, 1200, 900, 1100),
            )
        )
    return samples


class MappingSampleTests(unittest.TestCase):
    def test_mapping_capture_cli_reuses_prebuilt_runner(self) -> None:
        options = _parse_args(["--reuse-binary"])

        self.assertTrue(options.reuse_binary)
        self.assertEqual(options.samples, 60)

    def test_serializes_pose_and_four_range_readings(self) -> None:
        sample = MappingSample(4, 250, 1.25, -0.5, 0.75, (1000, -1, 850, 1200))

        self.assertEqual(sample.to_record(), "4 250 1.25 -0.5 0.75 1000 -1 850 1200")

    def test_rejects_invalid_samples_and_zero_quaternion(self) -> None:
        with self.assertRaisesRegex(ValueError, "int16"):
            MappingSample(32768, 0, 0, 0, 0, (1, 1, 1, 1)).to_record()
        with self.assertRaisesRegex(ValueError, "finite"):
            MappingSample(0, 0, math.nan, 0, 0, (1, 1, 1, 1)).to_record()
        with self.assertRaisesRegex(ValueError, "nonzero"):
            quaternion_yaw(0, 0, 0, 0)

    def test_converts_unit_quaternion_to_planar_yaw(self) -> None:
        yaw = math.pi / 3
        self.assertAlmostEqual(
            quaternion_yaw(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)), yaw
        )

    def test_summarizes_horizontal_odometry_excitation(self) -> None:
        samples = [
            MappingSample(0, 0, 0.0, 0.0, 0.0, (100, 100, 100, 100)),
            MappingSample(1, 100, 0.3, 0.0, 0.0, (100, 100, 100, 100)),
            MappingSample(2, 200, 0.3, 0.4, 0.0, (100, 100, 100, 100)),
        ]

        self.assertEqual(
            summarize_sample_path(samples),
            {
                "x_bounds_m": [0.0, 0.3],
                "y_bounds_m": [0.0, 0.4],
                "horizontal_path_length_m": 0.7,
            },
        )
        with self.assertRaisesRegex(ValueError, "at least one"):
            summarize_sample_path([])

    def test_rejects_mapping_without_three_valid_scan_windows(self) -> None:
        report = {
            "pose_count": 39,
            "scan_count": 1,
            "map_point_count": 1,
            "icp_constraint_count": 0,
            "optimized_pose_count": 39,
            "main_graph_size": 0,
            "has_nan": 0,
            "poses": [[0.0, 0.0, 0.0]] * 39,
            "points": [[0.0, 0.0]],
        }
        with self.assertRaisesRegex(ValueError, "three scan windows"):
            validate_mapping_result(report, minimum_poses=60)

    def test_retained_nearfield_report_reconciles_four_beams_and_scan_windows(
        self,
    ) -> None:
        report_path = (
            REPOSITORY_ROOT
            / "artifacts/gazebo_mapping_sitl/cf1_nearfield_firmware_cutoff.json"
        )
        report = json.loads(report_path.read_text(encoding="utf-8"))
        mapping = validate_mapping_result(report["mapping"], minimum_poses=60)
        samples = report["input_samples"]
        expected_directions = ("front", "back", "left", "right")

        self.assertEqual(len(samples), 60)
        self.assertEqual(report["mapping_range_limit_mm"], 1400)
        self.assertEqual(mapping["stored_constraint_count"], 4)
        self.assertEqual(mapping["optimized_constraint_count"], 4)
        for index, direction in enumerate(expected_directions, start=5):
            values = [int(sample.split()[index]) for sample in samples]
            accepted = [value for value in values if 0 <= value <= 1400]
            self.assertGreater(len(set(accepted)), 1)
            self.assertEqual(
                report["range_accounting"]["by_direction"][direction][
                    "accepted_at_or_below_limit"
                ],
                len(accepted),
            )

        in_range_per_window = [
            sum(
                any(0 <= int(value) <= 1400 for value in sample.split()[5:9])
                for sample in samples[start : start + 20]
            )
            for start in range(0, len(samples), 20)
        ]
        self.assertEqual(in_range_per_window, [20, 20, 20])
        self.assertEqual(report["scan_window_in_range_sample_counts"], [20, 20, 20])
        three_direction_per_window = [
            sum(
                sum(0 <= int(value) <= 1400 for value in sample.split()[5:9]) >= 3
                for sample in samples[start : start + 20]
            )
            for start in range(0, len(samples), 20)
        ]
        self.assertEqual(three_direction_per_window, [20, 19, 20])
        self.assertEqual(
            report["scan_window_three_direction_sample_counts"], [20, 19, 20]
        )


class NativeMappingReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._temporary_directory = tempfile.TemporaryDirectory()
        cls.binary = Path(cls._temporary_directory.name) / "mapping_replay"
        environment = os.environ.copy()
        environment["NANO_C_SLAM_HOST_SITL"] = str(cls.binary)
        subprocess.run(  # noqa: S603 -- fixed repository build script
            [str(REPOSITORY_ROOT / "scripts/build_host_mapping_sitl.sh")],
            check=True,
            cwd=REPOSITORY_ROOT,
            env=environment,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls._temporary_directory.cleanup()

    def test_runs_real_point_cloud_icp_and_pose_graph_path(self) -> None:
        records = "\n".join(sample.to_record() for sample in _deterministic_samples(60))
        result = subprocess.run(  # noqa: S603 -- built fixture executable
            [str(self.binary)],
            input=records + "\n",
            capture_output=True,
            check=True,
            text=True,
            timeout=30,
        )
        report = validate_mapping_result(json.loads(result.stdout))

        self.assertEqual(report["pose_count"], 60)
        self.assertEqual(report["scan_count"], 3)
        self.assertEqual(report["map_point_count"], 240)
        self.assertEqual(report["icp_constraint_attempt_count"], 2)
        self.assertEqual(report["stored_constraint_count"], 4)
        self.assertEqual(report["optimized_constraint_count"], 4)
        self.assertEqual(report["icp_constraint_count"], 4)
        self.assertEqual(report["optimized_pose_count"], 60)
        graph_size = report["main_graph_size"]
        assert isinstance(graph_size, int)
        self.assertGreaterEqual(graph_size, 2)

    def test_drops_ranges_beyond_firmware_loop_closure_limit(self) -> None:
        samples = [
            MappingSample(index, index * 100, 0.0, 0.0, 0.0, (1500, 1500, 1500, 1500))
            for index in range(40)
        ]
        result = subprocess.run(  # noqa: S603 -- built fixture executable
            [str(self.binary)],
            input="\n".join(sample.to_record() for sample in samples) + "\n",
            capture_output=True,
            check=False,
            text=True,
            timeout=30,
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("no projected lidar points", result.stderr)

    def test_runs_the_same_embedded_scan_and_optimization_path_as_gvsoc(self) -> None:
        executable = REPOSITORY_ROOT / "cslam-gap9-app/BUILD-host/gap9_mapping_smoke"
        result = subprocess.run(  # noqa: S603 -- built repository fixture
            [str(executable)],
            capture_output=True,
            check=True,
            text=True,
            timeout=30,
        )

        self.assertIn(
            "NANO_C_SLAM_GVSOC:mapping_ok poses=60 scans=3 map_points=240 "
            "optimized=60 graph=3 has_nan=0",
            result.stdout,
        )

    def test_rejects_malformed_replay_input(self) -> None:
        result = subprocess.run(  # noqa: S603 -- built fixture executable
            [str(self.binary)],
            input="not a valid sample\n",
            capture_output=True,
            check=False,
            text=True,
            timeout=10,
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("malformed replay sample", result.stderr)


if __name__ == "__main__":
    unittest.main()
