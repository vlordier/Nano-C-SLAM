"""Tests for converting Gazebo SITL data and running the owned mapping core."""

from __future__ import annotations

import copy
import json
import math
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
from typing import cast

from scripts.gazebo_mapping_sitl import (
    _format_capture_timeout,
    _parse_args,
    _run_mapping,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_model import (
    MappingSample,
    quaternion_yaw,
    summarize_range_accounting,
    summarize_sample_path,
    validate_capture_report,
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


def _boundary_samples(count: int = 20) -> list[MappingSample]:
    values = (-1, 0, 1, 1400, 1401)
    return [
        MappingSample(
            sample_id=index,
            timestamp_ms=(index + 1) * 100,
            x_m=index * 0.001,
            y_m=0.0,
            yaw_rad=0.0,
            ranges_mm=cast(
                tuple[int, int, int, int],
                tuple(values[(index * 4 + beam) % len(values)] for beam in range(4)),
            ),
        )
        for index in range(count)
    ]


class MappingSampleTests(unittest.TestCase):
    def test_mapping_capture_cli_reuses_prebuilt_runner(self) -> None:
        options = _parse_args(["--reuse-binary"])

        self.assertTrue(options.reuse_binary)
        self.assertEqual(options.samples, 60)

    def test_capture_timeout_reports_named_pairing_rejections(self) -> None:
        message = _format_capture_timeout(
            0, 60, {"stale_range": 12, "future_odometry": 3}
        )

        self.assertIn("captured 0 of 60 samples", message)
        self.assertIn("stale_range=12", message)
        self.assertIn("future_odometry=3", message)

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
        with self.assertRaisesRegex(ValueError, "integer millimeters"):
            MappingSample(
                0,
                0,
                0,
                0,
                0,
                cast(tuple[int, int, int, int], (1000, 1000.5, 1000, 1000)),
            ).to_record()
        with self.assertRaisesRegex(ValueError, "integer millimeters"):
            MappingSample(
                0,
                0,
                0,
                0,
                0,
                cast(tuple[int, int, int, int], (1000, math.nan, 1000, 1000)),
            ).to_record()

    def test_range_accounting_matches_exact_inclusive_gate_boundaries(self) -> None:
        summary = summarize_range_accounting(_boundary_samples())
        self.assertEqual(
            summary["range_accounting"],
            {
                "readings_total": 80,
                "invalid": 16,
                "valid": 64,
                "accepted_at_or_below_limit": 48,
                "rejected_above_limit": 16,
                "by_direction": {
                    direction: {
                        "valid": 16,
                        "accepted_at_or_below_limit": 12,
                        "distinct_accepted_values": 3,
                    }
                    for direction in ("front", "back", "left", "right")
                },
            },
        )
        self.assertEqual(
            summary["input_valid_range_bounds_mm"], {"minimum": 0, "maximum": 1401}
        )

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
            "icp_pair_attempt_count": 0,
            "icp_pair_attempt_pairs": [],
            "icp_constraint_add_attempt_count": 0,
            "icp_constraint_accepted_insertion_count": 0,
            "icp_constraint_accepted_endpoint_pairs": [],
            "stored_constraint_count": 0,
            "stored_unique_endpoint_pairs": [],
            "optimizer_input_constraint_count": 0,
            "optimizer_input_unique_edge_count": 0,
            "optimizer_input_unique_edge_pairs": [],
            "finite_pose_count": 0,
            "main_graph_size": 0,
            "main_solver_returned": 0,
            "successful_solver_call_count": 0,
            "completed_solver_iteration_count": 0,
            "optimization_completed": 0,
            "has_nan": 0,
            "poses": [[0.0, 0.0, 0.0]] * 39,
            "points": [[0.0, 0.0]],
        }
        with self.assertRaisesRegex(ValueError, "three scan windows"):
            validate_mapping_result(report, minimum_poses=60)

    def test_mapping_validator_rejects_graph_larger_than_scan_set(self) -> None:
        report = {
            "pose_count": 60,
            "scan_count": 3,
            "map_point_count": 3,
            "icp_pair_attempt_count": 2,
            "icp_pair_attempt_pairs": [[0, 1], [1, 2]],
            "icp_constraint_add_attempt_count": 2,
            "icp_constraint_accepted_insertion_count": 2,
            "icp_constraint_accepted_endpoint_pairs": [[0, 1], [1, 2]],
            "stored_constraint_count": 2,
            "stored_unique_endpoint_pairs": [[0, 1], [1, 2]],
            "optimizer_input_constraint_count": 2,
            "optimizer_input_unique_edge_count": 2,
            "optimizer_input_unique_edge_pairs": [[0, 1], [1, 2]],
            "main_graph_size": 4,
            "has_nan": 0,
            "poses": [[0.0, 0.0, 0.0]] * 60,
            "points": [[0.0, 0.0]] * 3,
        }
        with self.assertRaisesRegex(ValueError, "more nodes than captured scans"):
            validate_mapping_result(report)

        report["main_graph_size"] = 3
        report["has_nan"] = False
        with self.assertRaisesRegex(ValueError, "finite output"):
            validate_mapping_result(report)

        report["has_nan"] = 0
        report["main_graph_size"] = 3
        report["optimized_constraint_count"] = 2
        with self.assertRaisesRegex(
            ValueError, "unsupported optimized_constraint_count"
        ):
            validate_mapping_result(report)

        del report["optimized_constraint_count"]
        report["optimized_pose_count"] = 60
        with self.assertRaisesRegex(ValueError, "unsupported optimized_pose_count"):
            validate_mapping_result(report)
        del report["optimized_pose_count"]
        report["stored_unique_endpoint_pairs"] = [[0, 1], [0, 1]]
        with self.assertRaisesRegex(ValueError, "contains duplicate edges"):
            validate_mapping_result(report)

    def test_live_capture_report_enforces_cross_field_provenance(self) -> None:
        report_path = (
            REPOSITORY_ROOT
            / "artifacts/gazebo_mapping_sitl/cf1_nearfield_stamped_capture.json"
        )
        report = json.loads(report_path.read_text(encoding="utf-8"))
        self.assertEqual(validate_capture_report(report), report)

        invalid = copy.deepcopy(report)
        invalid["range_accounting"]["accepted_at_or_below_limit"] += 1
        with self.assertRaisesRegex(ValueError, "does not reconcile"):
            validate_capture_report(invalid)

        invalid = copy.deepcopy(report)
        invalid["input_pair_provenance"][0]["drone_id"] = "cf2"
        with self.assertRaisesRegex(ValueError, "different drone"):
            validate_capture_report(invalid)

        invalid = copy.deepcopy(report)
        invalid["evidence_class"] = "synthetic-unit-fixture"
        with self.assertRaisesRegex(ValueError, "unsupported evidence class"):
            validate_capture_report(invalid)

        invalid = copy.deepcopy(report)
        invalid.pop("source_provenance")
        with self.assertRaisesRegex(ValueError, "source provenance"):
            validate_capture_report(invalid)

        invalid = copy.deepcopy(report)
        invalid["input_pair_provenance"][0].pop("odometry_stamp_ns")
        with self.assertRaisesRegex(ValueError, "must be integers"):
            validate_capture_report(invalid)

        invalid = copy.deepcopy(report)
        invalid["mapping_range_limit_mm"] = 1401
        with self.assertRaisesRegex(ValueError, "exact 1400 mm"):
            validate_capture_report(invalid)

        invalid = copy.deepcopy(report)
        invalid["mapping_range_limit_mm"] = True
        with self.assertRaisesRegex(ValueError, "exact 1400 mm"):
            validate_capture_report(invalid)

        invalid = copy.deepcopy(report)
        invalid["input_pair_provenance"][0]["range_age_ns"] = 500_000_001
        invalid["input_pair_provenance"][0]["odometry_age_ns"] = 500_000_001
        with self.assertRaisesRegex(ValueError, "exceeds its limit"):
            validate_capture_report(invalid)

    def test_retained_gazebo_freshness_report_validates_pair_ages(self) -> None:
        report_path = (
            REPOSITORY_ROOT
            / "artifacts/gazebo_mapping_sitl/cf1_nearfield_stamped_freshness.json"
        )
        report = json.loads(report_path.read_text(encoding="utf-8"))
        validate_capture_report(report)

        pairs = report["input_pair_provenance"]
        self.assertEqual(len(pairs), 170)
        self.assertTrue(
            all("range_age_ns" in pair and "odometry_age_ns" in pair for pair in pairs)
        )
        self.assertLessEqual(
            max(max(pair["range_age_ns"], pair["odometry_age_ns"]) for pair in pairs),
            report["pairing_limits"]["maximum_age_ns"],
        )
        self.assertEqual(max(pair["pair_skew_ns"] for pair in pairs), 0)

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
        self.assertEqual(mapping["stored_constraint_count"], 3)
        self.assertEqual(mapping["optimizer_input_constraint_count"], 3)
        self.assertEqual(mapping["optimizer_input_unique_edge_count"], 3)
        self.assertEqual(
            mapping["stored_unique_endpoint_pairs"],
            [[0, 20], [20, 40], [0, 40]],
        )
        self.assertEqual(
            mapping["optimizer_input_unique_edge_pairs"],
            [[0, 1], [1, 2], [0, 2]],
        )
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
        self.assertEqual(report["icp_constraint_add_attempt_count"], 2)
        self.assertEqual(report["icp_constraint_accepted_insertion_count"], 2)
        self.assertEqual(report["stored_constraint_count"], 2)
        self.assertEqual(report["optimizer_input_constraint_count"], 2)
        self.assertEqual(report["optimizer_input_unique_edge_count"], 2)
        self.assertEqual(report["icp_pair_attempt_pairs"], [[0, 20], [20, 40]])
        self.assertEqual(
            report["icp_constraint_accepted_endpoint_pairs"], [[0, 20], [20, 40]]
        )
        self.assertEqual(report["stored_unique_endpoint_pairs"], [[0, 20], [20, 40]])
        self.assertEqual(report["optimizer_input_unique_edge_pairs"], [[0, 1], [1, 2]])
        self.assertEqual(report["main_solver_returned"], 1)
        self.assertEqual(report["successful_solver_call_count"], 3)
        self.assertEqual(report["completed_solver_iteration_count"], 9)
        self.assertEqual(report["optimization_completed"], 1)
        graph_size = report["main_graph_size"]
        assert isinstance(graph_size, int)
        self.assertGreaterEqual(graph_size, 2)

    def test_retained_live_reports_replay_through_owned_mapping_binary(self) -> None:
        artifact_paths = ("cf1_nearfield_stamped_capture.json",)
        for name in artifact_paths:
            with self.subTest(artifact=name):
                artifact_path = REPOSITORY_ROOT / "artifacts/gazebo_mapping_sitl" / name
                artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
                samples = []
                for record in artifact["input_samples"]:
                    fields = record.split()
                    samples.append(
                        MappingSample(
                            sample_id=int(fields[0]),
                            timestamp_ms=int(fields[1]),
                            x_m=float(fields[2]),
                            y_m=float(fields[3]),
                            yaw_rad=float(fields[4]),
                            ranges_mm=cast(
                                tuple[int, int, int, int],
                                tuple(int(value) for value in fields[5:]),
                            ),
                        )
                    )

                replayed = _run_mapping(self.binary, samples)
                retained = artifact["mapping"]
                self.assertEqual(
                    retained["legacy_solver_claims"][
                        "solver_return_iterations_and_completion"
                    ],
                    "not_recorded_in_original_capture; do not infer",
                )
                for field in (
                    "pose_count",
                    "scan_count",
                    "map_point_count",
                    "icp_pair_attempt_count",
                    "icp_pair_attempt_pairs",
                    "icp_constraint_add_attempt_count",
                    "icp_constraint_accepted_insertion_count",
                    "icp_constraint_accepted_endpoint_pairs",
                    "stored_constraint_count",
                    "stored_unique_endpoint_pairs",
                    "optimizer_input_constraint_count",
                    "optimizer_input_unique_edge_count",
                    "optimizer_input_unique_edge_pairs",
                    "has_nan",
                ):
                    self.assertEqual(replayed[field], retained[field], field)

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

    def test_c_replay_uses_inclusive_1400_mm_gate_and_rejects_nonfinite_pose(
        self,
    ) -> None:
        samples = _boundary_samples()
        result = subprocess.run(  # noqa: S603 -- built fixture executable
            [str(self.binary)],
            input="\n".join(sample.to_record() for sample in samples) + "\n",
            capture_output=True,
            check=True,
            text=True,
            timeout=10,
        )
        mapping = json.loads(result.stdout)
        self.assertEqual(mapping["map_point_count"], 48)
        range_summary = summarize_range_accounting(samples)
        accounting = range_summary["range_accounting"]
        assert isinstance(accounting, dict)
        self.assertEqual(
            accounting["accepted_at_or_below_limit"], mapping["map_point_count"]
        )

        invalid_pose = subprocess.run(  # noqa: S603 -- built fixture executable
            [str(self.binary)],
            input="0 100 nan 0 0 0 1 1400 1401\n",
            capture_output=True,
            check=False,
            text=True,
            timeout=10,
        )
        self.assertNotEqual(invalid_pose.returncode, 0)
        self.assertIn("sample IDs, timestamp, or pose are invalid", invalid_pose.stderr)

    def test_c_replay_projects_each_cardinal_ray_for_cardinal_body_yaws(self) -> None:
        offsets_m = (0.025, -0.025, 0.02, -0.02)
        direction_yaws = (0.0, math.pi, math.pi / 2, -math.pi / 2)
        body_yaws = (0.0, math.pi / 2, math.pi, -math.pi / 2)
        column_angle = math.pi / 64
        range_m = 1.0
        pose_x_m, pose_y_m = 1.25, -0.75

        # Derive expected coordinates from the C formula documented in
        # sim/gazebo/SENSOR_PROJECTION.md, then compare with pcl_extract_scan.
        for direction, direction_yaw in enumerate(direction_yaws):
            for body_yaw in body_yaws:
                with self.subTest(direction=direction, body_yaw=body_yaw):
                    yaw = body_yaw + direction_yaw
                    local_x = range_m + offsets_m[direction]
                    local_y = range_m * math.tan(column_angle)
                    expected = (
                        pose_x_m + local_x * math.cos(yaw) - local_y * math.sin(yaw),
                        pose_y_m + local_x * math.sin(yaw) + local_y * math.cos(yaw),
                    )
                    records = []
                    for sample_id in range(20):
                        ranges = [-1, -1, -1, -1]
                        ranges[direction] = 1000
                        sample = MappingSample(
                            sample_id=sample_id,
                            timestamp_ms=(sample_id + 1) * 100,
                            x_m=pose_x_m,
                            y_m=pose_y_m,
                            yaw_rad=body_yaw,
                            ranges_mm=cast(tuple[int, int, int, int], tuple(ranges)),
                        )
                        records.append(sample.to_record())

                    result = subprocess.run(  # noqa: S603 -- built fixture executable
                        [str(self.binary)],
                        input="\n".join(records) + "\n",
                        capture_output=True,
                        check=True,
                        text=True,
                        timeout=10,
                    )
                    mapping = json.loads(result.stdout)
                    points = mapping["points"]
                    self.assertEqual(len(points), 20)
                    for point in points:
                        self.assertAlmostEqual(point[0], expected[0], delta=1e-6)
                        self.assertAlmostEqual(point[1], expected[1], delta=1e-6)

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
            "NANO_C_SLAM_HOST:mapping_ok poses=60 scans=3 map_points=240 "
            "finite=60 graph=3 main_solver=1 solver_calls=3 "
            "solver_iterations=9 completed=1 has_nan=0",
            result.stdout,
        )
        self.assertNotIn("NANO_C_SLAM_GVSOC:", result.stdout)

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
