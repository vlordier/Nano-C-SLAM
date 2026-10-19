"""Fail-closed tests for the frozen GAP9 geometry acceptance profile."""

from __future__ import annotations

import hashlib
import json
import math
import unittest
from pathlib import Path

from scripts.validate_gap9_geometry import gate_report, validate_geometry

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "artifacts/gap9_gvsoc_evidence/geometry_baseline_20261019"


def _profile() -> dict[str, object]:
    return json.loads((EVIDENCE / "geometry-thresholds.json").read_text())


def _baseline_metrics() -> dict[str, object]:
    summary = json.loads((EVIDENCE / "baseline-summary.json").read_text())
    row = summary["runs"][0]
    row["host_sample_count"] = 60
    return row


class GeometryValidationTests(unittest.TestCase):
    def test_every_committed_baseline_run_passes_frozen_profile(self) -> None:
        profile = _profile()
        summary = json.loads((EVIDENCE / "baseline-summary.json").read_text())
        for row in summary["runs"]:
            with self.subTest(run_id=row["run_id"]):
                metrics = {**row, "host_sample_count": 60}
                self.assertEqual(validate_geometry(metrics, profile), [])

    def test_each_limit_rejects_a_value_just_outside_its_boundary(self) -> None:
        profile = _profile()
        baseline = _baseline_metrics()
        limits = profile["limits"]
        for metric, rule in limits.items():
            with self.subTest(metric=metric):
                altered = dict(baseline)
                boundary = rule["value"]
                altered[metric] = (
                    boundary + 1e-9 if rule["direction"] == "max" else boundary - 1e-9
                )
                self.assertTrue(validate_geometry(altered, profile))

    def test_missing_bool_and_nonfinite_metrics_fail_closed(self) -> None:
        profile = _profile()
        baseline = _baseline_metrics()
        for invalid in (None, True, math.nan, math.inf):
            with self.subTest(invalid=invalid):
                altered = dict(baseline)
                altered["pose_xy_rmse_m"] = invalid
                failures = validate_geometry(altered, profile)
                self.assertTrue(
                    any("pose_xy_rmse_m" in failure for failure in failures)
                )

    def test_run_identity_and_completion_fields_are_gated(self) -> None:
        profile = _profile()
        baseline = _baseline_metrics()
        for field, value in (
            ("target_pose_count", 59),
            ("host_sample_count", 59),
            ("mission_phase", "LAND"),
            ("mapping_route", "circle"),
            ("sdf_sha256", "0" * 64),
        ):
            with self.subTest(field=field):
                altered = dict(baseline)
                altered[field] = value
                self.assertTrue(validate_geometry(altered, profile))

    def test_audit_report_names_profile_and_observed_gate_values(self) -> None:
        metrics = _baseline_metrics()
        report = gate_report(metrics, _profile())
        self.assertEqual(report["profile_id"], "gap9-nearfield-square-synthetic-v1")
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(
            report["gates"]["pose_xy_rmse_m"]["observed"],
            metrics["pose_xy_rmse_m"],
        )

    def test_heldout_run_uses_frozen_numeric_limits_and_reports_pose_regression(
        self,
    ) -> None:
        artifact = (
            ROOT / "artifacts/gap9_gvsoc_evidence/heldout_geometry_20261019_attempt03"
        )
        metrics = json.loads((artifact / "geometry-metrics.json").read_text())
        run = json.loads((artifact / "gazebo-run.json").read_text())
        profile_path = EVIDENCE / "geometry-thresholds.json"
        profile = json.loads(profile_path.read_text())
        report = gate_report(
            metrics,
            profile,
            independent_scenario=True,
            run_bundle=run,
            profile_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),
        )
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(
            [
                name
                for name, gate in report["gates"].items()
                if gate["status"] == "FAIL"
            ],
            ["pose_xy_rmse_m"],
        )
        self.assertEqual(report["scenario"]["route"], "heldout-asymmetric")
        self.assertNotEqual(report["scenario"]["sdf_sha256"], profile["sdf_sha256"])
        self.assertEqual(
            report["profile_sha256"],
            hashlib.sha256(profile_path.read_bytes()).hexdigest(),
        )

    def test_heldout_mode_rejects_baseline_world_and_mismatched_run_bundle(
        self,
    ) -> None:
        profile = _profile()
        summary = json.loads((EVIDENCE / "baseline-summary.json").read_text())
        baseline = {**summary["runs"][0], "host_sample_count": 60}
        baseline["mission_phase"] = "COMPLETE"
        baseline_run = {
            "run_id": baseline["run_id"],
            "gazebo_world_session_id": baseline["world_session_id"],
            "child_commands": {"mission": ["ros2", "run", "mapping_route:=square"]},
            "capture_report": {"run_id": baseline["run_id"], "input_sample_count": 60},
            "mission_terminal": {"run_id": baseline["run_id"], "phase": "COMPLETE"},
            "process_exit_codes": {"collector": 0, "mission": 0},
        }
        failures = validate_geometry(
            baseline,
            profile,
            independent_scenario=True,
            run_bundle=baseline_run,
        )
        self.assertTrue(any("heldout-asymmetric" in failure for failure in failures))
        self.assertTrue(any("not independent" in failure for failure in failures))

        artifact = (
            ROOT / "artifacts/gap9_gvsoc_evidence/heldout_geometry_20261019_attempt03"
        )
        metrics = json.loads((artifact / "geometry-metrics.json").read_text())
        run = json.loads((artifact / "gazebo-run.json").read_text())
        run["run_id"] = "00000000-0000-4000-8000-000000000000"
        with self.assertRaisesRegex(ValueError, "not bound to one complete run"):
            validate_geometry(
                metrics,
                profile,
                independent_scenario=True,
                run_bundle=run,
            )


if __name__ == "__main__":
    unittest.main()
