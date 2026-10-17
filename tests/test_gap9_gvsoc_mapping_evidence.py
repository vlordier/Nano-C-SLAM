"""Tests for the GAP9 GVSOC mapping evidence boundary."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.capture_gap9_gvsoc_mapping_evidence import main
from test_support.gap9_mapping_fixtures import (
    GVSOC_MAPPING_PASS_V1,
    HOST_MAPPING_PASS,
)


class GvsocMappingEvidenceTests(unittest.TestCase):
    def _capture(self, marker: str) -> tuple[int, dict[str, object]]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            elf = directory / "fixture.elf"
            log = directory / "gvsoc.log"
            output = directory / "report.json"
            elf.write_bytes(b"test ELF placeholder")
            log.write_text(marker + "\n", encoding="utf-8")
            with patch(
                "sys.argv",
                [
                    "capture_gap9_gvsoc_mapping_evidence.py",
                    "--elf",
                    str(elf),
                    "--log",
                    str(log),
                    "--output",
                    str(output),
                ],
            ):
                result = main()
            report = json.loads(output.read_text(encoding="utf-8"))
        return result, report

    def test_accepts_exact_gvsoc_mapping_marker(self) -> None:
        result, report = self._capture(GVSOC_MAPPING_PASS_V1)

        self.assertEqual(result, 0)
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["evidence_class"], "gvsoc-target-execution")
        observed = report["observed"]
        self.assertIsInstance(observed, dict)
        assert isinstance(observed, dict)
        self.assertEqual(observed["successful_solver_calls"], 3)
        self.assertEqual(observed["completed_solver_iterations"], 9)
        self.assertTrue(observed["main_solver_returned"])
        self.assertTrue(observed["optimization_completed"])

    def test_host_marker_cannot_be_captured_as_gvsoc_evidence(self) -> None:
        result, report = self._capture(HOST_MAPPING_PASS)

        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "failed")
        self.assertIsNone(report["observed"])

    def test_wrong_gvsoc_metrics_fail_capture(self) -> None:
        result, report = self._capture(
            GVSOC_MAPPING_PASS_V1.replace(
                "finite=60 graph=3 main_solver=1 solver_calls=3 "
                "solver_iterations=9 completed=1 has_nan=0",
                "finite=0 graph=0 main_solver=0 solver_calls=0 "
                "solver_iterations=0 completed=0 has_nan=1",
            )
        )

        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["evidence_class"], "gvsoc-target-execution")

    def test_rejects_wrong_iteration_or_completion_metrics(self) -> None:
        result, report = self._capture(
            GVSOC_MAPPING_PASS_V1.replace("solver_iterations=9", "solver_iterations=8")
        )
        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "failed")


if __name__ == "__main__":
    unittest.main()
