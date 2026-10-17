"""Tests for the GAP9 GVSOC mapping evidence boundary."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.capture_gap9_gvsoc_mapping_evidence import main


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
        result, report = self._capture(
            "NANO_C_SLAM_GVSOC:mapping_ok poses=60 scans=3 map_points=240 "
            "optimized=60 graph=3 has_nan=0"
        )

        self.assertEqual(result, 0)
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["evidence_class"], "gvsoc-target-execution")

    def test_host_marker_cannot_be_captured_as_gvsoc_evidence(self) -> None:
        result, report = self._capture(
            "NANO_C_SLAM_HOST:mapping_ok poses=60 scans=3 map_points=240 "
            "optimized=60 graph=3 has_nan=0"
        )

        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "failed")
        self.assertIsNone(report["observed"])

    def test_wrong_gvsoc_metrics_fail_capture(self) -> None:
        result, report = self._capture(
            "NANO_C_SLAM_GVSOC:mapping_ok poses=60 scans=3 map_points=240 "
            "optimized=0 graph=0 has_nan=1"
        )

        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["evidence_class"], "gvsoc-target-execution")


if __name__ == "__main__":
    unittest.main()
