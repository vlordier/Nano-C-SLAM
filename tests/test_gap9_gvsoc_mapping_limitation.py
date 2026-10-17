"""Adversarial checks for the target-only GVSOC limitation artifact."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.capture_gap9_gvsoc_mapping_limitation import main


class GvsocMappingLimitationTests(unittest.TestCase):
    def _capture(self, marker: str, host_marker: str) -> tuple[int, dict[str, object]]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            elf = directory / "fixture.elf"
            log = directory / "gvsoc.log"
            host_elf = directory / "host-smoke"
            output = directory / "report.json"
            elf.write_bytes(b"test GVSOC ELF placeholder")
            log.write_text(marker + "\n", encoding="utf-8")
            host_elf.write_text(f"#!/bin/sh\nprintf '%s\\n' '{host_marker}'\n")
            host_elf.chmod(0o755)
            with patch(
                "sys.argv",
                [
                    "capture_gap9_gvsoc_mapping_limitation.py",
                    "--elf",
                    str(elf),
                    "--log",
                    str(log),
                    "--output",
                    str(output),
                    "--gvsoc-exit-code",
                    "1",
                    "--host-elf",
                    str(host_elf),
                ],
            ):
                result = main()
            report = json.loads(output.read_text(encoding="utf-8"))
        return result, report

    def test_records_reproduced_target_failure_against_executed_host_pass(self) -> None:
        result, report = self._capture(
            "NANO_C_SLAM_GVSOC:mapping_fail poses=60 scans=3 points=240 "
            "icp_constraints=0 matches=80 icp_error_milli=2147483647 "
            "icp_has_nan=1 optimized=0 graph=0 has_nan=0",
            "NANO_C_SLAM_HOST:mapping_ok poses=60 scans=3 map_points=240 "
            "finite=60 graph=3",
        )

        self.assertEqual(result, 0)
        self.assertEqual(report["status"], "reproduced-limitation")
        self.assertEqual(report["evidence_class"], "gvsoc-target-execution")
        self.assertEqual(report["host_core_reference"]["status"], "passed")
        self.assertEqual(
            report["observed"]["matched_points_last_pair"],
            80,
        )
        self.assertIn("source_provenance", report)

    def test_host_failure_cannot_be_reported_as_reproduced_divergence(self) -> None:
        result, report = self._capture(
            "NANO_C_SLAM_GVSOC:mapping_fail poses=60 scans=3 points=240 "
            "icp_constraints=0 matches=80 icp_error_milli=2147483647 "
            "icp_has_nan=1 optimized=0 graph=0 has_nan=0",
            "NANO_C_SLAM_HOST:mapping_fail poses=60 scans=3 map_points=240 "
            "finite=0 graph=0",
        )

        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "reproduced-limitation")
        self.assertEqual(report["host_core_reference"]["status"], "failed")

    def test_unexpected_gvsoc_metrics_do_not_pass(self) -> None:
        result, report = self._capture(
            "NANO_C_SLAM_GVSOC:mapping_fail poses=59 scans=3 points=240 "
            "icp_constraints=0 matches=80 icp_error_milli=2147483647 "
            "icp_has_nan=1 optimized=0 graph=0 has_nan=0",
            "NANO_C_SLAM_HOST:mapping_ok poses=60 scans=3 map_points=240 "
            "finite=60 graph=3",
        )

        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "unexpected-result")


if __name__ == "__main__":
    unittest.main()
