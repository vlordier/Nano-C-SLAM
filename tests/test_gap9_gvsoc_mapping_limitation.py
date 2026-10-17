"""Adversarial checks for the target-only GVSOC limitation artifact."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.capture_gap9_gvsoc_mapping_limitation import main
from test_support.gap9_mapping_fixtures import (
    GVSOC_MAPPING_LIMITATION_V1,
    HOST_MAPPING_FAILURE,
    HOST_MAPPING_PASS,
    write_executable_output,
)


class GvsocMappingLimitationTests(unittest.TestCase):
    def _capture(self, marker: str, host_marker: str) -> tuple[int, dict[str, object]]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            elf = directory / "synthetic-target.elf"
            log = directory / "gvsoc.log"
            host_elf = directory / "host-smoke"
            output = directory / "report.json"
            elf.write_bytes(b"test GVSOC ELF placeholder")
            log.write_text(marker + "\n", encoding="utf-8")
            write_executable_output(host_elf, host_marker + "\n")
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
        result, report = self._capture(GVSOC_MAPPING_LIMITATION_V1, HOST_MAPPING_PASS)

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
            GVSOC_MAPPING_LIMITATION_V1,
            HOST_MAPPING_FAILURE,
        )

        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "reproduced-limitation")
        self.assertEqual(report["host_core_reference"]["status"], "failed")

    def test_unexpected_gvsoc_metrics_do_not_pass(self) -> None:
        result, report = self._capture(
            GVSOC_MAPPING_LIMITATION_V1.replace("poses=60", "poses=59", 1),
            HOST_MAPPING_PASS,
        )

        self.assertEqual(result, 1)
        self.assertEqual(report["status"], "unexpected-result")


if __name__ == "__main__":
    unittest.main()
