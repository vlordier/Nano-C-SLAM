"""Repeat-run path selection for provenance-bound GAP9 evidence."""

from __future__ import annotations

import tempfile
import unittest
from datetime import UTC, datetime
from pathlib import Path

from scripts.select_gap9_mapping_output import select_mapping_output


class MappingOutputSelectionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.output = self.root / "mapping_diagnostic.json"
        self.fixed_time = datetime(2026, 10, 18, tzinfo=UTC)

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def test_default_uses_requested_path_when_both_outputs_are_free(self) -> None:
        selected = select_mapping_output(
            self.output, explicit=False, now=self.fixed_time
        )
        self.assertEqual(selected, self.output)

    def test_default_preserves_existing_json_and_selects_timestamped_path(self) -> None:
        self.output.write_text("old report", encoding="utf-8")
        selected = select_mapping_output(
            self.output, explicit=False, now=self.fixed_time
        )
        self.assertEqual(
            selected, self.root / "mapping_diagnostic-20261018T000000Z.json"
        )
        self.assertEqual(self.output.read_text(encoding="utf-8"), "old report")

    def test_default_avoids_a_stale_sidecar_and_timestamp_collision(self) -> None:
        self.output.with_suffix(".log").write_text("old log", encoding="utf-8")
        first = self.root / "mapping_diagnostic-20261018T000000Z.json"
        first.write_text("other report", encoding="utf-8")
        selected = select_mapping_output(
            self.output, explicit=False, now=self.fixed_time
        )
        self.assertEqual(
            selected, self.root / "mapping_diagnostic-20261018T000000Z-1.json"
        )

    def test_explicit_collision_fails_without_changing_existing_evidence(self) -> None:
        self.output.with_suffix(".log").write_text("preserve", encoding="utf-8")
        with self.assertRaisesRegex(FileExistsError, "already exists"):
            select_mapping_output(self.output, explicit=True, now=self.fixed_time)
        self.assertEqual(self.output.with_suffix(".log").read_text(), "preserve")


if __name__ == "__main__":
    unittest.main()
