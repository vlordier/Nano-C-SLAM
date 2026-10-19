"""Strict JSON parsing tests for hash-bound evidence manifests."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.evidence_json import load_json_file


class EvidenceJsonTests(unittest.TestCase):
    def test_rejects_duplicate_keys_at_any_object_depth(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "evidence.json"
            path.write_text(
                '{"run_id":"expected","capture":{"run_id":"first","run_id":"second"}}',
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "ambiguous duplicate keys"):
                load_json_file(path, label="target evidence")

    def test_loads_nested_unambiguous_manifest_values(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "evidence.json"
            path.write_text('{"run_id":"abc","counts":{"poses":60}}')

            self.assertEqual(
                load_json_file(path, label="target evidence"),
                {"run_id": "abc", "counts": {"poses": 60}},
            )


if __name__ == "__main__":
    unittest.main()
