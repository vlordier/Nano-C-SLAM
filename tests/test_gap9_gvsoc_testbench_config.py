"""Tests for the generated GVSOC SPI testbench preflight check."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scripts.validate_gap9_gvsoc_testbench import validate_config


class GvsocTestbenchConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.config_path = Path(self.temporary_directory.name) / "gvsoc_config.json"

    def _write_config(self, *, enabled: bool = True) -> None:
        config = {
            "target": {
                "addon_testbench_enabled": enabled,
                "components": ["chip", "testbench"],
                "bindings": [
                    ["chip->spim2_cs0_data", "testbench->spi2_cs0_data"],
                    ["chip->spim2_cs0", "testbench->spi2_cs0"],
                ],
            }
        }
        self.config_path.write_text(json.dumps(config), encoding="utf-8")

    def test_accepts_enabled_testbench_with_both_spi2_bindings(self) -> None:
        self._write_config()

        self.assertEqual(validate_config(self.config_path), [])

    def test_rejects_disabled_testbench_even_when_bindings_are_present(self) -> None:
        self._write_config(enabled=False)

        self.assertEqual(
            validate_config(self.config_path),
            ["target.addon_testbench_enabled is not true"],
        )

    def test_rejects_missing_component_and_each_spi2_binding(self) -> None:
        self._write_config()
        config = json.loads(self.config_path.read_text(encoding="utf-8"))
        config["target"]["components"].remove("testbench")
        config["target"]["bindings"].clear()
        self.config_path.write_text(json.dumps(config), encoding="utf-8")

        self.assertEqual(
            validate_config(self.config_path),
            [
                "target.components does not contain testbench",
                (
                    "missing SPI2 testbench binding: chip->spim2_cs0 -> "
                    "testbench->spi2_cs0"
                ),
                (
                    "missing SPI2 testbench binding: chip->spim2_cs0_data -> "
                    "testbench->spi2_cs0_data"
                ),
            ],
        )

    def test_reports_missing_or_invalid_json(self) -> None:
        self.assertIn("cannot read GVSOC config", validate_config(self.config_path)[0])
        self.config_path.write_text("not json", encoding="utf-8")

        self.assertIn("cannot read GVSOC config", validate_config(self.config_path)[0])


if __name__ == "__main__":
    unittest.main()
