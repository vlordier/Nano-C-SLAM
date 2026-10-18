"""Adversarial validation for the hash-bound GVSOC SPI receive report."""

from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from typing import cast

from scripts.capture_gap9_gvsoc_spi_probe_evidence import (
    TARGET_FRAME_MARKER,
    capture_evidence,
)
from scripts.gap9_gvsoc_spi_probe_protocol import make_scan_frame


class Gap9GvsocSpiProbeEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.config_path = self.root / "gvsoc_config.json"
        self.elf_path = self.root / "probe.elf"
        self.host_transfer_path = self.root / "host_transfer.json"
        self.log_path = self.root / "gvsoc.log"
        self.source_path = self.root / "spi_driver.c"
        self._write_valid_evidence()

    def _write_valid_evidence(self) -> None:
        config = {
            "target": {
                "addon_testbench_enabled": True,
                "components": ["testbench"],
                "bindings": [
                    ["chip->spim2_cs0_data", "testbench->spi2_cs0_data"],
                    ["chip->spim2_cs0", "testbench->spi2_cs0"],
                ],
            }
        }
        self.config_path.write_text(json.dumps(config), encoding="utf-8")
        self.elf_path.write_bytes(b"probe ELF fixture")
        self.source_path.write_text("void spi_probe(void) {}\n", encoding="utf-8")
        transfer = {
            "pre_cs_dummy_clock_cycles": 8,
            "frame_tx_length": 97,
            "frame_tx_hex": make_scan_frame().hex(),
        }
        self.host_transfer_path.write_text(json.dumps(transfer), encoding="utf-8")
        self.log_path.write_text(
            "NANO_C_SLAM_SPI_SLAVE_PROBE:armed frame=97\n"
            f"{TARGET_FRAME_MARKER}\n"
            "NANO_C_SLAM_SPI_SLAVE_PROBE:payload_ok\n",
            encoding="utf-8",
        )

    def _capture(self) -> dict[str, object]:
        return capture_evidence(
            config_path=self.config_path,
            elf_path=self.elf_path,
            host_transfer_path=self.host_transfer_path,
            log_path=self.log_path,
            source_paths=(self.source_path,),
        )

    def test_hash_binds_exact_frame_config_elf_and_target_log(self) -> None:
        report = self._capture()

        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["frame_bytes"], 97)
        hashes = cast(dict[str, object], report["sha256"])
        self.assertEqual(
            hashes["elf"], hashlib.sha256(self.elf_path.read_bytes()).hexdigest()
        )
        source_hashes = cast(dict[str, str], hashes["sources"])
        self.assertEqual(
            source_hashes[str(self.source_path)],
            hashlib.sha256(self.source_path.read_bytes()).hexdigest(),
        )

    def test_rejects_wrong_frame_even_if_target_log_claims_success(self) -> None:
        transfer = json.loads(self.host_transfer_path.read_text(encoding="utf-8"))
        transfer["frame_tx_hex"] = "00" * 97
        self.host_transfer_path.write_text(json.dumps(transfer), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "differ from the expected SCAN"):
            self._capture()

    def test_rejects_missing_spi2_binding(self) -> None:
        config = json.loads(self.config_path.read_text(encoding="utf-8"))
        config["target"]["bindings"].pop()
        self.config_path.write_text(json.dumps(config), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "invalid GVSOC configuration"):
            self._capture()

    def test_rejects_master_only_completion_without_target_receive_marker(self) -> None:
        self.log_path.write_text(
            "master transfer returned 97 bytes\n", encoding="utf-8"
        )

        with self.assertRaisesRegex(ValueError, "lacks the exact target-side"):
            self._capture()


if __name__ == "__main__":
    unittest.main()
