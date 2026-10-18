"""Adversarial checks for real-SPI production-main evidence capture."""

from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from typing import cast

from scripts.capture_gap9_gvsoc_spi_main_probe_evidence import (
    TARGET_MARKER,
    capture_evidence,
)
from scripts.gap9_gvsoc_spi_probe_protocol import make_pose_frame, make_scan_frame


class Gap9GvsocSpiMainProbeEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.config_path = self.root / "config.json"
        self.elf_path = self.root / "probe.elf"
        self.host_transfer_path = self.root / "host.json"
        self.log_path = self.root / "gvsoc.log"
        self.source_path = self.root / "main.c"
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
        self.elf_path.write_bytes(b"test ELF")
        self.source_path.write_text("int main(void) { return 0; }\n", encoding="utf-8")
        self._write_host_transfer((make_pose_frame(), make_scan_frame()))
        self.log_path.write_text(TARGET_MARKER + "\n", encoding="utf-8")

    def _write_host_transfer(self, frames: tuple[bytes, ...]) -> None:
        commands = ("POSE", "SCAN")
        transfer = {
            "pre_cs_dummy_clock_cycles": 8,
            "frames": [
                {"command": command, "frame_tx_hex": frame.hex(), "length": len(frame)}
                for command, frame in zip(commands, frames, strict=True)
            ],
        }
        self.host_transfer_path.write_text(json.dumps(transfer), encoding="utf-8")

    def _capture(self) -> dict[str, object]:
        return capture_evidence(
            config_path=self.config_path,
            elf_path=self.elf_path,
            host_transfer_path=self.host_transfer_path,
            log_path=self.log_path,
            source_paths=(self.source_path,),
        )

    def test_hash_binds_frames_target_log_and_owned_source(self) -> None:
        report = self._capture()

        self.assertEqual(report["status"], "passed")
        hashes = cast(dict[str, object], report["sha256"])
        self.assertEqual(hashes["elf"], hashlib.sha256(b"test ELF").hexdigest())
        self.assertEqual(
            hashes["gvsoc_log"], hashlib.sha256(self.log_path.read_bytes()).hexdigest()
        )
        sources = cast(dict[str, str], hashes["sources"])
        self.assertEqual(
            sources[str(self.source_path)],
            hashlib.sha256(self.source_path.read_bytes()).hexdigest(),
        )

    def test_rejects_wrong_or_reordered_host_frames(self) -> None:
        self._write_host_transfer((make_scan_frame(), make_pose_frame()))

        with self.assertRaisesRegex(ValueError, "bytes/order"):
            self._capture()

    def test_rejects_master_only_completion(self) -> None:
        self.log_path.write_text(
            "master transfer returned 97 bytes\n", encoding="utf-8"
        )

        with self.assertRaisesRegex(ValueError, "target-side production-main"):
            self._capture()

    def test_rejects_missing_testbench_binding(self) -> None:
        config = json.loads(self.config_path.read_text(encoding="utf-8"))
        config["target"]["bindings"].pop()
        self.config_path.write_text(json.dumps(config), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "invalid GVSOC configuration"):
            self._capture()


if __name__ == "__main__":
    unittest.main()
