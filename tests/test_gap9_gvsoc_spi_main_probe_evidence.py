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
from scripts.gap9_gvsoc_spi_probe_protocol import (
    make_mapping_trace,
    make_pose_frame,
    make_scan_frame,
)


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

    def _write_mapping_evidence(self, response_rx_hex: str | None = None) -> str:
        response_was_skipped = response_rx_hex is None
        frames = make_mapping_trace()
        commands = [
            "POSE_BAD_CHECKSUM"
            if index == 0
            else {0: "POSE", 1: "SCAN", 2: "OPTIMIZE"}[frame[1]]
            for index, frame in enumerate(frames)
        ]
        self.host_transfer_path.write_text(
            json.dumps(
                {
                    "sequence": "mapping",
                    "pre_cs_dummy_clock_cycles": 8,
                    "frames": [
                        {
                            "command": command,
                            "frame_tx_hex": frame.hex(),
                            "length": len(frame),
                        }
                        for command, frame in zip(commands, frames, strict=True)
                    ],
                    "response_rx_hex": response_rx_hex,
                    "control_phase": (
                        "optimizer_completed_response_not_clocked"
                        if response_was_skipped
                        else "optimizer_response_captured"
                    ),
                }
            ),
            encoding="utf-8",
        )
        expected_response = "010002000300040005000600070008000900"
        marker = (
            "NANO_C_SLAM_GVSOC_SPI_MAIN:mapping_ok packets=66 "
            "pose_decoded=60 pose_inserted=60 bad_pose_checksum=1 "
            "scans_decoded=4 scans_registered=3 constraints=2 "
            "optimizer_edges=2 optimize=1 graph=3 solver_calls=3 "
            f"response_expected_hex={expected_response}"
        )
        stats = (
            "NANO_C_SLAM_SPI:stats rx=66 malformed=0 unknown=0 "
            "frames=61,4,1,0,0,0,0 decode_ok=60,4,0,0,0,0,0 "
            "decode_error=1,0,0,0,0,0,0 pose_id_missing=0 "
            "pose_id_nonadvancing=0 "
            f"optimize_responses={int(not response_was_skipped)}"
        )
        skipped_marker = (
            "NANO_C_SLAM_GVSOC_SPI_MAIN:response_phase=skipped\n"
            if response_was_skipped
            else ""
        )
        self.log_path.write_text(
            skipped_marker + stats + "\n" + marker + "\n", encoding="utf-8"
        )
        return expected_response

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

    def test_accepts_full_mapping_ingress_but_labels_response_as_unvalidated(
        self,
    ) -> None:
        expected_response = self._write_mapping_evidence()

        report = self._capture()

        self.assertEqual(report["sequence"], "mapping")
        self.assertEqual(report["frame_count"], 66)
        self.assertEqual(report["optimizer_response_expected_hex"], expected_response)
        self.assertEqual(report["optimizer_response_status"], "skipped by test target")
        diagnostics = cast(dict[str, object], report["spi_diagnostics"])
        self.assertEqual(diagnostics["received_frames"], 66)
        self.assertEqual(diagnostics["completed_optimize_responses"], 0)
        self.assertTrue(
            any(
                "skips the SPI slave TX" in limitation
                for limitation in cast(list[str], report["limitations"])
            )
        )

    def test_rejects_claimed_response_bytes_when_target_skips_slave_tx(self) -> None:
        self._write_mapping_evidence("00" * 18)

        with self.assertRaisesRegex(ValueError, "differ from target slam_job_t"):
            self._capture()

    def test_accepts_captured_matching_optimizer_response(self) -> None:
        expected_response = self._write_mapping_evidence(
            "010002000300040005000600070008000900"
        )
        transfer = json.loads(self.host_transfer_path.read_text(encoding="utf-8"))
        transfer["control_phase"] = "optimizer_response_captured"
        transfer["response_rx_hex"] = expected_response
        self.host_transfer_path.write_text(json.dumps(transfer), encoding="utf-8")
        report = self._capture()

        self.assertEqual(
            report["optimizer_response_status"],
            "captured and matched target slam_job_t",
        )
        diagnostics = cast(dict[str, object], report["spi_diagnostics"])
        self.assertEqual(diagnostics["completed_optimize_responses"], 1)

    def test_rejects_missing_production_receive_diagnostics(self) -> None:
        self._write_mapping_evidence()
        lines = [
            line
            for line in self.log_path.read_text(encoding="utf-8").splitlines()
            if not line.startswith("NANO_C_SLAM_SPI:stats ")
        ]
        self.log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "lacks production SPI receive"):
            self._capture()

    def test_rejects_inconsistent_production_receive_counts(self) -> None:
        self._write_mapping_evidence()
        log = self.log_path.read_text(encoding="utf-8")
        self.log_path.write_text(log.replace("frames=61,4,1", "frames=60,4,1"))

        with self.assertRaisesRegex(ValueError, "diagnostics differ"):
            self._capture()


if __name__ == "__main__":
    unittest.main()
