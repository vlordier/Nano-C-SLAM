"""Adversarial checks for real-SPI production-main evidence capture."""

from __future__ import annotations

import hashlib
import json
import struct
import tempfile
import unittest
from collections.abc import Sequence
from pathlib import Path
from typing import cast

from scripts.capture_gap9_gvsoc_spi_main_probe_evidence import (
    TARGET_MARKER,
    capture_evidence,
)
from scripts.gap9_gvsoc_spi_probe_protocol import (
    make_mapping_trace,
    make_pose_frame,
    make_pose_request_frame,
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

    @staticmethod
    def _transfer_lengths(frames: Sequence[bytes]) -> list[dict[str, int]]:
        return [
            {"master_tx_bytes": len(frame), "master_rx_bytes": len(frame)}
            for frame in frames
        ]

    @staticmethod
    def _scan_busy_lines(frames: tuple[bytes, ...]) -> str:
        scan_ids = [
            int.from_bytes(frame[2:4], "little", signed=True)
            for frame in frames
            if frame[1] == 1
        ]
        return "".join(
            "NANO_C_SLAM_GVSOC_SPI_MAIN:scan_busy "
            f"index={index} pose_id={pose_id} duration_us={1000 + index}\n"
            for index, pose_id in enumerate(scan_ids)
        )

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
                    "command_transfer_lengths": self._transfer_lengths(frames),
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
        expected_response = struct.pack("<9h", 60, 3, 21, 128, 0, 1, 3, 9, 1).hex()
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
            self._scan_busy_lines(frames)
            + skipped_marker
            + stats
            + "\n"
            + marker
            + "\n",
            encoding="utf-8",
        )
        return expected_response

    def _write_pose_request_evidence(
        self,
        response_hex: str | None = None,
        *,
        include_invalid_requests: bool = False,
    ) -> None:
        positions = ((0.25, -0.5, 0.75), (1.25, -1.5, 1.75))
        requests = ((0, 0), (0, -1), (0, 101), (2, 1), (0, 2))
        frames = [
            make_pose_frame(0, position=positions[0]),
            make_pose_frame(1, position=positions[1]),
        ]
        commands = ["POSE", "POSE"]
        if include_invalid_requests:
            frames.extend(
                make_pose_request_frame(from_id, count) for from_id, count in requests
            )
            commands.extend(
                [
                    "POSE_REQ_ZERO",
                    "POSE_REQ_NEGATIVE",
                    "POSE_REQ_OVER_CAPACITY",
                    "POSE_REQ_UNAVAILABLE_RANGE",
                    "POSE_REQ_VALID",
                ]
            )
        else:
            frames.append(make_pose_request_frame(0, 2))
            commands.append("POSE_REQ")
        sequence = (
            "pose-request-boundaries" if include_invalid_requests else "pose-request"
        )
        self.host_transfer_path.write_text(
            json.dumps(
                {
                    "sequence": sequence,
                    "pre_cs_dummy_clock_cycles": 8,
                    "frames": [
                        {
                            "command": command,
                            "frame_tx_hex": frame.hex(),
                            "length": len(frame),
                        }
                        for command, frame in zip(commands, frames, strict=True)
                    ],
                    "command_transfer_lengths": self._transfer_lengths(frames),
                    "response_size": 24,
                    "response_rx_hex": response_hex
                    or struct.pack("<6f", *positions[0], *positions[1]).hex(),
                    "control_phase": "pose_response_captured",
                    "rejected_requests": [
                        {"from_id": from_id, "count": count}
                        for from_id, count in requests[:-1]
                    ]
                    if include_invalid_requests
                    else None,
                }
            ),
            encoding="utf-8",
        )
        if include_invalid_requests:
            duration_us = 1279
            rejection_markers = "".join(
                "NANO_C_SLAM_GVSOC_SPI_MAIN:pose_request_rejected "
                f"from={from_id} count={count}\n"
                for from_id, count in requests[:-1]
            )
            success_marker = (
                "NANO_C_SLAM_GVSOC_SPI_MAIN:pose_request_boundaries_ok packets=7 "
                "pose_decoded=2 pose_inserted=2 requests_decoded=5 "
                "requests_rejected=4 requests_served=1 poses=2\n"
            )
        else:
            duration_us = 109294
            rejection_markers = ""
            success_marker = (
                "NANO_C_SLAM_GVSOC_SPI_MAIN:pose_request_ok packets=3 "
                "pose_decoded=2 pose_inserted=2 requests_decoded=1 "
                "requests_served=1 poses=2\n"
            )
        self.log_path.write_text(
            rejection_markers + "NANO_C_SLAM_GVSOC_SPI_MAIN:pose_response_ready "
            "from=0 count=2 bytes=24 ready_latency_us=14\n"
            + "NANO_C_SLAM_GVSOC_SPI_MAIN:pose_response_done "
            + f"from=0 count=2 duration_us={duration_us}\n"
            + success_marker,
            encoding="utf-8",
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

    def test_pose_request_requires_ready_done_and_exact_pose_bytes(self) -> None:
        self._write_pose_request_evidence()

        report = self._capture()

        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["response_size"], 24)
        self.assertEqual(
            report["response_status"], "captured and matches both inserted float poses"
        )
        self.assertEqual(report["target_response_duration_us"], 109294)

    def test_pose_request_rejects_wrong_response_payload(self) -> None:
        self._write_pose_request_evidence("00" * 24)

        with self.assertRaisesRegex(ValueError, "differ from the two inserted poses"):
            self._capture()

    def test_pose_request_boundary_probe_requires_all_four_rejections(self) -> None:
        self._write_pose_request_evidence(include_invalid_requests=True)

        report = self._capture()

        self.assertEqual(report["sequence"], "pose-request-boundaries")
        self.assertIn("requests_rejected=4", str(report["target_marker"]))
        deadline = cast(dict[str, object], report["deadline_comparison"])
        self.assertEqual(deadline["target_decode_to_busy_high_us"], 14)
        self.assertEqual(deadline["observed_readiness_margin_us"], 99986)
        self.assertEqual(deadline["target_busy_high_to_response_return_us"], 1279)
        self.assertEqual(deadline["observed_simulator_margin_us"], 98721)

        self.log_path.write_text(
            self.log_path.read_text(encoding="utf-8").replace(
                "pose_request_rejected from=2 count=1\n", ""
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "rejection markers"):
            self._capture()

    def test_pose_request_boundary_probe_rejects_extra_ready_markers(self) -> None:
        self._write_pose_request_evidence(include_invalid_requests=True)
        ready_marker = (
            "NANO_C_SLAM_GVSOC_SPI_MAIN:pose_response_ready "
            "from=0 count=2 bytes=24 ready_latency_us=14\n"
        )
        self.log_path.write_text(
            self.log_path.read_text(encoding="utf-8") + ready_marker,
            encoding="utf-8",
        )

        with self.assertRaisesRegex(ValueError, "exactly one POSE_REQ ready"):
            self._capture()

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
        self.assertEqual(
            report["slam_job_fields"],
            {
                "number_of_poses": 60,
                "main_graph_size": 3,
                "max_subgraph_size": 21,
                "time_ms": 128,
                "has_nan": 0,
                "main_solver_returned": 1,
                "successful_solver_call_count": 3,
                "completed_solver_iteration_count": 9,
                "optimization_completed": 1,
            },
        )
        accounting = cast(dict[str, object], report["transaction_accounting"])
        self.assertEqual(accounting["completed_host_command_transfers"], 66)
        self.assertEqual(accounting["master_tx_bytes"], 66 * 97)
        self.assertEqual(accounting["master_rx_bytes"], 66 * 97)
        self.assertEqual(accounting["target_completed_receive_frames"], 66)
        self.assertTrue(
            any(
                "skips the SPI slave TX" in limitation
                for limitation in cast(list[str], report["limitations"])
            )
        )

    def test_accepts_max_capacity_optimizer_marker_and_records_runtime(self) -> None:
        frames = make_mapping_trace(260)
        commands = [
            "POSE_BAD_CHECKSUM"
            if index == 0
            else {0: "POSE", 1: "SCAN", 2: "OPTIMIZE"}[frame[1]]
            for index, frame in enumerate(frames)
        ]
        self.host_transfer_path.write_text(
            json.dumps(
                {
                    "sequence": "max-mapping",
                    "pre_cs_dummy_clock_cycles": 8,
                    "frames": [
                        {
                            "command": command,
                            "frame_tx_hex": frame.hex(),
                            "length": len(frame),
                        }
                        for command, frame in zip(commands, frames, strict=True)
                    ],
                    "command_transfer_lengths": self._transfer_lengths(frames),
                    "control_phase": "optimizer_completed_response_not_clocked",
                }
            ),
            encoding="utf-8",
        )
        marker = (
            "NANO_C_SLAM_GVSOC_SPI_MAIN:max_mapping_ok packets=276 "
            "pose_decoded=260 pose_inserted=260 bad_pose_checksum=1 "
            "scans_decoded=14 scans_registered=13 constraints=30 "
            "optimizer_edges=30 optimize=1 poses=260 graph=13 "
            "solver_calls=3 time_ms=4321 has_nan=0 completed=1 "
            "response_expected_hex="
            + struct.pack("<9h", 260, 13, 13, 4321, 0, 1, 3, 9, 1).hex()
        )
        stats = (
            "NANO_C_SLAM_SPI:stats rx=276 malformed=0 unknown=0 "
            "frames=261,14,1,0,0,0,0 decode_ok=260,14,0,0,0,0,0 "
            "decode_error=1,0,0,0,0,0,0 pose_id_missing=0 "
            "pose_id_nonadvancing=0 optimize_responses=0"
        )
        self.log_path.write_text(
            "NANO_C_SLAM_GVSOC_SPI_MAIN:response_phase=skipped\n"
            + self._scan_busy_lines(frames)
            + stats
            + "\n"
            + marker
            + "\n",
            encoding="utf-8",
        )

        report = self._capture()

        self.assertEqual(report["sequence"], "max-mapping")
        self.assertEqual(report["frame_count"], 276)
        self.assertEqual(report["optimizer_time_ms"], 4321)
        scan_durations = cast(list[dict[str, int]], report["scan_busy_durations_us"])
        self.assertEqual(len(scan_durations), 14)
        self.assertEqual(scan_durations[0]["duration_us"], 1000)
        self.assertEqual(
            report["evidence_class"],
            "gvsoc-production-max-capacity-mapping-spi2-ingress",
        )

    def test_accepts_scan_anchor_capacity_without_claiming_optimizer_coverage(
        self,
    ) -> None:
        frames = make_mapping_trace(
            260, scan_interval=2, periodic=True, scans_after_poses=True
        )
        commands = [
            "POSE_BAD_CHECKSUM"
            if index == 0
            else {0: "POSE", 1: "SCAN", 2: "OPTIMIZE"}[frame[1]]
            for index, frame in enumerate(frames)
        ]
        self.host_transfer_path.write_text(
            json.dumps(
                {
                    "sequence": "capacity-mapping",
                    "pre_cs_dummy_clock_cycles": 8,
                    "frames": [
                        {
                            "command": command,
                            "frame_tx_hex": frame.hex(),
                            "length": len(frame),
                        }
                        for command, frame in zip(commands, frames, strict=True)
                    ],
                    "command_transfer_lengths": self._transfer_lengths(frames),
                    "control_phase": "optimizer_response_captured",
                    "response_rx_hex": "040133000300840200000100330099000100",
                }
            ),
            encoding="utf-8",
        )
        response = "040133000300840200000100330099000100"
        marker = (
            "NANO_C_SLAM_GVSOC_SPI_MAIN:capacity_ingress_ok packets=393 "
            "pose_decoded=260 pose_inserted=260 bad_pose_checksum=1 "
            "scans_decoded=131 scans_registered=100 constraints=50 "
            "optimizer_edges=50 optimize=1 poses=260 graph=51 "
            f"solver_calls=51 time_ms=644 has_nan=0 completed=1 "
            f"response_expected_hex={response}"
        )
        stats = (
            "NANO_C_SLAM_SPI:stats rx=393 malformed=0 unknown=0 "
            "frames=261,131,1,0,0,0,0 decode_ok=260,131,0,0,0,0,0 "
            "decode_error=1,0,0,0,0,0,0 pose_id_missing=0 "
            "pose_id_nonadvancing=0 optimize_responses=1"
        )
        self.log_path.write_text(
            self._scan_busy_lines(frames) + stats + "\n" + marker + "\n",
            encoding="utf-8",
        )

        report = self._capture()

        self.assertEqual(report["sequence"], "capacity-mapping")
        self.assertEqual(report["frame_count"], 393)
        self.assertEqual(report["optimizer_time_ms"], 644)
        self.assertEqual(report["optimizer_graph_size"], 51)
        self.assertEqual(report["mapping_outcome"], "optimizer_completed")
        self.assertEqual(
            report["evidence_class"],
            "gvsoc-production-scan-and-constraint-capacity-spi2",
        )
        self.assertEqual(report["scan_busy_max_us"], 1130)
        self.assertTrue(
            any(
                "100-scan anchor" in limitation
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

    def test_rejects_incomplete_host_full_duplex_transfer(self) -> None:
        self._write_mapping_evidence()
        transfer = json.loads(self.host_transfer_path.read_text(encoding="utf-8"))
        transfer["command_transfer_lengths"][0]["master_rx_bytes"] -= 1
        self.host_transfer_path.write_text(json.dumps(transfer), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "TX/RX lengths"):
            self._capture()

    def test_rejects_slam_job_fields_that_disagree_with_target_marker(self) -> None:
        self._write_mapping_evidence()
        log = self.log_path.read_text(encoding="utf-8")
        log = log.replace("solver_calls=3", "solver_calls=4")
        self.log_path.write_text(log, encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "slam_job_t fields disagree"):
            self._capture()

    def test_rejects_missing_scan_busy_measurement(self) -> None:
        self._write_mapping_evidence()
        log = self.log_path.read_text(encoding="utf-8")
        lines = [
            line
            for line in log.splitlines()
            if not line.startswith("NANO_C_SLAM_GVSOC_SPI_MAIN:scan_busy ")
        ]
        self.log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "one SCAN busy duration"):
            self._capture()

    def test_rejects_scan_busy_measurement_for_wrong_pose_id(self) -> None:
        self._write_mapping_evidence()
        log = self.log_path.read_text(encoding="utf-8")
        self.log_path.write_text(
            log.replace("index=0 pose_id=0", "index=0 pose_id=1"),
            encoding="utf-8",
        )

        with self.assertRaisesRegex(ValueError, "pose IDs differ"):
            self._capture()

    def test_rejects_inconsistent_production_receive_counts(self) -> None:
        self._write_mapping_evidence()
        log = self.log_path.read_text(encoding="utf-8")
        self.log_path.write_text(log.replace("frames=61,4,1", "frames=60,4,1"))

        with self.assertRaisesRegex(ValueError, "diagnostics differ"):
            self._capture()


if __name__ == "__main__":
    unittest.main()
