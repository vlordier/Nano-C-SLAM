"""Adversarial tests for run-bound GAP9 map geometry measurements."""

from __future__ import annotations

import hashlib
import json
import shutil
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

from scripts.analyze_gap9_target_mapping_geometry import analyze_mapping
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_model import (
    MappingSample,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_packets import (
    SCAN_CMD,
    encode_mapping_samples,
)


def _float_bits(value: float) -> str:
    bits = struct.unpack("<I", struct.pack("<f", value))[0]
    return f"{bits:08x}"


def _target_log(
    samples: tuple[MappingSample, ...],
    *,
    pose_x_offset: float = 0.0,
    pose_yaws: dict[int, float] | None = None,
) -> tuple[str, list[dict[str, float | int]], str, str]:
    packets = encode_mapping_samples(samples)
    scan_ids = [packet.pose_id for packet in packets if packet.command == SCAN_CMD]
    response_hex = struct.pack("<9h", len(samples), 3, 21, 128, 0, 1, 1, 3, 1).hex()
    optimized_poses: list[dict[str, float | int]] = []
    lines = [
        f"NANO_C_SLAM_GVSOC_SPI_MAIN:gazebo_capture_ok packets={len(packets)} "
        f"pose_decoded={len(samples)} pose_inserted={len(samples)} "
        f"scans_decoded={len(scan_ids)} scans_registered={len(scan_ids)} "
        f"constraints=2 optimize=1 poses={len(samples)} graph=3 solver_calls=1 "
        f"has_nan=0 completed=1 response_expected_hex={response_hex}",
        f"NANO_C_SLAM_GVSOC_SPI_MAIN:pose_dump_start count={len(samples)}",
    ]
    for sample in samples:
        target_x = sample.x_m + pose_x_offset
        target_yaw = (pose_yaws or {}).get(sample.sample_id, sample.yaw_rad)
        pose = {
            "pose_id": sample.sample_id,
            "x_m": struct.unpack("<f", struct.pack("<f", target_x))[0],
            "y_m": struct.unpack("<f", struct.pack("<f", sample.y_m))[0],
            "z_m": struct.unpack("<f", struct.pack("<f", target_yaw))[0],
        }
        optimized_poses.append(pose)
        lines.append(
            f"NANO_C_SLAM_GVSOC_SPI_MAIN:optimized_pose id={sample.sample_id} "
            f"x_bits={_float_bits(target_x)} y_bits={_float_bits(sample.y_m)} "
            f"z_bits={_float_bits(target_yaw)} finite=1"
        )
    lines.append(
        f"NANO_C_SLAM_GVSOC_SPI_MAIN:pose_dump_done count={len(samples)} "
        f"finite={len(samples)}"
    )
    lines.append(
        "NANO_C_SLAM_GVSOC_SPI_MAIN:map_dump_start count=2 range_limit_mm=1400"
    )
    points = [
        {
            "point_id": point_id,
            "x_m": struct.unpack("<f", struct.pack("<f", 1.4))[0],
            "y_m": 0.0,
        }
        for point_id in range(2)
    ]
    lines.extend(
        (
            f"NANO_C_SLAM_GVSOC_SPI_MAIN:map_point id={point['point_id']} "
            f"x_bits={_float_bits(float(point['x_m']))} "
            f"y_bits={_float_bits(float(point['y_m']))} finite=1"
        )
        for point in points
    )
    lines.extend(
        (
            "NANO_C_SLAM_GVSOC_SPI_MAIN:map_dump_done count=2 finite=2",
            *(
                "NANO_C_SLAM_GVSOC_SPI_MAIN:scan_busy "
                f"index={index} pose_id={pose_id} duration_us={1000 + index}"
                for index, pose_id in enumerate(scan_ids)
            ),
            "NANO_C_SLAM_SPI:stats "
            f"rx={len(packets)} malformed=0 unknown=0 "
            f"frames={len(samples)},{len(scan_ids)},1,0,0,0,0 "
            f"decode_ok={len(samples)},{len(scan_ids)},0,0,0,0,0 "
            "decode_error=0,0,0,0,0,0,0 pose_id_missing=0 "
            f"pose_id_nonadvancing=0 optimize_responses=1",
        )
    )
    marker = lines[0]
    return "\n".join(lines) + "\n", optimized_poses, marker, response_hex


def _write_nearfield_fixture(path: Path) -> None:
    positions = {
        "wall1": (1.5, 0, 0.2, 3, 0),
        "wall2": (-1.5, 0, 0.2, 3, 0),
        "wall3": (0, 0.95, 0.2, 3, 1.57079632679),
        "wall4": (0, -1.5, 0.2, 3, 1.57079632679),
        "obstacle1": (0.65, -0.15, 0.3, 0.8, 0),
        "obstacle2": (-0.55, -1.0, 0.4, 0.8, 0),
        "obstacle3": (-1.0, -0.55, 0.6, 0.3, 0),
    }
    models = "".join(
        f'<model name="{name}"><static>true</static>'
        f"<pose>{x} {y} 0 0 0 {yaw}</pose>"
        f'<link name="link"><collision name="collision"><geometry>'
        f"<box><size>{width} {height} 1</size></box></geometry></collision>"
        f"</link></model>"
        for name, (x, y, width, height, yaw) in positions.items()
    )
    path.write_text(f"<sdf version='1.9'><world name='test'>{models}</world></sdf>")


class Gap9TargetMappingGeometryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.run_id = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
        self.session_id = "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"
        self.sdf_path = self.root / "nearfield.sdf"
        self.launched_sdf_path = self.root / "launched-nearfield.sdf"
        self.session_path = self.root / "session.json"
        self.world_attestation_path = self.root / "world-attestation.json"
        self.run_path = self.root / "run.json"
        self.target_path = self.root / "target.json"
        self.log_path = self.root / "gvsoc.log"
        self.elf_path = self.root / "target.elf"
        self.config_path = self.root / "gvsoc-config.json"
        self.host_transfer_path = self.root / "host-transfer.json"
        self.source_path = self.root / "main.c"
        self.elf_path.write_bytes(b"fixture GAP9 ELF")
        self.config_path.write_text(
            json.dumps(
                {
                    "target": {
                        "addon_testbench_enabled": True,
                        "components": ["testbench"],
                        "bindings": [
                            ["chip->spim2_cs0_data", "testbench->spi2_cs0_data"],
                            ["chip->spim2_cs0", "testbench->spi2_cs0"],
                        ],
                    }
                }
            ),
            encoding="utf-8",
        )
        self.source_path.write_text("int main(void) { return 0; }\n", encoding="utf-8")
        self.samples = tuple(
            MappingSample(
                sample_id=index,
                timestamp_ms=index + 1,
                x_m=index * 0.05,
                y_m=0.0,
                yaw_rad=0.0,
                ranges_mm=(1000, 1000, 1000, 1000),
                source_stamp_ns=(index + 1) * 1_000_000,
            )
            for index in range(20)
        )
        _write_nearfield_fixture(self.sdf_path)
        shutil.copyfile(self.sdf_path, self.launched_sdf_path)
        self.session_path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "session_id": self.session_id,
                    "scene": "nearfield-mapping",
                    "processes": [
                        {
                            "role": "gazebo-server",
                            "argv": [
                                "gz",
                                "sim",
                                str(self.launched_sdf_path.resolve()),
                            ],
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        sdf_hash = self._sha(self.sdf_path)
        self.world_attestation_path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "session_id": self.session_id,
                    "session_sha256": self._sha(self.session_path),
                    "gazebo_server_sdf_argument": str(self.launched_sdf_path.resolve()),
                    "world_sdf_sha256": sdf_hash,
                    "archived_sdf_sha256": sdf_hash,
                    "verified_while_server_active": True,
                }
            ),
            encoding="utf-8",
        )
        self._write_run_and_target()

    def _write_run_and_target(
        self,
        *,
        pose_x_offset: float = 0.0,
        pose_yaws: dict[int, float] | None = None,
    ) -> None:
        target_log, target_poses, marker, response_hex = _target_log(
            self.samples, pose_x_offset=pose_x_offset, pose_yaws=pose_yaws
        )
        self.log_path.write_text(target_log, encoding="utf-8")
        records = [sample.to_record() for sample in self.samples]
        provenance = [
            {
                "drone_id": "cf1",
                "range_age_ns": 0,
                "odometry_age_ns": 0,
                "pair_skew_ns": 0,
                "odometry_stamp_ns": sample.source_stamp_ns,
                "range_stamp_ns": sample.source_stamp_ns,
                "range_sequence": sample.sample_id + 1,
            }
            for sample in self.samples
        ]
        packets = encode_mapping_samples(self.samples)
        scan_ids = [packet.pose_id for packet in packets if packet.command == SCAN_CMD]
        host_transfer = {
            "sequence": "gazebo-live",
            "run_id": self.run_id,
            "drone_id": "cf1",
            "pre_cs_dummy_clock_cycles": 8,
            "frames": [
                {
                    "sequence": packet.sequence,
                    "command": packet.command,
                    "pose_id": packet.pose_id,
                    "source_stamp_ns": packet.source_stamp_ns,
                    "frame_tx_hex": packet.frame.hex(),
                    "length": len(packet.frame),
                }
                for packet in packets
            ],
            "command_transfer_lengths": [
                {
                    "master_tx_bytes": len(packet.frame),
                    "master_rx_bytes": len(packet.frame),
                }
                for packet in packets
            ],
            "control_phase": "optimizer_response_captured",
            "response_rx_hex": response_hex,
        }
        self.host_transfer_path.write_text(json.dumps(host_transfer), encoding="utf-8")
        self.run_path.write_text(
            json.dumps(
                {
                    "run_id": self.run_id,
                    "drone": "cf1",
                    "scene": "nearfield-mapping",
                    "gazebo_world_session_id": self.session_id,
                    "capture_report": {
                        "run_id": self.run_id,
                        "drone": "cf1",
                        "evidence_class": "host-execution-of-owned-gap9-mapping-core",
                        "input_sample_count": len(records),
                        "input_samples": records,
                        "pairing_limits": {
                            "maximum_age_ns": 100_000_000,
                            "maximum_skew_ns": 100_000_000,
                        },
                        "input_pair_provenance": provenance,
                    },
                    "mission_terminal": {
                        "run_id": self.run_id,
                        "drone_name": "cf1",
                        "phase": "COMPLETE",
                        "mapping_path_complete": True,
                    },
                    "process_exit_codes": {"collector": 0, "mission": 0},
                }
            ),
            encoding="utf-8",
        )
        command_counts = {
            "pose": len(self.samples),
            "scan": len(scan_ids),
            "optimize": 1,
            "pose_request": 0,
            "check": 0,
            "external_pose": 0,
            "pose_update": 0,
        }
        decode_success = {**command_counts, "optimize": 0}
        self.target_path.write_text(
            json.dumps(
                {
                    "status": "passed",
                    "evidence_class": (
                        "gvsoc-firmware-ipc-stream-of-inflight-gazebo-telemetry"
                    ),
                    "sequence": "gazebo-live",
                    "frame_count": len(packets),
                    "frames": [packet.frame.hex() for packet in packets],
                    "target_marker": marker,
                    "gazebo_source": {
                        "run_id": self.run_id,
                        "drone": "cf1",
                        "sample_count": len(self.samples),
                        "mission_terminal_phase": "COMPLETE",
                        "process_exit_codes": {"collector": 0, "mission": 0},
                        "run_bundle_sha256": self._sha(self.run_path),
                    },
                    "sha256": {
                        "elf": self._sha(self.elf_path),
                        "gvsoc_config": self._sha(self.config_path),
                        "host_transfer": self._sha(self.host_transfer_path),
                        "gvsoc_log": self._sha(self.log_path),
                        "sources": {
                            "cslam-gap9-app/main.c": self._sha(self.source_path)
                        },
                    },
                    "optimizer_response_expected_hex": response_hex,
                    "optimizer_response_rx_hex": response_hex,
                    "optimizer_response_status": (
                        "captured and matched target slam_job_t"
                    ),
                    "mapping_outcome": (
                        "gazebo_telemetry_streamed_over_ipc; optimizer_completed"
                    ),
                    "slam_job_fields": {
                        "number_of_poses": len(self.samples),
                        "main_graph_size": 3,
                        "max_subgraph_size": 21,
                        "time_ms": 128,
                        "has_nan": 0,
                        "main_solver_returned": 1,
                        "successful_solver_call_count": 1,
                        "completed_solver_iteration_count": 3,
                        "optimization_completed": 1,
                    },
                    "spi_diagnostics": {
                        "received_frames": len(packets),
                        "malformed_frames": 0,
                        "unknown_commands": 0,
                        "command_frames": command_counts,
                        "decode_success": decode_success,
                        "decode_failure": dict.fromkeys(command_counts, 0),
                        "missing_pose_ids": 0,
                        "nonadvancing_pose_ids": 0,
                        "completed_optimize_responses": 1,
                    },
                    "scan_busy_durations_us": [
                        {
                            "index": index,
                            "pose_id": pose_id,
                            "duration_us": 1000 + index,
                        }
                        for index, pose_id in enumerate(scan_ids)
                    ],
                    "target_optimized_poses": target_poses,
                    "target_projected_map_points": [
                        {
                            "point_id": point_id,
                            "x_m": struct.unpack("<f", struct.pack("<f", 1.4))[0],
                            "y_m": 0.0,
                        }
                        for point_id in range(2)
                    ],
                }
            ),
            encoding="utf-8",
        )

    def _analyze(self) -> dict[str, object]:
        return analyze_mapping(
            target_report_path=self.target_path,
            target_log_path=self.log_path,
            target_elf_path=self.elf_path,
            gvsoc_config_path=self.config_path,
            host_transfer_path=self.host_transfer_path,
            run_bundle_path=self.run_path,
            session_path=self.session_path,
            world_attestation_path=self.world_attestation_path,
            sdf_path=self.sdf_path,
        )

    def test_bound_target_outputs_produce_direct_pose_and_wall_metrics(self) -> None:
        report = self._analyze()

        self.assertEqual(report["status"], "measured-no-acceptance-threshold")
        claims = report["validation_claims"]
        self.assertEqual(claims["target_optimizer_health"], "validated")
        self.assertEqual(claims["sensor_model_fidelity"], "not-validated")
        self.assertEqual(claims["stm32_flight_firmware"], "not-validated")
        pose_error = report["pose_error"]
        self.assertAlmostEqual(pose_error["xy_rmse_m"], 0.0, places=6)
        self.assertAlmostEqual(pose_error["yaw_max_rad"], 0.0, places=8)
        self.assertEqual(pose_error["per_pose"][0]["pose_id"], 0)
        self.assertAlmostEqual(pose_error["per_pose"][1]["yaw_error_rad"], 0.0)
        self.assertAlmostEqual(pose_error["source_xy_bounds"]["span_x_m"], 0.95)
        self.assertAlmostEqual(pose_error["target_xy_bounds"]["span_x_m"], 0.95)
        map_metrics = report["target_map"]["sdf_surface_distance_m"]
        self.assertGreaterEqual(map_metrics["within_0_05m_fraction"], 0.0)
        self.assertLess(map_metrics["max"], 1e-6)
        self.assertAlmostEqual(report["target_map"]["xy_bounds"]["min_x_m"], 1.4)
        self.assertFalse(report["target_map"]["capacity_reached"])
        self.assertEqual(report["geometry"]["source_sdf_sha256"], self._sha_sdf())

    def test_yaw_error_uses_shortest_arc_across_wrap_boundary(self) -> None:
        self.samples = tuple(
            MappingSample(
                sample_id=sample.sample_id,
                timestamp_ms=sample.timestamp_ms,
                x_m=sample.x_m,
                y_m=sample.y_m,
                yaw_rad=3.1 if sample.sample_id == 1 else sample.yaw_rad,
                ranges_mm=sample.ranges_mm,
                source_stamp_ns=sample.source_stamp_ns,
            )
            for sample in self.samples
        )
        self._write_run_and_target(pose_yaws={1: -3.0})

        report = self._analyze()

        self.assertAlmostEqual(
            report["pose_error"]["per_pose"][1]["yaw_error_rad"],
            0.183185307,
            places=5,
        )

    def test_translation_is_reported_instead_of_fitted_away(self) -> None:
        self._write_run_and_target(pose_x_offset=0.25)

        report = self._analyze()

        self.assertAlmostEqual(report["pose_error"]["xy_rmse_m"], 0.25, places=6)
        self.assertAlmostEqual(report["pose_error"]["xy_max_m"], 0.25, places=6)
        self.assertAlmostEqual(
            report["pose_error"]["per_pose"][0]["xy_error_m"], 0.25, places=6
        )
        self.assertAlmostEqual(
            report["pose_error"]["target_to_source_path_length_ratio"],
            1.0,
            places=6,
        )

    def test_target_map_at_firmware_capacity_is_not_claimed_complete(self) -> None:
        lines = self.log_path.read_text(encoding="utf-8").splitlines()
        points = [
            f"NANO_C_SLAM_GVSOC_SPI_MAIN:map_point id={index} "
            f"x_bits={_float_bits(1.4)} y_bits={_float_bits(0.0)} finite=1"
            for index in range(700)
        ]
        lines = [line for line in lines if ":map_point " not in line]
        lines = [
            line.replace("map_dump_start count=2", "map_dump_start count=700").replace(
                "map_dump_done count=2 finite=2",
                "map_dump_done count=700 finite=700",
            )
            for line in lines
        ]
        done_index = next(i for i, line in enumerate(lines) if ":map_dump_done" in line)
        lines[done_index:done_index] = points
        self.log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        target = json.loads(self.target_path.read_text())
        target["target_projected_map_points"] = [
            {"point_id": index, "x_m": 1.399999976158142, "y_m": 0.0}
            for index in range(700)
        ]
        target["sha256"]["gvsoc_log"] = self._sha(self.log_path)
        self.target_path.write_text(json.dumps(target), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "SIZE_POINTS capacity"):
            self._analyze()

    def test_nonfinite_source_pose_is_rejected_before_metric_calculation(self) -> None:
        run = json.loads(self.run_path.read_text())
        run["capture_report"]["input_samples"][1] = "1 2 nan 0 0.1 1000 1000 1000 1000"
        self.run_path.write_text(json.dumps(run), encoding="utf-8")
        target = json.loads(self.target_path.read_text())
        target["gazebo_source"]["run_bundle_sha256"] = self._sha(self.run_path)
        self.target_path.write_text(json.dumps(target), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "pose coordinates must be finite"):
            self._analyze()

    def test_optimizer_report_fields_must_match_raw_target_response(self) -> None:
        target = json.loads(self.target_path.read_text())
        target["slam_job_fields"]["has_nan"] = 1
        self.target_path.write_text(json.dumps(target), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "optimizer fields differ"):
            self._analyze()

    def test_spi_diagnostics_must_match_injected_packet_counts(self) -> None:
        target = json.loads(self.target_path.read_text())
        target["spi_diagnostics"]["command_frames"]["pose"] -= 1
        self.target_path.write_text(json.dumps(target), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "SPI diagnostics differ"):
            self._analyze()

    def test_target_elf_must_match_the_capture_hash(self) -> None:
        self.elf_path.write_bytes(self.elf_path.read_bytes() + b"changed")

        with self.assertRaisesRegex(ValueError, "target ELF, config, transfer"):
            self._analyze()

    def test_host_transfer_is_checked_against_same_run_frames(self) -> None:
        transfer = json.loads(self.host_transfer_path.read_text())
        transfer["drone_id"] = "cf2"
        self.host_transfer_path.write_text(json.dumps(transfer), encoding="utf-8")
        target = json.loads(self.target_path.read_text())
        target["sha256"]["host_transfer"] = self._sha(self.host_transfer_path)
        self.target_path.write_text(json.dumps(target), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "host SPI transfer differs"):
            self._analyze()

    def test_raw_spi_log_cannot_disagree_with_target_evidence_report(self) -> None:
        log = self.log_path.read_text(encoding="utf-8")
        self.log_path.write_text(
            log.replace("frames=20,1,1", "frames=19,1,1"), encoding="utf-8"
        )
        target = json.loads(self.target_path.read_text())
        target["sha256"]["gvsoc_log"] = self._sha(self.log_path)
        self.target_path.write_text(json.dumps(target), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "raw target SPI diagnostics"):
            self._analyze()

    def test_stale_source_pair_provenance_is_rejected(self) -> None:
        run = json.loads(self.run_path.read_text())
        run["capture_report"]["input_pair_provenance"][1]["odometry_stamp_ns"] = 1
        self.run_path.write_text(json.dumps(run), encoding="utf-8")
        target = json.loads(self.target_path.read_text())
        target["gazebo_source"]["run_bundle_sha256"] = self._sha(self.run_path)
        self.target_path.write_text(json.dumps(target), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "stale or mismatched"):
            self._analyze()

    def test_duplicate_sdf_surface_model_is_rejected(self) -> None:
        tree = ET.parse(self.sdf_path)  # noqa: S314 - locally generated test fixture
        world = tree.getroot().find("world")
        assert world is not None
        wall = world.find("model[@name='wall1']")
        assert wall is not None
        world.append(deepcopy(wall))
        tree.write(self.sdf_path, encoding="utf-8")
        self._refresh_sdf_attestation()

        with self.assertRaisesRegex(ValueError, "duplicate surface model wall1"):
            self._analyze()

    def test_moving_or_offset_sdf_surface_model_is_rejected(self) -> None:
        tree = ET.parse(self.sdf_path)  # noqa: S314 - locally generated test fixture
        wall = tree.getroot().find("world/model[@name='wall1']")
        assert wall is not None
        wall.find("static").text = "false"
        tree.write(self.sdf_path, encoding="utf-8")
        self._refresh_sdf_attestation()

        with self.assertRaisesRegex(ValueError, "must be static"):
            self._analyze()

        wall.find("static").text = "true"
        link = wall.find("link")
        assert link is not None
        ET.SubElement(link, "pose").text = "0.25 0 0 0 0 0"
        tree.write(self.sdf_path, encoding="utf-8")
        self._refresh_sdf_attestation()

        with self.assertRaisesRegex(ValueError, "unsupported relative link geometry"):
            self._analyze()

    def _refresh_sdf_attestation(self) -> None:
        shutil.copyfile(self.sdf_path, self.launched_sdf_path)
        session = json.loads(self.session_path.read_text())
        session["world_sdf_path"] = str(self.launched_sdf_path.resolve())
        session["world_sdf_sha256"] = self._sha(self.sdf_path)
        session["world_sdf_bytes"] = self.sdf_path.stat().st_size
        self.session_path.write_text(json.dumps(session), encoding="utf-8")
        attestation = json.loads(self.world_attestation_path.read_text())
        attestation["session_sha256"] = self._sha(self.session_path)
        attestation["world_sdf_sha256"] = self._sha(self.sdf_path)
        attestation["archived_sdf_sha256"] = self._sha(self.sdf_path)
        self.world_attestation_path.write_text(
            json.dumps(attestation), encoding="utf-8"
        )

    def test_mixed_run_or_world_identity_is_rejected(self) -> None:
        session = json.loads(self.session_path.read_text())
        session["session_id"] = "cccccccc-cccc-4ccc-8ccc-cccccccccccc"
        self.session_path.write_text(json.dumps(session), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "identities or scenes"):
            self._analyze()

        session["session_id"] = self.session_id
        session["processes"][0]["argv"][2] = str(self.root / "other.sdf")
        self.session_path.write_text(json.dumps(session), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "world input attestation"):
            self._analyze()

    def test_modified_archived_sdf_is_rejected_against_live_server_input(self) -> None:
        self.sdf_path.write_text(self.sdf_path.read_text() + "<!-- changed -->")
        with self.assertRaisesRegex(ValueError, "world input attestation"):
            self._analyze()

    def test_retained_hash_attestation_allows_validation_after_temp_world_removal(
        self,
    ) -> None:
        self.launched_sdf_path.unlink()
        report = self._analyze()
        self.assertEqual(report["world_session_id"], self.session_id)

    def test_incomplete_mission_and_missing_target_point_are_rejected(self) -> None:
        target = json.loads(self.target_path.read_text())
        target["gazebo_source"]["mission_terminal_phase"] = "FAILED"
        self.target_path.write_text(json.dumps(target), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "incomplete or does not match"):
            self._analyze()

        target["gazebo_source"]["mission_terminal_phase"] = "COMPLETE"
        self.target_path.write_text(json.dumps(target), encoding="utf-8")
        self.log_path.write_text(
            self.log_path.read_text(encoding="utf-8").replace(
                "map_point id=1", "lost id=1"
            ),
            encoding="utf-8",
        )
        target["sha256"]["gvsoc_log"] = self._sha(self.log_path)
        self.target_path.write_text(json.dumps(target), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "every point"):
            self._analyze()

    def _sha_sdf(self) -> str:
        return self._sha(self.sdf_path)

    @staticmethod
    def _sha(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == "__main__":
    unittest.main()
