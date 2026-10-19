"""Adversarial tests for run-bound GAP9 map geometry measurements."""

from __future__ import annotations

import hashlib
import json
import shutil
import struct
import tempfile
import unittest
from pathlib import Path

from scripts.analyze_gap9_target_mapping_geometry import analyze_mapping


def _float_bits(value: float) -> str:
    bits = struct.unpack("<I", struct.pack("<f", value))[0]
    return f"{bits:08x}"


def _target_log() -> str:
    lines = [
        "NANO_C_SLAM_GVSOC_SPI_MAIN:pose_dump_start count=2",
        "NANO_C_SLAM_GVSOC_SPI_MAIN:optimized_pose id=0 "
        f"x_bits={_float_bits(0.0)} y_bits={_float_bits(0.0)} "
        f"z_bits={_float_bits(0.0)} finite=1",
        "NANO_C_SLAM_GVSOC_SPI_MAIN:optimized_pose id=1 "
        f"x_bits={_float_bits(1.0)} y_bits={_float_bits(0.0)} "
        f"z_bits={_float_bits(0.1)} finite=1",
        "NANO_C_SLAM_GVSOC_SPI_MAIN:pose_dump_done count=2 finite=2",
        "NANO_C_SLAM_GVSOC_SPI_MAIN:map_dump_start count=2 range_limit_mm=1400",
        "NANO_C_SLAM_GVSOC_SPI_MAIN:map_point id=0 "
        f"x_bits={_float_bits(1.4)} y_bits={_float_bits(0.0)} finite=1",
        "NANO_C_SLAM_GVSOC_SPI_MAIN:map_point id=1 "
        f"x_bits={_float_bits(1.4)} y_bits={_float_bits(0.0)} finite=1",
        "NANO_C_SLAM_GVSOC_SPI_MAIN:map_dump_done count=2 finite=2",
    ]
    return "\n".join(lines) + "\n"


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
        f'<model name="{name}"><pose>{x} {y} 0 0 0 {yaw}</pose>'
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
        _write_nearfield_fixture(self.sdf_path)
        shutil.copyfile(self.sdf_path, self.launched_sdf_path)
        self.log_path.write_text(_target_log(), encoding="utf-8")
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
        self.run_path.write_text(
            json.dumps(
                {
                    "run_id": self.run_id,
                    "scene": "nearfield-mapping",
                    "gazebo_world_session_id": self.session_id,
                    "capture_report": {
                        "run_id": self.run_id,
                        "input_samples": [
                            "0 1 0 0 0 1000 1000 1000 1000",
                            "1 2 1 0 0.1 1000 1000 1000 1000",
                        ],
                    },
                    "mission_terminal": {
                        "run_id": self.run_id,
                        "phase": "COMPLETE",
                    },
                    "process_exit_codes": {"collector": 0, "mission": 0},
                }
            ),
            encoding="utf-8",
        )
        self.target_path.write_text(
            json.dumps(
                {
                    "status": "passed",
                    "sequence": "gazebo-live",
                    "gazebo_source": {
                        "run_id": self.run_id,
                        "mission_terminal_phase": "COMPLETE",
                        "process_exit_codes": {"collector": 0, "mission": 0},
                        "run_bundle_sha256": self._sha(self.run_path),
                    },
                    "sha256": {"gvsoc_log": self._sha(self.log_path)},
                    "target_optimized_poses": [
                        {"pose_id": 0, "x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
                        {
                            "pose_id": 1,
                            "x_m": 1.0,
                            "y_m": 0.0,
                            "z_m": 0.10000000149011612,
                        },
                    ],
                    "target_projected_map_points": [
                        {
                            "point_id": 0,
                            "x_m": 1.399999976158142,
                            "y_m": 0.0,
                        },
                        {
                            "point_id": 1,
                            "x_m": 1.399999976158142,
                            "y_m": 0.0,
                        },
                    ],
                }
            ),
            encoding="utf-8",
        )

    def _analyze(self) -> dict[str, object]:
        return analyze_mapping(
            target_report_path=self.target_path,
            target_log_path=self.log_path,
            run_bundle_path=self.run_path,
            session_path=self.session_path,
            world_attestation_path=self.world_attestation_path,
            sdf_path=self.sdf_path,
        )

    def test_bound_target_outputs_produce_direct_pose_and_wall_metrics(self) -> None:
        report = self._analyze()

        self.assertEqual(report["status"], "measured-no-acceptance-threshold")
        pose_error = report["pose_error"]
        self.assertEqual(pose_error["xy_rmse_m"], 0.0)
        self.assertAlmostEqual(pose_error["yaw_max_rad"], 0.0, places=8)
        map_metrics = report["target_map"]["sdf_surface_distance_m"]
        self.assertGreaterEqual(map_metrics["within_0_05m_fraction"], 0.0)
        self.assertLess(map_metrics["max"], 1e-6)
        self.assertEqual(report["geometry"]["source_sdf_sha256"], self._sha_sdf())

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
        self.log_path.write_text(_target_log().replace("map_point id=1", "lost id=1"))
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
