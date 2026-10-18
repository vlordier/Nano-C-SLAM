"""Tamper tests for canonical Gazebo mission run bundles."""

from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from scripts.gazebo_run_manifest import (
    create_gazebo_run_manifest,
    main,
    validate_gazebo_mapping_bundle,
)
from scripts.source_provenance import SourceIdentity, source_identity_to_mapping
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance import (
    OVERLAY_PACKAGES,
    OWNED_INPUT_PATHS,
)

RUN_ID = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
BINARY_HASH = "1" * 64


def _hash(contents: bytes) -> str:
    return hashlib.sha256(contents).hexdigest()


def _report() -> dict[str, object]:
    return {
        "schema_version": 2,
        "run_id": RUN_ID,
        "drone": "cf1",
        "scene": "nearfield-mapping",
        "child_commands": {
            "collector": [
                "/python",
                "-m",
                "scripts.gazebo_mapping_sitl",
                "--binary",
                "/mapping_replay",
            ],
            "mission": ["/ros2", "run", "nano_c_slam_gazebo", "sim_mission"],
        },
        "process_exit_codes": {"collector": 0, "mission": 0},
        "capture_report": {
            "run_id": RUN_ID,
            "mapping_replay_provenance": {"binary_sha256": BINARY_HASH},
        },
        "run_provenance": {
            "schema_version": 1,
            "source": {
                "git_revision": "a" * 40,
                "git_dirty": False,
                "git_status_porcelain": "",
                "owned_input_sha256": {path: "2" * 64 for path in OWNED_INPUT_PATHS},
            },
            "executables": {
                "python": {"path": "/python", "version": "3.12", "sha256": "3" * 64},
                "ros2": {
                    "path": "/ros2",
                    "ros_distro": "jazzy",
                    "cli_version": "0.32",
                    "sha256": "4" * 64,
                },
                "gazebo": {"path": "/gz", "sim_version": "8.0", "sha256": "5" * 64},
                "mapping_binary": {"path": "/mapping_replay", "sha256": BINARY_HASH},
            },
            "ros_overlay_packages": {
                package: {
                    "prefix": f"/overlay/{package}",
                    "sha256": "6" * 64,
                    "file_count": 1,
                }
                for package in OVERLAY_PACKAGES
            },
            "runtime_environment": {
                "ros_domain_id": "42",
                "gazebo_partition": "nano_c_slam_501",
                "rmw_implementation": "rmw_cyclonedds_cpp",
                "prefix_paths": {
                    "AMENT_PREFIX_PATH": "/overlay:/ros",
                    "COLCON_PREFIX_PATH": "/overlay:/ros",
                },
            },
            "interface_identity": {
                "source_sha256": {
                    "nano_c_slam_interfaces/msg/StampedRanges.msg": "2" * 64,
                    "nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg": "2" * 64,
                },
                "overlay": {
                    "prefix": "/overlay/nano_c_slam_interfaces",
                    "sha256": "6" * 64,
                    "file_count": 1,
                },
            },
        },
    }


class GazeboRunManifestTests(unittest.TestCase):
    def publish_fixture(self, directory: Path) -> Path:
        report = _report()
        stream_contents = {
            "collector.stdout": b"collector output\n",
            "collector.stderr": b"collector warning\n",
            "mission.stdout": b"mission output\n",
            "mission.stderr": b"",
        }
        records: dict[str, object] = {}
        for name, contents in stream_contents.items():
            relative = Path("logs") / f"{name}.log"
            path = directory / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(contents)
            records[name] = {
                "path": relative.as_posix(),
                "sha256": _hash(contents),
                "bytes": len(contents),
            }
        report["process_streams"] = records
        report["raw_logs"] = {
            "collector": (
                stream_contents["collector.stdout"]
                + stream_contents["collector.stderr"]
            ).decode(),
            "mission": (
                stream_contents["mission.stdout"] + stream_contents["mission.stderr"]
            ).decode(),
        }
        source = SourceIdentity(
            revision="a" * 40,
            dirty=False,
            dirty_patch_sha256=None,
            submodules=(),
        )
        report["source_identity"] = source_identity_to_mapping(source)
        manifest = create_gazebo_run_manifest(
            report, source=source, stream_contents=stream_contents
        )
        report["provenance_manifest"] = {
            "path": "run.provenance.json",
            "sha256": _hash(manifest),
        }
        (directory / "run.provenance.json").write_bytes(manifest)
        report_path = directory / "run.json"
        report_path.write_text(json.dumps(report), encoding="utf-8")
        return report_path

    def test_valid_bundle_is_accepted_and_stream_mutation_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            report_path = self.publish_fixture(Path(temporary_directory))
            manifest = validate_gazebo_mapping_bundle(report_path)
            self.assertEqual(manifest.evidence_class.value, "gazebo-simulation")
            self.assertEqual(main([str(report_path)]), 0)
            with self.assertRaisesRegex(ValueError, "stream changed"):
                (report_path.parent / "logs/mission.stdout.log").write_bytes(b"mutated")
                validate_gazebo_mapping_bundle(report_path)

    def test_sidecar_source_and_command_mutations_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            report_path = self.publish_fixture(Path(temporary_directory))
            sidecar = report_path.with_name("run.provenance.json")
            original_manifest = sidecar.read_bytes()
            sidecar.write_bytes(original_manifest + b" ")
            with self.assertRaisesRegex(ValueError, "sidecar hash"):
                validate_gazebo_mapping_bundle(report_path)
            sidecar.write_bytes(original_manifest)

            report = json.loads(report_path.read_text(encoding="utf-8"))
            report["source_identity"]["revision"] = "b" * 40
            report_path.write_text(json.dumps(report), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "source identity"):
                validate_gazebo_mapping_bundle(report_path)

            report["source_identity"]["revision"] = "a" * 40
            report["child_commands"]["mission"][-1] = "different_mission"
            report_path.write_text(json.dumps(report), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "manifest commands"):
                validate_gazebo_mapping_bundle(report_path)

    def test_wrong_run_domain_binary_and_interface_joins_fail_closed(self) -> None:
        for mutation, expected in (
            (
                lambda report: report["capture_report"].update(run_id="b" * 36),
                "run IDs",
            ),
            (
                lambda report: report["run_provenance"]["runtime_environment"].update(
                    ros_domain_id="7"
                ),
                "ROS_DOMAIN_ID",
            ),
            (
                lambda report: report["run_provenance"]["runtime_environment"].update(
                    gazebo_partition="other"
                ),
                "GZ_PARTITION",
            ),
            (
                lambda report: report["run_provenance"]["runtime_environment"].update(
                    rmw_implementation="rmw_other_cpp"
                ),
                "RMW_IMPLEMENTATION",
            ),
            (
                lambda report: report["run_provenance"]["executables"][
                    "mapping_binary"
                ].update(sha256="9" * 64),
                "mapping binary does not match capture",
            ),
            (
                lambda report: report["run_provenance"]["interface_identity"][
                    "source_sha256"
                ].update(
                    {"nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg": "8" * 64}
                ),
                "interface identity",
            ),
        ):
            with (
                self.subTest(expected=expected),
                tempfile.TemporaryDirectory() as temporary_directory,
            ):
                report_path = self.publish_fixture(Path(temporary_directory))
                report = json.loads(report_path.read_text(encoding="utf-8"))
                mutation(report)
                report_path.write_text(json.dumps(report), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, expected):
                    validate_gazebo_mapping_bundle(report_path)


if __name__ == "__main__":
    unittest.main()
