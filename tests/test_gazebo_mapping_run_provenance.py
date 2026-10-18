"""Adversarial tests for source/runtime provenance in Gazebo run bundles."""

from __future__ import annotations

import copy
import hashlib
import tempfile
import unittest
from pathlib import Path
from subprocess import CompletedProcess
from unittest.mock import patch

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance import (
    OVERLAY_PACKAGES,
    OWNED_INPUT_PATHS,
    collect_run_provenance,
    require_provenance_unchanged,
    validate_combined_run_report,
)


def _sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _valid_provenance(binary_hash: str) -> dict[str, object]:
    return {
        "schema_version": 1,
        "source": {
            "git_revision": "a" * 40,
            "git_dirty": False,
            "git_status_porcelain": "",
            "owned_input_sha256": {path: "b" * 64 for path in OWNED_INPUT_PATHS},
        },
        "executables": {
            "python": {
                "path": "/python",
                "version": "3.11.0",
                "sha256": "c" * 64,
            },
            "ros2": {
                "path": "/ros2",
                "ros_distro": "jazzy",
                "sha256": "d" * 64,
            },
            "gazebo": {
                "path": "/gz",
                "sim_version": "8.0.0",
                "sha256": "e" * 64,
            },
            "mapping_binary": {
                "path": "/mapping_replay",
                "sha256": binary_hash,
            },
        },
        "ros_overlay_packages": {
            package: {
                "prefix": f"/overlay/{package}",
                "sha256": "f" * 64,
                "file_count": 1,
            }
            for package in OVERLAY_PACKAGES
        },
        "runtime_environment": {
            "ros_domain_id": "42",
            "gazebo_partition": "nano_c_slam_501",
        },
    }


def _valid_report() -> dict[str, object]:
    binary_hash = "1" * 64
    return {
        "schema_version": 2,
        "run_id": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
        "child_commands": {
            "collector": [
                "/python",
                str(
                    Path(__file__).resolve().parents[1]
                    / "scripts/gazebo_mapping_sitl.py"
                ),
                "--binary",
                "/mapping_replay",
            ],
            "mission": ["/ros2", "run", "nano_c_slam_gazebo", "sim_mission"],
        },
        "capture_report": {
            "run_id": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            "mapping_replay_provenance": {"binary_sha256": binary_hash},
        },
        "run_provenance": _valid_provenance(binary_hash),
    }


class GazeboRunProvenanceTests(unittest.TestCase):
    def test_report_rejects_legacy_missing_or_inconsistent_provenance(self) -> None:
        report = _valid_report()
        self.assertEqual(validate_combined_run_report(report), report)

        missing = copy.deepcopy(report)
        del missing["run_provenance"]
        with self.assertRaisesRegex(ValueError, "provenance"):
            validate_combined_run_report(missing)

        legacy = copy.deepcopy(report)
        legacy["schema_version"] = 1
        with self.assertRaisesRegex(ValueError, "schema v2"):
            validate_combined_run_report(legacy)

    def test_report_rejects_mutated_source_and_replay_hash_mismatch(self) -> None:
        report = _valid_report()
        provenance = report["run_provenance"]
        assert isinstance(provenance, dict)
        source = provenance["source"]
        assert isinstance(source, dict)
        hashes = source["owned_input_sha256"]
        assert isinstance(hashes, dict)
        hashes[OWNED_INPUT_PATHS[0]] = "not-a-hash"
        with self.assertRaisesRegex(ValueError, "invalid source hash"):
            validate_combined_run_report(report)

        report = _valid_report()
        provenance = report["run_provenance"]
        assert isinstance(provenance, dict)
        source = provenance["source"]
        assert isinstance(source, dict)
        source["git_dirty"] = True
        source["git_status_porcelain"] = " M scripts/gazebo_mapping_run.py"
        validate_combined_run_report(report)

        executables = provenance["executables"]
        assert isinstance(executables, dict)
        binary = executables["mapping_binary"]
        assert isinstance(binary, dict)
        binary["sha256"] = "2" * 64
        with self.assertRaisesRegex(ValueError, "does not match capture"):
            validate_combined_run_report(report)

        report = _valid_report()
        commands = report["child_commands"]
        assert isinstance(commands, dict)
        collector = commands["collector"]
        assert isinstance(collector, list)
        collector[-1] = "/different/replay"
        with self.assertRaisesRegex(ValueError, "does not use the recorded mapping"):
            validate_combined_run_report(report)

    def test_changed_source_or_overlay_during_run_fails_closed(self) -> None:
        before = _valid_provenance("1" * 64)
        dirty_source = copy.deepcopy(before)
        source = dirty_source["source"]
        assert isinstance(source, dict)
        hashes = source["owned_input_sha256"]
        assert isinstance(hashes, dict)
        hashes[OWNED_INPUT_PATHS[0]] = "2" * 64
        with self.assertRaisesRegex(ValueError, "changed during Gazebo run"):
            require_provenance_unchanged(before, dirty_source)

        dirty_overlay = copy.deepcopy(before)
        overlay = dirty_overlay["ros_overlay_packages"]
        assert isinstance(overlay, dict)
        package = overlay[OVERLAY_PACKAGES[0]]
        assert isinstance(package, dict)
        package["sha256"] = "3" * 64
        with self.assertRaisesRegex(ValueError, "changed during Gazebo run"):
            require_provenance_unchanged(before, dirty_overlay)

    def test_collection_hashes_owned_inputs_and_records_dirty_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for relative in OWNED_INPUT_PATHS:
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("fixture source", encoding="utf-8")
            binary = root / "mapping_replay"
            binary.write_bytes(b"fixture binary")
            python = root / "python"
            python.write_bytes(b"python executable fixture")
            ros2 = root / "ros2"
            ros2.write_bytes(b"ros2 executable fixture")
            gazebo = root / "gz"
            gazebo.write_bytes(b"gz executable fixture")
            prefixes: dict[str, Path] = {}
            for package in OVERLAY_PACKAGES:
                prefix = root / "overlay" / package
                prefix.mkdir(parents=True)
                (prefix / "package.xml").write_text(package, encoding="utf-8")
                prefixes[package] = prefix

            def run(command: list[str], **_: object) -> CompletedProcess[str]:
                self.assertEqual(command, [str(gazebo.resolve()), "sim", "--versions"])
                return CompletedProcess(command, 0, stdout="8.14.0\n", stderr="")

            status = " M scripts/gazebo_mapping_run.py"
            with (
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance.shutil.which",
                    side_effect=lambda name: {
                        "ros2": str(ros2),
                        "gz": str(gazebo),
                    }.get(name),
                ),
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance.sys.executable",
                    str(python),
                ),
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance.platform.python_version",
                    return_value="3.11.0",
                ),
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance._git_value",
                    side_effect=[status, "a" * 40],
                ),
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance._package_prefix",
                    side_effect=lambda _ros2, package: prefixes[package],
                ),
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance.subprocess.run",
                    side_effect=run,
                ),
                patch.dict("os.environ", {"ROS_DISTRO": "jazzy"}),
            ):
                provenance = collect_run_provenance(
                    root, ros2_executable=str(ros2), mapping_binary=binary
                )

            source = provenance["source"]
            assert isinstance(source, dict)
            self.assertTrue(source["git_dirty"])
            source_hashes = source["owned_input_sha256"]
            assert isinstance(source_hashes, dict)
            self.assertEqual(
                source_hashes[OWNED_INPUT_PATHS[0]],
                _sha256(b"fixture source"),
            )
            (root / OWNED_INPUT_PATHS[0]).write_text("mutated", encoding="utf-8")
            self.assertNotEqual(
                source_hashes[OWNED_INPUT_PATHS[0]],
                _sha256((root / OWNED_INPUT_PATHS[0]).read_bytes()),
            )


if __name__ == "__main__":
    unittest.main()
