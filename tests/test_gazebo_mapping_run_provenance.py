"""Adversarial tests for source/runtime provenance in Gazebo run bundles."""

from __future__ import annotations

import copy
import hashlib
import tempfile
import unittest
from pathlib import Path
from subprocess import CompletedProcess
from unittest.mock import patch

from scripts.runtime_provenance import GAZEBO_ROS_ENVIRONMENT
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance import (
    OVERLAY_PACKAGES,
    OWNED_INPUT_PATHS,
    _linked_library_identities,
    collect_run_provenance,
    require_provenance_unchanged,
    validate_combined_run_report,
    validate_discovered_topic_types,
)


def _sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _valid_provenance(binary_hash: str) -> dict[str, object]:
    environment = {name: None for name in GAZEBO_ROS_ENVIRONMENT}
    environment.update(
        {
            "ROS_DOMAIN_ID": "42",
            "GZ_PARTITION": "nano_c_slam_501",
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "AMENT_PREFIX_PATH": "/overlay:/ros",
            "COLCON_PREFIX_PATH": "/overlay:/ros",
        }
    )
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
                "linked_libraries": [_unknown_library()],
            },
            "ros2": {
                "path": "/ros2",
                "ros_distro": "jazzy",
                "cli_version": "ros2cli 0.32.0",
                "sha256": "d" * 64,
                "linked_libraries": [_unknown_library()],
            },
            "gazebo": {
                "path": "/gz",
                "sim_version": "8.0.0",
                "sha256": "e" * 64,
                "linked_libraries": [_unknown_library()],
            },
            "mapping_binary": {
                "path": "/mapping_replay",
                "sha256": binary_hash,
                "linked_libraries": [_unknown_library()],
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
        "interface_identity": {
            "source_sha256": {
                "nano_c_slam_interfaces/msg/StampedRanges.msg": "1" * 64,
                "nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg": "2" * 64,
            },
            "overlay": {
                "prefix": "/overlay/nano_c_slam_interfaces",
                "sha256": "f" * 64,
                "file_count": 1,
            },
        },
        "loaded_gazebo_server_libraries": {
            "status": "unknown",
            "reason": "fixture has no launch-owned process",
            "observer": "lsof -F0 -a -nP -p PID -d txt,mem",
            "session_id": None,
            "pid": None,
            "launch_pid": None,
            "executable_path": None,
            "observed_at_utc": "2026-10-19T00:00:00Z",
            "commands": [],
            "libraries": [],
        },
        "runtime_environment": {
            "ros_domain_id": "42",
            "gazebo_partition": "nano_c_slam_501",
            "rmw_implementation": "rmw_cyclonedds_cpp",
            "allowlisted_environment": environment,
            "prefix_paths": {
                "AMENT_PREFIX_PATH": "/overlay:/ros",
                "COLCON_PREFIX_PATH": "/overlay:/ros",
            },
        },
    }


def _unknown_library() -> dict[str, object]:
    return {
        "name": "linked-libraries:unknown",
        "install_name": "<unresolved fixture>",
        "path": None,
        "version": None,
        "disposition": "unknown",
        "sha256": None,
    }


def _valid_report() -> dict[str, object]:
    binary_hash = "1" * 64
    return {
        "schema_version": 2,
        "run_id": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
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
        "capture_report": {
            "run_id": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            "mapping_replay_provenance": {"binary_sha256": binary_hash},
        },
        "run_provenance": _valid_provenance(binary_hash),
    }


class GazeboRunProvenanceTests(unittest.TestCase):
    def test_direct_library_hashes_and_unresolved_names_are_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            binary = directory / "mapping_replay"
            binary.write_bytes(b"mapping executable")
            library = directory / "libfixture.dylib"
            library.write_bytes(b"linked library")
            inspector = directory / "otool"
            inspector.write_bytes(b"fixture inspector")
            output = (
                f"{binary}:\n"
                f"\t{library} (compatibility version 1.0.0, current version 1.2.3)\n"
                "\t@rpath/libunresolved.dylib "
                "(compatibility version 2.0.0, current version 2.1.0)\n"
            )
            with (
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance.shutil.which",
                    return_value=str(inspector),
                ),
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance.subprocess.run",
                    return_value=CompletedProcess(
                        [str(inspector), "-L", str(binary)], 0, stdout=output, stderr=""
                    ),
                ),
            ):
                identities = _linked_library_identities(binary, "mapping_binary")

            self.assertEqual(identities[0]["disposition"], "system-unpinned")
            self.assertEqual(identities[0]["sha256"], _sha256(b"linked library"))
            self.assertEqual(identities[1]["disposition"], "unknown")
            self.assertIsNone(identities[1]["sha256"])
            library.write_bytes(b"changed library")
            with (
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance.shutil.which",
                    return_value=str(inspector),
                ),
                patch(
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance.subprocess.run",
                    return_value=CompletedProcess(
                        [str(inspector), "-L", str(binary)], 0, stdout=output, stderr=""
                    ),
                ),
            ):
                changed = _linked_library_identities(binary, "mapping_binary")
            self.assertNotEqual(identities[0]["sha256"], changed[0]["sha256"])

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

    def test_loaded_library_hashes_must_stay_stable_across_observation_window(
        self,
    ) -> None:
        before = _valid_provenance("1" * 64)
        loaded = before["loaded_gazebo_server_libraries"]
        assert isinstance(loaded, dict)
        loaded.update(
            status="observed",
            reason=None,
            session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            pid=3456,
            launch_pid=1234,
            executable_path="/opt/gz/bin/gz",
            libraries=[
                {
                    "path": "/opt/gz/lib/libgazebo-plugin.dylib",
                    "sha256": "a" * 64,
                    "reason": None,
                }
            ],
            commands=[
                {
                    "argv": ["/usr/sbin/lsof", "-a", "-p", "3456"],
                    "returncode": 0,
                    "elapsed_s": 0.1,
                    "stdout_sha256": "c" * 64,
                    "stdout_bytes": 10,
                }
            ],
        )
        loaded_after = copy.deepcopy(before)
        after_evidence = loaded_after["loaded_gazebo_server_libraries"]
        assert isinstance(after_evidence, dict)
        after_evidence["observed_at_utc"] = "2026-10-19T00:00:10Z"
        after_evidence["commands"] = [
            {
                "argv": ["/usr/sbin/lsof", "-a", "-p", "3456"],
                "returncode": 0,
                "elapsed_s": 0.3,
                "stdout_sha256": "c" * 64,
                "stdout_bytes": 10,
            }
        ]

        stable = require_provenance_unchanged(before, loaded_after)
        stable_evidence = stable["loaded_gazebo_server_libraries"]
        assert isinstance(stable_evidence, dict)
        self.assertEqual(
            stable_evidence["observation_window"]["before_utc"],
            "2026-10-19T00:00:00Z",
        )
        self.assertEqual(
            stable_evidence["observation_window"]["after_utc"],
            "2026-10-19T00:00:10Z",
        )

        mutated = copy.deepcopy(loaded_after)
        mutated_evidence = mutated["loaded_gazebo_server_libraries"]
        assert isinstance(mutated_evidence, dict)
        libraries = mutated_evidence["libraries"]
        assert isinstance(libraries, list)
        plugin = libraries[0]
        assert isinstance(plugin, dict)
        plugin["sha256"] = "b" * 64
        with self.assertRaisesRegex(ValueError, "changed during Gazebo run"):
            require_provenance_unchanged(loaded_after, mutated)

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
                    "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance._ros2_cli_version",
                    return_value="0.32.0",
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
                patch.dict(
                    "os.environ",
                    {
                        "ROS_DISTRO": "jazzy",
                        "AMENT_PREFIX_PATH": "/overlay:/ros",
                        "COLCON_PREFIX_PATH": "/overlay:/ros",
                        "NCS_PRIVATE_TOKEN": "must-not-be-captured",
                    },
                    clear=True,
                ),
            ):
                provenance = collect_run_provenance(
                    root, ros2_executable=str(ros2), mapping_binary=binary
                )

            loaded = provenance["loaded_gazebo_server_libraries"]
            assert isinstance(loaded, dict)
            self.assertEqual(loaded["status"], "unknown")
            self.assertIn("manifest is unavailable", loaded["reason"])

            source = provenance["source"]
            assert isinstance(source, dict)
            self.assertTrue(source["git_dirty"])
            executable_identities = provenance["executables"]
            assert isinstance(executable_identities, dict)
            self.assertEqual(executable_identities["ros2"]["cli_version"], "0.32.0")
            environment = provenance["runtime_environment"]
            assert isinstance(environment, dict)
            self.assertEqual(environment["rmw_implementation"], "<rmw-default>")
            self.assertEqual(
                environment["prefix_paths"]["AMENT_PREFIX_PATH"], "/overlay:/ros"
            )
            allowlisted = environment["allowlisted_environment"]
            self.assertEqual(set(allowlisted), set(GAZEBO_ROS_ENVIRONMENT))
            self.assertEqual(allowlisted["RMW_IMPLEMENTATION"], "<rmw-default>")
            self.assertNotIn("NCS_PRIVATE_TOKEN", allowlisted)
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

    def test_provenance_rejects_missing_cli_or_stale_interfaces(self) -> None:
        report = _valid_report()
        provenance = report["run_provenance"]
        assert isinstance(provenance, dict)
        executables = provenance["executables"]
        assert isinstance(executables, dict)
        ros2 = executables["ros2"]
        assert isinstance(ros2, dict)
        del ros2["cli_version"]
        with self.assertRaisesRegex(ValueError, "CLI version"):
            validate_combined_run_report(report)

        report = _valid_report()
        provenance = report["run_provenance"]
        assert isinstance(provenance, dict)
        interfaces = provenance["interface_identity"]
        assert isinstance(interfaces, dict)
        interfaces["overlay"] = {"sha256": "9" * 64}
        with self.assertRaisesRegex(ValueError, "interface identity"):
            validate_combined_run_report(report)


class DiscoveredTopicTypeTests(unittest.TestCase):
    def test_accepts_only_exact_expected_mission_topic_types(self) -> None:
        validate_discovered_topic_types(
            [
                ("/cf1/odom", ["nav_msgs/msg/Odometry"]),
                (
                    "/nano_c_slam/sim/cf1/ranges_mm",
                    ["nano_c_slam_interfaces/msg/StampedPairwiseRanges"],
                ),
                ("/cf1/mission_status", ["std_msgs/msg/String"]),
            ],
            "cf1",
        )

    def test_rejects_missing_wrong_or_ambiguous_topic_types(self) -> None:
        valid = [
            ("/cf1/odom", ["nav_msgs/msg/Odometry"]),
            (
                "/nano_c_slam/sim/cf1/ranges_mm",
                ["nano_c_slam_interfaces/msg/StampedPairwiseRanges"],
            ),
            ("/cf1/mission_status", ["std_msgs/msg/String"]),
        ]
        invalid_graphs = (
            [item for item in valid if item[0] != "/cf1/odom"],
            [
                (topic, ["geometry_msgs/msg/Pose"] if topic == "/cf1/odom" else types)
                for topic, types in valid
            ],
            [
                (
                    topic,
                    [*types, "std_msgs/msg/String"]
                    if topic == "/nano_c_slam/sim/cf1/ranges_mm"
                    else types,
                )
                for topic, types in valid
            ],
        )
        for graph in invalid_graphs:
            with (
                self.subTest(graph=graph),
                self.assertRaisesRegex(RuntimeError, "expected exactly"),
            ):
                validate_discovered_topic_types(graph, "cf1")


if __name__ == "__main__":
    unittest.main()
