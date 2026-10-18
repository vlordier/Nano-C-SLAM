"""Adversarial tests for versioned validation provenance manifests."""

from __future__ import annotations

import copy
import unittest
from typing import cast

from scripts.validation_provenance import (
    parse_manifest_json,
    serialize_manifest,
    validate_manifest,
)


def _manifest() -> dict[str, object]:
    digest = "a" * 64
    return {
        "schema_version": 1,
        "status": "success",
        "evidence_class": "host-pmsis-stubs",
        "claim_boundary": "Host logic only; no target or flight claim.",
        "captured_at_utc": "2026-10-18T12:00:00Z",
        "source": {
            "revision": "b" * 40,
            "dirty": False,
            "dirty_patch_sha256": None,
            "submodules": [
                {
                    "path": "cflib",
                    "gitlink_revision": "c" * 40,
                    "checkout_revision": "c" * 40,
                    "dirty": False,
                    "dirty_patch_sha256": None,
                }
            ],
        },
        "build": {
            "command": ["cc", "-o", "build/host-smoke"],
            "configuration_sha256": digest,
            "toolchain": {
                "name": "clang",
                "version": "21.0.0",
                "executable_sha256": digest,
            },
            "executable": {"path": "build/host-smoke", "sha256": digest},
        },
        "runtime_dependencies": [],
        "execution": {
            "command": ["build/host-smoke"],
            "environment": {"LC_ALL": "C"},
            "started_at_utc": "2026-10-18T12:00:01Z",
            "finished_at_utc": "2026-10-18T12:00:02Z",
            "exit_code": 0,
            "stdout_sha256": digest,
            "stdout_bytes": 4,
            "stderr_sha256": digest,
            "stderr_bytes": 5,
        },
        "hardware_attestation": None,
    }


def _mapping(value: object) -> dict[str, object]:
    return cast(dict[str, object], value)


class ValidationProvenanceTests(unittest.TestCase):
    def test_manifest_json_round_trip_is_canonical_and_stable(self) -> None:
        parsed = validate_manifest(_manifest())

        encoded = serialize_manifest(parsed)
        decoded = parse_manifest_json(encoded)

        self.assertEqual(decoded, parsed)
        self.assertEqual(serialize_manifest(decoded), encoded)

    def test_manifest_json_rejects_duplicate_keys_and_nonfinite_numbers(self) -> None:
        encoded = serialize_manifest(validate_manifest(_manifest()))
        duplicate_key = encoded.replace(
            '"schema_version": 1,',
            '"schema_version": 1, "schema_version": 1,',
            1,
        )
        with self.assertRaisesRegex(ValueError, "duplicate JSON object key"):
            parse_manifest_json(duplicate_key)
        with self.assertRaisesRegex(ValueError, "non-finite JSON number"):
            parse_manifest_json('{"manifest": NaN}')

    def test_accepts_a_complete_host_pass_and_normalizes_environment(self) -> None:
        manifest = _manifest()
        _mapping(manifest["execution"])["environment"] = {"Z": "", "A": None}

        parsed = validate_manifest(manifest)

        self.assertEqual(parsed.status.value, "success")
        self.assertEqual(parsed.execution.environment, (("A", None), ("Z", "")))

    def test_rejects_unknown_fields_instead_of_silently_dropping_them(self) -> None:
        manifest = _manifest()
        manifest["hardware_flight_confirmed"] = True

        with self.assertRaisesRegex(ValueError, "exactly"):
            validate_manifest(manifest)

    def test_rejects_boolean_schema_version(self) -> None:
        manifest = _manifest()
        manifest["schema_version"] = True

        with self.assertRaisesRegex(ValueError, "schema_version"):
            validate_manifest(manifest)

    def test_build_command_preserves_unknown_and_rejects_empty_observed_command(
        self,
    ) -> None:
        unknown_command = _manifest()
        _mapping(unknown_command["build"])["command"] = None
        self.assertIsNone(validate_manifest(unknown_command).build.command)

        empty_command = _manifest()
        _mapping(empty_command["build"])["command"] = []
        with self.assertRaisesRegex(ValueError, "nonempty list"):
            validate_manifest(empty_command)

    def test_rejects_software_manifest_claiming_physical_flight_success(self) -> None:
        manifest = _manifest()
        manifest["evidence_class"] = "physical-flight"
        manifest["hardware_attestation"] = {
            "device_id_sha256": "a" * 64,
            "firmware_sha256": "a" * 64,
            "operator_id_sha256": "a" * 64,
            "raw_measurements_sha256": "a" * 64,
            "observed_at_utc": "2026-10-18T12:00:00Z",
        }

        with self.assertRaisesRegex(
            ValueError, "authenticated device evidence collector"
        ):
            validate_manifest(manifest)

    def test_physical_run_without_device_collector_can_only_be_blocked_or_not_run(
        self,
    ) -> None:
        for status in ("blocked", "not-run"):
            with self.subTest(status=status):
                manifest = _manifest()
                manifest["status"] = status
                manifest["evidence_class"] = "physical-flight"
                manifest["hardware_attestation"] = None

                parsed = validate_manifest(manifest)

                self.assertEqual(parsed.status.value, status)

    def test_accepts_nonphysical_evidence_classes_with_runtime_identities(self) -> None:
        for evidence_class in ("gvsoc-target-execution", "gazebo-simulation"):
            with self.subTest(evidence_class=evidence_class):
                manifest = _manifest()
                manifest["evidence_class"] = evidence_class
                runtime_names = (
                    ("gap9-gvsoc-model-plugin", "gvsoc-launcher")
                    if evidence_class == "gvsoc-target-execution"
                    else (
                        "executable:python",
                        "executable:ros2",
                        "executable:gazebo",
                        "ros-overlay:nano_c_slam_interfaces",
                    )
                )
                manifest["runtime_dependencies"] = [
                    {
                        "name": name,
                        "disposition": "system-unpinned",
                        "version": None,
                        "sha256": f"{index + 1:x}" * 64,
                        "path": None,
                    }
                    for index, name in enumerate(runtime_names)
                ]

                parsed = validate_manifest(manifest)

                self.assertEqual(parsed.evidence_class.value, evidence_class)

    def test_rejects_host_pass_with_failed_execution(self) -> None:
        manifest = _manifest()
        _mapping(manifest["execution"])["exit_code"] = 1

        with self.assertRaisesRegex(ValueError, "successful"):
            validate_manifest(manifest)

    def test_rejects_nonportable_and_duplicate_submodule_paths(self) -> None:
        for path in ("../outside", "/absolute/path", "sub\\module", "cflib"):
            with self.subTest(path=path):
                manifest = _manifest()
                source = copy.deepcopy(_mapping(manifest["source"]))
                submodules = cast(list[object], source["submodules"])
                submodules.append(
                    {
                        "path": path,
                        "gitlink_revision": "d" * 40,
                        "checkout_revision": "d" * 40,
                        "dirty": False,
                        "dirty_patch_sha256": None,
                    }
                )
                manifest["source"] = source
                with self.assertRaises(ValueError):
                    validate_manifest(manifest)

    def test_rejects_gvsoc_pass_without_runtime_dependency_identities(self) -> None:
        manifest = _manifest()
        manifest["evidence_class"] = "gvsoc-target-execution"

        with self.assertRaisesRegex(ValueError, "model plugin and GVSOC launcher"):
            validate_manifest(manifest)

    def test_rejects_host_evidence_promoted_to_gvsoc_by_class_string_only(self) -> None:
        manifest = _manifest()
        manifest["evidence_class"] = "gvsoc-target-execution"

        with self.assertRaisesRegex(ValueError, "model plugin and GVSOC launcher"):
            validate_manifest(manifest)

    def test_rejects_host_pass_with_arbitrary_runtime_hash_promoted_to_gvsoc(
        self,
    ) -> None:
        manifest = _manifest()
        manifest["evidence_class"] = "gvsoc-target-execution"
        manifest["runtime_dependencies"] = [
            {
                "name": "simulator-runtime",
                "disposition": "system-unpinned",
                "version": None,
                "sha256": "d" * 64,
                "path": None,
            }
        ]

        with self.assertRaisesRegex(ValueError, "model plugin and GVSOC launcher"):
            validate_manifest(manifest)

    def test_rejects_gazebo_pass_without_class_critical_interface_overlay(self) -> None:
        manifest = _manifest()
        manifest["evidence_class"] = "gazebo-simulation"
        manifest["runtime_dependencies"] = [
            {
                "name": name,
                "disposition": "system-unpinned",
                "version": None,
                "sha256": "d" * 64,
                "path": None,
            }
            for name in ("executable:python", "executable:ros2", "executable:gazebo")
        ]

        with self.assertRaisesRegex(ValueError, "interface-overlay identities"):
            validate_manifest(manifest)

    def test_rejects_dirty_source_without_patch_identity(self) -> None:
        manifest = _manifest()
        _mapping(manifest["source"])["dirty"] = True

        with self.assertRaisesRegex(ValueError, "dirty flag must agree"):
            validate_manifest(manifest)

    def test_submodule_checkout_mismatch_requires_root_dirty_patch_identity(
        self,
    ) -> None:
        manifest = _manifest()
        submodules = cast(list[object], _mapping(manifest["source"])["submodules"])
        submodule = _mapping(submodules[0])
        submodule["checkout_revision"] = "d" * 40
        with self.assertRaisesRegex(ValueError, "checkout/gitlink mismatch"):
            validate_manifest(manifest)

        _mapping(manifest["source"])["dirty"] = True
        _mapping(manifest["source"])["dirty_patch_sha256"] = "e" * 64
        parsed = validate_manifest(manifest)
        self.assertNotEqual(
            parsed.source.submodules[0].gitlink_revision,
            parsed.source.submodules[0].checkout_revision,
        )

    def test_rejects_runtime_hash_mutation_and_unknown_dependency_hash(self) -> None:
        manifest = _manifest()
        manifest["runtime_dependencies"] = [
            {
                "name": "simulator-runtime",
                "disposition": "system-unpinned",
                "version": None,
                "sha256": "not-a-hash",
                "path": None,
            }
        ]

        with self.assertRaisesRegex(ValueError, "SHA-256"):
            validate_manifest(manifest)

        runtime = cast(list[object], manifest["runtime_dependencies"])
        dependency = _mapping(runtime[0])
        dependency["sha256"] = "d" * 64
        dependency["disposition"] = "unknown"
        with self.assertRaisesRegex(ValueError, "unknown dependency cannot claim"):
            validate_manifest(manifest)

    def test_rejects_execution_time_reversal_for_pass(self) -> None:
        manifest = _manifest()
        _mapping(manifest["execution"])["started_at_utc"] = "2026-10-18T12:00:03Z"

        with self.assertRaisesRegex(ValueError, "precedes"):
            validate_manifest(manifest)


if __name__ == "__main__":
    unittest.main()
