"""Adversarial tests for versioned validation provenance manifests."""

from __future__ import annotations

import copy
import unittest
from typing import cast

from scripts.validation_provenance import validate_manifest


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
            "stderr_sha256": digest,
        },
        "hardware_attestation": None,
    }


def _mapping(value: object) -> dict[str, object]:
    return cast(dict[str, object], value)


class ValidationProvenanceTests(unittest.TestCase):
    def test_accepts_a_complete_host_pass_and_normalizes_environment(self) -> None:
        manifest = _manifest()
        _mapping(manifest["execution"])["environment"] = {"Z": "last", "A": "first"}

        parsed = validate_manifest(manifest)

        self.assertEqual(parsed.status.value, "success")
        self.assertEqual(parsed.execution.environment, (("A", "first"), ("Z", "last")))

    def test_rejects_unknown_fields_instead_of_silently_dropping_them(self) -> None:
        manifest = _manifest()
        manifest["hardware_flight_confirmed"] = True

        with self.assertRaisesRegex(ValueError, "exactly"):
            validate_manifest(manifest)

    def test_accepts_hardware_attested_physical_flight_claim(self) -> None:
        manifest = _manifest()
        manifest["evidence_class"] = "physical-flight"
        manifest["hardware_attestation"] = {
            "device_id_sha256": "a" * 64,
            "firmware_sha256": "a" * 64,
            "operator_id_sha256": "a" * 64,
            "raw_measurements_sha256": "a" * 64,
            "observed_at_utc": "2026-10-18T12:00:00Z",
        }

        parsed = validate_manifest(manifest)

        self.assertEqual(parsed.evidence_class.value, "physical-flight")

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

        with self.assertRaisesRegex(ValueError, "runtime dependency"):
            validate_manifest(manifest)

    def test_rejects_execution_time_reversal_for_pass(self) -> None:
        manifest = _manifest()
        _mapping(manifest["execution"])["started_at_utc"] = "2026-10-18T12:00:03Z"

        with self.assertRaisesRegex(ValueError, "precedes"):
            validate_manifest(manifest)


if __name__ == "__main__":
    unittest.main()
