"""Adversarial validation of shared synthetic host/GVSOC diagnostic artifacts."""

from __future__ import annotations

import copy
import json
import unittest
from pathlib import Path
from typing import cast

from scripts.gap9_mapping_diagnostic import validate_mapping_diagnostic

FIXTURE_DIRECTORY = Path(__file__).parent / "fixtures/gap9_mapping_diagnostics"


class MappingDiagnosticSchemaTests(unittest.TestCase):
    def load_fixture(self, name: str) -> dict[str, object]:
        value = json.loads((FIXTURE_DIRECTORY / name).read_text(encoding="utf-8"))
        assert isinstance(value, dict)
        assert all(isinstance(key, str) for key in value)
        return cast(dict[str, object], value)

    def test_accepts_host_stub_and_gvsoc_target_schema_examples(self) -> None:
        for name, evidence_class in (
            ("host-pmsis-stubs.json", "host-pmsis-stubs"),
            ("gvsoc-target-execution.json", "gvsoc-target-execution"),
        ):
            with self.subTest(evidence_class=evidence_class):
                artifact = self.load_fixture(name)
                self.assertEqual(
                    validate_mapping_diagnostic(artifact)["evidence_class"],
                    evidence_class,
                )

    def test_rejects_version_unknown_missing_and_duplicate_object_fields(self) -> None:
        valid = self.load_fixture("host-pmsis-stubs.json")
        for mutate in (
            lambda item: item.update(schema_version=3),
            lambda item: item.update(unrecognized=True),
            lambda item: item.pop("claim_boundary"),
        ):
            invalid = copy.deepcopy(valid)
            mutate(invalid)
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                validate_mapping_diagnostic(invalid)

        duplicate_key_json = json.dumps(valid).replace(
            '"schema_version": 2', '"schema_version": 2, "schema_version": 2'
        )
        with self.assertRaises(ValueError):
            decoded = json.loads(
                duplicate_key_json,
                object_pairs_hook=_reject_duplicate_keys,
            )
            validate_mapping_diagnostic(decoded)

    def test_rejects_hash_type_count_and_integer_boolean_confusion(self) -> None:
        mutations = (
            ("fixture", "source_sha256", "not-a-digest"),
            ("fixture", "scan_count", True),
            ("fixture", "point_count", 239),
            ("build", "binary_sha256", "A" * 64),
            ("execution", "exit_code", 256),
            ("mapping", "poses", 59),
            ("mapping", "finite_poses", 61),
        )
        for section, field, value in mutations:
            invalid = self.load_fixture("host-pmsis-stubs.json")
            nested = invalid[section]
            assert isinstance(nested, dict)
            nested[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_mapping_diagnostic(invalid)

    def test_rejects_claim_boundary_or_measurement_mislabeling(self) -> None:
        invalid_class = self.load_fixture("host-pmsis-stubs.json")
        with self.assertRaisesRegex(ValueError, "claim GVSOC"):
            invalid_class["claim_boundary"] = "GVSOC target passed on host stubs"
            validate_mapping_diagnostic(invalid_class)

        claims_hardware = self.load_fixture("gvsoc-target-execution.json")
        claims_hardware["hardware_measurement"] = True
        with self.assertRaisesRegex(ValueError, "hardware measurement"):
            validate_mapping_diagnostic(claims_hardware)

    def test_provenance_is_path_safe_complete_and_class_specific(self) -> None:
        mutations = (
            lambda item: item["build"]["source_sha256"].update(
                {"../outside.c": "a" * 64}
            ),
            lambda item: item["build"]["source_sha256"].update(
                {"cslam-gap9-app/icp/icp.c": "bad"}
            ),
            lambda item: item["build"]["source_sha256"].pop("cslam-gap9-app/icp/icp.c"),
            lambda item: item["fixture"].update(source_sha256="f" * 64),
            lambda item: item["execution"].update(log_sha256="not-a-hash"),
            lambda item: item["execution"].update(log_bytes=0),
        )
        for mutate in mutations:
            artifact = self.load_fixture("gvsoc-target-execution.json")
            mutate(artifact)
            with self.subTest(artifact=artifact), self.assertRaises(ValueError):
                validate_mapping_diagnostic(artifact)

        missing_sdk = self.load_fixture("gvsoc-target-execution.json")
        missing_sdk_build = missing_sdk["build"]
        assert isinstance(missing_sdk_build, dict)
        missing_sdk_build["sdk_metadata"] = None
        with self.assertRaisesRegex(ValueError, "fields do not match"):
            validate_mapping_diagnostic(missing_sdk)

        invalid_revision = self.load_fixture("gvsoc-target-execution.json")
        invalid_revision_build = invalid_revision["build"]
        assert isinstance(invalid_revision_build, dict)
        sdk_metadata = invalid_revision_build["sdk_metadata"]
        assert isinstance(sdk_metadata, dict)
        sdk_metadata["revision"] = "dirty"
        with self.assertRaisesRegex(ValueError, "Git commit hash"):
            validate_mapping_diagnostic(invalid_revision)

        host_claims_sdk = self.load_fixture("host-pmsis-stubs.json")
        host_claims_sdk_build = host_claims_sdk["build"]
        assert isinstance(host_claims_sdk_build, dict)
        host_claims_sdk_build["sdk_metadata"] = {
            "revision": "a" * 40,
            "release": "5.20.4",
            "worktree_dirty": False,
        }
        with self.assertRaisesRegex(ValueError, "cannot claim SDK"):
            validate_mapping_diagnostic(host_claims_sdk)

    def test_rejects_impossible_counts_and_null_stages_claimed_as_pass(self) -> None:
        too_many_constraints = self.load_fixture("host-pmsis-stubs.json")
        mapping = too_many_constraints["mapping"]
        assert isinstance(mapping, dict)
        mapping["accepted_constraints"] = 3
        with self.assertRaisesRegex(ValueError, "exceeds icp_attempts"):
            validate_mapping_diagnostic(too_many_constraints)

        missing_stage = self.load_fixture("host-pmsis-stubs.json")
        mapping = missing_stage["mapping"]
        assert isinstance(mapping, dict)
        mapping["successful_solver_calls"] = None
        with self.assertRaisesRegex(ValueError, "unobserved mapping stages"):
            validate_mapping_diagnostic(missing_stage)

    def test_failure_artifact_may_preserve_unobserved_stage_as_null(self) -> None:
        artifact = self.load_fixture("gvsoc-target-execution.json")
        artifact["status"] = "failed"
        mapping = artifact["mapping"]
        assert isinstance(mapping, dict)
        mapping["completed_solver_iterations"] = None
        self.assertEqual(validate_mapping_diagnostic(artifact)["status"], "failed")

    def test_reproduced_limitation_requires_failure_and_rejects_full_pass(self) -> None:
        for section, field, value in (
            ("execution", "exit_code", 0),
            ("execution", "marker_seen", False),
        ):
            artifact = self.load_fixture("gvsoc-target-execution.json")
            nested = artifact[section]
            assert isinstance(nested, dict)
            nested[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_mapping_diagnostic(artifact)

        claims_success = self.load_fixture("gvsoc-target-execution.json")
        mapping = claims_success["mapping"]
        assert isinstance(mapping, dict)
        mapping.update(
            finite_poses=60,
            icp_has_nan=False,
            icp_error_milli=0,
            accepted_constraints=2,
            graph_nodes=3,
            optimizer_input_constraints=2,
            optimizer_unique_edges=2,
            main_solver_returned=True,
            successful_solver_calls=3,
            completed_solver_iterations=9,
            optimization_completed=True,
        )
        with self.assertRaisesRegex(ValueError, "contradicts passing"):
            validate_mapping_diagnostic(claims_success)

        host_claims_target_failure = self.load_fixture("host-pmsis-stubs.json")
        host_claims_target_failure["status"] = "reproduced-limitation"
        with self.assertRaisesRegex(ValueError, "only target execution"):
            validate_mapping_diagnostic(host_claims_target_failure)

    def test_pass_requires_finite_icp_and_completed_optimizer_evidence(self) -> None:
        mutations = (
            ("icp_has_nan", True),
            ("optimization_completed", False),
            ("finite_poses", 59),
            ("main_solver_returned", False),
            ("successful_solver_calls", 0),
            ("icp_attempts", 0),
            ("accepted_constraints", 0),
            ("graph_nodes", 0),
            ("optimizer_input_constraints", 0),
            ("optimizer_unique_edges", 0),
            ("completed_solver_iterations", 8),
        )
        for field, value in mutations:
            artifact = self.load_fixture("host-pmsis-stubs.json")
            mapping = artifact["mapping"]
            assert isinstance(mapping, dict)
            mapping[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_mapping_diagnostic(artifact)

    def test_pass_allows_selected_graph_nodes_and_subgraph_solves(self) -> None:
        artifact = self.load_fixture("host-pmsis-stubs.json")
        mapping = artifact["mapping"]
        assert isinstance(mapping, dict)
        mapping.update(
            graph_nodes=12,
            successful_solver_calls=6,
            completed_solver_iterations=18,
        )
        self.assertEqual(validate_mapping_diagnostic(artifact)["status"], "passed")

        too_many_nodes = copy.deepcopy(artifact)
        mapping = too_many_nodes["mapping"]
        assert isinstance(mapping, dict)
        mapping["graph_nodes"] = 61
        with self.assertRaisesRegex(ValueError, "exceeds fixture.pose_count"):
            validate_mapping_diagnostic(too_many_nodes)


def _reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


if __name__ == "__main__":
    unittest.main()
