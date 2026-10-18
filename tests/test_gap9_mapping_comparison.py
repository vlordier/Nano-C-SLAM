"""Host/GVSOC comparison tests using the shared synthetic schema fixtures."""

from __future__ import annotations

import copy
import hashlib
import json
import struct
import tempfile
import unittest
from pathlib import Path

from scripts.compare_gap9_mapping_diagnostics import (
    _publish_atomically,
    compare_diagnostics,
    compare_files,
)

FIXTURE_ROOT = Path(__file__).parent / "fixtures/gap9_mapping_diagnostics"
HOST_FIXTURE = FIXTURE_ROOT / "host-pmsis-stubs.json"
GVSOC_FIXTURE = FIXTURE_ROOT / "gvsoc-target-execution.json"


def _float_bits(value: float) -> str:
    return struct.pack(">f", value).hex()


def _stage_log(*, center_x: float = 1.0, correspondence_p: int = 80) -> str:
    return (
        "\n".join(
            (
                "NANO_C_SLAM_ICP_DIAG:stage=cluster-pair p0=76 reverse_count=20",
                "NANO_C_SLAM_ICP_DIAG:iter=0 stage=correspondence "
                f"p={correspondence_p} q=80 forward=80 reverse=80 "
                "reverse_nonzero=4 invalid_forward=0 invalid_reverse=0",
                "NANO_C_SLAM_ICP_DIAG:iter=0 stage=center "
                f"p=({_float_bits(center_x)},00000000) "
                "q=(00000000,00000000)",
            )
        )
        + "\n"
    )


def _load_pair() -> tuple[dict[str, object], dict[str, object]]:
    host = json.loads(HOST_FIXTURE.read_text(encoding="utf-8"))
    target = json.loads(GVSOC_FIXTURE.read_text(encoding="utf-8"))
    target["status"] = "passed"
    target["claim_boundary"] = (
        "Synthetic GVSOC pass fixture for comparator unit tests; not hardware evidence."
    )
    target["execution"]["exit_code"] = 0
    target["execution"]["marker_seen"] = True
    target["mapping"] = copy.deepcopy(host["mapping"])
    return host, target


class Gap9MappingComparisonTests(unittest.TestCase):
    def test_equal_traces_and_summaries_are_equivalent(self) -> None:
        host, target = _load_pair()
        log = _stage_log()

        result = compare_diagnostics(host, target, log, log)

        self.assertEqual(result["comparison_result"], "equivalent-within-tolerance")
        self.assertIsNone(result["first_divergence"])
        self.assertEqual(result["compared_stage_record_count"], 3)

    def test_float_delta_is_reported_only_outside_named_tolerance(self) -> None:
        host, target = _load_pair()
        result = compare_diagnostics(
            host,
            target,
            _stage_log(),
            _stage_log(center_x=1.0000009),
        )
        self.assertIsNone(result["first_divergence"])
        self.assertGreater(result["tolerated_float_component_difference_count"], 0)

        result = compare_diagnostics(
            host,
            target,
            _stage_log(),
            _stage_log(center_x=1.00002),
        )
        divergence = result["first_divergence"]
        self.assertIsInstance(divergence, dict)
        self.assertEqual(divergence["kind"], "numeric-divergence")
        self.assertEqual(divergence["units"], "m")
        self.assertEqual(divergence["absolute_tolerance"], 1e-5)

    def test_integer_count_divergence_precedes_summary_difference(self) -> None:
        host, target = _load_pair()
        result = compare_diagnostics(
            host,
            target,
            _stage_log(),
            _stage_log(correspondence_p=79),
        )
        divergence = result["first_divergence"]
        self.assertIsInstance(divergence, dict)
        self.assertEqual(divergence["kind"], "count-divergence")
        self.assertEqual(divergence["stage"], "correspondence")
        self.assertEqual(divergence["field"], "p")

    def test_solver_counts_are_work_counters_not_convergence_claims(self) -> None:
        host, target = _load_pair()
        target["mapping"]["successful_solver_calls"] += 1
        target["mapping"]["completed_solver_iterations"] += 3
        result = compare_diagnostics(host, target, _stage_log(), _stage_log())

        semantics = result["solver_semantics"]
        self.assertEqual(semantics["residual_metrics"], "not measured")
        self.assertEqual(semantics["pose_delta_norms"], "not measured")
        self.assertEqual(semantics["convergence"], "unmeasured")
        self.assertEqual(semantics["host"]["successful_solver_calls"], 3)
        self.assertEqual(semantics["gvsoc"]["successful_solver_calls"], 4)
        self.assertEqual(result["first_divergence"]["stage"], "mapping-summary")
        self.assertEqual(result["first_divergence"]["field"], "successful_solver_calls")

    def test_missing_or_sentinel_stage_data_is_not_zero(self) -> None:
        host, target = _load_pair()
        host_log = _stage_log()
        sentinel_log = host_log.replace(
            f"p=({_float_bits(1.0)},00000000)", "p=(sentinel,00000000)"
        )
        result = compare_diagnostics(host, target, host_log, sentinel_log)
        divergence = result["first_divergence"]
        self.assertIsInstance(divergence, dict)
        self.assertEqual(divergence["kind"], "missing-or-sentinel-stage-value")

        result = compare_diagnostics(
            host, target, host_log, "\n".join(host_log.splitlines()[:2])
        )
        divergence = result["first_divergence"]
        self.assertIsInstance(divergence, dict)
        self.assertEqual(divergence["kind"], "missing-stage-instrumentation")

    def test_mismatched_fixture_or_shared_source_is_rejected(self) -> None:
        host, target = _load_pair()
        target["fixture"]["source_sha256"] = "1" * 64
        target["build"]["source_sha256"]["tests/gap9_sdk_gvsoc_mapping_smoke.c"] = (
            "1" * 64
        )
        with self.assertRaisesRegex(ValueError, "fixture identity"):
            compare_diagnostics(host, target, _stage_log(), _stage_log())

        host, target = _load_pair()
        target["build"]["source_sha256"]["cslam-gap9-app/icp/icp.c"] = "1" * 64
        with self.assertRaisesRegex(ValueError, "shared build input hashes"):
            compare_diagnostics(host, target, _stage_log(), _stage_log())

    def test_compare_files_rejects_log_hash_mismatch_and_preserves_output(self) -> None:
        host, target = _load_pair()
        host_log = _stage_log().encode()
        target_log = _stage_log().encode()
        for artifact, log in ((host, host_log), (target, target_log)):
            artifact["execution"]["log_bytes"] = len(log)
            artifact["execution"]["log_sha256"] = hashlib.sha256(log).hexdigest()

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            host_path = root / "host.json"
            target_path = root / "target.json"
            host_log_path = root / "host.log"
            target_log_path = root / "target.log"
            host_path.write_text(json.dumps(host), encoding="utf-8")
            target_path.write_text(json.dumps(target), encoding="utf-8")
            host_log_path.write_bytes(host_log)
            target_log_path.write_bytes(target_log)

            comparison = compare_files(
                host_path, target_path, host_log_path, target_log_path
            )
            self.assertEqual(
                comparison["comparison_result"], "equivalent-within-tolerance"
            )
            self.assertEqual(
                comparison["input_logs"]["host"]["sha256"],
                hashlib.sha256(host_log).hexdigest(),
            )
            self.assertEqual(
                comparison["input_artifacts"]["host_sha256"],
                hashlib.sha256(host_path.read_bytes()).hexdigest(),
            )
            target_log_path.write_bytes(b"X" + target_log[1:])
            with self.assertRaisesRegex(ValueError, "SHA-256"):
                compare_files(host_path, target_path, host_log_path, target_log_path)

            output = root / "comparison.json"
            _publish_atomically(output, comparison)
            with self.assertRaises(FileExistsError):
                _publish_atomically(output, comparison)
            self.assertEqual(json.loads(output.read_text(encoding="utf-8")), comparison)


if __name__ == "__main__":
    unittest.main()
