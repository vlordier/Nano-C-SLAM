"""Adversarial tests for host GAP9 mapping evidence capture."""

from __future__ import annotations

import contextlib
import copy
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import capture_gap9_host_mapping_diagnostic as capture
from scripts.capture_gap9_host_mapping_diagnostic import (
    main,
    validate_host_mapping_bundle,
)
from scripts.gap9_mapping_diagnostic import validate_mapping_diagnostic
from scripts.validation_provenance import SourceIdentity, parse_manifest_json
from test_support.gap9_mapping_fixtures import (
    HOST_MAPPING_FAILURE,
    HOST_MAPPING_PASS,
    write_executable_output,
)


class HostMappingDiagnosticCaptureTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        self.directory = Path(self.tempdir.name)
        self.binary = self.directory / "smoke"
        self.output = self.directory / "diagnostic.json"
        self.write_binary(HOST_MAPPING_PASS + "\n")

    def tearDown(self) -> None:
        self.tempdir.cleanup()

    def write_binary(self, output: str, exit_code: int = 0) -> None:
        write_executable_output(self.binary, output, exit_code=exit_code)

    def capture(self, *extra: str) -> tuple[int, str]:
        args = [
            "capture",
            "--binary",
            str(self.binary),
            "--output",
            str(self.output),
            "--toolchain-id",
            "test-cc 1.0",
            "--compiler-flags",
            "-O2 -fsanitize=address,undefined",
            *extra,
        ]
        stdout = io.StringIO()
        stderr = io.StringIO()
        with (
            patch("sys.argv", args),
            patch(
                "scripts.capture_gap9_host_mapping_diagnostic.collect_source_identity",
                return_value=SourceIdentity("b" * 40, False, None, ()),
            ),
            patch(
                "scripts.capture_gap9_host_mapping_diagnostic.verify_source_identity"
            ),
            contextlib.redirect_stdout(stdout),
            contextlib.redirect_stderr(stderr),
        ):
            try:
                return main(), stdout.getvalue()
            except SystemExit as error:
                return (
                    error.code if isinstance(error.code, int) else 1,
                    stdout.getvalue(),
                )

    def test_pass_emits_current_provenance_and_host_only_schema(self) -> None:
        code, _ = self.capture()
        self.assertEqual(code, 0)
        artifact = json.loads(self.output.read_text(encoding="utf-8"))
        validate_mapping_diagnostic(artifact)
        self.assertEqual(artifact["evidence_class"], "host-pmsis-stubs")
        self.assertEqual(artifact["fixture"]["source_sha256"], _sha256_fixture())
        self.assertEqual(artifact["build"]["binary_sha256"], _sha256(self.binary))
        self.assertFalse(artifact["hardware_measurement"])
        raw_log = self.output.with_suffix(".log").read_bytes()
        self.assertEqual(raw_log, (HOST_MAPPING_PASS + "\n").encode())
        self.assertEqual(artifact["execution"]["log_bytes"], len(raw_log))
        self.assertEqual(
            artifact["execution"]["log_sha256"], hashlib.sha256(raw_log).hexdigest()
        )
        provenance_path = self.output.with_suffix(".provenance.json")
        provenance_bytes = provenance_path.read_bytes()
        self.assertEqual(artifact["provenance_manifest"]["path"], provenance_path.name)
        self.assertEqual(
            artifact["provenance_manifest"]["sha256"],
            hashlib.sha256(provenance_bytes).hexdigest(),
        )
        provenance = parse_manifest_json(provenance_bytes.decode("utf-8"))
        self.assertEqual(validate_host_mapping_bundle(self.output), provenance)
        self.assertEqual(provenance.status.value, "success")
        executable = provenance.build.executable
        if executable is None:
            self.fail("successful provenance must contain executable identity")
        self.assertEqual(executable.sha256, artifact["build"]["binary_sha256"])
        self.assertEqual(
            provenance.execution.stdout_sha256, hashlib.sha256(raw_log).hexdigest()
        )
        capture._validate_manifest_join(
            artifact,
            provenance_bytes,
            provenance_path.name,
            raw_log,
            b"",
        )
        mutated_reference = copy.deepcopy(artifact)
        mutated_reference["provenance_manifest"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "manifest hash"):
            capture._validate_manifest_join(
                mutated_reference,
                provenance_bytes,
                provenance_path.name,
                raw_log,
                b"",
            )
        log_path = self.output.with_suffix(".log")
        log_path.write_bytes(raw_log + b"tampered")
        with self.assertRaisesRegex(ValueError, "length disagrees"):
            validate_host_mapping_bundle(self.output)
        log_path.write_bytes(raw_log)
        provenance_path.write_bytes(provenance_bytes + b" ")
        with self.assertRaisesRegex(ValueError, "sidecar hash"):
            validate_host_mapping_bundle(self.output)

    def test_rejects_absent_duplicate_malformed_target_and_inconsistent_markers(
        self,
    ) -> None:
        outputs = (
            "ordinary process output\n",
            "malformed marker with 'quoted' text\n",
            HOST_MAPPING_PASS + "\n" + HOST_MAPPING_PASS + "\n",
            HOST_MAPPING_PASS.replace("icp_has_nan=0", "icp_has_nan=0 extra=1"),
            HOST_MAPPING_PASS.replace("NANO_C_SLAM_HOST", "NANO_C_SLAM_GVSOC"),
            HOST_MAPPING_PASS.replace(
                "accepted_constraints=2", "accepted_constraints=3"
            ),
            HOST_MAPPING_PASS.replace("icp_has_nan=0", "icp_has_nan=1"),
            HOST_MAPPING_PASS.replace("completed=1", "completed=0"),
        )
        for marker in outputs:
            with self.subTest(marker=marker[-60:]):
                self.write_binary(marker)
                code, _ = self.capture()
                self.assertNotEqual(code, 0)
                self.assertFalse(self.output.exists())

    def test_exit_code_must_agree_with_marker_status(self) -> None:
        for marker, exit_code in (
            (HOST_MAPPING_PASS + "\n", 1),
            (HOST_MAPPING_FAILURE + "\n", 0),
        ):
            with self.subTest(exit_code=exit_code):
                self.write_binary(marker, exit_code)
                code, _ = self.capture()
                self.assertNotEqual(code, 0)
                self.assertFalse(self.output.exists())

    def test_failed_mapping_retains_a_valid_failure_artifact(self) -> None:
        self.write_binary(HOST_MAPPING_FAILURE + "\n", 1)
        code, _ = self.capture()
        self.assertEqual(code, 1)
        artifact = json.loads(self.output.read_text(encoding="utf-8"))
        self.assertEqual(validate_mapping_diagnostic(artifact)["status"], "failed")

    def test_rejects_empty_provenance_nonfinite_timeout_and_existing_output(
        self,
    ) -> None:
        for args in (
            ("--toolchain-id", " "),
            ("--compiler-flags", " "),
            ("--timeout-s", "nan"),
        ):
            with self.subTest(args=args):
                code, _ = self.capture(*args)
                self.assertNotEqual(code, 0)
        self.output.write_text("preserve", encoding="utf-8")
        code, _ = self.capture()
        self.assertNotEqual(code, 0)
        self.assertEqual(self.output.read_text(encoding="utf-8"), "preserve")

        self.output.unlink()
        log_output = self.output.with_suffix(".log")
        log_output.write_text("preserve", encoding="utf-8")
        code, _ = self.capture()
        self.assertNotEqual(code, 0)
        self.assertEqual(log_output.read_text(encoding="utf-8"), "preserve")

        log_output.unlink()
        provenance_output = self.output.with_suffix(".provenance.json")
        provenance_output.write_text("preserve", encoding="utf-8")
        code, _ = self.capture()
        self.assertNotEqual(code, 0)
        self.assertEqual(provenance_output.read_text(encoding="utf-8"), "preserve")

    def test_rejects_non_executable_binary(self) -> None:
        self.binary.chmod(0o644)
        code, _ = self.capture()
        self.assertNotEqual(code, 0)
        self.assertFalse(self.output.exists())


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return digest


def _sha256_fixture() -> str:
    from scripts.capture_gap9_host_mapping_diagnostic import FIXTURE

    return _sha256(FIXTURE)


if __name__ == "__main__":
    unittest.main()
