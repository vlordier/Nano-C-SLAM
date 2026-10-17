"""Adversarial tests for host GAP9 mapping evidence capture."""

from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.capture_gap9_host_mapping_diagnostic import main
from scripts.gap9_mapping_diagnostic import validate_mapping_diagnostic
from test_support.gap9_mapping_fixtures import HOST_MAPPING_FAILURE, HOST_MAPPING_PASS


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
        shell_literal = "'" + output.replace("'", "'\"'\"'") + "'"
        script = f"#!/bin/sh\nprintf '%s' {shell_literal}\nexit {exit_code}\n"
        self.binary.write_text(script, encoding="utf-8")
        self.binary.chmod(0o755)

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

    def test_rejects_absent_duplicate_malformed_target_and_inconsistent_markers(
        self,
    ) -> None:
        outputs = (
            "ordinary process output\n",
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
