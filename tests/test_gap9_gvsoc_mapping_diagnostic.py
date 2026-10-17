"""Adversarial tests for the schema-bound GAP9 GVSOC mapping capture."""

from __future__ import annotations

import contextlib
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from scripts import capture_gap9_gvsoc_mapping_diagnostic as capture
from scripts.gap9_mapping_diagnostic import validate_mapping_diagnostic
from scripts.write_gap9_build_provenance import create_provenance, git_worktree_dirty
from test_support.gap9_mapping_fixtures import (
    GVSOC_MAPPING_LIMITATION as LIMITATION_MARKER,
)
from test_support.gap9_mapping_fixtures import (
    GVSOC_MAPPING_PASS as PASS_MARKER,
)
from test_support.gap9_mapping_fixtures import (
    materialize_mapping_source_checkout,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class GvsocMappingDiagnosticTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.elf = self.root / "synthetic-target.elf"
        self.log = self.root / "target.log"
        self.commands = self.root / "compile_commands.json"
        self.provenance = self.root / "build-provenance.json"
        self.sdk = self.root / "sdk"
        self.output = self.root / "mapping.json"
        self.elf.write_bytes(b"GVSOC target ELF test bytes")
        self.log.write_text(LIMITATION_MARKER + "\n", encoding="utf-8")
        self.commands.write_text(
            json.dumps(
                [
                    {
                        "directory": "/project",
                        "file": "/project/cslam-gap9-app/icp/icp.c",
                        "command": "riscv32-unknown-elf-gcc -O2 -c icp.c",
                    }
                ]
            ),
            encoding="utf-8",
        )
        self.provenance.write_text(
            json.dumps(
                {
                    "toolchain_id": "riscv32-unknown-elf-gcc (GAP SDK) 12.2.0",
                    "binary_sha256": _sha256(self.elf),
                    "compiler_commands_sha256": _sha256(self.commands),
                    "sdk_metadata": {
                        "revision": "a" * 40,
                        "release": "5.20.4",
                        "worktree_dirty": True,
                    },
                }
            ),
            encoding="utf-8",
        )
        self.sdk.mkdir()

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def run_capture(
        self,
        exit_code: int = 1,
        *,
        mutate_log_during_capture: bool = False,
        mutate_source_during_capture: Path | None = None,
        sdk_metadata_override: dict[str, str | bool] | None = None,
    ) -> tuple[int, str]:
        args = [
            "capture",
            "--elf",
            str(self.elf),
            "--log",
            str(self.log),
            "--compile-commands",
            str(self.commands),
            "--build-provenance",
            str(self.provenance),
            "--sdk-root",
            str(self.sdk),
            "--output",
            str(self.output),
            "--gvsoc-exit-code",
            str(exit_code),
        ]
        stdout = io.StringIO()
        stderr = io.StringIO()
        sdk_calls = 0

        def read_sdk_metadata(_: Path) -> dict[str, str | bool]:
            nonlocal sdk_calls
            sdk_calls += 1
            if mutate_log_during_capture:
                self.log.write_text("modified after target exit\n", encoding="utf-8")
            if mutate_source_during_capture is not None and sdk_calls == 1:
                mutate_source_during_capture.write_bytes(
                    b"owned source changed during capture"
                )
            return sdk_metadata_override or {
                "revision": "a" * 40,
                "release": "5.20.4",
                "worktree_dirty": True,
            }

        with (
            patch("sys.argv", args),
            patch.object(capture, "sdk_metadata", side_effect=read_sdk_metadata),
            contextlib.redirect_stdout(stdout),
            contextlib.redirect_stderr(stderr),
        ):
            try:
                result = capture.main()
            except SystemExit as error:
                result = error.code if isinstance(error.code, int) else 1
        return result, stdout.getvalue() + stderr.getvalue()

    def test_captures_known_target_limitation_with_exact_provenance(self) -> None:
        result, _ = self.run_capture()
        self.assertEqual(result, 0)
        artifact = json.loads(self.output.read_text(encoding="utf-8"))
        validate_mapping_diagnostic(artifact)
        self.assertEqual(artifact["schema_version"], 2)
        self.assertEqual(artifact["status"], "reproduced-limitation")
        self.assertEqual(artifact["evidence_class"], "gvsoc-target-execution")
        self.assertEqual(artifact["execution"]["log_bytes"], self.log.stat().st_size)
        self.assertEqual(
            artifact["execution"]["log_sha256"], capture.sha256_file(self.log)
        )
        retained_log = self.output.with_suffix(".log")
        self.assertEqual(retained_log.read_bytes(), self.log.read_bytes())
        self.assertIn("-O2 -c icp.c", artifact["build"]["compiler_flags"])

    def test_accepts_only_a_complete_passing_target_marker_and_zero_exit(self) -> None:
        marker_status, mapping = capture.parse_target_marker(PASS_MARKER)
        self.assertEqual(marker_status, "passed")
        self.assertEqual(mapping["accepted_constraints"], 2)
        self.assertTrue(mapping["optimization_completed"])

        self.log.write_text(PASS_MARKER + "\n", encoding="utf-8")
        result, _ = self.run_capture(exit_code=0)
        self.assertEqual(result, 0)
        self.assertEqual(
            json.loads(self.output.read_text(encoding="utf-8"))["status"], "passed"
        )

    def test_rejects_ambiguous_wrong_class_malformed_and_contradictory_markers(
        self,
    ) -> None:
        invalid_logs = (
            "ordinary log output",
            LIMITATION_MARKER + "\n" + LIMITATION_MARKER,
            PASS_MARKER.replace("NANO_C_SLAM_GVSOC", "NANO_C_SLAM_HOST"),
            PASS_MARKER + "\nNANO_C_SLAM_HOST:mapping_ok forged-host-marker",
            PASS_MARKER + " unexpected=1",
            PASS_MARKER.replace("accepted_constraints=2", "accepted_constraints=3"),
            PASS_MARKER.replace("icp_has_nan=0", "icp_has_nan=1"),
            LIMITATION_MARKER.replace("optimizer_inputs=0", "optimizer_inputs=1"),
            LIMITATION_MARKER.replace("icp_constraints=0", "icp_constraints=1"),
            PASS_MARKER.replace("poses=60", "poses=59"),
        )
        for content in invalid_logs:
            with self.subTest(content=content[-50:]):
                self.log.write_text(content + "\n", encoding="utf-8")
                result, _ = self.run_capture(exit_code=1)
                self.assertNotEqual(result, 0)
                self.assertFalse(self.output.exists())

    def test_success_marker_with_failure_exit_is_not_published(self) -> None:
        self.log.write_text(PASS_MARKER + "\n", encoding="utf-8")
        result, _ = self.run_capture(exit_code=1)
        self.assertNotEqual(result, 0)
        self.assertFalse(self.output.exists())

    def test_existing_output_is_never_overwritten(self) -> None:
        self.output.write_text("preserve", encoding="utf-8")
        result, _ = self.run_capture()
        self.assertNotEqual(result, 0)
        self.assertEqual(self.output.read_text(encoding="utf-8"), "preserve")

    def test_changed_target_log_is_not_published_as_matching_provenance(self) -> None:
        result, _ = self.run_capture(mutate_log_during_capture=True)
        self.assertNotEqual(result, 0)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.output.with_suffix(".log").exists())

    def test_compile_commands_must_match_the_target_build_record(self) -> None:
        self.commands.write_text("[]", encoding="utf-8")
        result, _ = self.run_capture()
        self.assertNotEqual(result, 0)
        self.assertFalse(self.output.exists())

    def test_target_elf_must_match_the_build_record(self) -> None:
        self.elf.write_bytes(b"stale ELF from another build")
        result, diagnostic = self.run_capture()
        self.assertNotEqual(result, 0)
        self.assertIn("target ELF does not match", diagnostic)
        self.assertFalse(self.output.exists())

    def test_runtime_sdk_release_and_dirty_state_must_match_build_record(self) -> None:
        recorded_sdk = {
            "revision": "a" * 40,
            "release": "5.20.4",
            "worktree_dirty": True,
        }
        for key, value in (("release", "5.20.3"), ("worktree_dirty", False)):
            with self.subTest(key=key):
                self.output.unlink(missing_ok=True)
                metadata = dict(recorded_sdk)
                metadata[key] = value
                result, diagnostic = self.run_capture(sdk_metadata_override=metadata)
                self.assertNotEqual(result, 0)
                self.assertIn("GAP SDK differs", diagnostic)
                self.assertFalse(self.output.exists())

    def test_owned_source_changed_during_capture_is_rejected(self) -> None:
        isolated_repo = self.root / "source-under-test"
        fixture, owned_source = materialize_mapping_source_checkout(isolated_repo)
        sdk_state = {
            "revision": "a" * 40,
            "release": "5.20.4",
            "worktree_dirty": True,
        }
        with (
            patch.object(capture, "REPOSITORY_ROOT", isolated_repo),
            patch.object(
                capture,
                "mapping_source_paths",
                return_value=[fixture, owned_source],
            ),
        ):
            result, diagnostic = self.run_capture(
                mutate_source_during_capture=owned_source,
                sdk_metadata_override=sdk_state,
            )
        self.assertNotEqual(result, 0)
        self.assertIn("mapping source changed during capture", diagnostic)
        self.assertFalse(self.output.exists())

    def test_runtime_sdk_must_match_the_build_time_sdk_record(self) -> None:
        result, _ = self.run_capture(
            sdk_metadata_override={
                "revision": "b" * 40,
                "release": "5.20.4",
                "worktree_dirty": True,
            }
        )
        self.assertNotEqual(result, 0)
        self.assertFalse(self.output.exists())

    def test_build_provenance_binds_elf_compile_commands_and_sdk_state(self) -> None:
        toolchain = self.provenance.with_name("toolchain.txt")
        toolchain.write_text(
            "riscv32-unknown-elf-gcc (GAP SDK) 12.2.0\n", encoding="utf-8"
        )
        with (
            patch(
                "scripts.write_gap9_build_provenance.git_value",
                side_effect=("a" * 40, "release-v5.20.4"),
            ),
            patch(
                "scripts.write_gap9_build_provenance.git_worktree_dirty",
                return_value=True,
            ),
        ):
            provenance = create_provenance(
                self.elf,
                self.commands,
                toolchain,
                self.sdk,
            )
        self.assertEqual(provenance["binary_sha256"], _sha256(self.elf))
        self.assertEqual(provenance["compiler_commands_sha256"], _sha256(self.commands))
        self.assertTrue(provenance["sdk_metadata"]["worktree_dirty"])

    def test_sdk_dirty_check_handles_clean_and_modified_results(self) -> None:
        with patch(
            "scripts.write_gap9_build_provenance.subprocess.run",
            side_effect=(SimpleNamespace(returncode=0), SimpleNamespace(returncode=1)),
        ) as run:
            self.assertFalse(git_worktree_dirty(self.sdk))
            self.assertTrue(git_worktree_dirty(self.sdk))
        self.assertEqual(run.call_count, 2)
        self.assertEqual(run.call_args_list[0].args[0][-3:], ("--quiet", "HEAD", "--"))


if __name__ == "__main__":
    unittest.main()
