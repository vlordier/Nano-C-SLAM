"""Adversarial tests for the schema-bound GAP9 GVSOC mapping capture."""

from __future__ import annotations

import contextlib
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import capture_gap9_gvsoc_mapping_diagnostic as capture
from scripts.gap9_mapping_diagnostic import validate_mapping_diagnostic
from scripts.write_gap9_build_provenance import create_provenance


def passed_marker() -> str:
    return (
        "NANO_C_SLAM_GVSOC:mapping_ok poses=60 scans=3 map_points=240 "
        "finite=60 graph=3 main_solver=1 solver_calls=3 solver_iterations=9 "
        "completed=1 has_nan=0 icp_attempts=2 accepted_constraints=2 "
        "optimizer_inputs=2 optimizer_unique_edges=2 matches=80 "
        "icp_error_milli=0 icp_has_nan=0"
    )


def limitation_marker() -> str:
    return (
        "NANO_C_SLAM_GVSOC:mapping_fail poses=60 scans=3 points=240 "
        "icp_constraints=0 matches=80 icp_error_milli=2147483647 "
        "icp_has_nan=1 finite=0 graph=0 main_solver=0 solver_calls=0 "
        "solver_iterations=0 completed=0 has_nan=0 icp_attempts=2 "
        "accepted_constraints=0 optimizer_inputs=0 optimizer_unique_edges=0"
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class GvsocMappingDiagnosticTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.elf = self.root / "target.elf"
        self.log = self.root / "target.log"
        self.commands = self.root / "compile_commands.json"
        self.provenance = self.root / "build-provenance.json"
        self.sdk = self.root / "sdk"
        self.output = self.root / "mapping.json"
        self.elf.write_bytes(b"GVSOC target ELF test bytes")
        self.log.write_text(limitation_marker() + "\n", encoding="utf-8")
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
        marker_status, mapping = capture.parse_target_marker(passed_marker())
        self.assertEqual(marker_status, "passed")
        self.assertEqual(mapping["accepted_constraints"], 2)
        self.assertTrue(mapping["optimization_completed"])

        self.log.write_text(passed_marker() + "\n", encoding="utf-8")
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
            limitation_marker() + "\n" + limitation_marker(),
            passed_marker().replace("NANO_C_SLAM_GVSOC", "NANO_C_SLAM_HOST"),
            passed_marker() + "\nNANO_C_SLAM_HOST:mapping_ok forged-host-marker",
            passed_marker() + " unexpected=1",
            passed_marker().replace("accepted_constraints=2", "accepted_constraints=3"),
            passed_marker().replace("icp_has_nan=0", "icp_has_nan=1"),
            limitation_marker().replace("optimizer_inputs=0", "optimizer_inputs=1"),
            limitation_marker().replace("icp_constraints=0", "icp_constraints=1"),
            passed_marker().replace("poses=60", "poses=59"),
        )
        for content in invalid_logs:
            with self.subTest(content=content[-50:]):
                self.log.write_text(content + "\n", encoding="utf-8")
                result, _ = self.run_capture(exit_code=1)
                self.assertNotEqual(result, 0)
                self.assertFalse(self.output.exists())

    def test_success_marker_with_failure_exit_is_not_published(self) -> None:
        self.log.write_text(passed_marker() + "\n", encoding="utf-8")
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
        source_root = self.root / "checkout"
        fixture = source_root / "tests/gap9_sdk_gvsoc_mapping_smoke.c"
        owned_source = source_root / "cslam-gap9-app/icp/icp.c"
        fixture.parent.mkdir(parents=True)
        owned_source.parent.mkdir(parents=True)
        fixture.write_bytes(b"fixture source")
        owned_source.write_bytes(b"owned source before capture")
        sdk_state = {
            "revision": "a" * 40,
            "release": "5.20.4",
            "worktree_dirty": True,
        }
        with (
            patch.object(capture, "REPOSITORY_ROOT", source_root),
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
        with patch(
            "scripts.write_gap9_build_provenance.git_value",
            side_effect=("a" * 40, "release-v5.20.4", " M source.c"),
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


if __name__ == "__main__":
    unittest.main()
