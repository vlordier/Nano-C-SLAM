"""Tests for bounded GAP9 GVSOC target execution and raw-log retention."""

from __future__ import annotations

import contextlib
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from shutil import which
from unittest.mock import patch

from scripts.run_gap9_gvsoc_target import main


def _git(root: Path, *arguments: str, text: bool = False) -> str | bytes:
    executable = which("git")
    if executable is None:
        raise unittest.SkipTest("git is required for the source snapshot fixture")
    result = subprocess.run(  # noqa: S603 -- fixed git argv in a temporary fixture
        (executable, "-C", str(root), *arguments),
        check=True,
        capture_output=True,
        text=text,
    )
    if text:
        if not isinstance(result.stdout, str):
            raise TypeError("git fixture returned non-text output")
        return result.stdout
    if not isinstance(result.stdout, bytes):
        raise TypeError("git fixture returned non-byte output")
    return result.stdout


def _initialize_git_fixture(root: Path, contents: str) -> str:
    root.mkdir()
    executable = which("git")
    if executable is None:
        raise unittest.SkipTest("git is required for the source snapshot fixture")
    subprocess.run((executable, "init", "-q", str(root)), check=True)  # noqa: S603
    (root / "tracked.c").write_text(contents, encoding="utf-8")
    _git(root, "add", "tracked.c")
    _git(
        root,
        "-c",
        "user.name=Test",
        "-c",
        "user.email=test@example.invalid",
        "commit",
        "-qm",
        "fixture",
    )
    return str(_git(root, "rev-parse", "HEAD", text=True)).strip()


class GvsocTargetRunnerTests(unittest.TestCase):
    def test_success_and_failure_keep_combined_process_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            log = Path(temporary_directory) / "target.log"
            for exit_code in (0, 7):
                with self.subTest(exit_code=exit_code):
                    args = [
                        "run",
                        "--log",
                        str(log),
                        "--",
                        sys.executable,
                        "-c",
                        f"import sys; print('target-output'); sys.exit({exit_code})",
                    ]
                    stdout_bytes = io.BytesIO()
                    with (
                        patch("sys.argv", args),
                        patch("sys.stdout") as stdout,
                    ):
                        stdout.buffer = stdout_bytes
                        self.assertEqual(main(), exit_code)
                    self.assertIn("target-output", log.read_text(encoding="utf-8"))
                    self.assertIn(
                        "target-output", stdout_bytes.getvalue().decode("utf-8")
                    )

    def test_timeout_kills_run_and_keeps_partial_log(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            log = Path(temporary_directory) / "timed-out.log"
            args = [
                "run",
                "--log",
                str(log),
                "--timeout-s",
                "0.05",
                "--",
                sys.executable,
                "-c",
                "import time; print('started', flush=True); time.sleep(5)",
            ]
            stderr = io.StringIO()
            with (
                patch("sys.argv", args),
                contextlib.redirect_stderr(stderr),
            ):
                self.assertEqual(main(), 124)
            self.assertIn("started", log.read_text(encoding="utf-8"))
            self.assertIn("log retained", stderr.getvalue())

    def test_completion_marker_stops_persistent_target_successfully(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            log = Path(temporary_directory) / "target.log"
            args = [
                "run",
                "--log",
                str(log),
                "--completion-marker",
                "mapping-finished",
                "--timeout-s",
                "2",
                "--",
                sys.executable,
                "-u",
                "-c",
                "import time; print('mapping-finished'); time.sleep(5)",
            ]
            stdout_bytes = io.BytesIO()
            with patch("sys.argv", args), patch("sys.stdout") as stdout:
                stdout.buffer = stdout_bytes
                self.assertEqual(main(), 0)

            self.assertIn("mapping-finished", log.read_text(encoding="utf-8"))
            self.assertIn("mapping-finished", stdout_bytes.getvalue().decode())

    def test_completion_marker_is_required_when_configured(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            log = Path(temporary_directory) / "target.log"
            args = [
                "run",
                "--log",
                str(log),
                "--completion-marker",
                "mapping-finished",
                "--",
                sys.executable,
                "-c",
                "print('other-output')",
            ]
            with patch("sys.argv", args), patch("sys.stdout") as stdout:
                stdout.buffer = io.BytesIO()
                self.assertEqual(main(), 3)

    def test_source_receipt_binds_successful_target_to_recursive_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "source"
            revision = _initialize_git_fixture(root, "stable\n")
            log = Path(temporary_directory) / "target.log"
            receipt = Path(temporary_directory) / "source.json"
            stdout_log = Path(temporary_directory) / "stdout.log"
            stderr_log = Path(temporary_directory) / "stderr.log"
            args = [
                "run",
                "--log",
                str(log),
                "--source-root",
                str(root),
                "--source-receipt",
                str(receipt),
                "--stdout-log",
                str(stdout_log),
                "--stderr-log",
                str(stderr_log),
                "--",
                sys.executable,
                "-c",
                "print('target-output')",
            ]
            with patch("sys.argv", args), patch("sys.stdout") as stdout:
                stdout.buffer = io.BytesIO()
                self.assertEqual(main(), 0)
            payload = json.loads(receipt.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema_version"], 2)
            self.assertEqual(payload["source"]["revision"], revision)
            self.assertEqual(
                payload["execution"]["stdout_bytes"], len(b"target-output\n")
            )
            self.assertEqual(payload["execution"]["stderr_bytes"], 0)

    def test_source_mutation_during_target_prevents_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "source"
            _initialize_git_fixture(root, "before\n")
            log = Path(temporary_directory) / "target.log"
            receipt = Path(temporary_directory) / "source.json"
            stdout_log = Path(temporary_directory) / "stdout.log"
            stderr_log = Path(temporary_directory) / "stderr.log"
            code = (
                "from pathlib import Path; Path('tracked.c').write_text('changed\\n')"
            )
            args = [
                "run",
                "--log",
                str(log),
                "--source-root",
                str(root),
                "--source-receipt",
                str(receipt),
                "--stdout-log",
                str(stdout_log),
                "--stderr-log",
                str(stderr_log),
                "--cwd",
                str(root),
                "--",
                sys.executable,
                "-c",
                code,
            ]
            with patch("sys.argv", args), patch("sys.stdout") as stdout:
                stdout.buffer = io.BytesIO()
                self.assertEqual(main(), 2)
            self.assertFalse(receipt.exists())


if __name__ == "__main__":
    unittest.main()
