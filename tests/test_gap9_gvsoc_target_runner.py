"""Tests for bounded GAP9 GVSOC target execution and raw-log retention."""

from __future__ import annotations

import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.run_gap9_gvsoc_target import main


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


if __name__ == "__main__":
    unittest.main()
