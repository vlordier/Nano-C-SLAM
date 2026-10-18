"""Process evidence tests use local child processes and no flight services."""

from __future__ import annotations

import hashlib
import os
import sys
import unittest
from unittest.mock import patch

from scripts.process_provenance import capture_process


class ProcessProvenanceTests(unittest.TestCase):
    def test_captures_separate_stream_hashes_and_only_allowlisted_environment(
        self,
    ) -> None:
        with patch.dict(
            os.environ,
            {
                "NCS_PROVENANCE_ALLOWED": "",
                "NCS_PROVENANCE_SECRET": "must-not-be-recorded",
            },
        ):
            captured = capture_process(
                (
                    sys.executable,
                    "-c",
                    "import os; print('out:' + os.getenv("
                    "'NCS_PROVENANCE_ALLOWED', 'missing')); "
                    "print('err', file=__import__('sys').stderr)",
                ),
                environment_allowlist=(
                    "NCS_PROVENANCE_ALLOWED",
                    "NCS_PROVENANCE_ABSENT",
                ),
            )

        self.assertEqual(captured.execution.exit_code, 0)
        self.assertEqual(captured.stdout, b"out:\n")
        self.assertEqual(captured.stderr, b"err\n")
        self.assertEqual(
            captured.execution.stdout_sha256,
            hashlib.sha256(captured.stdout).hexdigest(),
        )
        self.assertEqual(
            captured.execution.stderr_sha256,
            hashlib.sha256(captured.stderr).hexdigest(),
        )
        self.assertEqual(
            captured.execution.environment,
            (("NCS_PROVENANCE_ABSENT", None), ("NCS_PROVENANCE_ALLOWED", "")),
        )
        self.assertNotIn("NCS_PROVENANCE_SECRET", dict(captured.execution.environment))
        self.assertFalse(captured.timed_out)

    def test_timeout_records_partial_streams_and_null_exit_code(self) -> None:
        captured = capture_process(
            (
                sys.executable,
                "-c",
                "import time; print('started', flush=True); time.sleep(10)",
            ),
            environment_allowlist=(),
            timeout_seconds=0.1,
        )

        self.assertTrue(captured.timed_out)
        self.assertIsNone(captured.execution.exit_code)
        self.assertIn(b"started", captured.stdout)
        self.assertIsNotNone(captured.execution.stdout_sha256)

    def test_rejects_unallowlisted_overrides_and_empty_commands(self) -> None:
        with self.assertRaisesRegex(ValueError, "allowlisted"):
            capture_process(
                (sys.executable, "-c", "pass"),
                environment_allowlist=(),
                environment_overrides={"SECRET": "value"},
            )
        with self.assertRaisesRegex(ValueError, "command"):
            capture_process((), environment_allowlist=())


if __name__ == "__main__":
    unittest.main()
