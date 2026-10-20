from __future__ import annotations

import base64
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import cast
from unittest.mock import patch

from scripts.capture_timestamped_process import (
    capture_process,
    measure_lima_clock_offset,
)


class CaptureTimestampedProcessTests(unittest.TestCase):
    def test_capture_preserves_bytes_and_records_timestamped_lines(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output_dir = Path(temporary_directory) / "capture"
            command = [
                sys.executable,
                "-c",
                (
                    "import os, time; "
                    "os.write(1, b'first'); time.sleep(0.02); "
                    "os.write(2, b'err\\n'); os.write(1, b' line\\nsecond\\nlast')"
                ),
            ]

            return_code = capture_process(command, output_dir, timeout_s=2)

            stdout_bytes = b"first line\nsecond\nlast"
            stderr_bytes = b"err\n"
            self.assertEqual(return_code, 0)
            self.assertEqual((output_dir / "stdout.raw").read_bytes(), stdout_bytes)
            self.assertEqual((output_dir / "stderr.raw").read_bytes(), stderr_bytes)
            events = [
                json.loads(line)
                for line in (output_dir / "events.jsonl").read_text().splitlines()
            ]
            self.assertEqual([event["sequence"] for event in events], list(range(4)))
            payloads_by_stream = {
                stream: [
                    base64.b64decode(event["payload_base64"])
                    for event in events
                    if event["stream"] == stream
                ]
                for stream in ("stdout", "stderr")
            }
            self.assertEqual(
                payloads_by_stream,
                {
                    "stdout": [b"first line\n", b"second\n", b"last"],
                    "stderr": [b"err\n"],
                },
            )
            self.assertEqual(
                [event["terminated_by_newline"] for event in events],
                [True, True, True, False],
            )
            timestamps = [event["capture_monotonic_time_ns"] for event in events]
            self.assertEqual(timestamps, sorted(timestamps))
            manifest = json.loads((output_dir / "process.json").read_text())
            self.assertEqual(manifest["return_code"], 0)
            self.assertFalse(manifest["timed_out"])
            self.assertLessEqual(
                manifest["capture_started_unix_time_ns"],
                manifest["capture_finished_unix_time_ns"],
            )
            self.assertEqual(
                manifest["bytes_captured_by_stream"],
                {"stdout": len(stdout_bytes), "stderr": len(stderr_bytes)},
            )
            self.assertEqual(
                manifest["bytes_missing_from_events_by_stream"],
                {"stdout": 0, "stderr": 0},
            )

    def test_timeout_terminates_child_and_records_timeout(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output_dir = Path(temporary_directory) / "capture"

            return_code = capture_process(
                [
                    sys.executable,
                    "-c",
                    "import time; print('started', flush=True); time.sleep(30)",
                ],
                output_dir,
                timeout_s=0.1,
                terminate_grace_s=0.1,
            )

            self.assertEqual(return_code, 124)
            manifest = json.loads((output_dir / "process.json").read_text())
            self.assertTrue(manifest["timed_out"])
            events = [
                json.loads(line)
                for line in (output_dir / "events.jsonl").read_text().splitlines()
            ]
            self.assertEqual(events[0]["text"], "started\n")

    def test_timeout_still_applies_after_child_closes_both_output_pipes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output_dir = Path(temporary_directory) / "capture"

            return_code = capture_process(
                [
                    sys.executable,
                    "-c",
                    "import os, time; os.close(1); os.close(2); time.sleep(30)",
                ],
                output_dir,
                timeout_s=0.1,
                terminate_grace_s=0.1,
            )

            self.assertEqual(return_code, 124)
            manifest = json.loads((output_dir / "process.json").read_text())
            self.assertTrue(manifest["timed_out"])

    def test_clock_offset_uses_lowest_round_trip_sample(self) -> None:
        completed = [
            subprocess.CompletedProcess([], 0, stdout="900\n"),
            subprocess.CompletedProcess([], 0, stdout="2000\n"),
        ]
        clock = iter([100, 200, 300, 400, 500, 600, 700, 800])
        with (
            patch(
                "scripts.capture_timestamped_process.subprocess.run",
                side_effect=completed,
            ),
            patch(
                "scripts.capture_timestamped_process.time.time_ns",
                side_effect=[900, 1100, 1900, 2100],
            ),
            patch(
                "scripts.capture_timestamped_process.time.monotonic_ns",
                side_effect=lambda: next(clock),
            ),
        ):
            offset = measure_lima_clock_offset("test-vm", sample_count=2, timeout_s=1)

        self.assertEqual(offset["guest_minus_host_offset_midpoint_ns"], -100)
        self.assertEqual(offset["guest_minus_host_offset_uncertainty_ns"], 100)
        best_sample = cast(dict[str, int], offset["best_sample"])
        self.assertEqual(best_sample["host_monotonic_round_trip_ns"], 100)


if __name__ == "__main__":
    unittest.main()
