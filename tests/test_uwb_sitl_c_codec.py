"""Compile the owned C UWB codec and compare it with the Python golden vector."""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class UwbSitlCCodecTests(unittest.TestCase):
    def test_strict_host_codec_matches_python_golden_vector(self) -> None:
        compiler = shutil.which("cc") or shutil.which("clang")
        if compiler is None:
            self.fail("host C compiler required for UWB protocol parity test")
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "uwb_sitl_protocol_test"
            result = subprocess.run(  # noqa: S603 -- fixed compiler and owned sources
                [
                    compiler,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-pedantic",
                    "-I",
                    str(REPOSITORY_ROOT / "cslam-stm32-app/sitl"),
                    str(REPOSITORY_ROOT / "tests/stm32_uwb_sitl_protocol_test.c"),
                    str(REPOSITORY_ROOT / "cslam-stm32-app/sitl/uwb_sitl_protocol.c"),
                    "-o",
                    str(executable),
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            execution = subprocess.run(  # noqa: S603 -- executes the just-built test
                [str(executable)],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )
        self.assertEqual(execution.returncode, 0, execution.stderr)
        self.assertIn("matches golden vector", execution.stdout)


if __name__ == "__main__":
    unittest.main()
