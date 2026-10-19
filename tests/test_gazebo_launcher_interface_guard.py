"""Tests for the Gazebo launcher's ROS message compatibility preflight."""

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = REPO_ROOT / "scripts/run_gazebo_macos.sh"
ABSEIL_GUARD = REPO_ROOT / "scripts/check_gazebo_abseil_opt.sh"


class GazeboLauncherInterfaceGuardTests(unittest.TestCase):
    def test_rejects_a_second_launch_without_changing_the_active_abseil_link(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            active_launch = root / "nano-c-slam-ros2.active" / "abseil"
            active_launch.mkdir(parents=True)
            homebrew_opt = root / "opt/abseil"
            homebrew_opt.parent.mkdir()
            homebrew_opt.symlink_to(active_launch, target_is_directory=True)

            result = subprocess.run(  # noqa: S603 - fixed repository helper and temporary path.
                ["/bin/bash", str(ABSEIL_GUARD), str(homebrew_opt)],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Another Nano-C-SLAM Gazebo launch", result.stderr)
            self.assertEqual(homebrew_opt.resolve(), active_launch.resolve())

    def test_allows_a_normal_homebrew_formula_link(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            formula = root / "Cellar/abseil/1.0.0"
            formula.mkdir(parents=True)
            homebrew_opt = root / "opt/abseil"
            homebrew_opt.parent.mkdir()
            homebrew_opt.symlink_to(formula, target_is_directory=True)

            result = subprocess.run(  # noqa: S603 - fixed repository helper and temporary path.
                ["/bin/bash", str(ABSEIL_GUARD), str(homebrew_opt)],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)

    def _run_preflight(
        self, *, command_status: int, output: str
    ) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            bin_directory = Path(temporary_directory)
            uv = bin_directory / "uv"
            uv.write_text(
                f"#!/bin/bash\nprintf '%s' {output!r}\nexit {command_status}\n",
                encoding="utf-8",
            )
            uv.chmod(0o755)
            launcher = LAUNCHER.read_text(encoding="utf-8")
            interface_guard = (
                "if ! interface_definition="
                + launcher.split("if ! interface_definition=", 1)[1].split(
                    "export PYTHONPATH", 1
                )[0]
            )
            script = "\n".join(
                (
                    "#!/bin/bash",
                    "set -euo pipefail",
                    f"uv='{uv}'",
                    'python311="/unused/python"',
                    "set -u",
                    'export PATH="$PATH"',
                    interface_guard,
                    "echo LAUNCH_ALLOWED",
                )
            )
            return subprocess.run(  # noqa: S603 - test script uses repository code and constants.
                ["/bin/bash", "-c", script],
                check=False,
                capture_output=True,
                text=True,
                env={**os.environ, "PATH": f"{bin_directory}:{os.environ['PATH']}"},
            )

    def test_rejects_missing_message_type(self) -> None:
        result = self._run_preflight(command_status=1, output="interface not found")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("does not provide StampedPairwiseRanges", result.stderr)
        self.assertNotIn("LAUNCH_ALLOWED", result.stdout)

    def test_rejects_stale_message_definition(self) -> None:
        result = self._run_preflight(
            command_status=0,
            output="std_msgs/Header header\nfloat32[] ranges_m\n",
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("interface is stale", result.stderr)
        self.assertNotIn("LAUNCH_ALLOWED", result.stdout)

    def test_accepts_current_message_definition(self) -> None:
        result = self._run_preflight(
            command_status=0,
            output=(
                "std_msgs/Header header\n"
                "builtin_interfaces/Time[] source_stamps\n"
                "float32[] ranges_m\n"
            ),
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("LAUNCH_ALLOWED", result.stdout)


if __name__ == "__main__":
    unittest.main()
