"""Hardware-free lifecycle checks for the three-process swarm supervisor."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.swarm_mission_processes import (
    ManagedProcess,
    supervise_swarm_mission_processes,
)


def _command(source: str) -> tuple[str, ...]:
    return (sys.executable, "-c", source)


class _FakeProcess:
    def __init__(self) -> None:
        self.returncode: int | None = None

    def poll(self) -> int | None:
        return self.returncode

    def terminate(self) -> None:
        self.returncode = -15

    def kill(self) -> None:
        self.returncode = -9

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        if self.returncode is None:
            raise subprocess.TimeoutExpired("fake-mission", 1.0)
        return self.returncode


class SwarmMissionProcessTests(unittest.TestCase):
    def commands(self, source: str = "pass") -> dict[str, tuple[str, ...]]:
        return {drone: _command(source) for drone in SUPPORTED_DRONE_NAMES}

    def test_runs_three_children_and_retains_each_console_log(self) -> None:
        commands = {
            drone: _command(f"print({drone!r})") for drone in SUPPORTED_DRONE_NAMES
        }
        with tempfile.TemporaryDirectory() as temporary:
            log_directory = Path(temporary) / "logs"
            result = supervise_swarm_mission_processes(
                commands,
                log_directory=log_directory,
                timeout_s=5.0,
            )

            self.assertEqual(result.exit_codes, dict.fromkeys(SUPPORTED_DRONE_NAMES, 0))
            self.assertIsNone(result.error)
            for drone, path in result.stdout_paths.items():
                self.assertTrue(path.is_file())
                self.assertIn(drone, path.read_text(encoding="utf-8"))
            self.assertTrue(
                all(path.is_file() for path in result.stderr_paths.values())
            )

    def test_retains_nonzero_exit_without_cancelling_other_missions(self) -> None:
        commands = self.commands()
        commands["cf3"] = _command("raise SystemExit(7)")
        with tempfile.TemporaryDirectory() as temporary:
            result = supervise_swarm_mission_processes(
                commands,
                log_directory=Path(temporary) / "logs",
                timeout_s=5.0,
            )

        self.assertEqual(result.exit_codes, {"cf1": 0, "cf3": 7, "cf4": 0})
        self.assertIsNone(result.error)

    def test_timeout_terminates_and_reaps_all_live_children(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            result = supervise_swarm_mission_processes(
                self.commands("import time; time.sleep(30)"),
                log_directory=Path(temporary) / "logs",
                timeout_s=0.1,
                poll_interval_s=0.01,
                terminate_grace_s=0.5,
            )

        self.assertEqual(result.error, "swarm missions exceeded host-side deadline")
        self.assertEqual(set(result.exit_codes), set(SUPPORTED_DRONE_NAMES))
        self.assertTrue(all(code is not None for code in result.exit_codes.values()))

    def test_partial_start_failure_reaps_started_process_and_records_missing_ones(
        self,
    ) -> None:
        processes: list[_FakeProcess] = []

        def process_factory(*_args: object, **_kwargs: object) -> ManagedProcess:
            if processes:
                raise OSError("injected process start failure")
            process = _FakeProcess()
            processes.append(process)
            return process

        with tempfile.TemporaryDirectory() as temporary:
            result = supervise_swarm_mission_processes(
                self.commands(),
                log_directory=Path(temporary) / "logs",
                timeout_s=1.0,
                process_factory=process_factory,
            )

        self.assertIn("could not start cf3", result.error or "")
        self.assertEqual(result.exit_codes, {"cf1": -15, "cf3": None, "cf4": None})
        self.assertEqual(processes[0].returncode, -15)

    def test_keyboard_interrupt_reaps_children_and_returns_partial_outcomes(
        self,
    ) -> None:
        processes: list[_FakeProcess] = []

        def process_factory(*_args: object, **_kwargs: object) -> ManagedProcess:
            process = _FakeProcess()
            processes.append(process)
            return process

        def interrupt(_duration_s: float) -> None:
            raise KeyboardInterrupt

        with tempfile.TemporaryDirectory() as temporary:
            result = supervise_swarm_mission_processes(
                self.commands(),
                log_directory=Path(temporary) / "logs",
                timeout_s=5.0,
                process_factory=process_factory,
                sleep=interrupt,
            )

        self.assertEqual(result.error, "swarm mission supervisor was interrupted")
        self.assertEqual(result.exit_codes, dict.fromkeys(SUPPORTED_DRONE_NAMES, -15))
        self.assertTrue(all(process.returncode == -15 for process in processes))


if __name__ == "__main__":
    unittest.main()
