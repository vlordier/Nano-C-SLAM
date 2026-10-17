"""Deterministic process-lifecycle tests without ROS or Gazebo."""

from __future__ import annotations

import signal
import subprocess
import tempfile
import unittest
from collections.abc import Callable
from pathlib import Path

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_run_model import (
    MappingRunCommands,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_run_processes import (
    MappingRunProcessResult,
    supervise_mapping_processes,
)


class _FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration_s: float) -> None:
        self.now += duration_s


class _FakeProcess:
    def __init__(self, *, exit_after: int | None, returncode: int) -> None:
        self.exit_after = exit_after
        self.returncode = returncode
        self.poll_count = 0
        self.terminated = False
        self.killed = False
        self.received_signal: int | None = None
        self.waited = False

    def poll(self) -> int | None:
        self.poll_count += 1
        if self.terminated:
            return -15
        if self.killed:
            return -9
        if self.exit_after is not None and self.poll_count >= self.exit_after:
            return self.returncode
        return None

    def terminate(self) -> None:
        self.terminated = True

    def send_signal(self, signal_number: int) -> None:
        self.received_signal = signal_number
        self.terminated = True

    def kill(self) -> None:
        self.killed = True

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        self.waited = True
        result = self.poll()
        if result is None:
            raise subprocess.TimeoutExpired("fake", 1.0)
        return result


class MappingRunProcessTests(unittest.TestCase):
    def setUp(self) -> None:
        self.commands = MappingRunCommands(
            collector=("collector",),
            mission=("mission",),
        )
        self.run_id = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
        self.last_processes: list[_FakeProcess] = []
        self.commands_started: list[tuple[str, ...]] = []

    def run_supervisor(
        self,
        temporary_directory: str,
        process_options: list[tuple[int | None, int]],
        *,
        create_ready: bool = True,
        pump_status: Callable[[], None] = lambda: None,
        subscriptions_ready: Callable[[], bool] = lambda: True,
        timeout_s: float = 2.0,
    ) -> tuple[MappingRunProcessResult, list[_FakeProcess], _FakeClock, Path]:
        directory = Path(temporary_directory)
        readiness_file = directory / "collector.ready"
        clock = _FakeClock()
        processes: list[_FakeProcess] = []
        self.last_processes = processes
        self.commands_started = []

        def process_factory(command: tuple[str, ...], **kwargs: object) -> _FakeProcess:
            del kwargs
            self.commands_started.append(command)
            if command == self.commands.mission:
                assert readiness_file.is_file()
                assert subscriptions_ready()
            index = 0 if command == self.commands.collector else 1
            if index >= len(processes):
                exit_after, returncode = process_options[index]
                process = _FakeProcess(exit_after=exit_after, returncode=returncode)
                processes.append(process)
            if index == 0 and create_ready:
                readiness_file.write_text(self.run_id, encoding="utf-8")
            return processes[index]

        result = supervise_mapping_processes(
            self.commands,
            readiness_file=readiness_file,
            run_id=self.run_id,
            log_directory=directory / "logs",
            timeout_s=timeout_s,
            pump_status=pump_status,
            process_factory=process_factory,
            subscriptions_ready=subscriptions_ready,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
            poll_interval_s=0.1,
            terminate_grace_s=0.2,
        )
        return result, processes, clock, readiness_file

    def test_starts_mission_only_after_ready_collector_and_waits_for_both(self) -> None:
        subscription_checks = 0

        def subscriptions_ready() -> bool:
            nonlocal subscription_checks
            subscription_checks += 1
            return subscription_checks >= 3

        with tempfile.TemporaryDirectory() as temporary_directory:
            result, processes, _clock, readiness_file = self.run_supervisor(
                temporary_directory,
                [(4, 0), (5, 0)],
                subscriptions_ready=subscriptions_ready,
            )
            self.assertEqual(result.collector_returncode, 0)
            self.assertEqual(result.mission_returncode, 0)
            self.assertTrue(result.collector_log.is_file())
            self.assertTrue(result.mission_log.is_file())
            self.assertTrue(all(process.waited for process in processes))
            self.assertEqual(readiness_file.read_text(encoding="utf-8"), self.run_id)
            self.assertEqual(
                self.commands_started,
                [self.commands.collector, self.commands.mission],
            )
            self.assertGreaterEqual(subscription_checks, 3)

    def test_rejects_collector_exit_before_readiness(self) -> None:
        with (
            tempfile.TemporaryDirectory() as temporary_directory,
            self.assertRaisesRegex(RuntimeError, "before subscribing"),
        ):
            self.run_supervisor(
                temporary_directory,
                [(1, 0), (1, 0)],
                create_ready=False,
            )
        self.assertEqual(len(self.last_processes), 1)
        self.assertFalse(self.last_processes[0].terminated)
        self.assertTrue(self.last_processes[0].waited)

    def test_rejects_wrong_readiness_identity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            ready = Path(temporary_directory) / "collector.ready"
            processes: list[_FakeProcess] = []

            def factory(_command: tuple[str, ...], **_kwargs: object) -> _FakeProcess:
                ready.write_text(
                    "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb", encoding="utf-8"
                )
                process = _FakeProcess(exit_after=None, returncode=0)
                processes.append(process)
                return process

            with self.assertRaisesRegex(RuntimeError, "does not match"):
                supervise_mapping_processes(
                    self.commands,
                    readiness_file=ready,
                    run_id=self.run_id,
                    log_directory=Path(temporary_directory) / "logs",
                    timeout_s=1,
                    pump_status=lambda: None,
                    process_factory=factory,
                )
            self.assertTrue(processes[0].terminated)
            self.assertTrue(processes[0].waited)

    def test_rejects_a_preexisting_readiness_marker_before_process_start(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            ready = Path(temporary_directory) / "collector.ready"
            ready.write_text(self.run_id, encoding="utf-8")
            processes: list[_FakeProcess] = []

            def factory(_command: tuple[str, ...], **_kwargs: object) -> _FakeProcess:
                process = _FakeProcess(exit_after=None, returncode=0)
                processes.append(process)
                return process

            with self.assertRaisesRegex(FileExistsError, "already exists"):
                supervise_mapping_processes(
                    self.commands,
                    readiness_file=ready,
                    run_id=self.run_id,
                    log_directory=Path(temporary_directory) / "logs",
                    timeout_s=1,
                    pump_status=lambda: None,
                    process_factory=factory,
                )
            self.assertEqual(processes, [])

    def test_mission_nonzero_exit_terminates_and_reaps_collector(self) -> None:
        with (
            tempfile.TemporaryDirectory() as temporary_directory,
            self.assertRaisesRegex(RuntimeError, "mission exited with status 2"),
        ):
            self.run_supervisor(
                temporary_directory,
                [(None, 0), (1, 2)],
            )
        self.assertTrue(self.last_processes[0].terminated)
        self.assertTrue(self.last_processes[0].waited)
        self.assertEqual(self.last_processes[0].received_signal, signal.SIGTERM)

    def test_collector_nonzero_exit_terminates_and_reaps_mission(self) -> None:
        with (
            tempfile.TemporaryDirectory() as temporary_directory,
            self.assertRaisesRegex(RuntimeError, "collector exited with status 3"),
        ):
            self.run_supervisor(
                temporary_directory,
                [(2, 3), (None, 0)],
            )
        self.assertTrue(self.last_processes[1].terminated)
        self.assertTrue(self.last_processes[1].waited)
        self.assertEqual(self.last_processes[1].received_signal, signal.SIGINT)

    def test_timeout_and_interrupt_both_terminate_children(self) -> None:
        with (
            tempfile.TemporaryDirectory() as temporary_directory,
            self.assertRaisesRegex(TimeoutError, "host-side deadline"),
        ):
            self.run_supervisor(
                temporary_directory,
                [(None, 0), (None, 0)],
                timeout_s=0.2,
            )
        self.assertTrue(all(process.terminated for process in self.last_processes))
        self.assertTrue(all(process.waited for process in self.last_processes))
        self.assertEqual(self.last_processes[1].received_signal, signal.SIGINT)
        self.assertEqual(self.last_processes[0].received_signal, signal.SIGTERM)

        with tempfile.TemporaryDirectory() as temporary_directory:
            pumps = 0

            def interrupt_after_launch() -> None:
                nonlocal pumps
                pumps += 1
                if pumps > 1:
                    raise KeyboardInterrupt

            with self.assertRaises(KeyboardInterrupt):
                self.run_supervisor(
                    temporary_directory,
                    [(None, 0), (None, 0)],
                    pump_status=interrupt_after_launch,
                )
            self.assertTrue(all(process.terminated for process in self.last_processes))
            self.assertTrue(all(process.waited for process in self.last_processes))
            self.assertEqual(self.last_processes[1].received_signal, signal.SIGINT)


if __name__ == "__main__":
    unittest.main()
