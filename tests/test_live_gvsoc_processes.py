"""Hardware-free tests for live Gazebo/GVSOC child lifecycle supervision."""

from __future__ import annotations

import socket
import tempfile
import unittest
from pathlib import Path
from typing import cast

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.live_gvsoc_processes import (
    LiveGvsocCommands,
    supervise_live_gvsoc_mapping,
)

RUN_ID = "15a06f08-657e-4012-922e-4ca35f7dda22"


class _Process:
    def __init__(self, code: int | None) -> None:
        self.code = code
        self.signals: list[int] = []
        self.killed = False

    def poll(self) -> int | None:
        return self.code

    def terminate(self) -> None:
        self.code = -15

    def kill(self) -> None:
        self.killed = True
        self.code = -9

    def send_signal(self, signal_number: int) -> None:
        self.signals.append(signal_number)
        self.code = -signal_number

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        if self.code is None:
            self.code = -15
        return self.code


class LiveGvsocProcessTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.socket_path = self.root / "stream.sock"
        self.ready_path = self.root / "stream.ready"
        self.log_directory = self.root / "run.logs"
        self.commands = LiveGvsocCommands(
            gvsoc=("gvsoc",),
            gazebo=("gazebo",),
            gvsoc_environment={"RUN_ID": RUN_ID},
            gazebo_environment={"ROS_DOMAIN_ID": "42"},
            working_directory=Path.cwd(),
        )
        self.server: socket.socket | None = None

    def tearDown(self) -> None:
        if self.server is not None:
            self.server.close()
        self.temp.cleanup()

    def _ready_gvsoc(self, *, code: int | None) -> _Process:
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(str(self.socket_path))
        self.server.listen(1)
        self.ready_path.write_text(f"{RUN_ID}\n", encoding="utf-8")
        return _Process(code)

    def test_starts_gazebo_only_after_gvsoc_publishes_matching_ready_socket(
        self,
    ) -> None:
        order: list[str] = []
        processes: list[_Process] = []

        def factory(command: tuple[str, ...], **kwargs: object) -> _Process:
            del kwargs
            order.append(command[0])
            process = (
                self._ready_gvsoc(code=0) if command[0] == "gvsoc" else _Process(0)
            )
            processes.append(process)
            return process

        result = supervise_live_gvsoc_mapping(
            self.commands,
            run_id=RUN_ID,
            socket_path=self.socket_path,
            readiness_path=self.ready_path,
            log_directory=self.log_directory,
            timeout_s=1.0,
            process_factory=factory,
        )

        self.assertEqual(order, ["gvsoc", "gazebo"])
        self.assertEqual((result.gvsoc_returncode, result.gazebo_returncode), (0, 0))
        self.assertTrue(result.gvsoc_stdout.is_file())
        self.assertFalse(self.ready_path.exists())
        self.assertFalse(self.socket_path.exists())

    def test_target_failure_before_readiness_never_starts_gazebo(self) -> None:
        started: list[str] = []

        def factory(command: tuple[str, ...], **kwargs: object) -> _Process:
            del kwargs
            started.append(command[0])
            return _Process(7)

        with self.assertRaisesRegex(RuntimeError, "before firmware readiness"):
            supervise_live_gvsoc_mapping(
                self.commands,
                run_id=RUN_ID,
                socket_path=self.socket_path,
                readiness_path=self.ready_path,
                log_directory=self.log_directory,
                timeout_s=1.0,
                process_factory=factory,
            )
        self.assertEqual(started, ["gvsoc"])

    def test_missing_readiness_times_out_and_signals_owned_target(self) -> None:
        clock = [0.0]
        process = _Process(None)

        def factory(command: tuple[str, ...], **kwargs: object) -> _Process:
            del command, kwargs
            return process

        with self.assertRaisesRegex(TimeoutError, "wall-clock deadline"):
            supervise_live_gvsoc_mapping(
                self.commands,
                run_id=RUN_ID,
                socket_path=self.socket_path,
                readiness_path=self.ready_path,
                log_directory=self.log_directory,
                timeout_s=0.2,
                process_factory=factory,
                monotonic=lambda: clock[0],
                sleep=lambda duration: clock.__setitem__(0, clock[0] + duration),
            )
        self.assertTrue(process.signals)

    def test_duplicate_launcher_lock_is_refused(self) -> None:
        lock_path = self.log_directory.with_suffix(".lock")
        lock_path.write_text("another run\n", encoding="utf-8")
        with self.assertRaises(FileExistsError):
            supervise_live_gvsoc_mapping(
                self.commands,
                run_id=RUN_ID,
                socket_path=self.socket_path,
                readiness_path=self.ready_path,
                log_directory=self.log_directory,
                timeout_s=1.0,
                process_factory=lambda *_args, **_kwargs: _Process(0),
            )
        self.assertEqual(lock_path.read_text(encoding="utf-8"), "another run\n")

    def test_child_failure_cleans_up_both_owned_processes_and_ipc_paths(self) -> None:
        started: list[_Process] = []

        def factory(command: tuple[str, ...], **kwargs: object) -> _Process:
            del kwargs
            process = (
                self._ready_gvsoc(code=None) if command[0] == "gvsoc" else _Process(4)
            )
            started.append(process)
            return process

        with self.assertRaisesRegex(RuntimeError, "Gazebo mapping runner exited"):
            supervise_live_gvsoc_mapping(
                self.commands,
                run_id=RUN_ID,
                socket_path=self.socket_path,
                readiness_path=self.ready_path,
                log_directory=self.log_directory,
                timeout_s=1.0,
                process_factory=factory,
            )
        self.assertEqual(len(started), 2)
        self.assertTrue(all(process.signals for process in started))
        self.assertFalse(self.socket_path.exists())
        self.assertFalse(self.ready_path.exists())
        self.assertFalse(self.log_directory.with_suffix(".lock").exists())

    def test_clean_exit_also_reaps_owned_descendant_process_groups(self) -> None:
        processes: list[_Process] = []

        def factory(command: tuple[str, ...], **kwargs: object) -> _Process:
            del kwargs
            process = (
                self._ready_gvsoc(code=0) if command[0] == "gvsoc" else _Process(0)
            )
            processes.append(process)
            return process

        with self.assertRaisesRegex(RuntimeError, "group remained"):
            supervise_live_gvsoc_mapping(
                self.commands,
                run_id=RUN_ID,
                socket_path=self.socket_path,
                readiness_path=self.ready_path,
                log_directory=self.log_directory,
                timeout_s=1.0,
                process_factory=factory,
                process_group_exists=lambda process: (
                    not bool(cast(_Process, process).signals)
                ),
            )
        self.assertEqual(len(processes), 2)
        self.assertTrue(all(process.signals for process in processes))


if __name__ == "__main__":
    unittest.main()
