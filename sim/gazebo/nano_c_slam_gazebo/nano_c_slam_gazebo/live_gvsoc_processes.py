"""Bounded process supervision for one live Gazebo-to-GVSOC mapping run."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time
from collections.abc import Callable, Mapping
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, cast


class LiveProcess(Protocol):
    """Subprocess operations used by the live-run coordinator."""

    def poll(self) -> int | None: ...

    def terminate(self) -> None: ...

    def kill(self) -> None: ...

    def send_signal(self, signal_number: int) -> None: ...

    def wait(self, timeout: float | None = None) -> int: ...


@dataclass(frozen=True)
class LiveGvsocCommands:
    """Commands and environment for the receiver and Gazebo mission runner."""

    gvsoc: tuple[str, ...]
    gazebo: tuple[str, ...]
    gvsoc_environment: Mapping[str, str]
    gazebo_environment: Mapping[str, str]
    working_directory: Path


@dataclass(frozen=True)
class LiveGvsocResult:
    """Child exit codes and retained stdout/stderr paths."""

    gvsoc_returncode: int
    gazebo_returncode: int
    gvsoc_stdout: Path
    gvsoc_stderr: Path
    gazebo_stdout: Path
    gazebo_stderr: Path


def supervise_live_gvsoc_mapping(
    commands: LiveGvsocCommands,
    *,
    run_id: str,
    socket_path: Path,
    readiness_path: Path,
    log_directory: Path,
    timeout_s: float,
    process_factory: Callable[..., LiveProcess] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    poll_interval_s: float = 0.05,
    terminate_grace_s: float = 1.0,
    process_group_exists: Callable[[LiveProcess], bool] | None = None,
) -> LiveGvsocResult:
    """Start GVSOC first, then Gazebo only after firmware RX readiness.

    The exclusive lock prevents overlapping launches. Paths are never removed
    before ownership is established, so stale endpoints fail safely.
    """
    for value, name in (
        (timeout_s, "timeout_s"),
        (poll_interval_s, "poll_interval_s"),
        (terminate_grace_s, "terminate_grace_s"),
    ):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be finite numeric data")
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not run_id or not socket_path.is_absolute() or not readiness_path.is_absolute():
        raise ValueError("run ID and absolute IPC paths are required")
    if socket_path.exists() or readiness_path.exists():
        raise FileExistsError("stale live GVSOC socket or readiness path exists")
    if log_directory.exists():
        raise FileExistsError(
            f"live GVSOC log directory already exists: {log_directory}"
        )
    lock_path = log_directory.with_suffix(".lock")
    log_directory.parent.mkdir(parents=True, exist_ok=True)
    lock_descriptor = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    try:
        os.write(lock_descriptor, f"{run_id}\n".encode())
    finally:
        os.close(lock_descriptor)
    try:
        log_directory.mkdir()
    except OSError:
        lock_path.unlink(missing_ok=True)
        raise
    stdout_paths = (
        log_directory / "gvsoc.stdout.log",
        log_directory / "gazebo.stdout.log",
    )
    stderr_paths = (
        log_directory / "gvsoc.stderr.log",
        log_directory / "gazebo.stderr.log",
    )
    start_process: Callable[..., LiveProcess] = (
        process_factory
        if process_factory is not None
        else cast(Callable[..., LiveProcess], subprocess.Popen)
    )
    group_exists = process_group_exists or _process_group_exists
    gvsoc: LiveProcess | None = None
    gazebo: LiveProcess | None = None
    deadline = monotonic() + timeout_s
    try:
        with (
            stdout_paths[0].open("xb") as gvsoc_stdout,
            stderr_paths[0].open("xb") as gvsoc_stderr,
        ):
            gvsoc = start_process(
                commands.gvsoc,
                stdout=gvsoc_stdout,
                stderr=gvsoc_stderr,
                env={**os.environ, **commands.gvsoc_environment},
                cwd=commands.working_directory,
                start_new_session=(os.name == "posix"),
            )
            while not readiness_path.is_file():
                code = gvsoc.poll()
                if code is not None:
                    raise RuntimeError(
                        f"GVSOC exited with status {code} before firmware readiness"
                    )
                _check_deadline(deadline, monotonic)
                sleep(poll_interval_s)
            if readiness_path.read_text(encoding="utf-8").strip() != run_id:
                raise RuntimeError("GVSOC readiness run ID does not match")
            if not socket_path.is_socket():
                raise RuntimeError("GVSOC readiness was published without its socket")

            with (
                stdout_paths[1].open("xb") as gazebo_stdout,
                stderr_paths[1].open("xb") as gazebo_stderr,
            ):
                gazebo = start_process(
                    commands.gazebo,
                    stdout=gazebo_stdout,
                    stderr=gazebo_stderr,
                    env={**os.environ, **commands.gazebo_environment},
                    cwd=commands.working_directory,
                    start_new_session=(os.name == "posix"),
                )
                while True:
                    gvsoc_code = gvsoc.poll()
                    gazebo_code = gazebo.poll()
                    if gvsoc_code not in (None, 0):
                        raise RuntimeError(f"GVSOC exited with status {gvsoc_code}")
                    if gazebo_code not in (None, 0):
                        raise RuntimeError(
                            f"Gazebo mapping runner exited with status {gazebo_code}"
                        )
                    if gvsoc_code == 0 and gazebo_code == 0:
                        break
                    _check_deadline(deadline, monotonic)
                    sleep(poll_interval_s)

        gvsoc_code = gvsoc.wait() if gvsoc is not None else 1
        gazebo_code = gazebo.wait() if gazebo is not None else 1
        if gvsoc_code != 0 or gazebo_code != 0:
            raise RuntimeError("live GVSOC child status changed while reaping")
        lingering_groups = [
            process
            for process in (gvsoc, gazebo)
            if process is not None and group_exists(process)
        ]
        if lingering_groups:
            for process in lingering_groups:
                _stop_process(
                    process,
                    terminate_grace_s,
                    monotonic=monotonic,
                    sleep=sleep,
                    process_group_exists=group_exists,
                )
            raise RuntimeError("owned child process group remained after clean exit")
        return LiveGvsocResult(
            gvsoc_code,
            gazebo_code,
            stdout_paths[0],
            stderr_paths[0],
            stdout_paths[1],
            stderr_paths[1],
        )
    except BaseException:
        _stop_process(
            gazebo,
            terminate_grace_s,
            monotonic=monotonic,
            sleep=sleep,
            process_group_exists=group_exists,
        )
        _stop_process(
            gvsoc,
            terminate_grace_s,
            monotonic=monotonic,
            sleep=sleep,
            process_group_exists=group_exists,
        )
        raise
    finally:
        with suppress(OSError):
            if readiness_path.read_text(encoding="utf-8").strip() == run_id:
                readiness_path.unlink(missing_ok=True)
        if gvsoc is not None and socket_path.is_socket():
            socket_path.unlink(missing_ok=True)
        lock_path.unlink(missing_ok=True)


def _check_deadline(deadline: float, monotonic: Callable[[], float]) -> None:
    if monotonic() >= deadline:
        raise TimeoutError("live Gazebo/GVSOC run exceeded its wall-clock deadline")


def _stop_process(
    process: LiveProcess | None,
    grace_s: float,
    *,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
    process_group_exists: Callable[[LiveProcess], bool],
) -> None:
    if process is None:
        return
    if isinstance(process, subprocess.Popen) and os.name == "posix":
        with suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGINT)
    else:
        with suppress(ProcessLookupError):
            process.send_signal(signal.SIGINT)
    with suppress(subprocess.TimeoutExpired):
        process.wait(timeout=grace_s)
    stop_deadline = monotonic() + grace_s
    while process_group_exists(process) and monotonic() < stop_deadline:
        sleep(min(0.05, max(0.0, stop_deadline - monotonic())))
    if process_group_exists(process):
        if isinstance(process, subprocess.Popen) and os.name == "posix":
            with suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
        else:
            with suppress(ProcessLookupError):
                process.kill()
    if process.poll() is None:
        process.wait()


def _process_group_exists(process: LiveProcess) -> bool:
    """Check whether a successfully launched process left owned descendants."""
    if isinstance(process, subprocess.Popen) and os.name == "posix":
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True
    return process.poll() is None
