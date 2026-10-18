"""Bounded supervision for three independent swarm mission processes."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time
from collections.abc import Callable, Mapping
from contextlib import ExitStack, suppress
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Protocol, cast

from .mission_model import SUPPORTED_DRONE_NAMES


class ManagedProcess(Protocol):
    """Process interface used by the supervisor and deterministic tests."""

    pid: int

    def poll(self) -> int | None: ...

    def terminate(self) -> None: ...

    def kill(self) -> None: ...

    def wait(self, timeout: float | None = None) -> int: ...


@dataclass(frozen=True)
class SwarmMissionProcessResult:
    """Child process outcomes and retained per-drone console logs."""

    exit_codes: dict[str, int | None]
    stdout_paths: dict[str, Path]
    stderr_paths: dict[str, Path]
    error: str | None


def supervise_swarm_mission_processes(
    commands: Mapping[str, tuple[str, ...]],
    *,
    log_directory: Path,
    timeout_s: float,
    process_factory: Callable[..., ManagedProcess] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    poll_interval_s: float = 0.05,
    terminate_grace_s: float = 1.0,
) -> SwarmMissionProcessResult:
    """Start all mission nodes and reap each under one host-side deadline."""
    for value, name in (
        (timeout_s, "timeout_s"),
        (poll_interval_s, "poll_interval_s"),
        (terminate_grace_s, "terminate_grace_s"),
    ):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be finite numeric data")
        try:
            finite = math.isfinite(value)
        except OverflowError as error:
            raise ValueError(f"{name} must be finite and positive") from error
        if not finite or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if set(commands) != set(SUPPORTED_DRONE_NAMES):
        raise ValueError("commands must name exactly cf1, cf3, and cf4")
    if any(
        not command or any(not isinstance(part, str) for part in command)
        for command in commands.values()
    ):
        raise ValueError("every mission command must be a nonempty string vector")
    if not log_directory.is_absolute():
        raise ValueError("log_directory must be an absolute path")
    if log_directory.exists():
        raise FileExistsError(f"run log directory already exists: {log_directory}")

    start_process = process_factory or cast(
        Callable[..., ManagedProcess], subprocess.Popen
    )
    log_directory.mkdir(parents=True)
    stdout_paths = {
        drone: log_directory / f"{drone}.stdout.log" for drone in SUPPORTED_DRONE_NAMES
    }
    stderr_paths = {
        drone: log_directory / f"{drone}.stderr.log" for drone in SUPPORTED_DRONE_NAMES
    }
    processes: dict[str, ManagedProcess] = {}
    exit_codes: dict[str, int | None] = dict.fromkeys(SUPPORTED_DRONE_NAMES)
    error: str | None = None

    try:
        with ExitStack() as stack:
            stdout_streams: dict[str, BinaryIO] = {
                drone: stack.enter_context(path.open("xb", buffering=0))
                for drone, path in stdout_paths.items()
            }
            stderr_streams: dict[str, BinaryIO] = {
                drone: stack.enter_context(path.open("xb", buffering=0))
                for drone, path in stderr_paths.items()
            }
            for drone in SUPPORTED_DRONE_NAMES:
                try:
                    processes[drone] = start_process(
                        commands[drone],
                        stdout=stdout_streams[drone],
                        stderr=stderr_streams[drone],
                        start_new_session=(os.name == "posix"),
                    )
                except OSError as start_error:
                    error = f"could not start {drone} mission process: {start_error}"
                    break

            deadline = monotonic() + timeout_s
            while (
                error is None
                and processes
                and any(process.poll() is None for process in processes.values())
            ):
                if monotonic() >= deadline:
                    error = "swarm missions exceeded host-side deadline"
                    break
                sleep(poll_interval_s)

            if error is not None:
                for process in processes.values():
                    _terminate_process(process, terminate_grace_s)

            for drone, process in processes.items():
                try:
                    exit_codes[drone] = process.wait(timeout=terminate_grace_s)
                except subprocess.TimeoutExpired:
                    _kill_process(process)
                    exit_codes[drone] = process.wait()
    except KeyboardInterrupt:
        error = "swarm mission supervisor was interrupted"
        for drone, process in processes.items():
            _terminate_process(process, terminate_grace_s)
            exit_codes[drone] = process.wait()
    except BaseException:
        for process in processes.values():
            _terminate_process(process, terminate_grace_s)
        raise

    return SwarmMissionProcessResult(exit_codes, stdout_paths, stderr_paths, error)


def _terminate_process(process: ManagedProcess, grace_s: float) -> None:
    if process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        if os.name == "posix" and isinstance(process, subprocess.Popen):
            os.killpg(process.pid, signal.SIGTERM)
        else:
            process.terminate()
    if os.name == "posix" and isinstance(process, subprocess.Popen):
        with suppress(subprocess.TimeoutExpired):
            process.wait(timeout=grace_s)
        deadline = time.monotonic() + grace_s
        while _process_group_exists(process) and time.monotonic() < deadline:
            time.sleep(min(0.05, deadline - time.monotonic()))
        with suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGKILL)
        if process.poll() is None:
            with suppress(ProcessLookupError):
                process.kill()
        process.wait()
        return
    try:
        process.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        _kill_process(process)
        process.wait()


def _kill_process(process: ManagedProcess) -> None:
    with suppress(ProcessLookupError):
        if os.name == "posix" and isinstance(process, subprocess.Popen):
            os.killpg(process.pid, signal.SIGKILL)
        else:
            process.kill()


def _process_group_exists(process: subprocess.Popen[bytes]) -> bool:
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True
