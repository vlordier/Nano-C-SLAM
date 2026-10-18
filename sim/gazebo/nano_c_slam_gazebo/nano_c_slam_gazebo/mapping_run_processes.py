"""Bounded, testable supervision for the mapping collector and mission."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, cast

from .mapping_run_model import MappingRunCommands


class ManagedProcess(Protocol):
    """Small subprocess surface required by the coordinator."""

    def poll(self) -> int | None: ...

    def terminate(self) -> None: ...

    def kill(self) -> None: ...

    def send_signal(self, signal_number: int) -> None: ...

    def wait(self, timeout: float | None = None) -> int: ...


@dataclass(frozen=True)
class MappingRunProcessResult:
    """Completed child statuses and retained raw log paths."""

    collector_returncode: int
    mission_returncode: int
    collector_log: Path
    mission_log: Path
    collector_stdout: Path
    collector_stderr: Path
    mission_stdout: Path
    mission_stderr: Path


def supervise_mapping_processes(
    commands: MappingRunCommands,
    *,
    readiness_file: Path,
    run_id: str,
    log_directory: Path,
    timeout_s: float,
    pump_status: Callable[[], None],
    subscriptions_ready: Callable[[], bool] = lambda: True,
    process_factory: Callable[..., ManagedProcess] | None = None,
    process_group_exists: Callable[[ManagedProcess], bool] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    poll_interval_s: float = 0.05,
    terminate_grace_s: float = 1.0,
) -> MappingRunProcessResult:
    """Start collector before mission, then reap both under one host deadline.

    ``pump_status`` should spin the ROS status listener and raise if it receives
    malformed or otherwise unusable status. The caller validates the complete
    event stream after both children exit successfully.
    """
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
            raise ValueError(f"{name} must be finite numeric data") from error
        if not finite or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not run_id or not readiness_file.is_absolute():
        raise ValueError("run_id and an absolute collector readiness path are required")
    if readiness_file.exists():
        raise FileExistsError(
            f"collector readiness marker already exists: {readiness_file}"
        )
    if log_directory.exists():
        raise FileExistsError(f"run log directory already exists: {log_directory}")
    start_process: Callable[..., ManagedProcess] = (
        process_factory
        if process_factory is not None
        else cast(Callable[..., ManagedProcess], subprocess.Popen)
    )
    group_exists = (
        process_group_exists
        if process_group_exists is not None
        else _process_group_exists
    )
    log_directory.mkdir(parents=True)
    readiness_file.parent.mkdir(parents=True, exist_ok=True)
    collector_log = log_directory / "collector.log"
    mission_log = log_directory / "mission.log"
    collector_stdout = log_directory / "collector.stdout.log"
    collector_stderr = log_directory / "collector.stderr.log"
    mission_stdout = log_directory / "mission.stdout.log"
    mission_stderr = log_directory / "mission.stderr.log"
    collector: ManagedProcess | None = None
    mission: ManagedProcess | None = None
    deadline = monotonic() + timeout_s
    try:
        with (
            collector_stdout.open("xb") as collector_stream,
            collector_stderr.open("xb") as collector_error_stream,
        ):
            collector = start_process(
                commands.collector,
                stdout=collector_stream,
                stderr=collector_error_stream,
                start_new_session=(os.name == "posix"),
            )
            while True:
                pump_status()
                if readiness_file.is_file():
                    announced_run_id = readiness_file.read_text(
                        encoding="utf-8"
                    ).strip()
                    if announced_run_id != run_id:
                        raise RuntimeError("collector readiness run ID does not match")
                    if subscriptions_ready():
                        break
                collector_code = collector.poll()
                if collector_code is not None:
                    raise RuntimeError(
                        "collector exited before subscribing to telemetry"
                    )
                _check_deadline(deadline, monotonic)
                sleep(poll_interval_s)

            with (
                mission_stdout.open("xb") as mission_stream,
                mission_stderr.open("xb") as mission_error_stream,
            ):
                mission = start_process(
                    commands.mission,
                    stdout=mission_stream,
                    stderr=mission_error_stream,
                    start_new_session=(os.name == "posix"),
                )
                while True:
                    pump_status()
                    collector_code = collector.poll()
                    mission_code = mission.poll()
                    if collector_code not in (None, 0):
                        raise RuntimeError(
                            f"collector exited with status {collector_code}"
                        )
                    if mission_code not in (None, 0):
                        raise RuntimeError(f"mission exited with status {mission_code}")
                    if collector_code == 0 and mission_code == 0:
                        break
                    _check_deadline(deadline, monotonic)
                    sleep(poll_interval_s)
        collector_returncode = collector.wait() if collector is not None else 1
        mission_returncode = mission.wait() if mission is not None else 1
        if collector_returncode != 0 or mission_returncode != 0:
            raise RuntimeError(
                "child return codes changed while reaping completed processes"
            )
        for combined, streams in (
            (collector_log, (collector_stdout, collector_stderr)),
            (mission_log, (mission_stdout, mission_stderr)),
        ):
            contents = b"".join(stream_path.read_bytes() for stream_path in streams)
            with combined.open("xb") as destination:
                destination.write(contents)
        return MappingRunProcessResult(
            collector_returncode=collector_returncode,
            mission_returncode=mission_returncode,
            collector_log=collector_log,
            mission_log=mission_log,
            collector_stdout=collector_stdout,
            collector_stderr=collector_stderr,
            mission_stdout=mission_stdout,
            mission_stderr=mission_stderr,
        )
    except BaseException:
        _stop_and_reap(
            mission,
            terminate_grace_s,
            signal_number=signal.SIGINT,
            process_group_exists=group_exists,
            monotonic=monotonic,
            sleep=sleep,
        )
        _stop_and_reap(
            collector,
            terminate_grace_s,
            signal_number=signal.SIGTERM,
            process_group_exists=group_exists,
            monotonic=monotonic,
            sleep=sleep,
        )
        raise


def _check_deadline(deadline: float, monotonic: Callable[[], float]) -> None:
    if monotonic() >= deadline:
        raise TimeoutError("mapping run exceeded its host-side deadline")


def _stop_and_reap(
    process: ManagedProcess | None,
    grace_s: float,
    *,
    signal_number: int,
    process_group_exists: Callable[[ManagedProcess], bool],
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    if process is None:
        return
    with suppress(ProcessLookupError):
        _signal_process_group(process, signal_number)
    grace_deadline = monotonic() + grace_s
    with suppress(subprocess.TimeoutExpired):
        process.wait(timeout=grace_s)
    while process_group_exists(process) and monotonic() < grace_deadline:
        sleep(min(0.05, grace_deadline - monotonic()))
    with suppress(ProcessLookupError):
        # The direct process can exit before descendants. Force-clean the
        # isolated group after its grace interval regardless of leader status.
        _signal_process_group(process, signal.SIGKILL)
    if process.poll() is None:
        with suppress(ProcessLookupError):
            process.kill()
    process.wait()


def _signal_process_group(process: ManagedProcess, signal_number: int) -> None:
    """Signal the isolated Popen session, with a direct-process test fallback."""
    if os.name == "posix" and isinstance(process, subprocess.Popen):
        os.killpg(process.pid, signal_number)
    else:
        process.send_signal(signal_number)


def _process_group_exists(process: ManagedProcess) -> bool:
    """Check whether the isolated POSIX session still has any live members."""
    if os.name == "posix" and isinstance(process, subprocess.Popen):
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True
    return process.poll() is None
