"""Observe and hash libraries mapped by one launch-recorded process."""

from __future__ import annotations

import hashlib
import json
import math
import platform
import shlex
import shutil
import subprocess
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path
from typing import cast

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model import (
    session_file_for_partition,
)


def _library_path(path_text: str) -> bool:
    path = Path(path_text.removesuffix(" (deleted)"))
    return (
        path.suffix in {".dylib", ".so", ".bundle"}
        or any(part.endswith(".framework") for part in path.parts)
        or path.name == "dyld"
    )


def parse_lsof_mapped_paths(
    output: bytes, *, expected_pid: int, expected_parent_pid: int | None = None
) -> tuple[Path, ...]:
    """Parse lsof NUL fields and reject any result not scoped to one PID."""
    if (
        isinstance(expected_pid, bool)
        or not isinstance(expected_pid, int)
        or expected_pid <= 0
    ):
        raise ValueError("expected process PID must be positive")
    fields = output.decode("utf-8", errors="replace").split("\0")
    process_ids: set[int] = set()
    parent_process_ids: set[int] = set()
    file_descriptor: str | None = None
    file_type: str | None = None
    file_name: str | None = None
    paths: set[Path] = set()

    def collect_file() -> None:
        if (
            file_descriptor in {"txt", "mem"}
            and file_type == "REG"
            and file_name is not None
            and Path(file_name.removesuffix(" (deleted)")).is_absolute()
            and _library_path(file_name)
        ):
            paths.add(Path(file_name.removesuffix(" (deleted)")))

    for raw_field in fields:
        field = raw_field.lstrip("\n")
        if not field:
            continue
        key, value = field[0], field[1:]
        if key == "p":
            collect_file()
            try:
                process_ids.add(int(value))
            except ValueError as error:
                raise ValueError("lsof output contains an invalid PID") from error
            file_descriptor = None
            file_type = None
            file_name = None
        elif key == "R":
            try:
                parent_process_ids.add(int(value))
            except ValueError as error:
                raise ValueError(
                    "lsof output contains an invalid parent PID"
                ) from error
        elif key == "f":
            collect_file()
            file_descriptor = value
            file_type = None
            file_name = None
        elif key == "t":
            file_type = value
        elif key == "n":
            file_name = value
    collect_file()

    if process_ids != {expected_pid}:
        raise ValueError(
            f"lsof output PIDs {sorted(process_ids)} do not match {expected_pid}"
        )
    if expected_parent_pid is not None and parent_process_ids != {expected_parent_pid}:
        raise ValueError(
            f"lsof parent PIDs {sorted(parent_process_ids)} do not match "
            f"launch PID {expected_parent_pid}"
        )
    return tuple(sorted(paths, key=str))


def _unknown_evidence(
    reason: str, *, session_id: str | None = None
) -> dict[str, object]:
    return {
        "status": "unknown",
        "reason": reason,
        "observer": "lsof -F0 -a -nP -p PID -d txt,mem",
        "session_id": session_id,
        "pid": None,
        "launch_pid": None,
        "executable_path": None,
        "observed_at_utc": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "commands": [],
        "libraries": [],
    }


def _command_record(
    argv: list[str], *, returncode: int | None, elapsed_s: float, output: bytes
) -> dict[str, object]:
    return {
        "argv": argv,
        "returncode": returncode,
        "elapsed_s": elapsed_s,
        "stdout_sha256": hashlib.sha256(output).hexdigest(),
        "stdout_bytes": len(output),
    }


def _matches_recorded_command(observed: str, expected: list[str]) -> bool:
    try:
        actual_argv = shlex.split(observed)
    except ValueError:
        return False
    if actual_argv == expected:
        return True
    # ros_gz_sim launches `ruby <gz> sim ...` through a shell. The Ruby
    # launcher later changes its process title to `gz sim ...`; it also
    # consumes the trailing --force-version option before doing so.
    gz_index = next(
        (
            index
            for index, argument in enumerate(expected)
            if Path(argument).name == "gz"
        ),
        None,
    )
    if gz_index is None or gz_index + 1 >= len(expected):
        return False
    expected_title = expected[gz_index:]
    if len(expected_title) >= 2 and expected_title[-2] == "--force-version":
        expected_title = expected_title[:-2]
    return (
        bool(actual_argv)
        and Path(actual_argv[0]).name == "gz"
        and actual_argv[1:] == expected_title[1:]
    )


def _invocation_uses_executable(argv: list[str], executable: Path) -> bool:
    expected = executable.resolve()
    for argument in argv:
        if Path(argument).name != "gz":
            continue
        candidate = Path(argument)
        if not candidate.is_absolute():
            located = shutil.which(argument)
            if located is None:
                continue
            candidate = Path(located)
        if candidate.resolve() == expected:
            return True
    return False


def _hash_mapped_file(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {"path": str(path), "sha256": None, "reason": "file is unavailable"}
    try:
        from scripts.runtime_provenance import sha256_file_stable

        digest = sha256_file_stable(path)
    except (OSError, ValueError) as error:
        return {
            "path": str(path),
            "sha256": None,
            "reason": f"stable hash failed: {type(error).__name__}",
        }
    return {"path": str(path.resolve()), "sha256": digest, "reason": None}


def _capture_process(
    *,
    session_id: str,
    pid: int,
    executable_path: str,
    launch_pid: int,
    expected_argv: list[str],
    timeout_s: float,
) -> dict[str, object]:
    started = time.monotonic()
    result: dict[str, object] = {
        "status": "unknown",
        "reason": None,
        "observer": "lsof -F0 -a -nP -p PID -d txt,mem",
        "session_id": session_id,
        "pid": pid,
        "launch_pid": launch_pid,
        "executable_path": executable_path,
        "observed_at_utc": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "commands": [],
        "libraries": [],
    }
    if platform.system() != "Darwin":
        result["reason"] = "mapped-library observer is implemented for macOS only"
        return result

    ps = shutil.which("ps")
    lsof = shutil.which("lsof") or "/usr/sbin/lsof"
    if ps is None or not Path(lsof).is_file():
        result["reason"] = "ps or lsof is unavailable"
        return result
    commands: list[dict[str, object]] = []
    result["commands"] = commands
    ps_argv = [ps, "-p", str(pid), "-o", "command="]
    try:
        command_started = time.monotonic()
        ps_result = subprocess.run(  # noqa: S603 -- fixed ps query for one PID
            ps_argv,
            check=False,
            capture_output=True,
            timeout=min(timeout_s, 3.0),
        )
        commands.append(
            _command_record(
                ps_argv,
                returncode=ps_result.returncode,
                elapsed_s=time.monotonic() - command_started,
                output=ps_result.stdout,
            )
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or b""
        if isinstance(output, str):
            output = output.encode("utf-8", errors="replace")
        commands.append(
            _command_record(
                ps_argv,
                returncode=None,
                elapsed_s=time.monotonic() - command_started,
                output=output,
            )
        )
        result["reason"] = "ps timed out while validating the recorded PID"
        result["elapsed_s"] = time.monotonic() - started
        return result
    if ps_result.returncode != 0 or not _matches_recorded_command(
        ps_result.stdout.decode("utf-8", errors="replace").strip(), expected_argv
    ):
        result["reason"] = "recorded PID is unavailable or its command changed"
        result["elapsed_s"] = time.monotonic() - started
        return result

    lsof_argv = [lsof, "-F0", "-a", "-nP", "-p", str(pid), "-d", "txt,mem"]
    try:
        command_started = time.monotonic()
        lsof_result = subprocess.run(  # noqa: S603 -- fixed lsof query for one PID
            lsof_argv,
            check=False,
            capture_output=True,
            timeout=timeout_s,
        )
        commands.append(
            _command_record(
                lsof_argv,
                returncode=lsof_result.returncode,
                elapsed_s=time.monotonic() - command_started,
                output=lsof_result.stdout,
            )
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or b""
        if isinstance(output, str):
            output = output.encode("utf-8", errors="replace")
        commands.append(
            _command_record(
                lsof_argv,
                returncode=None,
                elapsed_s=time.monotonic() - command_started,
                output=output,
            )
        )
        result["reason"] = "lsof timed out while observing mapped libraries"
        result["elapsed_s"] = time.monotonic() - started
        return result
    if lsof_result.returncode != 0:
        result["reason"] = "lsof could not observe the recorded process"
        result["elapsed_s"] = time.monotonic() - started
        return result
    try:
        mapped_paths = parse_lsof_mapped_paths(
            lsof_result.stdout,
            expected_pid=pid,
            expected_parent_pid=launch_pid,
        )
    except ValueError as error:
        result["reason"] = str(error)
        result["elapsed_s"] = time.monotonic() - started
        return result

    libraries = [_hash_mapped_file(path) for path in mapped_paths]
    result.update(
        status="observed" if libraries else "unknown",
        reason=None if libraries else "lsof reported no mapped library files",
        libraries=libraries,
        elapsed_s=time.monotonic() - started,
    )
    return result


def capture_session_gazebo_libraries(
    *,
    partition: str,
    expected_executable_path: Path | None = None,
    timeout_s: float = 10.0,
) -> dict[str, object]:
    """Capture mapped libraries only for the Gazebo PID in this session file."""
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("mapped-library timeout must be finite and positive")
    session_path = session_file_for_partition(partition)
    try:
        session = json.loads(session_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return _unknown_evidence("world-session manifest is unavailable")
    if not isinstance(session, dict) or not isinstance(session.get("session_id"), str):
        return _unknown_evidence("world-session manifest is malformed")
    session_id = cast(str, session["session_id"])
    try:
        if str(uuid.UUID(session_id)) != session_id:
            raise ValueError
    except ValueError:
        return _unknown_evidence("world-session identity is invalid")
    process_list = session.get("processes")
    if not isinstance(process_list, list):
        return _unknown_evidence(
            "world session has no launch-recorded process list", session_id=session_id
        )
    servers = [
        process
        for process in process_list
        if isinstance(process, dict) and process.get("role") == "gazebo-server"
    ]
    if len(servers) != 1:
        return _unknown_evidence(
            "world session must contain exactly one recorded Gazebo server PID",
            session_id=session_id,
        )
    server = cast(dict[str, object], servers[0])
    pid = server.get("pid")
    launch_pid = session.get("launch_pid")
    process_launch_pid = server.get("launch_pid")
    executable_path = server.get("executable_path")
    argv = server.get("argv")
    if (
        isinstance(pid, bool)
        or not isinstance(pid, int)
        or pid <= 0
        or isinstance(launch_pid, bool)
        or not isinstance(launch_pid, int)
        or launch_pid <= 0
        or isinstance(process_launch_pid, bool)
        or not isinstance(process_launch_pid, int)
        or process_launch_pid != launch_pid
        or not isinstance(executable_path, str)
        or not Path(executable_path).is_absolute()
        or not isinstance(argv, list)
        or not argv
        or any(not isinstance(argument, str) for argument in argv)
    ):
        return _unknown_evidence(
            "world-session Gazebo server identity is incomplete", session_id=session_id
        )
    if Path(executable_path).name != Path(cast(list[str], argv)[0]).name:
        return _unknown_evidence(
            "world-session Gazebo executable disagrees with command",
            session_id=session_id,
        )
    resolved_executable = Path(executable_path).resolve()
    command = cast(list[str], argv)
    if expected_executable_path is not None and not _invocation_uses_executable(
        command, expected_executable_path
    ):
        return _unknown_evidence(
            "session Gazebo command uses a different executable than this run",
            session_id=session_id,
        )
    return _capture_process(
        session_id=session_id,
        pid=pid,
        launch_pid=launch_pid,
        executable_path=str(resolved_executable),
        expected_argv=command,
        timeout_s=timeout_s,
    )
