"""Fail-closed identity and single-run claim for a launched Gazebo world."""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
import shutil
import tempfile
import uuid
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import TextIO

WORLD_SESSION_SCHEMA_VERSION = 1


def acquire_world_session_lock(path: Path) -> TextIO:
    """Hold an exclusive per-partition lock for the lifetime of the launch."""
    lock_path = path.with_name(f".{path.name}.lock")
    stream = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        stream.close()
        raise RuntimeError(
            "another Nano-C-SLAM Gazebo launch already owns this partition; "
            "stop it or choose a different GZ_PARTITION"
        ) from error
    return stream


def session_file_for_partition(partition: str) -> Path:
    """Return a stable temporary manifest path without exposing partition text."""
    if not partition or any(ord(character) < 32 for character in partition):
        raise ValueError("Gazebo partition must be a non-empty printable value")
    key = hashlib.sha256(partition.encode("utf-8")).hexdigest()[:24]
    return Path(tempfile.gettempdir()) / f"nano-c-slam-world-{key}.json"


def publish_world_session(path: Path, *, scene: str) -> str:
    """Atomically publish a new launch identity before starting Gazebo."""
    if scene not in ("default", "nearfield-mapping"):
        raise ValueError("world session scene is unsupported")
    session_id = str(uuid.uuid4())
    payload = {
        "schema_version": WORLD_SESSION_SCHEMA_VERSION,
        "session_id": session_id,
        "scene": scene,
        "launch_pid": os.getpid(),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(f".{path.name}.{session_id}.tmp")
    try:
        with temporary_path.open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.replace(path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return session_id


def world_process_role(argv: Sequence[str]) -> str | None:
    """Classify only known Gazebo server and ROS bridge process commands."""
    executable_names = {Path(argument).name for argument in argv}
    if argv and Path(argv[0]).name == "gz" and "sim" in argv:
        if "-s" in argv or "--server" in argv:
            return "gazebo-server"
        return None
    if "parameter_bridge" in executable_names:
        return "ros-gazebo-bridge"
    if "sensor_bridge" in executable_names:
        return "nano-sensor-bridge"
    return None


def record_world_process(
    path: Path, *, role: str, pid: int, argv: Sequence[str]
) -> None:
    """Atomically add an observed launch-owned process to the session manifest."""
    if role not in {
        "gazebo-server",
        "ros-gazebo-bridge",
        "nano-sensor-bridge",
    }:
        raise ValueError("unsupported world-session process role")
    if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
        raise ValueError("world-session process PID must be positive")
    command = list(argv)
    if not command or any(not isinstance(argument, str) for argument in command):
        raise ValueError("world-session process command must contain strings")

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError) as error:
        raise RuntimeError(
            "cannot record process without a valid world session"
        ) from error
    if (
        not isinstance(payload, dict)
        or payload.get("schema_version") != WORLD_SESSION_SCHEMA_VERSION
        or not isinstance(payload.get("session_id"), str)
        or not isinstance(payload.get("scene"), str)
    ):
        raise RuntimeError("cannot record process in an invalid world session")
    processes = payload.setdefault("processes", [])
    if not isinstance(processes, list):
        raise RuntimeError("world session process list is malformed")
    for existing in processes:
        if isinstance(existing, dict) and existing.get("pid") == pid:
            if existing.get("role") == role and existing.get("argv") == command:
                return
            raise RuntimeError("world-session PID was observed with conflicting data")

    executable = Path(command[0])
    executable_path: str | None = None
    if executable.is_absolute() and executable.is_file():
        executable_path = str(executable.resolve())
    else:
        located = shutil.which(command[0])
        if located is not None:
            executable_path = str(Path(located).resolve())
    processes.append(
        {
            "role": role,
            "pid": pid,
            "launch_pid": os.getpid(),
            "argv": command,
            "executable_path": executable_path,
            "observed_at_utc": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        }
    )

    temporary_path = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary_path.open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, allow_nan=False, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.replace(path)
    finally:
        temporary_path.unlink(missing_ok=True)


def record_world_process_event(
    path: Path, *, pid: int, argv: Sequence[str]
) -> str | None:
    """Filter a launch process event and record it when its role is known."""
    role = world_process_role(argv)
    if role is None:
        return None
    record_world_process(path, role=role, pid=pid, argv=argv)
    return role


def claim_world_session(path: Path, *, scene: str, run_id: str) -> str:
    """Claim the current launch exactly once, before mission children start."""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise RuntimeError(
            "no Nano-C-SLAM Gazebo launch session exists; start the world with "
            "scripts/run_gazebo_macos.sh and retry"
        ) from error
    if (
        not isinstance(payload, dict)
        or payload.get("schema_version") != WORLD_SESSION_SCHEMA_VERSION
        or payload.get("scene") != scene
        or not isinstance(payload.get("session_id"), str)
    ):
        raise RuntimeError(
            "Gazebo launch session is invalid or belongs to another scene"
        )
    session_id = payload["session_id"]
    try:
        if str(uuid.UUID(session_id)) != session_id:
            raise ValueError("non-canonical session ID")
    except ValueError as error:
        raise RuntimeError("Gazebo launch session has an invalid identity") from error
    claim_path = path.with_name(f".{path.name}.{session_id}.claimed")
    try:
        with claim_path.open("x", encoding="utf-8") as stream:
            stream.write(f"{run_id}\n")
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as error:
        raise RuntimeError(
            "this Gazebo launch already has a mapping-run claim; restart the "
            "near-field Gazebo world before another mapping run"
        ) from error
    return session_id
