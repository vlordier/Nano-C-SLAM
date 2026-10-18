"""Capture one argv process and the exact outputs/environment selected for evidence."""

from __future__ import annotations

import hashlib
import os
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

from scripts.validation_provenance import ExecutionIdentity


@dataclass(frozen=True)
class ProcessCapture:
    execution: ExecutionIdentity
    stdout: bytes
    stderr: bytes
    timed_out: bool


def _sha256(data: bytes | None) -> str | None:
    return None if data is None else hashlib.sha256(data).hexdigest()


def _utc_now() -> str:
    return datetime.now(UTC).isoformat().replace("+00:00", "Z")


def capture_process(
    command: tuple[str, ...],
    *,
    environment_allowlist: tuple[str, ...],
    environment_overrides: Mapping[str, str] | None = None,
    cwd: Path | None = None,
    timeout_seconds: float | None = None,
) -> ProcessCapture:
    """Run argv without a shell and hash stdout/stderr independently."""
    if not command or any(not arg or "\0" in arg for arg in command):
        raise ValueError("command must contain nonempty, NUL-free arguments")
    if len(set(environment_allowlist)) != len(environment_allowlist):
        raise ValueError("environment allowlist contains duplicate keys")
    if any(not key or "=" in key or "\0" in key for key in environment_allowlist):
        raise ValueError("environment allowlist contains an invalid key")
    overrides = dict(environment_overrides or {})
    if set(overrides) - set(environment_allowlist):
        raise ValueError("environment overrides must be explicitly allowlisted")
    if any("\0" in value for value in overrides.values()):
        raise ValueError("environment override contains a NUL byte")

    environment = {
        key: os.environ[key] for key in environment_allowlist if key in os.environ
    }
    environment.update(overrides)
    recorded_environment = tuple(
        (key, environment.get(key)) for key in sorted(set(environment_allowlist))
    )
    started_at = _utc_now()
    timed_out = False
    try:
        completed = subprocess.run(  # noqa: S603 -- argv is passed directly, without a shell
            command,
            check=False,
            cwd=cwd,
            env=environment,
            capture_output=True,
            timeout=timeout_seconds,
        )
        stdout = completed.stdout
        stderr = completed.stderr
        exit_code: int | None = completed.returncode
    except subprocess.TimeoutExpired as error:
        timed_out = True
        stdout = error.stdout
        stderr = error.stderr
        exit_code = None
    finished_at = _utc_now()
    execution = ExecutionIdentity(
        command=command,
        environment=recorded_environment,
        started_at_utc=started_at,
        finished_at_utc=finished_at,
        exit_code=exit_code,
        stdout_sha256=_sha256(stdout),
        stdout_bytes=len(stdout) if stdout is not None else None,
        stderr_sha256=_sha256(stderr),
        stderr_bytes=len(stderr) if stderr is not None else None,
    )
    return ProcessCapture(
        execution=execution,
        stdout=stdout or b"",
        stderr=stderr or b"",
        timed_out=timed_out,
    )
