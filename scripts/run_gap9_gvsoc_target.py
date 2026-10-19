"""Run a GVSOC command with a hard timeout and retained combined output."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from scripts.source_provenance import (
    collect_source_identity,
    source_identity_to_mapping,
    verify_source_identity,
)
from scripts.validation_provenance import SourceIdentity


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--cwd", type=Path, default=Path.cwd())
    parser.add_argument("--timeout-s", type=float, default=600.0)
    parser.add_argument(
        "--completion-marker",
        help="stop the target successfully after this marker appears in its log",
    )
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--source-receipt", type=Path)
    parser.add_argument("--stdout-log", type=Path)
    parser.add_argument("--stderr-log", type=Path)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("--timeout-s must be finite and positive")
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a target command is required after --")
    if (args.source_root is None) != (args.source_receipt is None):
        parser.error("--source-root and --source-receipt must be supplied together")
    if (args.stdout_log is None) != (args.stderr_log is None):
        parser.error("--stdout-log and --stderr-log must be supplied together")
    if args.source_receipt is not None and args.stdout_log is None:
        parser.error("source receipts require separate stdout and stderr logs")

    args.log.parent.mkdir(parents=True, exist_ok=True)
    if not args.cwd.is_dir():
        parser.error("--cwd must be an existing directory")
    try:
        source_before = (
            collect_source_identity(args.source_root)
            if args.source_root is not None
            else None
        )
    except (OSError, ValueError) as error:
        print(f"GVSOC source snapshot failed: {error}", file=sys.stderr)
        return 2
    try:
        started_at = _utc_now()
        if args.stdout_log is not None and args.stderr_log is not None:
            args.stdout_log.parent.mkdir(parents=True, exist_ok=True)
            args.stderr_log.parent.mkdir(parents=True, exist_ok=True)
            with (
                args.stdout_log.open("wb") as stdout_stream,
                args.stderr_log.open("wb") as stderr_stream,
            ):
                process = subprocess.Popen(  # noqa: S603 -- explicit local SDK command
                    command,
                    stdout=stdout_stream,
                    stderr=stderr_stream,
                    start_new_session=True,
                    cwd=args.cwd,
                )
                try:
                    result_code = _wait_for_target(
                        process,
                        args.timeout_s,
                        args.stdout_log,
                        args.completion_marker,
                    )
                except subprocess.TimeoutExpired:
                    _terminate_process_group(process)
                    result_code = 124
            _concatenate_logs(args.log, args.stdout_log, args.stderr_log)
        else:
            with args.log.open("wb") as stream:
                process = subprocess.Popen(  # noqa: S603 -- explicit local SDK command
                    command,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    cwd=args.cwd,
                )
                try:
                    result_code = _wait_for_target(
                        process, args.timeout_s, args.log, args.completion_marker
                    )
                except subprocess.TimeoutExpired:
                    _terminate_process_group(process)
                    result_code = 124
        finished_at = _utc_now()
        if args.source_root is not None and source_before is not None:
            verify_source_identity(args.source_root, source_before)
            if args.source_receipt is None:
                raise ValueError("source receipt path is required with source root")
            _publish_source_receipt(
                args.source_receipt,
                source_before,
                command=command,
                started_at=started_at,
                finished_at=finished_at,
                exit_code=result_code,
                stdout_log=args.stdout_log,
                stderr_log=args.stderr_log,
            )
    except OSError as error:
        print(f"GVSOC command could not start: {error}", file=sys.stderr)
        return 127
    except ValueError as error:
        print(f"GVSOC source changed during execution: {error}", file=sys.stderr)
        return 2
    sys.stdout.buffer.write(args.log.read_bytes())
    sys.stdout.buffer.flush()
    if result_code == 124:
        print(
            f"GVSOC command exceeded {args.timeout_s:g} seconds; "
            f"log retained at {args.log}",
            file=sys.stderr,
        )
    return result_code


def _wait_for_target(
    process: subprocess.Popen[bytes],
    timeout_s: float,
    output_path: Path | None,
    completion_marker: str | None,
) -> int:
    """Wait for process exit or stop a persistent target after its proof marker."""
    deadline = time.monotonic() + timeout_s
    marker_bytes = completion_marker.encode() if completion_marker else None
    while True:
        result_code = process.poll()
        if result_code is not None:
            if (
                marker_bytes is not None
                and output_path is not None
                and marker_bytes not in output_path.read_bytes()
            ):
                return result_code or 3
            return result_code
        if (
            marker_bytes is not None
            and output_path is not None
            and marker_bytes in output_path.read_bytes()
        ):
            _terminate_process_group(process)
            return 0
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0:
            raise subprocess.TimeoutExpired(process.args, timeout_s)
        time.sleep(min(0.05, remaining_s))


def _utc_now() -> str:
    return datetime.now(UTC).isoformat().replace("+00:00", "Z")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _concatenate_logs(output: Path, stdout_log: Path, stderr_log: Path) -> None:
    with output.open("wb") as destination:
        for path in (stdout_log, stderr_log):
            with path.open("rb") as source:
                shutil.copyfileobj(source, destination, length=1024 * 1024)
        destination.flush()
        os.fsync(destination.fileno())


def _publish_source_receipt(
    path: Path,
    identity: SourceIdentity,
    *,
    command: list[str],
    started_at: str,
    finished_at: str,
    exit_code: int,
    stdout_log: Path | None,
    stderr_log: Path | None,
) -> None:
    environment_keys = (
        "PATH",
        "PYTHONPATH",
        "GVSOC_MODULES",
        "DYLD_LIBRARY_PATH",
        "DYLD_FALLBACK_LIBRARY_PATH",
        "GAP9_REAL_GVSOC_LAUNCHER",
        "GAP_SDK_HOME",
        "GAPY_TARGET",
        "GVSOC_TARGETS",
    )
    environment = {key: os.environ.get(key) for key in environment_keys}
    if stdout_log is None or stderr_log is None:
        raise ValueError("source receipt requires independent stdout and stderr logs")
    execution: dict[str, Any] = {
        "command": command,
        "environment": environment,
        "started_at_utc": started_at,
        "finished_at_utc": finished_at,
        "exit_code": exit_code,
        "stdout_sha256": _sha256_file(stdout_log),
        "stdout_bytes": stdout_log.stat().st_size,
        "stderr_sha256": _sha256_file(stderr_log),
        "stderr_bytes": stderr_log.stat().st_size,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(
            {
                "schema_version": 2,
                "source": source_identity_to_mapping(identity),
                "execution": execution,
            },
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        + b"\n"
    )
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _terminate_process_group(process: subprocess.Popen[bytes]) -> None:
    with contextlib.suppress(ProcessLookupError):
        os.killpg(process.pid, signal.SIGTERM)
    with contextlib.suppress(subprocess.TimeoutExpired):
        process.wait(timeout=2.0)
    with contextlib.suppress(ProcessLookupError):
        os.killpg(process.pid, signal.SIGKILL)
    process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
