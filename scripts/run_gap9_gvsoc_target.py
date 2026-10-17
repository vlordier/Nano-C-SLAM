"""Run a GVSOC command with a hard timeout and retained combined output."""

from __future__ import annotations

import argparse
import contextlib
import math
import os
import signal
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--cwd", type=Path, default=Path.cwd())
    parser.add_argument("--timeout-s", type=float, default=600.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("--timeout-s must be finite and positive")
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a target command is required after --")

    args.log.parent.mkdir(parents=True, exist_ok=True)
    if not args.cwd.is_dir():
        parser.error("--cwd must be an existing directory")
    try:
        with args.log.open("wb") as stream:
            process = subprocess.Popen(  # noqa: S603 -- explicit local SDK command
                command,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                cwd=args.cwd,
            )
            try:
                result_code = process.wait(timeout=args.timeout_s)
            except subprocess.TimeoutExpired:
                _terminate_process_group(process)
                result_code = 124
    except OSError as error:
        print(f"GVSOC command could not start: {error}", file=sys.stderr)
        return 127
    sys.stdout.buffer.write(args.log.read_bytes())
    sys.stdout.buffer.flush()
    if result_code == 124:
        print(
            f"GVSOC command exceeded {args.timeout_s:g} seconds; "
            f"log retained at {args.log}",
            file=sys.stderr,
        )
    return result_code


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
