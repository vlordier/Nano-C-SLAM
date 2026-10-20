from __future__ import annotations

import argparse
import base64
import contextlib
import json
import os
import selectors
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import IO, BinaryIO, TextIO


def measure_lima_clock_offset(
    instance: str, sample_count: int, timeout_s: float
) -> dict[str, object]:
    """Measure a conservative host/guest realtime offset interval."""
    samples: list[dict[str, int]] = []
    guest_clock_command = ["limactl", "shell", instance, "--", "date", "+%s%N"]
    for _ in range(sample_count):
        host_before_unix_ns = time.time_ns()
        host_before_monotonic_ns = time.monotonic_ns()
        result = subprocess.run(  # noqa: S603 -- fixed Lima clock-probe argv
            guest_clock_command,
            check=True,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        host_after_monotonic_ns = time.monotonic_ns()
        host_after_unix_ns = time.time_ns()
        guest_unix_ns = int(result.stdout.strip())
        samples.append(
            {
                "host_unix_before_ns": host_before_unix_ns,
                "host_unix_after_ns": host_after_unix_ns,
                "host_monotonic_before_ns": host_before_monotonic_ns,
                "host_monotonic_after_ns": host_after_monotonic_ns,
                "guest_unix_ns": guest_unix_ns,
                "host_monotonic_round_trip_ns": (
                    host_after_monotonic_ns - host_before_monotonic_ns
                ),
                "guest_minus_host_min_ns": guest_unix_ns - host_after_unix_ns,
                "guest_minus_host_max_ns": guest_unix_ns - host_before_unix_ns,
            }
        )

    best = min(samples, key=lambda sample: sample["host_monotonic_round_trip_ns"])
    offset_min_ns = best["guest_minus_host_min_ns"]
    offset_max_ns = best["guest_minus_host_max_ns"]
    return {
        "lima_instance": instance,
        "guest_clock_command": guest_clock_command,
        "sample_count": sample_count,
        "best_sample": best,
        "guest_minus_host_offset_midpoint_ns": (offset_min_ns + offset_max_ns) // 2,
        "guest_minus_host_offset_uncertainty_ns": (offset_max_ns - offset_min_ns + 1)
        // 2,
        "samples": samples,
    }


def _write_event(
    output: TextIO,
    *,
    sequence: int,
    stream: str,
    payload: bytes,
    capture_unix_ns: int,
    capture_monotonic_ns: int,
    terminated_by_newline: bool,
) -> None:
    event = {
        "sequence": sequence,
        "stream": stream,
        "capture_unix_time_ns": capture_unix_ns,
        "capture_monotonic_time_ns": capture_monotonic_ns,
        "terminated_by_newline": terminated_by_newline,
        "payload_base64": base64.b64encode(payload).decode("ascii"),
        "text": payload.decode("utf-8", errors="backslashreplace"),
    }
    output.write(json.dumps(event, ensure_ascii=True, separators=(",", ":")) + "\n")
    output.flush()


def capture_process(
    command: list[str],
    output_dir: Path,
    timeout_s: float,
    terminate_grace_s: float = 2.0,
) -> int:
    """Capture raw stdout/stderr and timestamp every complete line from a child."""
    if not command or timeout_s <= 0 or terminate_grace_s <= 0:
        raise ValueError("command and positive timeouts are required")
    output_dir.mkdir(parents=True, exist_ok=False)
    raw_outputs: dict[str, BinaryIO] = {
        "stdout": (output_dir / "stdout.raw").open("xb"),
        "stderr": (output_dir / "stderr.raw").open("xb"),
    }
    buffers = {"stdout": bytearray(), "stderr": bytearray()}
    captured_bytes = {"stdout": 0, "stderr": 0}
    event_bytes = {"stdout": 0, "stderr": 0}
    event_counts = {"stdout": 0, "stderr": 0}
    sequence = 0
    timed_out = False
    capture_started_unix_ns = time.time_ns()
    termination_deadline: float | None = None
    start_monotonic = time.monotonic()
    deadline = start_monotonic + timeout_s
    with (output_dir / "events.jsonl").open("x", encoding="utf-8") as events:
        try:
            process = subprocess.Popen(  # noqa: S603 -- explicit argv, shell disabled
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
                start_new_session=True,
            )
        except OSError:
            for output in raw_outputs.values():
                output.close()
            raise

        if process.stdout is None or process.stderr is None:
            raise RuntimeError("captured child pipes were not created")
        selector = selectors.DefaultSelector()
        streams: dict[str, IO[bytes]] = {
            "stdout": process.stdout,
            "stderr": process.stderr,
        }
        for name, pipe in streams.items():
            os.set_blocking(pipe.fileno(), False)
            selector.register(pipe, selectors.EVENT_READ, data=name)

        try:
            while selector.get_map() or process.poll() is None:
                now = time.monotonic()
                if now >= deadline and not timed_out:
                    timed_out = True
                    termination_deadline = now + terminate_grace_s
                    with contextlib.suppress(ProcessLookupError):
                        os.killpg(process.pid, signal.SIGTERM)
                elif termination_deadline is not None and now >= termination_deadline:
                    with contextlib.suppress(ProcessLookupError):
                        os.killpg(process.pid, signal.SIGKILL)
                    termination_deadline = None

                for key, _ in selector.select(timeout=0.05):
                    stream = str(key.data)
                    pipe = streams[stream]
                    chunk = os.read(pipe.fileno(), 4096)
                    if not chunk:
                        remainder = bytes(buffers[stream])
                        if remainder:
                            _write_event(
                                events,
                                sequence=sequence,
                                stream=stream,
                                payload=remainder,
                                capture_unix_ns=time.time_ns(),
                                capture_monotonic_ns=time.monotonic_ns(),
                                terminated_by_newline=False,
                            )
                            event_bytes[stream] += len(remainder)
                            event_counts[stream] += 1
                            sequence += 1
                        selector.unregister(pipe)
                        pipe.close()
                        continue

                    raw_outputs[stream].write(chunk)
                    raw_outputs[stream].flush()
                    captured_bytes[stream] += len(chunk)
                    capture_unix_ns = time.time_ns()
                    capture_monotonic_ns = time.monotonic_ns()
                    buffer = buffers[stream]
                    buffer.extend(chunk)
                    while (newline := buffer.find(b"\n")) >= 0:
                        line = bytes(buffer[: newline + 1])
                        del buffer[: newline + 1]
                        _write_event(
                            events,
                            sequence=sequence,
                            stream=stream,
                            payload=line,
                            capture_unix_ns=capture_unix_ns,
                            capture_monotonic_ns=capture_monotonic_ns,
                            terminated_by_newline=True,
                        )
                        event_bytes[stream] += len(line)
                        event_counts[stream] += 1
                        sequence += 1
        finally:
            selector.close()
            return_code = process.wait()
            for output in raw_outputs.values():
                output.close()

    manifest = {
        "command": command,
        "return_code": return_code,
        "timed_out": timed_out,
        "capture_started_unix_time_ns": capture_started_unix_ns,
        "capture_finished_unix_time_ns": time.time_ns(),
        "bytes_captured_by_stream": captured_bytes,
        "bytes_in_events_by_stream": event_bytes,
        "bytes_missing_from_events_by_stream": {
            stream: captured_bytes[stream] - event_bytes[stream]
            for stream in captured_bytes
        },
        "event_counts_by_stream": event_counts,
    }
    (output_dir / "process.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return 124 if timed_out else return_code


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture child stdout/stderr byte-for-byte and write arrival-timestamped "
            "JSONL events."
        )
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, required=True)
    parser.add_argument("--lima-instance", default="ncs-sitl")
    parser.add_argument("--clock-probe-count", type=int, default=5)
    parser.add_argument("--clock-probe-timeout-s", type=float, default=5.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.command and args.command[0] == "--":
        args.command = args.command[1:]
    if not args.command:
        parser.error("provide the child command after --")
    if args.clock_probe_count < 1:
        parser.error("clock-probe-count must be positive")
    if args.timeout_s <= 0 or args.clock_probe_timeout_s <= 0:
        parser.error("timeouts must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        args.output_dir.mkdir(parents=True, exist_ok=False)
        clock_offset = measure_lima_clock_offset(
            args.lima_instance, args.clock_probe_count, args.clock_probe_timeout_s
        )
        (args.output_dir / "clock-offset.json").write_text(
            json.dumps(clock_offset, indent=2) + "\n", encoding="utf-8"
        )
        process_capture_dir = args.output_dir / "process"
        return capture_process(args.command, process_capture_dir, args.timeout_s)
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        print(f"capture failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
