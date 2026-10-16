#!/usr/bin/env python3
"""Check that a running native Gazebo SITL publishes usable ROS telemetry."""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from collections.abc import Sequence
from contextlib import suppress


def _read_topic(topic: str, timeout_s: float) -> str:
    """Read one ROS message and report command failures clearly."""
    if not topic.startswith("/") or any(char.isspace() for char in topic):
        raise ValueError("topic must be an absolute ROS name without whitespace")
    ros2 = shutil.which("ros2")
    if ros2 is None:
        raise FileNotFoundError("ros2 was not found; source the ROS 2 setup first")
    result = subprocess.run(  # noqa: S603 -- executable is resolved and args are validated
        [ros2, "topic", "echo", "--no-daemon", "--once", topic],
        capture_output=True,
        check=False,
        text=True,
        timeout=timeout_s,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or f"ros2 topic echo failed: {topic}")
    return result.stdout


def _array_values(message: str) -> list[float]:
    """Parse the simple `data: [...]` representation emitted by ros2 topic echo."""
    lines = message.splitlines()
    for index, line in enumerate(lines):
        key, separator, value = line.strip().partition(":")
        if key == "data" and separator:
            values = value.strip().strip("[]")
            if not values:
                values = ",".join(
                    item.strip()[2:]
                    for item in lines[index + 1 :]
                    if item.strip().startswith("- ")
                )
            if not values:
                return []
            try:
                return [float(item.strip()) for item in values.split(",")]
            except ValueError as error:
                raise RuntimeError(f"invalid numeric data array: {value}") from error
    raise RuntimeError("ROS message did not contain a data array")


def _odometry_timestamp_ns(message: str) -> int:
    """Parse the ROS header timestamp from an odometry YAML message."""
    stamp = re.search(
        r"(?m)^\s*stamp:\s*\n\s*sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)",
        message,
    )
    if stamp is None:
        raise RuntimeError("odometry message did not contain a valid header timestamp")
    return int(stamp.group(1)) * 1_000_000_000 + int(stamp.group(2))


def _require_odometry_progress(before_ns: int, after_ns: int) -> None:
    """Reject a mission check if simulation time stopped advancing."""
    if after_ns <= before_ns:
        raise RuntimeError(
            "odometry timestamp did not advance during the mission: "
            f"{before_ns} -> {after_ns} ns"
        )


def check_telemetry(timeout_s: float = 8.0, drone: str = "cf1") -> int:
    """Require odometry, four finite lidar values, and pairwise range telemetry."""
    odometry = _read_topic(f"/{drone}/odom", timeout_s)
    if "position:" not in odometry or "z:" not in odometry:
        raise RuntimeError(f"/{drone}/odom did not contain a position")
    timestamp_ns = _odometry_timestamp_ns(odometry)

    ranges = _array_values(
        _read_topic(f"/nano_c_slam/sim/{drone}/ranges_mm", timeout_s)
    )
    if len(ranges) != 4 or any(not math.isfinite(value) for value in ranges):
        raise RuntimeError(f"lidar must contain four finite values, got {ranges}")
    if any(value < -1 or value > 3490 for value in ranges):
        raise RuntimeError(f"lidar values outside expected millimeter bounds: {ranges}")

    pairwise = _array_values(
        _read_topic("/nano_c_slam/sim/uwb_pairwise_range_m", timeout_s)
    )
    if not pairwise or any(not math.isfinite(value) or value < 0 for value in pairwise):
        raise RuntimeError(
            f"pairwise ranges must be finite and nonnegative: {pairwise}"
        )
    print(
        f"Gazebo telemetry OK: /{drone}/odom, lidar={ranges} mm, "
        f"{len(pairwise)} pairwise ranges (m)"
    )
    return timestamp_ns


def run_mission(timeout_s: float = 45.0) -> None:
    """Run one takeoff-hover-land cycle and require a low-altitude completion."""
    ros2 = shutil.which("ros2")
    if ros2 is None:
        raise FileNotFoundError("ros2 was not found; source the ROS 2 setup first")
    process = subprocess.Popen(  # noqa: S603 -- resolved executable and fixed args
        [
            ros2,
            "run",
            "nano_c_slam_gazebo",
            "sim_mission",
            "--ros-args",
            "-p",
            "use_sim_time:=True",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        output, _ = process.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired as error:
        _signal_process_group(process.pid, signal.SIGTERM)
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            _signal_process_group(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
        raise RuntimeError(
            f"mission process timed out after {timeout_s:.1f} seconds\n{output.strip()}"
        ) from error
    final_altitude_m = _mission_landing_altitude(process.returncode, output)
    print(f"Gazebo mission OK: takeoff, hover, land at {final_altitude_m:.3f} m")


def _signal_process_group(process_id: int, signal_number: int) -> None:
    """Stop the ROS CLI and its child node together, tolerating an exit race."""
    with suppress(ProcessLookupError):
        os.killpg(process_id, signal_number)


def _mission_landing_altitude(returncode: int, output: str) -> float:
    """Require all mission phases and a successful landing near ground level."""
    required_phases = ("takeoff", "hover", "land")
    phases = re.findall(r"mission phase: (takeoff|hover|land)\b", output)
    if returncode or phases != list(required_phases):
        detail = f"mission exited {returncode}"
        if phases != list(required_phases):
            detail += f" with phases {phases}; expected {list(required_phases)}"
        raise RuntimeError(f"{detail}\n{output.strip()}")
    completion = re.search(
        r"completed takeoff, hover, and landing at altitude (-?\d+(?:\.\d+)?) m",
        output,
    )
    if completion is None:
        raise RuntimeError(
            f"mission did not report successful completion\n{output.strip()}"
        )
    final_altitude_m = float(completion.group(1))
    if abs(final_altitude_m) > 0.15:
        raise RuntimeError(
            f"mission completed outside the ground threshold: {final_altitude_m:.3f} m"
        )
    return final_altitude_m


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", default="cf1", help="Crazyflie name (default: cf1)")
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="overall telemetry timeout, in seconds",
    )
    parser.add_argument(
        "--mission",
        action="store_true",
        help="also run a complete takeoff-hover-land mission",
    )
    options = parser.parse_args(args)
    if not options.drone.startswith("cf") or any(
        char.isspace() for char in options.drone
    ):
        parser.error("--drone must be a Crazyflie name such as cf1")
    if options.timeout <= 0 or not math.isfinite(options.timeout):
        parser.error("--timeout must be finite and positive")
    try:
        before_ns = _check_telemetry_until(options.timeout, options.drone)
        if options.mission:
            run_mission(timeout_s=max(45.0, options.timeout))
            after_ns = _check_telemetry_until(options.timeout, options.drone)
            _require_odometry_progress(before_ns, after_ns)
            print("Gazebo odometry advanced across the mission")
        return 0
    except (RuntimeError, subprocess.TimeoutExpired, FileNotFoundError) as error:
        print(f"Gazebo check failed: {error}", file=sys.stderr)
        return 1


def _check_telemetry_until(timeout_s: float, drone: str) -> int:
    """Retry telemetry sampling until its wall-clock budget expires."""
    deadline = time.monotonic() + timeout_s
    last_error: RuntimeError | subprocess.TimeoutExpired | FileNotFoundError | None = (
        None
    )
    while time.monotonic() < deadline:
        try:
            return check_telemetry(
                timeout_s=max(0.1, deadline - time.monotonic()), drone=drone
            )
        except (RuntimeError, subprocess.TimeoutExpired, FileNotFoundError) as error:
            last_error = error
            time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
    raise RuntimeError(f"telemetry check failed: {last_error}")


if __name__ == "__main__":
    raise SystemExit(main())
