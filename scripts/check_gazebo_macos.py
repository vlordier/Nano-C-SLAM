#!/usr/bin/env python3
"""Check that a running native Gazebo SITL publishes usable ROS telemetry."""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import time
from collections.abc import Sequence


def _read_topic(topic: str, timeout_s: float) -> str:
    """Read one ROS message and report command failures clearly."""
    if not topic.startswith("/") or any(char.isspace() for char in topic):
        raise ValueError("topic must be an absolute ROS name without whitespace")
    ros2 = shutil.which("ros2")
    if ros2 is None:
        raise FileNotFoundError("ros2 was not found; source the ROS 2 setup first")
    result = subprocess.run(  # noqa: S603 -- executable is resolved and args are validated
        [ros2, "topic", "echo", "--once", topic],
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


def check_telemetry(timeout_s: float = 8.0, drone: str = "cf1") -> None:
    """Require odometry, four finite lidar values, and pairwise range telemetry."""
    odometry = _read_topic(f"/{drone}/odom", timeout_s)
    if "position:" not in odometry or "z:" not in odometry:
        raise RuntimeError(f"/{drone}/odom did not contain a position")

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


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", default="cf1", help="Crazyflie name (default: cf1)")
    parser.add_argument(
        "--timeout", type=float, default=8.0, help="timeout per topic read, in seconds"
    )
    options = parser.parse_args(args)
    if not options.drone.startswith("cf") or any(
        char.isspace() for char in options.drone
    ):
        parser.error("--drone must be a Crazyflie name such as cf1")
    if options.timeout <= 0 or not math.isfinite(options.timeout):
        parser.error("--timeout must be finite and positive")
    deadline = time.monotonic() + options.timeout
    last_error: RuntimeError | subprocess.TimeoutExpired | FileNotFoundError | None = (
        None
    )
    while time.monotonic() < deadline:
        try:
            check_telemetry(
                timeout_s=max(0.1, deadline - time.monotonic()), drone=options.drone
            )
            return 0
        except (RuntimeError, subprocess.TimeoutExpired, FileNotFoundError) as error:
            last_error = error
            time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
    print(f"Gazebo telemetry check failed: {last_error}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
