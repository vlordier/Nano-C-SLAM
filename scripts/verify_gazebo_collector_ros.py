#!/usr/bin/env python3
"""Exercise the real stamped-telemetry collector against controlled ROS data.

Run after sourcing ROS 2 Jazzy and a workspace containing the two Nano-C-SLAM
ROS packages. The verifier itself uses only the standard library; publisher and
collector children use the selected ROS-compatible Python interpreter.
"""

from __future__ import annotations

import argparse
import importlib
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_PUBLISHER_MARKER = "--publisher-fixture"


def _publisher(mode: str) -> int:
    """Publish simulated time plus deterministic matching telemetry."""
    rclpy = importlib.import_module("rclpy")
    StampedRanges = importlib.import_module("nano_c_slam_interfaces.msg").StampedRanges
    Odometry = importlib.import_module("nav_msgs.msg").Odometry
    Node = importlib.import_module("rclpy.node").Node
    Clock = importlib.import_module("rosgraph_msgs.msg").Clock

    class FixturePublisher(Node):
        def __init__(self) -> None:
            super().__init__("nano_c_slam_collector_fixture")
            self._tick = 0
            self._clock_pub = self.create_publisher(Clock, "/clock", 10)
            self._ranges = self.create_publisher(
                StampedRanges, "/nano_c_slam/sim/cf1/ranges_mm", 10
            )
            self._odometry = self.create_publisher(Odometry, "/cf1/odom", 10)
            self.create_timer(0.02, self._publish)

        def _publish(self) -> None:
            self._tick += 1
            now_ns = 10_000_000_000 + self._tick * 20_000_000
            range_age_ns = {
                "fresh": 100_000_000,
                "stale": 600_000_000,
                "future": -5_000_000_000,
                "wrong-drone": 100_000_000,
                "out-of-order": 100_000_000,
            }[mode]
            odometry_age_ns = range_age_ns
            range_ns = now_ns - range_age_ns
            odometry_ns = now_ns - odometry_age_ns
            range_sec, range_nanosec = divmod(range_ns, 1_000_000_000)
            odometry_sec, odometry_nanosec = divmod(odometry_ns, 1_000_000_000)

            self._publish_clock(now_ns)

            ranges = StampedRanges()
            ranges.header.stamp.sec = range_sec
            ranges.header.stamp.nanosec = range_nanosec
            ranges.drone_id = "cf2" if mode == "wrong-drone" else "cf1"
            ranges.sequence = (
                1_000_000 - self._tick if mode == "out-of-order" else self._tick
            )
            ranges.ranges_mm = [900, 1000, 1100, 1200]
            self._ranges.publish(ranges)

            odometry = Odometry()
            odometry.header.stamp.sec = odometry_sec
            odometry.header.stamp.nanosec = odometry_nanosec
            odometry.pose.pose.position.x = self._tick * 0.001
            odometry.pose.pose.orientation.w = 1.0
            self._odometry.publish(odometry)

        def _publish_clock(self, now_ns: int) -> None:
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(now_ns, 1_000_000_000)
            self._clock_pub.publish(clock)

    rclpy.init()
    node = FixturePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _stop(process: subprocess.Popen[str]) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def _run_capture(ros_python: Path, mode: str) -> subprocess.CompletedProcess[str]:
    domain = 70 + os.getpid() % 100
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain)
    environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    publisher = subprocess.Popen(  # noqa: S603 -- fixed local ROS test child
        [str(ros_python), str(Path(__file__).resolve()), _PUBLISHER_MARKER, mode],
        cwd=ROOT,
        env=environment,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        time.sleep(0.5)
        if publisher.poll() is not None:
            detail = publisher.stderr.read() if publisher.stderr is not None else ""
            raise RuntimeError(f"ROS fixture publisher exited early: {detail}")
        if mode == "fresh":
            code = (
                "import json; "
                "from scripts.gazebo_mapping_sitl import _capture_samples; "
                "result = _capture_samples('cf1', 60, 20.0); "
                "print('NCS_CAPTURE_OK:' + json.dumps({"
                "'samples': len(result.samples), "
                "'pairs': len(result.pair_provenance), "
                "'rejections': result.rejections, "
                "'max_age_ns': max(max(pair['range_age_ns'], "
                "pair['odometry_age_ns']) for pair in result.pair_provenance), "
                "'max_skew_ns': max(pair['pair_skew_ns'] "
                "for pair in result.pair_provenance)}))"
            )
            return subprocess.run(  # noqa: S603 -- fixed collector fixture code
                [str(ros_python), "-c", code],
                cwd=ROOT,
                env=environment,
                capture_output=True,
                check=False,
                text=True,
                timeout=25,
            )

        binary = ROOT / "cslam-gap9-app/BUILD-host/mapping_replay"
        if not binary.is_file():
            raise FileNotFoundError(
                "build the host replay first: scripts/build_host_mapping_sitl.sh"
            )
        with tempfile.TemporaryDirectory(prefix="ncs-ros-capture-") as directory:
            output_path = Path(directory) / "must-not-exist.json"
            result = subprocess.run(  # noqa: S603 -- repository collector script
                [
                    str(ros_python),
                    str(ROOT / "scripts/gazebo_mapping_sitl.py"),
                    "--reuse-binary",
                    "--binary",
                    str(binary),
                    "--samples",
                    "60",
                    "--timeout",
                    "2",
                    "--output",
                    str(output_path),
                ],
                cwd=ROOT,
                env=environment,
                capture_output=True,
                check=False,
                text=True,
                timeout=10,
            )
            if output_path.exists():
                raise AssertionError("failed capture unexpectedly wrote a report")
            return result
    finally:
        _stop(publisher)


def main() -> int:
    if len(sys.argv) >= 3 and sys.argv[1] == _PUBLISHER_MARKER:
        return _publisher(sys.argv[2])

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ros-python",
        type=Path,
        required=True,
        help="Python interpreter whose ABI matches the sourced ROS installation",
    )
    options = parser.parse_args()
    ros_python = options.ros_python.resolve()
    if not ros_python.is_file():
        parser.error(f"ROS Python interpreter does not exist: {ros_python}")

    fresh = _run_capture(ros_python, "fresh")
    if fresh.returncode != 0 or "NCS_CAPTURE_OK:" not in fresh.stdout:
        raise RuntimeError(
            f"fresh telemetry integration failed:\n{fresh.stdout}\n{fresh.stderr}"
        )
    print(f"fresh collector: {fresh.stdout.strip()}")

    rejection_cases = (
        ("stale", "stale_range="),
        ("future", "future_range="),
        ("wrong-drone", "wrong_drone="),
        ("out-of-order", "out_of_order_range_sequence="),
    )
    for mode, rejection in rejection_cases:
        result = _run_capture(ros_python, mode)
        diagnostic = result.stdout + result.stderr
        if result.returncode == 0:
            raise AssertionError(f"{mode} telemetry unexpectedly passed capture")
        if rejection not in diagnostic:
            raise AssertionError(
                f"{mode} capture omitted {rejection!r} rejection count:\n{diagnostic}"
            )
        if " of 60 samples before timeout" not in diagnostic:
            raise AssertionError(f"{mode} timeout lacked capture counts:\n{diagnostic}")
        print(f"{mode} collector rejected input and timed out: {diagnostic.strip()}")
    return 0


if __name__ == "__main__":
    try:
        if len(sys.argv) >= 3 and sys.argv[1] == _PUBLISHER_MARKER:
            raise SystemExit(_publisher(sys.argv[2]))
        raise SystemExit(main())
    except (
        AssertionError,
        FileNotFoundError,
        RuntimeError,
        subprocess.TimeoutExpired,
    ) as error:
        print(f"Gazebo collector ROS integration failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error
