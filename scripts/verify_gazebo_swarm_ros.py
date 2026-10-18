#!/usr/bin/env python3
"""Passively verify fresh, correctly namespaced telemetry for cf1/cf3/cf4."""

from __future__ import annotations

import argparse
import importlib
import math
import os
import subprocess
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.swarm_telemetry_model import (
    validate_pairwise_snapshot,
    validate_stamped_ranges,
)


def verify(timeout_s: float, *, require_mission_commands: bool) -> dict[str, object]:
    """Subscribe only; never publish flight or simulator-control commands."""
    rclpy = importlib.import_module("rclpy")
    node_type = importlib.import_module("rclpy.node").Node
    odometry_type = importlib.import_module("nav_msgs.msg").Odometry
    ranges_type = importlib.import_module("nano_c_slam_interfaces.msg").StampedRanges
    pairwise_type = importlib.import_module(
        "nano_c_slam_interfaces.msg"
    ).StampedPairwiseRanges
    parameter_type = importlib.import_module("rclpy.parameter").Parameter

    class TelemetryProbe(node_type):  # type: ignore[valid-type,misc]
        def __init__(self) -> None:
            super().__init__("nano_c_slam_swarm_telemetry_probe")
            self.set_parameters([parameter_type("use_sim_time", value=True)])
            self.odometry: dict[str, dict[int, tuple[float, float, float]]] = {
                drone: {} for drone in SUPPORTED_DRONE_NAMES
            }
            self.odom_counts = dict.fromkeys(SUPPORTED_DRONE_NAMES, 0)
            self.last_odom_stamp_ns: dict[str, int] = {}
            self.range_sequences: dict[str, int] = {}
            self.range_counts = dict.fromkeys(SUPPORTED_DRONE_NAMES, 0)
            self.pairwise_sequences: list[int] = []
            self.pending_pairwise: list[Any] = []
            self.pairwise_errors: list[str] = []
            self.range_errors: list[str] = []
            self.cmd_vel: dict[str, dict[str, list[str]]] = {}
            for drone in SUPPORTED_DRONE_NAMES:
                self.create_subscription(
                    odometry_type,
                    f"/{drone}/odom",
                    lambda msg, name=drone: self._odom(msg, name),
                    10,
                )
                self.create_subscription(
                    ranges_type,
                    f"/nano_c_slam/sim/{drone}/ranges_mm",
                    lambda msg, name=drone: self._ranges(msg, name),
                    10,
                )
            self.create_subscription(
                pairwise_type,
                "/nano_c_slam/sim/uwb_pairwise_range_m",
                self._pairwise,
                10,
            )

        def _odom(self, message: Any, drone: str) -> None:  # noqa: ANN401
            stamp = message.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            previous_stamp = self.last_odom_stamp_ns.get(drone)
            if stamp_ns <= 0 or (
                previous_stamp is not None and stamp_ns <= previous_stamp
            ):
                self.pairwise_errors.append(
                    f"{drone} odometry timestamp did not advance"
                )
                return
            point = message.pose.pose.position
            values = (float(point.x), float(point.y), float(point.z))
            if not all(map(math.isfinite, values)):
                self.pairwise_errors.append(f"{drone} odometry position is non-finite")
                return
            self.odometry[drone][stamp_ns] = values
            self.last_odom_stamp_ns[drone] = stamp_ns
            while len(self.odometry[drone]) > 128:
                del self.odometry[drone][min(self.odometry[drone])]
            self.odom_counts[drone] += 1
            pending, self.pending_pairwise = self.pending_pairwise, []
            for pairwise in pending:
                self._validate_pairwise(pairwise)

        def _ranges(self, message: Any, drone: str) -> None:  # noqa: ANN401
            try:
                _, sequence, _ = validate_stamped_ranges(
                    message,
                    expected_drone=drone,
                    now_ns=self.get_clock().now().nanoseconds,
                )
                previous = self.range_sequences.get(drone)
                if previous is not None and sequence <= previous:
                    raise ValueError(f"{drone} range sequence did not advance")
            except ValueError as error:
                transient_clock_error = "stale" in str(error) or "future" in str(error)
                if self.range_counts[drone] > 0 or not transient_clock_error:
                    self.range_errors.append(str(error))
                return
            self.range_sequences[drone] = sequence
            self.range_counts[drone] += 1

        def _pairwise(self, message: Any) -> None:  # noqa: ANN401
            self._validate_pairwise(message)

        def _validate_pairwise(self, message: Any) -> None:  # noqa: ANN401
            try:
                sequence, _ = validate_pairwise_snapshot(
                    message,
                    now_ns=self.get_clock().now().nanoseconds,
                    odometry=self.odometry,
                )
                if self.pairwise_sequences and sequence <= self.pairwise_sequences[-1]:
                    raise ValueError("pairwise sequence did not advance")
            except (ValueError, AttributeError, TypeError, OverflowError) as error:
                if "exact pairwise source odometry is unavailable" in str(error):
                    self.pending_pairwise.append(message)
                    self.pending_pairwise = self.pending_pairwise[-128:]
                    return
                transient_clock_error = "stale" in str(error) or "future" in str(error)
                if self.pairwise_sequences or not transient_clock_error:
                    self.pairwise_errors.append(str(error))
                return
            self.pairwise_sequences.append(sequence)

        def inspect_command_graph(self) -> dict[str, dict[str, list[str]]]:
            for drone in SUPPORTED_DRONE_NAMES:
                topic = f"/{drone}/cmd_vel"
                publishers = self.get_publishers_info_by_topic(topic)
                subscribers = self.get_subscriptions_info_by_topic(topic)
                self.cmd_vel[drone] = {
                    "publishers": [item.node_name for item in publishers],
                    "subscribers": [item.node_name for item in subscribers],
                }
            return self.cmd_vel

    rclpy.init(args=[])
    node = TelemetryProbe()
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
            if (
                all(count >= 3 for count in node.odom_counts.values())
                and all(count >= 3 for count in node.range_counts.values())
                and len(node.pairwise_sequences) >= 3
            ):
                break
        commands = node.inspect_command_graph()
        missing = [
            drone
            for drone, endpoints in commands.items()
            if not endpoints["publishers"] or not endpoints["subscribers"]
        ]
        if node.range_errors or node.pairwise_errors:
            raise RuntimeError(
                f"telemetry rejected: ranges={node.range_errors[-3:]}, "
                f"pairwise={node.pairwise_errors[-3:]}"
            )
        if any(count < 3 for count in node.odom_counts.values()):
            raise RuntimeError(
                "odometry did not advance for all drones: "
                f"{node.odom_counts}; cmd_vel endpoints={commands}"
            )
        if any(count < 3 for count in node.range_counts.values()):
            raise RuntimeError(
                f"four-ray ranges did not advance: {node.range_counts}; "
                f"cmd_vel endpoints={commands}"
            )
        if len(node.pairwise_sequences) < 3:
            raise RuntimeError(
                "fewer than three valid fresh pairwise snapshots received; "
                f"cmd_vel endpoints={commands}"
            )
        if require_mission_commands and missing:
            raise RuntimeError(
                f"cmd_vel publisher/subscriber wiring missing for {missing}"
            )
        return {
            "odometry_samples": node.odom_counts,
            "range_samples": node.range_counts,
            "pairwise_sequences": node.pairwise_sequences,
            "cmd_vel_endpoints": commands,
            "cmd_vel_missing_endpoints": missing,
        }
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _publish_controlled_fixture(mode: str) -> int:
    """Publish three deterministic stamped streams for isolated ROS testing."""
    rclpy = importlib.import_module("rclpy")
    odometry_type = importlib.import_module("nav_msgs.msg").Odometry
    interfaces = importlib.import_module("nano_c_slam_interfaces.msg")
    ranges_type = interfaces.StampedRanges
    pairwise_type = interfaces.StampedPairwiseRanges
    clock_type = importlib.import_module("rosgraph_msgs.msg").Clock
    node_type = importlib.import_module("rclpy.node").Node
    time_type = importlib.import_module("builtin_interfaces.msg").Time

    class FixturePublisher(node_type):  # type: ignore[valid-type,misc]
        def __init__(self) -> None:
            super().__init__("nano_c_slam_swarm_telemetry_fixture")
            self.tick = 0
            self.clock = self.create_publisher(clock_type, "/clock", 10)
            self.odometry = {
                drone: self.create_publisher(odometry_type, f"/{drone}/odom", 10)
                for drone in SUPPORTED_DRONE_NAMES
            }
            self.ranges = {
                drone: self.create_publisher(
                    ranges_type, f"/nano_c_slam/sim/{drone}/ranges_mm", 10
                )
                for drone in SUPPORTED_DRONE_NAMES
            }
            self.pairwise = self.create_publisher(
                pairwise_type, "/nano_c_slam/sim/uwb_pairwise_range_m", 10
            )
            self.create_timer(0.05, self.publish_snapshot)

        def publish_snapshot(self) -> None:
            self.tick += 1
            timestamp_ns = 10_000_000_000 + self.tick * 50_000_000
            sec, nanosec = divmod(timestamp_ns, 1_000_000_000)
            clock_message = clock_type()
            clock_message.clock.sec = sec
            clock_message.clock.nanosec = nanosec
            self.clock.publish(clock_message)
            coordinates = {
                "cf1": (0.0, 0.0, 0.0),
                "cf3": (1.0, 0.0, 0.0),
                "cf4": (0.0, 2.0, 0.0),
            }
            for drone in SUPPORTED_DRONE_NAMES:
                odometry = odometry_type()
                odometry.header.stamp.sec = sec
                odometry.header.stamp.nanosec = nanosec
                point = coordinates[drone]
                odometry.pose.pose.position.x = point[0]
                odometry.pose.pose.position.y = point[1]
                odometry.pose.pose.position.z = point[2]
                self.odometry[drone].publish(odometry)
                ranges = ranges_type()
                ranges.header.stamp.sec = sec
                ranges.header.stamp.nanosec = nanosec
                ranges.drone_id = (
                    "cf3" if mode == "wrong-drone" and drone == "cf1" else drone
                )
                ranges.sequence = self.tick
                ranges.ranges_mm = [800, 900, 1000, 1100]
                self.ranges[drone].publish(ranges)
            pairwise = pairwise_type()
            pairwise.header.stamp.sec = sec
            pairwise.header.stamp.nanosec = nanosec
            pairwise.sequence = self.tick
            pairwise.drone_ids = list(SUPPORTED_DRONE_NAMES)
            pairwise.source_stamps = [
                time_type(sec=sec, nanosec=nanosec) for _ in SUPPORTED_DRONE_NAMES
            ]
            pairwise.pair_labels = ["cf1:cf3", "cf1:cf4", "cf3:cf4"]
            pairwise.ranges_m = (
                [2.0, 1.0, math.sqrt(5.0)]
                if mode == "cross-assignment"
                else [1.0, 2.0, math.sqrt(5.0)]
            )
            self.pairwise.publish(pairwise)

    rclpy.init(args=[])
    node = FixturePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def verify_controlled_fixture(timeout_s: float, *, mode: str) -> dict[str, object]:
    """Run the passive verifier against a local fixture on a private ROS domain."""
    old_domain = os.environ.get("ROS_DOMAIN_ID")
    old_discovery = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE")
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    command = [
        sys.executable,
        "-m",
        "scripts.verify_gazebo_swarm_ros",
        "--fixture-publisher",
        "--fixture-mode",
        mode,
    ]
    os.environ.update(
        ROS_DOMAIN_ID=environment["ROS_DOMAIN_ID"],
        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
    )
    publisher = subprocess.Popen(  # noqa: S603 -- fixed local module and arguments
        command,
        cwd=Path.cwd(),
        env=environment,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        time.sleep(0.5)
        if publisher.poll() is not None:
            detail = publisher.stderr.read() if publisher.stderr is not None else ""
            raise RuntimeError(f"controlled ROS publisher exited early: {detail}")
        try:
            result = verify(timeout_s, require_mission_commands=False)
        except RuntimeError as error:
            if mode == "valid":
                raise
            return {"negative_fixture_rejected": mode, "diagnostic": str(error)}
        if mode != "valid":
            raise RuntimeError(f"invalid fixture unexpectedly passed: {mode}")
        return result
    finally:
        if publisher.poll() is None:
            publisher.terminate()
        try:
            publisher.wait(timeout=5)
        except subprocess.TimeoutExpired:
            publisher.kill()
            publisher.wait(timeout=5)
        if old_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = old_domain
        if old_discovery is None:
            os.environ.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
        else:
            os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = old_discovery


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--require-mission-commands",
        action="store_true",
        help="fail unless each /<drone>/cmd_vel topic has a publisher and subscriber",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run against deterministic publishers on a private local ROS domain",
    )
    parser.add_argument(
        "--fixture-publisher", action="store_true", help=argparse.SUPPRESS
    )
    parser.add_argument(
        "--fixture-mode",
        choices=("valid", "wrong-drone", "cross-assignment"),
        default="valid",
        help="controlled publisher case used with --self-test",
    )
    options = parser.parse_args(args)
    if options.fixture_publisher:
        return _publish_controlled_fixture(options.fixture_mode)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    try:
        result = (
            verify_controlled_fixture(options.timeout, mode=options.fixture_mode)
            if options.self_test
            else verify(
                options.timeout,
                require_mission_commands=options.require_mission_commands,
            )
        )
    except (RuntimeError, ImportError, AttributeError) as error:
        print(f"Gazebo swarm telemetry check failed: {error}", file=sys.stderr)
        return 1
    print(f"Gazebo swarm telemetry OK: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
