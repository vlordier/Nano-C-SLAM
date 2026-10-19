#!/usr/bin/env python3
"""Passively verify fresh, correctly namespaced telemetry for cf1/cf3/cf4."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import subprocess
import sys
import time
from collections.abc import Sequence
from dataclasses import asdict
from pathlib import Path
from typing import Any

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.swarm_telemetry_model import (
    StampSpan,
    analyze_pairwise_snapshot,
    pairwise_source_stamps_advanced,
    stamp_ns,
    validate_stamped_ranges,
)

_CLOCK_TYPE = "rosgraph_msgs/msg/Clock"
_ODOMETRY_TYPE = "nav_msgs/msg/Odometry"
_RANGES_TYPE = "nano_c_slam_interfaces/msg/StampedRanges"
_PAIRWISE_TYPE = "nano_c_slam_interfaces/msg/StampedPairwiseRanges"


def validate_passive_command_graph(
    node_names: Sequence[str],
    endpoints: dict[str, dict[str, list[str]]],
) -> dict[str, object]:
    """Reject any mission node or ROS publisher able to command a drone."""
    mission_nodes = [
        name
        for name in node_names
        if "sim_mission" in name or name.endswith("_mission")
    ]
    if mission_nodes:
        raise RuntimeError(
            f"passive telemetry graph contains mission nodes: {mission_nodes}"
        )
    missing = [
        f"/{drone}/{topic}"
        for drone in SUPPORTED_DRONE_NAMES
        for topic in ("cmd_vel", "enable")
        if f"/{drone}/{topic}" not in endpoints
    ]
    if missing:
        raise RuntimeError(f"passive telemetry graph omitted control topics: {missing}")
    publishers = {
        topic: graph["publishers"]
        for topic, graph in endpoints.items()
        if graph["publishers"]
    }
    if publishers:
        raise RuntimeError(
            f"passive telemetry graph has flight-control publishers: {publishers}"
        )
    return {
        "mission_nodes": mission_nodes,
        "control_topics": endpoints,
        "no_control_publishers": True,
    }


def verify(timeout_s: float, *, require_mission_commands: bool) -> dict[str, object]:
    """Subscribe only; never publish flight or simulator-control commands."""
    rclpy = importlib.import_module("rclpy")
    node_type = importlib.import_module("rclpy.node").Node
    odometry_type = importlib.import_module("nav_msgs.msg").Odometry
    ranges_type = importlib.import_module("nano_c_slam_interfaces.msg").StampedRanges
    pairwise_type = importlib.import_module(
        "nano_c_slam_interfaces.msg"
    ).StampedPairwiseRanges
    clock_type = importlib.import_module("rosgraph_msgs.msg").Clock
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
            self.odom_first_stamp_ns: dict[str, int] = {}
            self.range_sequences: dict[str, int] = {}
            self.range_stamps_ns: dict[str, int] = {}
            self.range_first_stamp_ns: dict[str, int] = {}
            self.range_first_sequence: dict[str, int] = {}
            self.range_counts = dict.fromkeys(SUPPORTED_DRONE_NAMES, 0)
            self.clock_progress = StampSpan()
            self.clock_errors: list[str] = []
            self.pairwise_first_stamp_ns: int | None = None
            self.pairwise_last_stamp_ns: int | None = None
            self.pairwise_observed_stamp_ns: int | None = None
            self.pairwise_source_stamps_ns: tuple[int, ...] | None = None
            self.pairwise_first_sequence: int | None = None
            self.pairwise_last_sequence: int | None = None
            self.pairwise_count = 0
            self.pairwise_source_age_min_ns: int | None = None
            self.pairwise_source_age_max_ns: int | None = None
            self.pairwise_source_skew_max_ns = 0
            self.pairwise_snapshots: list[dict[str, object]] = []
            self.pending_pairwise: list[Any] = []
            self.pairwise_errors: list[str] = []
            self.range_errors: list[str] = []
            self.cmd_vel: dict[str, dict[str, list[str]]] = {}
            self.create_subscription(clock_type, "/clock", self._on_sim_clock, 10)
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

        def _on_sim_clock(self, message: Any) -> None:  # noqa: ANN401
            try:
                current_stamp_ns = stamp_ns(message.clock)
                self.clock_progress.observe(current_stamp_ns)
            except (AttributeError, TypeError, ValueError, OverflowError) as error:
                self.clock_errors.append(f"invalid /clock message: {error}")

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
            self.odom_first_stamp_ns.setdefault(drone, stamp_ns)
            self.last_odom_stamp_ns[drone] = stamp_ns
            while len(self.odometry[drone]) > 128:
                del self.odometry[drone][min(self.odometry[drone])]
            self.odom_counts[drone] += 1
            pending, self.pending_pairwise = self.pending_pairwise, []
            for pairwise in pending:
                self._validate_pairwise(pairwise)

        def _ranges(self, message: Any, drone: str) -> None:  # noqa: ANN401
            try:
                source_stamp_ns, sequence, _ = validate_stamped_ranges(
                    message,
                    expected_drone=drone,
                    now_ns=self.get_clock().now().nanoseconds,
                )
                previous = self.range_sequences.get(drone)
                if previous is not None and sequence <= previous:
                    raise ValueError(f"{drone} range sequence did not advance")
                previous_stamp = self.range_stamps_ns.get(drone)
                if previous_stamp is not None and source_stamp_ns <= previous_stamp:
                    raise ValueError(f"{drone} range timestamp did not advance")
            except ValueError as error:
                transient_clock_error = "stale" in str(error) or "future" in str(error)
                if self.range_counts[drone] > 0 or not transient_clock_error:
                    self.range_errors.append(str(error))
                return
            self.range_sequences[drone] = sequence
            self.range_stamps_ns[drone] = source_stamp_ns
            self.range_first_stamp_ns.setdefault(drone, source_stamp_ns)
            self.range_first_sequence.setdefault(drone, sequence)
            self.range_counts[drone] += 1

        def _pairwise(self, message: Any) -> None:  # noqa: ANN401
            self._validate_pairwise(message)

        def _validate_pairwise(self, message: Any) -> None:  # noqa: ANN401
            try:
                evidence = analyze_pairwise_snapshot(
                    message,
                    now_ns=self.get_clock().now().nanoseconds,
                    odometry=self.odometry,
                )
                if (
                    self.pairwise_last_sequence is not None
                    and evidence.sequence <= self.pairwise_last_sequence
                ):
                    raise ValueError("pairwise sequence did not advance")
                if (
                    self.pairwise_observed_stamp_ns is not None
                    and evidence.publication_stamp_ns < self.pairwise_observed_stamp_ns
                ):
                    raise ValueError("pairwise publication timestamp regressed")
                source_advanced = pairwise_source_stamps_advanced(
                    self.pairwise_source_stamps_ns, evidence.source_stamps_ns
                )
            except (ValueError, AttributeError, TypeError, OverflowError) as error:
                if "exact pairwise source odometry is unavailable" in str(error):
                    self.pending_pairwise.append(message)
                    self.pending_pairwise = self.pending_pairwise[-128:]
                    return
                transient_clock_error = "stale" in str(error) or "future" in str(error)
                if self.pairwise_count or not transient_clock_error:
                    self.pairwise_errors.append(str(error))
                return
            self.pairwise_last_sequence = evidence.sequence
            self.pairwise_observed_stamp_ns = evidence.publication_stamp_ns
            self.pairwise_source_stamps_ns = evidence.source_stamps_ns
            if not source_advanced:
                return
            if self.pairwise_first_stamp_ns is None:
                self.pairwise_first_stamp_ns = evidence.publication_stamp_ns
                self.pairwise_first_sequence = evidence.sequence
            self.pairwise_last_stamp_ns = evidence.publication_stamp_ns
            self.pairwise_count += 1
            source_age_min = min(evidence.source_age_ns)
            source_age_max = max(evidence.source_age_ns)
            if self.pairwise_source_age_min_ns is None:
                self.pairwise_source_age_min_ns = source_age_min
                self.pairwise_source_age_max_ns = source_age_max
            else:
                self.pairwise_source_age_min_ns = min(
                    self.pairwise_source_age_min_ns, source_age_min
                )
                self.pairwise_source_age_max_ns = max(
                    self.pairwise_source_age_max_ns or source_age_max, source_age_max
                )
            self.pairwise_source_skew_max_ns = max(
                self.pairwise_source_skew_max_ns, evidence.source_skew_ns
            )
            self.pairwise_snapshots.append(asdict(evidence))
            self.pairwise_snapshots = self.pairwise_snapshots[-3:]

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

        def inspect_passive_command_graph(self) -> dict[str, dict[str, list[str]]]:
            result: dict[str, dict[str, list[str]]] = {}
            for drone in SUPPORTED_DRONE_NAMES:
                for topic in ("cmd_vel", "enable"):
                    full_topic = f"/{drone}/{topic}"
                    result[full_topic] = {
                        "publishers": [
                            item.node_name
                            for item in self.get_publishers_info_by_topic(full_topic)
                        ],
                        "subscribers": [
                            item.node_name
                            for item in self.get_subscriptions_info_by_topic(full_topic)
                        ],
                    }
            return result

    rclpy.init(args=[])
    node = TelemetryProbe()
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
            if (
                all(count >= 3 for count in node.odom_counts.values())
                and all(count >= 3 for count in node.range_counts.values())
                and node.clock_progress.count >= 3
                and node.pairwise_count >= 3
            ):
                break
            if node.clock_errors:
                break
        commands = node.inspect_command_graph() if require_mission_commands else {}
        passive_command_graph: dict[str, object] | None = None
        if not require_mission_commands:
            discovered_nodes = [
                f"{namespace.rstrip('/')}/{name}"
                for name, namespace in node.get_node_names_and_namespaces()
            ]
            passive_command_graph = validate_passive_command_graph(
                discovered_nodes, node.inspect_passive_command_graph()
            )
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
        if node.clock_errors:
            raise RuntimeError(f"simulator clock rejected: {node.clock_errors[-3:]}")
        try:
            node.clock_progress.require_at_least(3, "/clock")
        except ValueError as error:
            raise RuntimeError(str(error)) from error
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
        if node.pairwise_count < 3:
            raise RuntimeError(
                "fewer than three valid fresh pairwise snapshots received; "
                f"cmd_vel endpoints={commands}"
            )
        if require_mission_commands and missing:
            raise RuntimeError(
                f"cmd_vel publisher/subscriber wiring missing for {missing}"
            )
        expected_topics = {
            "/clock": _CLOCK_TYPE,
            **{f"/{drone}/odom": _ODOMETRY_TYPE for drone in SUPPORTED_DRONE_NAMES},
            **{
                f"/nano_c_slam/sim/{drone}/ranges_mm": _RANGES_TYPE
                for drone in SUPPORTED_DRONE_NAMES
            },
            "/nano_c_slam/sim/uwb_pairwise_range_m": _PAIRWISE_TYPE,
        }
        graph_types = dict(node.get_topic_names_and_types())
        topic_types = {topic: graph_types.get(topic, []) for topic in expected_topics}
        mismatched_topics = {
            topic: topic_types[topic]
            for topic, expected_type in expected_topics.items()
            if topic_types[topic] != [expected_type]
        }
        if mismatched_topics:
            raise RuntimeError(
                f"required telemetry topic types differ: {mismatched_topics}"
            )
        result: dict[str, object] = {
            "topic_types": topic_types,
            "clock": node.clock_progress.summary(),
            "odometry": {
                drone: {
                    "count": node.odom_counts[drone],
                    "first_stamp_ns": node.odom_first_stamp_ns.get(drone),
                    "last_stamp_ns": node.last_odom_stamp_ns.get(drone),
                }
                for drone in SUPPORTED_DRONE_NAMES
            },
            "ranges": {
                drone: {
                    "count": node.range_counts[drone],
                    "first_stamp_ns": node.range_first_stamp_ns.get(drone),
                    "last_stamp_ns": node.range_stamps_ns.get(drone),
                    "first_sequence": node.range_first_sequence.get(drone),
                    "last_sequence": node.range_sequences.get(drone),
                }
                for drone in SUPPORTED_DRONE_NAMES
            },
            "pairwise": {
                "count": node.pairwise_count,
                "first_stamp_ns": node.pairwise_first_stamp_ns,
                "last_stamp_ns": node.pairwise_last_stamp_ns,
                "first_sequence": node.pairwise_first_sequence,
                "last_sequence": node.pairwise_last_sequence,
                "source_age_min_ns": node.pairwise_source_age_min_ns,
                "source_age_max_ns": node.pairwise_source_age_max_ns,
                "source_skew_max_ns": node.pairwise_source_skew_max_ns,
                "last_validated_snapshots": node.pairwise_snapshots,
            },
        }
        if require_mission_commands:
            result["cmd_vel_endpoints"] = commands
            result["cmd_vel_missing_endpoints"] = missing
        elif passive_command_graph is not None:
            result["passive_command_graph"] = passive_command_graph
        return result
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
            timestamp_ns = (
                10_050_000_000
                if mode == "paused-clock"
                else 10_000_000_000 + self.tick * 50_000_000
            )
            sec, nanosec = divmod(timestamp_ns, 1_000_000_000)
            if mode != "missing-clock":
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
            pairwise.pair_labels = (
                ["cf1:cf3", "cf1:cf4"]
                if mode == "incomplete-pairs"
                else ["cf1:cf3", "cf1:cf4", "cf3:cf4"]
            )
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


def verify_controlled_fixture(
    timeout_s: float,
    *,
    mode: str,
    require_mission_commands: bool = False,
) -> dict[str, object]:
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
            result = verify(
                timeout_s, require_mission_commands=require_mission_commands
            )
        except RuntimeError as error:
            if mode == "valid":
                if require_mission_commands and "cmd_vel publisher/subscriber" in str(
                    error
                ):
                    return {
                        "negative_fixture_rejected": "missing-command-endpoints",
                        "diagnostic": str(error),
                    }
                raise
            return {"negative_fixture_rejected": mode, "diagnostic": str(error)}
        if mode != "valid":
            raise RuntimeError(f"invalid fixture unexpectedly passed: {mode}")
        if require_mission_commands:
            raise RuntimeError("empty mission command endpoints unexpectedly passed")
        if "cmd_vel_endpoints" in result or "cmd_vel_missing_endpoints" in result:
            raise RuntimeError(
                "passive telemetry result unexpectedly includes cmd_vel data"
            )
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


def verify_with_process_timeout(
    timeout_s: float, *, require_mission_commands: bool
) -> dict[str, object]:
    """Bound even an RMW or graph call that blocks inside the ROS client."""
    command = [
        sys.executable,
        "-m",
        "scripts.verify_gazebo_swarm_ros",
        "--probe-worker",
        "--timeout",
        str(timeout_s),
    ]
    if require_mission_commands:
        command.append("--require-mission-commands")
    try:
        result = subprocess.run(  # noqa: S603 -- fixed owned module and arguments
            command,
            cwd=Path.cwd(),
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout_s + 5.0,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            "ROS client exceeded the bounded probe deadline; an RMW call did not return"
        ) from error
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "ROS telemetry probe failed")
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError("ROS telemetry probe returned invalid JSON") from error
    if not isinstance(value, dict):
        raise RuntimeError("ROS telemetry probe returned a non-object result")
    return value


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0)
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
        choices=(
            "valid",
            "wrong-drone",
            "cross-assignment",
            "missing-clock",
            "paused-clock",
            "incomplete-pairs",
        ),
        default="valid",
        help="controlled publisher case used with --self-test",
    )
    parser.add_argument("--probe-worker", action="store_true", help=argparse.SUPPRESS)
    options = parser.parse_args(args)
    if options.fixture_publisher:
        return _publish_controlled_fixture(options.fixture_mode)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    try:
        if options.probe_worker:
            result = verify(
                options.timeout,
                require_mission_commands=options.require_mission_commands,
            )
            print(json.dumps(result))
            return 0
        result = (
            verify_controlled_fixture(
                options.timeout,
                mode=options.fixture_mode,
                require_mission_commands=options.require_mission_commands,
            )
            if options.self_test
            else verify_with_process_timeout(
                options.timeout,
                require_mission_commands=options.require_mission_commands,
            )
        )
    except (RuntimeError, ImportError, AttributeError) as error:
        print(f"Gazebo swarm telemetry check failed: {error}", file=sys.stderr)
        return 1
    print(f"Gazebo swarm telemetry OK: {json.dumps(result, sort_keys=True)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
