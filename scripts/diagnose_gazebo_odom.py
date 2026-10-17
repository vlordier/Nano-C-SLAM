#!/usr/bin/env python3
"""Capture bounded, passive ROS evidence for Gazebo odometry delivery.

Run from a shell with ROS 2 and the target workspace sourced. This probe only
subscribes; it never publishes clock, enable, or flight commands.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
import time
from pathlib import Path
from typing import Protocol


class _Stamp(Protocol):
    sec: int
    nanosec: int


class _Position(Protocol):
    z: float


class _Pose(Protocol):
    position: _Position


class _PoseWithCovariance(Protocol):
    pose: _Pose


class _Header(Protocol):
    stamp: _Stamp


class _OdometryMessage(Protocol):
    header: _Header
    pose: _PoseWithCovariance


class _ClockMessage(Protocol):
    clock: _Stamp


class _QosProfile(Protocol):
    reliability: object
    durability: object
    history: object
    depth: int


class _PublisherEndpoint(Protocol):
    node_name: str
    node_namespace: str
    topic_type: str
    qos_profile: _QosProfile


def _endpoint_record(endpoint: _PublisherEndpoint) -> dict[str, object]:
    qos = endpoint.qos_profile
    return {
        "node_name": endpoint.node_name,
        "node_namespace": endpoint.node_namespace,
        "topic_type": endpoint.topic_type,
        "qos": {
            "reliability": str(qos.reliability),
            "durability": str(qos.durability),
            "history": str(qos.history),
            "depth": qos.depth,
        },
    }


def _stamp_ns(stamp: _Stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def capture(drone: str, duration_s: float) -> dict[str, object]:
    """Subscribe for a bounded wall-clock interval and return evidence."""
    rclpy = importlib.import_module("rclpy")
    node_type = importlib.import_module("rclpy.node").Node
    odometry_type = importlib.import_module("nav_msgs.msg").Odometry
    clock_type = importlib.import_module("rosgraph_msgs.msg").Clock
    string_type = importlib.import_module("std_msgs.msg").String
    bool_type = importlib.import_module("std_msgs.msg").Bool
    twist_type = importlib.import_module("geometry_msgs.msg").Twist

    class ProbeNode(node_type):  # type: ignore[valid-type,misc]
        def __init__(self) -> None:
            super().__init__("nano_c_slam_odom_diagnostic")
            self.odometry_stamps_ns: list[int] = []
            self.odometry_altitudes_m: list[float] = []
            self.clock_stamps_ns: list[int] = []
            self.mission_status: list[str] = []
            self.enable_messages: list[bool] = []
            self.command_linear: list[tuple[float, float, float]] = []
            self.create_subscription(
                odometry_type,
                f"/{drone}/odom",
                self._on_odometry,
                10,
            )
            self.create_subscription(clock_type, "/clock", self._on_clock, 10)
            self.create_subscription(
                string_type,
                f"/{drone}/mission_status",
                lambda message: self.mission_status.append(message.data),
                10,
            )
            self.create_subscription(
                bool_type,
                f"/{drone}/enable",
                lambda message: self.enable_messages.append(bool(message.data)),
                10,
            )
            self.create_subscription(
                twist_type,
                f"/{drone}/cmd_vel",
                lambda message: self.command_linear.append(
                    (
                        float(message.linear.x),
                        float(message.linear.y),
                        float(message.linear.z),
                    )
                ),
                10,
            )

        def _on_odometry(self, message: _OdometryMessage) -> None:
            self.odometry_stamps_ns.append(_stamp_ns(message.header.stamp))
            self.odometry_altitudes_m.append(float(message.pose.pose.position.z))

        def _on_clock(self, message: _ClockMessage) -> None:
            self.clock_stamps_ns.append(_stamp_ns(message.clock))

    rclpy.init(args=[])
    node = ProbeNode()
    started = time.monotonic()
    deadline = started + duration_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            timeout_s = max(0.0, min(0.1, deadline - time.monotonic()))
            rclpy.spin_once(node, timeout_sec=timeout_s)
        elapsed_s = time.monotonic() - started
        odometry_publishers = node.get_publishers_info_by_topic(f"/{drone}/odom")
        return {
            "schema_version": 1,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "gazebo_partition": os.environ.get("GZ_PARTITION"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "duration_requested_s": duration_s,
            "duration_observed_s": elapsed_s,
            "node_names_and_namespaces": [
                {"name": name, "namespace": namespace}
                for name, namespace in node.get_node_names_and_namespaces()
            ],
            "odometry": {
                "topic": f"/{drone}/odom",
                "received_count": len(node.odometry_stamps_ns),
                "first_stamp_ns": node.odometry_stamps_ns[0]
                if node.odometry_stamps_ns
                else None,
                "last_stamp_ns": node.odometry_stamps_ns[-1]
                if node.odometry_stamps_ns
                else None,
                "first_altitude_m": node.odometry_altitudes_m[0]
                if node.odometry_altitudes_m
                else None,
                "last_altitude_m": node.odometry_altitudes_m[-1]
                if node.odometry_altitudes_m
                else None,
                "min_altitude_m": min(node.odometry_altitudes_m, default=None),
                "max_altitude_m": max(node.odometry_altitudes_m, default=None),
                "publishers": [_endpoint_record(item) for item in odometry_publishers],
            },
            "clock": {
                "received_count": len(node.clock_stamps_ns),
                "first_stamp_ns": node.clock_stamps_ns[0]
                if node.clock_stamps_ns
                else None,
                "last_stamp_ns": node.clock_stamps_ns[-1]
                if node.clock_stamps_ns
                else None,
            },
            "mission_status_messages": node.mission_status,
            "enable_messages": node.enable_messages,
            "cmd_vel": {
                "received_count": len(node.command_linear),
                "max_abs_linear_velocity_m_s": max(
                    (
                        abs(value)
                        for command in node.command_linear
                        for value in command
                    ),
                    default=0.0,
                ),
            },
            "publishes_flight_commands": False,
        }
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", default="cf1")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    options = parser.parse_args(args)
    if options.duration <= 0:
        parser.error("--duration must be positive")

    result = capture(options.drone, options.duration)
    payload = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if options.output is None:
        sys.stdout.write(payload)
    else:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(payload, encoding="utf-8")
        print(options.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
