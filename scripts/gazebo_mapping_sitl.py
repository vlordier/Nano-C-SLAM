#!/usr/bin/env python3
"""Capture Gazebo telemetry and replay it through the Nano-C-SLAM C core."""

# The script is also launched by file path; it adds the checkout root below.
# ruff: noqa: E402

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import platform
import subprocess
import sys
import tempfile
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Protocol, cast

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_model import (
    MappingSample,
    quaternion_yaw,
    summarize_sample_path,
    validate_mapping_result,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mission_model import (
    SUPPORTED_DRONE_NAMES,
)

MINIMUM_SAMPLES = 60
MAXIMUM_SAMPLES = 170


class _Stamp(Protocol):
    sec: int
    nanosec: int


class _Header(Protocol):
    stamp: _Stamp


class _Position(Protocol):
    x: float
    y: float
    z: float


class _Orientation(Protocol):
    x: float
    y: float
    z: float
    w: float


class _Pose(Protocol):
    position: _Position
    orientation: _Orientation


class _PoseWithCovariance(Protocol):
    pose: _Pose


class _OdometryMessage(Protocol):
    header: _Header
    pose: _PoseWithCovariance


class _RangeMessage(Protocol):
    data: list[float]


def _parse_args(args: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", choices=SUPPORTED_DRONE_NAMES, default="cf1")
    parser.add_argument(
        "--samples",
        type=int,
        default=60,
        help="number of fresh 10 Hz samples (minimum 60 for three 20-pose scans)",
    )
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument(
        "--mapping-path",
        action="store_true",
        help="record that sim_mission was launched with mapping_path:=True",
    )
    parser.add_argument(
        "--scene",
        choices=("default", "nearfield-mapping"),
        default="default",
        help="Gazebo scene used for capture; nearfield-mapping uses the 1.4 m walls",
    )
    parser.add_argument(
        "--reuse-binary",
        action="store_true",
        help="use the already-built host executable to avoid compiling during flight",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(tempfile.gettempdir()) / "nano-c-slam-gazebo-map.json",
    )
    parser.add_argument(
        "--binary",
        type=Path,
        default=REPOSITORY_ROOT / "cslam-gap9-app/BUILD-host/mapping_replay",
    )
    options = parser.parse_args(args)
    if not MINIMUM_SAMPLES <= options.samples <= MAXIMUM_SAMPLES:
        parser.error(
            f"--samples must be between {MINIMUM_SAMPLES} and {MAXIMUM_SAMPLES}"
        )
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    return options


def _capture_samples(
    drone: str, sample_limit: int, timeout_s: float
) -> list[MappingSample]:
    """Collect synchronized odometry and range messages from a running Gazebo."""
    rclpy: Any = importlib.import_module("rclpy")
    odometry_module = importlib.import_module("nav_msgs.msg")
    standard_messages = importlib.import_module("std_msgs.msg")
    ros_node_module = importlib.import_module("rclpy.node")
    odometry_message = odometry_module.Odometry
    array_message = standard_messages.Float32MultiArray
    node_base = ros_node_module.Node

    class CaptureNode(node_base):
        def __init__(self) -> None:
            super().__init__("nano_c_slam_mapping_capture")
            self._drone = drone
            self._sample_limit = sample_limit
            self._odometry: _OdometryMessage | None = None
            self._odometry_stamp_ns = -1
            self._ranges_mm: tuple[int, int, int, int] | None = None
            self._range_sequence = 0
            self._last_sample_range_sequence = 0
            self._last_sample_stamp_ns = -1
            self.samples: list[MappingSample] = []
            self.complete = False
            self.create_subscription(
                odometry_message, f"/{drone}/odom", self._on_odometry, 10
            )
            self.create_subscription(
                array_message,
                f"/nano_c_slam/sim/{drone}/ranges_mm",
                self._on_ranges,
                10,
            )
            self.create_timer(0.1, self._sample)

        def _on_odometry(self, message: _OdometryMessage) -> None:
            stamp = message.header.stamp
            self._odometry_stamp_ns = int(stamp.sec) * 1_000_000_000 + int(
                stamp.nanosec
            )
            self._odometry = message

        def _on_ranges(self, message: _RangeMessage) -> None:
            if len(message.data) != 4:
                return
            values = tuple(float(value) for value in message.data)
            if any(
                not math.isfinite(value)
                or value < -1
                or value > 3490
                or not value.is_integer()
                for value in values
            ):
                return
            self._ranges_mm = tuple(int(value) for value in values)
            self._range_sequence += 1

        def _sample(self) -> None:
            if (
                self._odometry is None
                or self._ranges_mm is None
                or self._range_sequence <= self._last_sample_range_sequence
                or self._odometry_stamp_ns <= self._last_sample_stamp_ns
            ):
                return
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._odometry_stamp_ns > 500_000_000:
                return
            message = self._odometry
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            try:
                yaw = quaternion_yaw(
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                )
                sample = MappingSample(
                    sample_id=len(self.samples),
                    timestamp_ms=self._odometry_stamp_ns // 1_000_000,
                    x_m=float(position.x),
                    y_m=float(position.y),
                    yaw_rad=yaw,
                    ranges_mm=self._ranges_mm,
                )
                sample.to_record()
            except ValueError as error:
                self.get_logger().warning(f"Skipping invalid sample: {error}")
                return
            self.samples.append(sample)
            self._last_sample_stamp_ns = self._odometry_stamp_ns
            self._last_sample_range_sequence = self._range_sequence
            if len(self.samples) % 10 == 0:
                self.get_logger().info(
                    f"Captured {len(self.samples)}/{self._sample_limit} samples"
                )
            self.complete = len(self.samples) >= self._sample_limit

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = CaptureNode()
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and not node.complete and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        samples = cast(list[MappingSample], node.samples)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if len(samples) < sample_limit:
        raise RuntimeError(
            f"captured {len(samples)} of {sample_limit} samples before timeout; "
            "check that Gazebo, the sensor bridge, and sim_mission are running"
        )
    return samples


def _build_host_runner(binary: Path) -> None:
    """Compile the repository mapping core with host-only PMSIS stubs."""
    build_script = REPOSITORY_ROOT / "scripts/build_host_mapping_sitl.sh"
    environment = os.environ.copy()
    environment["NANO_C_SLAM_HOST_SITL"] = str(binary)
    subprocess.run(  # noqa: S603 -- fixed build script from this repository
        [str(build_script)],
        check=True,
        cwd=REPOSITORY_ROOT,
        env=environment,
    )


def _run_mapping(binary: Path, samples: list[MappingSample]) -> dict[str, object]:
    records = "\n".join(sample.to_record() for sample in samples) + "\n"
    try:
        result = subprocess.run(  # noqa: S603 -- explicit local executable path
            [str(binary)],
            input=records,
            capture_output=True,
            check=False,
            text=True,
            timeout=30.0,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError("Nano-C-SLAM host mapping replay timed out") from error
    if result.returncode:
        raise RuntimeError(
            result.stderr.strip() or f"mapping replay exited {result.returncode}"
        )
    try:
        mapping = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError("mapping replay did not return valid JSON") from error
    return validate_mapping_result(mapping, minimum_poses=MINIMUM_SAMPLES)


def main(args: list[str] | None = None) -> int:
    options = _parse_args(args)
    binary = options.binary.resolve()
    if not options.reuse_binary:
        _build_host_runner(binary)
    if not binary.is_file():
        raise FileNotFoundError(f"host mapping executable was not built: {binary}")

    samples = _capture_samples(options.drone, options.samples, options.timeout)
    valid_readings = [
        value for sample in samples for value in sample.ranges_mm if value >= 0
    ]
    in_gate_readings = [value for value in valid_readings if value <= 1400]
    mapping = _run_mapping(binary, samples)
    report: dict[str, Any] = {
        "schema_version": 1,
        "captured_at_utc": datetime.now(UTC).isoformat(),
        "evidence_class": "host-execution-of-owned-gap9-mapping-core",
        "claim_boundary": (
            "Gazebo four-ray lidar and simulated odometry replayed through the "
            "Nano-C-SLAM C mapping/ICP/pose-graph modules with CPU PMSIS stubs; "
            "the replay applies the firmware loop-closure helper's 1.4 m scan "
            "cutoff; not GAP9 GVSOC, real 8x8 ToF, UWB, or hardware validation."
        ),
        "drone": options.drone,
        "simulator": "Gazebo Harmonic Crazyswarm2 swarm world",
        "scene": options.scene,
        "host": {"system": platform.system(), "machine": platform.machine()},
        "input_topics": [
            f"/{options.drone}/odom",
            f"/nano_c_slam/sim/{options.drone}/ranges_mm",
        ],
        "input_sample_count": len(samples),
        "input_frequency_hz": 10,
        "mapping_path_enabled": options.mapping_path,
        "input_horizontal_path": summarize_sample_path(samples),
        "input_sim_time_ms": {
            "start": samples[0].timestamp_ms,
            "end": samples[-1].timestamp_ms,
        },
        "input_valid_range_bounds_mm": {
            "minimum": min(valid_readings),
            "maximum": max(valid_readings),
        },
        "mapping_range_limit_mm": 1400,
        "range_accounting": {
            "readings_total": len(samples) * 4,
            "invalid": len(samples) * 4 - len(valid_readings),
            "valid": len(valid_readings),
            "accepted_at_or_below_limit": len(in_gate_readings),
            "rejected_above_limit": len(valid_readings) - len(in_gate_readings),
        },
        "input_samples": [sample.to_record() for sample in samples],
        "tof_projection": "one four-ray reading per direction in central ToF column",
        "mapping": mapping,
    }
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        f"Nano-C-SLAM host mapping OK: {mapping['pose_count']} poses, "
        f"{mapping['scan_count']} scans, {mapping['map_point_count']} points, "
        f"graph={mapping['main_graph_size']}; report: {options.output}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"Gazebo mapping SITL failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error
