#!/usr/bin/env python3
"""Inject one invalid odometry sample and observe Gazebo after mission exit.

Run in a shell with ROS 2, Gazebo, Crazyswarm2, and a Nano-C-SLAM overlay
sourced. The probe launches only the single cf1 SimMission child process. It
requires a grounded world before launch and never publishes movement commands.
Its only injected message is one deliberately invalid odometry sample after
the mission reports HOVER.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import os
import shutil
import subprocess
import sys
import time
import uuid
from importlib.util import find_spec
from pathlib import Path
from typing import Protocol, cast

from scripts.runtime_loaded_libraries import capture_session_gazebo_libraries

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MISSION_SOURCE = (
    REPOSITORY_ROOT / "sim/gazebo/nano_c_slam_gazebo/nano_c_slam_gazebo/sim_mission.py"
)
MISSION_STATUS_DRAIN_S = 0.5


class _Stamp(Protocol):
    sec: int
    nanosec: int


class _Vector3(Protocol):
    x: float
    y: float
    z: float


class _Header(Protocol):
    stamp: _Stamp


class _Pose(Protocol):
    position: _Vector3


class _PoseWithCovariance(Protocol):
    pose: _Pose


class _Twist(Protocol):
    linear: _Vector3


class _TwistWithCovariance(Protocol):
    twist: _Twist


class _OdometryMessage(Protocol):
    header: _Header
    pose: _PoseWithCovariance
    twist: _TwistWithCovariance


class _ClockMessage(Protocol):
    clock: _Stamp


class _StringMessage(Protocol):
    data: str


class _BoolMessage(Protocol):
    data: bool


class _TwistMessage(Protocol):
    linear: _Vector3
    angular: _Vector3


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _installed_mission_source() -> Path | None:
    try:
        spec = find_spec("nano_c_slam_gazebo.sim_mission")
    except (ImportError, ModuleNotFoundError, ValueError):
        return None
    if spec is None or spec.origin is None:
        return None
    return Path(spec.origin)


def _runtime_identity() -> dict[str, object]:
    ros2 = shutil.which("ros2")
    gazebo = shutil.which("gz")
    prefix_result = (
        subprocess.run(  # noqa: S603 - executable is the resolved ROS CLI.
            [ros2, "pkg", "prefix", "nano_c_slam_gazebo"],
            capture_output=True,
            check=False,
            text=True,
            timeout=5,
        )
        if ros2
        else None
    )
    package_prefix = (
        Path(prefix_result.stdout.strip())
        if prefix_result and prefix_result.returncode == 0
        else None
    )
    overlay_root = package_prefix.parent.parent if package_prefix else None
    build_receipt = overlay_root / "build-receipt.txt" if overlay_root else None
    source_snapshot = (
        overlay_root / "source-snapshot.before.txt" if overlay_root else None
    )
    installed_source = _installed_mission_source()
    gazebo_version = (
        subprocess.run(  # noqa: S603 - executable is the resolved Gazebo CLI.
            [gazebo, "sim", "--version"],
            capture_output=True,
            check=False,
            text=True,
            timeout=5,
        )
        if gazebo
        else None
    )
    return {
        "ros2_executable": ros2,
        "gazebo_executable": gazebo,
        "gazebo_version_output": (
            (gazebo_version.stdout + gazebo_version.stderr).strip()
            if gazebo_version
            else None
        ),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "gazebo_partition": os.environ.get("GZ_PARTITION"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "nano_c_slam_package_prefix": (str(package_prefix) if package_prefix else None),
        "overlay_build_receipt": str(build_receipt)
        if build_receipt and build_receipt.is_file()
        else None,
        "overlay_build_receipt_sha256": (
            _sha256(build_receipt)
            if build_receipt and build_receipt.is_file()
            else None
        ),
        "overlay_source_snapshot_sha256": _receipt_value(
            build_receipt, "source_snapshot_sha256"
        ),
        "overlay_source_revision": _snapshot_revision(source_snapshot),
        "source_revision": _git_output("rev-parse", "HEAD").strip(),
        "source_status": _git_output("status", "--porcelain").splitlines(),
        "mission_source_sha256": _sha256(MISSION_SOURCE),
        "installed_mission_path": str(installed_source) if installed_source else None,
        "installed_mission_sha256": (
            _sha256(installed_source)
            if installed_source and installed_source.is_file()
            else None
        ),
    }


def _receipt_value(receipt: Path | None, key: str) -> str | None:
    if receipt is None or not receipt.is_file():
        return None
    prefix = f"{key}: "
    for line in receipt.read_text(encoding="utf-8").splitlines():
        if line.startswith(prefix):
            return line.removeprefix(prefix)
    return None


def _snapshot_revision(snapshot: Path | None) -> str | None:
    if snapshot is None or not snapshot.is_file():
        return None
    prefix = "nano-c-slam-revision: "
    for line in snapshot.read_text(encoding="utf-8").splitlines():
        if line.startswith(prefix):
            return line.removeprefix(prefix)
    return None


def _git_output(*arguments: str) -> str:
    git = shutil.which("git")
    if git is None:
        return "unavailable"
    result = subprocess.run(  # noqa: S603 - fixed git subcommands, resolved CLI.
        [git, "-C", str(REPOSITORY_ROOT), *arguments],
        capture_output=True,
        check=False,
        text=True,
        timeout=5,
    )
    return result.stdout


def capture(
    *,
    drone: str,
    output: Path,
    preflight_timeout_s: float,
    mission_timeout_s: float,
    post_exit_duration_s: float,
    hover_injection_delay_s: float,
) -> dict[str, object]:
    """Run the bounded failure experiment and return its retained evidence."""
    rclpy = importlib.import_module("rclpy")
    node_type = importlib.import_module("rclpy.node").Node
    qos_module = importlib.import_module("rclpy.qos")
    odometry_type = importlib.import_module("nav_msgs.msg").Odometry
    clock_message_type = importlib.import_module("rosgraph_msgs.msg").Clock
    string_type = importlib.import_module("std_msgs.msg").String
    bool_type = importlib.import_module("std_msgs.msg").Bool
    twist_type = importlib.import_module("geometry_msgs.msg").Twist
    status_qos = qos_module.QoSProfile(
        depth=10,
        durability=qos_module.DurabilityPolicy.TRANSIENT_LOCAL,
        history=qos_module.HistoryPolicy.KEEP_LAST,
    )

    class ProbeNode(node_type):  # type: ignore[valid-type,misc]
        def __init__(self) -> None:
            super().__init__("nano_c_slam_failure_probe")
            self.started_wall_s = time.monotonic()
            self.odometry: list[dict[str, float | int]] = []
            self.status_events: list[dict[str, object]] = []
            self.commands: list[dict[str, float]] = []
            self.enables: list[dict[str, bool]] = []
            self.status_errors: list[str] = []
            self.clock_stamps_ns: list[int] = []
            self.injection_wall_s: float | None = None
            self.hover_wall_s: float | None = None
            self._odometry_publisher = self.create_publisher(
                odometry_type, f"/{drone}/odom", 10
            )
            self.create_subscription(
                odometry_type, f"/{drone}/odom", self._on_odometry, 10
            )
            self.create_subscription(clock_message_type, "/clock", self._on_clock, 10)
            self.create_subscription(
                string_type, f"/{drone}/mission_status", self._on_status, status_qos
            )
            self.create_subscription(
                bool_type,
                f"/{drone}/enable",
                self._on_enable,
                10,
            )
            self.create_subscription(
                twist_type,
                f"/{drone}/cmd_vel",
                self._on_command,
                10,
            )

        def _on_odometry(self, message: _OdometryMessage) -> None:
            header = message.header
            pose = message.pose
            twist = message.twist
            stamp = header.stamp
            position = pose.pose.position
            velocity = twist.twist.linear
            self.odometry.append(
                {
                    "received_wall_s": time.monotonic() - self.started_wall_s,
                    "stamp_ns": _stamp_ns(stamp),
                    "x_m": float(position.x),
                    "y_m": float(position.y),
                    "z_m": float(position.z),
                    "vx_m_s": float(velocity.x),
                    "vy_m_s": float(velocity.y),
                    "vz_m_s": float(velocity.z),
                }
            )

        def _on_status(self, message: _StringMessage) -> None:
            try:
                payload = json.loads(message.data)
            except (json.JSONDecodeError, TypeError) as error:
                self.status_errors.append(f"invalid status JSON: {error}")
                return
            if not isinstance(payload, dict):
                self.status_errors.append("mission status JSON is not an object")
                return
            event: dict[str, object] = dict(payload)
            event["received_wall_s"] = time.monotonic() - self.started_wall_s
            self.status_events.append(event)
            if event.get("phase") == "HOVER" and self.hover_wall_s is None:
                self.hover_wall_s = time.monotonic()

        def _on_clock(self, message: _ClockMessage) -> None:
            self.clock_stamps_ns.append(_stamp_ns(message.clock))

        def _on_enable(self, message: _BoolMessage) -> None:
            self.enables.append(
                {
                    "received_wall_s": time.monotonic() - self.started_wall_s,
                    "data": bool(message.data),
                }
            )

        def _on_command(self, message: _TwistMessage) -> None:
            self.commands.append(
                {
                    "received_wall_s": time.monotonic() - self.started_wall_s,
                    "linear_x_m_s": float(message.linear.x),
                    "linear_y_m_s": float(message.linear.y),
                    "linear_z_m_s": float(message.linear.z),
                    "angular_z_rad_s": float(message.angular.z),
                }
            )

        def inject_nonfinite_odometry(self) -> None:
            if not self.odometry:
                raise RuntimeError("cannot inject failure without observed odometry")
            source = self.odometry[-1]
            message = odometry_type()
            stamp = message.header.stamp
            stamp_ns = int(source["stamp_ns"])
            stamp.sec = stamp_ns // 1_000_000_000
            stamp.nanosec = stamp_ns % 1_000_000_000
            message.pose.pose.position.x = float("nan")
            message.pose.pose.position.y = float(source["y_m"])
            message.pose.pose.position.z = float(source["z_m"])
            self.injection_wall_s = time.monotonic()
            self._odometry_publisher.publish(message)

    run_id = str(uuid.uuid4())
    output.parent.mkdir(parents=True, exist_ok=True)
    stdout_path = output.with_suffix(".mission.stdout.log")
    stderr_path = output.with_suffix(".mission.stderr.log")
    rclpy.init(args=[])
    node = ProbeNode()
    process: subprocess.Popen[bytes] | None = None
    process_exit_wall_s: float | None = None
    preflight_samples: list[dict[str, float | int]] = []
    control_publishers_before_mission: dict[str, list[str]] | None = None
    failure_injection_description = (
        "one /{drone}/odom sample with NaN x after run-bound HOVER; "
        "the mission rejects non-finite position"
    )
    deadline = time.monotonic() + preflight_timeout_s + mission_timeout_s
    mission_started_wall_s: float | None = None
    post_exit_observed_duration_s: float | None = None
    try:
        while rclpy.ok():
            now = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.02)

            if process is None:
                sample = node.odometry[-1] if node.odometry else None
                if sample is not None:
                    is_grounded_finite = (
                        all(
                            math.isfinite(float(sample[key]))
                            for key in ("x_m", "y_m", "z_m")
                        )
                        and float(sample["z_m"]) <= 0.15
                    )
                    if not is_grounded_finite:
                        preflight_samples.clear()
                    elif (
                        not preflight_samples
                        and now
                        - (node.started_wall_s + float(sample["received_wall_s"]))
                        <= 0.2
                    ):
                        preflight_samples.append(sample)
                    elif preflight_samples and int(sample["stamp_ns"]) > int(
                        preflight_samples[-1]["stamp_ns"]
                    ):
                        source_gap_s = (
                            int(sample["stamp_ns"])
                            - int(preflight_samples[-1]["stamp_ns"])
                        ) / 1_000_000_000
                        receipt_gap_s = float(sample["received_wall_s"]) - float(
                            preflight_samples[-1]["received_wall_s"]
                        )
                        if source_gap_s <= 0.2 and receipt_gap_s <= 0.2:
                            preflight_samples.append(sample)
                        else:
                            preflight_samples[:] = [sample]
                clock_advances = (
                    len(node.clock_stamps_ns) >= 3
                    and node.clock_stamps_ns[-1] > node.clock_stamps_ns[-2]
                    and node.clock_stamps_ns[-2] > node.clock_stamps_ns[-3]
                )
                if len(preflight_samples) >= 3 and clock_advances:
                    if control_publishers_before_mission is None:
                        control_publishers_before_mission = {
                            f"/{drone}/cmd_vel": [
                                endpoint.node_name
                                for endpoint in node.get_publishers_info_by_topic(
                                    f"/{drone}/cmd_vel"
                                )
                            ],
                            f"/{drone}/enable": [
                                endpoint.node_name
                                for endpoint in node.get_publishers_info_by_topic(
                                    f"/{drone}/enable"
                                )
                            ],
                        }
                    if any(control_publishers_before_mission.values()):
                        raise RuntimeError(
                            "flight-control publishers exist before mission start"
                        )
                    ros2 = shutil.which("ros2")
                    if ros2 is None:
                        raise RuntimeError("ros2 executable is not on PATH")
                    mission_started_wall_s = time.monotonic()
                    command = [
                        ros2,
                        "run",
                        "nano_c_slam_gazebo",
                        "sim_mission",
                        "--ros-args",
                        "-p",
                        "use_sim_time:=True",
                        "-p",
                        f"drone_name:={drone}",
                        "-p",
                        f"run_id:={run_id}",
                        "-p",
                        "hover_duration_s:=20.0",
                    ]
                    with (
                        stdout_path.open("wb") as mission_stdout,
                        stderr_path.open("wb") as mission_stderr,
                    ):
                        process = subprocess.Popen(  # noqa: S603 - fixed ros2 argv.
                            command,
                            stdin=subprocess.DEVNULL,
                            stdout=mission_stdout,
                            stderr=mission_stderr,
                        )

            if (
                process is not None
                and process.poll() is None
                and node.hover_wall_s is not None
                and node.injection_wall_s is None
                and now - node.hover_wall_s >= hover_injection_delay_s
            ):
                node.inject_nonfinite_odometry()

            if process is not None and process_exit_wall_s is None:
                exit_code = process.poll()
                if exit_code is not None:
                    process_exit_wall_s = time.monotonic()
            if (
                process_exit_wall_s is not None
                and time.monotonic() - process_exit_wall_s >= post_exit_duration_s
            ):
                post_exit_observed_duration_s = time.monotonic() - process_exit_wall_s
                break
            if process is None and now - node.started_wall_s >= preflight_timeout_s:
                raise TimeoutError(
                    "three fresh grounded odometry samples not observed; "
                    f"odometry_samples={len(node.odometry)}, "
                    f"clock_samples={len(node.clock_stamps_ns)}, "
                    f"last_odometry={node.odometry[-1] if node.odometry else None}"
                )
            if now >= deadline:
                raise TimeoutError("mission failure probe exceeded its deadline")

        if process is None:
            raise RuntimeError("mission process was not started")
        exit_code = process.wait(timeout=1)
    except Exception:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    post_exit_samples = [
        sample
        for sample in node.odometry
        if process_exit_wall_s is not None
        and sample["received_wall_s"] >= process_exit_wall_s - node.started_wall_s
    ]
    failed_events = [
        event for event in node.status_events if event.get("phase") == "FAILED"
    ]
    failed_wall_s = (
        float(failed_events[-1]["received_wall_s"]) + node.started_wall_s
        if failed_events
        else None
    )
    commands_after_failure = [
        command
        for command in node.commands
        if failed_wall_s is not None
        and command["received_wall_s"] + node.started_wall_s >= failed_wall_s
    ]
    zero_commands_after_failure = [
        command
        for command in commands_after_failure
        if command["linear_x_m_s"] == 0.0
        and command["linear_y_m_s"] == 0.0
        and command["linear_z_m_s"] == 0.0
        and command["angular_z_rad_s"] == 0.0
    ]
    post_altitudes = [float(sample["z_m"]) for sample in post_exit_samples]
    post_vertical_speeds = [
        abs(float(sample["vz_m_s"])) for sample in post_exit_samples
    ]
    classification = _classify_post_exit_motion(post_altitudes, post_vertical_speeds)
    runtime_identity = _runtime_identity()
    gazebo_loaded_libraries = capture_session_gazebo_libraries(
        partition=os.environ.get("GZ_PARTITION", ""),
        expected_executable_path=(
            Path(str(runtime_identity["gazebo_executable"]))
            if runtime_identity.get("gazebo_executable")
            else None
        ),
    )
    matching_failed_events = [
        event
        for event in failed_events
        if event.get("run_id") == run_id and event.get("drone_name") == drone
    ]
    acceptance_checks = {
        "three_grounded_samples": len(preflight_samples) >= 3,
        "simulator_clock_advanced": (
            len(node.clock_stamps_ns) >= 3
            and node.clock_stamps_ns[-1] > node.clock_stamps_ns[-2]
            and node.clock_stamps_ns[-2] > node.clock_stamps_ns[-3]
        ),
        "no_command_publishers_before_mission": (
            control_publishers_before_mission is not None
            and not any(control_publishers_before_mission.values())
        ),
        "failure_was_injected_after_hover": (
            node.injection_wall_s is not None and node.hover_wall_s is not None
        ),
        "matching_failed_status_published": bool(matching_failed_events),
        "mission_exited_nonzero": exit_code != 0,
        "at_least_two_zero_commands_after_failure": (
            len(zero_commands_after_failure) >= 2
        ),
        "post_exit_odometry_observed": len(post_exit_samples) >= 3,
        "post_exit_window_met": (
            post_exit_observed_duration_s is not None
            and post_exit_observed_duration_s >= post_exit_duration_s
        ),
        "installed_mission_matches_source": _matching_hashes(
            runtime_identity.get("mission_source_sha256"),
            runtime_identity.get("installed_mission_sha256"),
        ),
        "velocity_controller_plugin_identity_observed": (
            _velocity_controller_plugin(gazebo_loaded_libraries) is not None
        ),
    }
    result: dict[str, object] = {
        "schema_version": 1,
        "run_id": run_id,
        "drone": drone,
        "runtime_identity": runtime_identity,
        "gazebo_loaded_libraries": gazebo_loaded_libraries,
        "mission_command": [
            str(shutil.which("ros2")),
            "run",
            "nano_c_slam_gazebo",
            "sim_mission",
            "--ros-args",
            "-p",
            "use_sim_time:=True",
            "-p",
            f"drone_name:={drone}",
            "-p",
            f"run_id:={run_id}",
            "-p",
            "hover_duration_s:=20.0",
        ],
        "injected_failure": failure_injection_description.format(drone=drone),
        "injection_received_wall_s": (
            node.injection_wall_s - node.started_wall_s
            if node.injection_wall_s is not None
            else None
        ),
        "grounded_preflight_samples": preflight_samples,
        "advancing_clock_sample_count": len(node.clock_stamps_ns),
        "clock_first_stamp_ns": node.clock_stamps_ns[0]
        if node.clock_stamps_ns
        else None,
        "clock_last_stamp_ns": node.clock_stamps_ns[-1]
        if node.clock_stamps_ns
        else None,
        "control_publishers_before_mission": control_publishers_before_mission,
        "mission_started_wall_s": (
            mission_started_wall_s - node.started_wall_s
            if mission_started_wall_s is not None
            else None
        ),
        "mission_exit_wall_s": (
            process_exit_wall_s - node.started_wall_s
            if process_exit_wall_s is not None
            else None
        ),
        "post_exit_observation_requested_s": post_exit_duration_s,
        "post_exit_observation_observed_s": post_exit_observed_duration_s,
        "mission_exit_code": exit_code,
        "acceptance_checks": acceptance_checks,
        "passed": all(acceptance_checks.values()),
        "mission_status_events": node.status_events,
        "status_errors": node.status_errors,
        "enable_events": node.enables,
        "cmd_vel_events": node.commands,
        "failure_command_summary": {
            "commands_after_failed_status": len(commands_after_failure),
            "zero_commands_after_failed_status": len(zero_commands_after_failure),
            "enable_true_after_failed_status": sum(
                1
                for event in node.enables
                if event["data"]
                and failed_wall_s is not None
                and event["received_wall_s"] + node.started_wall_s >= failed_wall_s
            ),
        },
        "post_exit_odometry": {
            "sample_count": len(post_exit_samples),
            "first_stamp_ns": post_exit_samples[0]["stamp_ns"]
            if post_exit_samples
            else None,
            "last_stamp_ns": post_exit_samples[-1]["stamp_ns"]
            if post_exit_samples
            else None,
            "altitude_min_m": min(post_altitudes, default=None),
            "altitude_max_m": max(post_altitudes, default=None),
            "altitude_first_m": post_altitudes[0] if post_altitudes else None,
            "altitude_last_m": post_altitudes[-1] if post_altitudes else None,
            "max_abs_vertical_velocity_m_s": max(post_vertical_speeds, default=None),
            "motion_classification": classification,
            "samples": post_exit_samples,
        },
        "mission_stdout_path": str(stdout_path),
        "mission_stderr_path": str(stderr_path),
    }
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return result


def _stamp_ns(stamp: _Stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _matching_hashes(source_hash: object, installed_hash: object) -> bool:
    return (
        isinstance(source_hash, str)
        and bool(source_hash)
        and isinstance(installed_hash, str)
        and bool(installed_hash)
        and source_hash == installed_hash
    )


def _velocity_controller_plugin(evidence: object) -> dict[str, object] | None:
    if not isinstance(evidence, dict):
        return None
    libraries = evidence.get("libraries")
    if not isinstance(libraries, list):
        return None
    for library in libraries:
        if not isinstance(library, dict):
            continue
        path = library.get("path")
        digest = library.get("sha256")
        if (
            isinstance(path, str)
            and "multicopter-control-system" in path
            and isinstance(digest, str)
            and digest
            and all(isinstance(key, str) for key in library)
        ):
            return cast(dict[str, object], library)
    return None


def _classify_post_exit_motion(
    altitudes_m: list[float], abs_vertical_speeds_m_s: list[float]
) -> str:
    if len(altitudes_m) < 3:
        return "unknown: fewer than three post-exit odometry samples"
    altitude_span_m = max(altitudes_m) - min(altitudes_m)
    if max(altitudes_m) <= 0.15:
        return "landed or remained grounded"
    if altitude_span_m > 0.10 or max(abs_vertical_speeds_m_s, default=0.0) > 0.10:
        return "drift or continued motion"
    if altitude_span_m <= 0.05 and max(abs_vertical_speeds_m_s, default=0.0) <= 0.05:
        return "held near a steady altitude during the observed window"
    return "unknown: observed motion is between hold and drift thresholds"


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", default="cf1")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preflight-timeout", type=float, default=15.0)
    parser.add_argument("--mission-timeout", type=float, default=45.0)
    parser.add_argument("--post-exit-duration", type=float, default=5.0)
    parser.add_argument("--hover-injection-delay", type=float, default=0.75)
    options = parser.parse_args(args)
    if options.drone not in {"cf1", "cf3", "cf4"}:
        parser.error("--drone must be one of cf1, cf3, cf4")
    for name in (
        "preflight_timeout",
        "mission_timeout",
        "post_exit_duration",
        "hover_injection_delay",
    ):
        value = getattr(options, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    try:
        result = capture(
            drone=options.drone,
            output=options.output,
            preflight_timeout_s=options.preflight_timeout,
            mission_timeout_s=options.mission_timeout,
            post_exit_duration_s=options.post_exit_duration,
            hover_injection_delay_s=options.hover_injection_delay,
        )
    except (OSError, RuntimeError, TimeoutError, subprocess.SubprocessError) as error:
        failure_path = options.output.with_suffix(".failure.json")
        failure_path.parent.mkdir(parents=True, exist_ok=True)
        failure_path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "passed": False,
                    "error": str(error),
                    "command": sys.argv if args is None else args,
                    "source_revision": _git_output("rev-parse", "HEAD").strip(),
                    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                    "gazebo_partition": os.environ.get("GZ_PARTITION"),
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        parser.exit(1, f"Gazebo failure probe failed: {error}\n")
    print(
        json.dumps(
            {
                "output": str(options.output),
                "mission_exit_code": result["mission_exit_code"],
                "post_exit_motion": result["post_exit_odometry"],
            },
            sort_keys=True,
        )
    )
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
