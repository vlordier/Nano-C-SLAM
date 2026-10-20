"""Reject CrazySim captures that do not prove a finite takeoff/hover/land."""

from __future__ import annotations

import argparse
import itertools
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

type Vector3 = tuple[float, float, float]
type Vector4 = tuple[float, float, float, float]

SPAWN_GROUND_Z_M = 2.04
GROUND_TOLERANCE_M = 0.10
MIN_TAKEOFF_RISE_M = 0.30
HOVER_TARGET_Z_M = 2.40
HOVER_TOLERANCE_M = 0.12
MIN_HOVER_DURATION_S = 5.0
MAX_HOVER_ANGLE_RAD = math.radians(15.0)
MAX_HOVER_ANGULAR_RATE_RAD_S = 0.5
MAX_SENSOR_GAP_S = 0.05
MAX_MOTOR_SPEED_RAD_S = 3052.0
MIN_SENSOR_SAMPLES = 5
MIN_AGGREGATE_REAL_TIME_FACTOR = 0.95
MAX_AGGREGATE_REAL_TIME_FACTOR = 1.05
MIN_WORLD_STATS_COVERAGE = 0.95

_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[-+]?(?:nan|inf)"
_FIELD = re.compile(rf"\b([xyzw]):\s*({_NUMBER})", re.IGNORECASE)
_STAMP = re.compile(r"stamp\s*\{\s*sec:\s*(-?\d+)(?:\s+nsec:\s*(-?\d+))?", re.DOTALL)
_HEADER_START = re.compile(r"(?m)^header\s*\{")


@dataclass(frozen=True)
class OdometrySample:
    time_s: float
    position: Vector3
    orientation: Vector4
    linear_velocity: Vector3
    angular_velocity: Vector3


@dataclass(frozen=True)
class WorldStatsSample:
    sim_time_s: float
    real_time_s: float
    real_time_factor: float


@dataclass(frozen=True)
class CaptureSummary:
    odometry_samples: int
    sensor_samples: int
    motor_components: int
    max_altitude_m: float
    max_motor_speed_rad_s: float
    hover_duration_s: float
    negative_control_drift_m: float
    aggregate_real_time_factor: float


def _messages(text: str) -> list[str]:
    starts = [match.start() for match in _HEADER_START.finditer(text)]
    ends = itertools.chain(starts[1:], (len(text),))
    return [text[start:end] for start, end in zip(starts, ends, strict=True)]


def _json_messages(text: str) -> list[dict[str, Any]] | None:
    """Read newline-delimited JSON emitted by `gz topic --json-output`."""
    if not text.lstrip().startswith("{"):
        return None
    messages: list[dict[str, Any]] = []
    try:
        for line in text.splitlines():
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                return None
            messages.append(value)
    except json.JSONDecodeError:
        return None
    return messages


def _object(value: object) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _json_number(value: object, default: float = 0.0) -> float:
    if not isinstance(value, (str, int, float)):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _json_timestamp(message: dict[str, Any], field: str = "header") -> float | None:
    stamp_value = _object(message.get(field)).get("stamp")
    if not isinstance(stamp_value, dict):
        return None
    stamp = stamp_value
    return _json_number(stamp.get("sec")) + _json_number(stamp.get("nsec")) / 1e9


def _json_vector(message: dict[str, Any], field: str, axes: str) -> tuple[float, ...]:
    values = _object(message.get(field))
    return tuple(_json_number(values.get(axis)) for axis in axes)


def _vector(message: str, field: str, axes: str) -> tuple[float, ...]:
    match = re.search(rf"\b{re.escape(field)}\s*\{{([^{{}}]*)\}}", message, re.DOTALL)
    if match is None:
        return tuple(0.0 for _ in axes)
    values = {axis: float(value) for axis, value in _FIELD.findall(match.group(1))}
    return tuple(values.get(axis, 0.0) for axis in axes)


def _timestamp_seconds(match: re.Match[str]) -> float:
    seconds = int(match.group(1))
    nanoseconds = int(match.group(2) or 0)
    return seconds + nanoseconds / 1_000_000_000


def parse_odometry(text: str) -> list[OdometrySample]:
    """Read protobuf text or JSON output for gz.msgs.Odometry."""
    samples: list[OdometrySample] = []
    json_messages = _json_messages(text)
    if json_messages is not None:
        for message in json_messages:
            timestamp = _json_timestamp(message)
            if timestamp is None:
                continue
            pose = _object(message.get("pose"))
            position = _json_vector(pose, "position", "xyz")
            orientation = _json_vector(pose, "orientation", "xyzw")
            twist = _object(message.get("twist"))
            linear = _json_vector(twist, "linear", "xyz")
            angular = _json_vector(twist, "angular", "xyz")
            samples.append(
                OdometrySample(
                    time_s=timestamp,
                    position=(position[0], position[1], position[2]),
                    orientation=(
                        orientation[0],
                        orientation[1],
                        orientation[2],
                        orientation[3],
                    ),
                    linear_velocity=(linear[0], linear[1], linear[2]),
                    angular_velocity=(angular[0], angular[1], angular[2]),
                )
            )
        return samples
    for message in _messages(text):
        stamp = _STAMP.search(message)
        if stamp is None:
            continue
        timestamp = _timestamp_seconds(stamp)
        position = _vector(message, "position", "xyz")
        orientation = _vector(message, "orientation", "xyzw")
        linear = _vector(message, "linear", "xyz")
        angular = _vector(message, "angular", "xyz")
        samples.append(
            OdometrySample(
                time_s=timestamp,
                position=(position[0], position[1], position[2]),
                orientation=(
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    orientation[3],
                ),
                linear_velocity=(linear[0], linear[1], linear[2]),
                angular_velocity=(angular[0], angular[1], angular[2]),
            )
        )
    return samples


def parse_imu_timestamps(text: str) -> list[float]:
    """Read timestamps from captured protobuf text or JSON IMU messages."""
    timestamps: list[float] = []
    json_messages = _json_messages(text)
    if json_messages is not None:
        return [
            timestamp
            for message in json_messages
            if (timestamp := _json_timestamp(message)) is not None
        ]
    for message in _messages(text):
        stamp = _STAMP.search(message)
        if stamp is None:
            continue
        timestamps.append(_timestamp_seconds(stamp))
    return timestamps


def parse_imu_vectors(text: str) -> list[tuple[Vector3, Vector3]]:
    """Read IMU vectors from captured protobuf text or JSON messages."""
    vectors: list[tuple[Vector3, Vector3]] = []
    json_messages = _json_messages(text)
    if json_messages is not None:
        for message in json_messages:
            if _json_timestamp(message) is None:
                continue
            acceleration = _json_vector(
                message,
                "linear_acceleration"
                if "linear_acceleration" in message
                else "linearAcceleration",
                "xyz",
            )
            angular = _json_vector(
                message,
                "angular_velocity"
                if "angular_velocity" in message
                else "angularVelocity",
                "xyz",
            )
            vectors.append(
                (
                    (acceleration[0], acceleration[1], acceleration[2]),
                    (angular[0], angular[1], angular[2]),
                )
            )
        return vectors
    for message in _messages(text):
        if _STAMP.search(message) is None:
            continue
        acceleration = _vector(message, "linear_acceleration", "xyz")
        angular = _vector(message, "angular_velocity", "xyz")
        vectors.append(
            (
                (acceleration[0], acceleration[1], acceleration[2]),
                (angular[0], angular[1], angular[2]),
            )
        )
    return vectors


def parse_motor_components(text: str) -> list[float]:
    """Read motor speeds from protobuf text or JSON Actuators messages."""
    json_messages = _json_messages(text)
    if json_messages is not None:
        return [
            _json_number(value)
            for message in json_messages
            for value in (
                message.get("velocity", [])
                if isinstance(message.get("velocity", []), list)
                else []
            )
        ]
    return [
        float(value)
        for value in re.findall(rf"\bvelocity:\s*({_NUMBER})", text, re.IGNORECASE)
    ]


def _capture_text(path: Path) -> str:
    """Read plain or gzip-compressed sampler captures."""
    if path.suffix == ".gz":
        import gzip

        with gzip.open(path, "rt", encoding="utf-8") as stream:
            return stream.read()
    return path.read_text(encoding="utf-8")


def validate_plugin_motor_trace(text: str, *, disconnected: bool = False) -> list[str]:
    """Validate bounded CrazySim receive records against their wire packets."""
    errors: list[str] = []
    records = 0
    nonzero = False
    for line_number, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            errors.append(f"plugin trace line {line_number} is not valid JSON")
            continue
        if not isinstance(record, dict):
            errors.append(f"plugin trace line {line_number} is not an object")
            continue
        packet = record.get("packet_bytes")
        ratios = record.get("motor_ratio")
        speeds = record.get("omega_rad_s")
        if (
            not isinstance(packet, list)
            or len(packet) != 9
            or any(type(value) is not int or not 0 <= value <= 255 for value in packet)
        ):
            errors.append(f"plugin trace line {line_number} has malformed packet bytes")
            continue
        if packet[0] != 0x90:
            errors.append(
                f"plugin trace line {line_number} has unexpected packet header"
            )
        expected = [
            packet[index * 2 + 1] | packet[index * 2 + 2] << 8 for index in range(4)
        ]
        if (
            not isinstance(ratios, list)
            or len(ratios) != 4
            or any(
                type(value) is not int or value != expected[index]
                for index, value in enumerate(ratios)
            )
        ):
            errors.append(
                f"plugin trace line {line_number} ratios disagree with packet bytes"
            )
            continue
        if (
            not isinstance(speeds, list)
            or len(speeds) != 4
            or any(
                not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in speeds
            )
        ):
            errors.append(f"plugin trace line {line_number} has malformed motor speeds")
            continue
        if any(value < 0.0 or value > MAX_MOTOR_SPEED_RAD_S for value in speeds):
            errors.append(
                f"plugin trace line {line_number} has out-of-range motor speed"
            )
        nonzero = nonzero or any(value != 0 for value in ratios)
        records += 1
    if records == 0:
        errors.append("plugin motor trace has no valid records")
    if not disconnected and records and not nonzero:
        errors.append("positive plugin motor trace has no nonzero firmware ratios")
    if disconnected and records and not nonzero:
        errors.append(
            "negative plugin motor trace has no nonzero received firmware ratios"
        )
    return errors


def parse_world_stats(text: str) -> list[WorldStatsSample]:
    """Read Gazebo WorldStatistics from protobuf text or JSON output."""
    json_messages = _json_messages(text)
    if json_messages is not None:
        samples: list[WorldStatsSample] = []
        for message in json_messages:
            sim_stamp = message.get("sim_time", message.get("simTime"))
            real_stamp = message.get("real_time", message.get("realTime"))
            sim_time = _json_timestamp({"header": {"stamp": sim_stamp}})
            real_time = _json_timestamp({"header": {"stamp": real_stamp}})
            if sim_time is None or real_time is None:
                continue
            samples.append(
                WorldStatsSample(
                    sim_time_s=sim_time,
                    real_time_s=real_time,
                    real_time_factor=_json_number(
                        message.get("real_time_factor", message.get("realTimeFactor")),
                        math.nan,
                    ),
                )
            )
        return samples
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[-+]?(?:nan|inf)"
    pattern = re.compile(
        rf"sim_time\s*\{{\s*sec:\s*(-?\d+)(?:\s+nsec:\s*(-?\d+))?\s*\}}\s*"
        rf"real_time\s*\{{\s*sec:\s*(-?\d+)(?:\s+nsec:\s*(-?\d+))?\s*\}}\s*"
        rf"iterations:\s*(\d+)\s*real_time_factor:\s*({number})",
        re.IGNORECASE,
    )
    samples: list[WorldStatsSample] = []
    for match in pattern.finditer(text):
        sim_sec, sim_nsec, real_sec, real_nsec = (
            int(value or 0) for value in match.groups()[:4]
        )
        samples.append(
            WorldStatsSample(
                sim_time_s=sim_sec + sim_nsec / 1_000_000_000,
                real_time_s=real_sec + real_nsec / 1_000_000_000,
                real_time_factor=float(match.group(6)),
            )
        )
    return samples


def _has_hover(
    samples: list[OdometrySample], target_z_m: float, tolerance_m: float
) -> float:
    longest = 0.0
    run_start: float | None = None
    previous_time: float | None = None
    for sample in samples:
        upright_error = _tilt_angle_rad(sample.orientation)
        angular_rate = math.sqrt(
            sum(value * value for value in sample.angular_velocity)
        )
        stable = (
            abs(sample.position[2] - target_z_m) <= tolerance_m
            and upright_error <= MAX_HOVER_ANGLE_RAD
            and angular_rate <= MAX_HOVER_ANGULAR_RATE_RAD_S
        )
        if not stable or (
            previous_time is not None and sample.time_s - previous_time > 0.02
        ):
            run_start = None
        if stable and run_start is None:
            run_start = sample.time_s
        if stable and run_start is not None:
            longest = max(longest, sample.time_s - run_start)
        previous_time = sample.time_s
    return longest


def _tilt_angle_rad(orientation: Vector4) -> float:
    """Return body-Z deviation from world vertical, independent of yaw."""
    norm_squared = sum(value * value for value in orientation)
    if not math.isfinite(norm_squared) or norm_squared <= 0.0:
        return math.inf
    x, y, _, _ = orientation
    world_vertical_alignment = 1.0 - 2.0 * (x * x + y * y) / norm_squared
    return math.acos(max(-1.0, min(1.0, world_vertical_alignment)))


def _command_ids(firmware_log: str) -> set[int]:
    return {
        int(value, 16)
        for value in re.findall(
            r"TRACE_HL_RX\s+header=8c\s+size=\d+\s+data=([0-9a-fA-F]{2})",
            firmware_log,
        )
    }


def _firmware_startup_count(firmware_log: str) -> int:
    """Count one of the firmware's explicit UDP or CRTP startup receipts."""
    return firmware_log.count("TRACE_CRTP_RX_STARTED") + firmware_log.count(
        "SOCKET_LINK: Create socket succeed"
    )


def _check_timestamps(
    timestamps: list[float], label: str, errors: list[str], *, max_gap_s: float
) -> None:
    if len(timestamps) < MIN_SENSOR_SAMPLES:
        errors.append(f"{label}: only {len(timestamps)} timestamped samples")
        return
    gaps = [later - earlier for earlier, later in itertools.pairwise(timestamps)]
    if any(gap <= 0.0 for gap in gaps):
        errors.append(f"{label}: timestamps are not strictly increasing")
    if gaps and max(gaps) > max_gap_s:
        errors.append(
            f"{label}: stale sample gap {max(gaps):.3f}s exceeds {max_gap_s:.3f}s"
        )


def validate_capture(
    *,
    odometry_text: str,
    imu_text: str,
    world_stats_text: str,
    motor_text: str,
    firmware_log: str,
    plugin_log: str,
    negative_control_odometry_text: str,
    negative_control_motor_text: str,
    negative_control_firmware_log: str,
    negative_control_plugin_log: str,
    firmware_exit_code: int = 0,
    gazebo_exit_code: int = 0,
    negative_control_firmware_exit_code: int = 0,
    negative_control_gazebo_exit_code: int = 0,
    firmware_process_count: int | None = None,
    negative_control_firmware_process_count: int | None = None,
    ground_z_m: float = SPAWN_GROUND_Z_M,
    hover_target_z_m: float = HOVER_TARGET_Z_M,
    plugin_motor_trace_text: str = "",
    negative_control_plugin_motor_trace_text: str = "",
) -> tuple[CaptureSummary | None, list[str]]:
    """Apply the pre-registered flight gate to one positive and one negative run."""
    errors: list[str] = []
    samples = parse_odometry(odometry_text)
    imu_times = parse_imu_timestamps(imu_text)
    imu_vectors = parse_imu_vectors(imu_text)
    world_stats = parse_world_stats(world_stats_text)
    motors = parse_motor_components(motor_text)
    negative_samples = parse_odometry(negative_control_odometry_text)
    negative_motors = parse_motor_components(negative_control_motor_text)

    if len(samples) < MIN_SENSOR_SAMPLES:
        errors.append("positive run has too few odometry samples")
    _check_timestamps(
        [sample.time_s for sample in samples],
        "odometry",
        errors,
        max_gap_s=0.02,
    )
    _check_timestamps(imu_times, "IMU", errors, max_gap_s=MAX_SENSOR_GAP_S)

    aggregate_real_time_factor = math.nan
    if len(world_stats) < 2:
        errors.append("world stats: at least two timestamped samples are required")
    elif samples:
        sim_times = [sample.sim_time_s for sample in world_stats]
        real_times = [sample.real_time_s for sample in world_stats]
        if any(
            not math.isfinite(value)
            for sample in world_stats
            for value in (
                sample.sim_time_s,
                sample.real_time_s,
                sample.real_time_factor,
            )
        ):
            errors.append("world stats contain NaN or infinite values")
        if any(later <= earlier for earlier, later in itertools.pairwise(sim_times)):
            errors.append(
                "world stats simulation timestamps are not strictly increasing"
            )
        if any(later <= earlier for earlier, later in itertools.pairwise(real_times)):
            errors.append(
                "world stats wall-clock timestamps are not strictly increasing"
            )
        sim_span = sim_times[-1] - sim_times[0]
        real_span = real_times[-1] - real_times[0]
        odometry_span = samples[-1].time_s - samples[0].time_s
        if sim_span <= 0.0 or real_span <= 0.0 or odometry_span <= 0.0:
            errors.append("world stats and odometry must cover a positive time span")
        else:
            coverage = sim_span / odometry_span
            if coverage < MIN_WORLD_STATS_COVERAGE:
                errors.append(
                    "world stats cover only "
                    f"{coverage:.1%} of the positive odometry interval; "
                    f"requires at least {MIN_WORLD_STATS_COVERAGE:.0%}"
                )
            aggregate_real_time_factor = sim_span / real_span
            if not (
                MIN_AGGREGATE_REAL_TIME_FACTOR
                <= aggregate_real_time_factor
                <= MAX_AGGREGATE_REAL_TIME_FACTOR
            ):
                errors.append(
                    "aggregate Gazebo real-time factor is "
                    f"{aggregate_real_time_factor:.3f}; requires "
                    f"[{MIN_AGGREGATE_REAL_TIME_FACTOR:.2f}, "
                    f"{MAX_AGGREGATE_REAL_TIME_FACTOR:.2f}]"
                )

    numeric_values = [
        value
        for sample in samples
        for vector in (
            sample.position,
            sample.orientation,
            sample.linear_velocity,
            sample.angular_velocity,
        )
        for value in vector
    ]
    imu_values = [
        value for sample in imu_vectors for vector in sample for value in vector
    ]
    if any(
        not math.isfinite(value) for value in (*numeric_values, *imu_values, *motors)
    ):
        errors.append("positive run contains NaN or infinite state/actuator values")
    if not motors:
        errors.append("positive run has no captured motor commands")
    elif len(motors) % 4 != 0:
        errors.append(
            "positive run has malformed motor messages (expected four values)"
        )
    elif any(value < 0.0 or value > MAX_MOTOR_SPEED_RAD_S for value in motors):
        errors.append(
            f"motor command is outside [0, {MAX_MOTOR_SPEED_RAD_S:.0f}] rad/s"
        )
    if motors and not any(value > 0.0 for value in motors):
        errors.append("positive run has no nonzero actuator command")
    if not re.search(
        r"TRACE_MOTOR_NONZERO|NCS_SITL_ACTUATOR\s+source=firmware", plugin_log
    ):
        if plugin_motor_trace_text:
            errors.extend(validate_plugin_motor_trace(plugin_motor_trace_text))
        else:
            errors.append(
                "Gazebo plugin log does not attribute motor output to firmware"
            )
    elif plugin_motor_trace_text:
        errors.extend(validate_plugin_motor_trace(plugin_motor_trace_text))

    firmware_instances = _firmware_startup_count(firmware_log)
    if firmware_instances != 1:
        errors.append(
            "positive run log must contain exactly one firmware startup marker; "
            f"found {firmware_instances}"
        )
    if firmware_process_count is None:
        errors.append("positive run is missing firmware process-count evidence")
    elif firmware_process_count != 1:
        errors.append(
            "positive run must have exactly one firmware process; "
            f"observed {firmware_process_count}"
        )

    required_markers = (
        "Self test passed!",
        "GYro Bias found",
        "Starting stabilizer loop",
    )
    errors.extend(
        f"firmware readiness marker missing: {marker}"
        for marker in required_markers
        if marker not in firmware_log
    )
    commands = _command_ids(firmware_log)
    commands.update(
        int(value)
        for value in re.findall(r"NCS_HIGH_LEVEL_COMMAND\s+id=(\d+)", firmware_log)
    )
    if 7 not in commands and "Take-off:" not in firmware_log:
        errors.append("firmware did not receive TAKEOFF_2")
    if 8 not in commands and "Mission ended after landing" not in firmware_log:
        errors.append("firmware did not receive LAND_2")
    forbidden = re.compile(
        r"State out of bounds|rate is off|prediction rate off|"
        r"Crashed, recovery required",
        re.IGNORECASE,
    )
    runtime_log = firmware_log.split("\n--- CAPTURE TRACE DATA ---\n", maxsplit=1)[0]
    if forbidden.search(runtime_log):
        errors.append("firmware reported estimator/rate/crash failure")
    if firmware_exit_code != 0:
        errors.append(f"firmware exited with status {firmware_exit_code}")
    if gazebo_exit_code != 0:
        errors.append(f"Gazebo exited with status {gazebo_exit_code}")
    if negative_control_firmware_exit_code != 0:
        errors.append(
            "negative-control firmware exited with status "
            f"{negative_control_firmware_exit_code}"
        )
    if negative_control_firmware_process_count is None:
        errors.append("negative control is missing firmware process-count evidence")
    elif negative_control_firmware_process_count != 1:
        errors.append(
            "negative control must have exactly one firmware process; "
            f"observed {negative_control_firmware_process_count}"
        )
    if _firmware_startup_count(negative_control_firmware_log) != 1:
        errors.append(
            "negative control log must contain exactly one firmware startup marker"
        )
    if negative_control_gazebo_exit_code != 0:
        errors.append(
            "negative-control Gazebo exited with status "
            f"{negative_control_gazebo_exit_code}"
        )

    if samples:
        start_z = samples[0].position[2]
        max_z = max(sample.position[2] for sample in samples)
        end_z = samples[-1].position[2]
        if abs(start_z - ground_z_m) > GROUND_TOLERANCE_M:
            errors.append("positive run spawn pose is outside the ground envelope")
        if max_z - start_z < MIN_TAKEOFF_RISE_M:
            errors.append("positive run did not rise by the required 0.30 m")
        hover_duration = _has_hover(samples, hover_target_z_m, HOVER_TOLERANCE_M)
        if hover_duration < MIN_HOVER_DURATION_S:
            errors.append(
                f"stable hover lasted {hover_duration:.2f}s; "
                f"requires {MIN_HOVER_DURATION_S:.2f}s"
            )
        if abs(end_z - ground_z_m) > GROUND_TOLERANCE_M:
            errors.append("positive run did not land within 0.10 m of the platform")
    else:
        start_z = max_z = end_z = hover_duration = 0.0

    negative_drift = 0.0
    if not negative_samples:
        errors.append("disconnected-actuator negative control has no odometry")
    else:
        negative_z = [sample.position[2] for sample in negative_samples]
        negative_drift = max(negative_z) - negative_z[0]
        if abs(negative_z[0] - ground_z_m) > GROUND_TOLERANCE_M:
            errors.append("negative control spawn pose is outside the ground envelope")
        negative_values = [
            value
            for sample in negative_samples
            for vector in (
                sample.position,
                sample.orientation,
                sample.linear_velocity,
                sample.angular_velocity,
            )
            for value in vector
        ]
        if any(
            not math.isfinite(value) for value in (*negative_values, *negative_motors)
        ):
            errors.append("negative control contains non-finite state/actuator values")
        if negative_drift > GROUND_TOLERANCE_M:
            errors.append("disconnected-actuator control rose more than 0.10 m")
        if any(value != 0.0 for value in negative_motors):
            errors.append(
                "disconnected-actuator control emitted nonzero motor commands"
            )
    if "actuator_bridge=disconnected" not in negative_control_plugin_log:
        errors.append("negative control lacks explicit disconnected-actuator marker")
    if negative_control_plugin_motor_trace_text:
        errors.extend(
            validate_plugin_motor_trace(
                negative_control_plugin_motor_trace_text, disconnected=True
            )
        )

    if errors:
        return None, errors
    return (
        CaptureSummary(
            odometry_samples=len(samples),
            sensor_samples=len(imu_times),
            motor_components=len(motors),
            max_altitude_m=max_z,
            max_motor_speed_rad_s=max(motors),
            hover_duration_s=hover_duration,
            negative_control_drift_m=negative_drift,
            aggregate_real_time_factor=aggregate_real_time_factor,
        ),
        [],
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odometry", required=True, type=Path)
    parser.add_argument("--imu", required=True, type=Path)
    parser.add_argument("--world-stats", required=True, type=Path)
    parser.add_argument("--motors", required=True, type=Path)
    parser.add_argument("--firmware-log", required=True, type=Path)
    parser.add_argument("--plugin-log", required=True, type=Path)
    parser.add_argument("--negative-control-odometry", required=True, type=Path)
    parser.add_argument("--negative-control-motors", required=True, type=Path)
    parser.add_argument("--negative-control-firmware-log", required=True, type=Path)
    parser.add_argument("--negative-control-plugin-log", required=True, type=Path)
    parser.add_argument("--plugin-motor-trace", type=Path)
    parser.add_argument("--negative-control-plugin-motor-trace", type=Path)
    parser.add_argument("--firmware-process-count", required=True, type=int)
    parser.add_argument(
        "--negative-control-firmware-process-count", required=True, type=int
    )
    parser.add_argument("--firmware-exit-code", type=int, default=0)
    parser.add_argument("--gazebo-exit-code", type=int, default=0)
    parser.add_argument("--negative-control-firmware-exit-code", type=int, default=0)
    parser.add_argument("--negative-control-gazebo-exit-code", type=int, default=0)
    parser.add_argument("--ground-z", type=float, default=SPAWN_GROUND_Z_M)
    parser.add_argument("--hover-target-z", type=float, default=HOVER_TARGET_Z_M)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    summary, errors = validate_capture(
        odometry_text=_capture_text(args.odometry),
        imu_text=_capture_text(args.imu),
        world_stats_text=_capture_text(args.world_stats),
        motor_text=_capture_text(args.motors),
        firmware_log=_capture_text(args.firmware_log),
        plugin_log=_capture_text(args.plugin_log),
        negative_control_odometry_text=_capture_text(args.negative_control_odometry),
        negative_control_motor_text=_capture_text(args.negative_control_motors),
        negative_control_firmware_log=_capture_text(args.negative_control_firmware_log),
        negative_control_plugin_log=_capture_text(args.negative_control_plugin_log),
        firmware_process_count=args.firmware_process_count,
        negative_control_firmware_process_count=(
            args.negative_control_firmware_process_count
        ),
        firmware_exit_code=args.firmware_exit_code,
        gazebo_exit_code=args.gazebo_exit_code,
        negative_control_firmware_exit_code=args.negative_control_firmware_exit_code,
        negative_control_gazebo_exit_code=args.negative_control_gazebo_exit_code,
        ground_z_m=args.ground_z,
        hover_target_z_m=args.hover_target_z,
        plugin_motor_trace_text=(
            _capture_text(args.plugin_motor_trace) if args.plugin_motor_trace else ""
        ),
        negative_control_plugin_motor_trace_text=(
            _capture_text(args.negative_control_plugin_motor_trace)
            if args.negative_control_plugin_motor_trace
            else ""
        ),
    )
    if errors:
        print(json.dumps({"passed": False, "errors": errors}, indent=2))
        return 1
    print(json.dumps({"passed": True, "summary": summary.__dict__}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
