"""Small, structurally realistic Gazebo protobuf-text captures for flight gates."""

from __future__ import annotations

from dataclasses import dataclass

GROUND_Z_M = 2.04
HOVER_Z_M = 2.40
SAMPLE_PERIOD_S = 0.01


@dataclass(frozen=True)
class FlightCapture:
    odometry: str
    imu: str
    motors: str
    firmware_log: str
    plugin_log: str
    negative_odometry: str
    negative_motors: str
    negative_log: str


def _header(time_s: float) -> str:
    sec = int(time_s)
    nsec = round((time_s - sec) * 1_000_000_000)
    return f"header {{\n  stamp {{ sec: {sec} nsec: {nsec} }}\n}}\n"


def _odom(time_s: float, z_m: float, *, angular_x: str = "0") -> str:
    return (
        _header(time_s)
        + "pose { position { x: 0 y: 0 z: "
        + f"{z_m:.6f}"
        + " } orientation { x: 0 y: 0 z: 0 w: 1 } }\n"
        + "twist { linear { x: 0 y: 0 z: 0 } "
        + f"angular {{ x: {angular_x} y: 0 z: 0 }}"
        + " }\n\n"
    )


def _imu(time_s: float, *, acceleration_z: str = "9.80665") -> str:
    return (
        _header(time_s)
        + "linear_acceleration { x: 0 y: 0 z: "
        + acceleration_z
        + " }\n"
        + "angular_velocity { x: 0 y: 0 z: 0 }\n\n"
    )


def _flight_z(time_s: float) -> float:
    if time_s < 1.0:
        return GROUND_Z_M + (HOVER_Z_M - GROUND_Z_M) * time_s
    if time_s < 7.0:
        return HOVER_Z_M
    if time_s < 8.0:
        return HOVER_Z_M - (HOVER_Z_M - GROUND_Z_M) * (time_s - 7.0)
    return GROUND_Z_M


def passing_capture() -> FlightCapture:
    """Build a stationary-ground, climb, six-second hover, and landing run."""
    count = 901
    odometry = "".join(
        _odom(index * SAMPLE_PERIOD_S, _flight_z(index * SAMPLE_PERIOD_S))
        for index in range(count)
    )
    imu = "".join(_imu(index * SAMPLE_PERIOD_S) for index in range(count))
    motors = "".join(
        "\n".join(f"velocity: {1800 if 100 <= index < 800 else 0}" for _ in range(4))
        + "\n\n"
        for index in range(count)
    )
    negative_odometry = "".join(
        _odom(index * SAMPLE_PERIOD_S, GROUND_Z_M) for index in range(51)
    )
    negative_motors = "".join("velocity: 0\n" * 4 + "\n" for _ in range(51))
    firmware_log = "\n".join(
        (
            "SYS: Self test passed!",
            "SENSORS: GYro Bias found",
            "STAB: Starting stabilizer loop",
            "TRACE_HL_RX header=8c size=15 data=07 00 00 00",
            "TRACE_HL_RX header=8c size=15 data=08 00 00 00",
        )
    )
    return FlightCapture(
        odometry=odometry,
        imu=imu,
        motors=motors,
        firmware_log=firmware_log,
        plugin_log="TRACE_MOTOR_NONZERO 20000,20000,20000,20000",
        negative_odometry=negative_odometry,
        negative_motors=negative_motors,
        negative_log="actuator_bridge=disconnected",
    )
