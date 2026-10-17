"""Small ROS message-shaped fixtures for hardware-free unit tests."""

from __future__ import annotations

from types import SimpleNamespace


def odometry_message(
    *,
    x: float,
    y: float = 0.0,
    z: float = 0.0,
    sec: int = 1,
    nanosec: int = 0,
) -> SimpleNamespace:
    """Build only the Odometry fields read by the mission and sensor bridge."""
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z))
        ),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
    )
