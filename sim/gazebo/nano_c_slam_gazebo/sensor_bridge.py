"""Publish Crazyswarm2 Gazebo sensor data in simple Nano-C-SLAM test topics."""

from __future__ import annotations

import time
from collections.abc import Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from .mission_model import SUPPORTED_DRONE_NAMES, validate_drone_names
from .sensor_model import (
    laser_scan_ranges_mm,
    pairwise_ranges_m,
    validate_odometry_timestamps,
)

MAX_ODOMETRY_AGE_S = 0.5
MAX_ODOMETRY_SKEW_S = 0.1
ODOMETRY_WARNING_INTERVAL_S = 2.0


class SensorBridge(Node):
    """Expose four-ray ranges and ideal inter-drone ranges for SITL experiments."""

    def __init__(self) -> None:
        super().__init__("nano_c_slam_gazebo_sensor_bridge")
        self.declare_parameter("drone_names", list(SUPPORTED_DRONE_NAMES))
        self._drone_names = validate_drone_names(
            self.get_parameter("drone_names").value
        )

        self._positions: dict[str, tuple[float, float, float]] = {}
        self._odometry_stamps_s: dict[str, float] = {}
        self._last_odometry_warning_s = float("-inf")
        self._range_publishers = {
            name: self.create_publisher(
                Float32MultiArray, f"/nano_c_slam/sim/{name}/ranges_mm", 10
            )
            for name in self._drone_names
        }
        self._uwb_publisher = self.create_publisher(
            Float32MultiArray, "/nano_c_slam/sim/uwb_pairwise_range_m", 10
        )

        for name in self._drone_names:
            self.create_subscription(
                LaserScan,
                f"/{name}/scan",
                lambda msg, drone=name: self._scan_callback(msg, drone),
                10,
            )
            self.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, drone=name: self._odom_callback(msg, drone),
                10,
            )

        self.get_logger().warning(
            "SITL bridge exposes a four-ray lidar and ideal odometry-derived ranges; "
            "it does not emulate ToF matrices, UWB radio effects, or GAP9/STM32 SPI."
        )

    def _scan_callback(self, msg: LaserScan, drone: str) -> None:
        try:
            ranges = laser_scan_ranges_mm(
                msg.ranges,
                range_min_m=msg.range_min,
                range_max_m=msg.range_max,
            )
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid scan from {drone}: {error}")
            return

        output = Float32MultiArray()
        output.layout.dim = [
            MultiArrayDimension(label="front,back,left,right", size=4, stride=4)
        ]
        output.data = [float(distance) for distance in ranges]
        self._range_publishers[drone].publish(output)

    def _odom_callback(self, msg: Odometry, drone: str) -> None:
        position = msg.pose.pose.position
        self._positions[drone] = (position.x, position.y, position.z)
        stamp = msg.header.stamp
        self._odometry_stamps_s[drone] = float(stamp.sec) + float(stamp.nanosec) / 1e9
        if len(self._positions) != len(self._drone_names):
            return

        try:
            now_s = self.get_clock().now().nanoseconds / 1e9
            validate_odometry_timestamps(
                self._odometry_stamps_s,
                now_s=now_s,
                max_age_s=MAX_ODOMETRY_AGE_S,
                max_skew_s=MAX_ODOMETRY_SKEW_S,
            )
            distances = pairwise_ranges_m(self._positions)
        except ValueError as error:
            self._warn_invalid_odometry(str(error))
            return
        output = Float32MultiArray()
        output.layout.dim = [
            MultiArrayDimension(
                label=",".join(f"{first}:{second}" for first, second, _ in distances),
                size=len(distances),
                stride=len(distances),
            )
        ]
        output.data = [distance for _, _, distance in distances]
        self._uwb_publisher.publish(output)

    def _warn_invalid_odometry(self, reason: str) -> None:
        """Throttle warnings while a drone's odometry stream is unavailable."""
        now_s = time.monotonic()
        if now_s - self._last_odometry_warning_s < ODOMETRY_WARNING_INTERVAL_S:
            return
        self._last_odometry_warning_s = now_s
        self.get_logger().warning(f"Skipping pairwise ranges: {reason}")


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
