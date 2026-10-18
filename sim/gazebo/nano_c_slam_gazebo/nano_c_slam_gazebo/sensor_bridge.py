"""Publish Crazyswarm2 Gazebo sensor data in simple Nano-C-SLAM test topics."""

from __future__ import annotations

import time
from collections.abc import Sequence

import rclpy
from nano_c_slam_interfaces.msg import StampedPairwiseRanges, StampedRanges
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .mission_model import (
    MAX_ODOMETRY_FUTURE_SKEW_S,
    SUPPORTED_DRONE_NAMES,
    validate_drone_names,
)
from .sensor_model import (
    laser_scan_ranges_mm,
    odometry_stamp_seconds,
    update_odometry_cache,
)

MAX_ODOMETRY_AGE_S = 0.5
MAX_ODOMETRY_SKEW_S = MAX_ODOMETRY_FUTURE_SKEW_S
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
        self._odometry_source_stamps: dict[str, object] = {}
        self._pairwise_sequence = 0
        self._last_odometry_warning_s = float("-inf")
        self._range_sequences = dict.fromkeys(self._drone_names, 0)
        self._range_publishers = {
            name: self.create_publisher(
                StampedRanges, f"/nano_c_slam/sim/{name}/ranges_mm", 10
            )
            for name in self._drone_names
        }
        self._uwb_publisher = self.create_publisher(
            StampedPairwiseRanges, "/nano_c_slam/sim/uwb_pairwise_range_m", 10
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
                angle_min_rad=msg.angle_min,
                angle_increment_rad=msg.angle_increment,
                range_min_m=msg.range_min,
                range_max_m=msg.range_max,
            )
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid scan from {drone}: {error}")
            return

        output = StampedRanges()
        output.header = msg.header
        output.drone_id = drone
        output.sequence = self._range_sequences[drone]
        # Generated fixed-array setters do not validate array length. The pure
        # converter guarantees four ints before assignment; keep the guard at
        # this serialization boundary to avoid malformed CDR messages.
        if len(ranges) != 4 or any(type(value) is not int for value in ranges):
            self.get_logger().error("Refusing malformed four-ray range payload")
            return
        output.ranges_mm = list(ranges)
        self._range_publishers[drone].publish(output)
        self._range_sequences[drone] += 1

    def _odom_callback(self, msg: Odometry, drone: str) -> None:
        position = msg.pose.pose.position
        stamp = msg.header.stamp
        try:
            now_s = self.get_clock().now().nanoseconds / 1e9
            update = update_odometry_cache(
                self._positions,
                self._odometry_stamps_s,
                drone=drone,
                position=(position.x, position.y, position.z),
                stamp_s=odometry_stamp_seconds(stamp.sec, stamp.nanosec),
                drone_names=self._drone_names,
                now_s=now_s,
                max_age_s=MAX_ODOMETRY_AGE_S,
                max_skew_s=MAX_ODOMETRY_SKEW_S,
            )
        except ValueError as error:
            self._warn_invalid_odometry(str(error))
            return
        self._positions = update.positions
        self._odometry_stamps_s = update.timestamps_s
        self._odometry_source_stamps[drone] = stamp
        distances = update.distances_m
        if distances is None:
            return
        output = StampedPairwiseRanges()
        output.header.stamp = self.get_clock().now().to_msg()
        output.sequence = self._pairwise_sequence
        output.drone_ids = list(self._drone_names)
        output.source_stamps = [
            self._odometry_source_stamps[name] for name in self._drone_names
        ]
        output.pair_labels = [f"{first}:{second}" for first, second, _ in distances]
        output.ranges_m = [distance for _, _, distance in distances]
        self._uwb_publisher.publish(output)
        self._pairwise_sequence += 1

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
