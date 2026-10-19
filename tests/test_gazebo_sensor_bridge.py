"""Hardware-free tests for odometry validation at the ROS bridge boundary."""

from __future__ import annotations

import importlib
import sys
import types
import unittest
from unittest.mock import patch

from test_support.ros_fixtures import odometry_message


class _Publisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class _Logger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class _Clock:
    def __init__(self) -> None:
        self.nanoseconds = 10_000_000_000

    def now(self) -> types.SimpleNamespace:
        return types.SimpleNamespace(
            nanoseconds=self.nanoseconds,
            to_msg=lambda: types.SimpleNamespace(
                sec=self.nanoseconds // 1_000_000_000,
                nanosec=self.nanoseconds % 1_000_000_000,
            ),
        )


class SensorBridgeOdometryTests(unittest.TestCase):
    def setUp(self) -> None:
        rclpy = types.ModuleType("rclpy")
        node_module = types.ModuleType("rclpy.node")
        node_module.Node = type("Node", (), {})
        interface = types.ModuleType("nano_c_slam_interfaces")
        interface.__path__ = []
        interface_msg = types.ModuleType("nano_c_slam_interfaces.msg")
        interface_msg.StampedRanges = type("StampedRanges", (), {})

        class StampedPairwiseRanges:
            def __init__(self) -> None:
                self.header = types.SimpleNamespace(stamp=None)

        interface_msg.StampedPairwiseRanges = StampedPairwiseRanges
        nav = types.ModuleType("nav_msgs")
        nav.__path__ = []
        nav_msg = types.ModuleType("nav_msgs.msg")
        nav_msg.Odometry = type("Odometry", (), {})
        sensor = types.ModuleType("sensor_msgs")
        sensor.__path__ = []
        sensor_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msg.LaserScan = type("LaserScan", (), {})

        self.modules = {
            "rclpy": rclpy,
            "rclpy.node": node_module,
            "nano_c_slam_interfaces": interface,
            "nano_c_slam_interfaces.msg": interface_msg,
            "nav_msgs": nav,
            "nav_msgs.msg": nav_msg,
            "sensor_msgs": sensor,
            "sensor_msgs.msg": sensor_msg,
        }
        for name, module in (
            ("nano_c_slam_interfaces", interface_msg),
            ("nav_msgs", nav_msg),
            ("sensor_msgs", sensor_msg),
        ):
            self.modules[name].msg = module
        self.stack = patch.dict(sys.modules, self.modules)
        self.stack.start()
        module_name = "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.sensor_bridge"
        sys.modules.pop(module_name, None)
        self.module = importlib.import_module(module_name)

    def tearDown(self) -> None:
        self.stack.stop()
        sys.modules.pop(
            "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.sensor_bridge", None
        )

    def _bridge(self) -> object:
        bridge = object.__new__(self.module.SensorBridge)
        bridge._drone_names = ("cf1", "cf3")
        bridge._positions = {"cf1": (0.0, 0.0, 0.0)}
        bridge._odometry_stamps_s = {"cf1": 10.0}
        bridge._odometry_source_stamps = {
            "cf1": types.SimpleNamespace(sec=10, nanosec=0)
        }
        bridge._deferred_odometry = {}
        bridge._pairwise_sequence = 0
        bridge._uwb_publisher = _Publisher()
        bridge._last_odometry_warning_s = float("-inf")
        clock = _Clock()
        bridge.get_clock = lambda: clock
        bridge._test_clock = clock
        bridge.get_logger = lambda: logger
        logger = _Logger()
        return bridge

    @staticmethod
    def _message(x: float, *, sec: int = 10, nanosec: int = 0) -> object:
        return odometry_message(x=x, sec=sec, nanosec=nanosec)

    def test_invalid_sample_does_not_poison_cache_and_later_sample_recovers(
        self,
    ) -> None:
        bridge = self._bridge()

        bridge._odom_callback(self._message(float("nan")), "cf3")

        self.assertEqual(bridge._positions, {"cf1": (0.0, 0.0, 0.0)})
        self.assertEqual(bridge._odometry_stamps_s, {"cf1": 10.0})
        self.assertEqual(bridge._uwb_publisher.messages, [])

        bridge._odom_callback(self._message(3.0), "cf3")

        self.assertEqual(len(bridge._uwb_publisher.messages), 1)
        message = bridge._uwb_publisher.messages[0]
        self.assertEqual(message.ranges_m, [3.0])
        self.assertEqual(message.sequence, 0)
        self.assertEqual(message.drone_ids, ["cf1", "cf3"])
        self.assertEqual(message.pair_labels, ["cf1:cf3"])
        self.assertEqual(
            [(stamp.sec, stamp.nanosec) for stamp in message.source_stamps],
            [(10, 0), (10, 0)],
        )
        self.assertEqual(bridge._pairwise_sequence, 1)

    def test_bad_stamp_regression_and_unrepresentable_range_never_publish(self) -> None:
        bridge = self._bridge()

        bridge._odom_callback(self._message(1.0, nanosec=1_000_000_000), "cf3")
        bridge._odom_callback(self._message(1.0, sec=9), "cf1")
        bridge._odom_callback(self._message(3.5e38), "cf3")

        self.assertEqual(bridge._uwb_publisher.messages, [])
        self.assertEqual(bridge._positions, {"cf1": (0.0, 0.0, 0.0)})
        self.assertEqual(bridge._odometry_stamps_s, {"cf1": 10.0})

    def test_bounded_future_sample_waits_for_clock_then_publishes(self) -> None:
        bridge = self._bridge()

        bridge._odom_callback(self._message(3.0, nanosec=50_000_000), "cf3")

        self.assertEqual(bridge._positions, {"cf1": (0.0, 0.0, 0.0)})
        self.assertEqual(bridge._uwb_publisher.messages, [])
        self.assertIn("cf3", bridge._deferred_odometry)

        bridge._test_clock.nanoseconds = 10_050_000_000
        bridge._clock_callback()

        self.assertEqual(len(bridge._uwb_publisher.messages), 1)
        message = bridge._uwb_publisher.messages[0]
        self.assertEqual(message.ranges_m, [3.0])
        self.assertEqual(
            [(stamp.sec, stamp.nanosec) for stamp in message.source_stamps],
            [(10, 0), (10, 50_000_000)],
        )
        self.assertEqual(bridge._deferred_odometry, {})


if __name__ == "__main__":
    unittest.main()
