"""Hardware-free tests for odometry validation at the ROS bridge boundary."""

from __future__ import annotations

import importlib
import sys
import types
import unittest
from unittest.mock import patch


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
    def now(self) -> types.SimpleNamespace:
        return types.SimpleNamespace(nanoseconds=10_000_000_000)


class SensorBridgeOdometryTests(unittest.TestCase):
    def setUp(self) -> None:
        rclpy = types.ModuleType("rclpy")
        node_module = types.ModuleType("rclpy.node")
        node_module.Node = type("Node", (), {})
        interface = types.ModuleType("nano_c_slam_interfaces")
        interface.__path__ = []
        interface_msg = types.ModuleType("nano_c_slam_interfaces.msg")
        interface_msg.StampedRanges = type("StampedRanges", (), {})
        nav = types.ModuleType("nav_msgs")
        nav.__path__ = []
        nav_msg = types.ModuleType("nav_msgs.msg")
        nav_msg.Odometry = type("Odometry", (), {})
        sensor = types.ModuleType("sensor_msgs")
        sensor.__path__ = []
        sensor_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msg.LaserScan = type("LaserScan", (), {})
        std = types.ModuleType("std_msgs")
        std.__path__ = []
        std_msg = types.ModuleType("std_msgs.msg")

        class Float32MultiArray:
            def __init__(self) -> None:
                self.layout = types.SimpleNamespace(dim=[])
                self.data: list[float] = []

        class MultiArrayDimension:
            def __init__(self, **kwargs: object) -> None:
                self.__dict__.update(kwargs)

        std_msg.Float32MultiArray = Float32MultiArray
        std_msg.MultiArrayDimension = MultiArrayDimension
        self.modules = {
            "rclpy": rclpy,
            "rclpy.node": node_module,
            "nano_c_slam_interfaces": interface,
            "nano_c_slam_interfaces.msg": interface_msg,
            "nav_msgs": nav,
            "nav_msgs.msg": nav_msg,
            "sensor_msgs": sensor,
            "sensor_msgs.msg": sensor_msg,
            "std_msgs": std,
            "std_msgs.msg": std_msg,
        }
        for name, module in (
            ("nano_c_slam_interfaces", interface_msg),
            ("nav_msgs", nav_msg),
            ("sensor_msgs", sensor_msg),
            ("std_msgs", std_msg),
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
        bridge._uwb_publisher = _Publisher()
        bridge._last_odometry_warning_s = float("-inf")
        bridge.get_clock = lambda: _Clock()
        bridge.get_logger = lambda: logger
        logger = _Logger()
        return bridge

    @staticmethod
    def _message(x: float, *, sec: int = 10, nanosec: int = 0) -> object:
        position = types.SimpleNamespace(x=x, y=0.0, z=0.0)
        return types.SimpleNamespace(
            pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=position)),
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec)
            ),
        )

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
        self.assertEqual(bridge._uwb_publisher.messages[0].data, [3.0])
        self.assertEqual(
            bridge._uwb_publisher.messages[0].layout.dim[0].label, "cf1:cf3"
        )

    def test_bad_stamp_regression_and_unrepresentable_range_never_publish(self) -> None:
        bridge = self._bridge()

        bridge._odom_callback(self._message(1.0, nanosec=1_000_000_000), "cf3")
        bridge._odom_callback(self._message(1.0, sec=9), "cf1")
        bridge._odom_callback(self._message(3.5e38), "cf3")

        self.assertEqual(bridge._uwb_publisher.messages, [])
        self.assertEqual(bridge._positions, {"cf1": (0.0, 0.0, 0.0)})
        self.assertEqual(bridge._odometry_stamps_s, {"cf1": 10.0})


if __name__ == "__main__":
    unittest.main()
