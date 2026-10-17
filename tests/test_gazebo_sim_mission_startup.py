"""Hardware-free tests for the ROS mission's startup safety policy."""

from __future__ import annotations

import importlib
import sys
import time
import types
import unittest
from unittest.mock import patch


class _Publisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class _Clock:
    class _Now:
        nanoseconds = 0

    def now(self) -> _Now:
        value = self._Now()
        value.nanoseconds = 1_000_000_000
        return value


class _Logger:
    def info(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


class _Node:
    def __init__(self, _name: str) -> None:
        self.parameters: dict[str, object] = {}
        self.publishers: list[_Publisher] = []
        self.subscriptions: list[object] = []
        self.timers: list[types.SimpleNamespace] = []
        self._logger = _Logger()

    def declare_parameter(self, name: str, default: object) -> None:
        self.parameters[name] = default

    def get_parameter(self, name: str) -> types.SimpleNamespace:
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *_args: object, **_kwargs: object) -> _Publisher:
        publisher = _Publisher()
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, *_args: object, **_kwargs: object) -> object:
        subscription = types.SimpleNamespace(callback=_args[2])
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(
        self, _period: float, callback: object, **kwargs: object
    ) -> types.SimpleNamespace:
        timer = types.SimpleNamespace(
            callback=callback,
            clock=kwargs.get("clock"),
            canceled=False,
        )
        timer.cancel = lambda: setattr(timer, "canceled", True)
        self.timers.append(timer)
        return timer

    def get_clock(self) -> _Clock:
        return _Clock()

    def get_logger(self) -> _Logger:
        return self._logger

    def destroy_node(self) -> None:
        pass


class SimMissionStartupTests(unittest.TestCase):
    def setUp(self) -> None:
        self.rclpy = types.ModuleType("rclpy")
        self.rclpy.ok = lambda: True
        self.geometry = types.ModuleType("geometry_msgs.msg")
        self.geometry.Twist = type(
            "Twist",
            (),
            {
                "__init__": lambda self: setattr(
                    self, "linear", types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                )
            },
        )
        self.nav = types.ModuleType("nav_msgs.msg")
        self.nav.Odometry = type("Odometry", (), {})
        self.rcl_node = types.ModuleType("rclpy.node")
        self.rcl_node.Node = _Node
        self.rcl_clock = types.ModuleType("rclpy.clock")
        self.rcl_clock.Clock = lambda **kwargs: types.SimpleNamespace(**kwargs)
        self.rcl_clock_type = types.ModuleType("rclpy.clock_type")
        self.rcl_clock_type.ClockType = types.SimpleNamespace(STEADY_TIME="steady")
        self.rcl_qos = types.ModuleType("rclpy.qos")
        self.rcl_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
        self.rcl_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
        self.rcl_qos.QoSProfile = lambda **_kwargs: object()
        self.std = types.ModuleType("std_msgs.msg")
        self.std.Bool = type("Bool", (), {})
        self.std.String = type("String", (), {})
        self.modules = {
            "rclpy": self.rclpy,
            "rclpy.node": self.rcl_node,
            "rclpy.clock": self.rcl_clock,
            "rclpy.clock_type": self.rcl_clock_type,
            "rclpy.qos": self.rcl_qos,
            "geometry_msgs": types.ModuleType("geometry_msgs"),
            "geometry_msgs.msg": self.geometry,
            "nav_msgs": types.ModuleType("nav_msgs"),
            "nav_msgs.msg": self.nav,
            "std_msgs": types.ModuleType("std_msgs"),
            "std_msgs.msg": self.std,
        }
        for parent, child in (
            ("geometry_msgs", self.geometry),
            ("nav_msgs", self.nav),
            ("std_msgs", self.std),
        ):
            self.modules[parent].msg = child
        for package in ("rclpy", "geometry_msgs", "nav_msgs", "std_msgs"):
            self.modules[package].__path__ = []
        self.rclpy.clock = self.rcl_clock
        self.rclpy.clock_type = self.rcl_clock_type
        self.stack = patch.dict(sys.modules, self.modules)
        self.stack.start()
        sys.modules.pop(
            "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.sim_mission", None
        )
        self.module = importlib.import_module(
            "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.sim_mission"
        )

    def tearDown(self) -> None:
        self.stack.stop()
        sys.modules.pop(
            "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.sim_mission", None
        )

    def test_missing_odometry_fails_after_wall_clock_deadline_without_takeoff(
        self,
    ) -> None:
        node = self.module.SimMission()
        node._odometry_startup_timeout_s = 0.1
        node._startup_started_wall_s = time.monotonic() - 0.2

        self.assertEqual(node.timers[1].clock.clock_type, "steady")
        node.timers[1].callback()

        self.assertEqual(node._sequence.phase.name, "FAILED")
        self.assertEqual(
            node._sequence.failure_reason,
            "odometry was not received before startup timeout",
        )
        self.assertEqual(node._publisher.messages[-1].linear.z, 0.0)
        self.assertEqual(node._enable_publisher.messages, [])
        self.assertIn('"phase": "FAILED"', node._status_publisher.messages[-1].data)
        self.assertEqual(self.module.mission_exit_code(node._sequence.phase), 1)

    def test_received_odometry_disables_startup_timeout(self) -> None:
        node = self.module.SimMission()
        node._odometry_startup_timeout_s = 0.1
        node._startup_started_wall_s = time.monotonic() - 1.0
        message = self.nav.Odometry()
        message.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            )
        )
        message.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=1, nanosec=0)
        )
        self.assertEqual(node._odometry_message_count, 0)
        node.subscriptions[0].callback(message)
        self.assertTrue(node.timers[1].canceled)

        node._step()

        self.assertEqual(node._sequence.phase.name, "TAKEOFF")
        self.assertEqual(node._odometry_message_count, 1)
        self.assertEqual(node._publisher.messages[-1].linear.z, 0.5)

    def test_invalid_first_odometry_fails_without_satisfying_readiness(self) -> None:
        node = self.module.SimMission()
        message = self.nav.Odometry()
        message.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=float("nan"))
            )
        )
        message.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=1, nanosec=0)
        )

        node.subscriptions[0].callback(message)
        node._step()

        self.assertEqual(node._sequence.phase.name, "FAILED")
        self.assertEqual(node._odometry_message_count, 0)
        self.assertIsNone(node._altitude_m)
        self.assertTrue(node.timers[1].canceled)
        self.assertEqual(node._enable_publisher.messages, [])
        self.assertEqual(node._publisher.messages[-1].linear.z, 0.0)

    def test_mapping_mission_rejects_airborne_start_before_enabling(self) -> None:
        node = self.module.SimMission()
        node.get_clock().now().nanoseconds = 1_000_000_000
        node._mapping_path_enabled = True
        message = self.nav.Odometry()
        message.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=1.0, y=2.0, z=0.75)
            )
        )
        message.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=1, nanosec=0)
        )
        node.subscriptions[0].callback(message)

        node._step()

        self.assertEqual(node._sequence.phase.name, "FAILED")
        self.assertEqual(node._enable_publisher.messages, [])
        self.assertEqual(node._publisher.messages[-1].linear.z, 0.0)
        self.assertIn('"phase": "FAILED"', node._status_publisher.messages[-1].data)
        self.assertEqual(self.module.mission_exit_code(node._sequence.phase), 1)

    def test_grounded_mapping_mission_keeps_takeoff_behavior(self) -> None:
        node = self.module.SimMission()
        node.get_clock().now().nanoseconds = 1_000_000_000
        node._mapping_path_enabled = True
        message = self.nav.Odometry()
        message.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=1.0, y=2.0, z=0.1)
            )
        )
        message.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=1, nanosec=0)
        )
        node.subscriptions[0].callback(message)

        node._step()

        self.assertEqual(node._sequence.phase.name, "TAKEOFF")
        self.assertEqual(len(node._enable_publisher.messages), 1)
        self.assertEqual(node._publisher.messages[-1].linear.z, 0.5)

    def test_teardown_skips_ros_publish_when_context_is_invalid(self) -> None:
        node = self.module.SimMission()
        before = len(node._publisher.messages)
        self.rclpy.ok = lambda: False

        node.destroy_node()

        self.assertEqual(len(node._publisher.messages), before)
