"""Pure tests for passive Gazebo ROS odometry diagnostics."""

from __future__ import annotations

import importlib.util
import types
import unittest
from pathlib import Path

_SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts/diagnose_gazebo_odom.py"
_SPEC = importlib.util.spec_from_file_location("diagnose_gazebo_odom", _SCRIPT_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


class GazeboOdomDiagnosticTests(unittest.TestCase):
    def test_converts_ros_publisher_qos_endpoint_to_json_fields(self) -> None:
        endpoint = types.SimpleNamespace(
            node_name="ros_gz_bridge",
            node_namespace="/",
            topic_type="nav_msgs/msg/Odometry",
            qos_profile=types.SimpleNamespace(
                reliability=1,
                durability=2,
                history=3,
                depth=10,
            ),
        )

        record = _MODULE._endpoint_record(endpoint)

        self.assertEqual(record["node_name"], "ros_gz_bridge")
        self.assertEqual(record["topic_type"], "nav_msgs/msg/Odometry")
        self.assertEqual(record["qos"]["depth"], 10)
        self.assertEqual(record["qos"]["reliability"], "1")

    def test_converts_ros_stamp_without_float_precision_loss(self) -> None:
        stamp = types.SimpleNamespace(sec=1_700_000_000, nanosec=123_456_789)

        self.assertEqual(_MODULE._stamp_ns(stamp), 1_700_000_000_123_456_789)


if __name__ == "__main__":
    unittest.main()
