"""Hardware-free tests for the passive swarm command-graph guard."""

from __future__ import annotations

import unittest

from scripts.verify_gazebo_swarm_ros import validate_passive_command_graph


def _endpoints() -> dict[str, dict[str, list[str]]]:
    return {
        f"/{drone}/{topic}": {"publishers": [], "subscribers": ["ros_gz_bridge"]}
        for drone in ("cf1", "cf3", "cf4")
        for topic in ("cmd_vel", "enable")
    }


class PassiveCommandGraphTests(unittest.TestCase):
    def test_accepts_subscriber_only_control_topics_without_mission_node(self) -> None:
        result = validate_passive_command_graph(
            ["ros_gz_bridge", "nano_c_slam_sensor_bridge", "telemetry_probe"],
            _endpoints(),
        )

        self.assertEqual(result["mission_nodes"], [])
        self.assertIs(result["no_control_publishers"], True)

    def test_rejects_mission_node_or_any_control_publisher(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "mission nodes"):
            validate_passive_command_graph(["nano_c_slam_sim_mission"], _endpoints())

        endpoints = _endpoints()
        endpoints["/cf3/enable"]["publishers"] = ["unexpected_operator"]
        with self.assertRaisesRegex(RuntimeError, "flight-control publishers"):
            validate_passive_command_graph(["telemetry_probe"], endpoints)

    def test_rejects_incomplete_control_topic_inventory(self) -> None:
        endpoints = _endpoints()
        del endpoints["/cf4/cmd_vel"]
        with self.assertRaisesRegex(RuntimeError, "omitted control topics"):
            validate_passive_command_graph(["telemetry_probe"], endpoints)


if __name__ == "__main__":
    unittest.main()
