"""Tests for the topic isolation used by Gazebo failure probes."""

from __future__ import annotations

import unittest

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.bridge_config_model import (
    CF1_TRUTH_ODOMETRY_ROS_TOPIC,
    split_cf1_odometry_bridge,
)


class SplitCf1OdometryBridgeTests(unittest.TestCase):
    def test_splits_only_mission_odometry_and_adds_read_only_truth_topic(self) -> None:
        original = [
            {
                "ros_topic_name": "/clock",
                "gz_topic_name": "/clock",
                "direction": "GZ_TO_ROS",
            },
            {
                "ros_topic_name": "/cf1/cmd_vel",
                "gz_topic_name": "/cf1/gazebo/command/twist",
                "direction": "ROS_TO_GZ",
            },
            {
                "ros_topic_name": "/cf1/odom",
                "gz_topic_name": "/model/cf1/odometry",
                "ros_type_name": "nav_msgs/msg/Odometry",
                "gz_type_name": "gz.msgs.Odometry",
                "direction": "GZ_TO_ROS",
            },
        ]

        main, mission, truth = split_cf1_odometry_bridge(original)

        self.assertEqual(
            [entry["ros_topic_name"] for entry in main], ["/clock", "/cf1/cmd_vel"]
        )
        self.assertEqual(mission["ros_topic_name"], "/cf1/odom")
        self.assertEqual(truth["ros_topic_name"], CF1_TRUTH_ODOMETRY_ROS_TOPIC)
        self.assertEqual(truth["gz_topic_name"], mission["gz_topic_name"])
        self.assertEqual(truth["direction"], "GZ_TO_ROS")
        self.assertEqual(original[2]["ros_topic_name"], "/cf1/odom")

    def test_requires_exactly_one_well_formed_odometry_mapping(self) -> None:
        cases = (
            [],
            [
                {"ros_topic_name": "/cf1/odom"},
                {"ros_topic_name": "/cf1/odom"},
            ],
            [
                {
                    "ros_topic_name": "/cf1/odom",
                    "gz_topic_name": "/model/cf1/odometry",
                    "ros_type_name": "nav_msgs/msg/Odometry",
                    "gz_type_name": "gz.msgs.Odometry",
                    "direction": "ROS_TO_GZ",
                }
            ],
        )
        for entries in cases:
            with self.subTest(entries=entries), self.assertRaises(ValueError):
                split_cf1_odometry_bridge(entries)
