"""Tests for safely selecting optional Gazebo physics engines."""

from __future__ import annotations

import unittest

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.physics_engine_model import (
    gazebo_server_arguments,
)


class GazeboPhysicsEngineModelTests(unittest.TestCase):
    def test_default_arguments_select_force_capable_featherstone(self) -> None:
        self.assertEqual(
            gazebo_server_arguments(),
            " -r -s --headless-rendering --render-engine-server=ogre2 "
            "--render-engine-server-api-backend=metal "
            "--physics-engine gz-physics-bullet-featherstone-plugin",
        )
        self.assertEqual(gazebo_server_arguments(" "), gazebo_server_arguments())

    def test_explicit_plugin_name_is_forwarded(self) -> None:
        self.assertEqual(
            gazebo_server_arguments("nano-c-slam-dartsim-plugin"),
            " -r -s --headless-rendering --render-engine-server=ogre2 "
            "--render-engine-server-api-backend=metal "
            "--physics-engine nano-c-slam-dartsim-plugin",
        )

    def test_non_plugin_argument_is_rejected(self) -> None:
        for invalid_name in ("plugin name", "--help", "foo;touch", "foo/bar"):
            with self.subTest(invalid_name=invalid_name), self.assertRaises(ValueError):
                gazebo_server_arguments(invalid_name)


if __name__ == "__main__":
    unittest.main()
