"""Fail-closed per-launch mapping-run claim tests."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model import (
    acquire_world_session_lock,
    claim_world_session,
    publish_world_session,
    record_world_process_event,
    session_file_for_partition,
    world_process_role,
)


class WorldSessionTests(unittest.TestCase):
    def test_partition_lock_rejects_concurrent_launch_and_releases_on_close(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "session.json"
            first_lock = acquire_world_session_lock(path)
            with self.assertRaisesRegex(RuntimeError, "already owns this partition"):
                acquire_world_session_lock(path)
            first_lock.close()
            second_lock = acquire_world_session_lock(path)
            second_lock.close()

    def test_partition_paths_are_stable_and_do_not_expose_partition(self) -> None:
        first = session_file_for_partition("nano_c_slam_501")
        self.assertEqual(first, session_file_for_partition("nano_c_slam_501"))
        self.assertNotIn("nano_c_slam_501", str(first))
        with self.assertRaises(ValueError):
            session_file_for_partition("")

    def test_missing_session_fails_with_launch_instruction(self) -> None:
        with (
            tempfile.TemporaryDirectory() as temporary_directory,
            self.assertRaisesRegex(RuntimeError, "start the world"),
        ):
            claim_world_session(
                Path(temporary_directory) / "missing.json",
                scene="nearfield-mapping",
                run_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            )

    def test_scene_mismatch_and_invalid_manifest_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "session.json"
            publish_world_session(path, scene="default")
            with self.assertRaisesRegex(RuntimeError, "another scene"):
                claim_world_session(
                    path,
                    scene="nearfield-mapping",
                    run_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
                )
            path.write_text(
                '{"schema_version": 1, "session_id": "bad", "scene": "default"}'
            )
            with self.assertRaisesRegex(RuntimeError, "invalid identity"):
                claim_world_session(
                    path,
                    scene="default",
                    run_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
                )

    def test_each_launch_allows_one_run_and_new_launch_rotates_identity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "session.json"
            first_session = publish_world_session(path, scene="nearfield-mapping")
            claimed = claim_world_session(
                path,
                scene="nearfield-mapping",
                run_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            )
            self.assertEqual(claimed, first_session)
            with self.assertRaisesRegex(
                RuntimeError, "already has a mapping-run claim"
            ):
                claim_world_session(
                    path,
                    scene="nearfield-mapping",
                    run_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
                )

            second_session = publish_world_session(path, scene="nearfield-mapping")
            self.assertNotEqual(second_session, first_session)
            self.assertEqual(
                claim_world_session(
                    path,
                    scene="nearfield-mapping",
                    run_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
                ),
                second_session,
            )

    def test_launch_process_events_record_only_known_runtime_roles(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "session.json"
            session_id = publish_world_session(path, scene="nearfield-mapping")
            server = SimpleNamespace(
                pid=1234,
                cmd=["/opt/gz-macos/bin/gz", "sim", "world.sdf", "-s"],
            )
            gui = SimpleNamespace(pid=1235, cmd=["/opt/gz/bin/gz", "sim", "-g"])
            other = SimpleNamespace(pid=1236, cmd=["/usr/bin/python", "unrelated.py"])
            with patch(
                "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model.shutil.which",
                return_value=None,
            ):
                self.assertEqual(
                    record_world_process_event(path, pid=server.pid, argv=server.cmd),
                    "gazebo-server",
                )
                self.assertIsNone(
                    record_world_process_event(path, pid=gui.pid, argv=gui.cmd)
                )
                self.assertIsNone(
                    record_world_process_event(path, pid=other.pid, argv=other.cmd)
                )

            manifest = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["session_id"], session_id)
            self.assertEqual(
                manifest["processes"],
                [
                    {
                        "role": "gazebo-server",
                        "pid": 1234,
                        "launch_pid": manifest["launch_pid"],
                        "argv": server.cmd,
                        "executable_path": None,
                        "observed_at_utc": manifest["processes"][0]["observed_at_utc"],
                    }
                ],
            )
            self.assertTrue(manifest["processes"][0]["observed_at_utc"].endswith("Z"))

    def test_process_recording_is_idempotent_and_preserves_run_claim(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "session.json"
            session_id = publish_world_session(path, scene="nearfield-mapping")
            claim_world_session(
                path,
                scene="nearfield-mapping",
                run_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            )
            command = ["gz", "sim", "world.sdf", "-s"]
            with patch(
                "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model.shutil.which",
                return_value=None,
            ):
                record_world_process_event(path, pid=4321, argv=command)
                record_world_process_event(path, pid=4321, argv=command)

            manifest = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["session_id"], session_id)
            self.assertEqual(len(manifest["processes"]), 1)
            self.assertEqual(
                manifest["processes"][0]["launch_pid"], manifest["launch_pid"]
            )
            with self.assertRaisesRegex(
                RuntimeError, "already has a mapping-run claim"
            ):
                claim_world_session(
                    path,
                    scene="nearfield-mapping",
                    run_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
                )

    def test_ros_bridge_process_roles_are_classified(self) -> None:
        self.assertEqual(
            world_process_role(["gz", "sim", "world.sdf", "-s"]),
            "gazebo-server",
        )
        self.assertEqual(
            world_process_role(
                ["/usr/bin/ruby", "/opt/gz-macos/bin/gz", "sim", "world.sdf", "-s"]
            ),
            "gazebo-server",
        )
        self.assertIsNone(
            world_process_role(["/usr/bin/ruby", "/opt/gz-macos/bin/gz", "sim", "-g"])
        )
        self.assertEqual(
            world_process_role(["/opt/ros/bin/parameter_bridge", "--ros-args"]),
            "ros-gazebo-bridge",
        )
        self.assertEqual(
            world_process_role(
                ["python", "/overlay/lib/nano_c_slam_gazebo/sensor_bridge"]
            ),
            "nano-sensor-bridge",
        )

    def test_conflicting_process_identity_and_invalid_pid_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "session.json"
            publish_world_session(path, scene="default")
            command = ["gz", "sim", "world.sdf", "-s"]
            with patch(
                "sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model.shutil.which",
                return_value=None,
            ):
                record_world_process_event(path, pid=99, argv=command)
                with self.assertRaisesRegex(RuntimeError, "conflicting data"):
                    record_world_process_event(
                        path,
                        pid=99,
                        argv=["gz", "sim", "other.sdf", "-s"],
                    )
                with self.assertRaisesRegex(ValueError, "positive"):
                    record_world_process_event(path, pid=0, argv=command)


if __name__ == "__main__":
    unittest.main()
