"""Fail-closed per-launch mapping-run claim tests."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model import (
    acquire_world_session_lock,
    claim_world_session,
    publish_world_session,
    session_file_for_partition,
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


if __name__ == "__main__":
    unittest.main()
