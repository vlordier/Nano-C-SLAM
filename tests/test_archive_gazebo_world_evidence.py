"""Tests for archiving exact SDF evidence from an active Gazebo session."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.archive_gazebo_world_evidence import archive_world_evidence
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.world_session_model import (
    publish_world_session,
    record_world_process,
)


class ArchiveGazeboWorldEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.session_path = self.root / "active-session.json"
        self.world_path = self.root / "nearfield.sdf"
        self.output_path = self.root / "evidence"
        self.world_path.write_bytes(b"<sdf><world name='nearfield'/></sdf>\n")
        publish_world_session(
            self.session_path,
            scene="nearfield-mapping",
            world_path=self.world_path,
        )
        record_world_process(
            self.session_path,
            role="gazebo-server",
            pid=os.getpid(),
            argv=["/usr/bin/gz", "sim", "-s", str(self.world_path)],
        )
        self.partition = "isolated-archive-test"
        self.session_path_patch = patch(
            "scripts.archive_gazebo_world_evidence.session_file_for_partition",
            return_value=self.session_path,
        )
        self.session_path_patch.start()
        self.addCleanup(self.session_path_patch.stop)

    def test_archives_exact_active_sdf_session_and_attestation(self) -> None:
        attestation = archive_world_evidence(self.partition, self.output_path)
        session_bytes = (self.output_path / "world-session.json").read_bytes()
        sdf_bytes = (self.output_path / "world.sdf").read_bytes()

        self.assertEqual(sdf_bytes, self.world_path.read_bytes())
        self.assertEqual(
            attestation["session_sha256"], hashlib.sha256(session_bytes).hexdigest()
        )
        self.assertEqual(
            attestation["archived_sdf_sha256"], hashlib.sha256(sdf_bytes).hexdigest()
        )
        self.assertTrue(attestation["verified_while_server_active"])
        self.assertEqual(
            json.loads((self.output_path / "world-input-attestation.json").read_text()),
            attestation,
        )

    def test_rejects_sdf_changed_after_session_creation(self) -> None:
        self.world_path.write_bytes(b"<sdf><world name='changed'/></sdf>\n")
        with self.assertRaisesRegex(ValueError, "differs from its world-session"):
            archive_world_evidence(self.partition, self.output_path)
        self.assertFalse(self.output_path.exists())

    def test_rejects_session_without_live_server(self) -> None:
        with (
            patch(
                "scripts.archive_gazebo_world_evidence.os.kill",
                side_effect=ProcessLookupError,
            ),
            self.assertRaisesRegex(ValueError, "no longer available"),
        ):
            archive_world_evidence(self.partition, self.output_path)
        self.assertFalse(self.output_path.exists())


if __name__ == "__main__":
    unittest.main()
