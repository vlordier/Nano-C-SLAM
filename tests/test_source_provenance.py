"""Temporary Git repositories exercise source provenance without project state."""

from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path

from scripts.source_provenance import (
    collect_source_identity,
    verify_source_identity,
)


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(  # noqa: S603 -- test invokes fixed Git argv without a shell
        ("git", "-C", str(root), *args),  # noqa: S607 -- Git is the required VCS
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _repository(root: Path, name: str = "main") -> Path:
    repository = root / name
    repository.mkdir()
    _git(repository, "init", "-q")
    _git(repository, "config", "user.email", "tests@example.invalid")
    _git(repository, "config", "user.name", "Provenance Tests")
    (repository / "tracked.txt").write_text("committed\n", encoding="utf-8")
    _git(repository, "add", "tracked.txt")
    _git(repository, "commit", "-qm", "initial")
    return repository


class SourceProvenanceTests(unittest.TestCase):
    def test_clean_snapshot_is_deterministic_and_verifiable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repository = _repository(Path(temporary))

            first = collect_source_identity(repository)
            second = collect_source_identity(repository)

            self.assertEqual(first, second)
            self.assertFalse(first.dirty)
            self.assertIsNone(first.dirty_patch_sha256)
            verify_source_identity(repository, first)

    def test_tracked_and_untracked_changes_are_included_in_dirty_identity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repository = _repository(Path(temporary))
            clean = collect_source_identity(repository)
            (repository / "tracked.txt").write_text("modified\n", encoding="utf-8")
            tracked_dirty = collect_source_identity(repository)
            (repository / "new-source.c").write_text("int value;\n", encoding="utf-8")
            untracked_dirty = collect_source_identity(repository)

            self.assertTrue(tracked_dirty.dirty)
            self.assertTrue(untracked_dirty.dirty)
            self.assertNotEqual(
                clean.dirty_patch_sha256, tracked_dirty.dirty_patch_sha256
            )
            self.assertNotEqual(
                tracked_dirty.dirty_patch_sha256, untracked_dirty.dirty_patch_sha256
            )
            with self.assertRaisesRegex(ValueError, "changed after snapshot"):
                verify_source_identity(repository, clean)

    def test_recursive_submodule_records_gitlink_checkout_and_dirty_patch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            child = _repository(root, "child")
            parent = _repository(root, "parent")
            _git(
                parent,
                "-c",
                "protocol.file.allow=always",
                "submodule",
                "add",
                "-q",
                str(child),
                "deps/child",
            )
            _git(parent, "commit", "-qam", "add submodule")

            snapshot = collect_source_identity(parent)
            self.assertEqual(len(snapshot.submodules), 1)
            self.assertEqual(snapshot.submodules[0].path, "deps/child")
            self.assertEqual(
                snapshot.submodules[0].gitlink_revision,
                snapshot.submodules[0].checkout_revision,
            )

            (parent / "deps/child/tracked.txt").write_text(
                "changed\n", encoding="utf-8"
            )
            dirty_snapshot = collect_source_identity(parent)
            submodule = dirty_snapshot.submodules[0]
            self.assertTrue(submodule.dirty)
            self.assertIsNotNone(submodule.dirty_patch_sha256)


if __name__ == "__main__":
    unittest.main()
