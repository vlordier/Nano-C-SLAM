"""Behavioral tests for isolating the GVSOC working-directory alias."""

from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
LINK_SCRIPT = REPOSITORY_ROOT / "scripts/link_gvsoc_workdir.sh"


class GvsocWorkdirLinkTests(unittest.TestCase):
    def test_custom_link_leaves_existing_workflow_alias_unchanged(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            existing_work = root / "existing-work"
            existing_work.mkdir()
            existing_link = root / "gvsoc_workdir"
            existing_link.symlink_to(existing_work, target_is_directory=True)
            isolated_work = root / "isolated-work"
            isolated_link = root / "isolated-alias"

            result = subprocess.run(  # noqa: S603 -- fixed repository-owned helper
                [str(LINK_SCRIPT), str(isolated_work), str(isolated_link)],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(isolated_work.is_dir())
            self.assertEqual(isolated_link.resolve(), isolated_work.resolve())
            self.assertEqual(existing_link.resolve(), existing_work.resolve())

    def test_rejects_relative_or_identical_paths_without_creating_links(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            relative = subprocess.run(  # noqa: S603 -- fixed repository-owned helper
                [str(LINK_SCRIPT), "relative-work", str(root / "link")],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )
            identical = subprocess.run(  # noqa: S603 -- fixed repository-owned helper
                [str(LINK_SCRIPT), str(root), str(root)],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )

            self.assertEqual(relative.returncode, 2)
            self.assertEqual(identical.returncode, 2)
            self.assertFalse((root / "link").exists())


if __name__ == "__main__":
    unittest.main()
