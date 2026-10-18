"""Runtime dependency capture tests use synthetic files and local Git history."""

from __future__ import annotations

import hashlib
import subprocess
import tempfile
import unittest
from pathlib import Path

from scripts.runtime_provenance import (
    capture_runtime_dependency,
    verify_mchan_build_receipt,
    verify_runtime_dependency,
)
from scripts.validation_provenance import DependencyDisposition


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(  # noqa: S603 -- test invokes fixed Git argv without a shell
        ("git", "-C", str(root), *args),  # noqa: S607 -- Git is the required VCS
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, content: bytes) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


class RuntimeProvenanceTests(unittest.TestCase):
    def test_unknown_dependency_stays_unknown_and_mutated_file_is_rejected(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            missing = capture_runtime_dependency(
                "optional-library",
                root / "missing.dylib",
                disposition=DependencyDisposition.IMMUTABLE_DOWNLOAD,
            )
            self.assertEqual(missing.disposition, DependencyDisposition.UNKNOWN)
            self.assertIsNone(missing.sha256)
            self.assertIsNone(missing.path)

            library = _write(root / "runtime.dylib", b"version one")
            captured = capture_runtime_dependency(
                "runtime-library",
                library,
                disposition=DependencyDisposition.IMMUTABLE_DOWNLOAD,
                version="1.0",
            )
            verify_runtime_dependency(captured, library)
            library.write_bytes(b"version two")
            with self.assertRaisesRegex(ValueError, "changed after capture"):
                verify_runtime_dependency(captured, library)

    def test_mchan_receipt_is_joined_to_pinned_sdk_and_actual_runtime_files(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sdk = root / "sdk"
            sdk.mkdir()
            _git(sdk, "init", "-q")
            _git(sdk, "config", "user.email", "tests@example.invalid")
            _git(sdk, "config", "user.name", "Runtime Tests")
            source = _write(
                sdk / "gvsoc/gvsoc_gap/gap/mchan/mchan_v7_impl.cpp", b"source code\n"
            )
            _git(sdk, "add", ".")
            _git(sdk, "commit", "-qm", "pinned source")
            revision = _git(sdk, "rev-parse", "HEAD")

            compiler = _write(root / "bin/c++", b"compiler identity")
            engine = _write(sdk / "build_gvsoc/engine/libpulpvp.dylib", b"engine")
            soc = _write(sdk / "install/workstation/models/gap9/soc.dylib", b"soc")
            model_root = root / "models"
            plugin = _write(
                model_root / "gap/mchan/mchan_v7_impl.dylib", b"rebuilt plugin"
            )
            receipt = root / "model-provenance.txt"
            receipt.write_text(
                "\n".join(
                    (
                        f"sdk_source_commit={revision}",
                        f"sdk_checkout_commit={revision}",
                        "sdk_worktree_dirty=false",
                        f"mchan_source_sha256={hashlib.sha256(source.read_bytes()).hexdigest()}",
                        f"compiler_path={compiler}",
                        "compiler_version=clang test",
                        f"compiler_sha256={_sha256(compiler)}",
                        f"engine_sha256={_sha256(engine)}",
                        f"soc_model_sha256={_sha256(soc)}",
                        f"plugin_sha256={_sha256(plugin)}",
                        f"model_root={model_root}",
                    )
                )
                + "\n",
                encoding="utf-8",
            )

            dependencies = verify_mchan_build_receipt(
                receipt,
                plugin=plugin,
                engine=engine,
                soc_model=soc,
                compiler=compiler,
                sdk_root=sdk,
                expected_source_revision=revision,
                observed_plugin_sha256=_sha256(plugin),
            )

            self.assertEqual(
                tuple(dependency.name for dependency in dependencies),
                ("gvsoc-mchan-plugin", "gvsoc-engine", "gap9-soc-model"),
            )
            plugin.write_bytes(b"replaced plugin")
            with self.assertRaisesRegex(ValueError, "does not match the captured file"):
                verify_mchan_build_receipt(
                    receipt,
                    plugin=plugin,
                    engine=engine,
                    soc_model=soc,
                    compiler=compiler,
                    sdk_root=sdk,
                    expected_source_revision=revision,
                    observed_plugin_sha256=_sha256(plugin),
                )


if __name__ == "__main__":
    unittest.main()
