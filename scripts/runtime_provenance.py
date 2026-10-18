"""Hash runtime artifacts and cross-check rebuilt GVSOC MCHAN receipts."""

from __future__ import annotations

import hashlib
import re
import subprocess
from pathlib import Path

from scripts.validation_provenance import (
    DependencyDisposition,
    RuntimeDependency,
)

_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def sha256_file_stable(path: Path) -> str:
    """Hash a regular file and reject replacement or mutation during the read."""
    resolved = path.resolve(strict=True)
    before = resolved.stat()
    if not resolved.is_file():
        raise ValueError(f"runtime artifact is not a regular file: {path}")
    digest = hashlib.sha256()
    with resolved.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    after = resolved.stat()
    if (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    ) != (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns):
        raise ValueError(f"runtime artifact changed while hashing: {path}")
    return digest.hexdigest()


def capture_runtime_dependency(
    name: str,
    path: Path | None,
    *,
    disposition: DependencyDisposition,
    version: str | None = None,
) -> RuntimeDependency:
    """Hash an observed runtime file, or retain an unavailable one as unknown."""
    if path is None or not path.exists():
        return RuntimeDependency(
            name=name,
            disposition=DependencyDisposition.UNKNOWN,
            version=version,
            sha256=None,
            path=None,
        )
    if disposition is DependencyDisposition.UNKNOWN:
        raise ValueError("observed file cannot use the unknown dependency disposition")
    return RuntimeDependency(
        name=name,
        disposition=disposition,
        version=version,
        sha256=sha256_file_stable(path),
        path=None,
    )


def verify_runtime_dependency(dependency: RuntimeDependency, path: Path) -> None:
    """Reject replacement of a previously captured runtime dependency."""
    if dependency.sha256 is None:
        raise ValueError(f"runtime dependency was not observed: {dependency.name}")
    if sha256_file_stable(path) != dependency.sha256:
        raise ValueError(f"runtime dependency changed after capture: {dependency.name}")


def _key_value_receipt(path: Path) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line or "=" not in line:
            raise ValueError(f"invalid MCHAN receipt line {line_number}")
        key, value = line.split("=", maxsplit=1)
        if not key or not value or key in fields:
            raise ValueError(
                f"invalid or duplicate MCHAN receipt key on line {line_number}"
            )
        fields[key] = value
    required = {
        "sdk_source_commit",
        "sdk_checkout_commit",
        "sdk_worktree_dirty",
        "mchan_source_sha256",
        "compiler_path",
        "compiler_version",
        "compiler_sha256",
        "engine_sha256",
        "soc_model_sha256",
        "plugin_sha256",
        "model_root",
    }
    if fields.keys() != required:
        raise ValueError("MCHAN receipt has missing or unexpected fields")
    for key in (
        "sdk_source_commit",
        "sdk_checkout_commit",
    ):
        if re.fullmatch(r"[0-9a-f]{40}", fields[key]) is None:
            raise ValueError(f"invalid MCHAN receipt revision: {key}")
    for key in (
        "mchan_source_sha256",
        "compiler_sha256",
        "engine_sha256",
        "soc_model_sha256",
        "plugin_sha256",
    ):
        if _SHA256.fullmatch(fields[key]) is None:
            raise ValueError(f"invalid MCHAN receipt digest: {key}")
    if fields["sdk_worktree_dirty"] not in {"true", "false"}:
        raise ValueError("invalid MCHAN receipt dirty flag")
    return fields


def verify_mchan_build_receipt(
    receipt_path: Path,
    *,
    plugin: Path,
    engine: Path,
    soc_model: Path,
    compiler: Path,
    sdk_root: Path,
    expected_source_revision: str,
    observed_plugin_sha256: str,
) -> tuple[RuntimeDependency, ...]:
    """Verify receipt hashes against runtime files and pinned Git source bytes."""
    receipt = _key_value_receipt(receipt_path)
    if (
        re.fullmatch(r"[0-9a-f]{40}", expected_source_revision) is None
        or receipt["sdk_source_commit"] != expected_source_revision
    ):
        raise ValueError("MCHAN receipt does not use the expected pinned SDK source")
    if _SHA256.fullmatch(observed_plugin_sha256) is None:
        raise ValueError("observed MCHAN plugin identity is not a SHA-256 digest")
    sdk_directory = sdk_root.resolve(strict=True)
    model_root = Path(receipt["model_root"]).resolve(strict=True)
    expected_paths = {
        "plugin": model_root / "gap/mchan/mchan_v7_impl.dylib",
        "engine": sdk_directory / "build_gvsoc/engine/libpulpvp.dylib",
        "soc_model": sdk_directory / "install/workstation/models/gap9/soc.dylib",
        "compiler": Path(receipt["compiler_path"]).resolve(strict=True),
    }
    supplied_paths = {
        "plugin": plugin.resolve(strict=True),
        "engine": engine.resolve(strict=True),
        "soc_model": soc_model.resolve(strict=True),
        "compiler": compiler.resolve(strict=True),
    }
    if supplied_paths != {
        name: path.resolve(strict=True) for name, path in expected_paths.items()
    }:
        raise ValueError(
            "supplied GVSOC files do not match the receipt's runtime paths"
        )
    actual = {
        "plugin_sha256": sha256_file_stable(plugin),
        "engine_sha256": sha256_file_stable(engine),
        "soc_model_sha256": sha256_file_stable(soc_model),
        "compiler_sha256": sha256_file_stable(compiler),
    }
    for key, digest in actual.items():
        if receipt[key] != digest:
            raise ValueError(f"MCHAN receipt {key} does not match the captured file")
    if observed_plugin_sha256 != actual["plugin_sha256"]:
        raise ValueError("GVSOC run report plugin hash differs from the captured file")
    source = subprocess.run(  # noqa: S603 -- fixed Git argv; never invoke a shell
        (  # noqa: S607 -- Git is the required VCS
            "git",
            "-C",
            str(sdk_root.resolve(strict=True)),
            "show",
            f"{expected_source_revision}:gvsoc/gvsoc_gap/gap/mchan/mchan_v7_impl.cpp",
        ),
        check=False,
        capture_output=True,
    )
    if source.returncode:
        raise ValueError("pinned MCHAN source is unavailable from the SDK repository")
    source_hash = hashlib.sha256(source.stdout).hexdigest()
    if source_hash != receipt["mchan_source_sha256"]:
        raise ValueError("MCHAN receipt source hash does not match pinned Git content")
    return (
        RuntimeDependency(
            name="gvsoc-mchan-plugin",
            disposition=DependencyDisposition.BUILT_FROM_SOURCE,
            version=expected_source_revision,
            sha256=actual["plugin_sha256"],
            path=None,
        ),
        RuntimeDependency(
            name="gvsoc-engine",
            disposition=DependencyDisposition.SYSTEM_UNPINNED,
            version=None,
            sha256=actual["engine_sha256"],
            path=None,
        ),
        RuntimeDependency(
            name="gap9-soc-model",
            disposition=DependencyDisposition.SYSTEM_UNPINNED,
            version=None,
            sha256=actual["soc_model_sha256"],
            path=None,
        ),
    )


GAZEBO_ROS_ENVIRONMENT = (
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "GZ_PARTITION",
    "GZ_SIM_RESOURCE_PATH",
    "GZ_SIM_SYSTEM_PLUGIN_PATH",
    "RMW_IMPLEMENTATION",
    "ROS_DOMAIN_ID",
    "ROS_DISTRO",
    "ROS_PACKAGE_PATH",
    "ROS_VERSION",
    "ROS_WORKSPACE",
)
