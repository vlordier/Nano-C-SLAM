"""Collect stable Git source identities, including recursive submodules."""

from __future__ import annotations

import hashlib
import re
import stat
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import cast

from scripts.validation_provenance import SourceIdentity, SubmoduleIdentity

_SUBMODULE_LINE = re.compile(r"^([ +-U])([0-9a-f]{40,64}) (.+)$")


@dataclass(frozen=True)
class _RepositoryState:
    revision: str
    dirty: bool
    patch_sha256: str | None


def _git(root: Path, *args: str, binary: bool = False) -> str | bytes:
    result = subprocess.run(  # noqa: S603 -- fixed Git executable; argv never uses a shell
        ("git", "-C", str(root), *args),  # noqa: S607 -- Git is the required VCS
        check=False,
        capture_output=True,
        text=not binary,
    )
    if result.returncode:
        stderr = (
            cast(bytes, result.stderr).decode(errors="replace")
            if binary
            else cast(str, result.stderr)
        )
        raise ValueError(f"git {' '.join(args)} failed in {root}: {stderr.strip()}")
    return cast(bytes, result.stdout) if binary else cast(str, result.stdout)


def _repository_state(root: Path) -> _RepositoryState:
    revision = str(_git(root, "rev-parse", "HEAD")).strip()
    if re.fullmatch(r"[0-9a-f]{40,64}", revision) is None:
        raise ValueError(f"invalid Git revision in {root}")
    tracked = _git(root, "diff", "--binary", "HEAD", "--", binary=True)
    untracked_paths = _git(
        root, "ls-files", "--others", "--exclude-standard", "-z", binary=True
    )
    if not isinstance(tracked, bytes) or not isinstance(untracked_paths, bytes):
        raise TypeError("binary Git output was not returned as bytes")
    digest = hashlib.sha256()
    dirty = bool(tracked)
    if tracked:
        digest.update(b"tracked\0")
        digest.update(tracked)
    for raw_path in sorted(filter(None, untracked_paths.split(b"\0"))):
        relative_path = Path(raw_path.decode("utf-8", errors="strict"))
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise ValueError("Git reported an unsafe untracked path")
        file_path = root / relative_path
        metadata = file_path.lstat()
        if stat.S_ISREG(metadata.st_mode):
            content = file_path.read_bytes()
        elif stat.S_ISLNK(metadata.st_mode):
            content = file_path.readlink().as_posix().encode("utf-8")
        else:
            raise ValueError(
                f"untracked source is not a file or symlink: {relative_path}"
            )
        dirty = True
        digest.update(b"untracked\0")
        digest.update(raw_path)
        digest.update(b"\0")
        digest.update(f"{stat.S_IMODE(metadata.st_mode):o}".encode("ascii"))
        digest.update(b"\0")
        digest.update(hashlib.sha256(content).digest())
    return _RepositoryState(
        revision=revision,
        dirty=dirty,
        patch_sha256=digest.hexdigest() if dirty else None,
    )


def _submodules(root: Path) -> tuple[SubmoduleIdentity, ...]:
    output = str(_git(root, "submodule", "status", "--recursive"))
    records: list[SubmoduleIdentity] = []
    for line in output.splitlines():
        match = _SUBMODULE_LINE.fullmatch(line)
        if match is None:
            raise ValueError(f"unrecognized recursive submodule status: {line!r}")
        marker, gitlink_revision, raw_path = match.groups()
        path = raw_path.split(" (", maxsplit=1)[0]
        relative = Path(path)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"unsafe submodule path: {path}")
        checkout = (root / relative).resolve(strict=True)
        state = _repository_state(checkout)
        if marker == "-":
            raise ValueError(f"submodule is not initialized: {path}")
        records.append(
            SubmoduleIdentity(
                path=relative.as_posix(),
                gitlink_revision=gitlink_revision,
                checkout_revision=state.revision,
                dirty=state.dirty,
                dirty_patch_sha256=state.patch_sha256,
            )
        )
    return tuple(records)


def collect_source_identity(root: Path) -> SourceIdentity:
    """Read a stable root/submodule snapshot; fail if it changes during capture."""
    repository = root.resolve(strict=True)
    before = _repository_state(repository)
    submodules_before = _submodules(repository)
    after = _repository_state(repository)
    submodules_after = _submodules(repository)
    if before != after or submodules_before != submodules_after:
        raise ValueError("repository or submodule source changed during snapshot")
    return SourceIdentity(
        revision=before.revision,
        dirty=before.dirty,
        dirty_patch_sha256=before.patch_sha256,
        submodules=submodules_before,
    )


def verify_source_identity(root: Path, expected: SourceIdentity) -> None:
    """Reject source changes after the caller's snapshot was collected."""
    if collect_source_identity(root) != expected:
        raise ValueError("repository or submodule source changed after snapshot")
