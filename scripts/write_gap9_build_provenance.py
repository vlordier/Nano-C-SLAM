"""Record immutable inputs emitted by a GAP9 GVSOC target build."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from scripts.source_provenance import (
    collect_source_identity,
    source_identity_from_mapping,
    source_identity_to_mapping,
    verify_source_identity,
)


def _strict_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    parsed: dict[str, object] = {}
    for key, value in pairs:
        if key in parsed:
            raise ValueError(f"duplicate JSON object key: {key}")
        parsed[key] = value
    return parsed


def _atomic_json(output: Path, value: dict[str, object]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def capture_source_snapshot(root: Path, output: Path) -> dict[str, object]:
    identity = collect_source_identity(root)
    receipt: dict[str, object] = {
        "schema_version": 1,
        "source": source_identity_to_mapping(identity),
    }
    _atomic_json(output, receipt)
    return receipt


def _read_source_snapshot(path: Path, root: Path) -> dict[str, object]:
    receipt = json.loads(
        path.read_text(encoding="utf-8"), object_pairs_hook=_strict_object
    )
    if (
        not isinstance(receipt, dict)
        or set(receipt) != {"schema_version", "source"}
        or isinstance(receipt.get("schema_version"), bool)
        or not isinstance(receipt.get("schema_version"), int)
        or receipt.get("schema_version") != 1
    ):
        raise ValueError("pre-build source snapshot does not match its schema")
    identity = source_identity_from_mapping(receipt["source"])
    verify_source_identity(root, identity)
    return source_identity_to_mapping(identity)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_value(sdk_root: Path, *args: str) -> str:
    result = subprocess.run(  # noqa: S603 -- fixed git argv and SDK path
        ("git", "-C", str(sdk_root), *args),  # noqa: S607
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ValueError(f"GAP SDK git {' '.join(args)} failed")
    return result.stdout.strip()


def git_worktree_dirty(sdk_root: Path) -> bool:
    result = subprocess.run(  # noqa: S603 -- fixed git argv and SDK path
        ("git", "-C", str(sdk_root), "diff-index", "--quiet", "HEAD", "--"),  # noqa: S607
        check=False,
    )
    if result.returncode not in (0, 1):
        raise ValueError("GAP SDK git diff-index failed")
    return result.returncode == 1


def create_provenance(
    elf: Path,
    compile_commands: Path,
    toolchain_id: Path,
    sdk_root: Path,
    *,
    source_root: Path | None = None,
    source_snapshot: Path | None = None,
) -> dict[str, Any]:
    if (source_root is None) != (source_snapshot is None):
        raise ValueError("source root and pre-build snapshot must be supplied together")
    compiler = toolchain_id.read_text(encoding="utf-8").strip()
    revision = git_value(sdk_root, "rev-parse", "HEAD")
    release = git_value(
        sdk_root, "describe", "--abbrev=0", "--tags", "--match", "release*"
    )
    dirty = git_worktree_dirty(sdk_root)
    if not compiler:
        raise ValueError("toolchain identity must not be empty")
    provenance: dict[str, Any] = {
        "toolchain_id": compiler,
        "binary_sha256": sha256_file(elf),
        "compiler_commands_sha256": sha256_file(compile_commands),
        "sdk_metadata": {
            "revision": revision,
            "release": release,
            "worktree_dirty": dirty,
        },
    }
    if source_root is not None and source_snapshot is not None:
        provenance["source_identity"] = _read_source_snapshot(
            source_snapshot, source_root
        )
    return provenance


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-source-snapshot", action="store_true")
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--source-snapshot", type=Path)
    parser.add_argument("--elf", type=Path)
    parser.add_argument("--compile-commands", type=Path)
    parser.add_argument("--toolchain-id", type=Path)
    parser.add_argument("--sdk-root", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        if args.output.exists():
            raise ValueError(f"output already exists: {args.output}")
        if args.capture_source_snapshot:
            if args.source_root is None:
                raise ValueError("--source-root is required for source snapshot mode")
            capture_source_snapshot(args.source_root, args.output)
            print(f"GAP9 pre-build source snapshot written: {args.output}")
            return 0
        required_paths = (
            args.elf,
            args.compile_commands,
            args.toolchain_id,
            args.sdk_root,
            args.source_root,
            args.source_snapshot,
        )
        if any(path is None for path in required_paths):
            raise ValueError(
                "ELF, compile commands, toolchain, SDK root, source root, and "
                "pre-build snapshot are required"
            )
        elf = args.elf.resolve(strict=True)
        compile_commands = args.compile_commands.resolve(strict=True)
        toolchain_id = args.toolchain_id.resolve(strict=True)
        sdk_root = args.sdk_root.resolve(strict=True)
        provenance = create_provenance(
            elf,
            compile_commands,
            toolchain_id,
            sdk_root,
            source_root=args.source_root.resolve(strict=True),
            source_snapshot=args.source_snapshot.resolve(strict=True),
        )
        _atomic_json(args.output, provenance)
    except (OSError, ValueError) as error:
        print(f"GAP9 build provenance failed: {error}")
        return 1
    print(f"GAP9 build provenance written: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
