"""Record immutable inputs emitted by a GAP9 GVSOC target build."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any


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
) -> dict[str, Any]:
    compiler = toolchain_id.read_text(encoding="utf-8").strip()
    revision = git_value(sdk_root, "rev-parse", "HEAD")
    release = git_value(
        sdk_root, "describe", "--abbrev=0", "--tags", "--match", "release*"
    )
    dirty = git_worktree_dirty(sdk_root)
    if not compiler:
        raise ValueError("toolchain identity must not be empty")
    return {
        "toolchain_id": compiler,
        "binary_sha256": sha256_file(elf),
        "compiler_commands_sha256": sha256_file(compile_commands),
        "sdk_metadata": {
            "revision": revision,
            "release": release,
            "worktree_dirty": dirty,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--compile-commands", required=True, type=Path)
    parser.add_argument("--toolchain-id", required=True, type=Path)
    parser.add_argument("--sdk-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        elf = args.elf.resolve(strict=True)
        compile_commands = args.compile_commands.resolve(strict=True)
        toolchain_id = args.toolchain_id.resolve(strict=True)
        sdk_root = args.sdk_root.resolve(strict=True)
        if args.output.exists():
            raise ValueError(f"output already exists: {args.output}")
        provenance = create_provenance(elf, compile_commands, toolchain_id, sdk_root)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(provenance, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError) as error:
        print(f"GAP9 build provenance failed: {error}")
        return 1
    print(f"GAP9 build provenance written: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
