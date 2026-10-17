"""Choose a fresh mapping-evidence path without replacing prior runs."""

from __future__ import annotations

import argparse
from datetime import UTC, datetime
from pathlib import Path


def _occupied(path: Path) -> bool:
    return path.exists() or path.with_suffix(".log").exists()


def select_mapping_output(
    requested: Path, *, explicit: bool, now: datetime | None = None
) -> Path:
    """Preserve existing evidence and reject collisions on explicit paths."""
    if explicit:
        if _occupied(requested):
            raise FileExistsError(f"mapping output or log already exists: {requested}")
        return requested
    if not _occupied(requested):
        return requested

    timestamp = (now or datetime.now(UTC)).strftime("%Y%m%dT%H%M%SZ")
    suffix = requested.suffix or ".json"
    stem = requested.stem if requested.suffix else requested.name
    for index in range(1000):
        ordinal = "" if index == 0 else f"-{index}"
        candidate = requested.with_name(f"{stem}-{timestamp}{ordinal}{suffix}")
        if not _occupied(candidate):
            return candidate
    raise FileExistsError("could not choose a unique mapping evidence filename")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--explicit", action="store_true")
    args = parser.parse_args()
    try:
        print(select_mapping_output(args.path, explicit=args.explicit))
    except FileExistsError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
