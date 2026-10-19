"""Strict JSON loading helpers for evidence and provenance manifests."""

from __future__ import annotations

import json
from pathlib import Path


class DuplicateJsonKeyError(ValueError):
    """Raised when an object repeats a key and would otherwise be ambiguous."""


def _unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise DuplicateJsonKeyError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def load_json_file(path: Path, *, label: str) -> object:
    """Load a JSON file while rejecting duplicate keys at every object depth."""
    try:
        return json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object
        )
    except DuplicateJsonKeyError as error:
        raise ValueError(f"{label} has ambiguous duplicate keys: {error}") from error
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {label}: {error}") from error
