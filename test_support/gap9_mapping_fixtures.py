"""Canonical GAP9 mapping marker and source-tree fixtures for tests."""

from __future__ import annotations

import shutil
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIRECTORY = REPOSITORY_ROOT / "tests/fixtures/gap9_mapping_diagnostics"


def _marker(name: str) -> str:
    return (FIXTURE_DIRECTORY / name).read_text(encoding="utf-8").rstrip("\n")


GVSOC_MAPPING_PASS = _marker("gvsoc-mapping-pass.marker")
GVSOC_MAPPING_PASS_V1 = _marker("gvsoc-mapping-pass-v1.marker")
GVSOC_MAPPING_LIMITATION = _marker("gvsoc-mapping-limitation.marker")
GVSOC_MAPPING_LIMITATION_V1 = _marker("gvsoc-mapping-limitation-v1.marker")
HOST_MAPPING_PASS = _marker("host-mapping-pass.marker")
HOST_MAPPING_FAILURE = _marker("host-mapping-failure.marker")


def materialize_mapping_source_checkout(destination: Path) -> tuple[Path, Path]:
    """Copy real fixture/core sources into an isolated temporary checkout."""
    fixture_relative = Path("tests/gap9_sdk_gvsoc_mapping_smoke.c")
    source_relative = Path("cslam-gap9-app/icp/icp.c")
    fixture = destination / fixture_relative
    source = destination / source_relative
    fixture.parent.mkdir(parents=True, exist_ok=True)
    source.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(REPOSITORY_ROOT / fixture_relative, fixture)
    shutil.copyfile(REPOSITORY_ROOT / source_relative, source)
    return fixture, source
