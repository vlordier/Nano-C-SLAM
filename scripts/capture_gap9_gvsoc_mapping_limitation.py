"""Record a reproducible GAP9 GVSOC cluster-ICP mapping limitation."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
from pathlib import Path

MARKER = re.compile(
    r"NANO_C_SLAM_GVSOC:mapping_fail poses=(\d+) scans=(\d+) "
    r"points=(\d+) icp_constraints=(\d+) matches=(\d+) "
    r"icp_error_milli=(-?\d+) icp_has_nan=(\d+) "
    r"optimized=(\d+) graph=(\d+) has_nan=(\d+)"
)
EXPECTED = ("60", "3", "240", "0", "80", "2147483647", "1", "0", "0", "0")
HOST_MARKER = re.compile(
    r"NANO_C_SLAM_HOST:mapping_ok poses=(\d+) scans=(\d+) map_points=(\d+) "
    r"finite=(\d+) graph=(\d+)"
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
REFERENCE_SOURCES = (
    "tests/gap9_sdk_gvsoc_mapping_smoke.c",
    "cslam-gap9-app/icp/icp.c",
    "cslam-gap9-app/point-cloud/point_cloud.c",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_revision(directory: Path) -> str | None:
    result = subprocess.run(  # noqa: S603 - fixed git argv and trusted repo path
        ("git", "-C", str(directory), "rev-parse", "HEAD"),  # noqa: S607
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def git_worktree_dirty(directory: Path) -> bool | None:
    result = subprocess.run(  # noqa: S603 - fixed git argv and trusted repo path
        ("git", "-C", str(directory), "status", "--porcelain"),  # noqa: S607
        check=False,
        capture_output=True,
        text=True,
    )
    return bool(result.stdout.strip()) if result.returncode == 0 else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--gvsoc-exit-code", required=True, type=int)
    parser.add_argument("--gvsoc-release")
    parser.add_argument(
        "--host-elf",
        type=Path,
        default=REPOSITORY_ROOT / "cslam-gap9-app/BUILD-host/gap9_mapping_smoke",
    )
    args = parser.parse_args()

    elf = args.elf.resolve(strict=True)
    log = args.log.resolve(strict=True)
    log_text = log.read_text(encoding="utf-8", errors="replace")
    match = MARKER.search(log_text)
    observed = match.groups() if match else None
    reproduced = observed == EXPECTED and args.gvsoc_exit_code == 1
    host_elf = args.host_elf.resolve(strict=True)
    host_result = subprocess.run(  # noqa: S603 - explicit local host ELF under test
        (str(host_elf),), capture_output=True, text=True, check=False
    )
    host_match = HOST_MARKER.search(host_result.stdout)
    host_passed = (
        host_result.returncode == 0
        and host_match is not None
        and host_match.groups() == ("60", "3", "240", "60", "3")
    )
    sdk_root = Path(
        os.environ.get(
            "GAP9_SDK_ROOT",
            Path.home() / "Work/da3_gap9/third_party/gap_sdk_private",
        )
    )
    report = {
        "schema_version": 1,
        "status": "reproduced-limitation" if reproduced else "unexpected-result",
        "evidence_class": "gvsoc-target-execution",
        "claim_boundary": (
            "The GAP9 GVSOC target boots and executes the fixture, but its "
            "cluster ICP path produces no accepted constraints and does not "
            "reach pose-graph optimization. This is not a GAP9 mapping pass, "
            "hardware validation, or mapping-accuracy result."
        ),
        "hardware_measurement": False,
        "test": "60-pose, three-scan point-cloud and cluster-ICP fixture",
        "observed": (
            {
                "poses": int(observed[0]),
                "scans": int(observed[1]),
                "map_points": int(observed[2]),
                "icp_constraints": int(observed[3]),
                "matched_points_last_pair": int(observed[4]),
                "icp_error_milli": int(observed[5]),
                "icp_has_nan": observed[6] == "1",
                "optimized_poses": int(observed[7]),
                "graph_nodes": int(observed[8]),
                "has_nan": observed[9] == "1",
            }
            if observed
            else None
        ),
        "gvsoc_exit_code": args.gvsoc_exit_code,
        "target_marker": match.group(0) if match else None,
        "target_log_path": str(log),
        "source_provenance": {
            "repository_revision": git_revision(REPOSITORY_ROOT),
            "repository_dirty": git_worktree_dirty(REPOSITORY_ROOT),
            "source_sha256": {
                path: sha256(REPOSITORY_ROOT / path) for path in REFERENCE_SOURCES
            },
        },
        "gvsoc_runtime": {
            "sdk_revision": git_revision(sdk_root),
            "sdk_worktree_dirty": git_worktree_dirty(sdk_root),
            "release": args.gvsoc_release
            or (
                release_match.group(1)
                if (release_match := re.search(r"Release version: ([^\s]+)", log_text))
                else None
            ),
        },
        "host_core_reference": {
            "status": "passed" if host_passed else "failed",
            "poses": 60,
            "scans": 3,
            "map_points": 240,
            "icp_constraints_at_least": 2,
            "optimized_poses": 60,
            "graph_nodes": 3,
            "execution": "native host with PMSIS compute-boundary stubs",
            "elf_sha256": sha256(host_elf),
            "exit_code": host_result.returncode,
            "stdout": host_result.stdout,
            "stderr": host_result.stderr,
        },
        "elf_bytes": elf.stat().st_size,
        "elf_sha256": sha256(elf),
        "log_sha256": sha256(log),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if reproduced and host_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
