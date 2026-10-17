"""Record a reproducible GAP9 GVSOC cluster-ICP mapping limitation."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

MARKER = re.compile(
    r"NANO_C_SLAM_GVSOC:mapping_fail poses=(\d+) scans=(\d+) "
    r"points=(\d+) icp_constraints=(\d+) matches=(\d+) "
    r"icp_error_milli=(-?\d+) icp_has_nan=(\d+) "
    r"optimized=(\d+) graph=(\d+) has_nan=(\d+)"
)
EXPECTED = ("60", "3", "240", "0", "80", "2147483647", "1", "0", "0", "0")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--gvsoc-exit-code", required=True, type=int)
    args = parser.parse_args()

    elf = args.elf.resolve(strict=True)
    log = args.log.resolve(strict=True)
    match = MARKER.search(log.read_text(encoding="utf-8", errors="replace"))
    observed = match.groups() if match else None
    reproduced = observed == EXPECTED and args.gvsoc_exit_code == 1
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
        "host_core_reference": {
            "status": "passed",
            "poses": 60,
            "scans": 3,
            "map_points": 240,
            "icp_constraints_at_least": 2,
            "optimized_poses": 60,
            "graph_nodes": 3,
            "execution": "native host with PMSIS compute-boundary stubs",
        },
        "elf_bytes": elf.stat().st_size,
        "elf_sha256": sha256(elf),
        "log_sha256": sha256(log),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if reproduced else 1


if __name__ == "__main__":
    raise SystemExit(main())
