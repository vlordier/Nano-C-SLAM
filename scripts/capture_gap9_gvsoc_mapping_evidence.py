"""Capture target-executed pose-graph mapping smoke evidence as JSON."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

MARKER = re.compile(
    r"NANO_C_SLAM_GVSOC:mapping_ok poses=(\d+) scans=(\d+) "
    r"map_points=(\d+) finite=(\d+) graph=(\d+) main_solver=(\d+) "
    r"solver_calls=(\d+) solver_iterations=(\d+) completed=(\d+) has_nan=(\d+)"
)
EXPECTED = ("60", "3", "240", "60", "3", "1", "3", "9", "1", "0")


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
    parser.add_argument("--model-root", type=Path)
    parser.add_argument("--pose-graph-fc-threshold", type=int)
    args = parser.parse_args()

    elf = args.elf.resolve(strict=True)
    log = args.log.resolve(strict=True)
    match = MARKER.search(log.read_text(encoding="utf-8", errors="replace"))
    observed = match.groups() if match else None
    passed = observed == EXPECTED
    report = {
        "schema_version": 1,
        "status": "passed" if passed else "failed",
        "evidence_class": "gvsoc-target-execution",
        "claim_boundary": (
            "Synthetic pose-graph mapping core on the GAP9 GVSOC model; "
            "not sensor, SPI, flight, or mapping-accuracy validation"
        ),
        "hardware_measurement": False,
        "test": "ICP loop closures and pose-graph optimization over synthetic poses",
        "expected": {
            "poses": 60,
            "scans": 3,
            "map_points": 240,
            "finite_poses": 60,
            "graph_nodes": 3,
            "main_solver_returned": True,
            "successful_solver_calls": 3,
            "completed_solver_iterations": 9,
            "optimization_completed": True,
            "has_nan": False,
        },
        "observed": (
            {
                "poses": int(observed[0]),
                "scans": int(observed[1]),
                "map_points": int(observed[2]),
                "finite_poses": int(observed[3]),
                "graph_nodes": int(observed[4]),
                "main_solver_returned": observed[5] == "1",
                "successful_solver_calls": int(observed[6]),
                "completed_solver_iterations": int(observed[7]),
                "optimization_completed": observed[8] == "1",
                "has_nan": observed[9] == "1",
            }
            if observed
            else None
        ),
        "elf_bytes": elf.stat().st_size,
        "elf_sha256": sha256(elf),
        "log_sha256": sha256(log),
    }
    if args.model_root is not None:
        model_root = args.model_root.resolve(strict=True)
        plugin = model_root / "gap/mchan/mchan_v7_impl.dylib"
        model = {
            "model_root": str(model_root),
            "mchan_plugin_sha256": sha256(plugin),
        }
        provenance = model_root.parent / "model-provenance.txt"
        if provenance.is_file():
            model["build_provenance"] = {
                key: value
                for line in provenance.read_text(encoding="utf-8").splitlines()
                if "=" in line
                for key, value in [line.split("=", maxsplit=1)]
            }
        report["gvsoc_model"] = model
    if args.pose_graph_fc_threshold is not None:
        report["pose_graph_fc_threshold"] = args.pose_graph_fc_threshold
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
