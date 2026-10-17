"""Capture the deterministic mapping smoke through host PMSIS stubs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from scripts.gap9_mapping_diagnostic import (
    mapping_source_paths,
    validate_mapping_diagnostic,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FIXTURE = REPOSITORY_ROOT / "tests/gap9_sdk_gvsoc_mapping_smoke.c"
HOST_MARKER = re.compile(
    r"^NANO_C_SLAM_HOST:mapping_ok "
    r"poses=(\d+) scans=(\d+) map_points=(\d+) finite=(\d+) graph=(\d+) "
    r"main_solver=([01]) solver_calls=(\d+) solver_iterations=(\d+) "
    r"completed=([01]) has_nan=([01]) icp_attempts=(\d+) "
    r"accepted_constraints=(\d+) optimizer_inputs=(\d+) "
    r"optimizer_unique_edges=(\d+) matches=(-?\d+) "
    r"icp_error_milli=(-?\d+) icp_has_nan=([01])"
)
FAIL_MARKER = re.compile(
    r"^NANO_C_SLAM_HOST:mapping_fail "
    r"poses=(\d+) scans=(\d+) points=(\d+) icp_constraints=(\d+) "
    r"matches=(-?\d+) icp_error_milli=(-?\d+) icp_has_nan=([01]) "
    r"finite=(\d+) graph=(\d+) main_solver=([01]) solver_calls=(\d+) "
    r"solver_iterations=(\d+) completed=([01]) has_nan=([01]) "
    r"icp_attempts=(\d+) accepted_constraints=(\d+) "
    r"optimizer_inputs=(\d+) optimizer_unique_edges=(\d+)"
)
EXPECTED_COUNTS = (60, 3, 240)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_marker(output: str) -> tuple[str, dict[str, int | bool]]:
    matches = [
        ("passed", match)
        for line in output.splitlines()
        if (match := HOST_MARKER.fullmatch(line.strip())) is not None
    ]
    matches.extend(
        ("failed", match)
        for line in output.splitlines()
        if (match := FAIL_MARKER.fullmatch(line.strip())) is not None
    )
    if len(matches) != 1:
        raise ValueError("host smoke must emit exactly one well-formed host marker")
    status, match = matches[0]
    values = tuple(int(value) for value in match.groups())
    if status == "passed":
        (
            poses,
            scans,
            points,
            finite_poses,
            graph_nodes,
            main_solver,
            solver_calls,
            solver_iterations,
            completed,
            has_nan,
            attempts,
            accepted,
            optimizer_inputs,
            unique_edges,
            matches_count,
            icp_error,
            icp_has_nan,
        ) = values
        legacy_accepted = accepted
    else:
        (
            poses,
            scans,
            points,
            legacy_accepted,
            matches_count,
            icp_error,
            icp_has_nan,
            finite_poses,
            graph_nodes,
            main_solver,
            solver_calls,
            solver_iterations,
            completed,
            has_nan,
            attempts,
            accepted,
            optimizer_inputs,
            unique_edges,
        ) = values
    if (poses, scans, points) != EXPECTED_COUNTS:
        raise ValueError("host marker counts do not match the deterministic fixture")
    if status == "failed" and legacy_accepted != accepted:
        raise ValueError("legacy and diagnostic accepted-constraint counts disagree")
    return status, {
        "poses": poses,
        "scans": scans,
        "points": points,
        "finite_poses": finite_poses,
        "has_nan": bool(has_nan),
        "icp_attempts": attempts,
        "correspondences_last_pair": matches_count,
        "icp_error_milli": icp_error,
        "icp_has_nan": bool(icp_has_nan),
        "accepted_constraints": accepted,
        "graph_nodes": graph_nodes,
        "optimizer_input_constraints": optimizer_inputs,
        "optimizer_unique_edges": unique_edges,
        "main_solver_returned": bool(main_solver),
        "successful_solver_calls": solver_calls,
        "completed_solver_iterations": solver_iterations,
        "optimization_completed": bool(completed),
    }


def _publish_atomically(output: Path, artifact: dict[str, Any]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            json.dump(artifact, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary_path, output)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--toolchain-id", required=True)
    parser.add_argument("--compiler-flags", required=True)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("--timeout-s must be finite and positive")
    if not args.toolchain_id.strip() or not args.compiler_flags.strip():
        parser.error("toolchain ID and compiler flags must be nonempty")

    binary = args.binary.resolve(strict=True)
    if not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error("--binary must be an executable file")
    output = args.output.resolve()
    if output.exists():
        parser.error(f"output already exists: {output}")
    fixture_hash_before = _sha256(FIXTURE)
    source_hashes = {
        path.relative_to(REPOSITORY_ROOT).as_posix(): _sha256(path)
        for path in mapping_source_paths(REPOSITORY_ROOT)
    }
    if source_hashes["tests/gap9_sdk_gvsoc_mapping_smoke.c"] != fixture_hash_before:
        parser.error("fixture hash disagrees with source provenance")
    binary_hash_before = _sha256(binary)
    try:
        result = subprocess.run(  # noqa: S603 -- explicit repository-built smoke ELF
            [str(binary)],
            capture_output=True,
            check=False,
            text=True,
            timeout=args.timeout_s,
        )
        status, mapping = _parse_marker(result.stdout)
    except (OSError, subprocess.TimeoutExpired, ValueError) as error:
        print(f"host mapping diagnostic capture failed: {error}")
        return 1
    if (status == "passed") != (result.returncode == 0):
        print("host mapping marker status disagrees with process exit code")
        return 1
    if _sha256(FIXTURE) != fixture_hash_before or _sha256(binary) != binary_hash_before:
        print("fixture or executable changed during capture")
        return 1

    artifact: dict[str, Any] = {
        "schema_version": 2,
        "status": status,
        "evidence_class": "host-pmsis-stubs",
        "claim_boundary": (
            "Synthetic mapping fixture through host PMSIS stubs only; "
            "not target execution, sensor, SPI, flight, or hardware validation."
        ),
        "hardware_measurement": False,
        "fixture": {
            "name": "gap9_sdk_gvsoc_mapping_smoke",
            "source_sha256": fixture_hash_before,
            "pose_count": EXPECTED_COUNTS[0],
            "scan_count": EXPECTED_COUNTS[1],
            "point_count": EXPECTED_COUNTS[2],
        },
        "build": {
            "toolchain_id": args.toolchain_id,
            "compiler_flags": args.compiler_flags,
            "binary_sha256": binary_hash_before,
            "source_sha256": source_hashes,
            "sdk_metadata": None,
            "compiler_commands_sha256": None,
        },
        "execution": {
            "exit_code": result.returncode,
            "marker_seen": True,
            "terminated": True,
            "log_sha256": None,
            "log_bytes": None,
        },
        "mapping": mapping,
    }
    try:
        validate_mapping_diagnostic(artifact)
        _publish_atomically(output, artifact)
    except (OSError, ValueError) as error:
        print(f"host mapping diagnostic artifact rejected: {error}")
        return 1
    print(f"Host-only mapping diagnostic written: {output}")
    return 0 if status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
