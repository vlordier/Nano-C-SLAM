"""Validate and publish typed evidence from a GAP9 GVSOC mapping run."""

from __future__ import annotations

import argparse
import hashlib
import json
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
FIXTURE_COUNTS = (60, 3, 240)
PASS_MARKER = re.compile(
    r"^NANO_C_SLAM_GVSOC:mapping_ok "
    r"poses=(\d+) scans=(\d+) map_points=(\d+) finite=(\d+) graph=(\d+) "
    r"main_solver=([01]) solver_calls=(\d+) solver_iterations=(\d+) "
    r"completed=([01]) has_nan=([01]) icp_attempts=(\d+) "
    r"accepted_constraints=(\d+) optimizer_inputs=(\d+) "
    r"optimizer_unique_edges=(\d+) matches=(-?\d+) "
    r"icp_error_milli=(-?\d+) icp_has_nan=([01])$"
)
FAIL_MARKER = re.compile(
    r"^NANO_C_SLAM_GVSOC:mapping_fail "
    r"poses=(\d+) scans=(\d+) points=(\d+) icp_constraints=(\d+) "
    r"matches=(-?\d+) icp_error_milli=(-?\d+) icp_has_nan=([01]) "
    r"finite=(\d+) graph=(\d+) main_solver=([01]) solver_calls=(\d+) "
    r"solver_iterations=(\d+) completed=([01]) has_nan=([01]) "
    r"icp_attempts=(\d+) accepted_constraints=(\d+) "
    r"optimizer_inputs=(\d+) optimizer_unique_edges=(\d+)$"
)
EXPECTED_LIMITATION = {
    "finite_poses": 0,
    "icp_attempts": 2,
    "correspondences_last_pair": 80,
    "icp_error_milli": 2_147_483_647,
    "icp_has_nan": True,
    "accepted_constraints": 0,
    "graph_nodes": 0,
    "optimizer_input_constraints": 0,
    "optimizer_unique_edges": 0,
    "main_solver_returned": False,
    "successful_solver_calls": 0,
    "completed_solver_iterations": 0,
    "optimization_completed": False,
    "has_nan": False,
}


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_target_marker(log_text: str) -> tuple[str, dict[str, int | bool]]:
    if "NANO_C_SLAM_HOST:" in log_text:
        raise ValueError("host-class marker cannot occur in a GVSOC target log")
    matches: list[tuple[str, re.Match[str]]] = []
    for line in log_text.splitlines():
        stripped = line.strip()
        if match := PASS_MARKER.fullmatch(stripped):
            matches.append(("passed", match))
        if match := FAIL_MARKER.fullmatch(stripped):
            matches.append(("failed", match))
    if len(matches) != 1:
        raise ValueError("GVSOC log must contain exactly one complete target marker")
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
            error_milli,
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
            error_milli,
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
    if (poses, scans, points) != FIXTURE_COUNTS:
        raise ValueError("GVSOC marker counts do not match the deterministic fixture")
    if legacy_accepted != accepted:
        raise ValueError("legacy and diagnostic accepted-constraint counts disagree")
    return status, {
        "poses": poses,
        "scans": scans,
        "points": points,
        "finite_poses": finite_poses,
        "has_nan": bool(has_nan),
        "icp_attempts": attempts,
        "correspondences_last_pair": matches_count,
        "icp_error_milli": error_milli,
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


def sdk_metadata(sdk_root: Path) -> dict[str, str | bool]:
    revision = _git(sdk_root, "rev-parse", "HEAD")
    release = _git(sdk_root, "describe", "--abbrev=0", "--tags", "--match", "release*")
    dirty_output = _git(sdk_root, "status", "--porcelain")
    if not revision or not release:
        raise ValueError("could not establish GAP SDK revision and release")
    return {
        "revision": revision,
        "release": release,
        "worktree_dirty": bool(dirty_output),
    }


def _git(directory: Path, *arguments: str) -> str:
    result = subprocess.run(  # noqa: S603 -- fixed git argv and SDK directory
        ("git", "-C", str(directory), *arguments),  # noqa: S607
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ValueError(f"GAP SDK git {' '.join(arguments)} failed")
    return result.stdout.strip()


def _compiler_commands(path: Path) -> str:
    commands = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(commands, list) or not commands:
        raise ValueError("compile_commands.json must contain compiler invocations")
    normalized: list[str] = []
    for entry in commands:
        if not isinstance(entry, dict):
            raise ValueError("compile_commands.json contains a malformed entry")
        command = entry.get("command")
        if not isinstance(command, str) or not command.strip():
            arguments = entry.get("arguments")
            if not isinstance(arguments, list) or not all(
                isinstance(argument, str) for argument in arguments
            ):
                raise ValueError("compile command is missing its exact invocation")
            command = json.dumps(arguments, separators=(",", ":"))
        normalized.append(command)
    return "\n".join(sorted(set(normalized)))


def _publish_atomically(output: Path, artifact: dict[str, Any]) -> None:
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
            json.dump(artifact, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _publish_bytes_atomically(output: Path, value: bytes) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--compile-commands", required=True, type=Path)
    parser.add_argument("--build-provenance", required=True, type=Path)
    parser.add_argument("--sdk-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--retained-log", type=Path)
    parser.add_argument("--gvsoc-exit-code", required=True, type=int)
    args = parser.parse_args()

    try:
        elf = args.elf.resolve(strict=True)
        log = args.log.resolve(strict=True)
        compile_commands = args.compile_commands.resolve(strict=True)
        build_provenance_file = args.build_provenance.resolve(strict=True)
        sdk_root = args.sdk_root.resolve(strict=True)
        if (
            not elf.is_file()
            or not compile_commands.is_file()
            or not log.is_file()
            or not build_provenance_file.is_file()
        ):
            raise ValueError(
                "ELF, compile commands, build provenance, and GVSOC log must be files"
            )
        output = args.output.resolve()
        retained_log = (
            args.retained_log.resolve()
            if args.retained_log is not None
            else output.with_suffix(".log")
        )
        if output.exists() or retained_log.exists():
            raise ValueError("diagnostic JSON or retained log already exists")
        before = {
            "elf": sha256_file(elf),
            "log": sha256_file(log),
            "compile_commands": sha256_file(compile_commands),
            "build_provenance": sha256_file(build_provenance_file),
        }
        build_provenance = json.loads(build_provenance_file.read_text(encoding="utf-8"))
        if not isinstance(build_provenance, dict) or set(build_provenance) != {
            "toolchain_id",
            "binary_sha256",
            "compiler_commands_sha256",
            "sdk_metadata",
        }:
            raise ValueError("build provenance fields do not match the target contract")
        if build_provenance["binary_sha256"] != before["elf"]:
            raise ValueError("target ELF does not match its build-time provenance")
        if build_provenance["compiler_commands_sha256"] != before["compile_commands"]:
            raise ValueError(
                "compile commands do not match their build-time provenance"
            )
        fixture_hash = sha256_file(
            REPOSITORY_ROOT / "tests/gap9_sdk_gvsoc_mapping_smoke.c"
        )
        source_hashes = {
            path.relative_to(REPOSITORY_ROOT).as_posix(): sha256_file(path)
            for path in mapping_source_paths(REPOSITORY_ROOT)
        }
        if source_hashes["tests/gap9_sdk_gvsoc_mapping_smoke.c"] != fixture_hash:
            raise ValueError("fixture hash disagrees with source provenance")
        log_bytes = log.read_bytes()
        log_text = log_bytes.decode("utf-8", errors="replace")
        marker_status, mapping = parse_target_marker(log_text)
        toolchain = build_provenance["toolchain_id"]
        if not isinstance(toolchain, str) or not toolchain.strip():
            raise ValueError("toolchain identity is missing from build provenance")
        commands = _compiler_commands(compile_commands)
        sdk = build_provenance["sdk_metadata"]
        if sdk != sdk_metadata(sdk_root):
            raise ValueError("GAP SDK differs from the SDK recorded at build time")
        if marker_status == "passed":
            if args.gvsoc_exit_code != 0:
                raise ValueError("mapping success marker disagrees with GVSOC exit")
            status = "passed"
        elif args.gvsoc_exit_code == 1 and mapping == {
            "poses": 60,
            "scans": 3,
            "points": 240,
            **EXPECTED_LIMITATION,
        }:
            status = "reproduced-limitation"
        else:
            raise ValueError("GVSOC failure does not match the known ICP limitation")
        after = {
            "elf": sha256_file(elf),
            "log": sha256_file(log),
            "compile_commands": sha256_file(compile_commands),
            "build_provenance": sha256_file(build_provenance_file),
        }
        if before != after:
            raise ValueError("ELF, log, or compile commands changed during capture")
        if source_hashes != {
            path.relative_to(REPOSITORY_ROOT).as_posix(): sha256_file(path)
            for path in mapping_source_paths(REPOSITORY_ROOT)
        }:
            raise ValueError("mapping source changed during capture")
        if sdk != sdk_metadata(sdk_root):
            raise ValueError("GAP SDK revision or worktree changed during capture")
        artifact: dict[str, Any] = {
            "schema_version": 2,
            "status": status,
            "evidence_class": "gvsoc-target-execution",
            "claim_boundary": (
                "Deterministic mapping fixture executed by the GAP9 target on "
                "GVSOC. This is not sensor, SPI, flight, hardware, or mapping-"
                "accuracy validation."
            ),
            "hardware_measurement": False,
            "fixture": {
                "name": "gap9_sdk_gvsoc_mapping_smoke",
                "source_sha256": fixture_hash,
                "pose_count": FIXTURE_COUNTS[0],
                "scan_count": FIXTURE_COUNTS[1],
                "point_count": FIXTURE_COUNTS[2],
            },
            "build": {
                "toolchain_id": toolchain,
                "compiler_flags": commands,
                "binary_sha256": before["elf"],
                "source_sha256": source_hashes,
                "sdk_metadata": sdk,
                "compiler_commands_sha256": before["compile_commands"],
            },
            "execution": {
                "exit_code": args.gvsoc_exit_code,
                "marker_seen": True,
                "terminated": True,
                "log_sha256": sha256_bytes(log_bytes),
                "log_bytes": len(log_bytes),
            },
            "mapping": mapping,
        }
        validate_mapping_diagnostic(artifact)
        _publish_bytes_atomically(retained_log, log_bytes)
        _publish_atomically(output, artifact)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"GVSOC mapping diagnostic capture failed: {error}")
        return 1
    print(f"GVSOC mapping diagnostic written: {output}; raw log: {retained_log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
