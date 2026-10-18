"""Capture the deterministic mapping smoke through host PMSIS stubs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import tempfile
from datetime import UTC, datetime
from pathlib import Path, PurePosixPath
from typing import Any

from scripts.gap9_mapping_diagnostic import (
    mapping_source_paths,
    validate_mapping_diagnostic,
)
from scripts.process_provenance import capture_process
from scripts.runtime_provenance import sha256_file_stable
from scripts.source_provenance import collect_source_identity, verify_source_identity
from scripts.validation_provenance import (
    BuildIdentity,
    DependencyDisposition,
    EvidenceClass,
    ExecutableIdentity,
    RunStatus,
    RuntimeDependency,
    ToolchainIdentity,
    ValidationManifest,
    parse_manifest_json,
    serialize_manifest,
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


def _validate_manifest_join(
    artifact: dict[str, Any],
    encoded_manifest: bytes,
    sidecar_name: str,
    stdout: bytes,
    stderr: bytes,
) -> None:
    """Ensure report, sidecar, executable, and raw process streams agree."""
    validate_mapping_diagnostic(artifact)
    reference = artifact["provenance_manifest"]
    if reference["path"] != sidecar_name:
        raise ValueError("manifest reference must name its portable sidecar")
    if reference["sha256"] != hashlib.sha256(encoded_manifest).hexdigest():
        raise ValueError("report manifest hash does not match serialized sidecar")
    manifest = parse_manifest_json(encoded_manifest.decode("utf-8"))
    expected_status = "success" if artifact["status"] == "passed" else "fail"
    if manifest.status.value != expected_status:
        raise ValueError("report and manifest status disagree")
    if manifest.evidence_class.value != artifact["evidence_class"]:
        raise ValueError("report and manifest evidence classes disagree")
    executable = manifest.build.executable
    if executable is None or executable.sha256 != artifact["build"]["binary_sha256"]:
        raise ValueError("report binary hash disagrees with manifest executable")
    if manifest.execution.exit_code != artifact["execution"]["exit_code"]:
        raise ValueError("report and manifest process exit codes disagree")
    if manifest.execution.stdout_sha256 != hashlib.sha256(stdout).hexdigest():
        raise ValueError("manifest stdout hash disagrees with captured stdout")
    if manifest.execution.stdout_bytes != len(stdout):
        raise ValueError("manifest stdout byte count disagrees with captured stdout")
    if manifest.execution.stderr_sha256 != hashlib.sha256(stderr).hexdigest():
        raise ValueError("manifest stderr hash disagrees with captured stderr")
    if manifest.execution.stderr_bytes != len(stderr):
        raise ValueError("manifest stderr byte count disagrees with captured stderr")
    if (
        artifact["execution"]["log_sha256"]
        != hashlib.sha256(stdout + stderr).hexdigest()
    ):
        raise ValueError("report raw log hash disagrees with captured process streams")


def validate_host_mapping_bundle(report_path: Path) -> ValidationManifest:
    """Load and cross-check a retained host report, log, and provenance sidecar."""
    report = json.loads(report_path.read_text(encoding="utf-8"))
    validate_mapping_diagnostic(report)
    reference = report.get("provenance_manifest")
    if not isinstance(reference, dict):
        raise ValueError("legacy diagnostic has no provenance manifest sidecar")
    relative = PurePosixPath(reference["path"])
    report_directory = report_path.resolve(strict=True).parent
    manifest_path = (report_directory / Path(*relative.parts)).resolve(strict=True)
    if not manifest_path.is_relative_to(report_directory):
        raise ValueError("provenance sidecar resolves outside the report directory")
    manifest_bytes = manifest_path.read_bytes()
    log_bytes = report_path.with_suffix(".log").read_bytes()
    if reference["sha256"] != hashlib.sha256(manifest_bytes).hexdigest():
        raise ValueError("provenance sidecar hash disagrees with diagnostic report")
    manifest = parse_manifest_json(manifest_bytes.decode("utf-8"))
    stdout_bytes = manifest.execution.stdout_bytes
    stderr_bytes = manifest.execution.stderr_bytes
    if stdout_bytes is None or stderr_bytes is None:
        raise ValueError("retained process stream lengths are unknown")
    if len(log_bytes) != stdout_bytes + stderr_bytes:
        raise ValueError("retained combined log length disagrees with stream lengths")
    stdout = log_bytes[:stdout_bytes]
    stderr = log_bytes[stdout_bytes:]
    _validate_manifest_join(
        report,
        manifest_bytes,
        relative.as_posix(),
        stdout,
        stderr,
    )
    return manifest


def _publish_atomically(
    output: Path,
    artifact: dict[str, Any],
    log: bytes,
    provenance: bytes,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    log_output = output.with_suffix(".log")
    provenance_output = output.with_suffix(".provenance.json")
    outputs = (log_output, output, provenance_output)
    if any(path.exists() for path in outputs):
        raise FileExistsError("diagnostic, log, or provenance sidecar already exists")
    temporary_paths: list[Path] = []
    published_paths: list[Path] = []
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=output.parent,
            prefix=f".{log_output.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            log_temporary = Path(stream.name)
            temporary_paths.append(log_temporary)
            stream.write(log)
            stream.flush()
            os.fsync(stream.fileno())
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            json_temporary = Path(stream.name)
            temporary_paths.append(json_temporary)
            json.dump(artifact, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=output.parent,
            prefix=f".{provenance_output.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            provenance_temporary = Path(stream.name)
            temporary_paths.append(provenance_temporary)
            stream.write(provenance)
            stream.flush()
            os.fsync(stream.fileno())
        for temporary, destination in zip(
            (log_temporary, json_temporary, provenance_temporary),
            outputs,
            strict=True,
        ):
            os.link(temporary, destination)
            published_paths.append(destination)
    except OSError:
        for path in published_paths:
            path.unlink(missing_ok=True)
        raise
    finally:
        for path in temporary_paths:
            path.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--toolchain-id", required=True)
    parser.add_argument("--compiler-path", type=Path)
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
    log_output = output.with_suffix(".log")
    provenance_output = output.with_suffix(".provenance.json")
    if output.exists() or log_output.exists() or provenance_output.exists():
        parser.error(f"output or log already exists: {output} / {log_output}")
    source_identity = collect_source_identity(REPOSITORY_ROOT)
    fixture_hash_before = _sha256(FIXTURE)
    source_hashes = {
        path.relative_to(REPOSITORY_ROOT).as_posix(): _sha256(path)
        for path in mapping_source_paths(REPOSITORY_ROOT)
    }
    if source_hashes["tests/gap9_sdk_gvsoc_mapping_smoke.c"] != fixture_hash_before:
        parser.error("fixture hash disagrees with source provenance")
    binary_hash_before = sha256_file_stable(binary)
    compiler_hash_before = (
        sha256_file_stable(args.compiler_path.resolve(strict=True))
        if args.compiler_path is not None
        else None
    )
    try:
        result = capture_process(
            (str(binary),),
            environment_allowlist=(),
            timeout_seconds=args.timeout_s,
        )
        if result.timed_out or result.execution.exit_code is None:
            raise TimeoutError(
                f"host mapping binary exceeded {args.timeout_s:g} seconds"
            )
        raw_log = result.stdout + result.stderr
        status, mapping = _parse_marker(raw_log.decode("utf-8", errors="replace"))
    except (OSError, TimeoutError, ValueError) as error:
        print(f"host mapping diagnostic capture failed: {error}")
        return 1
    if (status == "passed") != (result.execution.exit_code == 0):
        print("host mapping marker status disagrees with process exit code")
        return 1
    try:
        verify_source_identity(REPOSITORY_ROOT, source_identity)
    except ValueError as error:
        print(f"host mapping source changed during capture: {error}")
        return 1
    if (
        _sha256(FIXTURE) != fixture_hash_before
        or sha256_file_stable(binary) != binary_hash_before
    ):
        print("fixture or executable changed during capture")
        return 1
    if (
        args.compiler_path is not None
        and sha256_file_stable(args.compiler_path.resolve(strict=True))
        != compiler_hash_before
    ):
        print("compiler executable changed during capture")
        return 1
    toolchain = None
    if compiler_hash_before is not None:
        toolchain = ToolchainIdentity(
            name="C compiler",
            version=args.toolchain_id,
            executable_sha256=compiler_hash_before,
        )
    compiler_configuration = json.dumps(
        {
            "compiler_flags": args.compiler_flags,
            "toolchain_id": args.toolchain_id,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    binary_relative = (
        binary.relative_to(REPOSITORY_ROOT).as_posix()
        if binary.is_relative_to(REPOSITORY_ROOT)
        else None
    )
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
            "exit_code": result.execution.exit_code,
            "marker_seen": True,
            "terminated": True,
            "log_sha256": hashlib.sha256(raw_log).hexdigest(),
            "log_bytes": len(raw_log),
        },
        "mapping": mapping,
    }
    try:
        manifest = ValidationManifest(
            schema_version=1,
            status=RunStatus.SUCCESS if status == "passed" else RunStatus.FAIL,
            evidence_class=EvidenceClass.HOST,
            claim_boundary=artifact["claim_boundary"],
            captured_at_utc=datetime.now(UTC).isoformat().replace("+00:00", "Z"),
            source=source_identity,
            build=BuildIdentity(
                command=None,
                configuration_sha256=hashlib.sha256(compiler_configuration).hexdigest(),
                toolchain=toolchain,
                executable=ExecutableIdentity(binary_relative, binary_hash_before),
            ),
            runtime_dependencies=(
                RuntimeDependency(
                    name="host-runtime-linkage",
                    disposition=DependencyDisposition.UNKNOWN,
                    version=None,
                    sha256=None,
                    path=None,
                ),
            ),
            execution=result.execution,
            hardware_attestation=None,
        )
        provenance_text = serialize_manifest(manifest)
        provenance_bytes = provenance_text.encode("utf-8")
        provenance_name = output.with_suffix(".provenance.json").name
        artifact["provenance_manifest"] = {
            "path": provenance_name,
            "sha256": hashlib.sha256(provenance_bytes).hexdigest(),
        }
        _validate_manifest_join(
            artifact,
            provenance_bytes,
            provenance_name,
            result.stdout,
            result.stderr,
        )
        validate_mapping_diagnostic(artifact)
        _publish_atomically(output, artifact, raw_log, provenance_bytes)
    except (OSError, ValueError) as error:
        print(f"host mapping diagnostic artifact rejected: {error}")
        return 1
    print(f"Host-only mapping diagnostic written: {output}")
    return 0 if status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
