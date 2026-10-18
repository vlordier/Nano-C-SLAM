"""Validate and publish typed evidence from a GAP9 GVSOC mapping run."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
from datetime import UTC, datetime
from pathlib import Path, PurePosixPath
from typing import Any

from scripts.gap9_mapping_diagnostic import (
    mapping_source_paths,
    validate_mapping_diagnostic,
)
from scripts.source_provenance import (
    collect_source_identity,
    source_identity_from_mapping,
)
from scripts.validation_provenance import (
    BuildIdentity,
    DependencyDisposition,
    EvidenceClass,
    ExecutableIdentity,
    RunStatus,
    RuntimeDependency,
    ValidationManifest,
    _execution,
    parse_manifest_json,
    serialize_manifest,
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


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _strict_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number is forbidden: {value}")


def _validate_gvsoc_manifest_join(
    artifact: dict[str, Any],
    manifest_bytes: bytes,
    stdout: bytes,
    stderr: bytes,
) -> None:
    reference = artifact.get("provenance_manifest")
    if not isinstance(reference, dict):
        raise ValueError("GVSOC report does not reference its provenance manifest")
    if reference.get("sha256") != sha256_bytes(manifest_bytes):
        raise ValueError("GVSOC report manifest hash does not match its sidecar")
    manifest = parse_manifest_json(manifest_bytes.decode("utf-8"))
    expected_status = "success" if artifact["status"] == "passed" else "fail"
    if manifest.status.value != expected_status:
        raise ValueError("GVSOC report and manifest statuses disagree")
    if manifest.evidence_class.value != artifact["evidence_class"]:
        raise ValueError("GVSOC report and manifest evidence classes disagree")
    if manifest.build.executable is None:
        raise ValueError("GVSOC manifest executable identity is missing")
    if manifest.build.executable.sha256 != artifact["build"]["binary_sha256"]:
        raise ValueError("GVSOC report and manifest ELF hashes disagree")
    execution = manifest.execution
    if execution.exit_code != artifact["execution"]["exit_code"]:
        raise ValueError("GVSOC report and manifest exit codes disagree")
    if (
        execution.stdout_sha256 != sha256_bytes(stdout)
        or execution.stdout_bytes != len(stdout)
        or execution.stderr_sha256 != sha256_bytes(stderr)
        or execution.stderr_bytes != len(stderr)
    ):
        raise ValueError("GVSOC manifest stream identities do not match captured logs")


def validate_gvsoc_mapping_bundle(report_path: Path) -> ValidationManifest:
    """Load a retained GVSOC report and verify its log and manifest sidecars."""
    report_file = report_path.resolve(strict=True)
    report = json.loads(
        report_file.read_text(encoding="utf-8"),
        object_pairs_hook=_strict_json_object,
        parse_constant=_reject_json_constant,
    )
    validate_mapping_diagnostic(report)
    if report["evidence_class"] != "gvsoc-target-execution":
        raise ValueError("mapping bundle is not GVSOC evidence")
    reference = report.get("provenance_manifest")
    if not isinstance(reference, dict):
        raise ValueError("legacy GVSOC report has no canonical provenance manifest")
    relative = PurePosixPath(reference["path"])
    if (
        relative.is_absolute()
        or relative.as_posix() != reference["path"]
        or ".." in relative.parts
    ):
        raise ValueError("manifest path must be normalized and report-relative")
    manifest_path = (report_file.parent / Path(*relative.parts)).resolve(strict=True)
    if not manifest_path.is_relative_to(report_file.parent.resolve(strict=True)):
        raise ValueError("manifest resolves outside the report directory")
    manifest_bytes = manifest_path.read_bytes()
    if reference["sha256"] != sha256_bytes(manifest_bytes):
        raise ValueError("report manifest hash disagrees with sidecar bytes")
    manifest = parse_manifest_json(manifest_bytes.decode("utf-8"))
    log_bytes = report_file.with_suffix(".log").read_bytes()
    stdout_length = manifest.execution.stdout_bytes
    stderr_length = manifest.execution.stderr_bytes
    if stdout_length is None or stderr_length is None:
        raise ValueError("GVSOC manifest omits retained process stream lengths")
    if len(log_bytes) != stdout_length + stderr_length:
        raise ValueError("combined GVSOC log length disagrees with stream lengths")
    if report["execution"]["log_bytes"] != len(log_bytes) or report["execution"][
        "log_sha256"
    ] != sha256_bytes(log_bytes):
        raise ValueError("GVSOC report log identity disagrees with retained bytes")
    _validate_gvsoc_manifest_join(
        report,
        manifest_bytes,
        log_bytes[:stdout_length],
        log_bytes[stdout_length:],
    )
    return manifest


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
    dirty_output = _git_worktree_dirty(sdk_root)
    if not revision or not release:
        raise ValueError("could not establish GAP SDK revision and release")
    return {
        "revision": revision,
        "release": release,
        "worktree_dirty": dirty_output,
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


def _git_worktree_dirty(directory: Path) -> bool:
    result = subprocess.run(  # noqa: S603 -- fixed git argv and SDK directory
        ("git", "-C", str(directory), "diff-index", "--quiet", "HEAD", "--"),  # noqa: S607
        check=False,
    )
    if result.returncode not in (0, 1):
        raise ValueError("GAP SDK git diff-index failed")
    return result.returncode == 1


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


def _publish_atomically(
    output: Path,
    artifact: dict[str, Any],
    log: bytes | None = None,
    provenance: bytes | None = None,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if log is None and provenance is None:
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
        return
    if log is None or provenance is None:
        raise ValueError("canonical GVSOC bundle requires both log and manifest")
    log_output = output.with_suffix(".log")
    provenance_output = output.with_suffix(".provenance.json")
    outputs = (log_output, output, provenance_output)
    if any(path.exists() for path in outputs):
        raise FileExistsError("diagnostic bundle already contains an artifact")
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
    parser.add_argument("--source-receipt", type=Path)
    parser.add_argument("--stdout-log", type=Path)
    parser.add_argument("--stderr-log", type=Path)
    parser.add_argument("--model-root", type=Path)
    parser.add_argument("--gvsoc-exit-code", required=True, type=int)
    args = parser.parse_args()

    try:
        elf = args.elf.resolve(strict=True)
        log = args.log.resolve(strict=True)
        compile_commands = args.compile_commands.resolve(strict=True)
        build_provenance_file = args.build_provenance.resolve(strict=True)
        sdk_root = args.sdk_root.resolve(strict=True)
        stdout_log = args.stdout_log.resolve(strict=True) if args.stdout_log else None
        stderr_log = args.stderr_log.resolve(strict=True) if args.stderr_log else None
        model_root = args.model_root.resolve(strict=True) if args.model_root else None
        if (
            not elf.is_file()
            or not compile_commands.is_file()
            or not log.is_file()
            or not build_provenance_file.is_file()
        ):
            raise ValueError(
                "ELF, compile commands, build provenance, and GVSOC log must be files"
            )
        source_identity = None
        process_execution = None
        receipt_path: Path | None = None
        if args.source_receipt is not None:
            receipt_path = args.source_receipt.resolve(strict=True)
            receipt_bytes = receipt_path.read_bytes()
            receipt = json.loads(
                receipt_bytes.decode("utf-8"),
                object_pairs_hook=_strict_json_object,
                parse_constant=_reject_json_constant,
            )
            if (
                not isinstance(receipt, dict)
                or set(receipt) != {"schema_version", "source", "execution"}
                or isinstance(receipt["schema_version"], bool)
                or not isinstance(receipt["schema_version"], int)
                or receipt["schema_version"] != 2
            ):
                raise ValueError("GVSOC source receipt does not match its schema")
            identity = source_identity_from_mapping(receipt["source"])
            execution = _execution(receipt["execution"])
            if collect_source_identity(REPOSITORY_ROOT) != identity:
                raise ValueError("repository source differs from the in-run snapshot")
            if stdout_log is None or stderr_log is None or model_root is None:
                raise ValueError(
                    "canonical GVSOC capture requires stdout, stderr, and model root"
                )
            if execution.exit_code != args.gvsoc_exit_code:
                raise ValueError("GVSOC exit code differs from the in-run receipt")
            if model_root.is_dir() is False:
                raise ValueError("GVSOC model root is not a directory")
            source_identity = identity
            process_execution = execution
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
            "source_receipt": sha256_file(receipt_path) if receipt_path else None,
            "stdout": sha256_file(stdout_log) if stdout_log else None,
            "stderr": sha256_file(stderr_log) if stderr_log else None,
            "gvsoc_model": (
                sha256_file(model_root / "gap9/soc.dylib")
                if model_root is not None
                else None
            ),
            "gvsoc_launcher": sha256_file(
                sdk_root / "install/workstation/bin/gvsoc_launcher"
            )
            if receipt_path
            else None,
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
        stdout_bytes = stdout_log.read_bytes() if stdout_log else b""
        stderr_bytes = stderr_log.read_bytes() if stderr_log else b""
        if receipt_path is not None:
            if log_bytes != stdout_bytes + stderr_bytes:
                raise ValueError(
                    "combined log does not equal stdout followed by stderr"
                )
            if process_execution is None:
                raise ValueError("GVSOC process execution receipt is missing")
            if (
                process_execution.stdout_sha256 != sha256_bytes(stdout_bytes)
                or process_execution.stdout_bytes != len(stdout_bytes)
                or process_execution.stderr_sha256 != sha256_bytes(stderr_bytes)
                or process_execution.stderr_bytes != len(stderr_bytes)
            ):
                raise ValueError("GVSOC streams disagree with the in-run receipt")
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
        elif args.gvsoc_exit_code == 1:
            status = "reproduced-limitation"
        else:
            raise ValueError("GVSOC failure does not match the known ICP limitation")
        after = {
            "elf": sha256_file(elf),
            "log": sha256_file(log),
            "compile_commands": sha256_file(compile_commands),
            "build_provenance": sha256_file(build_provenance_file),
            "source_receipt": sha256_file(receipt_path) if receipt_path else None,
            "stdout": sha256_file(stdout_log) if stdout_log else None,
            "stderr": sha256_file(stderr_log) if stderr_log else None,
            "gvsoc_model": (
                sha256_file(model_root / "gap9/soc.dylib")
                if model_root is not None
                else None
            ),
            "gvsoc_launcher": sha256_file(
                sdk_root / "install/workstation/bin/gvsoc_launcher"
            )
            if receipt_path
            else None,
        }
        if before != after:
            raise ValueError("ELF, log, or compile commands changed during capture")
        if source_hashes != {
            path.relative_to(REPOSITORY_ROOT).as_posix(): sha256_file(path)
            for path in mapping_source_paths(REPOSITORY_ROOT)
        }:
            raise ValueError("mapping source changed during capture")
        if receipt_path is not None and before["source_receipt"] != sha256_file(
            receipt_path
        ):
            raise ValueError("GVSOC source receipt changed during capture")
        if (
            source_identity is not None
            and collect_source_identity(REPOSITORY_ROOT) != source_identity
        ):
            raise ValueError("repository source changed after the in-run snapshot")
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
        provenance_bytes = b""
        if source_identity is not None and process_execution is not None:
            if model_root is None:
                raise ValueError("GVSOC model root is required for provenance")
            model_plugin = model_root / "gap9/soc.dylib"
            launcher = sdk_root / "install/workstation/bin/gvsoc_launcher"
            if not model_plugin.is_file() or not launcher.is_file():
                raise ValueError("GVSOC model plugin or launcher is missing")
            try:
                executable_path = elf.relative_to(REPOSITORY_ROOT).as_posix()
            except ValueError:
                executable_path = None
            manifest = ValidationManifest(
                schema_version=1,
                status=(RunStatus.SUCCESS if status == "passed" else RunStatus.FAIL),
                evidence_class=EvidenceClass.GVSOC,
                claim_boundary=(
                    artifact["claim_boundary"]
                    + " The manifest records selected runtime environment keys; "
                    "other inherited environment values are not captured. The "
                    "source identity describes the target execution tree; this "
                    "build receipt does not attest that the ELF was compiled "
                    "from that exact root-tree snapshot."
                ),
                captured_at_utc=datetime.now(UTC).isoformat().replace("+00:00", "Z"),
                source=source_identity,
                build=BuildIdentity(
                    command=None,
                    configuration_sha256=before["build_provenance"],
                    toolchain=None,
                    executable=ExecutableIdentity(executable_path, before["elf"]),
                ),
                runtime_dependencies=(
                    RuntimeDependency(
                        name="gap9-gvsoc-model-plugin",
                        disposition=DependencyDisposition.SYSTEM_UNPINNED,
                        version=None,
                        sha256=before["gvsoc_model"],
                        path=None,
                    ),
                    RuntimeDependency(
                        name="gvsoc-launcher",
                        disposition=DependencyDisposition.SYSTEM_UNPINNED,
                        version=None,
                        sha256=before["gvsoc_launcher"],
                        path=None,
                    ),
                ),
                execution=process_execution,
                hardware_attestation=None,
            )
            provenance_bytes = serialize_manifest(manifest).encode("utf-8")
            artifact["provenance_manifest"] = {
                "path": output.with_suffix(".provenance.json").name,
                "sha256": sha256_bytes(provenance_bytes),
            }
            _validate_gvsoc_manifest_join(
                artifact,
                provenance_bytes,
                stdout_bytes,
                stderr_bytes,
            )
        validate_mapping_diagnostic(artifact)
        if provenance_bytes:
            if retained_log != output.with_suffix(".log"):
                raise ValueError("canonical GVSOC bundle uses the adjacent .log path")
            _publish_atomically(output, artifact, log_bytes, provenance_bytes)
        else:
            _publish_bytes_atomically(retained_log, log_bytes)
            _publish_atomically(output, artifact)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"GVSOC mapping diagnostic capture failed: {error}")
        return 1
    print(f"GVSOC mapping diagnostic written: {output}; raw log: {retained_log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
