"""Canonical manifest construction and offline verification for Gazebo runs."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import UTC, datetime
from pathlib import Path, PurePosixPath

from scripts.source_provenance import (
    SourceIdentity,
    source_identity_from_mapping,
)
from scripts.validation_provenance import (
    BuildIdentity,
    DependencyDisposition,
    EvidenceClass,
    ExecutableIdentity,
    ExecutionIdentity,
    RunStatus,
    RuntimeDependency,
    ValidationManifest,
    parse_manifest_json,
    serialize_manifest,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.run_provenance import (
    validate_combined_run_report,
)


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="Gazebo mission JSON report")
    options = parser.parse_args(args)
    try:
        manifest = validate_gazebo_mapping_bundle(options.report)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Gazebo bundle verification failed: {error}\n")
    print(
        f"Verified {manifest.evidence_class.value} {manifest.status.value} bundle: "
        f"{options.report.resolve()}"
    )
    return 0


def _digest(contents: bytes) -> str:
    return hashlib.sha256(contents).hexdigest()


def _mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"Gazebo manifest input is missing {name}")
    return value


def create_gazebo_run_manifest(
    report: dict[str, object],
    *,
    source: SourceIdentity,
    stream_contents: dict[str, bytes],
    started_at_utc: str | None = None,
    finished_at_utc: str | None = None,
) -> bytes:
    """Build a success-only manifest joined to report execution identities."""
    run_provenance = _mapping(report.get("run_provenance"), "runtime provenance")
    executables = _mapping(run_provenance.get("executables"), "executables")
    mapping = _mapping(executables.get("mapping_binary"), "mapping executable")
    overlays = _mapping(run_provenance.get("ros_overlay_packages"), "ROS overlays")
    environment = _mapping(
        run_provenance.get("runtime_environment"), "runtime environment"
    )
    commands = _mapping(report.get("child_commands"), "child commands")
    _mapping(report.get("process_streams"), "process streams")

    runtime_dependencies: list[RuntimeDependency] = []
    for name, executable in executables.items():
        executable = _mapping(executable, f"executable {name}")
        version = executable.get("version") or executable.get("sim_version")
        if name == "ros2":
            version = executable.get("cli_version")
        elif name == "python":
            version = executable.get("version")
        elif name == "mapping_binary":
            continue
        runtime_dependencies.append(
            RuntimeDependency(
                name=f"executable:{name}",
                disposition=DependencyDisposition.SYSTEM_UNPINNED,
                version=version if isinstance(version, str) and version else None,
                sha256=str(executable["sha256"]),
                path=None,
            )
        )
    for package, identity in overlays.items():
        identity = _mapping(identity, f"overlay {package}")
        runtime_dependencies.append(
            RuntimeDependency(
                name=f"ros-overlay:{package}",
                disposition=DependencyDisposition.SYSTEM_UNPINNED,
                version=None,
                sha256=str(identity["sha256"]),
                path=None,
            )
        )

    binary_path = Path(str(mapping["path"]))
    repository_root = Path(__file__).resolve().parents[1]
    relative_binary = (
        binary_path.resolve().relative_to(repository_root).as_posix()
        if binary_path.resolve().is_relative_to(repository_root)
        else None
    )
    combined_stdout = b"".join(
        stream_contents[f"{child}.stdout"] for child in ("collector", "mission")
    )
    combined_stderr = b"".join(
        stream_contents[f"{child}.stderr"] for child in ("collector", "mission")
    )
    finished = finished_at_utc or datetime.now(UTC).isoformat().replace("+00:00", "Z")
    started = started_at_utc or finished
    manifest = ValidationManifest(
        schema_version=1,
        status=RunStatus.SUCCESS,
        evidence_class=EvidenceClass.GAZEBO,
        claim_boundary=(
            "Synthetic Gazebo telemetry replay through the host Nano-C-SLAM core; "
            "not target firmware, mapping accuracy, flight, or hardware validation."
        ),
        captured_at_utc=finished,
        source=source,
        build=BuildIdentity(
            command=None,
            configuration_sha256=None,
            toolchain=None,
            executable=ExecutableIdentity(
                path=relative_binary,
                sha256=str(mapping["sha256"]),
            ),
        ),
        runtime_dependencies=tuple(runtime_dependencies),
        execution=ExecutionIdentity(
            command=tuple(
                [*commands["collector"], "--then-command--", *commands["mission"]]
            ),
            environment=tuple(
                sorted(
                    (key, value if isinstance(value, str) else None)
                    for key, value in {
                        "ROS_DOMAIN_ID": environment.get("ros_domain_id"),
                        "GZ_PARTITION": environment.get("gazebo_partition"),
                        "RMW_IMPLEMENTATION": environment.get("rmw_implementation"),
                    }.items()
                )
            ),
            started_at_utc=started,
            finished_at_utc=finished,
            exit_code=0,
            stdout_sha256=_digest(combined_stdout),
            stdout_bytes=len(combined_stdout),
            stderr_sha256=_digest(combined_stderr),
            stderr_bytes=len(combined_stderr),
        ),
        hardware_attestation=None,
    )
    return serialize_manifest(manifest).encode("utf-8")


def validate_gazebo_mapping_bundle(report_path: Path) -> ValidationManifest:
    """Verify report/manifest/process-stream hashes and critical run joins."""
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report = validate_combined_run_report(report)
    run_id = report.get("run_id")
    if not isinstance(run_id, str):
        raise ValueError("Gazebo report is missing run ID")
    reference = report.get("provenance_manifest")
    if not isinstance(reference, dict) or set(reference) != {"path", "sha256"}:
        raise ValueError("Gazebo report is missing manifest reference")
    manifest_relative = _safe_relative_path(reference["path"])
    manifest_path = (
        report_path.resolve(strict=True).parent / manifest_relative
    ).resolve(strict=True)
    report_directory = report_path.resolve(strict=True).parent
    if not manifest_path.is_relative_to(report_directory):
        raise ValueError("manifest sidecar resolves outside report directory")
    manifest_bytes = manifest_path.read_bytes()
    if reference["sha256"] != _digest(manifest_bytes):
        raise ValueError("manifest sidecar hash disagrees with report")
    manifest = parse_manifest_json(manifest_bytes.decode("utf-8"))
    if (
        manifest.evidence_class is not EvidenceClass.GAZEBO
        or manifest.status is not RunStatus.SUCCESS
    ):
        raise ValueError("Gazebo bundle requires successful Gazebo evidence")
    process_exit_codes = report.get("process_exit_codes")
    if process_exit_codes != {"collector": 0, "mission": 0}:
        raise ValueError(
            "successful Gazebo bundle requires both child exits to be zero"
        )
    capture = report.get("capture_report")
    provenance = report.get("run_provenance")
    streams = report.get("process_streams")
    if not isinstance(capture, dict) or capture.get("run_id") != run_id:
        raise ValueError("capture and report run IDs do not match")
    if not isinstance(provenance, dict) or not isinstance(streams, dict):
        raise ValueError("Gazebo report is missing runtime or process stream evidence")
    report_source = source_identity_from_mapping(report.get("source_identity"))
    if report_source != manifest.source:
        raise ValueError("manifest source identity differs from report")
    commands = report.get("child_commands")
    if not isinstance(commands, dict):
        raise ValueError("Gazebo report is missing child commands")
    expected_command = tuple(
        [
            *commands.get("collector", []),
            "--then-command--",
            *commands.get("mission", []),
        ]
    )
    if manifest.execution.command != expected_command:
        raise ValueError("manifest commands differ from report child commands")
    executables = provenance.get("executables")
    if not isinstance(executables, dict):
        raise ValueError("Gazebo report is missing executable identities")
    mapping_binary = executables.get("mapping_binary")
    capture_provenance = capture.get("mapping_replay_provenance")
    manifest_executable = manifest.build.executable
    if (
        not isinstance(mapping_binary, dict)
        or not isinstance(capture_provenance, dict)
        or manifest_executable is None
        or manifest_executable.sha256 != mapping_binary.get("sha256")
        or manifest_executable.sha256 != capture_provenance.get("binary_sha256")
    ):
        raise ValueError("manifest mapping executable differs from capture/report")
    dependencies = {
        dependency.name: dependency for dependency in manifest.runtime_dependencies
    }
    overlays = provenance.get("ros_overlay_packages")
    if not isinstance(overlays, dict):
        raise ValueError("Gazebo report is missing ROS overlay identities")
    for executable_name in ("python", "ros2", "gazebo"):
        identity = executables.get(executable_name)
        dependency = dependencies.get(f"executable:{executable_name}")
        if (
            not isinstance(identity, dict)
            or dependency is None
            or dependency.sha256 != identity.get("sha256")
        ):
            raise ValueError("manifest executable runtime identity differs from report")
    for package, identity in overlays.items():
        dependency = dependencies.get(f"ros-overlay:{package}")
        if (
            not isinstance(identity, dict)
            or dependency is None
            or dependency.sha256 != identity.get("sha256")
        ):
            raise ValueError("manifest ROS overlay identity differs from report")
    owned_hashes = provenance.get("source", {}).get("owned_input_sha256")
    interface_identity = provenance.get("interface_identity")
    if not isinstance(owned_hashes, dict) or not isinstance(interface_identity, dict):
        raise ValueError("Gazebo report is missing interface source identity")
    interface_hashes = interface_identity.get("source_sha256")
    if not isinstance(interface_hashes, dict):
        raise ValueError("Gazebo report is missing ROS interface source hashes")
    for interface_name, source_path in (
        (
            "nano_c_slam_interfaces/msg/StampedRanges.msg",
            "sim/gazebo/nano_c_slam_interfaces/msg/StampedRanges.msg",
        ),
        (
            "nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg",
            "sim/gazebo/nano_c_slam_interfaces/msg/StampedPairwiseRanges.msg",
        ),
    ):
        if interface_hashes.get(interface_name) != owned_hashes.get(source_path):
            raise ValueError("ROS interface identity differs from owned source hash")
    environment = provenance.get("runtime_environment")
    if not isinstance(environment, dict):
        raise ValueError("Gazebo report is missing runtime environment")
    manifest_environment = dict(manifest.execution.environment)
    for key, field in (
        ("ROS_DOMAIN_ID", "ros_domain_id"),
        ("GZ_PARTITION", "gazebo_partition"),
        ("RMW_IMPLEMENTATION", "rmw_implementation"),
    ):
        if manifest_environment.get(key) != environment.get(field):
            raise ValueError(f"manifest {key} does not match report runtime")

    content_by_name: dict[str, bytes] = {}
    for child in ("collector", "mission"):
        for stream in ("stdout", "stderr"):
            key = f"{child}.{stream}"
            record = streams.get(key)
            if not isinstance(record, dict) or set(record) != {
                "path",
                "sha256",
                "bytes",
            }:
                raise ValueError(f"invalid process stream record: {key}")
            relative = _safe_relative_path(record["path"])
            path = (report_directory / relative).resolve(strict=True)
            if not path.is_relative_to(report_directory):
                raise ValueError(
                    f"process stream resolves outside report directory: {key}"
                )
            contents = path.read_bytes()
            if record["sha256"] != _digest(contents) or record["bytes"] != len(
                contents
            ):
                raise ValueError(f"process stream changed after capture: {key}")
            content_by_name[key] = contents
    stdout = b"".join(
        content_by_name[f"{child}.stdout"] for child in ("collector", "mission")
    )
    stderr = b"".join(
        content_by_name[f"{child}.stderr"] for child in ("collector", "mission")
    )
    if manifest.execution.stdout_sha256 != _digest(
        stdout
    ) or manifest.execution.stdout_bytes != len(stdout):
        raise ValueError(
            "manifest stdout identity disagrees with retained process streams"
        )
    if manifest.execution.stderr_sha256 != _digest(
        stderr
    ) or manifest.execution.stderr_bytes != len(stderr):
        raise ValueError(
            "manifest stderr identity disagrees with retained process streams"
        )
    raw_logs = report.get("raw_logs")
    if not isinstance(raw_logs, dict):
        raise ValueError("Gazebo report is missing compatibility logs")
    for child in ("collector", "mission"):
        combined = (
            content_by_name[f"{child}.stdout"] + content_by_name[f"{child}.stderr"]
        ).decode("utf-8")
        if raw_logs.get(child) != combined:
            raise ValueError(f"embedded {child} log differs from retained streams")
    return manifest


def _safe_relative_path(value: object) -> Path:
    if not isinstance(value, str):
        raise ValueError("artifact path must be a string")
    path = PurePosixPath(value)
    if (
        path.is_absolute()
        or not path.parts
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise ValueError("artifact path must be normalized and relative")
    return Path(*path.parts)


if __name__ == "__main__":
    raise SystemExit(main())
