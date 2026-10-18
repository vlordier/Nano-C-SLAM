"""Typed schema for provenance-bound validation run manifests."""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime
from enum import StrEnum
from typing import cast

SCHEMA_VERSION = 1
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_GIT_REVISION = re.compile(r"^[0-9a-f]{40,64}$")


class EvidenceClass(StrEnum):
    HOST = "host-pmsis-stubs"
    GVSOC = "gvsoc-target-execution"
    GAZEBO = "gazebo-simulation"
    PHYSICAL_GAP9 = "physical-gap9"
    PHYSICAL_FLIGHT = "physical-flight"


class RunStatus(StrEnum):
    SUCCESS = "success"
    FAIL = "fail"
    BLOCKED = "blocked"
    NOT_RUN = "not-run"


class DependencyDisposition(StrEnum):
    EMBEDDED = "embedded"
    CHECKED_IN = "checked-in"
    IMMUTABLE_DOWNLOAD = "immutable-download"
    BUILT_FROM_SOURCE = "built-from-source"
    SYSTEM_UNPINNED = "system-unpinned"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class SubmoduleIdentity:
    path: str
    gitlink_revision: str
    checkout_revision: str
    dirty: bool
    dirty_patch_sha256: str | None


@dataclass(frozen=True)
class SourceIdentity:
    revision: str
    dirty: bool
    dirty_patch_sha256: str | None
    submodules: tuple[SubmoduleIdentity, ...]


@dataclass(frozen=True)
class ExecutableIdentity:
    path: str
    sha256: str


@dataclass(frozen=True)
class ToolchainIdentity:
    name: str
    version: str
    executable_sha256: str


@dataclass(frozen=True)
class BuildIdentity:
    command: tuple[str, ...]
    configuration_sha256: str | None
    toolchain: ToolchainIdentity | None
    executable: ExecutableIdentity | None


@dataclass(frozen=True)
class RuntimeDependency:
    name: str
    disposition: DependencyDisposition
    version: str | None
    sha256: str | None
    path: str | None


@dataclass(frozen=True)
class ExecutionIdentity:
    command: tuple[str, ...]
    environment: tuple[tuple[str, str | None], ...]
    started_at_utc: str | None
    finished_at_utc: str | None
    exit_code: int | None
    stdout_sha256: str | None
    stderr_sha256: str | None


@dataclass(frozen=True)
class HardwareAttestation:
    device_id_sha256: str
    firmware_sha256: str
    operator_id_sha256: str
    raw_measurements_sha256: str
    observed_at_utc: str


@dataclass(frozen=True)
class ValidationManifest:
    schema_version: int
    status: RunStatus
    evidence_class: EvidenceClass
    claim_boundary: str
    captured_at_utc: str
    source: SourceIdentity
    build: BuildIdentity
    runtime_dependencies: tuple[RuntimeDependency, ...]
    execution: ExecutionIdentity
    hardware_attestation: HardwareAttestation | None


def _object(value: object, name: str, fields: set[str]) -> dict[str, object]:
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError(f"{name} must contain exactly: {', '.join(sorted(fields))}")
    return cast(dict[str, object], value)


def _text(value: object, name: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str) or (not allow_empty and not value):
        raise ValueError(
            f"{name} must be a {'possibly empty ' if allow_empty else ''}string"
        )
    return value


def _repo_path(value: object, name: str) -> str:
    path = _text(value, name)
    parts = path.split("/")
    if path.startswith("/") or "\\" in path or any(p in {"", ".", ".."} for p in parts):
        raise ValueError(f"{name} must be a normalized repository-relative path")
    return path


def _optional_text(
    value: object, name: str, *, allow_empty: bool = False
) -> str | None:
    return None if value is None else _text(value, name, allow_empty=allow_empty)


def _sha256(value: object, name: str) -> str:
    digest = _text(value, name)
    if _SHA256.fullmatch(digest) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")
    return digest


def _optional_sha256(value: object, name: str) -> str | None:
    return None if value is None else _sha256(value, name)


def _revision(value: object, name: str) -> str:
    revision = _text(value, name)
    if _GIT_REVISION.fullmatch(revision) is None:
        raise ValueError(f"{name} must be a full Git revision")
    return revision


def _timestamp(value: object, name: str, *, nullable: bool = False) -> str | None:
    timestamp = _optional_text(value, name)
    if timestamp is None:
        if nullable:
            return None
        raise ValueError(f"{name} is required")
    try:
        parsed = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError(f"{name} must be an ISO-8601 timestamp") from error
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError(f"{name} must include a UTC offset")
    return timestamp


def _required_timestamp(value: object, name: str) -> str:
    timestamp = _timestamp(value, name)
    if timestamp is None:
        raise ValueError(f"{name} is required")
    return timestamp


def _dirty_state(
    dirty_value: object, patch_value: object, name: str
) -> tuple[bool, str | None]:
    if not isinstance(dirty_value, bool):
        raise ValueError(f"{name}.dirty must be a boolean")
    patch_hash = _optional_sha256(patch_value, f"{name}.dirty_patch_sha256")
    if dirty_value != (patch_hash is not None):
        raise ValueError(f"{name} dirty flag must agree with its patch hash")
    return dirty_value, patch_hash


def _string_list(value: object, name: str, *, allow_empty: bool) -> tuple[str, ...]:
    if not isinstance(value, list) or (not allow_empty and not value):
        raise ValueError(
            f"{name} must be {'a possibly empty' if allow_empty else 'a nonempty'} list"
        )
    return tuple(_text(item, f"{name}[]") for item in value)


def _build(value: object) -> BuildIdentity:
    raw = _object(
        value, "build", {"command", "configuration_sha256", "toolchain", "executable"}
    )
    command = _string_list(raw["command"], "build.command", allow_empty=True)
    config_hash = _optional_sha256(
        raw["configuration_sha256"], "build.configuration_sha256"
    )
    toolchain_raw = raw["toolchain"]
    toolchain = None
    if toolchain_raw is not None:
        fields = {"name", "version", "executable_sha256"}
        data = _object(toolchain_raw, "build.toolchain", fields)
        toolchain = ToolchainIdentity(
            name=_text(data["name"], "build.toolchain.name"),
            version=_text(data["version"], "build.toolchain.version"),
            executable_sha256=_sha256(
                data["executable_sha256"], "build.toolchain.executable_sha256"
            ),
        )
    executable_raw = raw["executable"]
    executable = None
    if executable_raw is not None:
        data = _object(executable_raw, "build.executable", {"path", "sha256"})
        executable = ExecutableIdentity(
            path=_repo_path(data["path"], "build.executable.path"),
            sha256=_sha256(data["sha256"], "build.executable.sha256"),
        )
    return BuildIdentity(command, config_hash, toolchain, executable)


def _source(value: object) -> SourceIdentity:
    raw = _object(
        value,
        "source",
        {"revision", "dirty", "dirty_patch_sha256", "submodules"},
    )
    dirty, patch_hash = _dirty_state(raw["dirty"], raw["dirty_patch_sha256"], "source")
    submodules_raw = raw["submodules"]
    if not isinstance(submodules_raw, list):
        raise ValueError("source.submodules must be a list")
    submodules: list[SubmoduleIdentity] = []
    paths: set[str] = set()
    for index, value in enumerate(submodules_raw):
        name = f"source.submodules[{index}]"
        data = _object(
            value,
            name,
            {
                "path",
                "gitlink_revision",
                "checkout_revision",
                "dirty",
                "dirty_patch_sha256",
            },
        )
        path = _repo_path(data["path"], f"{name}.path")
        if path in paths:
            raise ValueError(f"{name}.path must be unique and repository-relative")
        paths.add(path)
        submodule_dirty, submodule_patch_hash = _dirty_state(
            data["dirty"], data["dirty_patch_sha256"], name
        )
        submodules.append(
            SubmoduleIdentity(
                path=path,
                gitlink_revision=_revision(
                    data["gitlink_revision"], f"{name}.gitlink_revision"
                ),
                checkout_revision=_revision(
                    data["checkout_revision"], f"{name}.checkout_revision"
                ),
                dirty=submodule_dirty,
                dirty_patch_sha256=submodule_patch_hash,
            )
        )
    return SourceIdentity(
        revision=_revision(raw["revision"], "source.revision"),
        dirty=dirty,
        dirty_patch_sha256=patch_hash,
        submodules=tuple(submodules),
    )


def _runtime_dependencies(raw_value: object) -> tuple[RuntimeDependency, ...]:
    if not isinstance(raw_value, list):
        raise ValueError("runtime_dependencies must be a list")
    dependencies: list[RuntimeDependency] = []
    names: set[str] = set()
    for index, dependency_value in enumerate(raw_value):
        name = f"runtime_dependencies[{index}]"
        data = _object(
            dependency_value,
            name,
            {"name", "disposition", "version", "sha256", "path"},
        )
        dependency_name = _text(data["name"], f"{name}.name")
        if dependency_name in names:
            raise ValueError(f"duplicate runtime dependency: {dependency_name}")
        names.add(dependency_name)
        try:
            disposition = DependencyDisposition(
                _text(data["disposition"], f"{name}.disposition")
            )
        except ValueError as error:
            raise ValueError(f"{name}.disposition is unsupported") from error
        digest = _optional_sha256(data["sha256"], f"{name}.sha256")
        if disposition is DependencyDisposition.UNKNOWN and digest is not None:
            raise ValueError(f"{name} unknown dependency cannot claim a content hash")
        if disposition is not DependencyDisposition.UNKNOWN and digest is None:
            raise ValueError(f"{name} requires a content hash for its disposition")
        dependencies.append(
            RuntimeDependency(
                name=dependency_name,
                disposition=disposition,
                version=_optional_text(data["version"], f"{name}.version"),
                sha256=digest,
                path=(
                    None
                    if data["path"] is None
                    else _repo_path(data["path"], f"{name}.path")
                ),
            )
        )
    return tuple(dependencies)


def _execution(value: object) -> ExecutionIdentity:
    raw = _object(
        value,
        "execution",
        {
            "command",
            "environment",
            "started_at_utc",
            "finished_at_utc",
            "exit_code",
            "stdout_sha256",
            "stderr_sha256",
        },
    )
    command = _string_list(raw["command"], "execution.command", allow_empty=True)
    environment = raw["environment"]
    if not isinstance(environment, dict):
        raise ValueError("execution.environment must be an object")
    environment_items = tuple(
        sorted(
            (
                _text(key, "execution.environment key"),
                _optional_text(
                    value, f"execution.environment[{key}]", allow_empty=True
                ),
            )
            for key, value in environment.items()
        )
    )
    started = _timestamp(
        raw["started_at_utc"], "execution.started_at_utc", nullable=True
    )
    finished = _timestamp(
        raw["finished_at_utc"], "execution.finished_at_utc", nullable=True
    )
    if (started is None) != (finished is None):
        raise ValueError(
            "execution start and finish timestamps must both be known or null"
        )
    exit_code = raw["exit_code"]
    if exit_code is not None and (
        isinstance(exit_code, bool) or not isinstance(exit_code, int)
    ):
        raise ValueError("execution.exit_code must be an integer or null")
    return ExecutionIdentity(
        command=command,
        environment=environment_items,
        started_at_utc=started,
        finished_at_utc=finished,
        exit_code=exit_code,
        stdout_sha256=_optional_sha256(raw["stdout_sha256"], "execution.stdout_sha256"),
        stderr_sha256=_optional_sha256(raw["stderr_sha256"], "execution.stderr_sha256"),
    )


def _hardware_attestation(value: object) -> HardwareAttestation | None:
    if value is None:
        return None
    data = _object(
        value,
        "hardware_attestation",
        {
            "device_id_sha256",
            "firmware_sha256",
            "operator_id_sha256",
            "raw_measurements_sha256",
            "observed_at_utc",
        },
    )
    return HardwareAttestation(
        device_id_sha256=_sha256(
            data["device_id_sha256"], "hardware_attestation.device_id_sha256"
        ),
        firmware_sha256=_sha256(
            data["firmware_sha256"], "hardware_attestation.firmware_sha256"
        ),
        operator_id_sha256=_sha256(
            data["operator_id_sha256"], "hardware_attestation.operator_id_sha256"
        ),
        raw_measurements_sha256=_sha256(
            data["raw_measurements_sha256"],
            "hardware_attestation.raw_measurements_sha256",
        ),
        observed_at_utc=_required_timestamp(
            data["observed_at_utc"], "hardware_attestation.observed_at_utc"
        ),
    )


def validate_manifest(value: object) -> ValidationManifest:
    """Validate exact manifest keys and class-dependent evidence requirements."""
    raw = _object(
        value,
        "manifest",
        {
            "schema_version",
            "status",
            "evidence_class",
            "claim_boundary",
            "captured_at_utc",
            "source",
            "build",
            "runtime_dependencies",
            "execution",
            "hardware_attestation",
        },
    )
    if (
        isinstance(raw["schema_version"], bool)
        or not isinstance(raw["schema_version"], int)
        or raw["schema_version"] != SCHEMA_VERSION
    ):
        raise ValueError("unsupported manifest schema_version")
    try:
        status = RunStatus(_text(raw["status"], "status"))
        evidence_class = EvidenceClass(_text(raw["evidence_class"], "evidence_class"))
    except ValueError as error:
        raise ValueError("unsupported manifest status or evidence_class") from error
    captured_at = _required_timestamp(raw["captured_at_utc"], "captured_at_utc")
    manifest = ValidationManifest(
        schema_version=SCHEMA_VERSION,
        status=status,
        evidence_class=evidence_class,
        claim_boundary=_text(raw["claim_boundary"], "claim_boundary"),
        captured_at_utc=captured_at,
        source=_source(raw["source"]),
        build=_build(raw["build"]),
        runtime_dependencies=_runtime_dependencies(raw["runtime_dependencies"]),
        execution=_execution(raw["execution"]),
        hardware_attestation=_hardware_attestation(raw["hardware_attestation"]),
    )
    physical_class = evidence_class in {
        EvidenceClass.PHYSICAL_GAP9,
        EvidenceClass.PHYSICAL_FLIGHT,
    }
    if not physical_class and manifest.hardware_attestation is not None:
        raise ValueError("software evidence class cannot include hardware attestation")
    if status is RunStatus.SUCCESS:
        if physical_class:
            raise ValueError(
                "physical SUCCESS requires an authenticated device evidence collector"
            )
        execution = manifest.execution
        if (
            execution.exit_code != 0
            or execution.stdout_sha256 is None
            or execution.stderr_sha256 is None
            or execution.started_at_utc is None
            or execution.finished_at_utc is None
            or manifest.build.executable is None
        ):
            raise ValueError(
                "PASS requires a successful, hashed execution and executable"
            )
        started = datetime.fromisoformat(
            execution.started_at_utc.replace("Z", "+00:00")
        )
        finished = datetime.fromisoformat(
            execution.finished_at_utc.replace("Z", "+00:00")
        )
        if started > finished:
            raise ValueError("execution finish timestamp precedes its start")
        if evidence_class is EvidenceClass.GVSOC and not manifest.runtime_dependencies:
            raise ValueError(
                "GVSOC PASS requires explicit runtime dependency identities"
            )
        if evidence_class is EvidenceClass.GAZEBO and not manifest.runtime_dependencies:
            raise ValueError(
                "Gazebo PASS requires explicit runtime dependency identities"
            )
    if (
        physical_class
        and status is RunStatus.FAIL
        and manifest.hardware_attestation is None
    ):
        raise ValueError("physical FAIL requires hardware measurement attestation")
    return manifest
