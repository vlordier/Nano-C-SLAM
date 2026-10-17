"""Strict validation for shared host/GVSOC mapping diagnostic artifacts."""

from __future__ import annotations

import re
from collections.abc import Mapping
from pathlib import Path, PurePosixPath
from typing import Any

SCHEMA_VERSION = 2
EVIDENCE_CLASSES = {"host-pmsis-stubs", "gvsoc-target-execution"}
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_GIT_REVISION = re.compile(r"^[0-9a-f]{40,64}$")
_FIXTURE_NAME = "gap9_sdk_gvsoc_mapping_smoke"
_FIXTURE_COUNTS = (60, 3, 240)
_REQUIRED_SOURCE_PATHS = {
    "tests/gap9_sdk_gvsoc_mapping_smoke.c",
    "cslam-gap9-app/icp/icp.c",
    "cslam-gap9-app/point-cloud/point_cloud.c",
    "cslam-gap9-app/slam/graph-based-slam.c",
}
_MAPPING_SOURCE_FILES = {
    "tests/gap9_sdk_gvsoc_mapping_smoke.c",
    "tests/gap9_gvsoc_sdk.config",
    "cslam-gap9-app/CMakeLists.txt",
    "artifacts/gap9_gvsoc_evidence/mapping_diagnostic.schema.json",
    "pyproject.toml",
    "uv.lock",
    "scripts/capture_gap9_gvsoc_mapping_diagnostic.py",
    "scripts/write_gap9_build_provenance.py",
    "scripts/gap9_mapping_diagnostic.py",
    "scripts/gapy_private_wrapper.py",
    "scripts/create_gap9_gvsoc_layout.py",
    "scripts/run_gap9_gvsoc_target.py",
    "scripts/build_host_mapping_sitl.sh",
    "scripts/build_gap9_firmware.sh",
    "scripts/run_gap9_gvsoc_smoke.sh",
}
_MAPPING_SOURCE_DIRECTORIES = (
    "cslam-gap9-app/dma",
    "cslam-gap9-app/host-sitl",
    "cslam-gap9-app/icp",
    "cslam-gap9-app/point-cloud",
    "cslam-gap9-app/slam",
    "tests/stubs",
)
_TOP_LEVEL_KEYS = {
    "schema_version",
    "status",
    "evidence_class",
    "claim_boundary",
    "hardware_measurement",
    "fixture",
    "build",
    "execution",
    "mapping",
}


def validate_mapping_diagnostic(value: object) -> dict[str, Any]:
    """Validate and return one exact version-1 host or GVSOC artifact."""
    data = _object(value, "artifact", _TOP_LEVEL_KEYS)
    if _integer(data["schema_version"], "schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported mapping diagnostic schema_version")
    evidence_class = data["evidence_class"]
    if not isinstance(evidence_class, str) or evidence_class not in EVIDENCE_CLASSES:
        raise ValueError("unsupported mapping diagnostic evidence_class")
    if not isinstance(data["status"], str) or data["status"] not in {
        "passed",
        "failed",
        "reproduced-limitation",
    }:
        raise ValueError("invalid mapping diagnostic status")
    _text(data["claim_boundary"], "claim_boundary")
    if data["hardware_measurement"] is not False:
        raise ValueError(
            "synthetic mapping diagnostic cannot claim hardware measurement"
        )

    fixture = _object(
        data["fixture"],
        "fixture",
        {"name", "source_sha256", "pose_count", "scan_count", "point_count"},
    )
    if _text(fixture["name"], "fixture.name") != _FIXTURE_NAME:
        raise ValueError("unsupported mapping diagnostic fixture")
    _hash(fixture["source_sha256"], "fixture.source_sha256")
    poses = _integer(fixture["pose_count"], "fixture.pose_count")
    scans = _integer(fixture["scan_count"], "fixture.scan_count")
    points = _integer(fixture["point_count"], "fixture.point_count")
    if (poses, scans, points) != _FIXTURE_COUNTS:
        raise ValueError(
            "fixture counts do not match the deterministic mapping fixture"
        )

    build = _object(
        data["build"],
        "build",
        {
            "toolchain_id",
            "compiler_flags",
            "binary_sha256",
            "source_sha256",
            "sdk_metadata",
            "compiler_commands_sha256",
        },
    )
    _text(build["toolchain_id"], "build.toolchain_id")
    _text(build["compiler_flags"], "build.compiler_flags")
    _hash(build["binary_sha256"], "build.binary_sha256")
    source_hashes = build["source_sha256"]
    if not isinstance(source_hashes, Mapping) or not source_hashes:
        raise ValueError("build.source_sha256 must be a nonempty path-to-hash object")
    for path, digest in source_hashes.items():
        _text(path, "build.source_sha256 path")
        source_path = PurePosixPath(path)
        if (
            source_path.is_absolute()
            or ".." in source_path.parts
            or source_path.as_posix() != path
        ):
            raise ValueError(
                f"build.source_sha256 path is not repository-relative: {path}"
            )
        _hash(digest, f"build.source_sha256[{path}]")
    if not _REQUIRED_SOURCE_PATHS.issubset(source_hashes):
        raise ValueError("build.source_sha256 omits a required mapping source")
    if (
        source_hashes["tests/gap9_sdk_gvsoc_mapping_smoke.c"]
        != fixture["source_sha256"]
    ):
        raise ValueError("fixture hash disagrees with build source provenance")
    sdk_metadata = build["sdk_metadata"]
    compiler_commands_hash = build["compiler_commands_sha256"]
    if evidence_class == "host-pmsis-stubs":
        if sdk_metadata is not None:
            raise ValueError("host-stub evidence cannot claim SDK target metadata")
        if compiler_commands_hash is not None:
            raise ValueError("host-stub evidence cannot claim target compile commands")
    else:
        _hash(compiler_commands_hash, "build.compiler_commands_sha256")
        sdk = _object(
            sdk_metadata,
            "build.sdk_metadata",
            {"revision", "release", "worktree_dirty"},
        )
        revision = _text(sdk["revision"], "build.sdk_metadata.revision")
        if _GIT_REVISION.fullmatch(revision) is None:
            raise ValueError("build.sdk_metadata.revision must be a Git commit hash")
        _text(sdk["release"], "build.sdk_metadata.release")
        _boolean(sdk["worktree_dirty"], "build.sdk_metadata.worktree_dirty")

    execution = _object(
        data["execution"],
        "execution",
        {"exit_code", "marker_seen", "terminated", "log_sha256", "log_bytes"},
    )
    exit_code = _integer(
        execution["exit_code"], "execution.exit_code", minimum=-255, maximum=255
    )
    if not isinstance(execution["marker_seen"], bool) or not isinstance(
        execution["terminated"], bool
    ):
        raise ValueError("execution marker_seen and terminated must be booleans")
    if not execution["terminated"]:
        raise ValueError("mapping diagnostic process did not terminate")
    log_hash = execution["log_sha256"]
    log_bytes = execution["log_bytes"]
    if evidence_class == "gvsoc-target-execution":
        _hash(log_hash, "execution.log_sha256")
        _integer(log_bytes, "execution.log_bytes", minimum=1)
    elif log_hash is not None or log_bytes is not None:
        raise ValueError("host-stub artifacts must not invent a retained target log")

    mapping = _object(
        data["mapping"],
        "mapping",
        {
            "poses",
            "scans",
            "points",
            "finite_poses",
            "has_nan",
            "icp_attempts",
            "correspondences_last_pair",
            "icp_error_milli",
            "icp_has_nan",
            "accepted_constraints",
            "graph_nodes",
            "optimizer_input_constraints",
            "optimizer_unique_edges",
            "main_solver_returned",
            "successful_solver_calls",
            "completed_solver_iterations",
            "optimization_completed",
        },
    )
    observed_poses = _integer(mapping["poses"], "mapping.poses")
    observed_scans = _integer(mapping["scans"], "mapping.scans")
    observed_points = _integer(mapping["points"], "mapping.points")
    finite_poses = _integer(mapping["finite_poses"], "mapping.finite_poses")
    if (observed_poses, observed_scans, observed_points) != (poses, scans, points):
        raise ValueError("mapping counts do not match fixture counts")
    if finite_poses > poses:
        raise ValueError("mapping.finite_poses exceeds fixture.pose_count")
    _boolean(mapping["has_nan"], "mapping.has_nan")
    nullable_fields = {
        "icp_attempts": "integer",
        "correspondences_last_pair": "integer",
        "icp_error_milli": "integer",
        "icp_has_nan": "boolean",
        "accepted_constraints": "integer",
        "graph_nodes": "integer",
        "optimizer_input_constraints": "integer",
        "optimizer_unique_edges": "integer",
        "main_solver_returned": "boolean",
        "successful_solver_calls": "integer",
        "completed_solver_iterations": "integer",
        "optimization_completed": "boolean",
    }
    metrics: dict[str, int | bool | None] = {}
    for field, kind in nullable_fields.items():
        item = mapping[field]
        if item is None:
            metrics[field] = None
        elif kind == "integer":
            metrics[field] = _integer(item, f"mapping.{field}")
        else:
            metrics[field] = _boolean(item, f"mapping.{field}")
    attempts = metrics["icp_attempts"]
    accepted = metrics["accepted_constraints"]
    if attempts is not None and accepted is not None and accepted > attempts:
        raise ValueError("accepted_constraints exceeds icp_attempts")
    unique_edges = metrics["optimizer_unique_edges"]
    optimizer_constraints = metrics["optimizer_input_constraints"]
    if (
        unique_edges is not None
        and optimizer_constraints is not None
        and unique_edges > optimizer_constraints
    ):
        raise ValueError("optimizer_unique_edges exceeds optimizer input count")
    graph_nodes = metrics["graph_nodes"]
    if graph_nodes is not None and graph_nodes > scans:
        raise ValueError("graph_nodes exceeds fixture.scan_count")
    if data["status"] == "passed":
        if exit_code != 0 or not execution["marker_seen"] or mapping["has_nan"]:
            raise ValueError(
                "passed artifact contradicts execution or finite-value evidence"
            )
        if any(mapping[field] is None for field in nullable_fields):
            raise ValueError("passed artifact contains unobserved mapping stages")
        if mapping["icp_has_nan"] or not mapping["optimization_completed"]:
            raise ValueError("passed artifact contradicts ICP or optimizer status")
        if finite_poses != poses:
            raise ValueError("passed artifact contains non-finite poses")
        if (
            not mapping["main_solver_returned"]
            or mapping["successful_solver_calls"] == 0
        ):
            raise ValueError("passed artifact lacks successful solver evidence")
        if (
            mapping["icp_attempts"] != 2
            or mapping["accepted_constraints"] < 2
            or mapping["graph_nodes"] != scans
            or mapping["optimizer_input_constraints"] < 2
            or mapping["optimizer_unique_edges"] < 2
            or mapping["successful_solver_calls"] != scans
            or mapping["completed_solver_iterations"] != 9
            or mapping["icp_error_milli"] == 2_147_483_647
        ):
            raise ValueError("passed artifact lacks ICP or pose-graph evidence")
    if data["status"] == "reproduced-limitation":
        if evidence_class != "gvsoc-target-execution":
            raise ValueError("only target execution can reproduce a GVSOC limitation")
        if exit_code != 1 or not execution["marker_seen"]:
            raise ValueError("reproduced limitation contradicts target execution")
        limitation_metrics = {
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
        if any(
            mapping[field] != expected for field, expected in limitation_metrics.items()
        ):
            raise ValueError(
                "reproduced limitation metrics do not match the known failure"
            )
    if evidence_class == "host-pmsis-stubs" and "GVSOC target" in str(
        data["claim_boundary"]
    ):
        raise ValueError("host-stub evidence cannot claim GVSOC target execution")
    return dict(data)


def _object(value: object, name: str, keys: set[str]) -> dict[str, Any]:
    if not isinstance(value, Mapping) or set(value) != keys:
        raise ValueError(f"{name} fields do not match diagnostic schema")
    return dict(value)


def _integer(
    value: object,
    name: str,
    *,
    minimum: int = 0,
    maximum: int | None = None,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
        or (maximum is not None and value > maximum)
    ):
        upper = "" if maximum is None else f" and <= {maximum}"
        raise ValueError(f"{name} must be an integer >= {minimum}{upper}")
    return value


def _boolean(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a boolean")
    return value


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be nonempty text")
    return value


def _hash(value: object, name: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256 hex digest")
    return value


def mapping_source_paths(repository_root: Path) -> tuple[Path, ...]:
    """Return repository-owned mapping build inputs in stable path order."""
    paths = {repository_root / path for path in _MAPPING_SOURCE_FILES}
    for directory in _MAPPING_SOURCE_DIRECTORIES:
        paths.update(
            path
            for path in (repository_root / directory).rglob("*")
            if path.is_file() and path.suffix in {".c", ".h"}
        )
    return tuple(sorted(paths))
