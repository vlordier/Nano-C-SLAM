#!/usr/bin/env python3
"""Compare provenance-matched host and GVSOC mapping diagnostics."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import struct
import sys
from pathlib import Path
from typing import Any

from scripts.gap9_mapping_diagnostic import validate_mapping_diagnostic

_DIAG_PREFIX = "NANO_C_SLAM_ICP_DIAG:"
_STAGE_PATTERN = re.compile(r"(?:^|\s)stage=([^\s]+)")
_FIELD_PATTERN = re.compile(r"(?:^|\s)([A-Za-z_]+)=([^\s]+)")
_FLOAT_FIELDS: dict[tuple[str, str], tuple[str, float]] = {
    ("centered-sample", "p"): ("m", 1e-5),
    ("centered-sample", "q"): ("m", 1e-5),
    ("center", "p"): ("m", 1e-5),
    ("center", "q"): ("m", 1e-5),
    ("covariance", "values"): ("m^2", 1e-4),
    ("svd", "sigma"): ("m^2", 1e-4),
    ("svd", "u"): ("dimensionless", 1e-5),
    ("svd", "v"): ("dimensionless", 1e-5),
    ("transform", "t"): ("m", 1e-5),
    ("transform", "r"): ("dimensionless", 1e-5),
    ("error", "bits"): ("dimensionless", 1e-6),
}
_SUMMARY_FIELDS = (
    ("poses", "count"),
    ("scans", "count"),
    ("points", "count"),
    ("finite_poses", "count"),
    ("has_nan", "boolean"),
    ("icp_attempts", "count"),
    ("correspondences_last_pair", "count"),
    ("icp_error_milli", "dimensionless x 1000"),
    ("icp_has_nan", "boolean"),
    ("accepted_constraints", "count"),
    ("optimizer_input_constraints", "count"),
    ("optimizer_unique_edges", "count"),
    ("graph_nodes", "count"),
    ("main_solver_returned", "boolean"),
    ("successful_solver_calls", "count"),
    ("completed_solver_iterations", "iteration count"),
    ("optimization_completed", "boolean"),
)
_IGNORED_FIELDS = {"address"}
_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
_COMPARATOR_INPUTS = (
    "scripts/compare_gap9_mapping_diagnostics.py",
    "scripts/gap9_mapping_diagnostic.py",
    "artifacts/gap9_gvsoc_evidence/mapping_diagnostic.schema.json",
)


def _load_artifact(path: Path, expected_class: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read diagnostic artifact {path}: {error}") from error
    artifact = validate_mapping_diagnostic(value)
    if artifact["status"] != "passed":
        raise ValueError(f"diagnostic is not a passing artifact: {path}")
    if artifact["evidence_class"] != expected_class:
        raise ValueError(f"{path} must have evidence_class {expected_class!r}")
    return artifact


def _load_log(path: Path, execution: dict[str, Any]) -> str:
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise ValueError(f"cannot read diagnostic log {path}: {error}") from error
    if len(raw) != execution["log_bytes"]:
        raise ValueError(f"diagnostic log byte count does not match artifact: {path}")
    if hashlib.sha256(raw).hexdigest() != execution["log_sha256"]:
        raise ValueError(f"diagnostic log SHA-256 does not match artifact: {path}")
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError(f"diagnostic log is not UTF-8: {path}") from error


def _validate_pair(host: dict[str, Any], target: dict[str, Any]) -> None:
    if host["evidence_class"] != "host-pmsis-stubs":
        raise ValueError("first artifact must contain host-pmsis-stubs evidence")
    if target["evidence_class"] != "gvsoc-target-execution":
        raise ValueError("second artifact must contain GVSOC target evidence")
    host_fixture = host["fixture"]
    target_fixture = target["fixture"]
    if host_fixture != target_fixture:
        raise ValueError("host and GVSOC fixture identity or counts do not match")
    host_sources = host["build"]["source_sha256"]
    target_sources = target["build"]["source_sha256"]
    if set(host_sources) != set(target_sources):
        raise ValueError("host and GVSOC build input path sets do not match")
    shared_paths = sorted(host_sources)
    mismatches = [
        path for path in shared_paths if host_sources[path] != target_sources[path]
    ]
    if mismatches:
        raise ValueError(f"shared build input hashes differ: {mismatches[0]}")


def _parse_float_values(value: str) -> tuple[float, ...] | None:
    if value.startswith("(") and value.endswith(")"):
        encoded_values = value[1:-1].split(",")
    else:
        encoded_values = [value]
    floats: list[float] = []
    try:
        for encoded in encoded_values:
            if len(encoded) != 8:
                return None
            number = struct.unpack(">f", bytes.fromhex(encoded))[0]
            if not math.isfinite(number):
                return None
            floats.append(number)
    except (ValueError, struct.error):
        return None
    return tuple(floats)


def _parse_records(log: str) -> tuple[list[dict[str, Any]], int]:
    records: list[dict[str, Any]] = []
    ignored_addresses = 0
    for line_number, line in enumerate(log.splitlines(), start=1):
        if _DIAG_PREFIX not in line:
            continue
        payload = line.split(_DIAG_PREFIX, 1)[1]
        stage_match = _STAGE_PATTERN.search(payload)
        if stage_match is None:
            continue
        fields = dict(_FIELD_PATTERN.findall(payload))
        stage = stage_match.group(1)
        if "address" in fields:
            ignored_addresses += 1
            fields.pop("address")
        iteration = fields.pop("iter", None)
        records.append(
            {
                "line": line_number,
                "stage": stage,
                "iteration": iteration,
                "fields": fields,
            }
        )
    return records, ignored_addresses


def _first_stage_divergence(
    host_records: list[dict[str, Any]], target_records: list[dict[str, Any]]
) -> tuple[dict[str, Any] | None, int, int]:
    tolerated_differences = 0
    for record_index in range(max(len(host_records), len(target_records))):
        if record_index >= len(host_records) or record_index >= len(target_records):
            present = (
                host_records[record_index]
                if record_index < len(host_records)
                else target_records[record_index]
            )
            return (
                {
                    "kind": "missing-stage-instrumentation",
                    "stage": present["stage"],
                    "record_index": record_index,
                    "host_line": host_records[record_index]["line"]
                    if record_index < len(host_records)
                    else None,
                    "gvsoc_line": target_records[record_index]["line"]
                    if record_index < len(target_records)
                    else None,
                    "host": "<missing>"
                    if record_index >= len(host_records)
                    else present,
                    "gvsoc": "<missing>"
                    if record_index >= len(target_records)
                    else present,
                    "units": "record",
                    "absolute_tolerance": 0,
                },
                tolerated_differences,
                record_index + 1,
            )
        host_record = host_records[record_index]
        target_record = target_records[record_index]
        if host_record["stage"] != target_record["stage"]:
            return (
                {
                    "kind": "missing-stage-instrumentation",
                    "stage": host_record["stage"],
                    "record_index": record_index,
                    "host_line": host_record["line"],
                    "gvsoc_line": target_record["line"],
                    "host": host_record["stage"],
                    "gvsoc": target_record["stage"],
                    "units": "stage name",
                    "absolute_tolerance": 0,
                },
                tolerated_differences,
                record_index + 1,
            )
        host_fields = host_record["fields"]
        target_fields = target_record["fields"]
        field_names = list(host_fields)
        field_names.extend(name for name in target_fields if name not in host_fields)
        for field_name in field_names:
            host_value = host_fields.get(field_name)
            target_value = target_fields.get(field_name)
            if host_value is None or target_value is None:
                return (
                    {
                        "kind": "missing-stage-instrumentation",
                        "stage": host_record["stage"],
                        "iteration": host_record["iteration"],
                        "field": field_name,
                        "record_index": record_index,
                        "host_line": host_record["line"],
                        "gvsoc_line": target_record["line"],
                        "host": host_value if host_value is not None else "<missing>",
                        "gvsoc": target_value
                        if target_value is not None
                        else "<missing>",
                        "units": "field",
                        "absolute_tolerance": 0,
                    },
                    tolerated_differences,
                    record_index + 1,
                )
            float_metadata = _FLOAT_FIELDS.get((host_record["stage"], field_name))
            if float_metadata is not None:
                units, tolerance = float_metadata
                host_values = _parse_float_values(host_value)
                target_values = _parse_float_values(target_value)
                if host_values is None or target_values is None:
                    return (
                        {
                            "kind": "missing-or-sentinel-stage-value",
                            "stage": host_record["stage"],
                            "iteration": host_record["iteration"],
                            "field": field_name,
                            "record_index": record_index,
                            "host_line": host_record["line"],
                            "gvsoc_line": target_record["line"],
                            "host": host_value,
                            "gvsoc": target_value,
                            "units": units,
                            "absolute_tolerance": tolerance,
                        },
                        tolerated_differences,
                        record_index + 1,
                    )
                if len(host_values) != len(target_values):
                    return (
                        {
                            "kind": "numeric-shape-divergence",
                            "stage": host_record["stage"],
                            "iteration": host_record["iteration"],
                            "field": field_name,
                            "record_index": record_index,
                            "host_line": host_record["line"],
                            "gvsoc_line": target_record["line"],
                            "host": host_value,
                            "gvsoc": target_value,
                            "units": units,
                            "absolute_tolerance": tolerance,
                        },
                        tolerated_differences,
                        record_index + 1,
                    )
                for component, (host_number, target_number) in enumerate(
                    zip(host_values, target_values, strict=True)
                ):
                    delta = abs(host_number - target_number)
                    if delta > tolerance:
                        return (
                            {
                                "kind": "numeric-divergence",
                                "stage": host_record["stage"],
                                "iteration": host_record["iteration"],
                                "field": field_name,
                                "component": component,
                                "record_index": record_index,
                                "host_line": host_record["line"],
                                "gvsoc_line": target_record["line"],
                                "host": host_number,
                                "gvsoc": target_number,
                                "absolute_difference": delta,
                                "units": units,
                                "absolute_tolerance": tolerance,
                            },
                            tolerated_differences,
                            record_index + 1,
                        )
                    if delta > 0:
                        tolerated_differences += 1
            elif host_value != target_value:
                kind = (
                    "count-divergence"
                    if host_value.isdecimal() and target_value.isdecimal()
                    else "stage-value-divergence"
                )
                return (
                    {
                        "kind": kind,
                        "stage": host_record["stage"],
                        "iteration": host_record["iteration"],
                        "field": field_name,
                        "record_index": record_index,
                        "host_line": host_record["line"],
                        "gvsoc_line": target_record["line"],
                        "host": host_value,
                        "gvsoc": target_value,
                        "units": "count" if kind == "count-divergence" else "value",
                        "absolute_tolerance": 0,
                    },
                    tolerated_differences,
                    record_index + 1,
                )
    return None, tolerated_differences, min(len(host_records), len(target_records))


def _summary_divergence(
    host: dict[str, Any], target: dict[str, Any]
) -> dict[str, Any] | None:
    host_mapping = host["mapping"]
    target_mapping = target["mapping"]
    for field_name, units in _SUMMARY_FIELDS:
        host_value = host_mapping[field_name]
        target_value = target_mapping[field_name]
        if host_value != target_value:
            kind = "count-divergence" if units != "boolean" else "status-divergence"
            return {
                "kind": kind,
                "stage": "mapping-summary",
                "field": field_name,
                "host": host_value,
                "gvsoc": target_value,
                "units": units,
                "absolute_tolerance": 0,
            }
    return None


def compare_diagnostics(
    host: dict[str, Any],
    target: dict[str, Any],
    host_log: str,
    target_log: str,
) -> dict[str, Any]:
    """Compare validated artifact dictionaries and already loaded diagnostic logs."""
    host = validate_mapping_diagnostic(host)
    target = validate_mapping_diagnostic(target)
    _validate_pair(host, target)
    host_records, host_ignored = _parse_records(host_log)
    target_records, target_ignored = _parse_records(target_log)
    if not host_records or not target_records:
        raise ValueError("both logs must contain ICP stage diagnostics")
    divergence, tolerated_differences, compared_record_count = _first_stage_divergence(
        host_records, target_records
    )
    if divergence is None:
        divergence = _summary_divergence(host, target)
    comparator_hashes = {
        relative: hashlib.sha256((_REPOSITORY_ROOT / relative).read_bytes()).hexdigest()
        for relative in _COMPARATOR_INPUTS
    }
    return {
        "schema_version": 1,
        "status": "compared",
        "comparison_result": (
            "divergence" if divergence else "equivalent-within-tolerance"
        ),
        "fixture": host["fixture"],
        "builds": {
            "host": {
                "binary_sha256": host["build"]["binary_sha256"],
                "toolchain_id": host["build"]["toolchain_id"],
                "compiler_flags": host["build"]["compiler_flags"],
                "compiler_commands_sha256": host["build"]["compiler_commands_sha256"],
            },
            "gvsoc": {
                "binary_sha256": target["build"]["binary_sha256"],
                "toolchain_id": target["build"]["toolchain_id"],
                "compiler_flags": target["build"]["compiler_flags"],
                "compiler_commands_sha256": target["build"]["compiler_commands_sha256"],
                "sdk_metadata": target["build"]["sdk_metadata"],
            },
            "shared_input_count": len(
                set(host["build"]["source_sha256"])
                & set(target["build"]["source_sha256"])
            ),
        },
        "shared_source_sha256": host["build"]["source_sha256"],
        "comparator_inputs_sha256": comparator_hashes,
        "compared_stage_record_count": compared_record_count,
        "tolerated_float_component_difference_count": tolerated_differences,
        "ignored_target_specific_pointer_field_count": host_ignored + target_ignored,
        "instrumented_stages": sorted(
            {record["stage"] for record in host_records}
            | {record["stage"] for record in target_records}
        ),
        "not_instrumented": [
            "individual generated pose/range values; fixture source hash and "
            "counts are compared",
            "scan extraction internal values",
            "every intermediate optimized pose coordinate",
        ],
        "tolerances": {
            f"{stage}.{field}": {"units": units, "absolute": tolerance}
            for (stage, field), (units, tolerance) in _FLOAT_FIELDS.items()
        },
        "first_divergence": divergence,
        "interpretation_boundary": (
            "This comparison identifies the first observed difference in the "
            "instrumented fields. It does not establish root cause, mapping "
            "accuracy, or hardware behavior."
        ),
    }


def compare_files(
    host_artifact_path: Path,
    target_artifact_path: Path,
    host_log_path: Path,
    target_log_path: Path,
) -> dict[str, Any]:
    """Load artifacts, verify raw-log hashes, and compare one evidence pair."""
    host = _load_artifact(host_artifact_path, "host-pmsis-stubs")
    target = _load_artifact(target_artifact_path, "gvsoc-target-execution")
    host_log = _load_log(host_log_path, host["execution"])
    target_log = _load_log(target_log_path, target["execution"])
    result = compare_diagnostics(host, target, host_log, target_log)
    result["input_artifacts"] = {
        "host_sha256": hashlib.sha256(host_artifact_path.read_bytes()).hexdigest(),
        "gvsoc_sha256": hashlib.sha256(target_artifact_path.read_bytes()).hexdigest(),
    }
    result["input_logs"] = {
        "host": {
            "path": str(host_log_path),
            "sha256": host["execution"]["log_sha256"],
            "bytes": host["execution"]["log_bytes"],
        },
        "gvsoc": {
            "path": str(target_log_path),
            "sha256": target["execution"]["log_sha256"],
            "bytes": target["execution"]["log_bytes"],
        },
    }
    return result


def _publish_atomically(path: Path, value: dict[str, Any]) -> None:
    if path.exists():
        raise FileExistsError(f"comparison output already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    content = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-artifact", type=Path, required=True)
    parser.add_argument("--gvsoc-artifact", type=Path, required=True)
    parser.add_argument("--host-log", type=Path, required=True)
    parser.add_argument("--gvsoc-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args(args)
    try:
        comparison = compare_files(
            options.host_artifact,
            options.gvsoc_artifact,
            options.host_log,
            options.gvsoc_log,
        )
        _publish_atomically(options.output, comparison)
    except (FileExistsError, OSError, ValueError) as error:
        print(f"GAP9 mapping comparison failed: {error}", file=sys.stderr)
        return 1
    print(
        f"Compared host/GVSOC diagnostics: {comparison['comparison_result']} "
        f"({options.output})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
