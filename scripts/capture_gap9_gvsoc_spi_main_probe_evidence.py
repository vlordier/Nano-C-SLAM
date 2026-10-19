"""Validate and hash-bind real-SPI production-main GVSOC probe evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import sys
from pathlib import Path

from scripts.gap9_gvsoc_spi_probe_protocol import (
    SCAN_CMD,
    make_mapping_trace,
    make_pose_frame,
    make_pose_request_frame,
    make_scan_frame,
)
from scripts.validate_gap9_gvsoc_testbench import validate_config

TARGET_MARKER = (
    "NANO_C_SLAM_GVSOC_SPI_MAIN:run_ok packets=2 pose_decoded=1 "
    "pose_inserted=1 scans_decoded=1 scans_registered=1 poses=1 scan_origins=1"
)
SPI_DIAGNOSTICS_PATTERN = re.compile(
    r"^NANO_C_SLAM_SPI:stats rx=(\d+) malformed=(\d+) unknown=(\d+) "
    r"frames=([0-9,]+) decode_ok=([0-9,]+) decode_error=([0-9,]+) "
    r"pose_id_missing=(\d+) pose_id_nonadvancing=(\d+) "
    r"optimize_responses=(\d+)$",
    re.MULTILINE,
)
SCAN_BUSY_PATTERN = re.compile(
    r"^NANO_C_SLAM_GVSOC_SPI_MAIN:scan_busy index=(\d+) "
    r"pose_id=(-?\d+) duration_us=(\d+)$",
    re.MULTILINE,
)
COMMAND_NAMES = (
    "pose",
    "scan",
    "optimize",
    "pose_request",
    "check",
    "external_pose",
    "pose_update",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _capture_pose_request_evidence(
    *,
    config_path: Path,
    elf_path: Path,
    host_transfer_path: Path,
    log_path: Path,
    source_paths: tuple[Path, ...],
    host_transfer: dict[str, object],
    log: str,
) -> dict[str, object]:
    """Validate real target POSE_REQ response bytes and marker ordering."""
    positions = ((0.25, -0.5, 0.75), (1.25, -1.5, 1.75))
    expected_frames = [
        make_pose_frame(0, position=positions[0]),
        make_pose_frame(1, position=positions[1]),
        make_pose_request_frame(0, 2),
    ]
    expected_records = [
        {"command": name, "frame_tx_hex": frame.hex(), "length": len(frame)}
        for name, frame in zip(
            ("POSE", "POSE", "POSE_REQ"), expected_frames, strict=True
        )
    ]
    if (
        host_transfer.get("sequence") != "pose-request"
        or host_transfer.get("pre_cs_dummy_clock_cycles") != 8
        or host_transfer.get("frames") != expected_records
        or host_transfer.get("response_size") != 24
        or host_transfer.get("control_phase") != "pose_response_captured"
    ):
        raise ValueError("host transfer does not match the POSE_REQ probe contract")

    ready = re.search(
        r"NANO_C_SLAM_GVSOC_SPI_MAIN:pose_response_ready "
        r"from=0 count=2 bytes=24\b",
        log,
    )
    done = re.search(
        r"NANO_C_SLAM_GVSOC_SPI_MAIN:pose_response_done "
        r"from=0 count=2 duration_us=(\d+)\b",
        log,
    )
    target_marker = re.search(
        r"NANO_C_SLAM_GVSOC_SPI_MAIN:pose_request_ok packets=3 "
        r"pose_decoded=2 pose_inserted=2 requests_decoded=1 "
        r"requests_served=1 poses=2\b",
        log,
    )
    if ready is None or done is None or target_marker is None:
        raise ValueError(
            "GVSOC log lacks POSE_REQ ready, completion, or success evidence"
        )

    expected_response = struct.pack("<6f", *positions[0], *positions[1]).hex()
    response_hex = host_transfer.get("response_rx_hex")
    if response_hex != expected_response:
        raise ValueError("POSE_REQ response bytes differ from the two inserted poses")
    duration_us = int(done.group(1))
    return {
        "schema_version": 1,
        "status": "passed",
        "evidence_class": "gvsoc-production-main-spi2-pose-request",
        "sdk_release": "5.20.4",
        "interface": "SPI2/CS0, mode 0, 8-bit words",
        "sequence": "pose-request",
        "frame_count": len(expected_frames),
        "frames": [frame.hex() for frame in expected_frames],
        "target_marker": target_marker.group(0),
        "target_ready_marker": ready.group(0),
        "target_done_marker": done.group(0),
        "target_response_duration_us": duration_us,
        "response_size": 24,
        "response_expected_hex": expected_response,
        "response_rx_hex": response_hex,
        "response_status": "captured and matches both inserted float poses",
        "controller_timing": {
            "ready_settle_delay_ms": 10,
            "gvsoc_run_after_response_queue_ms": 100,
            "note": (
                "These are simulator scheduling intervals, not pin-edge measurements."
            ),
        },
        "sha256": {
            "elf": _sha256(elf_path),
            "gvsoc_config": _sha256(config_path),
            "host_transfer": _sha256(host_transfer_path),
            "gvsoc_log": _sha256(log_path),
            "sources": {str(path): _sha256(path) for path in source_paths},
        },
        "limitations": [
            "The target ready marker follows busy assertion and precedes the "
            "blocking production SPI send; the done marker follows busy deassertion.",
            "The controller uses a fixed 10 ms settle delay because GVSOC "
            "exposes no GPIO readback; this does not prove the master "
            "observed readiness.",
            "The 100 ms GVSOC run interval dominates target duration and "
            "cannot be compared precisely with STM32 wait deadlines.",
            "Does not validate electrical GPIO wiring, physical SPI timing, "
            "map accuracy, or flight behavior.",
        ],
    }


def capture_evidence(
    *,
    config_path: Path,
    elf_path: Path,
    host_transfer_path: Path,
    log_path: Path,
    source_paths: tuple[Path, ...] = (),
) -> dict[str, object]:
    """Reject incomplete production-main evidence and return bound metadata."""
    defects = validate_config(config_path)
    if defects:
        raise ValueError("invalid GVSOC configuration: " + "; ".join(defects))

    try:
        host_transfer = json.loads(host_transfer_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read host transfer record: {error}") from error
    if not isinstance(host_transfer, dict):
        raise ValueError("host transfer record must be a JSON object")
    frames = host_transfer.get("frames")
    sequence = host_transfer.get("sequence", "pose-scan")
    if sequence in {"mapping", "max-mapping", "capacity-mapping"}:
        expected = list(
            make_mapping_trace(
                260 if sequence != "mapping" else 60,
                scan_interval=2 if sequence == "capacity-mapping" else 20,
                periodic=sequence == "capacity-mapping",
                scans_after_poses=sequence == "capacity-mapping",
            )
        )
        expected_commands = [
            "POSE_BAD_CHECKSUM"
            if index == 0
            else {0: "POSE", 1: "SCAN", 2: "OPTIMIZE"}[frame[1]]
            for index, frame in enumerate(expected)
        ]
    elif sequence == "pose-scan":
        expected = [make_pose_frame(), make_scan_frame()]
        expected_commands = ["POSE", "SCAN"]
    elif sequence == "pose-request":
        expected = [
            make_pose_frame(0, position=(0.25, -0.5, 0.75)),
            make_pose_frame(1, position=(1.25, -1.5, 1.75)),
            make_pose_request_frame(0, 2),
        ]
        expected_commands = ["POSE", "POSE", "POSE_REQ"]
    else:
        raise ValueError(f"unsupported production-main SPI sequence: {sequence}")
    if host_transfer.get("pre_cs_dummy_clock_cycles") != 8 or not isinstance(
        frames, list
    ):
        raise ValueError("host transfer must contain the 8 pre-CS clocks and frames")
    expected_records = [
        {"command": command, "frame_tx_hex": frame.hex(), "length": len(frame)}
        for command, frame in zip(expected_commands, expected, strict=True)
    ]
    if frames != expected_records:
        raise ValueError("host transfer bytes/order differ from expected sequence")

    try:
        log = log_path.read_text(encoding="utf-8")
    except OSError as error:
        raise ValueError(f"cannot read GVSOC log: {error}") from error
    if sequence == "pose-request":
        return _capture_pose_request_evidence(
            config_path=config_path,
            elf_path=elf_path,
            host_transfer_path=host_transfer_path,
            log_path=log_path,
            source_paths=source_paths,
            host_transfer=host_transfer,
            log=log,
        )
    response_status: str | None = None
    spi_diagnostics: dict[str, object] | None = None
    scan_busy_durations_us: list[dict[str, int]] | None = None
    optimizer_time_ms: int | None = None
    optimizer_graph_size: int | None = None
    mapping_outcome: str | None = None
    if sequence == "pose-scan":
        if TARGET_MARKER not in log:
            raise ValueError(
                "GVSOC log lacks the target-side production-main success marker"
            )
        if "NANO_C_SLAM_GVSOC_SPI_MAIN:run_fail" in log:
            raise ValueError("GVSOC production-main target reported failure")
        target_marker = TARGET_MARKER
        response_expected_hex = None
        limitations = [
            "Proves POSE and SCAN dispatch through production main.c on GVSOC SPI2.",
            "Does not validate physical SPI, mapping accuracy, or flight behavior.",
        ]
    else:
        if host_transfer.get("control_phase") not in {
            "optimizer_completed_response_not_clocked",
            "optimizer_response_captured",
        }:
            raise ValueError("host controller did not complete the mapping phase")
        if sequence == "mapping":
            mapping_pattern = re.compile(
                r"NANO_C_SLAM_GVSOC_SPI_MAIN:mapping_ok packets=66 "
                r"pose_decoded=60 pose_inserted=60 bad_pose_checksum=1 "
                r"scans_decoded=4 scans_registered=3 constraints=2 "
                r"optimizer_edges=2 optimize=1 graph=3 solver_calls=\d+ "
                r"response_expected_hex=([0-9a-f]{36})"
            )
            expected_diagnostics = {
                "received_frames": 66,
                "malformed_frames": 0,
                "unknown_commands": 0,
                "command_frames": [61, 4, 1, 0, 0, 0, 0],
                "decode_success": [60, 4, 0, 0, 0, 0, 0],
                "decode_failure": [1, 0, 0, 0, 0, 0, 0],
            }
        elif sequence == "max-mapping":
            mapping_pattern = re.compile(
                r"NANO_C_SLAM_GVSOC_SPI_MAIN:max_mapping_ok packets=276 "
                r"pose_decoded=260 pose_inserted=260 bad_pose_checksum=1 "
                r"scans_decoded=14 scans_registered=13 constraints=\d+ "
                r"optimizer_edges=\d+ optimize=1 poses=260 graph=13 "
                r"solver_calls=\d+ time_ms=(\d+) has_nan=0 completed=1 "
                r"response_expected_hex=([0-9a-f]{36})"
            )
            expected_diagnostics = {
                "received_frames": 276,
                "malformed_frames": 0,
                "unknown_commands": 0,
                "command_frames": [261, 14, 1, 0, 0, 0, 0],
                "decode_success": [260, 14, 0, 0, 0, 0, 0],
                "decode_failure": [1, 0, 0, 0, 0, 0, 0],
            }
        else:
            mapping_pattern = re.compile(
                r"NANO_C_SLAM_GVSOC_SPI_MAIN:capacity_ingress_ok packets=393 "
                r"pose_decoded=260 pose_inserted=260 bad_pose_checksum=1 "
                r"scans_decoded=131 scans_registered=100 constraints=(\d+) "
                r"optimizer_edges=(\d+) optimize=1 poses=(\d+) graph=(\d+) "
                r"solver_calls=(\d+) time_ms=(\d+) has_nan=0 completed=(\d+) "
                r"response_expected_hex=([0-9a-f]{36})"
            )
            expected_diagnostics = {
                "received_frames": 393,
                "malformed_frames": 0,
                "unknown_commands": 0,
                "command_frames": [261, 131, 1, 0, 0, 0, 0],
                "decode_success": [260, 131, 0, 0, 0, 0, 0],
                "decode_failure": [1, 0, 0, 0, 0, 0, 0],
            }
        expected_scan_count = {
            "mapping": 4,
            "max-mapping": 14,
            "capacity-mapping": 131,
        }[sequence]
        expected_scan_pose_ids = [
            int.from_bytes(frame[2:4], "little", signed=True)
            for frame in expected
            if frame[1] == SCAN_CMD
        ]
        scan_busy_matches = SCAN_BUSY_PATTERN.findall(log)
        if (
            len(scan_busy_matches) != expected_scan_count
            or len(expected_scan_pose_ids) != expected_scan_count
        ):
            raise ValueError(
                "GVSOC log must contain one SCAN busy duration per decoded "
                f"SCAN ({expected_scan_count} expected, got "
                f"{len(scan_busy_matches)})"
            )
        scan_busy_durations_us = []
        for expected_index, (index_text, pose_id_text, duration_text) in enumerate(
            scan_busy_matches
        ):
            index = int(index_text)
            pose_id = int(pose_id_text)
            if index != expected_index:
                raise ValueError("GVSOC SCAN busy duration indices are not contiguous")
            expected_pose_id = expected_scan_pose_ids[expected_index]
            if pose_id != expected_pose_id:
                raise ValueError("GVSOC SCAN busy pose IDs differ from injected trace")
            scan_busy_durations_us.append(
                {
                    "index": index,
                    "pose_id": pose_id,
                    "duration_us": int(duration_text),
                }
            )
        match = mapping_pattern.search(log)
        if match is None:
            raise ValueError("GVSOC log lacks the validated full-mapping marker")
        target_marker = match.group(0)
        if sequence == "capacity-mapping":
            constraints, edges, poses, graph, calls, duration, completed = (
                int(value) for value in match.groups()[:7]
            )
            response_expected_hex = match.group(8)
            if not 0 <= edges <= constraints <= 50:
                raise ValueError("capacity ingress constraint counts exceed limits")
            if constraints == 0:
                if (poses, graph, calls, duration, completed) != (0, 0, 0, 0, 0):
                    raise ValueError(
                        "zero-constraint trace unexpectedly reports a solve"
                    )
                mapping_outcome = "no_loop_closures; optimizer_not_started"
            else:
                if not 2 <= graph <= 100 or poses != 260 or calls < 1 or completed != 1:
                    raise ValueError(
                        "accepted constraints lack a completed optimizer result"
                    )
                mapping_outcome = "optimizer_completed"
                optimizer_graph_size = graph
                optimizer_time_ms = duration
        elif sequence == "max-mapping":
            optimizer_time_ms = int(match.group(1))
            response_expected_hex = match.group(2)
        else:
            response_expected_hex = match.group(1)
        diagnostics_match = SPI_DIAGNOSTICS_PATTERN.search(log)
        if diagnostics_match is None:
            raise ValueError("GVSOC log lacks production SPI receive diagnostics")
        (
            received_frames,
            malformed_frames,
            unknown_commands,
            command_frames_csv,
            decode_success_csv,
            decode_failure_csv,
            missing_pose_ids,
            nonadvancing_pose_ids,
            completed_optimize_responses,
        ) = diagnostics_match.groups()
        command_frames = [int(value) for value in command_frames_csv.split(",")]
        decode_success = [int(value) for value in decode_success_csv.split(",")]
        decode_failure = [int(value) for value in decode_failure_csv.split(",")]
        if (
            len(command_frames) != len(COMMAND_NAMES)
            or len(decode_success) != len(COMMAND_NAMES)
            or len(decode_failure) != len(COMMAND_NAMES)
        ):
            raise ValueError("GVSOC SPI diagnostics have an invalid command vector")
        target_response_was_skipped = "response_phase=skipped" in log
        expected_diagnostics.update(
            {
                "missing_pose_ids": 0,
                "nonadvancing_pose_ids": 0,
                "completed_optimize_responses": int(not target_response_was_skipped),
            }
        )
        spi_diagnostics = {
            "received_frames": int(received_frames),
            "malformed_frames": int(malformed_frames),
            "unknown_commands": int(unknown_commands),
            "command_frames": dict(zip(COMMAND_NAMES, command_frames, strict=True)),
            "decode_success": dict(zip(COMMAND_NAMES, decode_success, strict=True)),
            "decode_failure": dict(zip(COMMAND_NAMES, decode_failure, strict=True)),
            "missing_pose_ids": int(missing_pose_ids),
            "nonadvancing_pose_ids": int(nonadvancing_pose_ids),
            "completed_optimize_responses": int(completed_optimize_responses),
            "counter_semantics": (
                "Cumulative since target boot; uint32 counters wrap modulo 2^32. "
                "Frames count only completed pi_spi_receive calls."
            ),
        }
        actual_diagnostics = {
            "received_frames": int(received_frames),
            "malformed_frames": int(malformed_frames),
            "unknown_commands": int(unknown_commands),
            "command_frames": command_frames,
            "decode_success": decode_success,
            "decode_failure": decode_failure,
            "missing_pose_ids": int(missing_pose_ids),
            "nonadvancing_pose_ids": int(nonadvancing_pose_ids),
            "completed_optimize_responses": int(completed_optimize_responses),
        }
        if actual_diagnostics != expected_diagnostics:
            raise ValueError(
                "production SPI diagnostics differ from the injected "
                f"{len(expected)}-frame "
                f"sequence: {actual_diagnostics!r}"
            )
        response_rx_hex = host_transfer.get("response_rx_hex")
        if response_rx_hex is None:
            if "response_phase=skipped" not in log:
                raise ValueError("missing SPI TX bytes without an explicit target skip")
            response_status = "skipped by test target"
            limitations = [
                "The test target skips the SPI slave TX response phase.",
            ]
        elif response_rx_hex != response_expected_hex:
            raise ValueError("SPI TX response bytes differ from target slam_job_t")
        else:
            response_status = "captured and matched target slam_job_t"
            limitations = []
        limitations.extend(
            [
                "Exercises the production command path over GVSOC SPI2 ingress "
                f"with {260 if sequence != 'mapping' else 60} poses.",
                "Does not validate map accuracy, physical SPI, or flight behavior.",
                "Receive counters cannot detect frames for which no clocks were "
                "observed, and sequence-less identical commands cannot be "
                "distinguished as dropped or duplicated.",
            ]
        )
        if sequence == "capacity-mapping":
            limitations.append(
                "Fills the point-cloud 100-scan anchor and 50-constraint "
                "limits with a repeated synthetic two-pose pattern. This "
                "proves capacity-path execution, not map accuracy or a "
                "representative flight trajectory."
            )

    return {
        "schema_version": 1,
        "status": "passed",
        "evidence_class": (
            "gvsoc-production-main-spi2-pose-scan"
            if sequence == "pose-scan"
            else (
                "gvsoc-production-max-capacity-mapping-spi2-ingress"
                if sequence == "max-mapping"
                else (
                    "gvsoc-production-mapping-spi2-ingress"
                    if sequence == "mapping"
                    else "gvsoc-production-scan-and-constraint-capacity-spi2"
                )
            )
        ),
        "sdk_release": "5.20.4",
        "interface": "SPI2/CS0, mode 0, 8-bit words",
        "sequence": sequence,
        "frame_count": len(expected),
        "frames": [frame.hex() for frame in expected],
        "target_marker": target_marker,
        "optimizer_response_expected_hex": response_expected_hex,
        "optimizer_time_ms": optimizer_time_ms,
        "optimizer_graph_size": optimizer_graph_size,
        "mapping_outcome": mapping_outcome if sequence == "capacity-mapping" else None,
        "scan_busy_durations_us": scan_busy_durations_us,
        "scan_busy_max_us": (
            max(item["duration_us"] for item in scan_busy_durations_us)
            if scan_busy_durations_us
            else None
        ),
        "optimizer_response_rx_hex": host_transfer.get("response_rx_hex")
        if sequence in {"mapping", "max-mapping", "capacity-mapping"}
        else None,
        "optimizer_response_status": response_status,
        "spi_diagnostics": spi_diagnostics,
        "sha256": {
            "elf": _sha256(elf_path),
            "gvsoc_config": _sha256(config_path),
            "host_transfer": _sha256(host_transfer_path),
            "gvsoc_log": _sha256(log_path),
            "sources": {str(path): _sha256(path) for path in source_paths},
        },
        "limitations": limitations,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--host-transfer", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--source", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)

    try:
        report = capture_evidence(
            config_path=args.config,
            elf_path=args.elf,
            host_transfer_path=args.host_transfer,
            log_path=args.log,
            source_paths=tuple(args.source),
        )
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"GAP9 GVSOC SPI main probe evidence error: {error}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"GAP9 GVSOC production-main SPI evidence captured: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
