"""Validate and hash-bind real-SPI production-main GVSOC probe evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

from scripts.gap9_gvsoc_spi_probe_protocol import make_pose_frame, make_scan_frame
from scripts.validate_gap9_gvsoc_testbench import validate_config

TARGET_MARKER = (
    "NANO_C_SLAM_GVSOC_SPI_MAIN:run_ok packets=2 pose_decoded=1 "
    "pose_inserted=1 scans_decoded=1 scans_registered=1 poses=1 scan_origins=1"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


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
    expected = [make_pose_frame(), make_scan_frame()]
    if host_transfer.get("pre_cs_dummy_clock_cycles") != 8 or not isinstance(
        frames, list
    ):
        raise ValueError("host transfer must contain the 8 pre-CS clocks and frames")
    expected_records = [
        {"command": command, "frame_tx_hex": frame.hex(), "length": len(frame)}
        for command, frame in zip(("POSE", "SCAN"), expected, strict=True)
    ]
    if frames != expected_records:
        raise ValueError("host transfer bytes/order differ from expected POSE, SCAN")

    try:
        log = log_path.read_text(encoding="utf-8")
    except OSError as error:
        raise ValueError(f"cannot read GVSOC log: {error}") from error
    if TARGET_MARKER not in log:
        raise ValueError(
            "GVSOC log lacks the target-side production-main success marker"
        )
    if "NANO_C_SLAM_GVSOC_SPI_MAIN:run_fail" in log:
        raise ValueError("GVSOC production-main target reported failure")

    return {
        "schema_version": 1,
        "status": "passed",
        "evidence_class": "gvsoc-production-main-spi2-pose-scan",
        "sdk_release": "5.20.4",
        "interface": "SPI2/CS0, mode 0, 8-bit words",
        "frames": [frame.hex() for frame in expected],
        "target_marker": TARGET_MARKER,
        "sha256": {
            "elf": _sha256(elf_path),
            "gvsoc_config": _sha256(config_path),
            "host_transfer": _sha256(host_transfer_path),
            "gvsoc_log": _sha256(log_path),
            "sources": {str(path): _sha256(path) for path in source_paths},
        },
        "limitations": [
            "Proves POSE and SCAN dispatch through production main.c on GVSOC SPI2.",
            "Does not validate physical SPI, mapping accuracy, or flight behavior.",
        ],
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
