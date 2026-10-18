"""Validate and hash-bind the isolated GAP9 GVSOC SPI receive probe."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

from scripts.gap9_gvsoc_spi_probe_protocol import make_scan_frame
from scripts.validate_gap9_gvsoc_testbench import validate_config

TARGET_FRAME_MARKER = (
    "NANO_C_SLAM_SPI_SLAVE_PROBE:frame=received bytes=97 first=53 "
    "cmd=01 id=0000 checksum=00000000 last=45"
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
    """Reject incomplete probe evidence and return hash-bound success metadata."""
    defects = validate_config(config_path)
    if defects:
        raise ValueError("invalid GVSOC configuration: " + "; ".join(defects))

    try:
        host_transfer = json.loads(host_transfer_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read host transfer record: {error}") from error
    if not isinstance(host_transfer, dict):
        raise ValueError("host transfer record must be a JSON object")
    expected_frame = make_scan_frame()
    if host_transfer.get("pre_cs_dummy_clock_cycles") != 8:
        raise ValueError("host transfer did not model the 8 pre-CS dummy clocks")
    if host_transfer.get("frame_tx_length") != len(expected_frame):
        raise ValueError("host transfer frame length is not 97 bytes")
    if host_transfer.get("frame_tx_hex") != expected_frame.hex():
        raise ValueError("host transfer bytes differ from the expected SCAN frame")

    try:
        log = log_path.read_text(encoding="utf-8")
    except OSError as error:
        raise ValueError(f"cannot read GVSOC log: {error}") from error
    if (
        TARGET_FRAME_MARKER not in log
        or "NANO_C_SLAM_SPI_SLAVE_PROBE:payload_ok" not in log
    ):
        raise ValueError(
            "GVSOC log lacks the exact target-side frame/payload success markers"
        )
    if "NANO_C_SLAM_SPI_SLAVE_PROBE:payload_mismatch" in log:
        raise ValueError("GVSOC target reported a payload mismatch")

    source_hashes = {str(path): _sha256(path) for path in source_paths}
    return {
        "schema_version": 1,
        "status": "passed",
        "evidence_class": "gvsoc-target-spi2-slave-receive",
        "sdk_release": "5.20.4",
        "interface": "SPI2/CS0, mode 0, 8-bit words",
        "pre_cs_dummy_clock_cycles": 8,
        "frame_bytes": len(expected_frame),
        "frame_hex": expected_frame.hex(),
        "target_markers": [
            "NANO_C_SLAM_SPI_SLAVE_PROBE:armed frame=97",
            TARGET_FRAME_MARKER,
            "NANO_C_SLAM_SPI_SLAVE_PROBE:payload_ok",
        ],
        "sha256": {
            "elf": _sha256(elf_path),
            "gvsoc_config": _sha256(config_path),
            "host_transfer": _sha256(host_transfer_path),
            "gvsoc_log": _sha256(log_path),
            "sources": source_hashes,
        },
        "limitations": [
            "Proves an exact SCAN frame reaches the GAP9 PMSIS SPI receive in GVSOC.",
            "Does not run the production SLAM loop or validate mapping accuracy.",
            "Does not validate a physical STM32/GAP9 connection or flight behavior.",
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
        print(f"GAP9 GVSOC SPI probe evidence error: {error}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"GAP9 GVSOC SPI2 receive evidence captured: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
