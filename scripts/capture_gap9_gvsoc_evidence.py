"""Capture a bounded GAP9 GVSOC smoke result as reproducible JSON evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

MARKER = re.compile(
    r"NANO_C_SLAM_GVSOC:icp_svd_ok sigma0_milli=(\d+) sigma1_milli=(\d+)"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    elf = args.elf.resolve(strict=True)
    log = args.log.resolve(strict=True)
    match = MARKER.search(log.read_text(encoding="utf-8", errors="replace"))
    passed = match is not None and match.groups() == ("3000", "2000")
    report = {
        "schema_version": 1,
        "status": "passed" if passed else "failed",
        "evidence_class": "gvsoc-target-execution",
        "claim_boundary": (
            "ICP SVD kernel under GVSOC only; not full SLAM or hardware validation"
        ),
        "hardware_measurement": False,
        "test": "diagonal covariance SVD in Nano-C-SLAM ICP implementation",
        "expected_sigma_milli": [3000, 2000],
        "observed_sigma_milli": [int(match[1]), int(match[2])] if match else None,
        "elf_bytes": elf.stat().st_size,
        "elf_sha256": sha256(elf),
        "log_sha256": sha256(log),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
