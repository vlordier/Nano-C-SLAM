"""Drive the isolated GAP9 SPI2 slave probe through the GVSOC testbench."""

from __future__ import annotations

import importlib
import json
import os
from pathlib import Path
from typing import Protocol, cast

from scripts.gap9_gvsoc_spi_probe_protocol import (
    make_mapping_trace,
    make_pose_frame,
    make_scan_frame,
)


class GvsocProxy(Protocol):
    def run(self, duration: int | None = None) -> None: ...


class SpiProxy(Protocol):
    def open(
        self,
        *,
        is_master: int,
        polarity: int,
        phase: int,
        dummy_cycles: int,
        frequency: int,
    ) -> None: ...

    def full_duplex(self, tx_data: bytes, rx_size: int) -> None: ...

    def receive(self, size: int) -> None: ...

    def get_received_bytes(self, size: int) -> bytes | None: ...


class TestbenchProxy(Protocol):
    def spi_get(self, interface: int, chip_select: int) -> SpiProxy: ...


class GvsocControlModule(Protocol):
    def Testbench(self, gvsoc: GvsocProxy) -> TestbenchProxy: ...


def target_control(gvsoc: GvsocProxy) -> int:
    """Drive the SCAN slave probe or bounded production-main probe."""
    config = json.loads(Path("gvsoc_config.json").read_text(encoding="utf-8"))
    target = config.get("target", {})
    if target.get("addon_testbench_enabled") is not True:
        raise RuntimeError("GVSOC testbench is not enabled in generated config")
    components = target.get("components", [])
    if "testbench" not in components:
        raise RuntimeError("GVSOC testbench component is absent")
    bindings = {tuple(binding) for binding in target.get("bindings", [])}
    required_bindings = {
        ("chip->spim2_cs0_data", "testbench->spi2_cs0_data"),
        ("chip->spim2_cs0", "testbench->spi2_cs0"),
    }
    if not required_bindings.issubset(bindings):
        raise RuntimeError("GVSOC SPI2 testbench bindings are absent")

    gv_control = cast(GvsocControlModule, importlib.import_module("gv.gvsoc_control"))
    testbench = gv_control.Testbench(gvsoc)
    spi = testbench.spi_get(2, 0)
    spi.open(is_master=1, polarity=0, phase=0, dummy_cycles=8, frequency=1_000_000)

    # The target boots from flash and arms its 97-byte PMSIS receive first.
    gvsoc.run(500_000_000_000)
    sequence = os.environ.get("GAP9_GVSOC_SPI_SEQUENCE", "scan")
    result: dict[str, object]
    if sequence == "scan":
        frames = [make_scan_frame()]
        result = {
            "sequence": sequence,
            "pre_cs_dummy_clock_cycles": 8,
            "frame_tx_hex": frames[0].hex(),
            "frame_tx_length": len(frames[0]),
            "master_rx_capture": "not required; acceptance uses "
            "target-side receive marker",
        }
    elif sequence == "pose-scan":
        frames = [make_pose_frame(), make_scan_frame()]
        result = {
            "sequence": sequence,
            "pre_cs_dummy_clock_cycles": 8,
            "frames": [
                {
                    "command": command,
                    "frame_tx_hex": frame.hex(),
                    "length": len(frame),
                }
                for command, frame in zip(("POSE", "SCAN"), frames, strict=True)
            ],
            "master_rx_capture": "not required; acceptance uses "
            "target-side receive marker",
        }
    elif sequence == "mapping":
        frames = list(make_mapping_trace())
        result = {
            "sequence": sequence,
            "pre_cs_dummy_clock_cycles": 8,
            "frames": [
                {
                    "command": (
                        "POSE_BAD_CHECKSUM"
                        if index == 0
                        else {0: "POSE", 1: "SCAN", 2: "OPTIMIZE"}[frame[1]]
                    ),
                    "frame_tx_hex": frame.hex(),
                    "length": len(frame),
                }
                for index, frame in enumerate(frames)
            ],
        }
    else:
        raise ValueError("GAP9_GVSOC_SPI_SEQUENCE must be scan, pose-scan, or mapping")
    output_root = Path(os.environ.get("GAP9_GVSOC_WORK_ROOT", "."))
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "spi_slave_probe_host.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    for frame in frames:
        spi.full_duplex(frame, len(frame))
        gvsoc.run(100_000_000_000)
        if sequence == "mapping":
            rx_data = spi.get_received_bytes(len(frame))
            if rx_data is None or len(rx_data) != len(frame):
                raise RuntimeError("GVSOC did not return the command transfer RX bytes")

    if sequence == "mapping":
        # Let production ICP/graph optimization finish. The current verified
        # fixture compiles an explicit target-side TX skip; response capture is
        # tracked separately while the GVSOC master receive sequence is debugged.
        result["control_phase"] = "waiting_for_optimizer_and_slave_tx"
        (output_root / "spi_slave_probe_host.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        gvsoc.run(1_000_000_000_000)
        result["control_phase"] = "optimizer_completed_response_not_clocked"
        (output_root / "spi_slave_probe_host.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
    return 0
