"""Drive the isolated GAP9 SPI2 slave probe through the GVSOC testbench."""

from __future__ import annotations

import importlib
import json
import os
from pathlib import Path
from typing import Protocol, cast

from scripts.gap9_gvsoc_spi_probe_protocol import (
    SCAN_CMD,
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

    def get_received_bytes(self, size: int | None = None) -> bytes | None: ...


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
    elif sequence == "max-mapping":
        frames = list(make_mapping_trace(260))
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
    elif sequence == "capacity-mapping":
        frames = list(
            make_mapping_trace(
                260,
                scan_interval=2,
                periodic=True,
                scans_after_poses=True,
            )
        )
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
        raise ValueError(
            "GAP9_GVSOC_SPI_SEQUENCE must be scan, pose-scan, mapping, "
            "max-mapping, or capacity-mapping"
        )
    output_root = Path(os.environ.get("GAP9_GVSOC_WORK_ROOT", "."))
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "spi_slave_probe_host.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    for frame in frames:
        spi.full_duplex(frame, len(frame))
        # SCAN dispatch runs synchronous ICP before the production target
        # arms its next receive. Give that command a longer virtual gap so
        # this diagnostic does not race target-side scan processing.
        settle_time_ps = 100_000_000_000
        if frame[1] == SCAN_CMD and sequence in {"mapping", "max-mapping"}:
            settle_time_ps = 1_000_000_000_000
        elif frame[1] == SCAN_CMD and sequence == "capacity-mapping":
            # The measured worst scan at 200 poses is below 100 ms; 250 ms
            # retains 2.5x virtual-time margin without making 131 scans take
            # more wall time than the bounded 600 s GVSOC test invocation.
            settle_time_ps = 250_000_000_000
        gvsoc.run(settle_time_ps)
        if sequence in {"mapping", "max-mapping", "capacity-mapping"}:
            rx_data = spi.get_received_bytes(len(frame))
            if rx_data is None or len(rx_data) != len(frame):
                raise RuntimeError("GVSOC did not return the command transfer RX bytes")

    if sequence in {"mapping", "max-mapping", "capacity-mapping"}:
        # Let production ICP/graph optimization finish and block in the real
        # slave send before queuing master-side response clocks.
        result["control_phase"] = "waiting_for_optimizer_and_slave_tx"
        (output_root / "spi_slave_probe_host.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        try:
            gvsoc.run(10_000_000_000_000)
        except BaseException as error:
            result["control_phase"] = "optimizer_wait_failed"
            result["control_error"] = repr(error)
            (output_root / "spi_slave_probe_host.json").write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8"
            )
            raise
        if os.environ.get("GAP9_GVSOC_SPI_CAPTURE_RESPONSE") == "1":
            result["control_phase"] = "queueing_master_response_receive"
            (output_root / "spi_slave_probe_host.json").write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8"
            )
            response_size = 18
            spi.receive(response_size)
            result["control_phase"] = "master_response_receive_queued"
            (output_root / "spi_slave_probe_host.json").write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8"
            )
            gvsoc.run(100_000_000_000)
            response = spi.get_received_bytes()
            if response is None or len(response) != response_size:
                response_length = None if response is None else len(response)
                result["response_rx_length"] = response_length
                result["control_phase"] = "optimizer_response_length_mismatch"
                (output_root / "spi_slave_probe_host.json").write_text(
                    json.dumps(result, indent=2) + "\n", encoding="utf-8"
                )
                raise RuntimeError(
                    "GVSOC returned "
                    f"{response_length} of {response_size} optimizer response bytes"
                )
            result["response_rx_hex"] = response.hex()
            result["control_phase"] = "optimizer_response_captured"
            print(f"NANO_C_SLAM_GVSOC_SPI_HOST:response_rx_hex={response.hex()}")
        else:
            result["control_phase"] = "optimizer_completed_response_not_clocked"
        (output_root / "spi_slave_probe_host.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
    return 0
