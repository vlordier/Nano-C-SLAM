"""Drive the isolated GAP9 SPI2 slave probe through the GVSOC testbench."""

from __future__ import annotations

import importlib
import json
import os
import socket
from pathlib import Path
from typing import Protocol, cast

from scripts.gap9_gvsoc_spi_probe_protocol import (
    POSE_REQ_CMD,
    SCAN_CMD,
    make_mapping_trace,
    make_pose_frame,
    make_pose_request_frame,
    make_scan_frame,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.firmware_protocol import (
    OPTIMIZE_CMD,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_packets import (
    encode_mapping_samples,
    samples_from_run_bundle,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_stream import (
    MappingStreamDecoder,
    iter_mapping_stream_records,
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


def _run_live_mapping_stream(
    gvsoc: GvsocProxy,
    spi: SpiProxy,
    *,
    socket_path: Path,
    ready_path: Path,
    run_id: str,
    drone_id: str,
    packet_count: int,
    timeout_s: float,
    output_path: Path,
) -> dict[str, object]:
    """Receive one bounded live run and clock each frame into production SPI."""
    if socket_path.exists() or ready_path.exists():
        raise FileExistsError("live GVSOC socket or readiness marker already exists")
    if len(os.fsencode(socket_path)) >= 104:
        raise ValueError("live GVSOC Unix socket path exceeds the local limit")
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client: socket.socket | None = None
    socket_owned = False
    readiness_owned = False
    frames: list[dict[str, object]] = []
    transfers: list[dict[str, int]] = []
    result: dict[str, object] = {
        "sequence": "gazebo-live",
        "run_id": run_id,
        "drone_id": drone_id,
        "pre_cs_dummy_clock_cycles": 8,
        "frames": frames,
        "command_transfer_lengths": transfers,
    }
    try:
        server.bind(str(socket_path))
        socket_owned = True
        server.listen(1)
        server.settimeout(timeout_s)
        with ready_path.open("x", encoding="utf-8") as ready:
            readiness_owned = True
            ready.write(f"{run_id}\n")
            ready.flush()
            os.fsync(ready.fileno())
        client, _ = server.accept()
        decoder = MappingStreamDecoder(run_id=run_id, drone_id=drone_id)
        optimize_seen = False
        for record in iter_mapping_stream_records(client, decoder, timeout_s=timeout_s):
            if optimize_seen:
                raise ValueError("live stream contains commands after OPTIMIZE")
            spi.full_duplex(record.frame, len(record.frame))
            gvsoc.run(
                1_000_000_000_000 if record.command == SCAN_CMD else 100_000_000_000
            )
            rx_data = spi.get_received_bytes(len(record.frame))
            if rx_data is None or len(rx_data) != len(record.frame):
                raise RuntimeError("GVSOC did not complete a live SPI frame")
            frames.append(
                {
                    "sequence": record.sequence,
                    "command": record.command,
                    "pose_id": record.pose_id,
                    "source_stamp_ns": record.source_stamp_ns,
                    "frame_tx_hex": record.frame.hex(),
                    "length": len(record.frame),
                }
            )
            transfers.append(
                {"master_tx_bytes": len(record.frame), "master_rx_bytes": len(rx_data)}
            )
            result["control_phase"] = f"received_{len(frames)}_frames"
            output_path.write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8"
            )
            optimize_seen = record.command == OPTIMIZE_CMD
        decoder.finish()
        if len(frames) != packet_count:
            raise ValueError(
                f"live stream ended after {len(frames)} frames; expected {packet_count}"
            )
        if not optimize_seen:
            raise ValueError("live stream ended without an OPTIMIZE command")
        result["control_phase"] = "waiting_for_optimizer_and_slave_tx"
        output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        gvsoc.run(10_000_000_000_000)
        spi.receive(18)
        gvsoc.run(100_000_000_000)
        response = spi.get_received_bytes()
        if response is None or len(response) != 18:
            raise RuntimeError("GVSOC did not capture the 18-byte optimizer response")
        result["response_rx_hex"] = response.hex()
        result["control_phase"] = "optimizer_response_captured"
        output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        return result
    finally:
        if client is not None:
            client.close()
        server.close()
        if socket_owned:
            socket_path.unlink(missing_ok=True)
        if readiness_owned:
            ready_path.unlink(missing_ok=True)


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
    elif sequence == "pose-request":
        frames = [
            make_pose_frame(0, position=(0.25, -0.5, 0.75)),
            make_pose_frame(1, position=(1.25, -1.5, 1.75)),
            make_pose_request_frame(0, 2),
        ]
        result = {
            "sequence": sequence,
            "pre_cs_dummy_clock_cycles": 8,
            "frames": [
                {
                    "command": command,
                    "frame_tx_hex": frame.hex(),
                    "length": len(frame),
                }
                for command, frame in zip(
                    ("POSE", "POSE", "POSE_REQ"), frames, strict=True
                )
            ],
            "response_size": 24,
        }
    elif sequence == "pose-request-boundaries":
        requests = ((0, 0), (0, -1), (0, 101), (2, 1), (0, 2))
        frames = [
            make_pose_frame(0, position=(0.25, -0.5, 0.75)),
            make_pose_frame(1, position=(1.25, -1.5, 1.75)),
            *(make_pose_request_frame(start, count) for start, count in requests),
        ]
        result = {
            "sequence": sequence,
            "pre_cs_dummy_clock_cycles": 8,
            "frames": [
                {
                    "command": command,
                    "frame_tx_hex": frame.hex(),
                    "length": len(frame),
                }
                for command, frame in zip(
                    (
                        "POSE",
                        "POSE",
                        "POSE_REQ_ZERO",
                        "POSE_REQ_NEGATIVE",
                        "POSE_REQ_OVER_CAPACITY",
                        "POSE_REQ_UNAVAILABLE_RANGE",
                        "POSE_REQ_VALID",
                    ),
                    frames,
                    strict=True,
                )
            ],
            "response_size": 24,
            "rejected_requests": [
                {"from_id": start, "count": count} for start, count in requests[:-1]
            ],
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
    elif sequence == "gazebo-capture":
        capture_path = os.environ.get("GAP9_GVSOC_CAPTURE_RUN_JSON")
        if not capture_path:
            raise ValueError(
                "GAP9_GVSOC_CAPTURE_RUN_JSON is required for gazebo-capture"
            )
        bundle = json.loads(Path(capture_path).read_text(encoding="utf-8"))
        run_id, samples = samples_from_run_bundle(bundle)
        packets = encode_mapping_samples(samples)
        frames = [packet.frame for packet in packets]
        result = {
            "sequence": sequence,
            "run_id": run_id,
            "source_run_json": str(Path(capture_path).resolve()),
            "input_pose_count": len(samples),
            "pre_cs_dummy_clock_cycles": 8,
            "frames": [
                {
                    "sequence": packet.sequence,
                    "command": packet.command,
                    "pose_id": packet.pose_id,
                    "source_stamp_ns": packet.source_stamp_ns,
                    "frame_tx_hex": packet.frame.hex(),
                    "length": len(packet.frame),
                }
                for packet in packets
            ],
            "command_transfer_lengths": [],
        }
    elif sequence == "gazebo-live":
        socket_value = os.environ.get("GAP9_GVSOC_SPI_STREAM_SOCKET")
        ready_value = os.environ.get("GAP9_GVSOC_SPI_STREAM_READY")
        run_id = os.environ.get("GAP9_GVSOC_SPI_STREAM_RUN_ID")
        drone_id = os.environ.get("GAP9_GVSOC_SPI_STREAM_DRONE", "cf1")
        if not socket_value or not ready_value or not run_id:
            raise ValueError(
                "live stream socket, readiness marker, and run ID environment "
                "variables are required"
            )
        try:
            timeout_s = float(os.environ.get("GAP9_GVSOC_SPI_STREAM_TIMEOUT_S", "15"))
            packet_count = int(
                os.environ.get("GAP9_BUILD_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT", "64")
            )
        except ValueError as error:
            raise ValueError(
                "live stream timeout and packet count must be numeric"
            ) from error
        live_result = _run_live_mapping_stream(
            gvsoc,
            spi,
            socket_path=Path(socket_value),
            ready_path=Path(ready_value),
            run_id=run_id,
            drone_id=drone_id,
            packet_count=packet_count,
            timeout_s=timeout_s,
            output_path=Path(os.environ.get("GAP9_GVSOC_WORK_ROOT", "."))
            / "spi_slave_probe_host.json",
        )
        print(
            "NANO_C_SLAM_GVSOC_SPI_HOST:live_response_rx_hex="
            f"{live_result['response_rx_hex']}"
        )
        return 0
    else:
        raise ValueError(
            "GAP9_GVSOC_SPI_SEQUENCE must be scan, pose-scan, pose-request, "
            "pose-request-boundaries, mapping, "
            "max-mapping, capacity-mapping, or gazebo-capture"
        )
    mapping_sequences = {
        "mapping",
        "max-mapping",
        "capacity-mapping",
        "gazebo-capture",
    }
    if sequence in mapping_sequences:
        result["command_transfer_lengths"] = []
    output_root = Path(os.environ.get("GAP9_GVSOC_WORK_ROOT", "."))
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "spi_slave_probe_host.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    for frame_index, frame in enumerate(frames):
        spi.full_duplex(frame, len(frame))
        # SCAN dispatch runs synchronous ICP before the production target
        # arms its next receive. Give that command a longer virtual gap so
        # this diagnostic does not race target-side scan processing.
        settle_time_ps = 100_000_000_000
        if frame[1] == SCAN_CMD and sequence in {
            "mapping",
            "max-mapping",
            "gazebo-capture",
        }:
            settle_time_ps = 1_000_000_000_000
        elif frame[1] == SCAN_CMD and sequence == "capacity-mapping":
            # The measured worst scan at 200 poses is below 100 ms; 250 ms
            # retains 2.5x virtual-time margin without making 131 scans take
            # more wall time than the bounded 600 s GVSOC test invocation.
            settle_time_ps = 250_000_000_000
        elif frame[1] == POSE_REQ_CMD and sequence in {
            "pose-request",
            "pose-request-boundaries",
        }:
            final_request = frame_index == len(frames) - 1
            settle_time_ps = 1_000_000_000 if final_request else 100_000_000_000
        gvsoc.run(settle_time_ps)
        if sequence in {
            "mapping",
            "max-mapping",
            "capacity-mapping",
            "gazebo-capture",
            "pose-request",
            "pose-request-boundaries",
        }:
            rx_data = spi.get_received_bytes(len(frame))
            if rx_data is None or len(rx_data) != len(frame):
                raise RuntimeError(
                    "GVSOC did not return the complete command transfer RX bytes"
                )
            if sequence in mapping_sequences:
                transfer_lengths = cast(
                    list[dict[str, int]], result["command_transfer_lengths"]
                )
                transfer_lengths.append(
                    {"master_tx_bytes": len(frame), "master_rx_bytes": len(rx_data)}
                )

    if sequence in {"pose-request", "pose-request-boundaries"}:
        # Let production dispatch assert ready and block in its real slave TX
        # before queuing the distinct master response transaction.
        result["control_phase"] = "allowing_target_to_assert_ready"
        (output_root / "spi_slave_probe_host.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        gvsoc.run(1_000_000_000)
        result["control_phase"] = "queueing_pose_response_receive"
        spi.receive(24)
        gvsoc.run(10_000_000_000)
        response = spi.get_received_bytes()
        if response is None or len(response) != 24:
            result["control_phase"] = "pose_response_length_mismatch"
            result["response_rx_length"] = None if response is None else len(response)
            (output_root / "spi_slave_probe_host.json").write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8"
            )
            raise RuntimeError("GVSOC did not capture all 24 POSE_REQ response bytes")
        result["response_rx_hex"] = response.hex()
        result["control_phase"] = "pose_response_captured"
        print(f"NANO_C_SLAM_GVSOC_SPI_HOST:pose_response_rx_hex={response.hex()}")
        (output_root / "spi_slave_probe_host.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )

    if sequence in mapping_sequences:
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
