"""Hardware-free test for live Unix-socket to GVSOC SPI command forwarding."""

from __future__ import annotations

import contextlib
import io
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from typing import cast

from scripts.gap9_gvsoc_spi_probe_protocol import make_pose_frame
from scripts.gvsoc_spi_slave_probe_control import _run_live_mapping_stream
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.firmware_protocol import (
    make_optimize_frame,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_packets import (
    MappingPacket,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_stream import (
    send_mapping_stream_record,
)

RUN_ID = "15a06f08-657e-4012-922e-4ca35f7dda22"


class _Gvsoc:
    def __init__(self) -> None:
        self.durations: list[int | None] = []

    def run(self, duration: int | None = None) -> None:
        self.durations.append(duration)


class _Spi:
    def __init__(self) -> None:
        self.frames: list[bytes] = []
        self.response_requested = False

    def open(
        self,
        *,
        is_master: int,
        polarity: int,
        phase: int,
        dummy_cycles: int,
        frequency: int,
    ) -> None:
        del is_master, polarity, phase, dummy_cycles, frequency

    def full_duplex(self, tx_data: bytes, rx_size: int) -> None:
        self.assert_length = rx_size
        self.frames.append(tx_data)

    def receive(self, size: int) -> None:
        self.response_requested = size == 18

    def get_received_bytes(self, size: int | None = None) -> bytes:
        return bytes(size if size is not None else 18)


class LiveGvsocReceiverTests(unittest.TestCase):
    def test_receiver_publishes_readiness_forwards_frames_and_captures_response(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            socket_path = root / "firmware.sock"
            ready_path = root / "firmware.ready"
            output_path = root / "host-transfer.json"
            gvsoc = _Gvsoc()
            spi = _Spi()
            outcome: list[dict[str, object]] = []
            failure: list[BaseException] = []
            control_output = io.StringIO()

            def target() -> None:
                try:
                    with contextlib.redirect_stdout(control_output):
                        outcome.append(
                            _run_live_mapping_stream(
                                gvsoc,
                                spi,
                                socket_path=socket_path,
                                ready_path=ready_path,
                                run_id=RUN_ID,
                                drone_id="cf1",
                                packet_count=2,
                                timeout_s=2.0,
                                output_path=output_path,
                            )
                        )
                except BaseException as error:
                    failure.append(error)

            target_thread = threading.Thread(target=target)
            target_thread.start()
            deadline = time.monotonic() + 1.0
            while not ready_path.is_file() and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(ready_path.is_file())
            pose = make_pose_frame(0, timestamp=100, position=(0.25, -0.5, 0.75))
            frames = (pose, make_optimize_frame())
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(str(socket_path))
                for sequence, (command, frame) in enumerate(
                    zip((0, 2), frames, strict=True)
                ):
                    send_mapping_stream_record(
                        client,
                        RUN_ID,
                        "cf1",
                        MappingPacket(
                            sequence=sequence,
                            command=command,
                            pose_id=0 if command == 0 else None,
                            source_stamp_ns=100_000_000 if command == 0 else None,
                            frame=frame,
                        ),
                        timeout_s=1.0,
                    )
                client.shutdown(__import__("socket").SHUT_WR)
            target_thread.join(timeout=2.0)

            self.assertFalse(target_thread.is_alive())
            self.assertEqual(failure, [])
            self.assertEqual(spi.frames, list(frames))
            self.assertTrue(spi.response_requested)
            self.assertEqual(outcome[0]["response_rx_hex"], bytes(18).hex())
            self.assertIn(
                f"NANO_C_SLAM_GVSOC_SPI_HOST:live_response_rx_hex={bytes(18).hex()}",
                control_output.getvalue(),
            )
            transfer_lengths = cast(
                list[object], outcome[0]["command_transfer_lengths"]
            )
            self.assertIsInstance(transfer_lengths, list)
            self.assertEqual(len(transfer_lengths), 2)
            self.assertFalse(socket_path.exists())
            self.assertFalse(ready_path.exists())


if __name__ == "__main__":
    unittest.main()
