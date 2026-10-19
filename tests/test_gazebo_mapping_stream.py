"""Tests for the bounded Gazebo-to-GVSOC local frame stream."""

from __future__ import annotations

import json
import socket
import time
import unittest
from contextlib import ExitStack

from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_model import (
    MappingSample,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_packets import (
    MappingPacket,
    encode_mapping_samples,
    mapping_sample_from_pair,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.mapping_stream import (
    MAX_RECORD_BYTES,
    MappingStreamDecoder,
    MappingStreamWriter,
    encode_mapping_stream_record,
    iter_mapping_stream_records,
    send_mapping_stream_record,
)
from sim.gazebo.nano_c_slam_gazebo.nano_c_slam_gazebo.telemetry_model import (
    PairedTelemetry,
    StampedOdometrySample,
    StampedRangeSample,
)

RUN_ID = "15a06f08-657e-4012-922e-4ca35f7dda22"


def _packets() -> tuple[MappingPacket, ...]:
    sample = MappingSample(
        sample_id=0,
        timestamp_ms=100,
        x_m=0.25,
        y_m=-0.5,
        yaw_rad=0.75,
        ranges_mm=(700, 800, 900, 1000),
        source_stamp_ns=100_000_123,
    )
    return encode_mapping_samples([sample], scan_window_size=1)


class MappingStreamTests(unittest.TestCase):
    def test_live_writer_frames_match_offline_packetization_byte_for_byte(self) -> None:
        pairs = tuple(
            PairedTelemetry(
                ranges=StampedRangeSample("cf1", stamp, index, (700, 800, 900, 1000)),
                odometry=StampedOdometrySample(
                    "cf1", stamp, index * 0.1, -index * 0.1, index * 0.01
                ),
                skew_ns=0,
            )
            for index, stamp in enumerate((100_000_000, 200_000_000))
        )
        expected_samples = [
            mapping_sample_from_pair(pair, pose_id=index)
            for index, pair in enumerate(pairs)
        ]
        expected_packets = encode_mapping_samples(expected_samples, scan_window_size=20)
        with ExitStack() as stack:
            sender, receiver = socket.socketpair()
            stack.enter_context(sender)
            stack.enter_context(receiver)
            writer = MappingStreamWriter(
                sender, run_id=RUN_ID, drone_id="cf1", timeout_s=1.0
            )
            for index, pair in enumerate(pairs):
                writer.push_pair(pair, pose_id=index)
            writer.finish()
            sender.shutdown(socket.SHUT_WR)
            actual_records = tuple(
                iter_mapping_stream_records(
                    receiver,
                    MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1"),
                    timeout_s=1.0,
                )
            )

        self.assertEqual(
            [record.frame for record in actual_records],
            [packet.frame for packet in expected_packets],
        )
        self.assertEqual(
            [record.sequence for record in actual_records],
            [packet.sequence for packet in expected_packets],
        )

    def test_live_writer_stops_on_peer_disconnect(self) -> None:
        pair = PairedTelemetry(
            StampedRangeSample("cf1", 100_000_000, 0, (700, 800, 900, 1000)),
            StampedOdometrySample("cf1", 100_000_000, 0.0, 0.0, 0.0),
            0,
        )
        with ExitStack() as stack:
            sender, receiver = socket.socketpair()
            stack.enter_context(sender)
            stack.enter_context(receiver)
            receiver.close()
            writer = MappingStreamWriter(
                sender, run_id=RUN_ID, drone_id="cf1", timeout_s=0.1
            )
            with self.assertRaises(OSError):
                writer.push_pair(pair, pose_id=0)

    def test_round_trip_fragmented_and_coalesced_records(self) -> None:
        packets = _packets()
        encoded = b"".join(
            encode_mapping_stream_record(RUN_ID, "cf1", packet) for packet in packets
        )
        decoder = MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1")

        first = decoder.feed(encoded[:17])
        second = decoder.feed(encoded[17:400])
        third = decoder.feed(encoded[400:])
        decoder.finish()

        records = first + second + third
        self.assertEqual(len(records), 3)
        self.assertEqual([record.sequence for record in records], [0, 1, 2])
        self.assertEqual(
            [record.frame for record in records], [p.frame for p in packets]
        )
        self.assertEqual(records[0].source_stamp_ns, 100_000_123)
        self.assertEqual(records[0].pose_id, 0)
        self.assertEqual(records[1].command, 1)
        self.assertIsNone(records[2].source_stamp_ns)

    def test_socket_pair_transfers_records_with_eof_validation(self) -> None:
        packets = _packets()
        decoder = MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1")
        with ExitStack() as stack:
            sender, receiver = socket.socketpair()
            stack.enter_context(sender)
            stack.enter_context(receiver)
            for packet in packets:
                send_mapping_stream_record(sender, RUN_ID, "cf1", packet, timeout_s=1.0)
            sender.shutdown(socket.SHUT_WR)
            received = tuple(
                iter_mapping_stream_records(receiver, decoder, timeout_s=1.0)
            )
        self.assertEqual(
            [record.frame for record in received], [p.frame for p in packets]
        )

    def test_rejects_wrong_identity_sequence_and_bad_wire_fields(self) -> None:
        packet = _packets()[0]
        encoded = encode_mapping_stream_record(RUN_ID, "cf1", packet)
        decoder_cases = (
            (MappingStreamDecoder(run_id=RUN_ID, drone_id="cf3"), "different drone"),
            (
                MappingStreamDecoder(
                    run_id="00000000-0000-0000-0000-000000000001", drone_id="cf1"
                ),
                "different run",
            ),
        )
        for decoder, message in decoder_cases:
            with (
                self.subTest(message=message),
                self.assertRaisesRegex(ValueError, message),
            ):
                decoder.feed(encoded)

        wrong_sequence = json.loads(encoded)
        wrong_sequence["sequence"] = 1
        wrong_identity = json.loads(encoded)
        wrong_identity["drone_id"] = "cf3"
        wrong_command = json.loads(encoded)
        wrong_command["command"] = 9
        malformed = (
            (wrong_sequence, "out of order"),
            (wrong_identity, "different drone"),
            (wrong_command, "command must be POSE"),
        )
        for value, message in malformed:
            line = json.dumps(value, separators=(",", ":")).encode() + b"\n"
            with (
                self.subTest(message=message),
                self.assertRaisesRegex(ValueError, message),
            ):
                MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1").feed(line)

    def test_rejects_invalid_checksums_oversized_records_and_partial_eof(self) -> None:
        packet = _packets()[0]
        invalid_frame = bytearray(packet.frame)
        invalid_frame[20] ^= 1
        value = json.loads(encode_mapping_stream_record(RUN_ID, "cf1", packet))
        value["frame_hex"] = invalid_frame.hex()
        invalid_record = json.dumps(value, separators=(",", ":")).encode() + b"\n"
        decoder = MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1")
        with self.assertRaisesRegex(ValueError, "checksum"):
            decoder.feed(invalid_record)

        with self.assertRaisesRegex(ValueError, "8192-byte"):
            MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1").feed(bytes(8193))
        with self.assertRaisesRegex(ValueError, "512-byte"):
            MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1").feed(
                b"x" * (MAX_RECORD_BYTES + 1)
            )
        partial = MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1")
        partial.feed(b'{"schema_version":1')
        with self.assertRaisesRegex(ValueError, "truncated"):
            partial.finish()

    def test_socket_read_timeout_is_finite(self) -> None:
        decoder = MappingStreamDecoder(run_id=RUN_ID, drone_id="cf1")
        with ExitStack() as stack:
            sender, receiver = socket.socketpair()
            stack.enter_context(sender)
            stack.enter_context(receiver)
            with self.assertRaises(TimeoutError):
                next(iter_mapping_stream_records(receiver, decoder, timeout_s=0.01))
            self.assertEqual(receiver.gettimeout(), None)

    def test_socket_backpressure_surfaces_as_a_finite_send_timeout(self) -> None:
        packet = _packets()[0]
        with ExitStack() as stack:
            sender, receiver = socket.socketpair()
            stack.enter_context(sender)
            stack.enter_context(receiver)
            sender.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)
            sent_count = 0
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    send_mapping_stream_record(
                        sender, RUN_ID, "cf1", packet, timeout_s=0.01
                    )
                except TimeoutError:
                    break
                sent_count += 1
            else:
                self.fail("undrained socket did not apply bounded backpressure")
            self.assertGreater(sent_count, 0)
            self.assertIsNone(sender.gettimeout())


if __name__ == "__main__":
    unittest.main()
