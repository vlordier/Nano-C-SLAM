"""Pure tests for Gazebo LaserScan conversion and the SITL UDP packet."""

from __future__ import annotations

import binascii
import json
import struct
import unittest

from scripts.crazysim_tof_bridge import (
    FRAME_CRC,
    FRAME_HEADER,
    TofUdpBridge,
    encode_matrix_frame,
    parse_scan_message,
)


def _scan(
    sensor_id: int,
    seconds: int,
    ranges: list[float | str] | None = None,
) -> bytes:
    return json.dumps(
        {
            "header": {"stamp": {"sec": str(seconds), "nsec": "1000000"}},
            "count": 8,
            "verticalCount": 8,
            "rangeMin": 0.02,
            "rangeMax": 4.0,
            "sensor_id": sensor_id,
            "ranges": ranges if ranges is not None else [1.25] * 64,
        }
    ).encode()


class CrazySimTofBridgeTests(unittest.TestCase):
    def test_scan_conversion_preserves_row_major_grid_and_invalid_returns(self) -> None:
        scan = parse_scan_message(
            _scan(0, 12, [0.02, 4.0, 1.2344, "Infinity", 0.01, 4.1] + [0.5] * 58)
        )

        self.assertEqual(scan.sim_time_ns, 12_001_000_000)
        self.assertEqual(scan.millimeters[:6], (20, 4000, 1234, -1, -1, -1))
        self.assertEqual(len(scan.millimeters), 64)

    def test_scan_rejects_missing_timestamp_and_wrong_matrix_dimensions(self) -> None:
        message = json.loads(_scan(0, 12))
        del message["header"]["stamp"]
        with self.assertRaisesRegex(ValueError, "timestamp"):
            parse_scan_message(json.dumps(message))

        message = json.loads(_scan(0, 12))
        message["verticalCount"] = 7
        with self.assertRaisesRegex(ValueError, "8 horizontal"):
            parse_scan_message(json.dumps(message))

    def test_four_coherent_scans_encode_one_wire_compatible_frame(self) -> None:
        bridge = TofUdpBridge(
            gz_command="gz",
            topic_prefix="/cf_0/tof",
            udp_host="127.0.0.1",
            udp_port=19951,
        )
        try:
            frame = None
            for sensor_id in range(4):
                frame = bridge.accept_scan(sensor_id, _scan(sensor_id, 10))

            self.assertIsNotNone(frame)
            assert frame is not None
            self.assertEqual(len(frame), 540)
            magic, version, sensors, rows, columns, sequence, sim_time, payload_size = (
                FRAME_HEADER.unpack_from(frame)
            )
            self.assertEqual(
                (
                    magic,
                    version,
                    sensors,
                    rows,
                    columns,
                    sequence,
                    sim_time,
                    payload_size,
                ),
                (b"NCTF", 1, 4, 8, 8, 0, 10_001_000_000, 512),
            )
            payload = struct.unpack_from("!256h", frame, FRAME_HEADER.size)
            self.assertEqual(payload, (1250,) * 256)
            checksum = FRAME_CRC.unpack_from(frame, len(frame) - FRAME_CRC.size)[0]
            self.assertEqual(checksum, binascii.crc32(frame[:-4]) & 0xFFFFFFFF)
        finally:
            bridge.socket.close()

    def test_skewed_or_repeated_scans_do_not_emit_duplicate_frames(self) -> None:
        bridge = TofUdpBridge(
            gz_command="gz",
            topic_prefix="/cf_0/tof",
            udp_host="127.0.0.1",
            udp_port=19951,
            max_skew_ns=10_000_000,
        )
        try:
            self.assertIsNone(bridge.accept_scan(0, _scan(0, 10)))
            self.assertIsNone(bridge.accept_scan(1, _scan(1, 10)))
            self.assertIsNone(bridge.accept_scan(2, _scan(2, 10)))
            self.assertIsNone(bridge.accept_scan(3, _scan(3, 11)))
            self.assertIsNone(bridge.accept_scan(0, _scan(0, 12)))
            self.assertIsNone(bridge.accept_scan(1, _scan(1, 12)))
            self.assertIsNone(bridge.accept_scan(2, _scan(2, 12)))
            frame = bridge.accept_scan(3, _scan(3, 12))
            self.assertIsNotNone(frame)
            self.assertIsNone(bridge.accept_scan(3, _scan(3, 12)))
        finally:
            bridge.socket.close()

    def test_matrix_encoder_rejects_out_of_range_sensor_values(self) -> None:
        matrices = ((0,) * 64, (0,) * 64, (0,) * 64, (4001,) * 64)
        with self.assertRaisesRegex(ValueError, "matrix values"):
            encode_matrix_frame(0, 1, matrices)


if __name__ == "__main__":
    unittest.main()
