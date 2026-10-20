"""Golden and adversarial vectors for the CrazySim UWB datagram contract."""

from __future__ import annotations

import binascii
import struct
import unittest
from dataclasses import replace

from scripts.uwb_sitl_protocol import (
    BROKER_ID,
    HEADER,
    RANGE_VECTOR,
    PacketKind,
    UwbDatagram,
    decode_datagram,
    encode_datagram,
)


class UwbSitlProtocolTests(unittest.TestCase):
    def test_registration_golden_packet_has_exact_header_and_crc(self) -> None:
        packet = UwbDatagram(
            kind=PacketKind.REGISTER,
            sequence=0,
            sim_time_ns=0,
            source_id=1,
            destination_id=BROKER_ID,
            payload=struct.pack("!H", 19953),
        )

        encoded = encode_datagram(packet)

        self.assertEqual(len(encoded), HEADER.size + 2 + 4)
        self.assertEqual(encoded[:4], b"NCSU")
        self.assertEqual(encoded[4:8], bytes((1, 1, 0, 0)))
        self.assertEqual(struct.unpack_from("!I", encoded, 8)[0], 0)
        self.assertEqual(struct.unpack_from("!Q", encoded, 12)[0], 0)
        self.assertEqual(encoded[20:24], bytes((1, 255, 0, 2)))
        self.assertEqual(struct.unpack_from("!H", encoded, 24)[0], 19953)
        self.assertEqual(
            struct.unpack_from("!I", encoded, len(encoded) - 4)[0],
            binascii.crc32(encoded[:-4]) & 0xFFFFFFFF,
        )
        self.assertEqual(
            encoded.hex(),
            "4e4353550101000000000000000000000000000001ff00024df1141c28f6",
        )
        self.assertEqual(decode_datagram(encoded), packet)

    def test_range_response_preserves_peer_truth_and_timestamp(self) -> None:
        packet = UwbDatagram(
            kind=PacketKind.RANGE_RESPONSE,
            sequence=7,
            sim_time_ns=12_345_000_000,
            source_id=1,
            destination_id=0,
            payload=RANGE_VECTOR.pack(2345, -100, 200, 1500),
        )

        decoded = decode_datagram(encode_datagram(packet))

        self.assertEqual(decoded, packet)
        self.assertEqual(
            encode_datagram(packet).hex(),
            "4e435355010400000000000700000002dfd1c0400100001000000929"
            "ffffff9c000000c8000005dc6d10c9a8",
        )
        self.assertEqual(RANGE_VECTOR.unpack(decoded.payload), (2345, -100, 200, 1500))

    def test_app_message_identity_and_size_are_bound_to_header(self) -> None:
        valid_message = bytes((0xDE, 0, 1, 7, 0x42))
        packet = UwbDatagram(
            kind=PacketKind.MESSAGE_REQUEST,
            sequence=3,
            sim_time_ns=1_000_000,
            source_id=0,
            destination_id=1,
            payload=valid_message,
        )
        encoded = encode_datagram(packet)
        self.assertEqual(decode_datagram(encoded), packet)
        self.assertEqual(
            encoded.hex(),
            "4e435355010500000000000300000000000f424000010005de00010742786c8d8e",
        )

        with self.assertRaisesRegex(ValueError, "source ID"):
            encode_datagram(replace(packet, payload=bytes((0xDE, 1, 1, 7, 0x42))))
        with self.assertRaisesRegex(ValueError, "API bound"):
            encode_datagram(
                replace(packet, payload=bytes((0xDE, 0, 1, 7)) + bytes(115))
            )

    def test_range_and_message_routes_are_directional(self) -> None:
        request = UwbDatagram(
            kind=PacketKind.RANGE_REQUEST,
            sequence=4,
            sim_time_ns=100,
            source_id=0,
            destination_id=1,
        )
        with self.assertRaisesRegex(ValueError, "node-to-node"):
            encode_datagram(replace(request, destination_id=BROKER_ID))
        with self.assertRaisesRegex(ValueError, "must differ"):
            encode_datagram(replace(request, destination_id=0))

        response_message = bytes((0xDE, 1, 0, 2, 0xAA))
        response = UwbDatagram(
            kind=PacketKind.MESSAGE_RESPONSE,
            sequence=8,
            sim_time_ns=200,
            source_id=1,
            destination_id=0,
            payload=response_message,
        )
        self.assertEqual(decode_datagram(encode_datagram(response)), response)
        with self.assertRaisesRegex(ValueError, "source ID"):
            encode_datagram(replace(response, source_id=0))

    def test_crc_length_kind_flags_and_time_fail_closed(self) -> None:
        valid = bytearray(
            encode_datagram(
                UwbDatagram(
                    kind=PacketKind.RANGE_REQUEST,
                    sequence=1,
                    sim_time_ns=10,
                    source_id=0,
                    destination_id=1,
                )
            )
        )
        damaged_crc = bytearray(valid)
        damaged_crc[-1] ^= 1
        with self.assertRaisesRegex(ValueError, "CRC"):
            decode_datagram(bytes(damaged_crc))
        with self.assertRaisesRegex(ValueError, "length"):
            decode_datagram(bytes(valid[:-1]))

        unknown_kind = bytearray(valid)
        unknown_kind[5] = 99
        unknown_kind[-4:] = struct.pack(
            "!I", binascii.crc32(unknown_kind[:-4]) & 0xFFFFFFFF
        )
        with self.assertRaisesRegex(ValueError, "unknown"):
            decode_datagram(bytes(unknown_kind))

        with self.assertRaisesRegex(ValueError, "flags"):
            encode_datagram(
                UwbDatagram(
                    kind=PacketKind.RANGE_REQUEST,
                    sequence=1,
                    sim_time_ns=10,
                    source_id=0,
                    destination_id=1,
                    flags=1,
                )
            )
        with self.assertRaisesRegex(ValueError, "nonzero timestamp"):
            encode_datagram(
                UwbDatagram(
                    kind=PacketKind.RANGE_REQUEST,
                    sequence=1,
                    sim_time_ns=0,
                    source_id=0,
                    destination_id=1,
                )
            )

    def test_zero_range_and_invalid_broker_ids_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "nonzero"):
            encode_datagram(
                UwbDatagram(
                    kind=PacketKind.RANGE_RESPONSE,
                    sequence=1,
                    sim_time_ns=10,
                    source_id=1,
                    destination_id=0,
                    payload=RANGE_VECTOR.pack(0, 0, 0, 0),
                )
            )
        with self.assertRaisesRegex(ValueError, "node ID"):
            encode_datagram(
                UwbDatagram(
                    kind=PacketKind.REGISTER,
                    sequence=0,
                    sim_time_ns=0,
                    source_id=6,
                    destination_id=BROKER_ID,
                    payload=struct.pack("!H", 19958),
                )
            )


if __name__ == "__main__":
    unittest.main()
