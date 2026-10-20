"""Adversarial tests for timestamped CrazySim UWB broker state transitions."""

from __future__ import annotations

import asyncio
import json
import socket
import struct
import time
import unittest
from collections.abc import Callable
from unittest.mock import patch

from scripts.uwb_sitl_broker import (
    BrokerError,
    CrazySimUwbBroker,
    UwbBrokerCore,
    _BrokerDatagramProtocol,
    parse_gazebo_clock,
    parse_gazebo_odometry,
)
from scripts.uwb_sitl_protocol import (
    BROKER_ID,
    RANGE_VECTOR,
    PacketKind,
    UwbDatagram,
    decode_datagram,
    encode_datagram,
)

NOW = 10_000_000_000
NODE0 = ("192.0.2.10", 19953)
NODE1 = ("192.0.2.11", 19954)


def _register(node_id: int, port: int) -> bytes:
    return encode_datagram(
        UwbDatagram(
            PacketKind.REGISTER,
            0,
            0,
            node_id,
            BROKER_ID,
            struct.pack("!H", port),
        )
    )


class UwbSitlBrokerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.core = UwbBrokerCore()
        self.core.observe_clock(NOW, 100)
        self.core.observe_pose(0, NOW - 2_000_000, (0.0, 0.0, 1.0), 110)
        self.core.observe_pose(1, NOW - 1_000_000, (1.234, 0.0, 1.0), 120)
        ack0 = self.core.handle(_register(0, NODE0[1]), NODE0, 130)
        ack1 = self.core.handle(_register(1, NODE1[1]), NODE1, 140)
        self.assertEqual(len(ack0), 1)
        self.assertEqual(len(ack1), 1)
        self.assertEqual(decode_datagram(ack0[0][1]).kind, PacketKind.REGISTER_ACK)

    def test_clock_sync_and_range_response_use_real_peer_pose(self) -> None:
        sync = self.core.tick(150)
        self.assertEqual(len(sync), 2)
        self.assertTrue(
            all(
                decode_datagram(payload).kind == PacketKind.TIME_SYNC
                for _, payload in sync
            )
        )

        request = UwbDatagram(PacketKind.RANGE_REQUEST, 1, NOW, 0, 1)
        replies = self.core.handle(encode_datagram(request), NODE0, 160)
        self.assertEqual(len(replies), 2)
        response = decode_datagram(replies[0][1])
        measurement = decode_datagram(replies[1][1])
        self.assertEqual(replies[0][0], NODE0)
        self.assertEqual(replies[1][0], NODE1)
        self.assertEqual(response.kind, PacketKind.RANGE_RESPONSE)
        self.assertEqual((response.source_id, response.destination_id), (1, 0))
        self.assertEqual(response.sim_time_ns, NOW - 1_000_000)
        self.assertEqual(RANGE_VECTOR.unpack(response.payload), (1234, 1234, 0, 1000))
        self.assertEqual(measurement.kind, PacketKind.MEASUREMENT)
        self.assertEqual((measurement.source_id, measurement.destination_id), (0, 1))
        self.assertEqual(RANGE_VECTOR.unpack(measurement.payload), (1234, 0, 0, 1000))

    def test_clock_rollback_starts_a_clean_gazebo_session(self) -> None:
        message = bytes((0xDE, 0, 1, 7, 0x41))
        request = UwbDatagram(PacketKind.MESSAGE_REQUEST, 7, NOW, 0, 1, message)
        self.assertEqual(len(self.core.handle(encode_datagram(request), NODE0, 150)), 1)
        self.assertTrue(self.core._guards)
        self.assertTrue(self.core._pending_messages)

        self.core.observe_clock(1_000_000_000, 200)

        self.assertEqual(self.core.clock_ns, 1_000_000_000)
        self.assertEqual(self.core.clock_received_monotonic_ns, 200)
        self.assertEqual(self.core._last_clock_sync_ns, 0)
        self.assertFalse(self.core._nodes)
        self.assertFalse(self.core._poses)
        self.assertFalse(self.core._broker_sequences)
        self.assertFalse(self.core._guards)
        self.assertFalse(self.core._pending_messages)

        self.core.observe_pose(0, 999_000_000, (0.0, 0.0, 0.5), 210)
        acknowledgement = self.core.handle(_register(0, NODE0[1]), NODE0, 220)
        self.assertEqual(
            decode_datagram(acknowledgement[0][1]).kind,
            PacketKind.REGISTER_ACK,
        )

    def test_repeated_nonzero_clock_sample_is_still_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "advance monotonically"):
            self.core.observe_clock(NOW, 200)

    def test_absent_peer_and_stale_pose_never_produce_a_range(self) -> None:
        unknown_request = UwbDatagram(PacketKind.RANGE_REQUEST, 2, NOW, 0, 2)
        unknown_reply = self.core.handle(encode_datagram(unknown_request), NODE0, 200)
        self.assertEqual(len(unknown_reply), 1)
        error = decode_datagram(unknown_reply[0][1])
        self.assertEqual(error.kind, PacketKind.ERROR)
        self.assertEqual(error.payload, bytes((BrokerError.UNKNOWN_NODE,)))

        self.core.observe_clock(NOW + 400_000_000, 300)
        stale_request = UwbDatagram(
            PacketKind.RANGE_REQUEST, 3, NOW + 400_000_000, 0, 1
        )
        stale_reply = self.core.handle(encode_datagram(stale_request), NODE0, 310)
        self.assertEqual(len(stale_reply), 1)
        stale_error = decode_datagram(stale_reply[0][1])
        self.assertEqual(stale_error.kind, PacketKind.ERROR)
        self.assertEqual(stale_error.payload, bytes((BrokerError.STALE_POSE,)))

    def test_pose_skew_and_source_ip_mismatch_fail_closed(self) -> None:
        skew_core = UwbBrokerCore()
        skew_core.observe_clock(NOW, 100)
        skew_core.observe_pose(0, NOW - 80_000_000, (0.0, 0.0, 1.0), 110)
        skew_core.observe_pose(1, NOW - 1_000_000, (1.234, 0.0, 1.0), 120)
        skew_core.handle(_register(0, NODE0[1]), NODE0, 130)
        skew_core.handle(_register(1, NODE1[1]), NODE1, 140)
        request = UwbDatagram(PacketKind.RANGE_REQUEST, 5, NOW, 0, 1)
        skewed = skew_core.handle(encode_datagram(request), NODE0, 220)
        self.assertEqual(
            decode_datagram(skewed[0][1]).payload, bytes((BrokerError.STALE_POSE,))
        )

        # A packet from a different source port is not accepted as the registered node.
        forged = skew_core.handle(encode_datagram(request), (NODE0[0], 50000), 230)
        self.assertEqual(forged, [])

    def test_message_delivery_requires_registered_live_peer_and_matching_reply(
        self,
    ) -> None:
        message = bytes((0xDE, 0, 1, 7, 0x41))
        request = UwbDatagram(PacketKind.MESSAGE_REQUEST, 7, NOW, 0, 1, message)
        delivery = self.core.handle(encode_datagram(request), NODE0, 250)
        self.assertEqual(len(delivery), 1)
        delivered = decode_datagram(delivery[0][1])
        self.assertEqual(delivery[0][0], NODE1)
        self.assertEqual(delivered.kind, PacketKind.MESSAGE_DELIVERY)
        self.assertEqual(delivered.payload, message)

        response_message = bytes((0xDE, 1, 0, 8, 0x42))
        response = UwbDatagram(
            PacketKind.MESSAGE_RESPONSE, 7, NOW, 1, 0, response_message
        )
        routed = self.core.handle(encode_datagram(response), NODE1, 260)
        self.assertEqual(routed, [(NODE0, encode_datagram(response))])
        self.assertEqual(self.core.handle(encode_datagram(response), NODE1, 270), [])

    def test_bad_crc_and_unregistered_endpoint_are_ignored(self) -> None:
        bad_crc = bytearray(_register(0, NODE0[1]))
        bad_crc[-1] ^= 1
        self.assertEqual(self.core.handle(bytes(bad_crc), NODE0, 300), [])
        wrong_endpoint = self.core.handle(_register(2, 19955), NODE0, 310)
        self.assertEqual(len(wrong_endpoint), 1)
        self.assertEqual(
            decode_datagram(wrong_endpoint[0][1]).payload,
            bytes((BrokerError.STALE_POSE,)),
        )

    def test_gazebo_clock_and_odometry_json_require_stamps_and_finite_pose(
        self,
    ) -> None:
        self.assertEqual(
            parse_gazebo_clock('{"sim":{"sec":"3","nsec":"12"}}'),
            3_000_000_012,
        )
        self.assertEqual(parse_gazebo_clock('{"sim":{"sec":"3"}}'), 3_000_000_000)
        odometry = {
            "header": {"stamp": {"sec": "4", "nsec": "15"}},
            "pose": {"position": {"x": 1.25, "y": -2, "z": 0.5}},
        }
        self.assertEqual(
            parse_gazebo_odometry(json.dumps(odometry)),
            (4_000_000_015, (1.25, -2.0, 0.5)),
        )
        with self.assertRaisesRegex(ValueError, "odometry JSON"):
            parse_gazebo_odometry('{"pose":{"position":{"x":0}}}')
        with self.assertRaisesRegex(ValueError, "finite"):
            parse_gazebo_odometry(
                '{"header":{"stamp":{"sec":"4","nsec":"15"}},'
                '"pose":{"position":{"x":"NaN","y":0,"z":0}}}'
            )

    def test_pose_timestamp_may_lead_clock_within_bounded_topic_skew(self) -> None:
        core = UwbBrokerCore(pose_max_skew_ns=50_000_000)
        core.observe_clock(NOW, 100)
        core.observe_pose(0, NOW + 1_000_000, (0.0, 0.0, 0.5), 110)
        acknowledgement = core.handle(_register(0, NODE0[1]), NODE0, 120)
        self.assertEqual(len(acknowledgement), 1)
        self.assertEqual(
            decode_datagram(acknowledgement[0][1]).kind,
            PacketKind.REGISTER_ACK,
        )
        # Lima's UDP proxy can rewrite the source port. Replies must target the
        # observed endpoint so the reverse datagram follows that NAT mapping.
        nat_core = UwbBrokerCore()
        nat_core.observe_clock(NOW, 100)
        nat_core.observe_pose(0, NOW - 1_000_000, (0.0, 0.0, 1.0), 110)
        nat_endpoint = ("127.0.0.1", 49152)
        nat_acknowledgement = nat_core.handle(_register(0, 21953), nat_endpoint, 130)
        self.assertEqual(nat_acknowledgement[0][0], nat_endpoint)
        with self.assertRaisesRegex(ValueError, "clock skew bound"):
            core.observe_pose(1, NOW + 60_000_000, (1.0, 0.0, 0.5), 130)


class UwbSitlBrokerUdpTests(unittest.IsolatedAsyncioTestCase):
    async def test_broker_subscribes_to_configured_gazebo_clock_topic(self) -> None:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        bind_port = probe.getsockname()[1]
        probe.close()
        broker = CrazySimUwbBroker(
            gz_command="gz",
            node_ids=(0, 1),
            topic_prefix="/cf",
            clock_topic="/world/crazysim_default/clock",
            bind_address="127.0.0.1",
            bind_port=bind_port,
        )
        subscribed_topics: list[str] = []
        clock_topic_seen = asyncio.Event()

        async def record_subscription(
            topic: str, _handler: Callable[[bytes, int], None]
        ) -> None:
            subscribed_topics.append(topic)
            if topic == broker.clock_topic:
                clock_topic_seen.set()
            await asyncio.Event().wait()

        with patch.object(broker, "_consume_topic", side_effect=record_subscription):
            running = asyncio.create_task(broker.run())
            try:
                await asyncio.wait_for(clock_topic_seen.wait(), timeout=1.0)
                self.assertIn("/world/crazysim_default/clock", subscribed_topics)
            finally:
                running.cancel()
                with self.assertRaises(asyncio.CancelledError):
                    await running

    async def test_registration_and_range_exchange_cross_real_udp_sockets(self) -> None:
        core = UwbBrokerCore()
        wall_now_ns = time.monotonic_ns()
        core.observe_clock(NOW, wall_now_ns)
        core.observe_pose(0, NOW - 2_000_000, (0.0, 0.0, 1.0), wall_now_ns)
        core.observe_pose(1, NOW - 1_000_000, (1.234, 0.0, 1.0), wall_now_ns)
        loop = asyncio.get_running_loop()
        transport, _protocol = await loop.create_datagram_endpoint(
            lambda: _BrokerDatagramProtocol(core),
            local_addr=("127.0.0.1", 0),
        )
        server = transport.get_extra_info("sockname")
        clients: list[socket.socket] = []
        try:
            for node_id in (0, 1):
                client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                client.setblocking(False)
                client.bind(("127.0.0.1", 0))
                clients.append(client)
                port = client.getsockname()[1]
                await loop.sock_sendto(
                    client,
                    _register(node_id, port),
                    ("127.0.0.1", server[1]),
                )
                ack, _ = await asyncio.wait_for(
                    loop.sock_recvfrom(client, 256), timeout=1.0
                )
                self.assertEqual(decode_datagram(ack).kind, PacketKind.REGISTER_ACK)

            request = UwbDatagram(PacketKind.RANGE_REQUEST, 1, NOW, 0, 1)
            await loop.sock_sendto(
                clients[0],
                encode_datagram(request),
                ("127.0.0.1", server[1]),
            )
            response, _ = await asyncio.wait_for(
                loop.sock_recvfrom(clients[0], 256), timeout=1.0
            )
            measurement, _ = await asyncio.wait_for(
                loop.sock_recvfrom(clients[1], 256), timeout=1.0
            )
            self.assertEqual(decode_datagram(response).kind, PacketKind.RANGE_RESPONSE)
            self.assertEqual(decode_datagram(measurement).kind, PacketKind.MEASUREMENT)
        finally:
            for client in clients:
                client.close()
            transport.close()


if __name__ == "__main__":
    unittest.main()
