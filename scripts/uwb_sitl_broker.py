"""Fail-closed routing and range core for the CrazySim Nano UWB broker."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import socket
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, cast

from scripts.uwb_sitl_protocol import (
    BROKER_ID,
    MAX_NODE_ID,
    RANGE_VECTOR,
    PacketKind,
    SequenceFreshnessGuard,
    UwbDatagram,
    decode_datagram,
    encode_datagram,
)

type Address = tuple[str, int]
type Outbound = tuple[Address, bytes]
LOGGER = logging.getLogger(__name__)


def _object(value: object) -> dict[str, object]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError("Gazebo JSON field must be an object")
    return cast(dict[str, object], value)


def _integer(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        raise ValueError("Gazebo timestamp fields must be integer text")
    return int(value)


def _number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError("Gazebo coordinates must be numeric")
    return float(value)


def _timestamp_ns(value: object) -> int:
    stamp = _object(value)
    try:
        seconds = _integer(stamp["sec"])
        # gz.msgs time fields omit zero-valued protobuf scalars in JSON mode.
        nanoseconds = _integer(stamp.get("nsec", "0"))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("Gazebo timestamp is malformed") from error
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("Gazebo timestamp is outside the valid range")
    return seconds * 1_000_000_000 + nanoseconds


def parse_gazebo_clock(payload: str | bytes) -> int:
    """Parse one gz.msgs.Clock JSON message and return simulation nanoseconds."""
    try:
        message = _object(json.loads(payload))
        return _timestamp_ns(message["sim"])
    except (json.JSONDecodeError, UnicodeDecodeError, KeyError, TypeError) as error:
        raise ValueError("Gazebo clock JSON is malformed") from error


def parse_gazebo_odometry(
    payload: str | bytes,
) -> tuple[int, tuple[float, float, float]]:
    """Parse one gz.msgs.Odometry sample without filling missing fields."""
    try:
        message = _object(json.loads(payload))
        header = _object(message["header"])
        pose = _object(message["pose"])
        position = _object(pose["position"])
        sim_time_ns = _timestamp_ns(header["stamp"])
        xyz = (
            _number(position["x"]),
            _number(position["y"]),
            _number(position["z"]),
        )
    except (
        json.JSONDecodeError,
        UnicodeDecodeError,
        KeyError,
        TypeError,
        ValueError,
    ) as error:
        raise ValueError("Gazebo odometry JSON is malformed") from error
    if any(not math.isfinite(value) for value in xyz):
        raise ValueError("Gazebo odometry coordinates must be finite")
    return sim_time_ns, xyz


class BrokerError(IntEnum):
    UNKNOWN_NODE = 1
    STALE_POSE = 2
    INVALID_REQUEST = 3
    BUSY = 4


@dataclass(frozen=True)
class PoseSample:
    sim_time_ns: int
    xyz_m: tuple[float, float, float]
    received_monotonic_ns: int

    @property
    def xyz_mm(self) -> tuple[int, int, int]:
        millimeters = tuple(round(value * 1000) for value in self.xyz_m)
        if any(not -(2**31) <= value < 2**31 for value in millimeters):
            raise ValueError("Gazebo position is outside int32 millimeter range")
        return (millimeters[0], millimeters[1], millimeters[2])


@dataclass(frozen=True)
class RegisteredNode:
    node_id: int
    address: Address
    last_seen_monotonic_ns: int


class UwbBrokerCore:
    """Pure broker state machine; the CLI owns Gazebo and UDP I/O."""

    def __init__(
        self,
        *,
        pose_max_age_ns: int = 250_000_000,
        pose_max_skew_ns: int = 50_000_000,
        node_lease_ns: int = 500_000_000,
        pending_lease_ns: int = 1_000_000_000,
        max_pending_messages: int = 64,
    ) -> None:
        if min(pose_max_age_ns, node_lease_ns, pending_lease_ns) <= 0:
            raise ValueError("broker leases must be positive")
        if pose_max_skew_ns < 0 or max_pending_messages <= 0:
            raise ValueError("pose skew must be nonnegative and pending bound positive")
        self.pose_max_age_ns = pose_max_age_ns
        self.pose_max_skew_ns = pose_max_skew_ns
        self.node_lease_ns = node_lease_ns
        self.pending_lease_ns = pending_lease_ns
        self.max_pending_messages = max_pending_messages
        self.clock_ns = 0
        self.clock_received_monotonic_ns = 0
        self._last_clock_sync_ns = 0
        self._nodes: dict[int, RegisteredNode] = {}
        self._poses: dict[int, PoseSample] = {}
        self._broker_sequences: dict[int, int] = {}
        self._guards: dict[tuple[PacketKind, int, int], SequenceFreshnessGuard] = {}
        self._pending_messages: dict[tuple[int, int, int], int] = {}

    def observe_clock(self, sim_time_ns: int, received_monotonic_ns: int) -> None:
        if sim_time_ns <= self.clock_ns:
            raise ValueError("Gazebo clock must advance monotonically")
        self.clock_ns = sim_time_ns
        self.clock_received_monotonic_ns = received_monotonic_ns

    def observe_pose(
        self,
        node_id: int,
        sim_time_ns: int,
        xyz_m: tuple[float, float, float],
        received_monotonic_ns: int,
    ) -> None:
        if not 0 <= node_id <= MAX_NODE_ID:
            raise ValueError("pose node ID is outside 0..5")
        if len(xyz_m) != 3 or any(not math.isfinite(value) for value in xyz_m):
            raise ValueError("Gazebo pose must have three finite coordinates")
        if sim_time_ns <= 0 or sim_time_ns - self.clock_ns > self.pose_max_skew_ns:
            raise ValueError("pose timestamp is outside the clock skew bound")
        previous = self._poses.get(node_id)
        if previous is not None and sim_time_ns <= previous.sim_time_ns:
            raise ValueError("Gazebo pose timestamp must increase")
        sample = PoseSample(sim_time_ns, xyz_m, received_monotonic_ns)
        _ = sample.xyz_mm
        self._poses[node_id] = sample

    def tick(self, now_monotonic_ns: int) -> list[Outbound]:
        self._expire_pending(now_monotonic_ns)
        if (
            self.clock_ns == 0
            or self.clock_ns <= self._last_clock_sync_ns
            or now_monotonic_ns - self.clock_received_monotonic_ns > self.node_lease_ns
        ):
            return []
        outputs: list[Outbound] = []
        for node_id, node in tuple(self._nodes.items()):
            if now_monotonic_ns - node.last_seen_monotonic_ns > self.node_lease_ns:
                del self._nodes[node_id]
                continue
            sequence = self._next_broker_sequence(node_id)
            packet = UwbDatagram(
                kind=PacketKind.TIME_SYNC,
                sequence=sequence,
                sim_time_ns=self.clock_ns,
                source_id=BROKER_ID,
                destination_id=node_id,
            )
            outputs.append((node.address, encode_datagram(packet)))
        self._last_clock_sync_ns = self.clock_ns
        return outputs

    def handle(
        self, payload: bytes, remote: Address, now_monotonic_ns: int
    ) -> list[Outbound]:
        try:
            packet = decode_datagram(payload)
        except ValueError:
            return []
        if packet.kind == PacketKind.REGISTER:
            return self._register(packet, remote, now_monotonic_ns)
        node = self._nodes.get(packet.source_id)
        if node is None or node.address != remote:
            return []
        if packet.kind not in {
            PacketKind.MESSAGE_REQUEST,
            PacketKind.MESSAGE_RESPONSE,
            PacketKind.RANGE_REQUEST,
        }:
            return []
        if packet.sim_time_ns > self.clock_ns:
            return []
        guard_key = (packet.kind, packet.source_id, packet.destination_id)
        guard = self._guards.setdefault(guard_key, SequenceFreshnessGuard())
        try:
            guard.accept(
                packet,
                now_sim_time_ns=self.clock_ns,
                max_age_ns=self.pose_max_age_ns,
            )
        except ValueError:
            return []
        self._nodes[packet.source_id] = RegisteredNode(
            packet.source_id, node.address, now_monotonic_ns
        )

        if packet.kind == PacketKind.MESSAGE_REQUEST:
            return self._route_message_request(packet, now_monotonic_ns)
        if packet.kind == PacketKind.MESSAGE_RESPONSE:
            return self._route_message_response(packet, now_monotonic_ns)
        if packet.kind == PacketKind.RANGE_REQUEST:
            return self._range(packet, now_monotonic_ns)
        return []

    def _register(
        self, packet: UwbDatagram, remote: Address, now_monotonic_ns: int
    ) -> list[Outbound]:
        node_id = packet.source_id
        advertised_port = struct.unpack("!H", packet.payload)[0]
        if advertised_port == 0:
            return []
        # Use the observed UDP endpoint so replies traverse guest NAT mappings.
        # On a direct network it is the same IP/port the node advertised.
        endpoint = remote
        existing = self._nodes.get(node_id)
        if existing is not None and (
            now_monotonic_ns - existing.last_seen_monotonic_ns <= self.node_lease_ns
        ):
            if existing.address != endpoint:
                return []
            self._nodes[node_id] = RegisteredNode(
                node_id, existing.address, now_monotonic_ns
            )
            return []
        if existing is not None:
            self._guards = {
                key: guard
                for key, guard in self._guards.items()
                if node_id not in key[1:]
            }
            self._pending_messages = {
                key: sent_at
                for key, sent_at in self._pending_messages.items()
                if node_id not in key[:2]
            }
        if not self._pose_is_fresh(node_id, now_monotonic_ns):
            return [self._error(node_id, *endpoint, BrokerError.STALE_POSE)]
        address = endpoint
        self._nodes[node_id] = RegisteredNode(node_id, address, now_monotonic_ns)
        pose = self._poses[node_id]
        ack = UwbDatagram(
            kind=PacketKind.REGISTER_ACK,
            sequence=self._next_broker_sequence(node_id),
            sim_time_ns=pose.sim_time_ns,
            source_id=BROKER_ID,
            destination_id=node_id,
        )
        return [(address, encode_datagram(ack))]

    def _route_message_request(
        self, packet: UwbDatagram, now_monotonic_ns: int
    ) -> list[Outbound]:
        peer = self._live_node(packet.destination_id, now_monotonic_ns)
        requester = self._nodes[packet.source_id]
        if peer is None:
            return [
                self._error(
                    packet.source_id,
                    requester.address[0],
                    requester.address[1],
                    BrokerError.UNKNOWN_NODE,
                )
            ]
        key = (packet.source_id, packet.destination_id, packet.sequence)
        self._expire_pending(now_monotonic_ns)
        if key in self._pending_messages:
            return []
        if len(self._pending_messages) >= self.max_pending_messages:
            return [
                self._error(
                    packet.source_id,
                    requester.address[0],
                    requester.address[1],
                    BrokerError.BUSY,
                )
            ]
        self._pending_messages[key] = now_monotonic_ns
        delivery = UwbDatagram(
            kind=PacketKind.MESSAGE_DELIVERY,
            sequence=packet.sequence,
            sim_time_ns=packet.sim_time_ns,
            source_id=packet.source_id,
            destination_id=packet.destination_id,
            payload=packet.payload,
        )
        return [(peer.address, encode_datagram(delivery))]

    def _route_message_response(
        self, packet: UwbDatagram, now_monotonic_ns: int
    ) -> list[Outbound]:
        key = (packet.destination_id, packet.source_id, packet.sequence)
        sent_at = self._pending_messages.pop(key, None)
        source_node = self._live_node(packet.source_id, now_monotonic_ns)
        destination_node = self._live_node(packet.destination_id, now_monotonic_ns)
        if (
            sent_at is None
            or source_node is None
            or destination_node is None
            or now_monotonic_ns - sent_at > self.pending_lease_ns
        ):
            return []
        return [(destination_node.address, encode_datagram(packet))]

    def _range(self, packet: UwbDatagram, now_monotonic_ns: int) -> list[Outbound]:
        requester = self._live_node(packet.source_id, now_monotonic_ns)
        peer = self._live_node(packet.destination_id, now_monotonic_ns)
        if requester is None:
            return []
        if peer is None:
            return [
                self._error(
                    packet.source_id,
                    requester.address[0],
                    requester.address[1],
                    BrokerError.UNKNOWN_NODE,
                )
            ]
        requester_pose = self._poses.get(packet.source_id)
        peer_pose = self._poses.get(packet.destination_id)
        if (
            requester_pose is None
            or peer_pose is None
            or not self._pose_is_fresh(packet.source_id, now_monotonic_ns)
            or not self._pose_is_fresh(packet.destination_id, now_monotonic_ns)
            or abs(requester_pose.sim_time_ns - peer_pose.sim_time_ns)
            > self.pose_max_skew_ns
        ):
            return [
                self._error(
                    packet.source_id,
                    requester.address[0],
                    requester.address[1],
                    BrokerError.STALE_POSE,
                )
            ]
        distance_m = math.dist(requester_pose.xyz_m, peer_pose.xyz_m)
        range_mm = round(distance_m * 1000)
        if not 0 < range_mm <= 0xFFFFFFFF:
            return [
                self._error(
                    packet.source_id,
                    requester.address[0],
                    requester.address[1],
                    BrokerError.INVALID_REQUEST,
                )
            ]
        response = UwbDatagram(
            kind=PacketKind.RANGE_RESPONSE,
            sequence=packet.sequence,
            sim_time_ns=peer_pose.sim_time_ns,
            source_id=packet.destination_id,
            destination_id=packet.source_id,
            payload=RANGE_VECTOR.pack(range_mm, *peer_pose.xyz_mm),
        )
        measurement = UwbDatagram(
            kind=PacketKind.MEASUREMENT,
            sequence=packet.sequence,
            sim_time_ns=requester_pose.sim_time_ns,
            source_id=packet.source_id,
            destination_id=packet.destination_id,
            payload=RANGE_VECTOR.pack(range_mm, *requester_pose.xyz_mm),
        )
        return [
            (requester.address, encode_datagram(response)),
            (peer.address, encode_datagram(measurement)),
        ]

    def _error(self, node_id: int, host: str, port: int, code: BrokerError) -> Outbound:
        packet = UwbDatagram(
            kind=PacketKind.ERROR,
            sequence=self._next_broker_sequence(node_id),
            sim_time_ns=self.clock_ns,
            source_id=BROKER_ID,
            destination_id=node_id,
            payload=bytes((int(code),)),
        )
        return (host, port), encode_datagram(packet)

    def _pose_is_fresh(self, node_id: int, now_monotonic_ns: int) -> bool:
        pose = self._poses.get(node_id)
        if pose is None:
            return False
        clock_delta_ns = self.clock_ns - pose.sim_time_ns
        return bool(
            (
                0 <= clock_delta_ns <= self.pose_max_age_ns
                or -self.pose_max_skew_ns <= clock_delta_ns < 0
            )
            and 0 <= now_monotonic_ns - pose.received_monotonic_ns <= self.node_lease_ns
        )

    def _live_node(self, node_id: int, now_monotonic_ns: int) -> RegisteredNode | None:
        node = self._nodes.get(node_id)
        if (
            node is None
            or now_monotonic_ns - node.last_seen_monotonic_ns > self.node_lease_ns
        ):
            return None
        return node

    def _next_broker_sequence(self, node_id: int) -> int:
        sequence = self._broker_sequences.get(node_id, 0)
        if sequence >= 0xFFFFFFFF:
            raise OverflowError("broker sequence exhausted; restart session")
        self._broker_sequences[node_id] = sequence + 1
        return sequence

    def _expire_pending(self, now_monotonic_ns: int) -> None:
        expired = [
            key
            for key, sent_at in self._pending_messages.items()
            if now_monotonic_ns - sent_at > self.pending_lease_ns
        ]
        for key in expired:
            del self._pending_messages[key]


class _BrokerDatagramProtocol(asyncio.DatagramProtocol):
    def __init__(self, core: UwbBrokerCore) -> None:
        self.core = core
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        if not isinstance(transport, asyncio.DatagramTransport):
            raise TypeError("broker requires a UDP datagram transport")
        self.transport = transport

    def datagram_received(self, data: bytes, addr: tuple[Any, ...]) -> None:
        if self.transport is None:
            return
        if (
            len(addr) != 2
            or not isinstance(addr[0], str)
            or not isinstance(addr[1], int)
        ):
            return
        address = (addr[0], addr[1])
        try:
            packet = decode_datagram(data)
        except ValueError:
            return
        outbound = self.core.handle(data, address, time.monotonic_ns())
        if packet.kind == PacketKind.REGISTER and outbound:
            response = decode_datagram(outbound[0][1])
            if response.kind == PacketKind.REGISTER_ACK:
                LOGGER.info(
                    "registered UWB node=%d endpoint=%s:%d pose_sim_ns=%d",
                    packet.source_id,
                    *outbound[0][0],
                    response.sim_time_ns,
                )
            elif response.kind == PacketKind.ERROR:
                LOGGER.warning(
                    "rejected UWB registration node=%d endpoint=%s:%d error=%d",
                    packet.source_id,
                    *outbound[0][0],
                    response.payload[0],
                )
        self.send(outbound)

    def send(self, outbound: list[Outbound]) -> None:
        if self.transport is None:
            return
        for address, payload in outbound:
            self.transport.sendto(payload, address)


class CrazySimUwbBroker:
    def __init__(
        self,
        *,
        gz_command: str,
        node_ids: tuple[int, ...],
        topic_prefix: str,
        clock_topic: str,
        bind_address: str,
        bind_port: int,
    ) -> None:
        if not node_ids or len(set(node_ids)) != len(node_ids):
            raise ValueError("at least one distinct node ID is required")
        if any(not 0 <= node_id <= MAX_NODE_ID for node_id in node_ids):
            raise ValueError("node IDs must be in 0..5")
        if not topic_prefix.startswith("/") or topic_prefix.endswith("/"):
            raise ValueError("topic prefix must be absolute without trailing slash")
        if not clock_topic.startswith("/") or clock_topic.endswith("/"):
            raise ValueError("clock topic must be absolute without trailing slash")
        if not 1 <= bind_port <= 65535:
            raise ValueError("UDP bind port must be in 1..65535")
        self.gz_command = gz_command
        self.node_ids = node_ids
        self.topic_prefix = topic_prefix
        self.clock_topic = clock_topic
        self.bind_address = bind_address
        self.bind_port = bind_port
        self.core = UwbBrokerCore()
        self.protocol: _BrokerDatagramProtocol | None = None

    async def _consume_topic(
        self, topic: str, handler: Callable[[bytes, int], None]
    ) -> None:
        process = await asyncio.create_subprocess_exec(
            self.gz_command,
            "topic",
            "-e",
            "--json-output",
            "-t",
            topic,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        if process.stdout is None:
            raise RuntimeError(f"gz topic has no output stream for {topic}")
        try:
            async for line in process.stdout:
                try:
                    handler(line, time.monotonic_ns())
                except ValueError as error:
                    LOGGER.warning("discarding invalid %s sample: %s", topic, error)
            return_code = await process.wait()
            raise RuntimeError(f"gz topic exited for {topic} ({return_code})")
        finally:
            if process.returncode is None:
                process.terminate()
                try:
                    await asyncio.wait_for(process.wait(), timeout=2.0)
                except TimeoutError:
                    process.kill()
                    await process.wait()

    def _clock_sample(self, payload: bytes, received_ns: int) -> None:
        sim_time_ns = parse_gazebo_clock(payload)
        self.core.observe_clock(sim_time_ns, received_ns)

    def _pose_sample(self, node_id: int) -> Callable[[bytes, int], None]:
        def accept(payload: bytes, received_ns: int) -> None:
            sim_time_ns, xyz_m = parse_gazebo_odometry(payload)
            self.core.observe_pose(node_id, sim_time_ns, xyz_m, received_ns)

        return accept

    async def _clock_sync_loop(self) -> None:
        while True:
            await asyncio.sleep(0.02)
            if self.protocol is not None:
                self.protocol.send(self.core.tick(time.monotonic_ns()))

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: _BrokerDatagramProtocol(self.core),
            local_addr=(self.bind_address, self.bind_port),
            family=socket.AF_INET,
        )
        self.protocol = protocol  # type: ignore[assignment]
        topics = [
            asyncio.create_task(
                self._consume_topic(self.clock_topic, self._clock_sample)
            ),
            *(
                asyncio.create_task(
                    self._consume_topic(
                        f"{self.topic_prefix}_{node_id}/odom",
                        self._pose_sample(node_id),
                    )
                )
                for node_id in self.node_ids
            ),
            asyncio.create_task(self._clock_sync_loop()),
        ]
        LOGGER.info(
            "UWB broker listening on %s:%d; clock=%s; odometry=%s",
            self.bind_address,
            self.bind_port,
            self.clock_topic,
            ",".join(
                f"{self.topic_prefix}_{node_id}/odom" for node_id in self.node_ids
            ),
        )
        try:
            await asyncio.gather(*topics)
        finally:
            for task in topics:
                task.cancel()
            await asyncio.gather(*topics, return_exceptions=True)
            transport.close()


def _parse_node_ids(raw_value: str) -> tuple[int, ...]:
    try:
        node_ids = tuple(int(value) for value in raw_value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "node IDs must be comma-separated integers"
        ) from error
    if (
        not node_ids
        or len(set(node_ids)) != len(node_ids)
        or any(not 0 <= node_id <= MAX_NODE_ID for node_id in node_ids)
    ):
        raise argparse.ArgumentTypeError("node IDs must be distinct values in 0..5")
    return node_ids


async def _run_cli(arguments: argparse.Namespace) -> None:
    broker = CrazySimUwbBroker(
        gz_command=arguments.gz_command,
        node_ids=arguments.node_ids,
        topic_prefix=arguments.topic_prefix,
        clock_topic=arguments.clock_topic,
        bind_address=arguments.bind_address,
        bind_port=arguments.bind_port,
    )
    await broker.run()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gz-command", default="gz")
    parser.add_argument("--node-ids", type=_parse_node_ids, default=(0, 1))
    parser.add_argument("--topic-prefix", default="/cf")
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument("--bind-address", default="0.0.0.0")  # noqa: S104
    parser.add_argument("--bind-port", type=int, default=19952)
    parser.add_argument("--verbose", action="store_true")
    arguments = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO if arguments.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    try:
        asyncio.run(_run_cli(arguments))
    except KeyboardInterrupt:
        LOGGER.info("UWB broker stopped")


if __name__ == "__main__":
    main()
