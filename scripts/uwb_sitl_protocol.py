"""Version-one datagram framing for the CrazySim Nano UWB broker."""

from __future__ import annotations

import binascii
import struct
from dataclasses import dataclass
from enum import IntEnum

MAGIC = b"NCSU"
VERSION = 1
MAX_NODE_ID = 5
BROKER_ID = 255
MAX_UWB_DATA_BYTES = 114
MAX_UWB_MESSAGE_BYTES = 4 + MAX_UWB_DATA_BYTES
HEADER = struct.Struct("!4sBBBBIQBBH")
CRC = struct.Struct("!I")
RANGE_VECTOR = struct.Struct("!Iiii")


class PacketKind(IntEnum):
    REGISTER = 1
    REGISTER_ACK = 2
    RANGE_REQUEST = 3
    RANGE_RESPONSE = 4
    MESSAGE_REQUEST = 5
    MESSAGE_DELIVERY = 6
    MESSAGE_RESPONSE = 7
    MEASUREMENT = 8
    ERROR = 255


@dataclass(frozen=True)
class UwbDatagram:
    kind: PacketKind
    sequence: int
    sim_time_ns: int
    source_id: int
    destination_id: int
    payload: bytes = b""
    flags: int = 0


def _validate_node_id(node_id: int, *, allow_broker: bool) -> None:
    if not (0 <= node_id <= MAX_NODE_ID or (allow_broker and node_id == BROKER_ID)):
        raise ValueError("node ID is outside the protocol range")


def _validate_packet(packet: UwbDatagram) -> None:
    if not 0 <= packet.sequence <= 0xFFFFFFFF:
        raise ValueError("sequence must fit uint32")
    if not 0 <= packet.sim_time_ns <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("simulation time must fit uint64")
    if packet.flags != 0:
        raise ValueError("v1 flags must be zero")
    _validate_node_id(packet.source_id, allow_broker=True)
    _validate_node_id(packet.destination_id, allow_broker=True)

    payload_length = len(packet.payload)
    exact_sizes = {
        PacketKind.REGISTER: 2,
        PacketKind.REGISTER_ACK: 0,
        PacketKind.RANGE_REQUEST: 0,
        PacketKind.RANGE_RESPONSE: RANGE_VECTOR.size,
        PacketKind.MEASUREMENT: RANGE_VECTOR.size,
        PacketKind.ERROR: 1,
    }
    expected = exact_sizes.get(packet.kind)
    if expected is not None and payload_length != expected:
        raise ValueError(f"{packet.kind.name} payload must be {expected} bytes")
    if packet.kind in {
        PacketKind.MESSAGE_REQUEST,
        PacketKind.MESSAGE_DELIVERY,
        PacketKind.MESSAGE_RESPONSE,
    }:
        if not 4 <= payload_length <= MAX_UWB_MESSAGE_BYTES:
            raise ValueError("UWB message payload length is outside the API bound")
        if packet.payload[0] != 0xDE:
            raise ValueError("UWB message control byte must be 0xDE")
        if packet.payload[1] != packet.source_id:
            raise ValueError("embedded UWB source ID does not match packet header")
        if packet.payload[2] != packet.destination_id:
            raise ValueError("embedded UWB destination ID does not match packet header")
    if packet.kind == PacketKind.REGISTER:
        if packet.source_id == BROKER_ID or packet.destination_id != BROKER_ID:
            raise ValueError("REGISTER route must be node-to-broker")
    elif packet.kind == PacketKind.REGISTER_ACK:
        if packet.source_id != BROKER_ID or packet.destination_id == BROKER_ID:
            raise ValueError("REGISTER_ACK route must be broker-to-node")
    elif packet.kind in {
        PacketKind.RANGE_REQUEST,
        PacketKind.RANGE_RESPONSE,
        PacketKind.MESSAGE_REQUEST,
        PacketKind.MESSAGE_DELIVERY,
        PacketKind.MESSAGE_RESPONSE,
        PacketKind.MEASUREMENT,
    }:
        if packet.source_id == BROKER_ID or packet.destination_id == BROKER_ID:
            raise ValueError(f"{packet.kind.name} route must be node-to-node")
        if packet.source_id == packet.destination_id:
            raise ValueError(f"{packet.kind.name} source and destination must differ")
    if packet.kind in {
        PacketKind.MESSAGE_REQUEST,
        PacketKind.MESSAGE_DELIVERY,
        PacketKind.MESSAGE_RESPONSE,
    }:
        message_source = packet.payload[1]
        message_destination = packet.payload[2]
        if (message_source, message_destination) != (
            packet.source_id,
            packet.destination_id,
        ):
            raise ValueError("embedded UWB IDs must match packet route")
    if packet.kind == PacketKind.ERROR and (
        packet.source_id != BROKER_ID or packet.destination_id == BROKER_ID
    ):
        raise ValueError("ERROR route must be broker-to-node")
    if packet.kind == PacketKind.REGISTER and packet.sim_time_ns != 0:
        raise ValueError("REGISTER request timestamp must be zero")
    if (
        packet.kind == PacketKind.REGISTER
        and struct.unpack("!H", packet.payload)[0] == 0
    ):
        raise ValueError("REGISTER receive port must be nonzero")
    if packet.kind in {PacketKind.RANGE_RESPONSE, PacketKind.MEASUREMENT}:
        range_mm = RANGE_VECTOR.unpack(packet.payload)[0]
        if range_mm == 0:
            raise ValueError("successful UWB range must be nonzero")
    if packet.kind not in {PacketKind.REGISTER, PacketKind.ERROR} and (
        packet.sim_time_ns == 0
    ):
        raise ValueError("simulator-originated packets require a nonzero timestamp")


def encode_datagram(packet: UwbDatagram) -> bytes:
    _validate_packet(packet)
    header = HEADER.pack(
        MAGIC,
        VERSION,
        int(packet.kind),
        packet.flags,
        0,
        packet.sequence,
        packet.sim_time_ns,
        packet.source_id,
        packet.destination_id,
        len(packet.payload),
    )
    body = header + packet.payload
    return body + CRC.pack(binascii.crc32(body) & 0xFFFFFFFF)


def decode_datagram(data: bytes) -> UwbDatagram:
    if len(data) < HEADER.size + CRC.size:
        raise ValueError("datagram length is shorter than the header and CRC")
    (
        magic,
        version,
        raw_kind,
        flags,
        reserved,
        sequence,
        sim_time_ns,
        source_id,
        destination_id,
        payload_length,
    ) = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError("datagram magic is invalid")
    if version != VERSION:
        raise ValueError("unsupported UWB datagram version")
    if reserved != 0:
        raise ValueError("reserved header byte must be zero")
    try:
        kind = PacketKind(raw_kind)
    except ValueError as error:
        raise ValueError("unknown UWB packet kind") from error
    expected_length = HEADER.size + payload_length + CRC.size
    if len(data) != expected_length:
        raise ValueError("datagram length does not match payload length")
    expected_crc = CRC.unpack_from(data, len(data) - CRC.size)[0]
    if expected_crc != binascii.crc32(data[: -CRC.size]) & 0xFFFFFFFF:
        raise ValueError("datagram CRC-32 mismatch")
    packet = UwbDatagram(
        kind=kind,
        sequence=sequence,
        sim_time_ns=sim_time_ns,
        source_id=source_id,
        destination_id=destination_id,
        payload=data[HEADER.size : -CRC.size],
        flags=flags,
    )
    _validate_packet(packet)
    return packet
