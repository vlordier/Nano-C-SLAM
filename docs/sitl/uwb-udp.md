# CrazySim UWB UDP contract (draft v1)

This is the host boundary for the Linux `uwb_api_sitl.c` provider. The C
provider and broker core now exchange packets over UDP. The provider registers
against a live broker, applies per-flow replay/freshness checks, implements the
public UWB API and fails closed when simulator time, a peer, or a valid response
is missing. A two-process host test exercises ACK2 delivery, ranging and the
measurement queue through the C provider and Python broker core. The contract
is still draft until the same path runs against live Gazebo and CrazySim.

Every datagram is network byte order and has a 24-byte header, payload, and a
four-byte IEEE CRC-32 over header plus payload:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | ASCII `NCSU` |
| 4 | 1 | Protocol version, `1` |
| 5 | 1 | Packet kind |
| 6 | 1 | Flags, zero in v1 |
| 7 | 1 | Reserved, zero |
| 8 | 4 | Request/event sequence; responses echo the request sequence |
| 12 | 8 | Gazebo simulation timestamp in nanoseconds; zero only for registration request |
| 20 | 1 | Source Nano node ID, `0..5` |
| 21 | 1 | Destination Nano node ID, `0..5`, or `255` for broker |
| 22 | 2 | Payload length in bytes |
| 24 | variable | Packet-kind payload |
| 24 + payload length | 4 | IEEE CRC-32 |

Packet kinds are `1 REGISTER`, `2 REGISTER_ACK`, `3 RANGE_REQUEST`,
`4 RANGE_RESPONSE`, `5 MESSAGE_REQUEST`, `6 MESSAGE_DELIVERY`,
`7 MESSAGE_RESPONSE`, `8 MEASUREMENT`, `9 TIME_SYNC`, and `255 ERROR`.

`REGISTER` carries the node's local receive UDP port (`uint16`). The host
broker validates it as nonzero, then binds the node ID to the UDP source
endpoint it actually observed. This matters behind Lima NAT: the host can see a
loopback address and translated port, and replies to the guest's advertised
address would not arrive. Later packets must still come from that registered
endpoint. The broker returns `REGISTER_ACK` only after observing a finite,
fresh Gazebo pose for the node. Registration response timestamp and sequence
identify that pose snapshot. Firmware repeats the zero-time registration packet
as a bounded lease renewal; the broker refreshes the node lease without
replying to every renewal.

`TIME_SYNC` has an empty payload and routes broker-to-node. It is emitted
when Gazebo simulation time advances and carries the current clock in the
header timestamp. Firmware uses the latest sync as its freshness reference,
permits a bounded 50 ms future skew for a response racing the next sync, and
fails closed when the wall-clock sync lease expires. The broker uses one
monotonic event sequence per node for registration acknowledgements, clock
syncs, and errors.

`RANGE_REQUEST` has an empty payload and routes from requester to target peer.
`RANGE_RESPONSE` routes back from the peer to the requester and carries
`range_mm` (`uint32`) plus that peer's Gazebo-truth xyz position in millimetres
(`3 × int32`). `MEASUREMENT` is delivered to the peer and carries the same
range plus the requester's xyz position; its source ID is the requester. This
matches the broker's ranging exchange. The C API returns range in metres and
measurement position fields in millimetres because the existing
`uwb_ranging.c` caller divides `posx`/`posy` by 1000. All response/event
timestamps come from the exact Gazebo pose snapshots used for distance
calculation. Firmware rejects replayed sequence numbers, non-increasing
timestamps within a packet flow, and data older than the configured freshness
bound.

The C and Python codecs expose a per-flow freshness guard. Callers provide
current Gazebo simulation time and a nonzero maximum age. The C adapter keeps
independent state by packet kind and source/destination IDs. Replies echo the
originating request sequence for matching; sequence order is monotonic within
each such flow. Broker-originated events use an independent per-node sequence.

`MESSAGE_REQUEST`, `MESSAGE_DELIVERY`, and `MESSAGE_RESPONSE` payloads contain
the existing UWB API byte representation: `ctrl, src, dest, code, data[0..n)`.
The payload length is `4+n` and determines `n`; v1 limits `n` to
`MSG_DATA_LEN`, so the UWB message bytes are at most 118 bytes. The two-byte
radio FCS is not carried inside the UDP payload; when returning a message via
`uwb_send_msg_wait_rsp`, the C provider appends two zero padding bytes to the
local receive buffer so the existing `uwb_message_from_array` length
convention yields the same `data_len`. The broker routes messages only to a
currently registered live node. With responder and acknowledgements enabled,
the C provider returns the legacy ACK response containing the request's first
data byte, or ACK2 containing its data length, and exposes deliveries through
`get_msg_from_queue`.

Every packet kind has an exact payload size except the three application
message kinds, which must be between 4 and `4 + MSG_DATA_LEN` bytes with a matching
embedded node identity. Unknown kinds, flags, versions, IDs, lengths, CRCs,
stale snapshots, duplicate sequences, and unregistered destinations are
rejected. Broker absence, peer loss, malformed datagrams, and timed-out
requests never produce a successful range or message; malformed datagrams are
ignored and the bounded request wait returns `UWB_RX_TIMEOUT`. A broker may
not synthesize ACKs or token return messages for absent firmware peers in the
acceptance path.

`ERROR` carries one code byte: `1` unknown/unregistered peer, `2` missing,
stale, or skewed pose, `3` invalid or unrepresentable range request, and `4`
broker pending-message capacity exhausted. All errors originate at the broker
and are addressed to the requesting node.

Run the broker on the native Gazebo host with `uv run --locked python -m
scripts.uwb_sitl_broker --node-ids 0,1 --bind-address 0.0.0.0 --bind-port
19952`. For Gazebo Harmonic's `crazysim_default` world, add
`--clock-topic /world/crazysim_default/clock`; the broker default remains
`/clock` for worlds publishing an unscoped clock. The Linux Lima firmware defaults to the Lima host gateway
`192.168.5.2`; set `NCS_SITL_UWB_BROKER_HOST` explicitly when the guest uses a
different gateway. The firmware process must also receive the gateway as its
second positional argument (`cf2 <firmware-port> 192.168.5.2`) so CrazySim
publishes IMU packets to the host. The broker subscribes through
`gz topic -e --json-output` to the configured clock topic and
`/cf_<id>/odom`, then sends UWB datagrams to each registered firmware IP and
port. Host tests cover the broker state machine, local UDP registration/ranging
and two independent C firmware-provider processes exchanging ACK2 and a
timestamped range/measurement; a missing peer is also verified to return
`UWB_RX_TIMEOUT`; CRC-corrupted broker errors, stale peer poses, and a valid
range datagram addressed to the wrong node also leave the range at zero and
return timeout. They do not exercise the Gazebo `gz topic` subscribers,
CrazySim task scheduler, motor path, flight, or mapping.

The repository has Python and C framing/field validators plus four shared
golden packets (registration, range response, app-message request, and clock
sync) compiled and checked against the same exact bytes. Keep this contract in
draft until cross-process positive and fail-closed tests pass against the live
Gazebo broker and CrazySim firmware.
