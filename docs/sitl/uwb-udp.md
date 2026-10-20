# CrazySim UWB UDP contract (draft v1)

This contract is the planned host boundary for the Linux `uwb_api_sitl.c`
provider. It is not implemented end to end yet. The provider must remain
fail-closed until a broker is listening and a live Gazebo pose snapshot exists.
The broker will use timestamped Gazebo odometry as the range source and route
application messages between registered firmware processes. It must not return
a successful range for a missing or stale peer.

Every datagram is network byte order and has a 24-byte header, payload, and a
four-byte IEEE CRC-32 over header plus payload:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | ASCII `NCSU` |
| 4 | 1 | Protocol version, `1` |
| 5 | 1 | Packet kind |
| 6 | 1 | Flags, zero in v1 |
| 7 | 1 | Reserved, zero |
| 8 | 4 | Per-node request/event sequence |
| 12 | 8 | Gazebo simulation timestamp in nanoseconds; zero only for registration request |
| 20 | 1 | Source Nano node ID, `0..5` |
| 21 | 1 | Destination Nano node ID, `0..5`, or `255` for broker |
| 22 | 2 | Payload length in bytes |
| 24 | variable | Packet-kind payload |
| 24 + payload length | 4 | IEEE CRC-32 |

Packet kinds are `1 REGISTER`, `2 REGISTER_ACK`, `3 RANGE_REQUEST`,
`4 RANGE_RESPONSE`, `5 MESSAGE_REQUEST`, `6 MESSAGE_DELIVERY`,
`7 MESSAGE_RESPONSE`, `8 MEASUREMENT`, and `255 ERROR`.

`REGISTER` carries the node's advertised receive UDP port (`uint16`). The host
broker binds each registered node ID to the source IP plus advertised port and
returns `REGISTER_ACK` only after observing a finite, fresh Gazebo pose for
that node. Registration response timestamp and sequence identify that pose
snapshot.

`RANGE_REQUEST` has an empty payload and routes from requester to target peer.
`RANGE_RESPONSE` routes back from the peer to the requester and carries
`range_mm` (`uint32`) plus that peer's Gazebo-truth xyz position in millimetres
(`3 × int32`). `MEASUREMENT` carries the same range and peer position, with
source ID set to the measured peer and destination ID set to the node receiving
the measurement event. All response/event timestamps come from the exact
Gazebo pose snapshot used for the distance calculation. Firmware records its
local FreeRTOS receive tick separately and rejects replayed sequence numbers,
non-increasing simulation timestamps, and data older than the configured
freshness bound.

`MESSAGE_REQUEST`, `MESSAGE_DELIVERY`, and `MESSAGE_RESPONSE` payloads contain
the existing UWB API byte representation: `ctrl, src, dest, code, data[0..n)`.
The payload length is `4+n` and determines `n`; v1 limits `n` to
`MSG_DATA_LEN`, so the UWB message bytes are at most 118 bytes. The two-byte
radio FCS is not carried inside the UDP payload; when returning a message via
`uwb_send_msg_wait_rsp`, the C provider appends two zero padding bytes to the
local receive buffer so the existing `uwb_message_from_array` length
convention yields the same `data_len`. The broker routes messages only to a
currently registered live node. The C provider retains the existing ACK / ACK2
response semantics and exposes received messages through
`get_msg_from_queue`.

Every packet kind has an exact payload size except the three application
message kinds, which must be between 4 and `4 + MSG_DATA_LEN` bytes with a matching
embedded node identity. Unknown kinds, flags, versions, IDs, lengths, CRCs,
stale snapshots, duplicate sequences, and unregistered destinations are
rejected. Broker absence and peer loss map to `UWB_RX_TIMEOUT`; malformed
input maps to `UWB_RX_ERROR`. A broker may not synthesize ACKs or token return
messages for absent firmware peers in the acceptance path.

The repository currently has Python and C framing/field validators plus three
shared golden packets (registration, range response, and app-message request)
compiled and checked against the same exact bytes. This verifies framing only:
sequence replay and freshness enforcement, the FreeRTOS API adapter, and the
Gazebo pose broker are still unimplemented. Keep this contract in draft until
those behaviors have cross-process positive and negative tests.
