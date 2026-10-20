# CrazySim UWB UDP contract (draft v1)

This is the host boundary for the Linux `uwb_api_sitl.c` provider. The C
provider remains fail-closed until the runtime adapter is implemented. The
broker core and local UDP transport are implemented; the broker uses timestamped
Gazebo odometry as its range source and routes application messages between
registered firmware processes. It must not return a successful range for a
missing or stale peer.

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

`REGISTER` carries the node's advertised receive UDP port (`uint16`). The host
broker binds each registered node ID to the source IP plus advertised port and
returns `REGISTER_ACK` only after observing a finite, fresh Gazebo pose for
that node. Registration response timestamp and sequence identify that pose
snapshot. Firmware repeats the zero-time registration packet as a bounded
lease renewal; the broker refreshes the node lease without replying to every
renewal.

`TIME_SYNC` has an empty payload and routes broker-to-node. It is emitted
periodically from Gazebo's simulation clock and carries the current clock in
the header timestamp. Firmware uses the most recently received TIME_SYNC as
`now_sim_time_ns` for freshness checks; if the clock heartbeat exceeds its
wall-clock lease or simulation time stops advancing, ranging and message
responses fail closed. The broker uses one monotonic event sequence per node
for registration acknowledgements, clock syncs, and errors.

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

The C and Python codecs expose a per-flow freshness guard. Callers provide
current Gazebo simulation time and a nonzero maximum age. The guard rejects
zero/future/stale timestamps and non-increasing sequence or simulation time;
the API adapter must keep independent state by packet kind and source and
destination IDs. Replies echo the originating request sequence for matching;
sequence order is monotonic within each such flow. The guard is unit-tested but
is not yet wired into `uwb_api_sitl.c`.

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

`ERROR` carries one code byte: `1` unknown/unregistered peer, `2` missing,
stale, or skewed pose, `3` invalid or unrepresentable range request, and `4`
broker pending-message capacity exhausted. All errors originate at the broker
and are addressed to the requesting node.

Run the broker on the native Gazebo host with `uv run --locked python -m
scripts.uwb_sitl_broker --node-ids 0,1 --bind-address 0.0.0.0 --bind-port
19952`. It subscribes through `gz topic -e --json-output` to `/clock` and
`/cf_<id>/odom`, then sends UWB datagrams to each registered firmware IP and
port. Host tests cover the broker state machine and local UDP registration and
range exchange. The broker has not yet been exercised against live Gazebo or
CrazySim firmware. The C API adapter remains fail-closed, so these tests do not
prove that Nano tasks start or drones fly.

The repository has Python and C framing/field validators plus four shared
golden packets (registration, range response, app-message request, and clock
sync) compiled and checked against the same exact bytes. Replay/freshness
guards are unit-tested but not wired into `uwb_api_sitl.c`. Keep this contract
in draft until the C adapter and live Gazebo broker pass cross-process positive
and negative tests.
