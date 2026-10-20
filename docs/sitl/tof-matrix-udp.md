# CrazySim ToF matrix UDP frame

The SITL-only provider in `cslam-stm32-app/sitl/tof_matrix_sitl.c` binds UDP
port `19951` by default. Set `NCS_SITL_TOF_UDP_PORT` to choose another local
port. A Gazebo bridge must send one complete frame per simulation update; until
the first valid frame arrives the firmware returns `TOF_NOT_READY`, and a frame
older than 500 ms also becomes unavailable. No zero-filled matrix is reported
as valid data.

All multibyte fields use network byte order. The packet is exactly 540 bytes:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | ASCII `NCTF` |
| 4 | 1 | Version, `1` |
| 5 | 1 | Sensor count, `4` |
| 6 | 1 | Rows, `8` |
| 7 | 1 | Columns, `8` |
| 8 | 4 | Sequence number, strictly increasing modulo 2³² |
| 12 | 8 | Gazebo simulation time in nanoseconds, strictly increasing |
| 20 | 4 | Payload size, `512` |
| 24 | 512 | Four row-major matrices of signed millimetres, sensor IDs 0–3 |
| 536 | 4 | IEEE CRC-32 over bytes 0–535 |

Each cell is a signed 16-bit value: `-1` means no return; `0..4000` are valid
millimetres. Values below `-1`, larger distances, malformed headers, wrong
lengths, stale sequence numbers, non-increasing simulation time, and CRC
mismatches are discarded. Freshness uses the firmware task tick after receive,
not a comparison between Gazebo host time and the Linux firmware guest clock.

This defines and tests the firmware-side boundary. It does not by itself
provide Gazebo range data: a separate bridge must convert the four Gazebo
range sensors to the same matrix ordering and transmit frames to the guest.
