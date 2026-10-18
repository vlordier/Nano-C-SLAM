# GAP9 SPI integration in GVSOC

## Current execution boundary

`cslam-gap9-app/CMakeLists.txt` selects three GVSOC-only fixtures before its
production `else()` branch. `NANO_C_SLAM_GVSOC_MAPPING_SMOKE` builds
`tests/gap9_sdk_gvsoc_mapping_smoke.c`, DMA, ICP, SLAM, and point-cloud sources;
it does not compile `main.c` or `spi/*.c`. The ordinary production target
`nano_c_slam_gap9` compiles `main.c`, `spi/*.c`, and the mapping sources. The
existing mapping report therefore says nothing about SPI reception or the
production command loop.

## Wire contract

Every STM32 command exchange is a 97-byte frame. Bytes 0 and 96 are the `S`
and `E` guards, byte 1 is the command, and command payloads begin at byte 2.
Multi-byte fields and raw C structs are copied in the STM32's native
little-endian representation. The command values are POSE=0, SCAN=1,
OPTIMIZE=2, POSE_REQ=3, CHECK=4, EXT_POSE=5, and UPD_EXT_POSE=6. The STM32
builders are in `cslam-stm32-app/src/spi_comm/spi_comm.c`.

| Command | Payload and checksum in the 97-byte frame | GAP9 handling |
| --- | --- | --- |
| POSE (0) | 84-byte `aug_pose_t` at [2,86); uint32 byte-sum at [86,90) | `spi_decode_pose`; checksum is the sum of all 84 payload bytes |
| SCAN (1) | int16 pose id at [2,4); uint32 byte-sum at [4,8) | `spi_decode_scan_info`; checksum sums bytes 2–3 and compares to signed int16 read at 4 |
| OPTIMIZE (2) | No payload/checksum; `E` at byte 96 | Production loop runs `pcl_optimize`, then sends raw `sizeof(slam_job_t)` feedback |
| POSE_REQ (3) | int16 `from_id` [2,4), int16 count [4,6), uint32 byte-sum [6,10) | `spi_decode_pose_req`; checksum sums bytes 2–5 and compares to signed int16 read at 6; response is `3 * sizeof(float) * count` bytes |
| CHECK (4) | No matching `case` in the current GAP9 `main.c` | Frame is ignored |
| EXT_POSE (5) | 84-byte `ext_aug_pose_t` [2,86); uint32 byte-sum [86,90) | `spi_decode_ext_pose`; checksum covers all 84 payload bytes |
| UPD_POSE (6) | int16 pose id [2,4), int16 drone id [4,6), three floats [6,18), uint32 byte-sum [18,22) | `spi_decode_ext_pose_upd`; checksum covers only the 12 position bytes |

The STM32 builders issue a separate one-byte dummy-clock transaction before
each command transaction. OPTIMIZE then waits for the GAP9 busy GPIO and makes
a second dummy-clock transaction before reading the raw `slam_job_t`. POSE_REQ
also reads its pose array in a separate transaction. These transfer boundaries
are part of the behavior; concatenating all bytes into one SPI CS assertion
would not reproduce the STM32 exchange sequence.

On the GAP9, `main.c::slam_loop` initializes the SPI slave and repeatedly calls
`spi_rcv_blocking(spi_buf, SPI_PKT_SIZE)`, waits 100 microseconds, checks the
frame guards, decodes the command, and dispatches it. The driver uses PMSIS SPI
interface 2, chip select 0, 8-bit words, mode 0, and pads 56/57/58/59. The
decoder implementation is `cslam-gap9-app/spi/spi_cmds_decode.c`.

## GVSOC capability and selected route

The locally installed GAP9 GVSOC model has a real testbench master path; no
firmware transport bypass is needed. Evidence in SDK 5.20.4 source:

* `gvsoc/gvsoc_gap/gap9/gap9_v2.py` selects `gap9.evk.Gap9_v2`, a subclass of
  `Gap9_evk`.
* `gvsoc/gvsoc_gap/gap9/evk.py` has an `addon_testbench_enabled` property. When
  enabled, it binds the GAP's `spim2_cs0_data` and `spim2_cs0` ports to the
  testbench's `spi2_cs0_data` and `spi2_cs0` ports. The project's firmware
  config is SPI2/CS0, so the interface matches.
* `gvsoc/gvsoc/models/gvsoc/gvsoc_control.py::Testbench_spi` exposes
  `open(is_master=1)`, `send`, `full_duplex`, and `receive`. The testbench
  component's `spi master_cmd` handler enqueues real SPI transfers in
  `gvsoc/gvsoc/models/devices/testbench/testbench.cpp` and drives them through
  the QSPIM slave interface. `--control-script` is supported by the GVSOC
  runner, so a deterministic host script can drive this API.

The next integration run should enable the platform property
`addon_testbench_enabled=true`, supply a GVSOC control script, select SPI2/CS0,
and run a production-linked firmware ELF. It must model each STM32 transaction
as its own transfer, including dummy clocks and the separate OPTIMIZE feedback
read. Keep `main.c`, `spi_driver.c`, and `spi_cmds_decode.c` in that ELF. Count
accepted POSE/SCAN commands and observed optimizer calls from firmware-side
markers; the GVSOC smoke must fail if an expected marker or response is absent.

No runtime SPI packet has yet been driven through this route. This note proves
that the installed model exposes the needed master operation and identifies
the exact next run, but does not claim production-main or mapping-path
validation. If the testbench route fails on the installed runtime, the fallback
is a compile-time test transport that replaces only the blocking receive/send
calls; it must feed the same 97-byte buffer to `get_spi_command`, the same
production decoders, and the same command dispatch loop. The existing mapping
smoke is not such a transport test because it bypasses all three.
