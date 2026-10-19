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
also reads its pose array in a separate transaction. The GAP9 asserts its busy
GPIO before blocking on the pose-array send; the STM32 waits for that ready
level before generating response clocks and waits for busy-low before using the
received floats. Busy waits are deadline-bounded, and failed pose reads leave
the caller's output buffer unchanged. These transfer boundaries are part of
the behavior; concatenating all bytes into one SPI CS assertion would not
reproduce the STM32 exchange sequence. Host tests validate the state and
timeout paths, but this does not establish the physical GPIO wiring or timing.

## Deterministic mapping trace

The first packet-driven mapping trace uses the production 60-pose trajectory
from `tests/gap9_sdk_gvsoc_mapping_smoke.c` and lets production `main.c`
dispatch production decoder results. In this target only the blocking SPI
transport is replaced by `tests/gap9_gvsoc_spi_trace_transport.c`; packets
remain 97 bytes, and point-cloud/ICP/SLAM work runs in the production code.
The fixture is finite and terminates the firmware after the last packet.

1. Send one deliberately malformed POSE frame for ID 0, then 60 valid POSE
   frames with IDs 0–59 in ascending order. The malformed frame has one
   checksum bit flipped and must be rejected without inserting a pose. For
   each valid ID `i`, encode
   timestamp `100*i`, position `(0.15*cos(0.02*i), 0.15*sin(0.02*i),
   0.02*i + pi/2)`, and set column 3 of each of the four ToF directions to
   `[1000, 1200, 900, 1100]` mm. Set every other ToF value to -1. The 84-byte
   payload is zero-initialized first, including its ABI padding; checksum is
   `sum(payload) mod 2^32`. Each valid frame must pass guard and checksum
   validation and increment the stored pose count once.
2. Send SCAN IDs 0, 20, and 40, each as its own 97-byte transfer. Their
   two-byte ID byte-sum fits the decoder's signed-int16 comparison and the
   checksum word has two zero high bytes. Each ID resolves to the same
   zero-based internal pose index and is registered once. Candidate scan
   origins are separated by less than the production 1.2 m gate. The attempted
   scan windows are `[0,20)`, `[20,40)`, and `[40,60)` at 1400 mm. The first
   scan has no reference candidate; the next two each have at least the first
   scan as a distance-eligible candidate. The owned core smoke establishes
   that this identical trajectory and range pattern yield at least two
   accepted ICP constraints; the packet test must confirm the production
   `check_internal_constraints` route yields the same minimum.
3. Send one OPTIMIZE frame. The request decoder is not involved; main dispatches
   command 2 directly. In the STM32 protocol the response is read in a separate
   transaction; this transport-seam test captures the bytes passed to the
   production `spi_send_blocking` boundary. The response is
   `sizeof(slam_job_t)` = 18 bytes, not a 97-byte frame: nine native
   little-endian int16 fields in declaration order
   (`number_of_poses`, `main_graph_size`, `max_subgraph_size`, `time_ms`,
   `has_nan`, `main_solver_returned`, `successful_solver_call_count`,
   `completed_solver_iteration_count`, `optimization_completed`). Expect 60
   poses, a finite completed solve, no NaN, and at least one solver call. The
   production scan policy should store exactly the undirected constraint
   endpoint pairs `{0,20}` and `{0,40}`; the trace validator checks both.

The 66-packet trace consists of 61 POSE frames (one malformed plus 60 valid),
four SCAN frames (three unique IDs plus one duplicate), and one OPTIMIZE frame.
The packet report distinguishes 60 valid frames decoded/inserted from 3 scans
registered and from unique constraints accepted by ICP; it counts exactly one
optimize command. A duplicate SCAN ID decodes successfully but does not
increase the scan count. The malformed POSE checksum is rejected without
increasing the pose count. The
decoder unit boundary should also reject a 96-byte POSE buffer before reading
fields; the physical main loop always requests exactly 97 bytes, so a short
hardware transfer is a transport failure, not a decoder-length case. Existing
focused host coverage lives in `tests/gap9_spi_decode_test.c` (run by
`tests/run_point_cloud_test.sh`).

On the GAP9, `main.c::slam_loop` initializes the SPI slave and repeatedly calls
`spi_rcv_blocking(spi_buf, SPI_PKT_SIZE)`, waits 100 microseconds, checks the
frame guards, decodes the command, and dispatches it. The driver uses PMSIS SPI
interface 2, chip select 0, 8-bit words, mode 0, and pads 56/57/58/59. The
decoder implementation is `cslam-gap9-app/spi/spi_cmds_decode.c`.

`pi_spi_open` has an integer status contract (`0` success, nonzero failure) in
the installed PMSIS `spi.h`. `spi_slave_driver_init` propagates that status and
`slam_loop` exits with failure before entering normal packet processing when
the peripheral cannot open. The synchronous `pi_spi_receive` and
`pi_spi_send` APIs are `void`; the installed GAP9 SPI driver has no public
timeout or abort operation. A missing clock source can therefore still block
the transport indefinitely. Do not treat `pi_evt_timeout_set` as a SPI timeout:
the installed SPI driver does not consume that event field. Bounded transfer
failure behavior is tracked separately in bead `ncs-1hd.12.1` until DMA
ownership and cancellation can be handled safely.

## GVSOC capability and validated software route

The locally installed GAP9 GVSOC model exposes a testbench master path in its
SDK source, but its end-to-end transfer has not yet been observed at the
firmware. Evidence in SDK 5.20.4 source:

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

Earlier testbench probes used `dummy_cycles=0` and did not produce the
firmware's SCAN marker. The exposed Python method returns after enqueuing
`master_cmd`; that alone is not evidence that the simulated transfer completed
or reached the slave. The isolated receive probe below now closes this virtual
target boundary with a target-side marker and the exact received frame.

## Isolated SPI2 receive probe

The GAP SDK 5.20.4 testbench can drive the production PMSIS slave driver when
its pre-CS wait clocks are configured. `Testbench_spi.open(dummy_cycles=8)`
generates the STM32 dummy byte's eight SCK edges before asserting CS;
`full_duplex` then sends the separate 97-byte SCAN frame with CS active. A
standalone one-byte `full_duplex` is not equivalent: it asserts CS and is
consumed as data by `pi_spi_receive`.

The isolated target at `tests/gap9_gvsoc_spi_slave_probe.c` links the owned
`spi/spi_driver.c`, arms a single 97-byte `pi_spi_receive`, prints all protocol
fields it observed, and validates the SCAN frame. The SDK control script checks
the generated `gvsoc_config.json` for the enabled testbench and both SPI2
bindings before it sends anything. The target marker is the acceptance
criterion; the testbench master's returned RX length is not used as proof.

Reproduce with the documented local Colima Docker context:

```sh
DOCKER_HOST=ssh://colima-act-ci GAP9_BUILD_GVSOC_SPI_SLAVE_PROBE=1 \
  bash scripts/build_gap9_firmware.sh
GAP9_GVSOC_ENABLE_TESTBENCH=1 \
GAP9_GVSOC_CONTROL_SCRIPT="$PWD/scripts/gvsoc_spi_slave_probe_control.py" \
GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-spi-slave-probe/nano_c_slam_gvsoc_spi_slave_probe" \
GAP9_GVSOC_EXPECTED_MARKER='NANO_C_SLAM_SPI_SLAVE_PROBE:payload_ok' \
GAP9_GVSOC_WORK_ROOT="${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-slave" \
  bash scripts/run_gap9_gvsoc_smoke.sh
uv run --locked python -m scripts.capture_gap9_gvsoc_spi_probe_evidence \
  --config "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-slave/gvsoc_config.json" \
  --elf cslam-gap9-app/BUILD-gvsoc-spi-slave-probe/nano_c_slam_gvsoc_spi_slave_probe \
  --host-transfer "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-slave/spi_slave_probe_host.json" \
  --log "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-slave/gvsoc.log" \
  --source cslam-gap9-app/spi/spi_driver.c \
  --source cslam-gap9-app/spi/spi_config.h \
  --source tests/gap9_gvsoc_spi_slave_probe.c \
  --output artifacts/gap9_gvsoc_evidence/spi_slave_probe.json
```

Evidence for the current probe is retained in
`artifacts/gap9_gvsoc_evidence/spi_slave_probe_20261018.json`. It binds the
ELF, generated GVSOC config, host-transmitted frame, and raw GVSOC log by
SHA-256; it also records SHA-256 for each supplied target source file. The
target received 97 bytes with `S`, `SCAN`, ID 0, checksum 0, and
`E`, then printed `payload_ok`. This validates one SPI2/CS0 receive on the
virtual target. It does not run production `main.c`/SLAM or establish mapping
accuracy, and it is not physical board or flight evidence.

For deterministic firmware-path coverage, the separate
`NANO_C_SLAM_GVSOC_SPI_TRACE` target substitutes only `spi_rcv_blocking` and
`spi_send_blocking`; it also links production `spi/spi_buffers.c`, which
`main.c` requires. It includes production `main.c`,
`spi/spi_cmds_decode.c`, and owned mapping sources. A 66-frame finite trace
passes through `get_spi_command`, the production decoders, `main.c` dispatch,
scan registration, ICP constraint generation, graph optimization, and response
capture. GVSOC passed on 2026-10-18: 60 poses decoded and inserted, one bad
pose checksum rejected, four scan packets decoded with three scan origins
registered (including duplicate ID 40 rejection), two constraints and two
optimizer edges, one optimizer command, and an 18-byte response matching the
firmware's `slam_job_t`. The test-only receive seam means this validates the
production software path on the GAP9 virtual target, not SPI electrical or
testbench transactions. It also does not measure map accuracy or hardware
behavior.

Reproduce the build and run with:

```sh
DOCKER_HOST=ssh://colima-act-ci GAP9_BUILD_GVSOC_SPI_TRACE=1 \
  bash scripts/build_gap9_firmware.sh
GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-spi-trace/nano_c_slam_gvsoc_spi_trace" \
  GAP9_GVSOC_EXPECTED_MARKER='NANO_C_SLAM_GVSOC_SPI:mapping_ok' \
  GAP9_GVSOC_WORK_ROOT="${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-trace" \
  bash scripts/run_gap9_gvsoc_smoke.sh
```

Run evidence for the rebuilt target: ELF SHA-256
`113bcebc85fc99ce50e0f10fa9a7774b2dd99ac2ecdc91b27ace9b1bc4ed8252`;
GVSOC log SHA-256
`b255dddfa539463ab423efc6b485ea4fcdcdfe261d54e00835dc7b3e30b65514`; the
raw log is checked in at
`artifacts/gap9_gvsoc_evidence/spi_trace_20261018.log`. It records each packet's
index, command, ID, 97-byte receive length, checksum validity (`-1` means not
applicable for OPTIMIZE), decoder result, and consumed pose/scan/constraint
counts. The aggregate marker records optimizer edges, solver calls, and the
18-byte response; the target validator compares the captured response with
the production `slam_job_t`. The rebuilt trace ELF link report uses 645296 B
of 1536 KiB L2 shared (41.03%) and 117612 B of 131068 B L1 (89.73%), leaving
13456 B L1. `riscv32-unknown-elf-size` reports 59942 B text, 525752 B data,
and 63084 B BSS. L1 headroom is tight; the trace build does not establish
production flight-memory margin.

## Production main loop over testbench SPI2

The separate `NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE` target joins the real
testbench SPI path to production command dispatch. It compiles `main.c`,
`spi/spi_driver.c`, `spi/spi_cmds_decode.c`, and the owned point-cloud/ICP/graph
sources. Only the probe build defines the packet bound; the normal firmware
main loop remains unbounded. Cumulative SPI accounting is enabled in both
builds by default and can be disabled with
`GAP9_BUILD_SPI_DIAGNOSTICS=OFF` when invoking the build script.

The controller sends one 97-byte POSE frame in the production 84-byte
`pcl_pose_t` wire layout, including four valid range samples and the unsigned
byte-sum checksum, followed by a 97-byte SCAN frame. It waits for the target
between transactions. The target exits successfully only after both production
decoders succeed, the pose table contains one pose, and scan history contains
one registered origin. A master-side transfer return without the target marker
is not accepted.

Reproduce using the documented Colima Docker context:

```sh
DOCKER_HOST=ssh://colima-act-ci GAP9_BUILD_GVSOC_SPI_MAIN_PROBE=1 \
  bash scripts/build_gap9_firmware.sh
GAP9_GVSOC_ENABLE_TESTBENCH=1 \
  GAP9_GVSOC_SPI_SEQUENCE=pose-scan \
  GAP9_GVSOC_CONTROL_SCRIPT="$PWD/scripts/gvsoc_spi_slave_probe_control.py" \
  GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-spi-main-probe/nano_c_slam_gvsoc_spi_main_probe" \
  GAP9_GVSOC_EXPECTED_MARKER='NANO_C_SLAM_GVSOC_SPI_MAIN:run_ok' \
  GAP9_GVSOC_WORK_ROOT="${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-probe" \
  bash scripts/run_gap9_gvsoc_smoke.sh
uv run --locked python -m scripts.capture_gap9_gvsoc_spi_main_probe_evidence \
  --config "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-probe/gvsoc_config.json" \
  --elf cslam-gap9-app/BUILD-gvsoc-spi-main-probe/nano_c_slam_gvsoc_spi_main_probe \
  --host-transfer "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-probe/spi_slave_probe_host.json" \
  --log "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-probe/gvsoc.log" \
  --source cslam-gap9-app/main.c \
  --source cslam-gap9-app/spi/spi_driver.c \
  --source cslam-gap9-app/spi/spi_cmds_decode.c \
  --source cslam-gap9-app/point-cloud/point_cloud.c \
  --output artifacts/gap9_gvsoc_evidence/spi_main_probe.json
```

Run evidence for 2026-10-18 is retained in
`artifacts/gap9_gvsoc_evidence/spi_main_probe_20261018.json`. The GVSOC target
reported two packets consumed, one valid pose decoded and inserted, one scan
decoded and registered. The ELF uses 641264 B of 1536 KiB L2 shared (40.77%)
and 117612 B of 131068 B L1 (89.73%); this is a test ELF resource report, not a
 production flight-memory margin. This demonstrates a short production software
 path through simulated SPI2. It does not run ICP/graph optimization, prove map
 quality, validate a physical bus, or establish flight behavior.

## Full mapping ingress over production SPI2

The 66-command scenario drives production `main.c` and `spi/spi_driver.c` over
the GVSOC SPI2/CS0 testbench. Unlike the trace target above, it does not replace
the receive transport. It sends 61 POSE frames (including one invalid
checksum), four SCAN frames (three unique origins plus one duplicate), and one
OPTIMIZE frame. The target must report 60 poses inserted, one checksum
rejection, three registered scan origins, two ICP constraints/optimizer edges,
and a finite completed graph solve.

The normal probe build keeps the production 18-byte slave response enabled.
After the optimizer has run, the testbench queues a master RX transaction,
clocks the actual PMSIS `spi_send_blocking`, and records the received bytes.
The bounded test target waits briefly after its result marker so the host can
drain and persist the asynchronous RX FIFO before simulation exit. The target
marker and host RX bytes must both match; a target-side expected-value marker
alone is insufficient.

Build and run with the documented local Colima context:

```sh
DOCKER_HOST=ssh://colima-act-ci \
  GAP9_BUILD_GVSOC_SPI_MAIN_PROBE=1 \
  GAP9_BUILD_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT=66 \
  bash scripts/build_gap9_firmware.sh
GAP9_GVSOC_ENABLE_TESTBENCH=1 \
  GAP9_GVSOC_SPI_SEQUENCE=mapping \
  GAP9_GVSOC_SPI_CAPTURE_RESPONSE=1 \
  GAP9_GVSOC_CONTROL_SCRIPT="$PWD/scripts/gvsoc_spi_slave_probe_control.py" \
  GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-spi-main-probe/nano_c_slam_gvsoc_spi_main_probe" \
  GAP9_GVSOC_EXPECTED_MARKER='NANO_C_SLAM_GVSOC_SPI_MAIN:mapping_ok' \
  GAP9_GVSOC_WORK_ROOT="${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-mapping" \
  bash scripts/run_gap9_gvsoc_smoke.sh
uv run --locked python -m scripts.capture_gap9_gvsoc_spi_main_probe_evidence \
  --config "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-mapping/gvsoc_config.json" \
  --elf cslam-gap9-app/BUILD-gvsoc-spi-main-probe/nano_c_slam_gvsoc_spi_main_probe \
  --host-transfer "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-mapping/spi_slave_probe_host.json" \
  --log "${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-mapping/gvsoc.log" \
  --source cslam-gap9-app/main.c \
  --source cslam-gap9-app/CMakeLists.txt \
  --source cslam-gap9-app/spi/spi_driver.c \
  --source cslam-gap9-app/spi/spi_cmds_decode.c \
  --source cslam-gap9-app/point-cloud/point_cloud.c \
  --source cslam-gap9-app/point-cloud/external_point_cloud.c \
  --source scripts/build_gap9_firmware.sh \
  --source scripts/gvsoc_spi_slave_probe_control.py \
  --source scripts/gap9_gvsoc_spi_probe_protocol.py \
  --source scripts/capture_gap9_gvsoc_spi_main_probe_evidence.py \
  --output artifacts/gap9_gvsoc_evidence/spi_mapping_main_response_20261018.json
```

The retained report hash-binds the generated config, ELF, full host TX frame
sequence, actual host RX response, GVSOC log, and production plus test-harness
sources. Two consecutive runs returned the same nonzero 18-byte response,
`3c0003001500800000000100030009000100`, matching `slam_job_t`. The run reports
641488 B of 1536 KiB L2 shared (40.78%) and 117612 B of 131068 B L1 (89.73%);
these are probe-build figures, not production flight-memory margin. This
validates the mapping path and feedback transfer on synthetic packet data in
the virtual target. It does not establish map accuracy, physical SPI behavior,
or flight readiness.

### Maximum-capacity optimizer timing probe

The bounded production-main probe also accepts packet count `276`, which
injects one rejected bad-checksum pose, all 260 supported poses, 13 scan
anchors, a duplicate final scan, and one OPTIMIZE command. It checks that all
260 poses and 13 scan anchors were retained and that the optimizer returned a
finite completed result. The success marker includes `slam_job.time_ms`; this
is the firmware-reported optimizer duration, not wall-clock GVSOC duration.

Run the probe three times with distinct work roots and retain each report/log
before selecting a timeout. A single deterministic trajectory is only
evidence for that workload; it does not prove a mathematical upper bound for
all sensor data or graphs. Apply an explicit safety margin and verify the
selected threshold against the observed maximum before removing the existing
unbounded STM32 waits.

The test controller uses a longer virtual delay after SCAN commands because
the production target performs synchronous ICP before posting its next SPI
receive. This is diagnostic pacing for the virtual testbench; it is not a
busy-pin timeout or evidence that the physical SPI link has the same behavior.

```sh
DOCKER_HOST=ssh://colima-act-ci \
  GAP9_BUILD_GVSOC_SPI_MAIN_PROBE=1 \
  GAP9_BUILD_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT=276 \
  bash scripts/build_gap9_firmware.sh
for run in 1 2 3; do
  work_root="${TMPDIR:-/tmp}/nano-c-slam-gvsoc-spi-main-max-${run}"
  GAP9_GVSOC_ENABLE_TESTBENCH=1 \
    GAP9_GVSOC_SPI_SEQUENCE=max-mapping \
    GAP9_GVSOC_SPI_CAPTURE_RESPONSE=1 \
    GAP9_GVSOC_CONTROL_SCRIPT="$PWD/scripts/gvsoc_spi_slave_probe_control.py" \
    GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-spi-main-probe/nano_c_slam_gvsoc_spi_main_probe" \
    GAP9_GVSOC_EXPECTED_MARKER='NANO_C_SLAM_GVSOC_SPI_MAIN:max_mapping_ok' \
    GAP9_GVSOC_WORK_ROOT="$work_root" \
    bash scripts/run_gap9_gvsoc_smoke.sh
  uv run --locked python -m scripts.capture_gap9_gvsoc_spi_main_probe_evidence \
    --config "$work_root/gvsoc_config.json" \
    --elf cslam-gap9-app/BUILD-gvsoc-spi-main-probe/nano_c_slam_gvsoc_spi_main_probe \
    --host-transfer "$work_root/spi_slave_probe_host.json" \
    --log "$work_root/gvsoc.log" \
    --source cslam-gap9-app/main.c \
    --source cslam-gap9-app/CMakeLists.txt \
    --source cslam-gap9-app/point-cloud/point_cloud.c \
    --source scripts/gvsoc_spi_slave_probe_control.py \
    --source scripts/gap9_gvsoc_spi_probe_protocol.py \
    --source scripts/capture_gap9_gvsoc_spi_main_probe_evidence.py \
    --output "artifacts/gap9_gvsoc_evidence/max_mapping_${run}.json"
done
```

With `NANO_C_SLAM_SPI_DIAGNOSTICS` enabled, each completed OPTIMIZE command
prints one `NANO_C_SLAM_SPI:stats` line. `rx` counts received fixed-size
transactions; `malformed` counts transactions without the `S...E` delimiters;
`unknown` counts delimited transactions with an unrecognized command. The
seven-entry `frames`, `decode_ok`, and `decode_error` lists correspond, in
order, to POSE, SCAN, OPTIMIZE, POSE_REQ, CHECK, EXT_POSE, and UPD_EXT_POSE.
OPTIMIZE and CHECK do not have payload decoders, so their decode counts remain
zero. `pose_id_missing` counts forward gaps between successfully decoded POSE
IDs, while `pose_id_nonadvancing` counts duplicate or backward IDs; the
sequence is a 16-bit counter and wraparound is handled. `optimize_responses`
counts completed blocking sends, not commands or attempted transmissions.
These counters are cumulative and printed only when optimization is requested;
they cannot reveal clocks lost before a complete fixed-size transaction or
losses in commands that carry no sequence number.

The captured 66-frame report with diagnostics enabled is retained at
`artifacts/gap9_gvsoc_evidence/spi_mapping_main_diagnostics_20261019.json`.
It observed 66 transactions, no malformed or unknown commands, 61 POSE, four
SCAN, and one OPTIMIZE frames; the pose decoder accepted 60 and rejected one
checksum, all four scan decodes succeeded, and one optimization response
completed. These are simulator accounting results, not physical bus-loss
measurements. To build the normal firmware without the counters, set
`GAP9_BUILD_SPI_DIAGNOSTICS=OFF`; the SPI protocol and mission behavior are
unchanged.
