# Reproduction status

Tested on macOS (Apple Silicon), with the evidence dates recorded below. The repository's pinned submodules and `changes.patch` were used. Gazebo and ROS 2 SITL ran natively on macOS. No hardware flight or mapping-accuracy result was reproduced.

## Current capability status (2026-10-18)

| Capability | Status | Evidence and boundary |
| --- | --- | --- |
| One Crazyflie flying a bounded mission in native Gazebo | Passed 2026-10-17 | Native ROS 2 Jazzy/Gazebo Harmonic run documented below and in `sim/gazebo/README.md`; this is simulated flight. |
| One bounded Gazebo mission bound to host mapping replay | Passed for retained historical run | `artifacts/gazebo_mapping_sitl/cf1_nearfield_bound_mission_20261017.json` (schema v1) binds mission events and host replay to run ID `f2aa2fc0-eff5-4718-946c-9149028ff6d0`. The mission reached `COMPLETE` after its requested path and landed at 0.0847 m; the captured replay processed 60 poses, 3 scans, 201 projected points and 3 unique optimizer-input edges, with a finite 3-node graph and 3 successful solver calls. Replay provenance binds the native-host binary hash, but this legacy report does not hash the coordinator, runtime tools, or ROS overlay. New coordinator runs emit schema v2 with those identities. The evidence establishes the integrated simulated-input pipeline, not map quality, convergence, or firmware execution. |
| Owned host C mapper processing Gazebo telemetry | Passed for retained captures | A separate near-field replay has 170 timestamped pose/range pairs, 9 scans, 588 projected points and 11 stored constraints submitted to the final optimizer call. The shared deterministic host diagnostic is `artifacts/gap9_gvsoc_evidence/host_mapping_diagnostic.json` with its raw `.log`. These results prove pipeline execution, not map accuracy or convergence. |
| GAP9 firmware build | Passed 2026-10-18 | Fresh SDK 5.20.4 build with corrected ICP correspondence output: 63,382 text, 526,920 data, 56,700 BSS; L1 117,612/131,068 B (89.73%), L2 shared 643,536/1,572,864 B (40.91%). |
| GAP9 GVSOC ICP kernel smoke | Build passed; execution evidence is older | Retained `report.json` is kernel-only evidence. It does not exercise mapping. |
| GAP9 GVSOC mapping-core smoke | Passed on 2026-10-18 | Fresh schema-v2 evidence is retained in `artifacts/gap9_gvsoc_evidence/mapping_diagnostic.json` and `.log`, bound to the target ELF, compile commands, SDK and source hashes. The fixture completed 60/60 finite poses, 3 scans, 240 points, 2 accepted ICP constraints, 2 unique optimizer edges, and 6 solver calls/18 fixed iterations with no NaN. The host fixture also passes but reports 3 graph nodes/3 calls/9 iterations; GVSOC reports 12 nodes/6 calls/18 iterations because `pcl_reduce` can add distance-selected poses. This topology difference remains under host/target comparison. This fixture is distinct from the production-main SPI test below; neither run measures map accuracy. |
| Forced cluster Cholesky on synthetic fixture | Passed with isolated model rebuild | The 60-pose fixture passes with the FC fallback disabled only when GVSOC uses MCHAN rebuilt from committed GAP SDK source. The installed model fails the same ELF. See `artifacts/gap9_gvsoc_evidence/cluster_cholesky_clean_model_report.json` and the raw `.log`; the production fallback remains enabled pending physical-target confirmation. |
| Production GAP9 main loop consuming deterministic SPI packets | Passed with test-only transport seam on 2026-10-18 | `NANO_C_SLAM_GVSOC_SPI_TRACE` builds production `main.c`, decoder, and mapping sources; the fixture replaces only blocking SPI receive/send. GVSOC consumed 60 valid poses, rejected one bad checksum, registered 3 of 4 scan packets, stored exact constraint edges 20→0 and 40→0, optimized once, and returned an 18-byte `slam_job_t`. This validates production software dispatch/mapping on the virtual GAP9 target, not the GVSOC SPI testbench bus, physical SPI, map accuracy, or hardware. See [cslam-gap9-app/spi/GVSOC_INTEGRATION.md](cslam-gap9-app/spi/GVSOC_INTEGRATION.md) for command, resource usage and ELF/log hashes. |
| Production GAP9 main loop over GVSOC testbench SPI2 | Passed twice on 2026-10-18 | `NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE` drives 66 SPI2/CS0 testbench frames through production `main.c`, decoder, and mapping sources. It inserted 60 poses, rejected one bad pose checksum, registered 3 of 4 scans, accepted 2 ICP constraints, optimized once, and captured the expected 18-byte `slam_job_t`. The received response matches the expected bytes in `artifacts/gap9_gvsoc_evidence/spi_mapping_main_response_20261018.json`, which binds the ELF, GVSOC config, host transfer, log, and target sources by SHA-256. This validates the virtual SPI2 path, not physical SPI, map accuracy, or hardware. See [cslam-gap9-app/spi/GVSOC_INTEGRATION.md](cslam-gap9-app/spi/GVSOC_INTEGRATION.md) for reproduction and resource details. |
| Host/GVSOC first recorded divergence | Compared 2026-10-18 | The host and target mapping reports share fixture SHA `56bd95c6…43a0fa` and 51 matching source hashes. `artifacts/gap9_gvsoc_evidence/host_gvsoc_comparison_20261018_v2.json` identifies the first out-of-tolerance recorded difference in `reverse-cluster.total` (41 host, 80 GVSOC), before final graph summaries. It records residuals, pose-delta norms, and convergence as unmeasured. This identifies the first logged discrepancy, not its root cause. |
| Three-drone mission, map fusion, physical flight, quantitative map accuracy | Not demonstrated | Separate open work remains; per-drone mapping is not map fusion, and simulated four-ray lidar/ideal odometry are not hardware ToF/UWB. |

The provenance-bound cluster diagnosis identified lost correspondence-array values at the cluster-to-FC boundary; bounded cluster-master stores fixed that target smoke. Production `main.c` has since passed both the deterministic transport-seam trace and a testbench SPI2 transfer sequence, including capture of a byte-matched optimizer response. The host/GVSOC graph topology and solver-work difference remains open (`ncs-ju6.3.2`). Neither synthetic fixtures nor host/GVSOC execution establishes mapping accuracy or physical hardware behavior. Repeated captures preserve earlier logs/reports; the implicit default output becomes timestamped when occupied, while an explicit path collision fails before the build.

## Host versus GVSOC first-divergence comparison

Compare the retained host/target artifacts and the raw logs whose hashes are
bound by those artifacts:

```sh
uv run --locked python -m scripts.compare_gap9_mapping_diagnostics \
  --host-artifact artifacts/gap9_gvsoc_evidence/host_mapping_diagnostic.json \
  --gvsoc-artifact artifacts/gap9_gvsoc_evidence/mapping_diagnostic.json \
  --host-log artifacts/gap9_gvsoc_evidence/host_mapping_diagnostic.log \
  --gvsoc-log artifacts/gap9_gvsoc_evidence/mapping_diagnostic.log \
  --output /tmp/nano-c-slam-host-gvsoc-comparison-rerun-v2.json
```

The comparator rejects mismatched fixtures, shared source hashes, invalid
artifacts, or log hashes before writing output. On the retained pair, the first
observed out-of-tolerance trace difference is `reverse-cluster.total`: host
reports 41 and GVSOC reports 80 (host log line 834; target line 919), after ICP
iteration 24. It records 1,557 earlier float-component differences within the
declared per-field tolerances. The later summaries differ at graph topology
(3 versus 12 nodes) and optimizer work (3/9 versus 6/18 solver calls/iterations).
This localizes the earliest recorded discrepancy; it does not explain its root
cause. Individual pose/range values, scan extraction internals, and every
intermediate optimized pose are not present in these logs, and are listed as
unobserved by the comparator. Solver call and fixed-iteration counts describe
work performed; residuals, pose-delta norms, objective improvement, and
convergence are unmeasured.

## STM32 firmware

Prerequisites: an ARM GNU toolchain, `make`, and the repository's submodules. The tested toolchain was `arm-none-eabi-gcc` 15.3.1.

```sh
git submodule update --init --recursive
cd crazyflie-firmware
git apply ../changes.patch
cd ../cslam-stm32-app
make all
```

The build succeeds on this branch and produces `cslam-stm32-app/build/cf2.bin`. The linker reports 90% RAM and 95% CCM usage. The patch modifies files inside the pinned Crazyflie firmware submodule; it is required by the upstream build instructions and is not a submodule revision change.

Runtime buffers are fixed-size. The STM32 tracks up to 30 scan poses and queues up to 100 external augmented poses or 200 pose updates between transfers to GAP9. When a buffer fills, new data is dropped and a debug message is emitted. Longer missions and larger swarms require a measured capacity plan.

## GAP9 firmware

The local GAP9 SDK at `~/Work/da3_gap9/third_party/gap_sdk_private` reports release 5.20.4. The direct Make flow refers to absent FreeRTOS rules. On 2026-10-18, the SDK CMake build completed through the x86_64 `colima-act-ci` VM using Docker-over-SSH and `uv`, without modifying the SDK checkout. After the ICP fix, the current firmware reports 63,382 bytes text, 526,920 data, and 56,700 BSS; linker placement uses 643,536 B L2 shared (40.91%) and 117,612 B L1 (89.73%). L1 headroom is tight. The SDK emits a `pi_assert` implicit declaration warning from its own `udma_core.h` include chain. A subsequent attempt to rebuild the SVD smoke in the same VM stopped with `No space left on device`; the mapping target had already been rebuilt and its fresh GVSOC run passed.

The native GVSOC launcher is available locally on macOS, and the tracked result in `artifacts/gap9_gvsoc_evidence/report.json` observes the expected ICP SVD singular values (3.000, 2.000). This is kernel-only evidence. The full firmware and fixtures compile with the SDK 5.20.4 toolchain in the x86_64 Colima VM. The current schema-v2 report at `artifacts/gap9_gvsoc_evidence/mapping_diagnostic.json` binds the passing target mapping-core run to its ELF, compile commands, SDK, sources and raw log. The identical 60-pose/3-scan/240-point fixture accepts 2 ICP constraints and completes pose-graph optimization with all poses finite. Bounded cluster-master stores transfer correspondence arrays correctly. GVSOC selected 12 graph nodes and 6 solver calls versus host's 3 and 3, a remaining topology/work difference for `ncs-ju6.3.2`. A separate 66-frame production-main testbench SPI2 run is recorded below; its captured 18-byte response matches the expected `slam_job_t`. These GVSOC results do not validate physical SPI, sensors, flight, or map accuracy.

```sh
DOCKER_HOST=ssh://colima-act-ci bash scripts/build_gap9_firmware.sh
bash scripts/build_host_mapping_sitl.sh
cslam-gap9-app/BUILD-host/gap9_mapping_smoke
GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke" \
bash scripts/run_gap9_gvsoc_smoke.sh # exits 1 after the expected mapping_fail marker
uv run --locked python scripts/capture_gap9_gvsoc_mapping_limitation.py \
  --elf cslam-gap9-app/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke \
  --log "$TMPDIR/nano-c-slam-gvsoc/gvsoc.log" \
  --output artifacts/gap9_gvsoc_evidence/mapping_limitation.json \
  --gvsoc-exit-code 1
```

The build command uses four jobs by default (`GAP9_BUILD_JOBS` can override this); set `GAP9_BUILD_DNS` if the Colima VM needs a different resolver. The separate host smoke passes the same owned point-cloud/ICP/pose-graph fixture with PMSIS stubs at the cluster compute boundary. Its result does not turn the GVSOC failure into a firmware mapping pass. Neither run establishes sensor, SPI, flight, hardware timing, or mapping accuracy. The complete firmware ELF was also observed reaching `main()` and opening the cluster in this virtual model, then waiting for the STM32 SPI packet; GVSOC has no STM32/SPI stimulus in this setup.

### Isolated cluster Cholesky / GVSOC MCHAN diagnostic

A separate N=63 diagnostic found that cluster Cholesky computes a factor in L1, but the installed GVSOC MCHAN plugin does not return it to FC-visible L2. A four-word probe reproduces the same symptom for both the repository DMA wrapper and direct PMSIS LOC2EXT. The installed `mchan_v7_impl.dylib` (SHA-256 `e4677a2e…f1735aa`) fails the probe and, with the serial fallback disabled in a temporary worktree, the synthetic mapping fixture fails its pose-bound check after scan 20. Rebuilding the plugin from the SDK's committed `mchan_v7_impl.cpp` in an isolated model directory (SDK commit `45247566dbfd1ec7bb021f158fefe133c2aa8e10`, source SHA-256 `e5e72f1d…ca9f4d`) passes both DMA directions and the same cluster-path fixture. The rebuilt model SHA-256 is `63c7a156…9d719bd1`; the fixture completes 60 poses, 3 scans, 240 points, 3 graph nodes, and 3 solver calls/9 iterations with all poses finite. This narrows the discrepancy to the installed model artifact/build, but does not establish why that artifact differs or validate physical GAP9 DMA. Keep the default FC fallback threshold at 64 until a board confirms the transfer and solve.

The diagnostic model builder reads the committed SDK source with `git show`, so it does not consume or overwrite uncommitted SDK source edits. These commands rebuild it in a temporary model tree, build a diagnostic ELF with the cluster path forced, then run the DMA and mapping probes:

```sh
GAP9_GVSOC_MODEL_BUILD_ROOT="$TMPDIR/ncs-gvsoc-model" \
  bash scripts/build_gap9_gvsoc_mchan_model.sh
DOCKER_HOST=ssh://colima-act-ci GAP9_BUILD_GVSOC_CLUSTER_DIAGNOSTIC=1 \
  bash scripts/build_gap9_firmware.sh
DOCKER_HOST=ssh://colima-act-ci GAP9_BUILD_GVSOC_DMA_SMOKE=1 \
  bash scripts/build_gap9_firmware.sh
GAP9_GVSOC_MODEL_ROOT="$TMPDIR/ncs-gvsoc-model/models" \
GAP9_GVSOC_POSE_GRAPH_FC_THRESHOLD=0 \
GAP9_GVSOC_MAPPING_REPORT="$PWD/artifacts/gap9_gvsoc_evidence/cluster_cholesky_clean_model_report.json" \
GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-cluster-diag/nano_c_slam_gvsoc_mapping_smoke" \
GAP9_GVSOC_EXPECTED_MARKER='NANO_C_SLAM_GVSOC:mapping_ok' \
  bash scripts/run_gap9_gvsoc_smoke.sh
GAP9_GVSOC_MODEL_ROOT="$TMPDIR/ncs-gvsoc-model/models" \
GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-dma/nano_c_slam_gvsoc_dma_smoke" \
GAP9_GVSOC_EXPECTED_MARKER='NANO_C_SLAM_GVSOC_DMA:ok words=4' \
  bash scripts/run_gap9_gvsoc_smoke.sh
```

`NANO_C_SLAM_POSE_GRAPH_FC_THRESHOLD` defaults to 64; setting it to 0 is only for this diagnostic build. The DMA ELF is built separately with `GAP9_BUILD_GVSOC_DMA_SMOKE=1`. These results are GVSOC evidence only. The production-main testbench SPI2 path is covered by the separate test below; physical SPI, real sensors, flight, map accuracy, and physical DMA remain unvalidated.

## Gazebo SITL

`sim/gazebo/` contains a ROS 2 bridge for the four-ray Crazyswarm2 Gazebo backend. ROS 2 Jazzy and Gazebo Harmonic ran natively on macOS using the setup documented in [sim/gazebo/README.md](sim/gazebo/README.md). The bounded square mission completed takeoff, mapping-path flight, hover, and landing; a separate 150-sample replay passed through the host C mapping core using the Gazebo 3.49 m lidar range. Its report, `artifacts/gazebo_mapping_sitl/cf1_native_gazebo_mapping_path.json`, is explicitly historical and not firmware-faithful because the mapper then used a broader range cutoff.

The default Gazebo arena returns no projected points under the firmware's 1.4 m loop-closure cutoff. The optional `NANO_C_SLAM_NEARFIELD_MAPPING=1` launch derives a temporary near-field world from the installed Crazyswarm2 SDF without editing that checkout or changing the cutoff. A native macOS run captured 60 fresh poses during a bounded square mission and completed landing; the owned host C mapper produced 3 scans, 200 projected points, 4 unique constraints stored and passed to optimization (2 explicit ICP correction attempts plus internal scan constraints), and a finite 3-node graph. All three 20-pose windows had in-range readings; across the run, each of the four beams had multiple distinct readings at or below 1400 mm. Range accounting recorded 240 readings: 11 invalid, 200 accepted, and 29 valid readings above the limit. The capture and mission log are retained in `artifacts/gazebo_mapping_sitl/cf1_nearfield_firmware_cutoff.json` and `artifacts/gazebo_mapping_sitl/cf1_nearfield_mission.log`. This proves host mapping-pipeline execution on simulated four-ray lidar and odometry, not map accuracy, GAP9 firmware/SPI execution, or hardware ToF/UWB behavior.

A newer stamped capture at `artifacts/gazebo_mapping_sitl/cf1_nearfield_stamped_capture.json` contains 170 same-drone range/odometry pairs, 9 scans, 588 projected points, 11 unique stored constraints submitted to the final `ls_slam` call, and a finite 9-node graph. The report records adjacent ICP scan-pair attempts and accepted insertions separately from all stored canonical endpoint pairs and compact optimizer-input edges. Source timestamps had 0 ns pair skew and 10 Hz median frequency; 680 readings reconcile to 11 invalid, 588 within the exact 1400 mm cutoff, and 81 valid readings above it. This demonstrates execution of the owned host mapping path on captured Gazebo telemetry. It does not establish optimizer convergence or map accuracy, nor does it bind the mission's human-readable landing result to this capture; those remain open beads. See `cslam-gap9-app/point-cloud/CONSTRAINT_ACCOUNTING.md` for the exact constraint semantics.

`artifacts/gazebo_mapping_sitl/cf1_nearfield_stamped_freshness.json` is a separate native freshness capture with per-pair age measurements: 170 pairs at 10 Hz, 100–112 ms source age, and 0 ns range/odometry skew. The host mapping replay also completed (170 poses, 9 scans, 645 projected points, 15 unique stored constraints submitted to the final optimizer call). Adversarial review found only 2.7 µm horizontal path length, so this artifact supports telemetry freshness and replay execution only; it does not demonstrate spatial excitation or a useful map. A subsequent attempt at a clean square mapping run failed when Gazebo odometry went stale, and that failed run wrote no report.

The older `cf1_native_gazebo*.json` captures have no retained raw pose/range samples or endpoint pairs. Their historic scalar constraint count is marked `legacy-unverifiable` and cannot establish unique graph edges or optimizer input.

## Python demo

The demo dependencies are locked in `pyproject.toml` and `uv.lock`. The tested environment uses Python 3.12 and `cflib` 0.1.34:

```sh
uv sync --locked
uv run --locked ruff check .
uv run --locked ruff format --check .
uv run --locked ty check --error-on-warning
uv run --locked python -m unittest discover -s tests -p 'test_*.py'
uv run --locked python -m py_compile cslam-stm32-app/run_demo.py
```

The runtime requires a Crazyradio and configured drones. From `cslam-stm32-app/`, run `uv run --locked --project .. python run_demo.py 2`. This command reaches the radio connection step on the test machine, then exits because no Crazyradio is attached. No flight was started.
