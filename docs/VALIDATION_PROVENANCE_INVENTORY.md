# Validation provenance inventory

This inventory describes what the current collectors observe from the process
and files that produced a result. A path, a Git revision, or a hash copied from
another run does not establish that a dependency was loaded by this run.

## Evidence sources and gaps

| Evidence class | Current collector and fields | What the evidence does not establish yet |
| --- | --- | --- |
| Host GAP9 mapping fixture | `scripts/capture_gap9_host_mapping_diagnostic.py` hashes the host binary, owned mapping sources, fixture, compile flags, and raw log. The schema validator rejects incomplete host evidence and target-only SDK metadata. | It does not record the superproject commit, recursive submodule gitlinks/checkouts, or a dirty patch. The compiler identity is supplied by the invoking command rather than resolved from the running compiler. |
| GAP9 GVSOC mapping fixture | `scripts/capture_gap9_gvsoc_mapping_diagnostic.py` binds ELF, compile-command file, build-provenance file, SDK revision/release/dirty flag, required owned source hashes, target exit/termination, and raw log hash. It rejects host evidence labeled as target execution. | Build provenance currently omits the root repository revision and submodule state. The SDK dirty check uses `git diff-index` and does not include untracked files. A target artifact may have `model_metadata: null`; the latest `gvsoc_icp_accounting_20261018.json` is an observed example. It therefore does not bind the MCHAN plugin actually loaded by that run. A separate runtime comparison may identify plugin files, but that is not a substitute for per-run loaded-dependency provenance. |
| Gazebo-to-host mapping replay | `scripts/gazebo_mapping_sitl.py` records the root revision/dirty status, mapper executable SHA, selected source dirty state, input samples/pair timestamps, replay metrics, and simulator/run identity. The coordinator's `nano_c_slam_gazebo/run_provenance.py` additionally hashes an explicit owned-source set, Python/`ros2`/Gazebo/mapper executables, installed overlay package trees, ROS domain and Gazebo partition. | Coordinator dirty status is scoped to its owned input set, not recursive submodule state. The environment record omits `RMW_IMPLEMENTATION`, sourced prefix order, and a general allowlisted environment snapshot. The combined report does not retain a hash for the complete launcher stdout/stderr stream. It establishes simulated pipeline execution, not map accuracy. |
| Gazebo mission without mapping replay | Mission reports and passive telemetry diagnostics bind run IDs and sampled topics/statuses, but coverage differs by report schema. | An observed graph endpoint is not command receipt. A model-only test is not simulator execution. Do not infer mission completion, mapping execution, or physical safety from a telemetry-only artifact. |
| Physical GAP9 / Crazyflie flight | No checked-in collector currently emits an accepted hardware evidence bundle. | GVSOC, host tests, ideal odometry, and Gazebo cannot be relabeled as hardware/flight evidence. Until a hardware collector and retained raw evidence exist, this row is `NOT RUN` or `BLOCKED`, with the missing device/prerequisite stated. |

## Existing implementation and test anchors

- GAP9 source/build schema: `scripts/gap9_mapping_diagnostic.py`,
  `scripts/write_gap9_build_provenance.py`, and both
  `scripts/capture_gap9_*_mapping_diagnostic.py` entrypoints.
- Host/GVSOC evidence-class and source matching: `scripts/compare_gap9_mapping_diagnostics.py`.
- Gazebo source/executable/overlay collection and validation:
  `sim/gazebo/nano_c_slam_gazebo/nano_c_slam_gazebo/run_provenance.py`.
- Gazebo mapping report and host mapper identity:
  `scripts/gazebo_mapping_sitl.py`.
- Existing mutation and fail-closed tests:
  `tests/test_gazebo_mapping_run_provenance.py`,
  `tests/test_gap9_mapping_comparison.py`, and
  `tests/test_gap9_host_mapping_diagnostic.py`.
- Latest per-pair ICP and scan-window captures are
  `artifacts/gap9_gvsoc_evidence/host_icp_accounting_20261018.*` and
  `gvsoc_icp_accounting_20261018.*`. The comparison is
  `host_gvsoc_comparison_icp_accounting_20261018.json`. The GVSOC capture's
  `model_metadata: null` is a concrete missing-provenance example, not an
  assertion that its external runtime model was embedded in the ELF.

## Implementation order

1. Resolve repository, recursive submodule, dirty-tree, compiler, and loaded
   model/plugin identities from the same process/run that produces each
   artifact. Keep unknown values explicitly unknown.
2. Make the MCHAN runtime dependency available from pinned SDK source or retain
   it as a content-addressed artifact; verify output bytes rather than trusting
   a developer-local absolute path.
3. Extend the strict versioned manifest and mutation tests to cover source,
   submodules, ELF, runtime dependencies, report, and raw logs. Evidence class
   must be derived from the execution path and cannot be promoted by editing a
   string.
4. Add ROS/RMW and sourced-overlay identity from the native running process.
   Use an allowlist for environment values; do not serialize secrets.
5. Only then assemble the one-command capability matrix. A missing physical
   prerequisite remains `NOT RUN`/`BLOCKED`; it must never inherit a host or
   GVSOC pass.
