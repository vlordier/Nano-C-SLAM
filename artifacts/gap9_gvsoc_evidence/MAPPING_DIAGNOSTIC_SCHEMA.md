# Shared mapping diagnostic schema

`mapping_diagnostic.schema.json` defines version 2. Python producers and
consumers should additionally call `scripts.gap9_mapping_diagnostic`
validation because the JSON Schema documents field types while the validator
enforces cross-field invariants.

New host and GVSOC captures may include a `provenance_manifest` reference to a
canonical `.provenance.json` sidecar. The report, retained combined log, and
sidecar are published as one bundle. GVSOC captures also retain the source and
process receipt until collection; the sidecar records the recursive source
snapshot and separate stdout/stderr identities. Use
`validate_gvsoc_mapping_bundle()` to verify an offline GVSOC bundle. Older
reports without a sidecar remain valid legacy artifacts and are not upgraded.

The artifact binds a run to the SHA-256 of
`tests/gap9_sdk_gvsoc_mapping_smoke.c`, the repository-owned C/header and build
inputs in `build.source_sha256`, its built binary hash, compiler/toolchain
identity and flags. GVSOC evidence also binds the SDK commit, release and dirty
state, plus the exact compile database hash. Both host PMSIS and GVSOC artifacts
retain the exact combined process output and bind its SHA-256 and byte count.
For canonical GVSOC reports, the manifest records hashes of the selected model
plugin and launcher files; this does not prove dynamic-loader residency. The
manifest records selected GVSOC environment keys, not every inherited value.
Host PMSIS evidence leaves SDK and target-build fields null; its retained log
does not imply target execution. Fixture counts describe poses, extracted scans
and final map points. `evidence_class` must be `host-pmsis-stubs` or
`gvsoc-target-execution`; these classes are never interchangeable. Neither
class is hardware measurement.

Mapping fields use counts unless named otherwise. `icp_error_milli` is the
reported ICP error multiplied by 1000 (dimensionless algorithm value; it is
not a distance). `correspondences_last_pair` counts matched points in the last
pair only. `accepted_constraints` counts successful stored insertions, while
`optimizer_input_constraints` and `optimizer_unique_edges` count what the
optimizer received. Solver-call and iteration counts describe the final
`pcl_optimize` call; solver calls include successful main and subgraph solves,
with three fixed updates counted per successful solve. `has_nan` and
`icp_has_nan` are explicit non-finite indicators.

`null` means that a producer did not observe or instrument that stage. It does
not mean zero or success. A `passed` artifact must have all mapping-stage
fields observed and consistent, a successful process exit, a seen marker, and
no non-finite values. A failure artifact may contain nulls, but may not promote
them to successful measurements. `reproduced-limitation` records a complete
GVSOC `mapping_fail` marker and nonzero target process exit. It preserves the
exact metrics observed; it is not restricted to one failure signature and
cannot be used when all mapping pass invariants hold. Unknown fields require a
schema version change. Example documents in
`tests/fixtures/gap9_mapping_diagnostics/` are synthetic schema fixtures, not
execution evidence.
