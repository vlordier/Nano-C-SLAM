# Shared mapping diagnostic schema

`mapping_diagnostic.schema.json` defines version 1. Python producers and
consumers should additionally call `scripts.gap9_mapping_diagnostic`
validation because the JSON Schema documents field types while the validator
enforces cross-field invariants.

The artifact binds a run to the SHA-256 of
`tests/gap9_sdk_gvsoc_mapping_smoke.c`, its built binary hash, compiler/toolchain
identity and flags. Fixture counts describe poses, extracted scans and final
map points. `evidence_class` must be `host-pmsis-stubs` or
`gvsoc-target-execution`; these classes are never interchangeable. Neither
class is hardware measurement.

Mapping fields use counts unless named otherwise. `icp_error_milli` is the
reported ICP error multiplied by 1000 (dimensionless algorithm value; it is
not a distance). `correspondences_last_pair` counts matched points in the last
pair only. `accepted_constraints` counts successful stored insertions, while
`optimizer_input_constraints` and `optimizer_unique_edges` count what the
optimizer received. Solver-call and iteration counts are cumulative for the
fixture. `has_nan` and `icp_has_nan` are explicit non-finite indicators.

`null` means that a producer did not observe or instrument that stage. It does
not mean zero or success. A `passed` artifact must have all mapping-stage
fields observed and consistent, a successful process exit, a seen marker, and
no non-finite values. A failure artifact may contain nulls, but may not promote
them to successful measurements. Unknown fields require a schema version
change. Example documents in `tests/fixtures/gap9_mapping_diagnostics/` are
synthetic schema fixtures, not execution evidence.
