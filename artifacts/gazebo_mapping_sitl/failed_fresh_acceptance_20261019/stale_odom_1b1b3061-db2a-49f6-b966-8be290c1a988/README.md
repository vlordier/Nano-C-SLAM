# Fresh-run failure: stale odometry under system load

The domain-56 fresh session started the mission, but the collector exited 1
with `mission failed: odometry data is stale`. No combined mapping acceptance
report was published. This followed heavy Rust compilation activity elsewhere
on the workstation; that timing correlation is an observation, not a proven
cause. Retained logs show the exact collector and mission output. A later clean
fresh run completed, so keep this as failure evidence rather than treating it
as a persistent mapping defect.
