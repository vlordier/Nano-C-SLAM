# Pose-graph constraint accounting

`pcl_add_constraint` and `pcl_add_external_constraint` accept at most one
stored measurement for each undirected pair of pose-table indices. The first
valid measurement wins. A later same-direction or reversed pair, self-edge,
invalid endpoint, full storage array, or non-finite measurement is rejected and
does not change the stored count. The APIs return `true` only when they append
one new constraint. This bounds deduplication work to the existing fixed array
of `MAX_CONSTRAINTS`; it adds no per-edge memory.

Before `pcl_optimize` calls `ls_slam`, it copies the stored constraints and
adds valid external constraints whose canonical undirected pair is not already
in that input. It resets the last-call observation at the start of every
optimization attempt. `pcl_get_optimizer_input_constraint_count` and
`pcl_get_optimizer_input_unique_edge_count` are populated at the `ls_slam` call
site, after graph reduction and endpoint remapping. They report what was passed
to the solver, not successful convergence or objective improvement. The
endpoint accessor returns compact graph-node indices from that reduced input;
stored endpoint accessors return pose-table indices.

`slam_job_t.main_solver_returned` records the main `ls_slam` bool result;
`successful_solver_call_count` includes the main solve and successful subgraph
solves; `completed_solver_iteration_count` counts fully applied fixed solver
updates (three per successful `ls_slam` call); and `optimization_completed`
means `pcl_optimize` reached the end, reconstructed every pose, and passed its
finite-value check. `number_of_poses` is therefore reported as
`finite_pose_count`, not as an optimization or accuracy metric. The replay
validator requires these states to reconcile. The solver computes no
validated objective/residual comparison and exposes no convergence test, so
these fields do not claim convergence or improvement.

The host replay separately records all nonempty adjacent scan pairs passed to
ICP, candidates that passed the ICP result gate and called the insertion API,
accepted ICP insertions, all stored canonical endpoint pairs, and the unique
edges in the final `ls_slam` input. Internal scan-pose loop closures therefore
appear in stored pairs without being misreported as explicit host ICP
insertions. The mapping result validator reconciles pair arrays with counts,
requires accepted ICP pairs to exist in storage, rejects duplicate pairs, and
rejects the legacy `icp_constraint_count` and `optimized_constraint_count`
fields that conflated these meanings.
