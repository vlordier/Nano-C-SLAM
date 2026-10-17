# Host replay scan observability

The host replay emits the coordinates produced by the owned `pcl_extract_scan`
function for non-overlapping 20-pose windows. These coordinates use the
firmware projection convention and the inclusive `measurement <= 1400 mm` gate.
Incomplete final windows are retained and evaluated with their actual pose
count. This is a host-only diagnostic over the four cardinal Gazebo rays
projected into the central ToF column; it does not establish sensor hardware
quality or map accuracy.

For each window the report records point count, exact unique-coordinate count,
axis-aligned x/y span in meters, and the major/minor eigenvalues of the
population covariance matrix in square meters. `observable_2d` is true when the
minor eigenvalue is at least `0.1 m²`. The eigenvalues are computed from the
emitted C coordinates, and the validator recomputes every metric before
accepting the report.

The threshold separates the retained near-field capture from two deliberately
degenerate controls in the host tests: a collinear point set and repeated
coincident points both have a minor eigenvalue of `0 m²`; a four-corner
non-collinear control has `0.25 m²`. The retained 170-pose capture's nine
windows range from `0.1686` to `0.4968 m²`, so its minimum is above the selected
threshold. This fixture-derived rank/spread check rejects those degenerate
examples. It does not test robustness to noise, registration accuracy, or map
completeness. Those remain separate validation work.
