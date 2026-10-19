# GAP9 synthetic geometry baseline and frozen thresholds

Five independent live Gazebo→GVSOC runs completed the 60-sample square mission, transferred the expected 64-frame optimizer response, and produced 60 target poses. Each run used a separate Gazebo session and the identical archived SDF (`5d2d9cc1…1815a2`). `baseline-summary.json` binds each report to its run, target transfer, ELF/config, active-world attestation, and exact SDF hash, then records min/max/mean/sample standard deviation.

The frozen limits in `geometry-thresholds.json` sit above the observed five-run maxima (or below the observed minimum for the coverage fraction), with rounded headroom. They are regression checks for this deterministic synthetic scene, not statistical confidence bounds. A held-out capture must be evaluated with `scripts/validate_gap9_geometry.py` and this profile without tuning it from the held-out result.

To validate one archived metric report, run:

```sh
uv run --locked python scripts/validate_gap9_geometry.py \
  --metrics path/to/geometry-metrics.json \
  --profile artifacts/gap9_gvsoc_evidence/geometry_baseline_20261019/geometry-thresholds.json
```

The command emits a JSON report containing the profile ID, every observed gate value and threshold, and metric-specific failures. All five baseline rows pass all nine numeric gates plus the mission, route, SDF, and sample-count checks.

The sensor bridge uses idealized simulator-derived four-ray ranges. These measurements do not validate physical ToF/UWB behavior, STM32 flight firmware, multi-drone interaction, or real-world mapping accuracy. The next step is deliberate validator perturbation tests, then a separately archived held-out run.

The per-model coverage aggregate is diagnostic, not an acceptance gate: obstacle3, wall1, and wall4 have zero points within 5 cm across all five runs (the reported per-model fraction is the share of all map points assigned near each model). This is a material limitation of the current route and four-ray synthetic sensor setup. Passing the global surface-distance gate does not mean every wall or obstacle was mapped.
