# Heldout GAP9/GVSOC geometry attempt 09

This fresh live run evaluates the scan-constraint search fix against the same
heldout route and SDF used by attempts 03, 05, 06, and 07. The frozen profile
was not modified.

Run `eeef7b00-fc76-44df-b97f-d1576b59d1c6` completed with 60 paired samples,
64 live SPI frames, three registered scan anchors, a `COMPLETE` mission, and
zero child exit codes. Session `3c38ad9d-8839-40f5-b4c5-f310aa7ab04c` is bound
to the archived SDF SHA-256
`20eb690d09ca1e2b2974a4adaf345bc308b0eba6be12cfeb70e4e27b554eda7b`.

## Observed graph change

Attempt 07 logged only ICP candidate `40 -> 0`: it passed the gate, and the
function returned before trying the next existing anchor. Attempt 09 logs all
three candidates, all accepted:

```text
from=20 to=0 distance_mm=140 ref_points=80 cur_points=70 accepted=1
from=40 to=0 distance_mm=295 ref_points=80 cur_points=74 accepted=1
from=40 to=20 distance_mm=265 ref_points=70 cur_points=74 accepted=1
```

The target confirms stored pose edges `[20->0, 40->0, 40->20]` and optimizer
edges `[1->0, 2->0, 2->1]`, a cycle over all three reduced nodes. This proves
the early return hid a valid accepted constraint; it does not prove the
additional edge improves pose accuracy.

## Frozen gate result

The run still **fails** all-pose XY RMSE: `0.0422543510 m` against the frozen
`0.03 m` threshold. XY maximum is `0.0552157 m` and passes its `0.06 m` limit;
yaw RMSE is `0.00850994 rad` and passes. The target map has 224 points and all
map geometry gates pass (surface median `0.0208678 m`, p95 `0.0474881 m`, max
`0.0531324 m`, 98.21% within 5 cm).

The closure run cannot be compared as a paired experiment to attempts 06 or
07: each was a separate Gazebo mission with different telemetry. Their XY
RMSEs (`0.0415352 m` and `0.0428830 m`) are close to this run and do not show a
clear improvement from the extra edge. Scan anchor 20 remains the largest
source of error (`0.0552157 m`). The root cause of the pose gate failure
therefore remains open.

The input uses idealized simulator-derived four-ray ranges. This artifact does
not validate physical ToF/UWB behavior or STM32 flight firmware.
