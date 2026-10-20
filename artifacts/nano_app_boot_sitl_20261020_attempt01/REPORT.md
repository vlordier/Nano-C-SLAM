# Nano app CrazySim boot attempt 01

This run proves that the Nano app is compiled into the Linux CrazySim `cf2`
target, `appInit()` reaches `appMain()`, and an unavailable UWB provider blocks
startup explicitly. It does not prove task-graph readiness, flight, or
mapping.

The native Gazebo Sim 8.15 server accepted model `nano_cf0`; the firmware in
the Lima Ubuntu guest completed its UDP handshake and Crazyflie self-test. The
app then logged:

```text
NCS_SITL_DEVICE_MISSING device=uwb provider=unconfigured node=0
NCS_APP_INIT_FAILED device=uwb operation=init error=-5
```

The app intentionally returned before creating mission, ToF, or UWB tasks.
The stabilizer later started, but the trace records zero command and motor
ratios. The run used a four-second timeout and exited through the normal SITL
shutdown handler (script-recorded child exit 124).

`firmware.log.gz` is the full PTY capture (line endings normalized to LF,
trailing log spaces retained). The decompressed SHA-256 is recorded in
`manifest.json`. This is a successful startup/fail-closed integration test
and a failed flight attempt by design.
