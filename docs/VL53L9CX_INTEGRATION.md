# VL53L9CX integration decision and open evidence

## Intended role

VL53L9CX is an additional depth source for Nano-C-SLAM; it does not replace
the four VL53L5CX units already used by the STM32 application. The current
application configures those four devices as 8x8 grids at 15 Hz and exposes
them as four `int16_t` distance matrices in `cslam-stm32-app/src/tof_daq.c`
and `src/tof-matrix/tof_matrix.c`. A 54x42 VL53L9CX frame cannot be silently
coerced into that interface without losing most of its spatial information.

## Datasheet facts checked

The supplied `/Users/vincent/Downloads/DS_vl53l9cx.pdf` is ST datasheet
DS14879 Rev 9 (October 2026). It describes a dToF module with 54x42 zones,
55x42 degree field of view, range from 50 mm to 8.8 m, and up to 100 Hz.
Its package has a 1.2 V digital rail, I/O rails at 1.2/1.8 V, and additional
analog/laser supply requirements; it is not a drop-in breakout replacement.

MIPI CSI-2 is the default output at up to 1 Gbit/s. The datasheet permits
reading sensor data over the control interface in I2C or I3C mode, but says
the achieved frame rate falls with resolution and recommends I3C for higher
bandwidth. The device lists I2C up to 1 MHz and I3C up to 12.5 MHz. A raw
54x42 array of 16-bit depth values alone is 4,536 bytes/frame; at 100 Hz that
is 453,600 bytes/s, before confidence/status/headers and bus overhead. That
exceeds the theoretical 125,000 bytes/s ceiling of 1-MHz I2C, so full
resolution at 100 Hz cannot be an I2C operating point. A smaller/binned frame
or lower measured cadence may still be viable over I2C; the exact mode must be
measured with the official API and actual board.

ST lists STSW-IMG053 as an official capture-driver and postprocessing example
for the STEVAL-VL53L9 evaluation board on STM32N6570-DK, with examples using
different interfaces. The software is request-gated on ST's site. That is
evidence of a supported reference platform, not evidence that the current
Crazyflie STM32F405 deck or our GAP9 carrier is wired or supported. The local
GAP SDK checkout at `/Users/vincent/Work/da3_gap9/third_party/gap_sdk` contains
GAP9 I3C and CSI-2 driver APIs/examples; silicon/SDK support does not prove
the target board routes these pins or supplies the sensor rails.

## Integration decision

Keep the existing VL53L5CX path as the first STM32 SITL sensor contract. Do
not add a pretend VL53L9CX implementation to the current I2C driver or claim
that GVSOC has sensor support. The one-drone Nano firmware flight can use an
explicitly labelled simulated VL53L5CX-compatible frame source so it can
validate the flight-control causal loop. VL53L9CX qualification proceeds as
a separate required sensor workstream before dual-sensor mapping is claimed.

The first hardware decision is where the sensor is physically connected:

- **Crazyflie STM32F405 / existing deck:** the source code currently has an
  I2C transport but no evidence of compatible CSI-2, I3C, rail sequencing, or
  VL53L9CX driver support. I2C may be possible only at a selected, measured
  lower-resolution/lower-rate profile.
- **GAP9:** the local SDK includes I3C and CSI-2 APIs/examples, making it a
  plausible acquisition host. The board schematic, pad routing, PHY/lane
  routing, power and reset wiring, and transfer path into the mapper remain
  unverified. Do not infer those from SDK headers or GVSOC configuration.
- **External capture board/bridge:** ST's STEVAL-VL53L9 plus STM32N6570-DK is
  a candidate for early sensor characterization, but requires a documented
  data bridge into this mapper and is not an onboard drone solution by itself.

The normalized frame contract must retain source ID, width/height, sequence,
sensor and host timestamps, millimetre units, invalid/status and confidence
values, intrinsics/FoV, and calibrated sensor-to-body extrinsics. It must
budget frame buffering and transport on the selected host link. The mapping
acceptance test must compare the two sensors independently and jointly against
held-out geometry; synthetic Gazebo frames and physical sensor captures must
remain distinct evidence classes.

## Beads

- `ncs-fea.5.9.1`: establish actual electrical and signal routing.
- `ncs-fea.5.9.2`: verify official driver and host-interface feasibility.
- `ncs-fea.5.9.3`: define a bandwidth-bounded common depth-frame contract.
- `ncs-fea.5.9.4`: add a synthetic Gazebo source and dual-source geometry
  tests.
- `ncs-fea.5.9.5`: validate concurrent physical sensors and mapped geometry.

The VL53L9CX datasheet does not itself specify the sensor's export packet
layout, selected profile bandwidth, or the target board routing. Those remain
verification tasks rather than assumptions in application code.
