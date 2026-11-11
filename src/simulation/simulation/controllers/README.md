# Coordinated WEAVE_XY_Z = 1 on a 20 mm floor seam

Four simulated ENCOS wheel drives advance at a requested 12 mm/s while the
arm weaves the actual APW503 wire tip from −10 to +10 mm on the centered
floor seam. The target is 2 mm above the wheel-contact plane. DOF1 holds
zero. DOF2–6 track wire position and keep the nozzle-to-wire axis pointing
outward at 45° to the floor. The simulated cart and joints move under actuator
commands, gravity and contact. MuJoCo checks for arm contact during settling,
approach and every step of travel.

## Moving preview

```bash
.venv/bin/python scripts/preview_weave.py --hz 2 --cycles 16 --loop
```

The Windows launcher opens the same controller in native MuJoCo. The live overlay
shows cart travel, lateral wire position, DOF6 angle and wire inclination. Amber is the commanded path
and teal is the measured wire-tip trace. Drag to orbit. Use
`--amplitude-mm 0` for a straight pass. `--plan` explicitly requests a
stationary inspection of the target.

## Gun mounting and reach

The 45° solution assumes the gun is turned **180° about its cylindrical barrel
in the DOF6 clamp** relative to the CAD export. The barrel centerline is at
local Y = 111 mm. The bracket stays fixed to DOF6. The visible APW503 parts,
collision meshes and wire-tip frame undergo the same rigid rotation. The
experiment keeps the exported 0.197 kg tool-link inertia and all stock
actuator ranges. Separate gun and cable mass properties are unverified.

The seam starts at world (−440, 0, 0) mm. The reversed setup starts the chassis
at X = −20 mm, with both the shoulder and seam on Y = 0. The chassis feeds +X
and the wire points toward −X and down. At the center of the weave the arm
reaches straight along the seam in top view. The lateral excursions come from
the wrist and arm.

With the gun mounted exactly as exported, the search found no clear centered
45° start in the nominal command ranges. An alternative with the original
mount starts the chassis at X = +20 mm and uses 74° outward. Run this comparison
with the angle explicit.

```bash
.venv/bin/python scripts/preview_weave.py --gun-mount cad --work-angle-deg 74 --hz 2 --loop
```

## Simulated results

Six cycles per frequency, 20 mm commanded stroke, reversed mounting, 45°
target. Position errors below use the measured three-dimensional wire tip.

| Frequency | Measured cart travel | Measured tip stroke | RMS / peak tip error |
| --- | ---: | ---: | ---: |
| 1 Hz | 70.4 mm | 20.17 mm | 0.037 / 0.166 mm |
| 2 Hz | 34.4 mm | 20.35 mm | 0.095 / 0.319 mm |
| 3 Hz | 22.4 mm | 20.60 mm | 0.176 / 0.474 mm |

Across these runs DOF6 swept about 22.4°, DOF1 stayed within 0.15° of zero,
and the wire stayed between 44.98° and 45.04° to the floor. The modeled
wire tip reached up to 0.31 mm past a seam edge at 3 Hz. The default 16-cycle
viewer command at 2 Hz advances about 94.5 mm before repeating.

## Tester handoff

1. On the rig, compare the bent neck direction, barrel clocking, bracket,
   wire extension, cable clearance and tool mass with the simulated setup.
   Physical fit and loaded response still need measurement.
2. Run the straight pass, then repeat six cycles at each frequency. Compare
   the actual wire direction in side view and lateral extrema in top view.
   Inspect the CSV for overshoot, height, cart travel, wheel signs, D3 closure
   and wrist travel. Record the tolerances accepted for the intended seam.
3. Keep motor pose teaching and speed trials in the [arm bench guide](../../../../docs/ARM_POSE_TEST.md).

```bash
mkdir -p build/weave
for hz in 1 2 3; do
  .venv/bin/python scripts/preview_weave.py --hz "$hz" --cycles 6 --csv "build/weave/${hz}Hz.csv"
done
```
