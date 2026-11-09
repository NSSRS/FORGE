# Arm clearance study — simulation only

This preview sweeps DOF1–6 individually through the full MuJoCo model. It finds
the first sampled collision/clearance or closure boundary in each direction,
then animates the resulting interval. The chassis and other five independently
driven joints stay at the saved `terrain_spawn` pose. D3's passive rod and bearing
are solved at every pose; no physics time advances and no motor transport opens.

## Run and inspect

From the FORGE workspace:

```bash
.venv/bin/python -m pip install -r src/simulation/requirements.txt
.venv/bin/python scripts/preview_arm_sweep.py
```

The native MuJoCo window cycles through all six joints. **Space** pauses/resumes,
**1–6** selects a joint, **R** restarts its cycle, and **C** toggles collision hulls.
Use the mouse to orbit/zoom. Teal marks the active joint body; orange marks the
limiting parts near a boundary. Angles and intervals use model coordinates.

Scan without opening a window and retain the numerical evidence:

```bash
.venv/bin/python scripts/preview_arm_sweep.py --headless \
  --report build/simulation/arm-clearance-study.json
```

Export a 48-second, six-joint MP4, one boundary still per joint and a contact sheet:

```bash
.venv/bin/python -m pip install -r src/simulation/requirements-preview.txt
MUJOCO_GL=egl .venv/bin/python scripts/preview_arm_sweep.py --headless \
  --report build/simulation/arm-clearance-study.json \
  --video build/simulation/arm-clearance-sweep.mp4
```

`--clearance-mm`, `--step-deg`, `--extent-deg` and `--cycle-seconds` configure
the study. Defaults are 2 mm, 1 degree, 180 degrees in each direction relative
to the saved pose, and 8 seconds per joint. `--seconds` closes a native viewer
after a bounded smoke test. Video rendering needs a working graphics driver;
plain headless scans do not.

The rod dimension is 150 mm between connection centers (140 mm bare rod).
The exact center-distance correction and reclosed saved poses are described in
[the model guide](ONSHAPE_ARM_PREVIEW.md#joint-coordinates-and-defaults).

## First candidate results (2026-11-09)

MuJoCo 3.13.0, saved flat-wall scene, 2 mm threshold, 1-degree steps and boundary
refinement to a bracket no wider than 0.1 degree:

| DOF | Reference (deg) | Lower (deg) | Upper (deg) | Lower / upper limiting pair |
| --- | ---: | ---: | ---: | --- |
| 1 | 0.00 | -53.62 | +53.31 | Chassis–tool / chassis–tool |
| 2 | +5.40 | -102.04 | +12.15 | Chassis–D3 rod / steel wall–tool |
| 3 | -25.21 | -59.34 | +55.98 | Chassis–tool / upper arm–D3 rod |
| 4 | 0.00 | -180.00* | +180.00* | No boundary found within the search |
| 5 | -10.77 | -12.40 | +148.92 | Forearm–D5 wrist / forearm–tool |
| 6 | 0.00 | -155.56 | +155.56 | Forearm–tool / forearm–tool |

These are rounded display values; retain the unrounded JSON for reproduction.
Every detected endpoint above is a clearance boundary. Asterisks mark search
endpoints, **not mechanical limits or evidence of unlimited D4 rotation**.
After exact 150 mm center-spacing normalization and saved-pose repair, all
six sampled intervals above remained unchanged at the current resolution.
The reference's nearest eligible pair is the forearm/D5 wrist at about 3.25 mm.
The 2 mm margin is a visual-study setting, not a validated operating margin.
D2's positive bound is environment-dependent because the tool approaches the wall.

## What the study measures

`simulation.arm_sweep` loads `scene.xml`, including the real collision meshes.
It does not use the contact-disabled `arm_preview.xml`. In-memory contact margins
and gaps are enlarged to query nearby eligible pairs without stepping dynamics.
Pairs involving at least one arm body are inspected; unrelated chassis-only
contacts are outside this arm scan. The first non-clear sample brackets a boundary,
which is refined from the last closed, clear pose. Playback checks clearance again
and stops if its interpolated pose fails. Existing authored joint ranges, if any,
also bound the search; actuator preview control ranges do not define geometry.

The JSON records the reference pose, configuration, nearest pairs, closure gaps,
boundary causes, selected source hashes, explicit contact exclusions and omitted
surfaces. Source model files and hardware configuration are never rewritten.

## Limits of this evidence and next work

- These are six one-dimensional slices of configuration space. Combining their
  endpoints can cause collisions; a multi-joint path needs checks along the path.
- Mesh contacts use convex hulls. Concave clearance may be underestimated. Parent,
  welded-body and collision-mask filters remain active. In particular, the model
  excludes `b2_arm_d1`/`l3_arm_linkage_d3` and
  `b4_1_arm_rod_d3`/`b4_2_arm_d3`; those interfaces need separate CAD review.
  Two zero-thickness CAD surfaces remain omitted, as recorded in
  `omitted_surfaces.json`. This study does not claim those interfaces are clear.
- Discrete stepping can miss narrow collision regions; local endpoint refinement
  does not certify continuous motion. Smaller steps are useful sensitivity checks,
  not a substitute for complete geometry/path validation.
- Cables, tolerances, backlash, braking distance, load/gravity and hard stops are
  not established by this kinematic preview. Nominal animation speed is for viewing.
- Model zero/sign/units are not calibrated hardware coordinates. The ±90-degree
  unloaded jog window, Pocket CW signs and production confirmation flags remain
  unchanged. CAL-01 and PLAN-01 stay open.

Next: inspect the highlighted interfaces, improve collision geometry/filters where
CAD evidence supports it, evaluate combined poses and paths, then measure hardware
zero/sign/ratios and confirm conservative limits without deliberately crashing joints.

Implementation references: [MuJoCo collision detection](https://mujoco.readthedocs.io/en/3.13.0/computation/index.html#collision-detection)
and [passive Python viewer](https://mujoco.readthedocs.io/en/3.13.0/python.html#passive-viewer).
