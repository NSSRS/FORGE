# Model-guided arm calibration

The standalone tool derives a selected joint's reachable path from **all six
current raw readings**, the raw-to-model mapping, model collision geometry and
D3 closure. It never moves to the saved simulation pose on startup. Normal
manual controls have no reference-capture panel.

## Entry points

From the workspace root, with every link supported before changing applications:

```bash
bash scripts/control_arm.sh
```

This is the existing 10°/s manual hold/jog workflow, with 1–6 selection, H, E,
arrows, Space pause/hold and D disable. It has no capture sequence.
Close the previous owner before opening another; the agent must not interrupt
a running owner. Closing, D, faults and power loss remove torque.

The original eight-position recorder remains available separately:

```bash
bash scripts/record_arm_positions.sh
```

Its Python entry point is now `scripts/record_arm_positions.py`; capture flags
are not accepted by `scripts/jog_dm_arm.py`.

## Model-guided workflow

Install the optional model/triangle-collision dependencies once:

```bash
.venv/bin/python -m pip install -r src/simulation/calibration-requirements.txt
```

Offline rehearsal, or calculation without a window:

```bash
.venv/bin/python scripts/calibrate_arm.py --simulate
.venv/bin/python scripts/calibrate_arm.py --plan-only --joint 1 --extent-deg 30
```

The demonstration pose is only used offline. `--raw-deg A1 A2 A3 A4 A5 A6`
can supply a different offline pose; hardware rejects that option.

After closing the previous owner on mechanical support:

```bash
.venv/bin/python scripts/calibrate_arm.py --execute --supported
```

1. Select a joint with 1–6. H explicitly captures the measured positions and
   holds all joints. It does not command an initial pose or guessed zero.
2. P calculates both directions from fresh feedback. Calculation runs in a
   separate process; it does not own the controller, renew its motion lease or
   start a movement when finished. The panel shows raw/model endpoints and the
   limiting model contact, configured branch, or search horizon.
3. `[` / `]` (or the two Run buttons) starts one explicitly selected path.
   Calibration defaults to 2°/s, configurable up to 5°/s. Each commanded
   waypoint is at most 1 raw degree. Other joints keep holding.
4. Space or Escape pauses and holds, including while a text field has focus.
   Focus loss also pauses. Pausing invalidates an active path; use P and a fresh
   Run action to continue. There is no automatic reverse, resume or axis advance.
5. If real equipment obstructs a path, pause before contact. Once the support
   carries every link, D disables torque. Hand-position to the observed constraint,
   choose **lower/upper raw angle**, enter its cause and click Record (C).
   Recording requests fresh disabled/stationary feedback through the existing
   owner, saves all six raw readings, and never enables torque.
6. Back away by hand from the constraint. H and P then calculate a new path with
   the saved constraint and its backoff. Never force a resistant joint or use
   powered contact to find a stop. Space is powered position hold, not a brake.

The software checks complete fresh feedback, the start pose, other-axis drift,
the planned selected-axis corridor and progress timeouts. The existing MCU
watchdogs, speed/torque trips, latched faults and explicit recovery remain.
R recovery never resumes a path. Plan errors do not authorize motion; feedback
or control errors use the existing fault/disable policy.

## Change gear and rerun

`src/robot_description/config/arm_calibration_candidate.json` is the current
candidate mapping, not a production calibration. Each axis uses
`model_deg = model_per_raw * raw_deg + model_intercept_deg`, plus an explicit
continuous raw encoder branch. ±180° is not silently normalized. The A2/A3
endpoint-derived maps and A1/A4/A5/A6 original initial anchors are retained;
the arbitrary power-cycle test positions are not new anchors.

For new mounted geometry, add its collision meshes to the model and rerun with
`--equipment-revision new-mount-name`. Use `--scene path/to/scene.xml` for an
appropriate modeled environment. The default `robot.xml` checks robot/self
geometry without assuming the simulation's steel wall is present on the bench.
Cables and obstacles missing from the model still need operator observation.

If mounting changes encoder alignment or external gearing, copy/update the
mapping file and pass `--mapping path/to/mapping.json`; update sign, intercept,
ratio and raw branch from known physical/model references before executing.
Do not substitute session Relative° for Raw°. The optional `simulation_model_deg`
values are demonstration coordinates only.

Each run writes a new versioned JSON file under `build/arm_tests/`, or the path
given by `--output`. Existing outputs are never overwritten on startup.
Constraints, model/mapping fingerprints, calculated paths and the final session
report are saved; disk fsync does not block motor refresh.

```bash
.venv/bin/python scripts/calibrate_arm.py --execute --supported \
  --load-constraints build/arm_tests/calibration-PREVIOUS.json
```

Reuse requires an identical model, mapping and equipment revision. A changed
model/mounting starts a new record; simulation constraints cannot authorize
hardware movement. Saved physical constraints narrow paths with angular backoff;
they do not certify all combinations of joint positions.

## Geometry and scope

The planner checks the whole arm at its measured pose and projects D3 onto its
150 mm closure. Default clearance is 2 mm; raw/model sampling is at most 0.25°
per sample, followed by 2° of angular backoff from the last clear point. These
are configurable commissioning margins, not measured stopping distances.
The two explicitly identified adjacent A2/A3 stop pairs are also checked with
triangle geometry (default 0.1 mm clearance), even though the default MuJoCo
parent-body filter omits them. `adjacent_stop_pairs` can be extended for new
modeled stops. Other model collision masks/exclusions remain in effect.

Only the selected one-axis path is authorized. The other five coordinates must
remain near their planned values; ranges from different poses are not combined.
A search horizon means no boundary was found within that search, not a physical
stop. Collision sampling, model fidelity, encoder mapping, physical stopping and
loaded holding still require commissioning evidence. The A3 saved initial-pose
overlap remains unresolved and is not used as an automatic hardware target.
