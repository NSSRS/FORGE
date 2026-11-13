# Arm pose teaching and jog-speed bench test

Updated November 12, 2026. The eight requested raw positions are recorded in
[the current handoff](ARM_SESSION_HANDOFF.md#recorded--initial-positions-and-both-a2a3-travel-ends)
and [the reference JSON](../src/robot_description/config/arm_reference_positions_2026-11-12.json).
Repeatability, model mapping and loaded stopping remain unqualified. Run from the patched
FORGE checkout on the Ubuntu rig. One keyboard window owns the controller.

Manual jogging now uses `bash scripts/control_arm.sh` with no capture panel.
Reference recording is a separate application; model-derived paths and obstacle
retuning use [the reusable calibration tool](ARM_MODEL_CALIBRATION.md).

## Current procedure: initial positions and A2/A3 travel ends

The requested workflow is **A1/A4/A5/A6 initial positions and both CW/CCW ends
for each of A2/A3**, with no notes questionnaire. Use `scripts/record_arm_positions.py --capture-file`
(default `--capture-mode reference`) with group hold. From the repository root,
`bash scripts/record_arm_positions.sh` starts this workflow and writes a fresh
`build/arm_tests/positions-.../initial-positions.json`. It never loads the saved
reference record as motion targets or overwrites the earlier measurement run.

| Motor | Instruction shown in the window |
| --- | --- |
| A1 | Point the arm forward in its initial position. |
| A2 | Save the most clockwise travel end, then the most counterclockwise end; stop short of hard contact. |
| A3 | Save the most clockwise travel end, then the most counterclockwise end; stop short of hard contact. |
| A4 | Align the wrist to its initial position. |
| A5 | Align the wrist to its initial position. **Not the older CW-stop reference.** |
| A6 | Align the tool to its initial position. |

CW/CCW is viewed from the motor's output-shaft side, looking toward its body.
Press H to activate group holding, select 1–6, then E and an arrow to position
the selected joint. Release, Space, wait for **Ready**, then C (or click Capture).
The Position recording panel keeps the **Saved A1 reference: raw …°** result
visible, shows progress across all six motors, and offers **Next: select A2**.
Click Next or press 2, then follow A2's instruction. After saving A2 CW, the
guide changes to **A2 — 2/2 (CCW extreme)**. Press E and jog to that end, then
release, Space, wait for Ready and C again. Next becomes available after both
ends are saved. Repeat the same two steps for A3; A4/A5/A6 each need one save.
There are eight captures total; no automatic movement or selection.
Capturing and selection preserve other joints' holds. The terminal prints the
output filename and each capture is saved immediately in the background.
If the panel says **NOT SAVED**, the position was not recorded: it names the
blocking condition (for example, motion still armed, A4 speed, or stale feedback).
Wait for Ready and retry C. Capture results remain visible through Space,
focus loss and joint selection; no confirmation dialog needs to be dismissed.

Version 2 files declare `capture_steps` for each motor. A1/A4/A5/A6 have
`reference_raw_rad`, `reference_kind: initial` and `initial_raw_rad`. A2/A3 each
have `cw_extreme` and `ccw_extreme` samples, `raw_cw_rad` and `raw_ccw_rad`, plus
sorted `raw_lower_rad`/`raw_upper_rad` after the second save. CW is not assumed
to increase the encoder reading. These are operator-chosen clearance endpoints
with unknown model mapping, not verified hard-contact limits. No A2/A3 initial
pose is inferred, and no zero or production limit is changed. Previous files
remain unchanged; use a fresh run for the eight-capture sequence.
`--fixture-note` is optional. Rest the arm before changing motor applications;
startup, closing, D, faults and power loss can remove torque.

If startup cannot confirm stationary disabled feedback, the window stays in
FAULT and names the affected joint and last drive status/feedback age. Status 9
on the 8009P means undervoltage; inspect motor power, including A1's supply.
After the physical issue is resolved, R retries disabled preparation. Successful
recovery stays disarmed; H explicitly activates all holds. Do not bypass this
check or treat the generic Qt `QSocketNotifier` warning as the motor diagnosis.

## Earlier reference-teaching setup

Support the complete assembly through hand placement, motion and torque release.
Work on one joint at a time. Photograph the fixture, witness marks and the viewing
side used for CW/CCW. Use that same side on each visit. With the drives disabled,
bring stop joints gently to first physical contact by hand. Back off by hand
before enabling. Never seek a hard stop under motor power.

| DOF and drive | Physical reference for C | Initial MuJoCo goal |
| --- | --- | --- |
| A1 base, 8009P | On the new rotating base, align the shoulder to the D3 bearing as shown in the model. Use a witness mark for this axis without a physical stop. | 0 rad (0°) |
| A2 upper arm, 4340P | Most clockwise, at its physical constraint. | +0.0942 rad (+5.40°) |
| A3 driven crank, 4340P | Most counterclockwise, at its physical constraint. Use the driven crank as the reference and let the passive rod follow. | −0.4400 rad (−25.21°) |
| A4 wrist, 4310P | Manually align to the pictured initial wrist orientation and mark it. | 0 rad (0°) |
| A5 wrist, 4310P | Most clockwise, at its physical constraint. | −0.1880 rad (−10.77°) |
| A6 tool rotation, 4310P | Manually align the tool orientation to the picture and mark it. | 0 rad (0°) |

Open the static viewer with `.venv/bin/python scripts/preview_native.py`.
Use **Key 0 → Load key** for the saved pose. The teach UI reads the model's
`initial_pose.json`; D4 and D6 default to the model's zero. The table gives
model coordinates. The raw encoder reading at a physical stop can be any value.
A single stop observation gives no model angle, transmission ratio or sign.

## Guided bounds and initial-position capture with group hold

This optional longer workflow requires **`--capture-mode bounds`** and is not
the operator's current eight-capture task. Use it later for CAL-01's
raw travel endpoints and initial-position measurements.
The older `--teach-file` workflow below records one reference/goal and requires
disabled hand placement; it does not record both travel endpoints.

The recorder prompts for **one usable endpoint → opposite usable endpoint →
initial pose**, separately for each selected joint. Either endpoint may be first;
the saved lower/upper values are sorted by raw encoder angle, not assumed CW/CCW.
Choose endpoints short of contact, cable tension or linkage interference. This
powered workflow records clearance bounds, not the exact hard-stop locations in
the reference table. Do not drive into a stop to discover it. A2/A3/A5 initial
poses must be aligned to the model, not placed against the old reference stops.

Run one controller owner. Before switching from a running jogger, place the arm
in a rest/restraint that carries it with torque off: closing or starting a new
session disables torque. Once active, H and normal selection preserve group
holding; C does not disable or command any joint. A fault, D, Q, window close or
power loss still removes torque. No firmware update is required for capture;
the installed controller must already support the chosen hold/smoothing mode.

From the repository root, after the existing six-joint disabled connection check:

```bash
run="build/arm_tests/positions-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$run"
python3 scripts/record_arm_positions.py --execute --supported --manual-travel \
  --hold-all --smooth-jog --speed-deg-s 10 \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 --capture-mode bounds \
  --capture-file "$run/bounds-initial.json" --report "$run/session.json"
```

The command preserves the operator's requested 10 deg/s holding trial. For an
offline rehearsal replace `--execute` with `--simulate` and omit `--smooth-jog`;
use a different output directory. Files refuse reuse so raw encoder branches
from different sessions cannot silently mix.

1. Press **H** to capture and hold the current positions of all six joints.
2. Select **1–6**. Follow the displayed step: **E**, then hold an arrow to position
   the joint. **Release**, press **Space**, wait at least **0.5 s**, then press
   **C** (or the capture button). Wait for **Saved**. Space disarms movement
   while maintaining holding.
3. Repeat for the opposite endpoint and then the model initial pose. The UI
   displays the model initial angle; the physical alignment is still yours to
   verify. After the third save, select the next joint. No automatic movement
   or replay occurs, and capturing does not change the hold targets.
4. Each save updates `bounds-initial.json` immediately via a background atomic
   write. The file contains `raw_lower_rad`, `raw_upper_rad`, `initial_raw_rad`,
   individual samples in radians/degrees, model goal, route inventory, firmware
   identity, fixture note and every joint's angle at every capture. The session
   report also includes the capture record when closed normally.

Capture requires fresh enabled feedback for every configured joint, no armed
motion, and at least 0.5 s of stationary samples. All angles must settle within
0.2 degrees and reported speed within 0.03 rad/s. Stale/moving/wrap-ambiguous
data, duplicate endpoints and initial positions outside the interval are
rejected. File errors leave holding active and the failed step can be retried.

These are **raw absolute readings in the current encoder branch**, not a
power-cycle-qualified absolute calibration. Bounds depend on the other links'
poses and do not qualify combined paths. No encoder zero, motor setting,
production limit or sign/offset/ratio is changed. Validate repeatability, actual
model alignment and additional poses before using the file for startup or motion.

## Start a reference-teaching session

The six-axis profile uses a **1.5 rad/s measured-speed trip on A1–A6**.
A1's former 0.3 rad/s trip was raised at the operator's request. The normal
jog command remains **5°/s**; this profile change does not qualify faster motion.

1. Use the A2–A6 profile while A1 is disconnected. On the new fixture, once
   A1 has room to rotate with cable clearance, explicitly select
   `src/robot_description/config/arm_rig_a1_a6.json` for **both** commands.
   Expected routing is A1 on CAN1 and A2–A6 on CAN2. Command IDs are the DOF
   numbers and feedback IDs are 17–22. Stop on missing or unexpected IDs.
2. Use a separate teach file for each mounting, viewing convention and robot.
   The file holds per-joint travel in raw shaft radians and is updated at each
   capture. Closing the window writes a separate timestamped session report.
   For a rehearsal use `--simulate` in place of `--execute` in the keyboard
   command and a separate teach file. **Place simulated shaft by hand** lets
   you enter the raw shaft position while disarmed.
3. On hardware run the disabled check; inspect the stationary feedback rows for
   every selected joint before starting the keyboard session.

```bash
plan=src/robot_description/config/arm_rig_a2_a6.json
run=build/arm_tests/pose-$(date +%Y%m%d-%H%M%S)
mkdir -p "$run"
python3 scripts/test_dm_arm.py --all --check --feedback --supported \
  --plan "$plan" --port /dev/ttyACM0 --report "$run/disabled.json"
```

```bash
python3 scripts/jog_dm_arm.py --execute --supported --manual-travel \
  --plan "$plan" --port /dev/ttyACM0 --speed-deg-s 5 \
  --teach-file build/arm_tests/fixture-pose.json --report "$run/teach.json"
```

## Capture, compare and adjust

1. Select the physical DOF with **1–6**. Press **Space** to disarm. Place the
   joint at its table reference, wait for it to settle, then press **C**.
   The host saves the disabled encoder reading. The drive's stored zero stays
   intact. For A1/A4/A6 the first capture establishes the initial goal.
2. For A2/A3/A5, move **inward by hand** at least 0.5 raw degrees, clear of
   contact, let the shaft settle, then press **B**. This records the actual
   inward sign for that encoder. Leave enough physical clearance for the test.
3. On a first visit with unknown stop geometry, hand-align the joint to its
   pictured goal, settle and press **T**. This records the raw travel from
   the reference. If you have independently measured the model coordinate at
   the stop and the signed ratio, you can instead add
   `--stop-map DOF MODEL_STOP_DEG SIGNED_RAW_PER_MODEL` to the launch command
   once per stop joint. The ratio is raw shaft radians per model radian.
   The computed goal is `raw_reference + (model_goal - model_stop) × ratio`
   with angles in radians. Inspect the displayed raw goal and confirm the
   move is inward. Take the hand-teach route when either input is unknown.
4. To test replay, hand-place the disarmed joint at a clear interior start.
   Press **E**, then hold **H**. It moves only that joint, up to 5 raw degrees
   toward its goal per press. Release to disable torque. Re-press for each
   further increment. The UI reports arrival within 0.5 raw degrees.
   Watch the linkage, cables and other joints at each step.
5. Compare the actual pose with the saved MuJoCo view from the same side.
   While armed, each fresh arrow press trims up to 1 raw degree; up/right
   increases the encoder coordinate. Or disarm and correct the joint by hand.
   **Release → Space → T** saves the corrected goal. Re-approach from a
   different clear start and repeat the comparison. Record before/after raw
   angles and physical alignment error in degrees or at a named witness mark.
6. Repeat for every connected joint. After reopening, recovery, or power
   cycling, repeat **C** and, for stop joints, **B**. The UI reuses saved
   relative travel only after those session checks. If power or the encoder
   branch changes mid-session, close and restart. Record repeatability after a
   normal power cycle and after controlled movement while powered off. A
   different reading, direction, physical interference or a jumped target
   calls for a new measurement before any further powered replay.

This teaches one pose. Opposite travel limits, combined paths, gravity holding
and production encoder calibration still need their own measurements.

## Compare A6 response and qualify faster jogging

Close the teach window. Start in a clear interior pose with comparable support
and load. Use the same baseline and test conditions for A4, A5 and A6
(all 4310P). First jog at **5°/s** in both directions. Next try **7.5°/s**,
then **10°/s**, one window and report per rate. These are proposed test rates.
The command below runs one stage; change `rate` and repeat only after reviewing
the previous stage.

Trial mode uses the same target update on every joint. Each tick advances at
the requested rate, allowing 100 ms of target lead beyond measured position
(1° at 10°/s). If the shaft lags, the target stops advancing until feedback
catches up. Missed travel is discarded. The desktop display reuses the
control loop's samples during motion.

```bash
rate=5
python3 scripts/jog_dm_arm.py --execute --supported --manual-travel --speed-trial \
  --plan "$plan" --port /dev/ttyACM0 --speed-deg-s "$rate" \
  --report "$run/jog-$rate-$(date +%H%M%S).json"
```

- Select one joint, press **E**, then hold an arrow. Pick the same short hold
  duration that fits the available clearance at all trial speeds. Repeat both
  directions from comparable starts and record the load, duration and viewing
  side. Release and observe actual stopping. Record a timestamped video or
  independent angle measurements if available.
- Compare requested versus measured speed, response onset, target minus
  feedback angle and stopping motion for A6 versus A4/A5. In
  `motion_samples` inspect the steady part of each hold with the commanded
  direction. Repeated `lead_limited` samples show where feedback constrained
  target advance. Check `motion_summary` for peak tracking error, sample gap and
  feedback age. Review rotor/MOS temperatures, torque feedback and any trip.
  `motion_stop` records host stop request to stationary acknowledgement and
  travel from the last sampled position. Video or an independent measurement
  is needed for physical stopping distance from key release.
- Set acceptable error, release distance and temperature for the fixture
  before increasing speed. Keep the last acceptable rate per joint. Stop the
  trial on late feedback, persistent lag, overshoot, unexpected movement,
  excessive temperature or a fault. Save the report and fault evidence;
  compare A6 at 5°/s again before considering drive tuning.
- Return the fixture photos, teach file, disabled check, per-rate reports,
  and a row per DOF with raw reference, raw goal, achieved pose/error,
  CW/CCW viewpoint and repeatability. Mark untested axes and rates explicitly.
