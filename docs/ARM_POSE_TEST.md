# Arm pose teaching and jog-speed bench test

Prepared November 11, 2026. Hardware results are pending. Run from the patched
FORGE checkout on the Ubuntu rig. One keyboard window owns the controller.

## Joint setup

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

## Start a session

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
