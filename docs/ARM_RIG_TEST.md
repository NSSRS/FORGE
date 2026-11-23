# Supported arm rig controls

The current fixture includes all six motors, including reconnected A1. Use
`bash scripts/control_arm.sh` for the six-axis manual interface and the
[current handoff](ARM_SESSION_HANDOFF.md) for firmware and evidence. The optional
A2-A6 subset below is for an explicitly disconnected A1 only.
For reference capture, model-pose replay and
opt-in speed trials, use the [next bench test](ARM_POSE_TEST.md).
The inventory remains A1/CAN1 and A2–A6/CAN2 with command
IDs 1–6 and feedback IDs 17–22. `arm_rig_a2_a6.json` selects physical joints
2–6 without renumbering IDs or MCU slots. In that subset only, A1 is unavailable. Another missing
motor fails the check; the code never silently drops a joint or clears faults.

The subset procedure is a disabled connection/encoder check followed by optional slow manual
jogging. It is not gravity holding, homing, joint calibration, or an automatic
collision-limit test. Model sweep angles are not sent to the motors.

The [source and artifact map](SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map)
lists the owning files and report/log folders. In commands below, replace
`/path/to/FORGE` with your checkout root.

Manual-only six-axis controls: `bash scripts/control_arm.sh`. Position recording
is a separate application (`bash scripts/record_arm_positions.sh`); model-guided
calibration and obstacle retuning are in [ARM_MODEL_CALIBRATION.md](ARM_MODEL_CALIBRATION.md).

**Current rig profile (Nov 19):** A2/A3 feedback TMAX is 40 Nm and A4-A6 is
12.5 Nm. Use the [expanded-profile readback and weld commands](#expanded-gun-load-profile-a2a3-30-nm-a4-a6-10-nm).
The baseline-profile examples elsewhere in this guide describe the original
ranges and will fail range matching on the commissioned rig. For its manual
launcher, explicitly select the matching trial profile:

```bash
bash scripts/control_arm.sh --plan src/robot_description/config/arm_rig_gun_a2_a6_load.json
```

This selects the same authorized trial trips; it does not qualify loaded manual
holding. Do not bypass a range mismatch in other diagnostic/calibration tools.

## Independent encoder-to-model mirror

Use this separate diagnostic to compare physical joint motion with raw encoders
and the candidate model mapping before attempting recovery. The current six
directions are all `-1`; their offsets/signs still need independent physical
validation. A collision in that mapped pose is not proof of physical contact.

With the base secured and the catch carrying the links on torque loss, close the
other arm application yourself and leave all six motors disabled and stationary
for connection. Only one process may own `/dev/ttyACM0`:

```bash
cd /home/forge2/forge2_ws
.venv/bin/python scripts/mirror_arm.py \
  --port /dev/ttyACM0 --disabled-refresh --supported
```

This mode verifies IDs/ranges while disabled, configures the MCU's observation
routes, then sends disable frames at about 10 Hz to obtain new feedback. It never
enables motors, sends motion targets, writes motor parameters/zeros or resets a
fault. The motors do not continuously stream fresh feedback while disabled;
`--port` without `--disabled-refresh` only inspects the cached state, which may
be stale and has unverified route labels. Active/faulted controllers are refused.
There is no handoff from a powered owner and no automatic recovery.

A5's current endpoints are cable-constrained. Both ends and an independent
straight pose were recorded on Nov 16; see the [current handoff](ARM_SESSION_HANDOFF.md).
Use this procedure when the cable arrangement changes again.
They are **not** the CAD mechanical stops and cannot identify the model zero.
No angle entry or manual view-offset adjustment is needed:

1. With torque disabled, gently place A5 at the operator-observed CW usable end,
   without tensioning the cable, and click **Capture A5 CW limit**.
2. Place it at the CCW usable end and click **Capture A5 CCW limit**.
3. Extend the A5 wrist straight, with the forearm roll shaft (A4) and tool roll
   shaft (A6) pointing along the same line, rather than folded back, and click
   **Capture A5 straight pose**. The gun is not needed for this reference.
4. Move away from straight to compare the candidate with the physical arm.

Each capture requires seven fresh, disabled, settled six-axis samples over at
least 0.5 seconds. Both cable ends define a raw travel interval with a provisional
2-degree inward command margin. Only the independent straight pose fits A5's
offset; the model reference is computed from the A4/A6 shaft axes (about +93.17
model degrees in this CAD). Cable span is never used to fit gain or model zero.
The margin and candidate alignment still require physical qualification.

Observations are saved under `build/arm_mirror/a5-capture-*/observation.json`.
Only a consistent complete CW + CCW + straight set exports `mapping.json` and
applies the candidate to the viewer. A partial/invalid set preserves the original
view mapping. Recapturing either endpoint after a completed pair invalidates the
opposite old endpoint; record both again. Each new window starts a fresh recording
round. A candidate may be reopened explicitly with `--mapping PATH` for inspection.
The other five axes, A5 gain and source files remain unchanged. This does not
install a welding map: review independent poses and regenerate raw standby targets
and recovery/weld plans before adopting it. The earlier single-CW mechanical-stop
fit is obsolete for the cable-constrained setup and must not be used.

A4 has a separate fixed cable envelope: **CW 270 degrees and CCW 270 degrees from
the preserved initial raw angle +173.183435236 degrees**, as specified by the
operator on Nov 16. The resulting absolute raw interval is approximately
**[-96.816565, +443.183435] degrees** (540 degrees total). This is not a new zero or
an allowance renewed at launch. Manual sessions configure this interval in the
existing controller; holding-mode outward jogging reaches a bounded target and
keeps the existing hold until the operator reverses. Planning checks spline
extrema as well as sampled poses; no modulo-360 or equivalent-angle substitution
may cross a boundary. A changed cable installation or encoder reset needs review
of this fixed reference. Physical stopping and cable-clearance qualification are
still open; this envelope is not a model of the cable throughout every arm pose.

For general inspection:

1. Compare the resting physical arm with the 3D view. The model includes the gun
   even when absent physically. **Focus** selects Arm, Wrist or Robot; camera
   controls and **Show chassis** change only
   the view; the physical table/catch is not modeled.
2. Click **Set delta baseline**, then move one disabled joint gently within known
   free travel. Compare **Raw deg**, **Delta raw deg**, **Model deg** and the
   corresponding model link. Do not force a joint against a stop. The baseline
   only starts a delta measurement; it never changes encoder or model zero.
3. Optional expert inspection: **View gain**
   and **View offset deg** are temporary hypotheses for this window only; they
   never alter the production mapping and are not needed for A5 capture. **Reset view mapping** restores the
   full-precision candidate. Raw readings are never wrapped, clipped or multiplied
   by the internal gear ratio. Out-of-branch values are shown with warnings.
4. Add a physical observation and click **Save snapshot**. Reports go to
   `build/arm_mirror/pose-*.json`, including raw radians/degrees, source age,
   mapping/fingerprint, model angles and baseline deltas. These are observations,
   not new calibration anchors or qualification. Check the age/status: a stale
   or historical pose is not the current arm pose.

The renderer directly sets the six mapped drive angles and solves the D3 passive
linkage without physics or welding IK. Contacts do not hide this diagnostic pose;
an infeasible D3 closure withholds the image. This does not relax any execution
collision checks. `--simulate` gives offline raw-angle controls; `--report PATH`
shows a saved six-axis feedback report without opening a controller. Both are
explicitly labeled offline. Code and interface text remain English.

## Weld panel with embedded model preview

The panel defaults to `--tracker pid` for bounded host PID position tracking and
velocity lead. Use the same reviewed mapping, standby and launch speed arguments;
an already running panel does not reload changed code. **Diagnostics & evidence**
shows **Tracker: PID + velocity feedforward**. **Targets & response** shows
A1–A6 actual Hz and travel; expand **Frequency & travel details** for comparison
with planned peak-to-peak travel.
It labels a half-weave-frequency pattern as **Two-cycle response**; other rates
are shown relative to the weave. Individual joints may have harmonics of that
weave, so this ratio alone is not a tracking-error score. It updates every 0.5 s
after at least three repeatable cycles; unavailable or paused measurements show
a dash. Requested and planned weave Hz are labeled separately.
Measured motor Hz and peak-to-peak travel appear after several welding cycles;
“insufficient motion” is expected when an
axis moves by only a few encoder counts. Frequency alone does not prove the full
commanded travel. `--tracker paced` explicitly selects the previous controller
for comparison. Record the journal and physical response; PID software tests do
not qualify stiffness, smoothness or five-second recovery.

Use the matching reviewed mapping and standby files. From the repository root:

```bash
.venv/bin/python scripts/control_weld.py --simulate --arm --profile weld \
  --cycles 1 --prep-seconds 5 \
  --mapping PATH_TO_REVIEWED_MAPPING --initial-pose PATH_TO_MATCHING_STANDBY
```

The complete Nov 16 A5 capture is a separate candidate; do not substitute the old
default A5 target. Confirm alignment away from the straight reference and ensure
fresh raw positions are inside its two-degree cable backoff before physical
planning. Reopening the panel does not require repeating that capture.

If A1 reports the same independently verified physical orientation on a different
encoder turn after restart, select that branch explicitly with
`--a1-encoder-turns 1` (raw readings +360 degrees relative to the saved mapping)
or `--a1-encoder-turns -1` (raw readings -360 degrees). For example, raw +163.1074
corresponds to the old -196.8926 branch; selecting +1 also moves the saved A1
standby from -174.6916 to +185.3084 raw degrees. Check the physical pose and cable
state before selecting a branch; the software cannot infer turn history from a
single reading. A full physical revolution is not an encoder restart correction.

The branch is fixed for that session and shown in the panel/journal/report.
Feedback stays in the motor's actual raw coordinates; calibration intercept,
raw bounds, standby and planned targets translate together. The saved calibration
and standby files are unchanged, their original fingerprints must match, and
old search solutions need regeneration for the selected session mapping. A4's
fixed cable reference and every other joint remain unchanged. An unexpected
branch change during operation is still rejected; this option does not clear
faults, change motor zero or qualify encoder retention across power cycles.

Both profiles default to 5 deg/s preparation. The panel names the active waypoint
or weld-start/end goal, reports reference tracking separately, and continues
reporting fixed-target hold error after arrival. The 3 deg tracking trip and
0.25 deg stationary arrival threshold remain. `--prep-seconds` is a total
preparation goal, not a timeout or playback speed. The operator accepted the
Nov 23 8.22 s run under its 10 s goal; the earlier five-second goal is retired.
Add `--optimize-preparation` to explicitly
enable checked staging on hardware. It can replace the saved standby with a
shorter intermediate pose, while retaining the full weld target and all path/
motion checks. Diagnostics show **Route: optimized staging**.
Use `--max-speed-deg-s 10 --prep-seconds 5` for the speed and goal of the recorded
five-second comparison; other starts and physical tracking may take longer.
See [route comparison](WELD_MOTION.md#checked-preparation-route-comparison).
The preparation request now accepts up to `--max-speed-deg-s 100`; the existing
motor guard still limits actual commands to 68.7549 deg/s. This changes the
requested speed allowance, not acceleration, tracking, clearance or overspeed
protections. A reported clearance fault needs pose/path diagnosis independently
of the requested speed.

For the Nov 17 direct-approach floor fault, add `--approach-lift-mm 30`. This
requires a waypoint with the tool 30 mm above the weld point, followed by a
checked lowering route to the original weld start. It does not increase the
final wire gap, change mapping, or remove the gun/floor check. Optimization must
preserve the requested lift; if it cannot find that route, planning fails rather
than silently choosing the direct joint blend. The panel identifies the lift.
Preparation time depends on the starting pose and checked route; it can exceed
the selected goal, and measured settling can extend it further. This is an alternative for supported operator testing,
not physical qualification or a guarantee for arbitrary joint lag.

The subsequent Nov 17 disabled/readback check passed and the operator reports
normal manual A5 operation. Do not repeat that check solely because of the older
shutdown report. With the fixture carrying torque loss, close the manual owner
before launching the weld panel with the matching candidate mapping/standby,
`--optimize-preparation --max-speed-deg-s 100 --approach-lift-mm 30` and the verified
A1 encoder branch. Calculate from fresh feedback, review the raised route, then
explicitly start recovery. Leave welding stopped at ready for the hold acceptance
below. A fresh tracking or clearance failure needs its new report, not weaker trips.

The default **Load & temperature** tab shows signed torque, absolute load versus
trip, motor/MOS temperatures, cached sample ages, 30-second display-rate trends
and stage/run torque peaks. Hover a card for temperature peaks. Missing or stale
values show a dash. **Diagnostics & evidence** contains the complete controller
message, first-fault snapshot, profile definitions and journal path. The first
fault remains visible after an explicit reset. The manual arm panel shares the
telemetry tab. The status banner names operator, focus and heartbeat pauses;
calculation time and measured preparation time are separate.

No report argument is needed for normal launches. The latest launch is always
available at `build/weld/latest/`: `session.jsonl` is the live journal and
`session.json` is written after close with shutdown results. Each launch gets
its own retained directory under `build/weld/sessions/`; restarting also preserves
an interrupted run's journal. The fixed `latest` link changes at launch, while
each writer keeps its original archive path. The terminal/panel show that archive
path for unambiguous analysis. An optional `--output` still selects a custom,
unused report path.

The journal can be read while the owner remains active and records cached data
only. A bounded writer reports lost records or disk errors.

An **Unavailable** axis has no valid cached feedback. If any axis lacks valid
feedback, the current-pose view is unavailable too; retained encoder numbers
must not be interpreted as a measured six-axis pose. A startup disable-acknowledgement
fault blocks planning and motion. Reset still requires every retained motor's
disabled, stationary acknowledgement and cannot repair a missing reply by itself.
**Cached** marks historical idle/fault samples. Output selection alone does not
start live polling, so waiting for recovery need not show a stale-feedback fault.
The original 80 ms freshness guard remains active during holding and motion.

Select **Arm output**, then **1 Calculate path**. The embedded
**Current pose** view displays cached encoder feedback in the model, with sample
age; **Standby target** shows the target. Choose **Overview**, **Side**, **Wrist**
or **Contact**; Contact centers the camera on the first blocking or refined pair.
**Contact parts** hides every other part, colors the two original meshes blue
and orange, and marks their nearest surface points in white. It changes only
the rendered view; the full robot remains in every clearance check.
Selection/calculation refresh disabled feedback through the same owner when no
hold exists. Display updates never send CAN requests or renew a command lease.
In hardware mode, Current pose accepts measured angles outside the saved pose
interval and simulator joint ranges, retaining self/environment collision warnings.
Calculate uses that measured start without changing the saved calibration.
Recovery can include the extra angle extent needed to return into the normal
range, but cannot command farther outward. A4/A5/A6 cable envelopes, protocol bounds,
freshness and start-drift checks remain. All subsequent sampled path poses must
pass collision checks, and the temporary extent ends before approach/welding.
A reported starting contact does not permit moving through that contact; an
unavailable collision-free route remains blocked. Simulation keeps its pose limits.
There is no new launch flag. Readings are never wrapped or clamped. An impossible
D3 closure still withholds the pose image. Blocked actions are saved in the journal, including
the planning-start readings when available, even when no motor was enabled.

A6 remains limited to **+/-270 degrees from its saved initial reference**, raw
**+64.161048535 degrees**. Its fixed raw interval is
**[-205.838951465, +334.161048535] degrees**. This applies to manual jogging,
recovery and welding; restarting never moves the reference. No new launch flag
or firmware flash is needed. A shorter-looking full-turn alternative is rejected
if it crosses this interval; for example, the Nov 23 start at -169.927 degrees
cannot use the -296-degree equivalent of the +64-degree standby.

The reviewed A3 upper endpoint has a **0.25 degree startup measurement allowance**:
raw readings up to **151.64 degrees** can qualify for a separate inward-only
retreat to **149.39 degrees**. The same retreat can handle the nearby modeled
A3 stop contact inside the saved interval. Calculation checks the original
triangle meshes, rejects ambiguous/deepening contact or overlap deeper than
0.15 mm, and checks the full retreat before ordinary staging. Other collisions,
D3 closure, normal target bounds, cable travel and explicit A1 encoder-branch
selection remain checked. The hardware measured-start policy above is separate
from this contact-retreat exception. This is a provisional model policy, not a new
physical travel limit or confirmed encoder-drift diagnosis.

When applicable, the accepted plan and animation explicitly show **A3 inward
backoff** first. **Start recovery** verifies the fresh six-axis pose, enables
current-position hold through the same owner and verifies stationary feedback.
Only A3 then retreats, at **at most 1 degree/s and 5 degrees/s²** (or a lower
checked limit); the other joints retain their measured hold targets. PID lead
is omitted for this stage and outward target changes are rejected. Actual
outward excursion beyond the fixed 0.25 degree measurement budget faults; the
budget never follows later outward readings. Arrival still requires 0.25 degree
accuracy and 0.2 s of stationary feedback, plus the normal geometry checks.
The exception ends before ordinary recovery/approach and welding. Pause retains
a measured hold and requires recalculation; disable/fault/close retain their
existing behavior. A larger discrepancy or failed separating-contact check stays
blocked. No calibration file or drive parameter is rewritten.

The journal records `endpoint_recovery_planned`, `endpoint_recovery_started`
and `endpoint_recovery_completed`, and a separate `endpoint_recovery` sample
stage. The HOLD-01 analyzer reports total time to ready, retreat time and normal
preparation time separately. `normal_preparation_to_ready_s` starts after hold
verification, or after retreat when present; it excludes both startup steps.
Compare the same stages and starts
between no-EE and loaded runs; a retreat in only one run is not payload-induced
preparation delay. Software/model verification does not qualify loaded recovery.

After a plan passes, **Play preview** animates recovery, approach and welding
without enabling or commanding motors. Use the slider and 1x/2x/4x playback.
Calculation and preview continue when switching windows; an unused accepted
plan stays available. If **Play preview** is disabled, wait for calculation to
finish or resolve the displayed blocking/cancellation reason and calculate again.
**Start recovery** is the separate physical action; it checks the fresh start,
verifies hold, then moves to standby and the weld start. **Start welding motion**
is a further explicit action. At `ready`, Space/Escape, switching windows and a
lost UI heartbeat preserve the fixed hold and checked plan. **3 Start welding**
stays available; return to the panel and click it when ready. A fresh start still
requires stationary feedback within 0.25 deg of the weld start and valid clearance.
During travel, Space/Escape, focus loss or a lost heartbeat pause motion and
require a fresh calculation and explicit restart. These keys also work when a
preview control has focus. Queued starts and an enable in progress are cancelled
even if focus immediately returns. Closing/disable/fault removes arm torque.

The required normal operation is continuous welding until the operator clicks
**Pause / Hold (Space)**, including when
the launch command contains `--cycles 1`. The cycle count describes the checked
plan and preview block; the panel repeats its closed path without a dwell or
motor disable between blocks. Selected chassis feed continues across the boundary.
Pause stops feed and captures a fixed arm hold. Existing focus, heartbeat and
fault stops remain active. Do not include `--single-pass` in normal launch
commands: it executes the planned cycle count once and then holds. Use that
option only when the operator explicitly requests a finite run. Expand the
panel's diagnostics and confirm **continuous until Pause / Hold**; a
**one planned pass** label means the
process was launched in finite mode. Increasing `--cycles` does not correct a
finite launch; remove `--single-pass` from the next launch command.

After addressing a fault, use **Reset motor fault (R)** in this same panel. It is
available only for a connected, faulted arm, including a retained startup latch.
The existing owner verifies disabled/stationary feedback and saves the original
fault in a unique `.fault-*.json` alongside the session report before resetting.
No second owner is opened. A failed reset leaves the fault visible for diagnosis
and retry. Success leaves all motors disabled, clears output selections and
discards the old plan; select outputs, calculate and explicitly start recovery
again. Commands entered during reset are discarded. This button never resumes
welding or energizes motors automatically.

A blocked plan remains blocked: inspect the current/target views and numbered
screen-space contact rings, which remain visible through occluding surfaces.
Red means the clearance check failed. Passed mounting-gap labels, rings and leaders
are hidden in **Overview**, **Side** and **Wrist**, including path playback.
Only the explicit contact views show blue diagnostics for the reviewed concave
pair's triangle-mesh gap passing despite overlapping convex envelopes. Labels identify
the parts and show both mesh and hull distances; these are model measurements,
not sensors. The gun remains modeled even while physically absent.
The preview uses a fixed chassis and kinematic posing, not gravity simulation or
a camera view. It does not include an unmeasured table/catch geometry.

For offline reproduction, use `--simulate`. Add `--start-report PATH` to initialize fake shafts from a
saved six-axis disabled diagnostic; hardware rejects this option. Without it,
offline simulation starts at the configured mapping's simulation pose.
`--no-preview` disables rendering if the graphics backend is unavailable.

## Weld motion accuracy test

The Nov 23 operator session provides a joint-diagnostic and encoder-reconstructed
tool-pose baseline; see the [review](ARM_SESSION_HANDOFF.md#accuracy-01-and-speed-01-review)
and [complete offline archive](measurements/weld/2026-11-23/README.md).
Prioritize wire-tip position and stroke in the workpiece frame. On a continuous
arm-only pass, allow one fixed time shift for position scoring. Keep same-time
tool orientation as a separate secondary criterion with a **0.5 deg target**
and **1 deg maximum**. Collect independent synchronized measurements before
setting the tip-position tolerance in mm. Use joint errors as diagnostics.
For a repeat or a matched tracking comparison, use the supported fixture and validated
arm profile, mapping and matching standby from the
[weld panel procedure](#weld-panel-with-embedded-model-preview). Perform a
motion-only test. Use the same reviewed hardware launch command with
`--cycles 4`, without `--single-pass`. Reports are saved automatically under
`build/weld/latest/`. Start with **Arm output** selected.
For the A1 comparison, run two arm-only passes at the archived 1.730 Hz and
20 mm stroke with the same 1.96 kg payload, checked path and settings. Use
`--cycles 1` as in the archived run. Use the standard PID tracker for the
first pass and add `--weld-a1-shaper-only` to the arm-only launch for the
second. The trial retains command shaping and disables
the additive A1 correction during welding. Save both session paths.
For a later combined run, verify the four-wheel mapping before selecting
**Chassis feed** and use the normal explicit chassis launch flags.

Calculate and inspect the checked path. Start recovery, wait for Ready, and
start welding motion. After the desired observation interval, click
**Pause / Hold**; the cycle count does not end the run automatically. Then close
the panel with the fixture supporting the arm. The panel writes `session.jsonl`
and the final `session.json`. Analyze the journal from the same model checkout:

```bash
.venv/bin/python scripts/analyze_weld_accuracy.py \
  build/weld/latest/session.jsonl \
  --fit-tip-lag --orientation-target-deg 0.5 --max-orientation-deg 1.0 \
  --output build/arm_tests/accuracy/accuracy.json
```

Use a fresh session for a repeat or select an existing pass with
`--run-number N`. The report covers the moving weld interval and lists RMS and
peak tracking error on A1–A6, measured and planned joint travel, feedback ages,
and the same-tick difference from dispatched targets. It reconstructs the
tool position and full 3D orientation with the checked model and fixed base.
Orientation error is the smallest rotation between measured and same-time planned
tool frames, including roll, without phase alignment. Target/maximum coverage
are reported separately; the maximum is used for acceptance. These estimates depend on the
saved mapping, rigid geometry and correct tool mounting.

For workpiece-relative accuracy, capture independent synchronized tip positions
and export `elapsed_s,x_mm,y_mm,z_mm` in increasing time order. Use the
`weld_started` event as time zero and retain samples over the checked moving
interval shown as `duration_s`. Register the measurement frame to the **planned**
starting tip on the fixture, with +X along commanded travel, +Y transverse and
+Z up from the workpiece in the flat-ground test. Retain the measured initial
offset. Record the source, spatial calibration and synchronization uncertainty
with the CSV. The comparison uses the checked retimed path and adds the planned
feed in +X when Chassis feed was selected.

```bash
.venv/bin/python scripts/analyze_weld_accuracy.py \
  build/weld/latest/session.jsonl \
  --fit-tip-lag --orientation-target-deg 0.5 --max-orientation-deg 1.0 \
  --tip-csv build/arm_tests/accuracy/tip.csv \
  --output build/arm_tests/accuracy/accuracy-with-tip.json
```

Use `--orientation-target-deg 0.5 --max-orientation-deg 1.0` for the confirmed
tool-orientation criterion. `--peak-joint-deg` remains an optional diagnostic
threshold; the operator has not specified a per-axis joint limit. When the
separate position tolerance is defined, `--peak-tip-mm` checks the same-time
maximum 3D error from independent tip measurements. The optional
`--fit-tip-lag` report uses the independent tip CSV when supplied and otherwise
uses the encoder estimate. It summarizes the lag-adjusted 3D error and measured
stroke width, with positive edge error for overshoot and negative for shortfall.
The original same-time metrics and acceptance thresholds still apply. For the
matched A1 comparison, compare the adjusted error and both edge errors.
The command returns status 2 for incomplete evidence or a missed threshold and
preserves the report when an output path is supplied. The moving interval
requires usable coverage with gaps no larger than 100 ms. Interrupted, faulted,
stale or dropped data remain visible in the report.
The current analyzer also returns 2 for an intentional continuous-run pause.
Inspect the recorded pause cause and coverage: `operator_input` is the expected
manual stop, not a motor fault. Do not switch to `--single-pass` to suppress this
reporting limitation. Without numeric thresholds, `passed` remains null.

## Post-preparation hold acceptance

**HOLD-01 was resolved by operator confirmation on Nov 23.** The latest 1.96 kg
EE session recorded 25.19 s of fixed-target Ready holding, A5 maximum error
0.053 deg, no faults and verified final cleanup. See the
[closure and historical evidence](ARM_SESSION_HANDOFF.md#hold-01-resolution).
The following remains a reusable procedure for future hold checks; it is not
a requirement to repeat the completed issue's acceptance.

1. Use the installed supported fixture, matching reviewed mapping/standby and a
   single motor owner. Retain the original gain/limit readbacks while disabled;
   the [readback procedure](#disabled-currentloop-parameter-inspection-2026-11-10)
   does not change drive settings. Keep the journal path for this exact run.
2. Review the fresh pose and calculated route before explicitly starting prep.
   At `ready`, leave welding stopped and observe at least 15 s. Compare physical
   motion with each joint's displayed goal/error and the recorded target,
   allowance, enabled/status/torque, velocity and feedback age.
3. Confirm that `weld_endpoint_hold` retains the final target and allowance.
   `HOLD DRIFT` / `hold_deviation` mark error above 0.25 deg. Record each axis's
   peak/settled error, physical drop and oscillation. A changing hold target,
   disable or fault is a different failure from sag with an unchanged enabled
   target. Preserve the evidence before recovery or parameter changes.
4. At `ready`, pause/focus loss keep the fixed endpoint command, checked plan and
   **Start welding motion** action available.
   During unfinished travel, pause captures current measured positions once and
   retains its speed allowance. Repeated pause/focus loss must not reset the hold
   target to a drifted measurement. After the weld, verify the same fixed hold.
   Explicit disable, close and faults remove torque. Qualify these behaviors with
   support carrying any loss of torque; hold is not a mechanical lock.
5. If future drive-loop tuning is required, retain per-axis disabled readbacks
   and validate bounded adjustments, restoration and thermal/current behavior.
   Host speed allowance is not a stiffness gain. No gain preset or gain-write interface is installed
   by the current implementation. Stop and diagnose unexpected drop/oscillation before a weld.

After stable holding, separately measure total prep timing. The default hardware
standby is unchanged. The operator accepted the Nov 23 8.22 s preparation.
Explicit `--optimize-preparation`
allows the checked shorter staging route on hardware; review its preview and
reported duration before recovery. Loaded response and actual timing remain
physical acceptance work.

### Matched runs with and without the end effector

Preparation time and calculation time are separate. `--prep-seconds` sets a
reported goal; it does not override the explicitly requested speed cap. The
Nov 19 long-wrist start needs about 91.6 degrees of A6 travel, so 5 deg/s cannot
reach Ready within 5–10 seconds. The offline comparison uses
`--max-speed-deg-s 30 --prep-seconds 10`, retaining `--optimize-preparation` and
`--approach-lift-mm 30`. It gives an 8.71 s nominal plan, 8.88 s ideal-shaft
replay and 11.12 s with synthetic lag. A2 approaches the unchanged 3-degree
tracking trip in the lagging replay; this is not physical speed qualification.
Match any explicitly selected speed setting across both payload runs.

For LOAD-01 payload characterization, first record the assembled arm with the
end effector absent, then repeat with the **1.96 kg** end effector installed
(operator-reported mass, not a measured COM/inertia). The arm itself remains loaded in both
conditions. Keep the full gun collision envelope in the model for both runs.

Use the same reviewed mapping/standby, verified encoder branch, controller,
speed request, acceleration limits and raised approach. Use the same physical
starting pose and support arrangement, and record any support contact carrying
weight during measurement. A catch for torque loss should not change the measured
load between runs. Record initial temperatures and cable routing. Changing the
start can cause the staging optimizer to choose another path: review the saved
plans before attributing a timing or torque difference to the end effector.
Never force a saved path onto changed fresh coordinates to make runs match.

1. With the fixture supported and the previous owner closed by the operator,
   use the normal reviewed weld-panel command, adding a unique
   `--output build/arm_tests/HOLD_RUN/no-ee/session.json`. The corresponding
   `session.jsonl` starts automatically; do not start another telemetry owner.
2. Calculate, review the route and explicitly **Start recovery**. Leave welding
   stopped at **Ready**, and observe **15 seconds untouched**. Record visible
   sag/oscillation and any pushes or support contact. Timing is recovery request
   to verified Ready, including initial hold verification and settling; planning
   time is separate. This first pair does not qualify post-weld holding.
3. Read the JSONL while the panel remains active if needed. Park/rest or restrain
   the links before disable/close. Retain the final shutdown result. Install the
   end effector only with the arm supported and torque disabled; mounting while
   relying on powered hold is not part of this procedure.
4. Repeat with `--output build/arm_tests/HOLD_RUN/with-ee/session.json`, keeping
   the conditions above unchanged. If either run faults or is interrupted, retain
   it separately and use a new filename for a retry. Do not hide the failure in
   aggregate timing. Repeat pairs after a clean first comparison to assess
   repeatability and temperature effects.

Offline analysis (also works on a live journal; no device is opened):

```bash
python3 scripts/analyze_weld_hold.py build/arm_tests/HOLD_RUN/no-ee/session.jsonl \
  --label no-ee --ee-mass-kg 0

python3 scripts/analyze_weld_hold.py build/arm_tests/HOLD_RUN/no-ee/session.jsonl \
  --label no-ee --ee-mass-kg 0 \
  --compare build/arm_tests/HOLD_RUN/with-ee/session.jsonl \
  --compare-label with-ee --compare-ee-mass-kg 1.96 \
  --output build/arm_tests/HOLD_RUN/comparison.json
```

The analyzer separates attempts and contiguous stages; prints measured mean/peak
speed, signed mean/RMS/peak torque feedback, both temperature peaks and observed
temperature changes per axis; and retains command error, fixed-target/speed-allowance
span, temperatures, sample gaps and feedback age in JSON. New hardware journals
also preserve the selected arm profile and its load/torque policy. Motion command error is relative to the dispatched lead-corrected
target, not the controller's nominal-reference tracking trip. For Ready holding
the fixed target is the acceptance reference. Compare equal hold durations.
Stale (effective age >=80 ms), disabled or faulted samples are excluded and counted;
repeated cached samples cannot extend observed holding time. Dropped records,
partial live lines and final shutdown results stay explicit. The sample-weighted
statistics and metadata checks do not establish physical acceptance or calibrated
shaft torque. Unknown payloads in historical journals must remain unlabeled.

### A2 gun-load torque and temperature recording

The earlier A2-only setup is complete: operator reports
`a2-setup-20261119-175046.json` and `readback-20261119-175312.json` verify
A2 TMAX=40 after power cycling. The original A2-only profile and staged image
remain available for provenance. After expanding other drive ranges, use the
following profile; the old profile's range mismatches must not be bypassed.

### Expanded gun-load profile: A2/A3 30 Nm, A4-A6 10 Nm

The operator confirmed **A3 matches A2 at 30 Nm; A4, A5 and A6 use 10 Nm**.
A1 stays 20 Nm. These are absolute feedback trips, not constant torque commands
or qualified continuous-load ratings. The rated references remain 12 Nm for
A2/A3 and 3.5 Nm for A4-A6. There is **no added overload-duration timer**.
Both existing 80 C temperature trips, speed/tracking/freshness checks and leases
remain. A fault still removes holding torque from the whole arm.

The new profile is `src/robot_description/config/arm_rig_gun_a2_a6_load.json`.
It requires physical feedback TMAX **40 Nm on A2/A3** and **12.5 Nm on A4-A6**.
The original A4-A6 encoding ended at 10 Nm and could not observe a crossing
above 10 Nm, so the commissioning changed both range and trip together. TMAX changes encoding;
it does not raise current, gain or temperature settings. Standard profiles and
model planning torque budgets are unchanged; qualification flags stay false.

Firmware build **f78f6b0d**, capability mask **0x1FFFF8F**, is staged at
`build/dm_mc02/arm-load-30-10/forge_arm.elf`. The operator deployed it on Nov 19;
`load-range-setup-20261119-185854.json` and subsequent fresh-MCU / full-profile
readbacks verify the target ranges. The agent performed offline work only.
See the [run review](measurements/arm-gun-load-review-2026-11-19.json).
**Setup is complete on the current rig; do not repeat it for each launch.**
The following remains the deployment procedure for a board/rig needing setup:

1. Mechanically support every link and the gun for torque loss. Disable/close
   the current owner, turn motor PSU off, and connect the SWD programmer. Flash:

   ```bash
   cd /home/forge2/forge2_ws
   sudo /home/forge2/Documents/Codex/2026-10-30/read/tools/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
     -c port=SWD freq=240 mode=NORMAL reset=SWrst \
     -w "$PWD/build/dm_mc02/arm-load-30-10/forge_arm.elf" -v -rst
   ```

2. After successful verification, restore motor power with the arm supported
   and other owners closed. Run the disabled setup:

   ```bash
   cd /home/forge2/forge2_ws
   .venv/bin/python scripts/setup_arm_load_ranges.py --supported --write \
     --report "build/arm_tests/hold-01-20261119/with-ee/load-range-setup-$(date +%Y%m%d-%H%M%S).json"
   ```

   It requires a fresh unconfigured MC02, no latched fault, all six known routes,
   and fresh stationary disabled feedback. It records all discovery registers
   and A2-A6 control/protection settings before writing. A2 already at 40 is
   skipped; A3 changes 28 to 40, A4-A6 change 10 to 12.5. Each drive is read back
   independently before one save, then checked again. Only live VBus register
   60 is excluded from configuration equality; all original values stay in
   evidence. The motor save persists all parameters together, so a changed
   unrelated setting blocks the operation. No motor is enabled and no mode,
   gain, current, thermal, ID or zero parameter is changed by this command.

   An uncertain write/save is never retried. It prevents setup of the next
   motor, and motion stays locked until MCU restart, even after fault reset.
   On failure, preserve the printed report and resolve it before launching.

3. After setup succeeds, **power-cycle BOTH motor power and MC02**. Restarting
   Python alone does not verify drive persistence or clear the MCU setup lock.

4. Require a successful disabled readback of all five changed ranges before launch:

   ```bash
   cd /home/forge2/forge2_ws
   .venv/bin/python scripts/test_dm_arm.py \
     --plan src/robot_description/config/arm_rig_gun_a2_a6_load.json \
     --all --check --feedback --supported \
     --report "build/arm_tests/hold-01-20261119/with-ee/load-readback-$(date +%Y%m%d-%H%M%S).json"
   ```

   Expect six fresh disabled readings and **PASS**. Firmware/profile and actual
   drive ranges must match; do not edit expected ranges just to bypass failure.
   After PASS, launch the same motion settings with the new profile:

   ```bash
   .venv/bin/python scripts/control_weld.py \
     --execute --supported --arm --profile weld \
     --arm-profile src/robot_description/config/arm_rig_gun_a2_a6_load.json \
     --mapping build/arm_tests/hold-01-20261119/mapping.json \
     --initial-pose build/arm_tests/hold-01-20261119/standby.json \
     --a1-encoder-turns 0 --tracker pid \
     --max-speed-deg-s 30 --prep-seconds 10 \
     --optimize-preparation --approach-lift-mm 30 \
     --lowering-seconds 2.3 --cycles 1
   ```

   This preserves the 30 deg/s preparation cap, 10-second goal and 2.3-second
   final lowering minimum. Physical settling can exceed that goal. Calculate,
   review the preview against the physical pose, then explicitly start recovery.
   At Ready, record a 15-second untouched hold before separately starting welding.
   `--cycles 1` sets the checked preview block; welding repeats until paused.
   Keep `--single-pass` out of this command. Confirm that the panel says
   **continuous until Pause / Hold** before starting welding.
   Reports are saved automatically at `build/weld/latest/`, with earlier runs
   retained under `build/weld/sessions/`.

5. The existing owner's cached journal records signed torque, MOS temperature
   and motor temperature on all six axes during preparation, holding and welding.
   Do not open another motor connection for monitoring. Offline analysis:

   ```bash
   python3 scripts/analyze_weld_hold.py /path/to/the/with-ee/session.jsonl \
     --label gun-attached-load-30-10 --ee-mass-kg 1.96 \
     --output /path/to/a/new/torque-temperature-summary.json
   ```

Keep the changed thresholds explicit in comparisons. The 12.0137 Nm event in
`session-branch0-20261119-164725.jsonl` had the gun attached despite its `no-ee/`
directory; its load-context sidecar records that correction. Temperature logging
alone does not qualify sustained torque above the manufacturer's rated reference.

## Optional A2-A6 keyboard workflow, A1 disconnected

This subset uses the keyboard, without a radio. Use the six-axis launcher above
when A1 is connected. The following example retains release-disable behavior.

```bash
cd /path/to/FORGE
source scripts/ros2_env.sh
python3 scripts/jog_dm_arm.py --execute --supported \
  --plan src/robot_description/config/arm_rig_a2_a6.json \
  --port /dev/ttyACM0 --manual-travel --speed-deg-s 5 \
  --report "build/arm_tests/manual-keyboard-$(date +%Y%m%d-%H%M%S).json"
```

Select 2–6, press E once, then briefly hold an arrow. Up/right is raw-positive
(previously observed CCW); down/left is raw-negative. Release stops. Space,
focus loss, selection change or close disarm. A1's button is disabled and its
keyboard key stops/disarms with an unavailable message. Simulation accepts the
same plan/manual-travel/speed with `--simulate` instead of `--execute --supported`.

The compact window starts at 880 × 420 pixels and grows if needed for fault text.
After a trip, release arrows, address the cause and press **R / Recover fault**.
It saves evidence, checks every retained motor is disabled/stationary, then clears
the software latch. Wait for **Recovered and disarmed**, then use fresh **E +
arrow**. A retained startup fault opens in lockout with R available. See
[keyboard recovery](ARM_POSITION_JOG.md#recover-a-trip-inside-the-keyboard-window).
The keyboard uses no radio. Current firmware already supports it; normal
sessions need no reboot or flash.

## Optional selected-joint position hold

The default jogger is a short bench test: a held jog expires after 12 seconds
on the host, with a separate 15-second MCU enable cutoff. Release disables
torque, so a gravity-loaded joint can fall. These duration bounds are separate
from the 150 ms command lease and 100 ms MCU feedback watchdog.

`--hold-position` explicitly selects a different keyboard policy for supported
fixture trials. It requires matching MC02 firmware with capability **0x10000**;
the previously deployed `156f20c8` image cannot run it. Follow the
[build/flash procedure](DAMIAO_ARM.md#build-flash-run), with motor PSU off and
all links mechanically supported, then verify the new build identity before use.

After a disabled six-axis check, launch from this checkout:

```bash
python3 scripts/jog_dm_arm.py --execute --supported --manual-travel \
  --hold-position --speed-deg-s 5 \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 \
  --report "build/arm_tests/hold-$(date +%Y%m%d-%H%M%S).json"
```

- Select a joint while disabled, press **E**, then hold an arrow. E alone
  authorizes motion; the first arrow performs current-position preload and enable.
- **Release** captures the fresh measured position once and keeps the selected
  joint enabled at that fixed mode-2 target. A further arrow resumes jogging
  without disabling/re-enabling. Drive position control supplies holding torque;
  no guessed constant torque, current limit or gain is written.
- Jogging starts with 100 ms of target lead, then advances at the selected
  rate with up to 5 degrees of forward lead over feedback. This accommodates
  position-loop lag: a 0.5-degree lead cap reduced recorded A2 motion from
  approximately 5 to 0.6 deg/s. At the cap, missed travel is discarded;
  release still replaces the moving target with the measured release position.
  This host correction needs no firmware reflash. Its physical retest is pending.
- **Space / Escape / focus loss** stop travel and retain the hold, while
  revoking arrow authorization. Use fresh E and arrow to resume. The UI shows
  HOLDING separately from motion authorization.
- **D / Disable torque / Q / closing the window** disable torque. Mechanically
  support the load before these actions. Selection is locked while the selected
  joint is enabled; explicitly disable before changing joints. Other joints stay
  disabled. This is not coordinated whole-arm holding.
- This mode has no 12/15-second bench cutoff. Its fixed hold target is refreshed
  by the existing control loop; telemetry reads do not renew the lease. Command
  loss, stale feedback, stalled UI, USB/CAN faults, drive faults and configured
  speed/torque/temperature trips still disable and require explicit recovery.
  A cancelled initial enable also disables. Power loss cannot provide holding.

First acceptance: under mechanical support, raise A2 only a small distance and
release. Measure hold drift, tracking error, torque feedback and temperatures;
test Space/focus loss, fresh E resume, and a supported D disable. Cross 15 seconds
only after the initial short hold behaves correctly. Save the report and physical
observations. Software/fake-motor checks do not qualify loaded holding or thermal
capacity; keep production confirmation flags false.

## Hold all connected joints

For group holding, use the explicit mode below; the original `--hold-position`
selection lock remains appropriate for a single energized joint.

`--hold-all` adds an explicit **H / Hold all connected joints** activation.
It implies `--hold-position` and still requires supported manual travel. A new
firmware image with capability **0x20000** is required; the earlier selected-joint
hold firmware intentionally rejects group enable. The prepared build is
`82794a1b` / capabilities `0x3FF8F`, saved under `build/dm_mc02/group-hold/`.
Do not flash, close a holding session or turn off power while the load depends
on motor torque. Park/rest or restrain the complete arm first. These commands
do not qualify unsupported operation or replace a brake/restraint on torque loss.

After flashing with motor PSU off, reconnect motor power and complete the
disabled six-joint check. Then launch from the task checkout:

```bash
python3 scripts/jog_dm_arm.py --execute --supported --manual-travel \
  --hold-all --speed-deg-s 5 \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 \
  --report "build/arm_tests/group-hold-$(date +%Y%m%d-%H%M%S).json"
```

The window starts disabled. **H** captures fresh stationary positions for all
configured joints, preloads each target, validates the complete enable mask and
enables the group. It stays disarmed for jogging. Use **1–6**, **E**, then an
arrow; release captures the selected joint's measured position once. Other
targets remain fixed. Selecting another joint pauses travel and disarms jogging
without disabling any joint; use fresh E and arrow after selection. Repeated H
does not adopt passive drift as new targets. Space/focus loss hold the whole
group and disarm movement. The UI shows fresh feedback for every held joint.

**D / Q / closing** disable the complete group. Any joint's trip, missing
feedback or expired command lease also disables the group. Recovery remains
explicit and disabled; another H is required. The firmware rejects partial
group enable, mixed group/single-axis hold policies and individual enable on a
group-configured slot. No background owner or motor parameter polling is added.

Firmware and fake-motor/Qt checks passed, and an operator-run group session is
recorded. A1/A4/A5/A6 moving jitter and an A4 hold overspeed need investigation;
see the current handoff. Physical loaded holding, CAN timing and thermal behavior
remain unqualified. Do not interpret average 5 deg/s travel as smoothness acceptance.

## Optional smooth jog trial

The operator reports no improvement from the 5 deg/s smoothing trial and requested
doubling all six jog speeds. Build `d61f6d16` / capabilities `0xFFF8F` enables
10 deg/s `--smooth-jog`; holding-mode defaults are now 10 deg/s. It advances
moving targets at the existing 2 ms CAN cadence. Release/Space/focus loss still capture a fixed hold target
without applying the jog ramp to that stop. It changes no drive gains or trips.
The forward-lead limit scales with speed (5 degrees at 5 deg/s, 10 at 10) to
avoid slowing the lagging A2/A3 position loops. This trial has passed software
checks; physical jitter reduction is unverified.

Park/rest or restrain the arm so it cannot fall when torque is removed. Close
the previous owner and turn motor PSU **off** before flashing. From this checkout:

```bash
sudo /home/forge2/Documents/Codex/2026-10-30/read/tools/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
  -c port=SWD freq=240 mode=NORMAL reset=SWrst \
  -w "$PWD/build/dm_mc02/jog-10/forge_arm.elf" -v -rst
```

Only after programming/verification succeed, turn motor PSU on. With the fixture
able to carry the links on torque loss, run:

```bash
run="build/arm_tests/jog-10-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$run"
python3 scripts/test_dm_arm.py --all --check --feedback --supported \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 --report "$run/disabled.json" &&
python3 scripts/jog_dm_arm.py --execute --supported --manual-travel \
  --hold-all --smooth-jog --speed-deg-s 10 \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 --report "$run/holding.json"
```

The UI should say **Smooth jog (2 ms)**. H holds all, then 1 → E → arrow tests
A1. Start with a brief press each direction within known clearance, releasing
between directions; compare moving smoothness and steady hold. Repeat for A4,
A5 and A6, then use A2 as a comparison. Space pauses/holds; D/Q/close/fault
removes all torque. Park/rest before closing to save the report.

For an A/B session on the same firmware, omit `--smooth-jog` and use a different
report path. Holding reports include firmware identity, target policy and every
existing host sample in an 18,000-row ring. At six joints and 50 Hz that is
roughly the most recent minute; sample timing depends on actual owner cadence.
Targets in this report are host endpoints, not a capture of every smoothed CAN
target. Feedback/command batch durations and whole owner-poll time are recorded
without extra control traffic; cached MCU timing/error counters are read after
disabled cleanup. A physical CAN capture is still needed to resolve faster oscillations.

## Fixture and first startup

All attached links, including the unpowered A1 axis, must be mechanically
supported throughout the test and on torque loss. Support must carry the
weight while allowing the selected manual motion; do not drive a joint against
a rigid restraint. `--supported` / `arm_supported:=true` is the operator's
confirmation of that arrangement. If the arm can fall when disabled, arrange
support before running either the feedback check or physical jogging.

Stop the previous Pocket/keyboard launch and any other MC02 owner. The commands
below assume the existing temporary CAN wiring and MC02 at `/dev/ttyACM0`;
substitute its actual path if enumeration changed. They do not manage the PSU.

If MC02 still retains a six-motor session from before A1 was disconnected, its
disabled-feedback check will fail waiting for A1. With the arm supported and
motor power off, restart MC02 once to clear the old volatile routes, then restore
motor power for the check. This is explicit recovery after a topology change,
not a routine reset between successful runs. Do not automatically clear a fault
or reset MC02 while the arm relies on motor torque. The updated manual-travel profile requires firmware capability **0x8000**.
Build **156f20c8** / mask **0xFF8F** was verified deployed by passive
readback on 2026-11-10. The corresponding image is
`build/dm_mc02/manual-travel/forge_arm.elf`; it also includes disabled control
readback (0x4000). No additional flash is required. Older images refuse this manual launch before motor setup.

## Connection and encoder check

From the workspace:

```bash
cd /path/to/FORGE
python3 scripts/test_dm_arm.py \
  --plan src/robot_description/config/arm_rig_a2_a6.json \
  --all --check --feedback --supported \
  --port /dev/ttyACM0 \
  --report "build/arm_tests/rig-a2-a6-check-$(date +%Y%m%d-%H%M%S).json"
```

Here `--all` validates the complete six-motor inventory; `active_joints` in the
plan limits this check to A2–A6. The output lists their raw angles, feedback
ages and CAN addresses, and explicitly lists A1 as omitted. Expect `PASS` and
five fresh, stationary, disabled rows. Investigate any failure before jogging.

This checks the existing ID/range/mode/damping registers, configures volatile
MC02 routes for slots 1–5 and requests fresh normal feedback with disable
frames. It never enables a motor, sends a movement target, writes motor
parameters, saves a zero or changes the motor's mode. It does remove torque.
The JSON retains raw radians/degrees and routing; angles are not model angles.
The current firmware does not expose direct `p_m`/`xout` register reads, so this
test does not claim to verify those readbacks or power-cycle/wrap behavior.

For offline plan validation, omit `--check --feedback --supported`; no serial
device opens. For register checks without a new feedback capture, use `--check`
alone; an existing session can still require disabled-feedback refresh.

## Recover a latched fault without unplugging MC02

Stop the previous Pocket/keyboard process. Check the cause of the trip (for
A2, support, clearance and binding) with all links supported. Then run:

```bash
cd /path/to/FORGE
source scripts/ros2_env.sh
python3 scripts/test_dm_arm.py \
  --plan src/robot_description/config/arm_rig_a2_a6.json \
  --all --check --feedback --supported --recover \
  --port /dev/ttyACM0 \
  --report build/arm_tests/rig-a2-a6-recovery.json
```

`--recover` is explicit authorization to clear the controller fault after the
cause has been addressed. It requires the disabled check, support confirmation
and a report path. Normal checks and Pocket launches still leave faults latched.
The existing firmware supports this command; no reflash or USB unplug is needed
for a recoverable fault such as the recorded A2 torque trip.

Recovery captures the original fault, verifies every retained motor has fresh,
stationary disabled feedback, and saves a unique companion file
`build/arm_tests/rig-a2-a6-recovery.fault-<unique>.json` before issuing reset.
The summary JSON records that path, reset attempt/acknowledgement/verification,
pre/post states, and the subsequent normal ID/encoder check. Companion evidence
survives reuse of the summary filename. If saving evidence fails, no reset is
sent. Reset invalidates the MCU range cache, so recovery re-probes the existing
routes and confirms fresh disabled feedback again. Motor parameters, modes,
thresholds and encoder zeros are unchanged; no enable or movement target is sent.

Expect `Recovery complete; motors remain disabled`, five A2–A6 encoder rows and
`PASS`. Reopen the keyboard, select a motor and explicitly arm again.
Recovery never resumes a jog. If no fault is latched, the option verifies the
retained disabled state and runs the check without sending a reset.

Missing retained motors (including A1 from an old six-motor session), ongoing
motion, drive errors, invalid/hot feedback or lost communications prevent a
verified recovery. A missing motor in the requested new plan fails the normal
check even if recovery itself succeeded. Resolve the reported cause rather
than repeatedly clearing faults. The option does not remove old routes, reboot
a hung USB device or unlock an ID-commissioning session; topology/ID-session
restarts described above remain separate. A reset acknowledgement timeout is
reported as uncertain, never silently retried; the preserved evidence remains
available for the next explicit check.

## Optional Pocket jogging

Build the changed launch/config packages once:

```bash
source scripts/ros2_env.sh
colcon --log-base logs/colcon build --base-paths src \
  --build-base build/ros2 --install-base install/ros2 --symlink-install \
  --packages-select teleop bringup robot_description
source scripts/ros2_env.sh
```

After the check passes, and while all links remain supported:

```bash
ROS_DOMAIN_ID=93 ros2 launch bringup pocket.launch.py \
  simulate:=true radio_port:=auto arm_execute:=true arm_supported:=true \
  arm_port:=/dev/ttyACM0 \
  arm_plan:="$PWD/src/robot_description/config/arm_rig_a2_a6.json" \
  radio_config:="$PWD/src/bringup/config/pocket_controls_rig.json" \
  arm_report:="$PWD/build/arm_tests/rig-a2-a6-pocket-$(date +%Y%m%d-%H%M%S).json"
```

Radio discovery follows the ESP32 when it changes USB sockets. Turn on the
transmitter/receiver and close other serial readers before launch. It receives
five fresh CSV rows from exactly one eligible adapter before starting nodes;
no match, multiple matches or busy ports stop startup. Explicit
`radio_port:=/dev/serial/by-path/ACTUAL_RADIO` still overrides discovery. See
[USB identification and exclusions](POCKET_ARM_CONTROL.md#usb-radio-discovery).

Chassis remains simulated. Right-stick left/up=CW, right/down=CCW. SB middle
and bottom select A2/A3; SC selects A4/A5/A6. SB top shows A1 unavailable and
revokes motion authorization. Select a connected joint, release SE, center the
right stick for 0.3 seconds, then hold SE and briefly deflect one axis.

This profile requests **5 degrees/s with operator-controlled manual travel**
(`window_deg: null`). The operator requested removal of startup-angle constraints
for this commissioning stage. All four diagonals stop without revoking an
existing SE prime. Centering, diagonals and SE release disable torque; other
joints remain disabled. The 12-second host / 15-second MCU cutoffs, measured
speed/torque/temperature trips and communication watchdogs remain. Finite raw
protocol bounds (+/-min(12.5, PMAX) rad) remain; these are not mechanical limits.

For offline rehearsal with the same five-joint selection, use
`arm_execute:=false` and omit `arm_supported:=true` and `arm_port`.
Do not run keyboard and Pocket hardware owners together.

The Pocket summary is written to the timestamped `arm_report` path during
session shutdown. Use a different filename for each retained run; the same
destination is overwritten. Omitting `arm_report` leaves summary-file output
disabled. Console/ROS logs are separate, normally under `~/.ros/log/`.

## Disabled current/loop-parameter inspection (2026-11-10)

Disabled readback completed for A2–A6 at 19:56; see
[the register values and interpretation](DAMIAO_TORQUE_LIMITS.md). The deployed
156f20c8 firmware supports capability 0x4000. With the previous controller owner
closed and all links supported, a new read can be recorded with:

```bash
python3 /path/to/FORGE/scripts/test_dm_arm.py \
  --plan /path/to/FORGE/src/robot_description/config/arm_rig_a2_a6.json \
  --all --check --feedback --supported --inspect-control \
  --port /dev/ttyACM0 \
  --report "/path/to/FORGE/build/arm_tests/rig-control-registers-$(date +%Y%m%d-%H%M%S).json"
```

This disables and verifies stationary feedback before each register read; it
never enables, commands movement, writes gains/limits or saves parameters.
Normal checks and launches never automatically inspect these registers. It
records raw bits, decoded types and read status per motor. A no-reply/nonfinite
value makes the inspection incomplete; do not substitute zero or keep retrying
to hide it. A wrong reply identity aborts. The 8009P's undocumented Imax/VBus
are explicitly skipped. `PASS` means the documented reads completed, not that
factory limits, current/torque calibration or persistence are validated.
Fault recovery remains separate and explicit; inspection does not clear faults.

## Limits and evidence

The supported baseline is **5 degrees/s**. Explicit manual travel removes
startup-angle windows; relative zero is only a session display reference.
Bounded mode remains separately available with a window of at most +/-3 degrees.
All active A2–A6 measured-speed trips are **1.5 rad/s**, at the operator's request.
The unchanged baseline profiles use **12 Nm for A2/A3** and **3.5 Nm for A4-A6**.
The currently deployed gun-load trial instead uses **30 / 10 Nm** with matching
40 / 12.5 Nm feedback ranges; use its [profile and commands](#expanded-gun-load-profile-a2a3-30-nm-a4-a6-10-nm).
A baseline profile with the old feedback ranges will fail readback on this rig.
These trips are reactive stops, not current commands or continuous-load ratings.
Finite bench runs retain their 12 s host / 15 s MCU cutoffs; explicit holding
and continuous welding have no such run cutoff or added overload timer. Protocol,
freshness/lease, speed, temperature and fault guards remain. Loaded holding and
calibrated paths remain [open work](ARM_LAYER_TODOS.md).

The shared table separates `Feedback Nm` from `Trip Nm`, shows feedback age,
and blanks stale dynamics. Shutdown reports retain enabled motion samples,
sampled peaks, session origins/bounds, startup registers and faults. Peaks are
host-sampled, not every CAN frame or an independent torque/current measurement.
Use unique report paths; close the window to write the full session report.

The operator's five-motor motion confirmation is recorded in the
[current handoff](ARM_SESSION_HANDOFF.md). Earlier incidents/readback and software
verification remain in ignored local `build/arm_tests/` and `logs/arm_tests/`.
Reproduce fake-motor/Qt/ROS checks using [tests/README.md](../tests/README.md).
Basic motion does not complete calibrated startup, faster-speed qualification,
current protection, loaded holding or the physical failure matrix.
