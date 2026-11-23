# Workspace guidance

## Language

Keep first-party code, comments, GUI text, logs and documentation in English.
The language used in a chat does not change the application or repository language.
Preserve original vendor filenames and quoted identifiers.

## Start here

Read `README.md`, then `docs/ARM_SESSION_HANDOFF.md` for current arm status and
the next session. Use `docs/ARM_LAYER_TODOS.md` and the owning package's `TODO.md`
for open work; remove completed tasks instead of appending incident histories.
Operator procedures belong in `docs/ARM_RIG_TEST.md`; architecture and ownership
are in `docs/ARM_BASE_LAYER.md` and `docs/SOURCE_LAYOUT.md`.

Welder communication is separate from arm motion. The read-only RTU/TCP
probe and wiring choices are in `src/welding_machine/README.md`. Use the
machine manual for its pinout, link settings, unit ID and register map.

Calibration evidence (2026-11-13): all six motors are connected. The Nov 12 eight
reference readings remain in the dated reference JSON; the Nov 13 stationary
cycle and all six off-power return observations are recorded in
`docs/measurements/arm-return-checks-2026-11-13.json`. Do not repeat that same
round solely because manually returned angles differ slightly. CAL-01 still
needs broader encoder-branch coverage, independent model alignment and physical
qualification. The candidate map preserves the Nov 13 corrected A3 endpoint
labels (CW 72.77°, CCW 151.39°); arbitrary return-test poses are not anchors.

Manual control is `bash scripts/control_arm.sh`, with no position capture UI.
Reference recording is separate (`scripts/record_arm_positions.py` / its shell
launcher). `scripts/calibrate_arm.py` is the reusable model-guided planner and
supported calibration UI: measured six-axis pose, explicit path start, bounded
waypoints, Space pause/hold, disabled hand-position constraint recording, and
model/mapping/equipment fingerprints for reruns. See `docs/ARM_MODEL_CALIBRATION.md`.
No new calibration was promoted to production and the agent did not access
hardware. Keep unknown fields null and production flags false. The saved A3
model initial pose overlaps its stop; it is not an automatic hardware target.

The Nov 12 saved reference session verifies firmware `d61f6d16`, capabilities `0xFFF8F`, but
ended with exit 2: final feedback was disabled on all six joints, while A2 speed
was -0.168498 rad/s and stationary-disable/mode-restoration checks failed.
Do not infer successful loaded shutdown from successful position capture. No
rest/restraint for torque loss has been confirmed; do not interrupt a powered
owner or flash/resume physical tests while the load depends on torque.

The operator performs hardware work themselves. Do offline preparation and give
commands; do not launch/stop/reset/flash a motor owner without new authorization.
Use `bash scripts/record_arm_positions.sh` for the eight-position workflow.
`--capture-mode bounds` is the optional longer two-ends-plus-initial sequence
for every joint. The older `--teach-file` requires disabled hand placement.
Capture UI results persist, with readiness reasons and explicit Next selection.

Coordinated tracking/jitter work resumed under SPEED-01 by operator request on
Nov 17; broader manual-jog investigation remains deferred. A2/A3 move smoothly by operator
report; A1/A4/A5/A6 were reported to jitter during travel while manual holds stayed steady.
The 5 deg/s smoothing trial did not fix travel jitter. The separate A5 fixed-hold
fault (HOLD-01) was resolved by operator confirmation on Nov 23. Holding-mode
jogging now defaults to 10 deg/s by explicit operator request;
non-holding/teaching remains 5 deg/s. All six measured-speed
trips remain 1.5 rad/s. Current protection, thermal endurance and physical stopping
remain unqualified. Do not increase reactive trips or claim jitter resolved.
The operator confirmed the earlier A1 undervoltage issue was fixed.

## Current welding implementation (2026-11-24)

ACCURACY-01 and SPEED-01 prioritize wire-tip position and stroke. Continuous
arm-only tip reporting allows one fixed lag. The separate same-time tool
orientation target is 0.5 deg with a 1.0 deg maximum. Runtime limits remain
3 degrees for tracking and 0.25 degree for arrival. The operator accepted
8.22 s preparation under the 10 s trial goal and retired the five-second goal.
Calculation latency remains separate. The accuracy analyzer reads saved data
without a transport. Planner caches reuse unchanged scene and weave inputs. Recovery is checked from each fresh measured start.

PLAN-01 and BASE-04 were resolved by operator confirmation on Nov 23. Do not
carry their historical planning delays or CAN incidents forward as open tasks.
The Nov 23 accuracy/speed review records an existing physical baseline: A1/A6
measured travel 3.891/1.333 deg versus planned 2.261/0.428 deg; measured frequency
1.734/1.728 Hz versus checked 1.730 Hz. Prioritize overswing under SPEED-01.
Phase remains diagnostic for the arm-only position comparison. Encoder-reconstructed full tool orientation has 1.408 deg RMS /
2.633 deg peak error, exceeding the 1-degree maximum. Independent
workpiece-relative tool-pose measurements and a tip-mm tolerance remain
missing. Complete logs, calibration, accepted plan, issues and offline commands
are tracked in `docs/measurements/weld/2026-11-23/README.md`. See
`docs/measurements/arm-accuracy-speed-review-2026-11-23.json`.

Use `docs/ARM_SESSION_HANDOFF.md` for current evidence and `docs/WELD_MOTION.md`
for architecture. Shared planning belongs in `src/controls/controls/controller/`;
`hardware/` and `simulation/` are siblings. Torque and both temperatures,
trends/peaks and cached ages are visible in the
weld/manual panels. State/progress/fault/pause causes are distinct; diagnostics
are expandable. The fixed-reference holding software is corrected. HOLD-01 is
resolved by operator confirmation; do not carry the historical A5 hold fault
forward as an open blocker. See `docs/ARM_SESSION_HANDOFF.md#hold-01-resolution`.

The operator deployed MC02 `f78f6b0d`, capabilities `0x1FFFF8F`, and verified the
explicit `arm_rig_gun_a2_a6_load.json` profile: A2/A3 trips 30 Nm with TMAX 40;
A4-A6 trips 10 Nm with TMAX 12.5; A1 unchanged at 20/54 Nm. These are authorized
trial settings with no added overload timer, not qualified continuous ratings.
Standard profiles, drive gains and current settings remain unchanged. Both 80 C
trips, all 1.5 rad/s speed trips, the 3-degree tracking guard, 0.25-degree arrival
and freshness/lease/fault semantics remain. Do not raise other trips to hide lag.
Range setup is complete; do not repeat setup/flash for normal runs. Manual
`control_arm.sh` needs the same explicit `--plan` profile on this rig; baseline
profiles retain original ranges and must fail mismatched readback.

The Nov 19 review records two attempts: first, untouched A5 movement during a
fixed Ready hold and a 1.51648 rad/s speed trip; second, Ready in 12.53 s,
8.69 s fixed hold, 45.71 s welding and verified final disable/mode restoration.
Those incidents remain historical evidence. On Nov 23 the operator confirmed
the A5 holding issue no longer exists and closed HOLD-01. The latest session,
with the operator-confirmed 1.96 kg EE, reached Ready in 8.22 s and held fixed
targets for 25.19 s; A5's maximum error was 0.053 deg, with no recorded fault
and verified final disable/mode restoration. A1/A6 travel accuracy and
thermal/continuous-load qualification remain separate work.
The gun was attached during the earlier 12.0137 Nm A2 fault
although its raw journal is under `no-ee/`; preserve the load-context correction.

The default host PID tracker follows nominal preparation time with bounded lead,
filtered derivative feedback, anti-windup and acceleration-bounded braking inside
finite path extrema. `--tracker paced` retains the older common clock. Settled
endpoints retain the original fixed target and checked speed allowance. A pause
during travel captures measured hold once and requires replanning; repeated
pauses do not recapture drift. Ready focus/heartbeat pauses retain the checked
plan and explicit Start welding action. Fault/disable/close remove torque.
Reset motor fault (R) saves evidence through the same owner and verifies disabled,
stationary feedback; success stays disabled with selections off and plan invalid.

The operator requires welding to repeat until manual Pause / Hold, without a
cycle-count stop. `--cycles` sets only the checked preview block, even at 1.
Never include `--single-pass` in a normal launch command; use it only when the
operator explicitly requests a finite run. Preserve focus/heartbeat stops and
periodic cubic checks. Moving ticks use one cached feedback batch and
one target batch. Weld host lead/PID is omitted below 0.3 deg planned axis travel;
nominal targets remain unchanged. Measured Hz/travel uses cached feedback.

Preparation defaults to 5 deg/s and <=30 deg/s². The current explicit loaded
trial uses 30 deg/s, a 10 s goal, `--optimize-preparation`, 30 mm approach lift
and `--lowering-seconds 2.3`. Final-lowering stretching preserves earlier timing
and path geometry. The CLI accepts explicit settings up to 100 deg/s, bounded by
68.7549 deg/s motor command caps; measured-speed trips remain 1.5 rad/s. Planning
latency (~40 s in the offline comparison) is distinct from physical motion time.
A duration goal never overrides guards or truncates motion.

Hardware accepts fresh measured starting poses independently of saved pose
windows and simulator joint ranges. The plan records a session-local recovery
extent only as far as that measured start; no outward extension, wrapping or
saved calibration change is allowed. A4/A5/A6 cable envelopes and motor protections
remain. Starting contact is a visible warning; subsequent path samples still
require collision clearance. The allowance ends before approach/welding.
Simulation retains strict pose limits. Physical qualification remains open.

The special A3 contact retreat permits up to 0.25 deg startup excess above 151.39 deg, with a separate
checked inward retreat to 149.39 deg at <=1 deg/s and <=5 deg/s². It requires
shallow separating contact at the reviewed stop pair and strict normal handoff;
other cable/branch/collision checks remain. Raw readings are never wrapped or
clamped. Physical loaded retreat still needs qualification.

Use the independent A5 candidate with its matching standby; production flags
remain false. Candidate gain magnitude is one because feedback is already geared.
A3 standby is raw 149.39 deg. Output switches gate commands independently without
counterpart compensation. Welding target remains 2 Hz / 12 mm/s. LOAD-01 and
FEEDBACK-01 follow the first qualified run; no gravity feedforward, Cartesian
correction or ESO is active. JSONL and the offline hold analyzer use cached owner
data without another owner or enabled-state register polling.

## Source and verification

- ENCOS is chassis: C11 driver/utilities in `src/encos_driver/`; shared Python
  APIs in `src/motors/`. Damiao is arm: DM-MC02 firmware in
  `src/damiao_firmware/`. Native Ubuntu 24.04 is the deployment target.
- Shared launch/config belongs in `src/bringup/`; reusable code belongs in the
  owning package; `scripts/` contains thin wrappers/build helpers. Follow
  `docs/SOURCE_LAYOUT.md`; do not add root-level implementations.
- Run the relevant checks in `tests/README.md`. Distinguish fake-motor, ROS/PTY,
  model and physical evidence. Generated reports/builds/logs stay ignored.
- Keep vendor contents unchanged. Use references indexed by `vendor/encos/README.md`
  when available; do not scan/extract `vendor/encos/_archive/` unless requested
  for the corresponding Windows, USB-CAN, bridge, IgH, demo/video or CAD work.
- ENCOS hardware context: `docs/ENCOS_WORKSPACE.md`, `docs/ENCOS_BRINGUP.md`,
  `docs/ENCOS_TEST_BENCH.md`, `docs/ROS2_MOTOR_CONTROL_STATUS.md`.

## Motor contracts

- One serial owner per controller and one radio reader. Telemetry consumes
  cached feedback; never add CAN register polling while enabled or let a
  telemetry read renew a motion lease. Preserve raw-shaft SI units and freshness.
- Preserve latched faults and explicit recovery. Keyboard **R** saves evidence,
  verifies disabled/stationary motors and recovers through the existing owner;
  success stays disarmed until fresh **E + arrow**. Never auto-reset/re-enable.
- Current rig: A2–A6 on CAN2, original IDs 2–6 and feedback IDs 18–22; A1's
  inventory route is CAN1/ID1/feedback17. Subsets retain physical slot numbers
  and never silently omit another missing motor. See `docs/ARM_MOTOR_IDS.md`.
- Manual travel removes startup-angle windows only when explicitly selected.
  Default release disables torque; nonselected joints stay disabled. The explicit
  keyboard `--hold-position` mode holds only the selected joint on release,
  Space and focus loss; D/close/fault disables. It requires firmware capability
  0x10000, locks selection while active and retains communication watchdogs.
  `--hold-all` instead requires capability 0x20000 and explicit H activation of
  every configured joint; selection preserves all holds and disarms jogging.
  D/close/fault still disables all. This implementation is not physically qualified.
  Mechanical support must carry all links on torque loss. Loaded holding remains
  physically unqualified; do not infer coordinated holding from this mode.
- Session zero, raw encoder position, model zero and calibrated physical limits
  are distinct. Keep unknown calibration null and production flags false until
  their documented acceptance criteria are measured.
- A4 cable travel is fixed at CW/CCW 270 degrees each from the preserved initial
  raw angle +173.183435236 degrees (`arm_cable_limits.json`). Never rebase that
  allowance at startup or wrap equivalent orientations across its bounds. A6 also
  has fixed CW/CCW 270-degree travel from raw +64.161048535 degrees
  (`arm_a6_cable_limits.json`), confirmed Nov 23; measured-start recovery cannot
  extend it. Manual, planned and native command bounds all retain this limit. A5's
  Nov 16 cable-constrained CW/CCW ends and independent straight pose are recorded
  in `docs/measurements/arm-a5-alignment-2026-11-16.json`; ends do not define model
  zero. The saved complete candidate is separate from the default mapping and
  still needs alignment checks away from straight and physical qualification.
- Pocket policy and commissioning live in `docs/POCKET_ARM_CONTROL.md`. Preserve
  original input deadlines, SE priming, diagonal stop/resume and cancellation
  during enable. Its GUI consumes cached telemetry and requests stop; it must
  not acquire another motor/radio owner or publish competing `/cmd_vel`.

## Modeling

Read `docs/ONSHAPE_ARM_PREVIEW.md` before model edits. MuJoCo model and rendering code lives in
`src/simulation/simulation/`, coordinated motion in `src/controls/controls/`,
and shared assets in `src/robot_description/mujoco/`.
Preserve D3 closure and the 150 mm connection-center spacing; keep simulation
coordinates separate from hardware calibration. Run the relevant layout/model,
overview, color, magnetic, terrain and actuator checks. Never commit
`src/simulation/.env`, `src/robot_description/mujoco/config.json` or backups.
