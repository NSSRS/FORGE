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
report; A1/A4/A5/A6 jitter during travel but hold steady. The 5 deg/s smoothing
trial did not fix it. Holding-mode jogging now defaults to 10 deg/s by explicit
operator request; non-holding/teaching remains 5 deg/s. All six measured-speed
trips remain 1.5 rad/s. Current protection, loaded holding and physical stopping
remain unqualified. Do not increase reactive trips or claim jitter resolved.
The operator confirmed the earlier A1 undervoltage issue was fixed.

## Current welding implementation (2026-11-17)

Shared planning is in `src/controls/controls/controller/`; `hardware/` and
`simulation/` are siblings. See `docs/WELD_MOTION.md` and the current handoff.
Prioritize HOLD-01 for the reported post-preparation sag and loaded stiffness.
The host now latches the settled endpoint and unchanged speed allowance. Idle/pause at an
already settled endpoint preserve that fixed target; interrupted travel still
captures a measured hold. Fault/disable/close semantics and all trips remain.
At ready, pause/focus/heartbeat loss retain the checked plan and explicit Start
welding motion action. Interrupted travel still requires replanning. Reset motor
fault (R) uses the same owner, saves evidence and verifies disabled/stationary
feedback; success leaves selections off, the plan invalid and motors disabled.
The panel repeats welding until Pause / Hold by default, as requested Nov 17.
`--cycles` sets the checked preview block; `--single-pass` opts into finite
completion. Continuous execution requires the closed periodic cubic path and
keeps existing focus/heartbeat/fault stops. Pause captures a fixed measured hold.
The Nov 17 controller sweep removes duplicate active feedback/command batches,
keeps the checked speed allowance through welding/holding, and makes repeated
interrupted pauses preserve the first captured hold. Soft pacing now slows at
1 degree and requests a stopped clock at 2 degrees; the 3-degree trip remains.
No drive gains or current settings were changed. An earlier operator journal
records 12.76 s at ready within 0.177 degree on all six fixed targets; the operator
confirmed pushing during later recovered deviations. Loaded stiffness and
post-weld holding remain unqualified under HOLD-01.

The panel now defaults to bounded host PID tracking (`--tracker pid`): velocity
lead, filtered derivative feedback, anti-windup, stage command derivative limits
and checked corrections. Preparation uses nominal time; `--tracker paced` retains
the former shared-clock policy. Finite endpoints shed correction and latch the
original fixed hold. Preparation lead and shaped commands are bounded by the
current leg's exact position extrema, with acceleration-bounded braking before
those extrema; mandatory recovery stops have separate bounds. This fixes a
recorded A6 command overshoot during approach, not the separate A1 oscillation
observed with fixed paused targets. Loaded holding remains unqualified.
Measured per-axis Hz/travel uses cached feedback only.
Weld host PID/lead is omitted for axes below 0.3 deg planned peak-to-peak travel;
their nominal targets remain unchanged. Tiny A2–A5 corrections were constraining
A1/A6 through the shared clearance projection. The tracker-v3 physical report
`weld-1794913974470083980.json` reaches ready in 10.54 s and records 15.58 s of
welding. A1/A6 both measure about 1.731 Hz, with full correction admitted, but
travel exceeds the plan (3.694/2.260 deg and 0.634/0.427 deg). The earlier A6
two-cycle response is absent in that window; amplitude and loaded qualification
remain open. The readout separates measured pattern rate from travel. The latest
report ends with missing feedback during cleanup: disable acknowledgement and
mode restoration are false. Do not infer verified shutdown from completed motion.
Five original reports are indexed in `docs/measurements/weld/2026-11-17/README.md`.
Preparation defaults to 5 deg/s and <=30 deg/s². Do not
widen 0.25-degree arrival or 3-degree tracking tolerances to hide lag. The
staging search is available for hardware with explicit `--optimize-preparation`,
as requested by the operator on Nov 17. It preserves checked limits and full
weld targets. The recorded Nov 16 case reaches ready in 4.62 s with ideal fake
shafts; physical five-second timing remains unqualified. Default speed stays
5 deg/s; the 10 deg/s comparison requires explicit `--max-speed-deg-s 10`.
The operator requested a 100 deg/s preparation setting on Nov 17. The CLI/planner
accept up to 100, still bounded by the existing 68.7549 deg/s motor command guards;
the 1.5 rad/s measured-speed trips and <=30 deg/s² preparation acceleration remain.
The Nov 17 15:50 six-axis disabled/control readback passed; A5 settings match Nov 10
and manual holding/motion are reported normal. Do not assume drive tuning is needed.
`--approach-lift-mm 30` requires a raised-tool route before lowering to the same
weld target; it preserves clearance checks and can exceed five seconds. Physical
coordinated tracking and timing still require testing; see the current handoff.
Weld target remains 2 Hz / 12 mm/s, with all six command caps 68.7549 deg/s.
Historical CMA-ES reached 1.815288 Hz under provisional CAD acceleration; stale
mapping/model/constraint fingerprints require regeneration.

Use the independent A5 candidate with its matching standby; production flags
remain false. Candidate gain magnitude is one because feedback is already geared.
A3 standby is raw 149.39 deg, two degrees inside the corrected CCW endpoint.
Output switches gate commands independently without counterpart compensation.
LOAD-01 and FEEDBACK-01 follow the first qualified arm welding run; no gravity
feedforward, Cartesian correction or ESO is active. JSONL evidence uses cached
owner data without another motor owner or enabled-state register polling.

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
  allowance at startup or wrap equivalent orientations across its bounds. A5's
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
