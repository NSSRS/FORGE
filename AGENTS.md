# Workspace guidance

## Start here

Read `README.md`, then `docs/ARM_SESSION_HANDOFF.md` for current arm status and
the next session. Use `docs/ARM_LAYER_TODOS.md` and the owning package's `TODO.md`
for open work; remove completed tasks instead of appending incident histories.
Operator procedures belong in `docs/ARM_RIG_TEST.md`; architecture and ownership
are in `docs/ARM_BASE_LAYER.md` and `docs/SOURCE_LAYOUT.md`.

Current arm status (2026-11-13): all six motors are connected. The Nov 12 eight
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

Jitter research is deferred under SPEED-01. A2/A3 move smoothly by operator
report; A1/A4/A5/A6 jitter during travel but hold steady. The 5 deg/s smoothing
trial did not fix it. Holding-mode jogging now defaults to 10 deg/s by explicit
operator request; non-holding/teaching remains 5 deg/s. All six measured-speed
trips remain 1.5 rad/s. Current protection, loaded holding and physical stopping
remain unqualified. Do not increase reactive trips or claim jitter resolved.
The operator confirmed the earlier A1 undervoltage issue was fixed.

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
