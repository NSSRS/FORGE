# Workspace guidance

## Start here

Read `README.md`, then `docs/ARM_SESSION_HANDOFF.md` for current arm status and
the next session. Use `docs/ARM_LAYER_TODOS.md` and the owning package's `TODO.md`
for open work; remove completed tasks instead of appending incident histories.
Operator procedures belong in `docs/ARM_RIG_TEST.md`; architecture and ownership
are in `docs/ARM_BASE_LAYER.md` and `docs/SOURCE_LAYOUT.md`.

Current arm status (2026-11-12): A1 is reconnected; all six disabled identity/
feedback checks passed. The operator completed eight requested raw recordings:
A1 front/initial, A2 CW and CCW ends, A3 CW and CCW ends, A4/A5/A6 initial.
The dated reference JSON under `src/robot_description/config/` preserves the
operator's readings; `docs/measurements/` preserves the matching full-precision
capture and final cleanup summary. Read the handoff for exact angles. No values
were installed as runtime limits, motor zero or automatic startup targets.
A2/A3 initial poses, other joints' bounds, model sign/offset/ratio and power-cycle
repeatability remain open under CAL-01. Keep unknown fields null and production
flags false. The operator input `-174,692` means A1 -174.692 degrees, corroborated
by the original capture.

The same saved session verifies firmware `d61f6d16`, capabilities `0xFFF8F`, but
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
