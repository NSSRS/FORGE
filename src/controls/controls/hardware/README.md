# Weld execution layer

This layer owns real-motor execution and its fake-motor test mode. Offline
planning is in sibling `../controller/`; the original dynamic simulation is in
`../simulation/`. It is usable through the CLI and the ROS 2 `control_weld` entry
point; production lifecycle/atomic group execution remain separate acceptance work.

| Module | Responsibility |
| --- | --- |
| `weld_control.py` | State machine, PID/legacy-paced execution, arrival, fixed endpoint hold, drift observation, pause/fault handling |
| `weld_ports.py` | Damiao `ArmOwner` / `TrajectoryArmPort`, ENCOS `ChassisOwner`, and `SimulatedArm` |
| `weld_app.py` | Qt panel, planning process and sole execution/transport thread |
| `weld_recording.py` | Bounded asynchronous JSONL writer; copies owner-maintained cached evidence |
| `weld_plan.py` | Compatibility exports from `controller/weld_plan.py` |
| `simulation.weld_preview` | Separate renderer process; receives poses, never transport handles |

## Adapter contract

The execution thread owns `begin_cycle`, `poll`, feedback reads, activation, `command`,
`finish_trajectory`, pause, disable and close. Planner/GUI work cannot acquire
another owner. `Feedback` uses ordered physical slots A1-A6, raw shaft degrees,
deg/s and age in ms; adapters convert to radians/rad/s at the motor API boundary.

Arm commands follow `WeldController -> ArmOwner -> TrajectoryArmPort ->
ArmGroupHoldPort -> ArmBus -> DM-MC02 -> CAN`. Native mode 2 / USB type 4 carries
position and speed allowance. The default host PID tracker adds bounded position
lead using reference velocity and measured error; the drive velocity field
remains a ceiling. No torque feedforward, gain writes or type-25 jog smoothing
are added. Complete targets are validated before sequential dispatch;
this is not firmware-atomic multi-axis application.

Moving ticks use `begin_cycle` to read/validate six fresh samples and `command`
to consume them in one target dispatch, without a duplicate poll/old-target
refresh. Dispatch rechecks sample age after calculation. Standalone `command`
reads its own batch when none is pending; idle `poll` reads and refreshes fixed
holds. Each stage keeps its checked mode-2 speed allowance through arrival.

`--tracker pid` keeps nominal preparation time, shapes corrected command speed
and acceleration, checks corrected targets and removes offsets before endpoint
hold. Finite preparation commands brake inside the current leg's exact extrema;
mandatory recovery stops have separate bounds. Weld correction is enabled only
for axes with at least 0.3 deg planned peak-to-peak travel; all nominal axis
trajectories remain commanded. This keeps tiny-axis lag estimates from consuming
the shared clearance margin. `--tracker paced` retains the old common-clock algorithm. The reusable
`WeldController` accepts explicit `TrackingSettings`; callers omitting it retain
legacy behavior. Tracking/frequency calculations consume the same feedback batch.
The table distinguishes planned weave Hz from measured per-axis mechanical Hz
and peak-to-peak travel. Journal/report records retain tracker settings,
unmodified reference, dispatched command, correction, saturation and lag estimate.

Chassis commands use the existing `DifferentialDrive -> MotorBus` owner with a
complete four-wheel mapping. Participation switches gate outputs only, without
compensation for an inactive counterpart.

## Execution and hold semantics

Measured pose -> verify current-position hold -> checked recovery waypoints ->
approach -> ready -> explicit weld until pause (or complete in single-pass mode).
`finish_trajectory()` is called
only after the controller's measured arrival/settling check. It latches the
already-dispatched endpoint and speed allowance, clears trajectory intent, and
continues native command refresh and protection. A pause at that settled endpoint
preserves the fixed target; an interrupted move captures measured hold targets
once, retaining its allowance. Repeated pauses do not recapture passive drift.
Disable/fault/close remove torque and do not auto-reset or re-enable.

At `ready`, pause, focus loss and UI heartbeat loss retain the accepted plan and
Start welding motion action while holding. Only a fresh explicit start can begin
welding; stationary feedback, arrival tolerance and clearance are rechecked.
Interrupted travel, selection changes, recalculation, disable, close and faults
invalidate the plan. Focus generations reject starts queued before focus changed.

Reset motor fault (R) calls `WeldController.reset_motor_fault()` through the same
owner thread. `ArmOwner.recover()` delegates to the existing disabled/stationary
recovery and evidence writer. Faulted startup retains that owner for explicit
reset. Success leaves motors disabled, selections off and the plan invalid;
failure preserves the original fault. Commands during recovery are discarded.

The panel enables `continuous_weld=True`: the checked periodic cubic weld block
repeats until pause, with uninterrupted chassis feed when selected. Phase wraps
without a completion/settling step or another enable. Only the repeating weld
bypasses the finite-pass arrival timeout; all live guards remain active. Pause
captures one measured hold. `--single-pass` opts into finite completion; direct
`WeldController` callers retain finite execution unless they request continuous
welding. `--cycles` controls the checked block and preview, not the continuous
run length. Session records include the selected run policy.

This fixed-reference hold is not a stiffness-gain setting. Physical sag and drive
stiffness are tracked under HOLD-01. Ready/complete observations use cached feedback
and record per-axis error; they issue no extra commands or register reads.

## Running and verification

Use `scripts/control_weld.py --plan-only` for offline planning, `--simulate` for
fake motors, and explicit `--execute` for reviewed hardware sessions. Selection
and planning do not enable motors. Preview playback is separate from execution.
`--optimize-preparation` compares checked staging poses and explicitly permits
their use on hardware. `WeldController` requires `allow_optimized_preparation=True`
to accept these plans with physical outputs; loading never enables motors.
Historical plans marked `offline_only` remain rejected. Five seconds is a goal,
not a deadline; the recorded comparison uses explicit 10 deg/s preparation.
Generated plans/logs stay ignored. Explicitly preserved session reports are
indexed in [the evidence archive](../../../../docs/measurements/weld/2026-11-17/README.md).

See [WELD_MOTION.md](../../../../docs/WELD_MOTION.md) for limits/search/timing,
[ARM_RIG_TEST.md](../../../../docs/ARM_RIG_TEST.md) for operator procedures,
[the handoff](../../../../docs/ARM_SESSION_HANDOFF.md) for current evidence, and
[tests/README.md](../../../../tests/README.md) for native fake-CAN, model and Qt
verification. Loaded holding, shutdown and five-second prep require physical evidence.
