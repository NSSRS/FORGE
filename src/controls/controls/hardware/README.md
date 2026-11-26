# Weld execution layer

This layer owns real-motor execution and its fake-motor test mode. Offline
planning is in sibling `../controller/`; the original dynamic simulation is in
`../simulation/`. It is usable through the CLI and the ROS 2 `control_weld` entry
point; production lifecycle integration remains separate work.

| Module | Responsibility |
| --- | --- |
| `weld_control.py` | State machine, PID/legacy-paced execution, arrival, fixed endpoint hold, drift observation, pause/fault handling |
| `weld_ports.py` | Damiao `ArmOwner` / `TrajectoryArmPort`, ENCOS `ChassisOwner`, and `SimulatedArm` |
| `weld_app.py` | Qt panel, planning process and sole execution/transport thread |
| `weld_recording.py` | Bounded asynchronous JSONL writer; copies owner-maintained cached evidence |
| `hold_analysis.py` | Offline per-attempt/stage timing, speed, torque, temperatures, fixed-target error and evidence-quality summaries |
| `weld_plan.py` | Compatibility exports from `controller/weld_plan.py` |
| `simulation.weld_preview` | Separate renderer process; receives poses, never transport handles |

## Adapter contract

The execution thread owns `begin_cycle`, `poll`, feedback reads, activation, `command`,
`finish_trajectory`, pause, disable and close. Planner/GUI work cannot acquire
another owner. `Feedback` uses ordered physical slots A1-A6, raw shaft degrees,
deg/s and age in ms; adapters convert to radians/rad/s at the motor API boundary.

Arm commands follow `WeldController -> ArmOwner -> TrajectoryArmPort ->
ArmGroupHoldPort -> ArmBus -> DM-MC02 -> CAN`. Native mode 2 carries position
and speed allowance. Firmware with capability `0x04000000` receives one complete
type-30 batch; older firmware retains sequential type 4. The report records
`group_command_transport` and `usb_command_type`. An explicit `--weld-response-profile` selects the
operator-selected v7 bounded periodic A1/A6 response baseline, with the original
quintic first-cycle blend and capped residual feedback (0.20/0.05 deg).
Its exact physical-run profile is restored; preparation is unchanged.
The plan, preview and accuracy analyzer share `weld_entry`; only periodic lag
fitting excludes startup.
The provisional profile requires the recorded arm-only pose, amplitude,
frequency, mapping and scene. See `docs/ARM_RIG_TEST.md` for its command.
The default host PID tracker adds bounded position
lead using reference velocity and measured error; the drive velocity field
remains a ceiling. No torque feedforward, gain writes or type-25 jog smoothing
are added. Complete targets are validated before dispatch. Type 30 validates all
six slots again in the MCU, then updates all targets and leases before the next
control tick. CAN frames remain serialized. A failed or uncertain batch stops
through the existing owner without retry or fallback. Intent loss around a
successful dispatch retains the existing measured pause/hold behavior. See
[the protocol contract](../../../../docs/ARM_BASE_LAYER.md#base-06-atomic-group-targets).

Moving ticks use `begin_cycle` to read/validate six fresh samples and `command`
to consume them in one target dispatch, without a duplicate poll/old-target
refresh. Dispatch rechecks sample age after calculation. Standalone `command`
reads its own batch when none is pending; idle `poll` reads and refreshes fixed
holds. Each stage keeps its checked mode-2 speed allowance through arrival.

`--tracker pid` slows preparation time when shaft error grows, shapes corrected command speed
and acceleration, checks corrected targets and removes offsets before endpoint
hold. Finite preparation commands brake inside both the current leg's exact extrema
and the 12-degree correction envelope;
mandatory recovery stops have separate bounds. Weld correction is enabled only
for axes with at least 0.3 deg planned peak-to-peak travel; all nominal axis
trajectories remain commanded. This keeps tiny-axis lag estimates from consuming
the shared clearance margin in enforced spatial mode. Welding retains its
nominal clock. The sampling fix retained in v7 advances by exact sampled
reference increments, then shapes residual correction;
it does not integrate the current reference velocity again. Sampled command
rates use interval-midpoint spacing for acceleration checks on irregular ticks.
Preparation pursuit and all existing ceilings remain unchanged. The A1-only
comparison switch still disables only its additive weld correction; other
enabled-axis PID lead is not retuned by this command-generation fix.
`--tracker paced` uses the common clock without PID lead. The reusable
`WeldController` accepts explicit `TrackingSettings`; callers omitting it retain
legacy behavior. Tracking/frequency calculations consume the same feedback batch.
The table distinguishes planned weave Hz from measured per-axis mechanical Hz
and peak-to-peak travel. Journal/report records retain tracker settings,
unmodified reference, dispatched command, correction, saturation and lag estimate.

Chassis commands use the existing `DifferentialDrive -> MotorBus` owner with a
complete four-wheel mapping. Participation switches gate outputs only, without
compensation for an inactive counterpart.

## Execution and hold semantics

Measured pose -> verify current-position hold -> optional checked A3 endpoint
retreat -> recovery waypoints -> approach/lowering -> Ready -> explicit weld
until pause (or complete in single-pass mode). The exceptional retreat has its
own inward-only guard and strict return to normal limits; see the shared
`controller/endpoint_recovery.py` policy.
For hardware, `controller/measured_start.py` accepts the observed start outside
saved pose windows or authored simulator joint ranges. Planning, live recovery
checks and the native owner share its bounded temporary extent. Start contact
is a warning. By operator request, model spatial clearance is advisory throughout
hardware planning and execution, including tracker correction; simulation keeps
enforced spatial checks. Explicit mechanical stops, D3 closure and A4/A5/A6 cable bounds
remain fixed. Both actual and commanded endpoints must pass normal checks before
the owner removes the allowance and enters approach. Simulation stays strict.
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

This fixed-reference hold is not a stiffness-gain setting. HOLD-01 was
[resolved by operator confirmation on Nov 23](../../../../docs/ARM_SESSION_HANDOFF.md#hold-01-resolution).
Ready/complete observations use cached feedback
and record per-axis error; they issue no extra commands or register reads.

## Running and verification

Use `scripts/control_weld.py --plan-only` for offline planning, `--simulate` for
fake motors, and explicit `--execute` for reviewed hardware sessions. Selection
and planning do not enable motors. Preview playback is separate from execution.
`--optimize-preparation` compares checked staging poses and explicitly permits
their use on hardware. `WeldController` requires `allow_optimized_preparation=True`
to accept these plans with physical outputs; loading never enables motors.
Historical plans marked `offline_only` remain rejected. Five seconds is a goal,
not a deadline. The current loaded trial explicitly uses 30 deg/s, a 10 s goal,
a 30 mm raised approach and 2.3 s final-lowering minimum; defaults remain unchanged.
Generated plans/logs stay ignored. Explicitly preserved session reports are
indexed in [the evidence archive](../../../../docs/measurements/weld/2026-11-17/README.md).
Normal launches need no `--output`: `build/weld/latest/session.jsonl` exposes the
live journal and `build/weld/latest/session.json` the final report after close.
Each run writes to its own directory under `build/weld/sessions/`; the latest
link changes atomically without redirecting an earlier writer or deleting evidence.
An explicit `--output` remains available for custom unused report paths.

See [WELD_MOTION.md](../../../../docs/WELD_MOTION.md) for limits/search/timing,
[ARM_RIG_TEST.md](../../../../docs/ARM_RIG_TEST.md) for operator procedures,
[the handoff](../../../../docs/ARM_SESSION_HANDOFF.md) for current evidence, and
[tests/README.md](../../../../tests/README.md) for native fake-CAN, model and Qt
verification. The Nov 23 final close verified disable/mode restoration;
the selected stationary [BASE-07 check](../../../../docs/ARM_SESSION_HANDOFF.md#base-07-command-loss-check)
is also recorded. The
[weld accuracy test](../../../../docs/ARM_RIG_TEST.md#weld-motion-accuracy-test)
uses cached trajectory records from this owner.
