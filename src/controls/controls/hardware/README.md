# Weld execution layer

This layer owns real-motor execution and its fake-motor test mode. Offline
planning is in sibling `../controller/`; the original dynamic simulation is in
`../simulation/`. It is usable through the CLI and the ROS 2 `control_weld` entry
point; production lifecycle/atomic group execution remain separate acceptance work.

| Module | Responsibility |
| --- | --- |
| `weld_control.py` | State machine, feedback pacing, arrival, fixed endpoint hold, drift observation, pause/fault handling |
| `weld_ports.py` | Damiao `ArmOwner` / `TrajectoryArmPort`, ENCOS `ChassisOwner`, and `SimulatedArm` |
| `weld_app.py` | Qt panel, planning process and sole execution/transport thread |
| `weld_recording.py` | Bounded asynchronous JSONL writer; copies owner-maintained cached evidence |
| `weld_plan.py` | Compatibility exports from `controller/weld_plan.py` |
| `simulation.weld_preview` | Separate renderer process; receives poses, never transport handles |

## Adapter contract

The execution thread owns `poll`, feedback reads, activation, `command`,
`finish_trajectory`, pause, disable and close. Planner/GUI work cannot acquire
another owner. `Feedback` uses ordered physical slots A1-A6, raw shaft degrees,
deg/s and age in ms; adapters convert to radians/rad/s at the motor API boundary.

Arm commands follow `WeldController -> ArmOwner -> TrajectoryArmPort ->
ArmGroupHoldPort -> ArmBus -> DM-MC02 -> CAN`. Native mode 2 / USB type 4 carries
position and speed allowance; no torque/feedforward, gain writes or type-25 jog
smoothing are added. Complete targets are validated before sequential dispatch;
this is not firmware-atomic multi-axis application.

Chassis commands use the existing `DifferentialDrive -> MotorBus` owner with a
complete four-wheel mapping. Participation switches gate outputs only, without
compensation for an inactive counterpart.

## Execution and hold semantics

Measured pose -> verify current-position hold -> checked recovery waypoints ->
approach -> ready -> explicit weld -> complete. `finish_trajectory()` is called
only after the controller's measured arrival/settling check. It latches the
already-dispatched endpoint and speed allowance, clears trajectory intent, and
continues native command refresh and protection. A pause at that settled endpoint
preserves the fixed target; an interrupted move captures measured hold targets.
Disable/fault/close remove torque and do not auto-reset or re-enable.

This fixed-reference hold is not a stiffness-gain setting. Physical sag and drive
stiffness are tracked under HOLD-01. Ready/complete observations use cached feedback
and record per-axis error; they issue no extra commands or register reads.

## Running and verification

Use `scripts/control_weld.py --plan-only` for offline planning, `--simulate` for
fake motors, and explicit `--execute` for reviewed hardware sessions. Selection
and planning do not enable motors. Preview playback is separate from execution.
`--optimize-preparation` compares staging poses offline and is rejected by physical
owners. Generated plans/logs are not distributed through Git.

See [WELD_MOTION.md](../../../../docs/WELD_MOTION.md) for limits/search/timing,
[ARM_RIG_TEST.md](../../../../docs/ARM_RIG_TEST.md) for operator procedures,
[the handoff](../../../../docs/ARM_SESSION_HANDOFF.md) for current evidence, and
[tests/README.md](../../../../tests/README.md) for native fake-CAN, model and Qt
verification. Loaded holding, shutdown and five-second prep require physical evidence.
