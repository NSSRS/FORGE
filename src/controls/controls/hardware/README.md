# Physical motor welding control

This directory is the `controls` package's hardware execution layer, alongside
`../simulation/`. The shared `../controller/` layer provides calibration,
speed/acceleration constraints, trajectory solving and CMA-ES search.
Hardware execution is implemented; loaded welding motion is not physically qualified.

| File | Responsibility |
| --- | --- |
| `weld_ports.py` | `ArmOwner` / `TrajectoryArmPort` connect the six Damiao axes; `ChassisOwner` connects the ENCOS chassis; `SimulatedArm` supports offline tests |
| `weld_control.py` | Shared execution state machine: measured pose → hold verification → standby → weld start → explicit start → welding motion → hold; feedback, arrival, pause and fault handling |
| `weld_app.py` | Control panel and single execution thread; hardware or simulated adapters with independent arm/chassis output switches |
| `simulation.weld_preview` | Embedded current/target poses and accepted-path playback; renderer process without a motor owner |
| `weld_plan.py` | Compatibility exports; the solver lives in `../controller/weld_plan.py` so there is one planning implementation |

## Physical motor command path

Arm commands pass through `WeldController` → `ArmOwner.command()` →
`TrajectoryArmPort.set_targets()` → the existing `ArmGroupHoldPort` / Damiao bus
→ DM-MC02 → CAN motors. Commands specify raw output-shaft positions and speeds
for all six axes using native position-velocity mode 2 / USB type 4. They do not
add the manual jogger's type-25 smoothing. Axis messages are sent sequentially;
dispatch is not atomic across all six axes in firmware.

Chassis commands pass through `WeldController` → `ChassisOwner.velocity()` →
`DifferentialDrive` four-wheel allocation → `MotorBus.velocity()` → the existing
ENCOS driver. Output switches gate commands; disabling chassis output does not
add arm compensation.

Hardware adapters are selected only with `scripts/control_weld.py --execute`,
explicit `--arm` / `--chassis` selection, and the existing fixture/wheel-mapping
confirmation arguments. Opening the hardware panel does not start welding:
select participation, calculate a path from fresh feedback, start recovery to
verify hold and reach standby/weld start, then explicitly start welding motion.
The application does not energize the welder. The operator performs hardware
work; see the [weld procedure](../../../../docs/WELD_MOTION.md).

## Animation, offline execution and hardware modes

- `--plan-only`: uses the hardware path's solver to generate and check a complete
  trajectory without creating motor owners.
- `--simulate`: runs the same `WeldController` with simulated motors to check
  sequencing, commands, feedback and stopping logic.
  `--start-report PATH` optionally seeds fake shafts from a saved disabled
  six-axis diagnostic; hardware rejects this historical-pose option.
- The embedded **Play preview** button animates a checked route without starting
  execution. Current/standby views remain available for blocked plans. Rendering
  consumes cached poses in a separate process; `--no-preview` disables it.
- `scripts/render_weld.py`: replays a saved trajectory and renders the MuJoCo
  model without motor owners. It is kinematic playback, not recorded physical
  motion or loaded dynamics simulation.
- `--execute`: selects the existing hardware adapters. The earlier animation and
  fake-motor tests did not execute this mode.

The saved animation uses `build/weld/cmaes-animation-plan.json`; its complete
12-cycle sequence has also passed a separate simulated-motor state-machine
replay. The project target remains 2 Hz; the checked plan reaches about 1.8153 Hz
under current model constraints. Both profiles default to 5°/s for
standby/approach; all six welding speed
caps are 68.7549°/s.

Preparation retains a steady checked speed allowance for the motor's position
loop, and `controller/preparation.py` slows the common path clock for lagging axes.
The clock slew uses remaining acceleration headroom; targets stay on the original
path. Arrival, collision, speed/torque trips and explicit restart contracts remain
in force. A1 recovery jitter is still physically unresolved; this is not measured
backlash compensation or torque feedforward.
The preparation budget is now 5 seconds (`--prep-seconds`); it is a target,
not a deadline that can falsely complete or truncate a lagging move. Planning
reports nominal time and an optimistic limits-only lower bound. Paced motion
uses a per-axis 10-second progress watchdog and a separate 10-second final
settling timeout. The Nov 16 A5 physical-motion report remains unresolved.
`weld_recording.py` writes bounded cached evidence to JSONL from another thread,
without transport requests. Shutdown cannot replace the primary fault reason.

Weight compensation follows the first arm welding run under LOAD-01. Sensor
position feedback and ESO evaluation are tracked under FEEDBACK-01. Motor-side
position control and host tracking checks exist; external torch-position
correction, load feedforward and ESO are not installed.

Adapter verification is in `tests/test_weld_hardware.py`: native C firmware with
fake CAN motors checks Damiao group commands, pause/hold and feedback loss;
the ENCOS Python simulator checks chassis feed. These tests do not qualify
physical loads, bus timing or trajectory tracking. Simulated shafts have no
gravity/load model, so hanging-start load pickup remains unqualified.
