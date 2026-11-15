# Coordinated controls

[The Shape 1 trajectory](controls/weave_xy_z.py) produces wire positions and
velocities for the original simulation. The hardware planner preserves the same
stroke/dwell intent with quintic segments and automatically retimed joint splines. The [MuJoCo controller](controls/simulation/weave_xy_z.py)
drives the simulated chassis and arm. The
[supported hardware controller](../../docs/WELD_MOTION.md) now uses the shared
trajectory with existing ENCOS/Damiao owners, independent participation switches,
and the initial -> weld start -> explicit weld-motion sequence. Offline solving
lives in `controls/controller/`: measurement-derived offset alignment, Damiao
physical envelopes, MuJoCo closed-chain IK and automatic trajectory retiming.
Its default is a measured-bound controller standby; original sources are retained.
Physical qualification is still pending.

## Control package layout and hardware boundary

The package is named `controls`. Actual motor execution is in `hardware/`,
at the same level as `simulation/`; `controller/` is their shared offline
planning layer:

```text
src/controls/controls/
  simulation/                 Original MuJoCo motion controller
  hardware/                   Real-motor execution and its offline test mode
    weld_ports.py             Damiao arm and ENCOS chassis owner adapters
    weld_control.py           Initial -> approach -> ready -> weld -> hold
    weld_app.py               Panel; explicit real/simulated owner selection
    weld_plan.py              Compatibility imports from controller/weld_plan.py
  controller/                 Shared calibration, limits, path planning, CMA-ES
```

The [hardware directory guide](controls/hardware/README.md) identifies the
physical command path. `--execute` selects real owners; `--simulate` selects
fake motors while running the same execution state machine. The exported
CMA-ES animation is a third mode: `simulation/weld_animation.py` in the
separate simulation package draws the saved controller plan kinematically.
It does not call the real motor adapters or display measured hardware motion.
The exact animated plan was separately replayed through `WeldController` with
fake shafts. These checks establish shared planning/execution code, not loaded
physical tracking. The renderer is kept with model/rendering code as required
by the repository layout.

Run `.venv/bin/python scripts/control_weld.py --simulate --profile weld --cycles 1` from the
workspace root. No hand-selected time scale is needed. The solver compares fixed
D1 and free-D1 candidates; `--lock-base-yaw` preserves the older fixed-D1 choice.
Arm/chassis switches only gate output and never change the nominal arm path.
The original weld target is 2 Hz / 12 mm/s; 1.73 Hz is an accepted fallback.
Temporary per-axis caps in
`config/arm_motion_policy.json` have been withdrawn: all six axes are null.
The original physical/protection constraints give 68.7549 deg/s weld commands
with unchanged 1.5 rad/s measured-speed trips. These are project guard values,
not factory motor register limits or manufacturer rated speeds. The solver weights
joint motion by available speed and reduces frequency/feed together when needed.
Initial and approach stages retain their 5 deg/s policy. Without `--profile weld`, the default debug profile
keeps low speed/acceleration throughout. No physical speed trip is increased.
The checked CMA-ES candidate reaches 1.815288 Hz / 10.891728 mm/s under
provisional CAD acceleration bounds. No physical welding acceptance has been
completed. LOAD-01 weight compensation and FEEDBACK-01 sensor/ESO feedback
follow that first run; neither is active in the hardware controller.
The experiment described below is the original simulation baseline.

`scripts/optimize_weld.py` adds offline multi-start CMA-ES over joint mobility,
center yaw and wrist IK seeds. Larger mobility favors use of that joint; lower
speed/acceleration and fewer reversals are rewarded at the same requested clock.
Configured caps and all existing collision/closure/torque checks remain enforced.
See [search setup, saved candidates and reuse](../../docs/WELD_MOTION.md#offline-cma-es-weight-and-branch-search).
Reuse an accepted candidate with `control_weld.py --profile weld --solution PATH`;
it checks identity and regenerates the entire route before motion.

## WEAVE_XY_Z = 1 on a 20 mm floor seam

The ENCOS wheel drives feed the chassis along +X at a requested 12 mm/s. The arm
weaves the APW503 wire tip from −10 to +10 mm across the centered seam, 2 mm
above the wheel-contact plane. The gun, bracket and wire stay rigidly fixed to
DOF6 in their exported CAD positions. DOF1 holds its zero target and DOF2–6
move the mounted tool. The chassis starts at X = 0 and the seam at X = −440 mm.
Both are centered on Y = 0.

## Wire direction

The experiment assumes a flat butt joint between coplanar plates. Looking along
the seam, the wire points straight down (90° transverse work angle). From the
side, the wire points down and outward toward −X, 10° from vertical, or 80° to
the floor. With the cart feeding +X this is a drag angle. A 45° transverse work
angle is typical for an equal-leg fillet between perpendicular plates.
[Miller MIG joint guide](https://www.millerwelds.com/en-US/resources/knowledge-hub/mig-welding/how-to/mig-basics-for-automotive-welding-repair)

## Run the moving preview

```bash
.venv/bin/python scripts/preview_weave.py --hz 2 --cycles 16 --loop
```

After building and sourcing the controls ROS package in the simulation Python environment,
use `ros2 run controls preview_weave --hz 2 --cycles 16 --loop`.

On WSLg the viewer requests the NVIDIA GPU through Mesa D3D12 and prints the
active OpenGL renderer when the window opens. The viewer stops on a software
fallback. Physics and arm control run on the CPU.

Amber shows the commanded path and teal shows the measured wire tip. The
overlay reports cart travel, lateral wire position, DOF6 angle and wire
inclination. Use `--amplitude-mm 0` for a straight pass. Use `--plan` to inspect
a stationary start pose. `--wire-to-floor-deg` changes the longitudinal target
with the clamp fixed. An unreachable centered pose stops the moving run.

## Simulated results

Six cycles per frequency, 20 mm commanded stroke. Errors are measured in 3D.

| Frequency | Cart travel | Tip stroke | RMS / peak tip error |
| --- | ---: | ---: | ---: |
| 1 Hz | 70.4 mm | 20.17 mm | 0.029 / 0.116 mm |
| 2 Hz | 34.4 mm | 20.35 mm | 0.078 / 0.226 mm |
| 3 Hz | 22.4 mm | 20.61 mm | 0.149 / 0.375 mm |

The measured wire inclination was 79.97–80.03°. DOF6 swept 23.7–25.4° while
the gun stayed fixed in the clamp. The largest seam-edge overshoot was 0.32 mm
at 3 Hz. The 16-cycle viewer run at 2 Hz advanced the cart 94.5 mm.

## Tester handoff

1. Confirm the joint type and the permitted travel angle and direction for the
   weld procedure. Check the CAD neck direction, cable clearance and wire
   extension against the installed tool, leaving it fixed in the clamp.
2. Run the straight pass, then the three frequencies below. In the side view
   check the outward lean. In the top view check the centered stroke. Compare
   tip position, DOF6 travel, cart motion and D3 closure in the CSV. Record the
   accepted tracking tolerance for the intended seam. These are motion trials.
3. For the physical motor checks, follow the [pose and speed trial guide](../../docs/ARM_POSE_TEST.md).

```bash
mkdir -p build/weave
for hz in 1 2 3; do
  .venv/bin/python scripts/preview_weave.py --hz "$hz" --cycles 6 --csv "build/weave/${hz}Hz.csv"
done
```
