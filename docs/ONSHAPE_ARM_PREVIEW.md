# FORGE model, static posing, and full simulation

This guide describes the current nathan checkout. The older forge_sim checkout
remains a historical simulation source; it is not required to run these entry points.

## Setup

```bash
cd ~/forge_ws/FORGE-nathan
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-sim.txt
```

MuJoCo 3.13.0 and onshape-to-robot 1.8.3 are pinned. The model is a preview;
missing CAD mass properties, approximate collision geometry and provisional gains
prevent treating successful simulation as validated hardware dynamics.

## Choose the correct viewer

| Command | Behavior |
| --- | --- |
| `.venv/bin/python preview_native.py --renderer gpu` | Static posing with native Joint panels; never steps physics |
| `.venv/bin/python preview_sim.py` | Full physics, wall magnets, wheel driving, arm targets, terrain menu and telemetry |
| `.venv/bin/python preview_arm.py` | Optional isolated arm dynamics: fixed chassis, no gravity or contact |
| `.venv/bin/python -m mujoco.viewer --mjcf=model/scene.xml` | Stock MuJoCo; no custom magnets or static linkage solver |
| `scripts/run_motor_viewer.sh` | Read-only wheel encoder display, not full physics or arm feedback |

For full simulation GPU rendering on WSL:
```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA .venv/bin/python preview_sim.py
```

The native launcher selects NVIDIA D3D12 on WSL unless software rendering was
explicitly requested. Its GPU path was verified on an RTX 4090.
Software fallback: `--renderer software --render-threads 4`.
Rendering options do not make physics GPU-based.

## Static posing

Launch preview_native.py, then click Simulation > Pause or press Space once
to enable Joint editing. Neither UI Run nor Pause advances physics in this launcher.
Drag Joint sliders; Control sliders do not command motion in static mode.
D3's rod and passive bearing are solved kinematically after edits; infeasible
closure edits revert to the last valid pose. This is not collision-aware IK:
the viewer does not block edits based on collision clearance.

The initial camera is rotated 180 degrees from the stock model's default view.
Key 0 > Load key restores the saved pose; native Reset returns the model reference
configuration, which is different from the saved default pose.
The static adapter uses MuJoCo 3.13 private viewer internals; recheck on upgrades.
It does not modify installed packages.

## Joint coordinates and defaults

All angles are radians. Model/initial_pose.json is the authoritative configuration.

| DOF | Joint | Default |
| --- | --- | --- |
| 1 | b2_arm_d1 | 0 |
| 2 | b3_arm_d2 | 0.0942 |
| 3 | b4_arm_d3 | -0.44 |
| 4 | b5_arm_d4 | 0 |
| 5 | b6_arm_d5 | -0.188 |
| 6 | b7_arm_d6 | 0 |

DOF1 zero is rebased: simulation 0 equals original CAD -0.48746289486607497 rad.
The physical aligned pose is preserved. D2/D3/D5 were NOT rebased.
These are simulation coordinates, not commissioned encoder offsets.

The 19 CAD joints are retained; adding a floating chassis gives 20 joints.
There are four wheel velocity actuators and six arm position actuators.
D3's near ball joint is b4_1_arm_rod_d3; b4_2_arm_d3 is the passive bearing.
The closing_b4_2_arm_rod_d3 sites are joined by one connect equality.
Only b4_arm_d3 is the D3 motor command. The rod can spin passively.

## Full simulation

Space runs/pauses; WASD drives; X clears wheel targets; R resets and pauses.
M opens terrain settings; C displays collision geometry and contacts.
Select arm DOF with 1-6; [ and ] adjust its target by five degrees.
The reset keyframe restores default arm targets and zero wheel targets.
Wheel joints remain wheel_1 through wheel_6; wheels 1-4 are driven.
Signs remain (-1,-1,+1,+1). DRIVE_RPM and TURN_RPM are currently 60 and 30;
wheel targets are capped at 75 RPM and actuator torque at 12 Nm.
Arm kp=50 and kv=5 are illustrative, not calibrated Damiao motor behavior.
Default +/-pi/2 arm command ranges are preview limits when CAD provides none.

The 4 m by 4 m steel wall, floor, lighting, magnetic parameters (100 N maximum
per wheel, 20 mm cutoff, 2 mm decay), curved/bumpy terrain and telemetry are retained.
This is not proof of adhesion, static holding, or successful terrain traversal.
Terrain fitting adjusts the base and five passive chassis joints. It preserves the
configured arm pose, solves its closure, checks eligible contacts and writes
terrain_spawn with matching actuator targets. Terrain generation can reject a pose.

## Export and generated files

Put API credentials in ignored .env and the assembly-tab URL in ignored
model/config.json, using the supplied examples. Never commit credentials.

```bash
.venv/bin/python export_onshape.py
.venv/bin/python preview_sim.py --check
```

Export stages downloads, backs up unmodified source XML/meshes under ignored
.sim-backups/raw-*, converts unsupported STL files, omits zero-thickness meshes,
and integrates the old scene features. It merges visual geometry by rigid body
while retaining separate per-part collision geoms. It does not merge across joints.
Validation precedes promotion; promotion failures roll back.
model/omitted_surfaces.json records omissions.

| File | Meaning |
| --- | --- |
| model/cad_robot.xml | Loadable CAD export before simulation names/zero rebasing |
| model/robot.xml | Integrated full robot with stable chassis names and arm zero offset |
| model/scene_template.xml | Authored wall/floor/light/magnetic scene; edit this for persistent scene changes |
| model/scene.xml | Integrated scene with fitted spawn keyframe |
| model/scene_curved.xml, scene_bumpy.xml | Regenerated terrain scenes |
| model/initial_pose.json | Desired joint angles and CAD-to-simulation zero offset |
| model/simulation_name_map.json | CAD-to-existing-simulation chassis name mapping |
| model/assets/ | Source collision meshes and merged visual meshes |
| model/arm_preview.xml | Regenerated optional arm-only scene |

Run integrate_sim.py to rebuild integration from cad_robot.xml without downloading.
Edit scene_template.xml, not generated scene.xml, for changes that must survive export.

## Collision and planning limits

Collision meshes are convex per part; concave gaps may falsely register contact.
Only these preview self-collision body pairs are excluded:
- b2_arm_d1 / l3_arm_linkage_d3
- b4_1_arm_rod_d3 / b4_2_arm_d3

Their collisions with other bodies and the environment remain eligible.
Review these exclusions and refine collision geometry before planning.
Two zero-thickness motor mesh assets are omitted (see omission report); this is
distinct from missing mass properties in other CAD parts.
Magnets use merged visual wheel envelopes; physical contact uses per-part geoms.
Mink and MoveIt 2 are not installed or integrated by this workflow.

## Compatibility with hardware and ROS

The wheel-feedback viewer accepts this scene via --scene and preserves its
terrain_spawn pose for all unmeasured joints. It updates only four wheel angles;
the arm pose is not measured arm feedback and no physics steps run.
Its setup still defaults to the pinned old forge_sim model. Use
`bash scripts/setup_motor_viewer.sh --local-model` for a local model snapshot;
rerun after export, or pass an absolute --scene path for the live checkout model.
Private configuration and credentials are not copied.

Damiao profiles retain their hardware coordinates, placeholder joint mapping,
and uncommissioned status. The simulation DOF1 zero offset is NOT applied to
hardware. robot_description has inventories and Xacro macros, not a completed
six-axis MoveIt robot with the D3 loop integrated.

## Verification

```bash
.venv/bin/python preview_native.py --check
.venv/bin/python preview_sim.py --check
.venv/bin/python preview_sim.py --terrain curved --check
.venv/bin/python preview_sim.py --terrain bumpy --check
.venv/bin/python -m unittest discover -s tests -p 'test_model_compatibility.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_magnetic.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_velocity_control.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_terrain.py' -v
```

These establish software consistency and short-run numerical behavior, not physical
accuracy, complete motion-range clearance, or hardware safety.
