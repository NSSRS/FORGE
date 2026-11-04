# FORGE model, static posing, and full simulation

This guide describes the full-robot model shared by main and nathan. The older forge_sim checkout
remains a historical simulation source; it is not required to run these entry points.

## Setup

```bash
cd /path/to/FORGE
python3 -m venv .venv
.venv/bin/python -m pip install -r src/simulation/requirements.txt
```

MuJoCo 3.13.0 and onshape-to-robot 1.8.3 are pinned. The model is a preview;
missing CAD mass properties, approximate collision geometry and provisional gains
prevent treating successful simulation as validated hardware dynamics.

## Choose the correct viewer

| Command | Behavior |
| --- | --- |
| `.venv/bin/python scripts/preview_native.py --renderer gpu` | Static posing with native Joint panels; never steps physics |
| `.venv/bin/python scripts/preview_sim.py` | Full physics, wall magnets, wheel driving, arm targets, terrain menu and telemetry |
| `.venv/bin/python scripts/preview_arm.py` | Optional isolated arm dynamics: fixed chassis, no gravity or contact |
| `.venv/bin/python -m mujoco.viewer --mjcf=src/robot_description/mujoco/scene.xml` | Stock MuJoCo; no custom magnets or static linkage solver |
| `scripts/run_motor_viewer.sh` | Read-only wheel encoder display, not full physics or arm feedback |

For full simulation GPU rendering on WSL:
```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA .venv/bin/python scripts/preview_sim.py
```

The native launcher selects NVIDIA D3D12 on WSL unless software rendering was
explicitly requested. Its GPU path was verified on an RTX 4090.
Software fallback: `--renderer software --render-threads 4`.
Rendering options do not make physics GPU-based.

## Static posing

Launch scripts/preview_native.py, then click Simulation > Pause or press Space once
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

All angles are radians. src/robot_description/mujoco/initial_pose.json is the authoritative configuration.

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

Put API credentials in ignored src/simulation/.env and the assembly-tab URL in ignored
src/robot_description/mujoco/config.json, using the supplied examples. Never commit credentials.

```bash
.venv/bin/python scripts/export_onshape.py
.venv/bin/python scripts/preview_sim.py --check
```

Export stages downloads, backs up unmodified source XML/meshes under ignored
build/simulation/backups/raw-*, converts unsupported STL files, omits zero-thickness meshes,
and integrates the old scene features. It merges visual geometry by rigid body
while retaining separate per-part collision geoms. It does not merge across joints.
Validation precedes promotion; promotion failures roll back.
src/robot_description/mujoco/omitted_surfaces.json records omissions.

| File | Meaning |
| --- | --- |
| src/robot_description/mujoco/cad_robot.xml | Loadable CAD export before simulation names/zero rebasing |
| src/robot_description/mujoco/robot.xml | Integrated full robot with stable chassis names and arm zero offset |
| src/robot_description/mujoco/scene_template.xml | Authored wall/floor/light/magnetic scene; edit this for persistent scene changes |
| src/robot_description/mujoco/scene.xml | Integrated scene with fitted spawn keyframe |
| src/robot_description/mujoco/scene_curved.xml, scene_bumpy.xml | Regenerated terrain scenes |
| src/robot_description/mujoco/initial_pose.json | Desired joint angles and CAD-to-simulation zero offset |
| src/robot_description/mujoco/simulation_name_map.json | CAD-to-existing-simulation chassis name mapping |
| src/robot_description/mujoco/assets/ | Source collision meshes and merged visual meshes |
| src/robot_description/mujoco/arm_preview.xml | Regenerated optional arm-only scene |

Run scripts/integrate_sim.py to rebuild integration from cad_robot.xml without downloading.
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
Run `bash scripts/setup_motor_viewer.sh` once for its Python environment.
The viewer reads src/robot_description/mujoco/scene.xml directly; restart it after CAD updates.
Use --scene to explicitly select another scene. The historical chassis-only
fetch and --local-model snapshot option have been removed. Existing ignored
build/motor_viewer/model snapshots are no longer used.

Arm preview coordinates are simulation-only; no arm hardware driver or calibrated encoder mapping is included.

## Verification

```bash
.venv/bin/python scripts/preview_native.py --check
.venv/bin/python scripts/preview_sim.py --check
.venv/bin/python scripts/preview_sim.py --terrain curved --check
.venv/bin/python scripts/preview_sim.py --terrain bumpy --check
.venv/bin/python -m unittest discover -s tests -p 'test_model_compatibility.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_motor_overview_models.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_body_colors.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_magnetic.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_velocity_control.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_terrain.py' -v
```

These establish software consistency and short-run numerical behavior, not physical
accuracy, complete motion-range clearance, or hardware safety.

## Moving-part colors

Every rigid body has a stable, muted visual color, defined in src/simulation/simulation/body_colors.py
and applied by scripts/integrate_sim.py during export. Parts attached to the same rigid
body share its color; joints keep separate colors. The checked-in robot.xml and
arm_preview.xml use the same palette. Wheel checker patterns remain visible.
Collision geometry, inertia, coordinates, joint limits and control behavior are
unchanged. In the read-only motor viewer, LIVE/STALE/WAITING/ABSENT colors still
override the four driven wheels; other bodies retain their identity colors.

| Body / movement | Color |
| --- | --- |
| Chassis (`body`) | `#87939d` |
| Left suspension (`b2_suspension_left_2`) | `#b9a38c` |
| Right suspension (`b2_suspension_right_3`) | `#8daaa5` |
| Front suspension (`suspension_3`) | `#a499b4` |
| Left front swivel (`freedrive_1`) | `#b89fa9` |
| Right front swivel (`b3_wheel_linkage_6`) | `#a6b397` |
| Wheel 1 (`linkage_wheel_1`) | `#c5b39b` |
| Wheel 2 (`linkage_wheel_2`) | `#bf9c94` |
| Wheel 3 (`linkage_wheel_3`) | `#94b3b7` |
| Wheel 4 (`linkage_wheel_4`) | `#92a5bb` |
| Wheel 5 (`linkage_wheel_5`) | `#baa8bf` |
| Wheel 6 (`linkage_wheel_6`) | `#b2bca0` |
| Arm D1 shoulder (`b2_arm_d1`) | `#8b9fb4` |
| Arm D2 upper link (`b3_arm_d2`) | `#bca18b` |
| Arm D3 crank (`l3_arm_linkage_d3`) | `#98ad96` |
| D3 passive rod (`b4_1_arm_rod_d3`) | `#c3b88f` |
| D3 passive bearing / forearm (`b4_2_arm_d3`) | `#aa9db7` |
| Arm D4 wrist (`l4_arm_linkage_d4`) | `#bd9da3` |
| Arm D5 wrist (`l5_arm_linkage_d5`) | `#8fada6` |
| Arm D6 tool (`l6_arm_linkage_d6`) | `#9db5c2` |

## Source and installed packages

Simulation implementations live in src/simulation/simulation; scripts contains
only launchers. Shared assets live in src/robot_description/mujoco and are installed
by the robot_description package. The motor overview resolves the same assets.
For installed-package terrain edits, set FORGE_MODEL_DIR to a writable copy of
the model directory. CAD export/integration uses the source checkout and writes
backups under build/simulation/backups. Credentials belong in src/simulation/.env.
The assembly URL belongs in ignored src/robot_description/mujoco/config.json.
