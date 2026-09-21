# FORGE simulation

The `forge_sim` branch contains the Onshape-to-MJCF export pipeline and MuJoCo preview.
Hardware drivers, motor control, and ROS 2 remain on [main](https://github.com/NSSRS/FORGE/tree/main).

## Setup and preview

In Ubuntu / WSL:

```bash
bash setup_sim.sh
source .venv/bin/activate
python preview_sim.py
```

For NVIDIA GPU rendering under WSLg:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA python preview_sim.py
```

The preview starts paused; Space toggles physics. Press R to reset the initial pose, velocities, forces, and simulation time, then pause. For editable Joint sliders,
use the full viewer and pause it with Space:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA python -m mujoco.viewer --mjcf=model/scene.xml
```

## Model and export

`scene.xml` includes `robot.xml` and adds a horizontal floor and lighting.
The committed CAD preview contains 13 merged body meshes, 11 hinge joints, and
one floating joint, with no actuators. The front suspension position/connectivity
still needs rebuilding in Onshape. Collision geometry is approximate.

Follow the Onshape-to-MJCF guide below to configure local
credentials and export. Keys, local assembly configuration, caches, and model
backups are ignored by Git.

## Files

- `model/`: XML, local export configuration, and mesh files in `assets/`.
- Root entry points: `preview_sim.py`, `export_onshape.py`, and `terrain.py`.
- `forge_sim/`: magnetic forces, wheel control, terrain menu, spawn fitting, and CAD processing.
- `tests/`: regression tests.
- `setup_sim.sh` and `requirements-sim.txt`: environment setup.

## Verification

```bash
python -m unittest discover -s tests -p 'test_*.py' -v
python preview_sim.py --check
```

## Onshape to MJCF and MuJoCo preview

This branch sets up CAD export and a passive preview only. It does not implement
magnetic adhesion, motor control, ROS integration, or a training environment.
The committed `robot.xml` is the merged FORGE CAD preview. The front suspension
position/connectivity still needs rebuilding in Onshape. A new export needs your
assembly URL and locally supplied Onshape keys.

## 1. WSL workspace

This machine uses Ubuntu-24.04 on WSL 2, Python 3.12, and WSLg. The working Linux
checkout is `/home/ns_wsl/forge_ws/FORGE` on branch `forge_sim`.

From PowerShell, open Ubuntu with `wsl -d Ubuntu-24.04`. Then:

```bash
cd ~/forge_ws/FORGE
bash setup_sim.sh
source .venv/bin/activate
python preview_sim.py
```

The virtual environment is local and ignored by Git. The setup pins MuJoCo
3.13.0 and onshape-to-robot 1.8.3. On another Ubuntu installation, first install
`python3-venv` with `sudo apt-get update && sudo apt-get install python3-venv`.
The Python MuJoCo wheel includes the engine; no separate engine installation is
needed ([MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html)).

Space toggles passive physics. The preview starts paused so a newly imported
robot can be inspected before gravity acts. Close the window to exit. For a
display-free load and 500-step smoke check:

```bash
python preview_sim.py --check
python -m unittest discover -s tests -p 'test_*.py' -v
```

The offline export tests check that failed downloads and invalid MJCF preserve
the existing model, and that a successful export retains the project scene.

## 2. Prepare the Onshape assembly

Use a top-level assembly with the chassis first in the instance list. Give the
moving mates unique `dof_` names, such as `dof_left_wheel`; fixed groups can use
`fix_`. Use `frame_` names for sites. Set joint limits and check the mate's Z axis
for joint direction. Assign realistic materials/densities and check mass
properties. Use Onshape's **Fixed** property deliberately for a fixed base;
a mobile chassis should be floating. Check exporter warnings for disconnected
parts. Closed mechanisms need the documented `closing_` convention.

See the [exporter's CAD conventions](https://onshape-to-robot.readthedocs.io/en/latest/design.html)
and [loop guidance](https://onshape-to-robot.readthedocs.io/en/latest/kinematic_loops.html).

## 3. Create credentials locally

In Onshape, open **My account > Developer > Create new API key** (or the
[developer keys page](https://dev-portal.onshape.com/keys)). Give the key the read
permissions required to access your document. Save both the access key and secret
when shown. Do not put them in config.json, chat, screenshots, or Git.

```bash
cd ~/forge_ws/FORGE
cp -n .env.example .env
chmod 600 .env
nano .env
```

Fill in `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`. Keep
`ONSHAPE_API=https://cad.onshape.com`. The export helper loads this ignored local
file; existing shell variables take precedence. Authentication follows the
[upstream setup](https://onshape-to-robot.readthedocs.io/en/latest/getting_started.html).

## 4. Select and export your assembly

Open the actual **assembly tab**, then copy its entire URL, including `/e/...`.
A version URL (`/v/...`) makes the CAD source reproducible; `/w/...` exports the
selected live workspace.

```bash
cp -n model/config.example.json model/config.json
nano model/config.json
```

Replace only the example URL initially. The template selects MuJoCo output,
`robot.xml`, and `assets/`. It disables generated actuators for this passive
preview. Geometry, inertia, joints, and limits still come from CAD. Additional
options are documented in the [configuration reference](https://onshape-to-robot.readthedocs.io/en/latest/config.html).

```bash
source .venv/bin/activate
python export_onshape.py
python preview_sim.py
```

The helper exports in a temporary staging directory, then merges all part meshes
within each exported rigid body into one body-local OBJ and one geom. It never
merges across joints. Top-level subassemblies connected rigidly by the exporter
may already form one body together. Relative part transforms are baked into the
merged vertices; body/joint transforms and explicit mass/inertia remain unchanged.
Colors are unified per body. Collision uses one convex hull per merged body, so
gaps and concave regions have approximate contacts: this is a preview model, not
a validated contact-dynamics model.

Only dense source meshes with over 50,000 triangles are reduced, retaining at
least half their faces. Ordinary parts are left intact. The 10,000-face per-body
target is soft and can be exceeded to protect shape details. Use
`python export_onshape.py --faces-per-body 0` for lossless geometry merging.
To merge an existing unmerged local model without calling Onshape, use
`python export_onshape.py --merge-existing`. Repeated reduction of an
already simplified model is cumulative; start from a fresh export or original
backup when comparing settings.

Before promotion, the helper compares joints, hierarchy, inertia, and link poses
at three joint configurations. It moves the previous `robot.xml` and entire
`assets/` into the ignored `.sim-backups/` folder before installing the new model.
Thus only merged OBJ files remain in the active assets folder. The existing
`scene.xml` is retained. Failed generation/validation leaves the active model
intact, and promotion failures roll back. Custom processors, additional XML,
geom-referencing sensors/contacts, and primitive geoms are outside this workflow.

The project-local compatibility wrapper exports continuous wheel joints as
unlimited hinges. STL files exceeding MuJoCo's 200,000-triangle STL limit (and
ASCII STL files) are converted to OBJ before body merging. Individual source
meshes are absent from the final active output. Installed third-party packages
are not modified.

```text
model/
  config.example.json  # tracked template
  config.json          # ignored, actual assembly selection
  robot.xml            # merged FORGE CAD preview
  scene.xml            # includes robot.xml, floor, lighting
  assets/              # exported meshes
```

Inspect scale, base placement, wheel axes, joint limits, masses, and contacts.
The floor is at Z=0; prepare CAD placement accordingly. A model compiling or
passing the smoke check does not establish realistic robot dynamics. Floating
robots may fall when unpaused; magnetic adhesion is not implemented.

## 5. Review and publish a CAD update

The initial branch can be shared before exporting CAD. After a successful export,
review the generated files before committing:

```bash
git diff --stat
git status --short
git add model/robot.xml model/assets
git commit -m "Import FORGE assembly from Onshape"
git push origin forge_sim
```

Record the non-secret Onshape version URL in your commit description or project
documentation if it can be shared. Never force-add `.env` or exporter caches.

## Troubleshooting

- **No viewer window:** confirm `echo "$DISPLAY"` is nonempty in Ubuntu. Use
  `--check` to separate model errors from WSLg graphics errors. If OpenGL fails,
  try `LIBGL_ALWAYS_SOFTWARE=1 python preview_sim.py`.
- **WSLg Mesa crashes on large meshes:** the preview sets `LP_NUM_THREADS=1`
  on WSL unless already set. This avoids the observed Mesa multi-thread crash
  without forcing a different graphics backend or changing system settings.
- **401/403:** check local key values, read permissions, and document access.
- **Wrong assembly:** ensure the URL is from an assembly tab, not a Part Studio.
- **Missing joints:** check `dof_` naming and the exporter warnings.
- **Mesh/inertia errors:** fix CAD mass properties or geometry, then re-export.
- **Package-manager lock:** allow Ubuntu automatic updates to finish, then retry.
- **GitHub timeout on this machine:** Windows uses a localhost proxy on port 7890.
  WSL's NAT network cannot automatically use it. PyPI and Onshape were reachable
  directly. The initial branch is published using Windows Git through that proxy;
  future native WSL pushes need working WSL network routing and Git authentication.
  No credentials or proxy secrets are stored in the repository.

For this machine's current network, commit in WSL, then push the same Linux
checkout from **PowerShell** using the existing Windows Git credential manager:

```powershell
git -c safe.directory=//wsl.localhost/Ubuntu-24.04/home/ns_wsl/forge_ws/FORGE `
    -c core.filemode=false -c http.proxy=http://127.0.0.1:7890 `
    -C '\\wsl.localhost\Ubuntu-24.04\home\ns_wsl\forge_ws\FORGE' push origin forge_sim
```

These options apply to that command only; they do not change global Git settings.

The upstream exporter can also generate its own test scene; see its
[MuJoCo output documentation](https://onshape-to-robot.readthedocs.io/en/latest/exporter_mujoco.html).


## GPU rendering on WSL (NVIDIA)

The current preview was tested with WSLg D3D12 on an RTX 4090. Select it for
this process without changing global settings:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA python preview_sim.py
```

For manual joint inspection, use the full viewer, pause with Space, then move
its Joint sliders (the passive preview disables these sliders):

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA python -m mujoco.viewer --mjcf=model/scene.xml
```

GPU selection accelerates rendering; standard MuJoCo physics still runs on CPU.
The current model has no actuators. The front suspension position/connectivity
still needs rebuilding in Onshape before the next export.


## Steel wall and magnetic wheels (local prototype)

`model/scene.xml` defines `steel_wall`, a fixed 2 m wide, 1.5 m tall plate,
20 mm thick (box half-thickness 0.01 m), centered at `(0, -0.8, 0.75)`.
All six `linkage_wheel_1` through `linkage_wheel_6` bodies are tagged as magnets.
Names are resolved to IDs on load; IDs are not hard-coded across CAD exports.
Other robot parts keep ordinary collision behavior, including existing self contacts.

Run `python preview_sim.py` to enable the force model. The standalone
`python -m mujoco.viewer` loads the wall but does **not** run `forge_sim/magnetic.py`.

The scene's `magnetic_parameters` are demo values: 100 N maximum per wheel,
0.02 m cutoff, and 0.002 m decay length. For nonnegative surface gap d below
cutoff R, force magnitude is `Fmax * (1-d/R)^2 / (1+d/decay)^2`; it is zero
outside the cutoff and capped at Fmax during penetration. These parameters are
not measured magnet performance. Plate thickness does not automatically determine
magnetic strength. The force uses closest points on the existing convex wheel
collision geometry, not a detailed magnetic field calculation.

The robot now starts vertically against the positive-Y face of the steel wall,
with a free base and wheel gaps of approximately 0.2-2.1 mm. Exports into a scene
containing steel_wall automatically use this wall spawn. Without that wall, the
export helper uses the floor placement. The preview starts paused.
This is not a static-holding controller: the current wheels can turn and the robot
moves down the wall after unpausing. Braking and contact calibration remain to be
implemented. Existing merged-mesh self-collision approximations remain.


## EC-A4310-P2-36 wheel velocity control

Only wheel_1 through wheel_4 are driven; wheel_5/6 and suspension/steering joints
remain passive. Motor data comes from the user-provided parameter table:
12 Nm rated output torque, 75 RPM rated output speed; 36 Nm and 89 RPM are peak
values. The 36:1 reduction is already accounted for in output-shaft values, so
actuator gear is 1. The 156 W specification is not used as a constant mechanical
output-power limit; electrical, thermal, and peak-duration behavior are not modeled.

Each native MuJoCo velocity actuator computes
`torque = clip(2 * (target_rad_s - joint_rad_s), -12, 12)`.
Gain 2 Nm/(rad/s) is a provisional simulation choice, not a datasheet value.
Target speed is limited to +/-75 RPM (+/-7.85398 rad/s). This limits commands,
not actual speed under external forcing. Torque limits apply in both directions,
including braking. Zero target brakes rotation but does not rigidly lock a wheel.

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA .venv/bin/python preview_sim.py --wheel-rpm 10 10 10 10
```

The four numbers are signed **joint** RPM for wheels 1, 2, 3, 4; equal signs do
not imply vehicle-forward motion because CAD joint axes may differ. Use the
right-hand Control sliders (rad/s) for individual targets. Space starts/pauses;
R resets to the wall pose, pauses, and sets all target speeds to zero. Omitting
--wheel-rpm also starts with zero targets. Native motor limits work in the stock
viewer, but magnetic forces still require preview_sim.py.

The export helper reinstalls these actuators after CAD merging. Scene integration
uses implicitfast for the velocity feedback. No controller communicates with hardware.


## WASD driving

Focus the preview window. W/S command forward/reverse; A/D command differential
turning in place; X commands zero speed. Space pauses/runs (pausing clears speed
commands); R resets pose, pauses, and clears all commands. WASD commands latch:
releasing a key does not stop motion. Press X to stop. Start physics with Space.
Keyboard control replaces the Control sliders in preview_sim.py.

Tune the bottom parameter block in forge_sim/velocity_control.py: DRIVE_RPM (10), TURN_RPM
(6), WHEEL_SIDES, WHEEL_DIRECTIONS, and WHEEL_RPM_SCALES. All four-entry lists are
ordered wheel_1 to wheel_4. Current geometry gives robot-left wheels 1/2, right
wheels 3/4 and direction signs (-1,-1,+1,+1). W points toward the CAD front,
upwards at the initial wall pose. Wheel parameters can be edited and the preview
restarted. Torque and RPM limits still apply; front wheels 5/6 remain passive.


## Live telemetry and appearance

The preview shows world XYZ (m), body roll/pitch/yaw (degrees, ZYX Euler
convention), simulation time, all six actual joint RPM, and target RPM plus
actuator output-shaft torque for driven wheels 1-4. Torque is motor actuation,
not total contact/joint load. Passive wheels show no motor torque. Signed RPM
follows each CAD joint axis; opposite wheel signs can mean the same travel direction.
The steel plate is now 4 m wide by 4 m tall and 20 mm thick. The current robot
spawn height is retained. Non-wheel bodies have distinct muted colors; wheels
retain checkerboard textures. Export reapplies the colors automatically.

Edit DRIVE_RPM and TURN_RPM in forge_sim/velocity_control.py for keyboard speed targets,
and WHEEL_RPM_SCALES/WHEEL_DIRECTIONS/WHEEL_SIDES for per-wheel tuning; restart
after editing. RPM means output-shaft revolutions per minute, with rad/s = RPM*pi/30.
The 75 RPM command cap and 12 Nm torque cap still apply. Actual RPM is measured
from simulation joint velocity and can differ from the target under load.


## Quick terrain presets

Run from the repository root in the simulation environment. Keep the same WSL GPU
prefix as for the usual preview. No manual XML swapping is needed:

```bash
python preview_sim.py --terrain flat
python preview_sim.py --terrain curved
python preview_sim.py --terrain bumpy
python preview_sim.py --terrain bumpy --tilt 90
```

Defaults are arc length **10 m**, total turn **60 degrees**, width **4 m**, and
nominal steel thickness **20 mm**. Radius is length/radians(angle) = **9.549 m**.
These are test dimensions, not verified ship dimensions. Curvature varies along
the driving direction, +/-30 degrees from the center tangent; width is straight.
Default curvature represents the convex exterior of a hull. --inward reverses it.
--tilt 0 is vertical; --tilt 90 puts the robot underneath a horizontal center tangent.

`bumpy` adds ten local convex caps ahead of the spawn along both wheel tracks,
with default diameter **100 mm** and height **5 mm**. Diameter is not height.
The caps are geometric proxies for local steel deformation, not elastic sheet
mechanics or a plate-thickness-dependent magnetic model.

```bash
python preview_sim.py --terrain bumpy --arc-length 10 --arc-angle 30 \
    --plate-width 4 --bump-diameter 0.10 --bump-height 0.01 --tilt 45
```

Starting a generated preset rebuilds model/scene_curved.xml or scene_bumpy.xml
from the current robot and base scene, so current colors/motors carry over.
The original robot.xml and scene.xml are left untouched. R resets to that preset's
spawn; launch --terrain flat to return to the original flat wall. Generated scene
XMLs also load standalone, but custom magnetic forces require preview_sim.py.

For generation only: `python terrain.py bumpy --length 10 --angle 30`.
A 10 m arc uses 100 shared-edge convex slab segments, with maximum chord sag
about 0.131 mm at default curvature. A whole curved mesh would otherwise become
one convex collision hull. Magnetism selects the nearest steel segment/cap per
wheel and keeps a per-wheel force cap, so seams do not double magnetic force.
Facets, cap boundaries, collision approximations, and magnetic calibration still
limit physical fidelity. Checks establish loading and numerical stability, not
successful traversal or validated ship adhesion. Tight curvature can leave some
wheels outside the magnetic cutoff at the undeformed initial joint configuration.


## In-window terrain menu

Press **M** in the preview to open terrain settings without closing the window.
Click each row's +/- buttons, or use Up/Down to select and Left/Right to change.
Click **APPLY & RESET ROBOT** (or Enter) to rebuild and load the selected terrain
in the same window. M/Escape closes the menu; Space resumes after applying.
Use `python preview_sim.py --menu` to start directly at the menu.

Opening the menu pauses physics and clears wheel commands. Applying resets robot
pose, simulation time, controls, and camera, and remains paused. Invalid settings
leave the previous loaded scene in place and show an error. The flat preset uses
the original scene.xml; curvature/bump/tilt fields do not apply to that preset.
Settings are session-local; generated preset XMLs retain the applied terrain.


## Automatic terrain spawn fitting

Generating/applying curved or bumpy terrain now fits a starting pose before
loading it. The solver moves the free chassis and adjusts suspension_1/2/3 and
freedrive_1/2; wheel rotation angles and magnetic force parameters stay unchanged.
It targets 0.5 mm wheel gaps while penalizing robot/steel, ground, and
engine-eligible self penetration. Each wheel must end within a conservative 10 mm
spawn-fitting gap (the magnetic cutoff is 20 mm); penetration greater than 0.05 mm causes rejection. Failure keeps
the previous generated scene and the menu reports the reason.

Existing CAD joint limits are respected. Unlimited passive joints get a temporary
+/-30 degree search bound (not a validated hardware limit); base orientation search
is +/-20 degrees per axis. These search parameters are in forge_sim/spawn_fit.py. This is
geometric initialization, not a static-equilibrium or guaranteed-traversal solver.

The result is saved as the terrain_spawn keyframe. preview_sim.py loads it at
startup, terrain application, and R reset. A stock MuJoCo viewer must load that
keyframe explicitly; it does not automatically start at the fitted state.
The original robot.xml remains unchanged by fitting. scipy is pinned in the
simulation requirements for the bounded optimizer.


Magnetic broad-phase filtering caches world-fixed steel bounds and filters them
in NumPy batches. Only nearby panels/bumps receive exact mesh-distance queries;
force magnitude, application points, collision meshes, and the 20 mm cutoff are
unchanged. Moving steel bodies recompute their bounds each step.


Bump spacing is adjustable in the M menu (Bump spacing, 0.15..5 m, step
0.05 m), or with `--bump-spacing 0.4` in preview_sim.py / terrain.py.
Default spacing is 0.8 m along each track; the first bump stays 0.8 m ahead
to preserve spawn clearance. Alternating track offsets remain 0.12 m.
Click Apply to rebuild. Spacing affects only bumpy terrain; spacing smaller
than bump diameter produces overlapping bumps.


Bump lateral spacing controls the center-to-center width between the two rows
(symmetrical about the plate center). Adjust Bump lateral spacing in the M menu
or pass `--bump-track-spacing 0.5`. Default is 0.737 m; longitudinal spacing
is a separate setting. Row spacing plus bump diameter must fit the plate width.


Press C to toggle collision debugging (or launch with --collision-debug).
MuJoCo renders collision convex hulls instead of detailed meshes, with contact
points and force arrows. The top-right lists active contact body/terrain pairs
and penetration in mm, prioritizing bumps. Space pauses for inspection; C
returns to normal rendering without changing physics or collision geometry.


## Per-part collision restoration

Fresh Onshape exports now retain original per-part collision geoms (group 3)
inside each rigid body, while merging only visual meshes (group 2). Visual
meshes have contype/conaffinity zero. C switches between visual meshes and
collision hulls; no extra joints or bodies are created. Individual concave
parts still use a convex hull. Magnetic wheel distance currently uses the
merged visual wheel envelope, independent of per-part physical collision.
Raw XML and source assets are retained under .sim-backups/raw-* before merging.
This supersedes the older one-convex-hull-per-body description above.
