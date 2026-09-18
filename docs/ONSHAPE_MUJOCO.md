# Onshape to MJCF and MuJoCo preview

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
bash scripts/setup_sim.sh
source .venv/bin/activate
python scripts/preview_sim.py
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
python scripts/preview_sim.py --check
python -m unittest discover -s tests/sim -v
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
cp -n src/forge_sim/model/config.example.json src/forge_sim/model/config.json
nano src/forge_sim/model/config.json
```

Replace only the example URL initially. The template selects MuJoCo output,
`robot.xml`, and `assets/`. It disables generated actuators for this passive
preview. Geometry, inertia, joints, and limits still come from CAD. Additional
options are documented in the [configuration reference](https://onshape-to-robot.readthedocs.io/en/latest/config.html).

```bash
source .venv/bin/activate
python scripts/export_onshape.py
python scripts/preview_sim.py
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
`python scripts/export_onshape.py --faces-per-body 0` for lossless geometry merging.
To merge an existing unmerged local model without calling Onshape, use
`python scripts/export_onshape.py --merge-existing`. Repeated reduction of an
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
src/forge_sim/model/
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
git add src/forge_sim/model/robot.xml src/forge_sim/model/assets
git commit -m "Import FORGE assembly from Onshape"
git push origin forge_sim
```

Record the non-secret Onshape version URL in your commit description or project
documentation if it can be shared. Never force-add `.env` or exporter caches.

## Troubleshooting

- **No viewer window:** confirm `echo "$DISPLAY"` is nonempty in Ubuntu. Use
  `--check` to separate model errors from WSLg graphics errors. If OpenGL fails,
  try `LIBGL_ALWAYS_SOFTWARE=1 python scripts/preview_sim.py`.
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
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA python scripts/preview_sim.py
```

For manual joint inspection, use the full viewer, pause with Space, then move
its Joint sliders (the passive preview disables these sliders):

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA python -m mujoco.viewer --mjcf=src/forge_sim/model/scene.xml
```

GPU selection accelerates rendering; standard MuJoCo physics still runs on CPU.
The current model has no actuators. The front suspension position/connectivity
still needs rebuilding in Onshape before the next export.
