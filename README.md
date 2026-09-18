# FORGE simulation

The `forge_sim` branch contains the Onshape-to-MJCF export pipeline and MuJoCo preview.
Hardware drivers, motor control, and ROS 2 remain on [main](https://github.com/NSSRS/FORGE/tree/main).

## Setup and preview

In Ubuntu / WSL:

```bash
bash scripts/setup_sim.sh
source .venv/bin/activate
python scripts/preview_sim.py
```

For NVIDIA GPU rendering under WSLg:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA python scripts/preview_sim.py
```

The preview starts paused; Space toggles physics. For editable Joint sliders,
use the full viewer and pause it with Space:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA python -m mujoco.viewer --mjcf=src/forge_sim/model/scene.xml
```

## Model and export

`scene.xml` includes `robot.xml` and adds a horizontal floor and lighting.
The committed CAD preview contains 13 merged body meshes, 11 hinge joints, and
one floating joint, with no actuators. The front suspension position/connectivity
still needs rebuilding in Onshape. Collision geometry is approximate.

Follow the [Onshape-to-MJCF guide](docs/ONSHAPE_MUJOCO.md) to configure local
credentials and export. Keys, local assembly configuration, caches, and model
backups are ignored by Git.

## Layout

- `src/forge_sim/model/`: robot, scene, mesh assets, and export configuration template.
- `scripts/`: simulation setup, export, mesh merging, floor placement, and preview.
- `tests/sim/`: offline export and model-processing regression tests.
- `docs/ONSHAPE_MUJOCO.md`: setup, CAD preparation, export, and GPU rendering.

## Verification

```bash
python -m unittest discover -s tests/sim -v
python scripts/preview_sim.py --check
```
