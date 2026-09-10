# FORGE

FORGE collects engineering tools for the magnetic wall-climbing welding robot.

| Project | Purpose |
|---|---|
| [`suspension_wheel_geometry_sim`](suspension_wheel_geometry_sim/) | Interactive six-wheel geometry, suspension, caster, contact, clearance, and terrain visualization |
| [`mag_thermal_thickness_sim`](mag_thermal_thickness_sim/) | Three-dimensional plate-conduction, magnet heating, and empirical magnetic-adhesion screening |
| [`src/encos_query`](src/encos_query/) | Ubuntu EtherCAT-to-CAN bridge diagnostic and bounded ENCOS motor motion test |

The simulators are intended for early mechanical and thermal design exploration. The ENCOS utilities are bench bring-up tools and require a controlled, powered hardware fixture.

## ENCOS motor workspace

On Ubuntu 24.04, build the EtherCAT-CAN diagnostic and bounded motion utility with:

```bash
./scripts/build.sh
```

The first build fetches the pinned public SOEM v1.4.0 dependency. Supplier files, extracted supplier demos, local equipment records, and motion logs are intentionally excluded from Git.

See [the ENCOS workspace guide](docs/ENCOS_WORKSPACE.md), [the protocol guide](docs/ENCOS_BRINGUP.md), and [the bench procedure](docs/ENCOS_TEST_BENCH.md).

## Quick start

### Suspension and wheel geometry

```powershell
cd suspension_wheel_geometry_sim
python main.py
```

The interactive window supports live geometry sliders, multiple terrain modes, driven rear-wheel animation, fixed transverse front suspension geometry, and passive front caster yaw.

Run its tests with:

```powershell
python -m unittest -v
```

### Thermal and magnetic adhesion

```powershell
cd mag_thermal_thickness_sim
python -m pip install -r requirements.txt
python heat_visualizer.py
```

Run its numerical regression suite with:

```powershell
python test.py
```

The thermal tests run real finite-difference cases and may take about one minute.

## Repository layout

```text
FORGE/
├── suspension_wheel_geometry_sim/
├── mag_thermal_thickness_sim/
├── src/encos_query/
├── docs/
├── scripts/
└── README.md
```

Each project has its own documentation with its model assumptions, controls, outputs, and limitations.

## Verified state

- Suspension simulator: 6 regression tests passing
- Thermal simulator: 15 unit, formula, and numerical regression tests passing
- ENCOS query and bounded-motion utilities: clean build and protocol test passing on Ubuntu 24.04

## Safety and interpretation

The geometry simulator does not calculate magnetic adhesion, friction, motor torque, or structural loads. The thermal simulator uses constant material properties, a simplified heat source, a lumped magnet, and empirical adhesion corrections. Results should be calibrated against measured temperatures and pull-force data before they inform physical design decisions.
