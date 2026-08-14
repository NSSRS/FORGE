# FORGE

FORGE collects two engineering-screening simulators for a magnetic wall-climbing welding robot.

| Project | Purpose |
|---|---|
| [`suspension_wheel_geometry_sim`](suspension_wheel_geometry_sim/) | Interactive six-wheel geometry, suspension, caster, contact, clearance, and terrain visualization |
| [`mag_thermal_thickness_sim`](mag_thermal_thickness_sim/) | Three-dimensional plate-conduction, magnet heating, and empirical magnetic-adhesion screening |

These tools are intended for early mechanical and thermal design exploration. They are not validated substitutes for physical testing, welding-procedure qualification, magnetic finite-element analysis, or safety engineering.

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
│   ├── main.py
│   ├── simulation_view.py
│   ├── contact.py
│   └── README.md
├── mag_thermal_thickness_sim/
│   ├── heat_visualizer.py
│   ├── test.py
│   ├── requirements.txt
│   └── README.md
└── README.md
```

Each project has its own README with its model assumptions, controls, equations, outputs, and limitations.

## Verified state

- Suspension simulator: 6 regression tests passing
- Thermal simulator: 15 unit, formula, and numerical regression tests passing
- Python sources compile successfully under Python 3.12

## Safety and interpretation

The geometry simulator does not calculate magnetic adhesion, friction, motor torque, or structural loads. The thermal simulator uses constant material properties, a simplified heat source, a lumped magnet, and empirical adhesion corrections. Results should be calibrated against measured temperatures and pull-force data before they inform physical design decisions.
