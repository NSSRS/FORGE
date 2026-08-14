# Robot Geometry Input Form

Confirmed dimensions are recorded below. Code should use metres internally; measurements here are shown in millimetres.

## 1. Units and surface

- Length unit: millimetres (mm)
- Angle unit: degrees
- Surface: flat steel plate
- Cylinder geometry: not applicable to the initial model

## 2. Body

- Plan-view body shape: isosceles triangle
- Rear side: 600 mm
- Two equal sides: 500 mm each
- Rear pivot line to body front vertex: calculated from the rear side and equal-side length; 400 mm for the confirmed 600-500-500 mm triangle
- Front suspension pivot extension ahead of the front vertex: 50 mm
- Rigid front link from body vertex to suspension pivot: 50 mm long and 30 mm thick
- Rear pivot line to front suspension pivot: 450 mm
- Body-length sweep: not yet specified
- Side-view body thickness: 100 mm
- Desired nominal body-to-surface clearance: 50 mm
- Minimum permitted body-to-surface clearance: 5 mm
- Longitudinal center of mass: 1/3 of the 400 mm body length from the rear pivot line (equivalently 2/3 from the front vertex)
- Vertical center of mass: middle of the body thickness

## 3. Module locations and symmetry

- Rear-left and rear-right modules are identical.
- Rear-left pivot in longitudinal body coordinates `[x, z]`: `[0, 0]` mm
- Rear-right pivot in longitudinal body coordinates `[x, z]`: `[0, 0]` mm; it differs only laterally in plan view
- Front pivot in longitudinal body coordinates `[x, z]`: `[450, 0]` mm
- Visualization: identify all three modules separately, but retain the coincident longitudinal geometry of the two rear modules in calculations

## 4. Dual-wheel suspension geometry

**Selected type: D — a rigid two-arm assembly rotating freely about one shared suspension pivot.**

The two arms in each module are firmly attached, retain a fixed included angle, and rotate together. They do not rotate independently. Angles are measured downward from a horizontal line parallel to the robot frame. The stated angles define the assembly's reference orientation; the whole assembly is free-moving about its pivot.

### Rear modules (left and right, identical)

- Suspension-plane orientation: longitudinal (fore-aft)
- Rearward-facing arm length: 300 mm
- Rearward-facing arm reference angle: 30 degrees below the rearward horizontal
- Forward-facing arm length: 150 mm
- Forward-facing arm reference angle: 60 degrees below the forward horizontal
- Relative angle between arms: fixed
- Motion: both arms rotate together as one freely moving rigid assembly
- Prescribed minimum/maximum angle: none; feasible rotation is determined by collision/interference
- Wheel radius: 63 mm
- Wheel tread width used in top-view rendering: 32 mm
- Wheel setup: one equal-radius wheel centered at the end of each arm
- Suspension-arm thickness: 30 mm
- Collision-envelope radius: 15 mm

### Front module

- Suspension-plane orientation: transverse (left-right), perpendicular to the rear suspension planes
- Suspension rotation relative to body: locked; the transverse arms are firmly attached to the body
- Rearward-facing arm length: 200 mm
- Rearward-facing arm reference angle: 45 degrees below the rearward horizontal
- Forward-facing arm length: 200 mm
- Forward-facing arm reference angle: 45 degrees below the forward horizontal
- Relative angle between arms: fixed and symmetric
- Motion: both arms rotate together as one freely moving rigid assembly
- Prescribed minimum/maximum angle: none; feasible rotation is determined by collision/interference
- Wheel radius: 63 mm
- Wheel setup: one equal-radius wheel centered at the end of each arm
- Side-view appearance: the two front wheels overlap, so only one wheel profile is visible and should be labelled `x2`
- Wheel rolling direction: the same forward direction as all four rear wheels
- Each wheel swivels freely in yaw about a caster axis at the arm endpoint.
- Caster wheel-center offset from swivel axis: 20 mm rearward (`-x`) and 10 mm toward the steel plate (`-z` in the model coordinate system)
- Suspension-arm thickness: 30 mm
- Collision-envelope radius: 15 mm

### Parameter meanings

- `Arm length`: distance from the shared suspension pivot to the wheel center.
- `Wheel offset`: not applicable; each wheel center is at its arm endpoint.
- No spring, damper, stiffness, or preferred restoring angle is modeled.

## 5. Wheel and body interference

- Minimum allowed gap between different wheels: 1 mm
- Minimum allowed wheel-to-body gap: 1 mm
- Unintended touching counts as interference: yes
- Intended wheel-to-surface contact is not interference.
- Body outline: a 600-500-500 mm triangle in plan view; the longitudinal side-view envelope uses its 400 mm length and 100 mm thickness

## 6. Contact and feasibility

- Contact residual tolerance: not yet specified
- Required contacts: all six wheels
- Minimum remaining travel from angular limits: not applicable; there are no prescribed angular limits
- Minimum geometric stability margin: not yet specified
- Optimize body height and pitch: not yet specified
- Allow longitudinal translation while finding a pose: not yet specified

## 7. Parameter sweep

Sweep ranges are not yet specified. The initial implementation should support later configuration of body length, suspension dimensions/reference orientation, wheel radius, and surface geometry. Surface-radius sweep is not applicable to the initial flat-plate case.

## 8. Output preferences

- Python version: not constrained
- Permitted dependencies: not yet constrained
- Command-line preferences: not yet specified
- Additional CSV columns: none specified beyond the README
