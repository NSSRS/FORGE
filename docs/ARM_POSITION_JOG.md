# Joint-angle API and manual arm jog

2026-11-05. This is a reusable position port plus an unloaded commissioning UI.
It is separate from ENCOS WASD driving. No hardware is automatically enabled
at startup. Production ROS, calibrated arm coordinates, multi-axis trajectories,
collision checking and gravity holding remain upper-layer integration work.

The [2026-11-09 assembled-arm handoff](ASSEMBLED_ARM_BRINGUP.md) explains
session zero versus absolute encoder/model position, the Pocket changes,
constraint study and supported calibration/holding work still required.

## Survey and design choice

[Universal Robots manual jogging](https://www.universal-robots.com/articles/ur/programming/how-to-jog-the-robot/)
uses individual-joint arrows as well as a separate tool translation/rotation
interface. [UR Move tab](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-usr-man/complianceUR3e/SW_sections/first_program/move_tab.htm)
distinguishes joint positioning from tool-frame operations.
[MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
provides joint-jog, twist and pose interfaces, with collision/singularity handling
and optional smoothing at the robotics-control level. These are references
for UI/layer separation, not a claim that this keyboard tool has those protections.

The Python/PyQt5 desktop window uses real key press/release events. Select a
joint and arm once; holding an arrow moves continuously at 45°/s. Release
stops and disables torque while retaining authorization for that selection.
Switching motors, explicit stop, focus loss or faults disarm. Auto-repeat
press/release events are ignored; they do not queue moves.

## Run

From `/home/forge2/forge2_ws`, try the UI offline first:

```bash
python3 scripts/jog_dm_arm.py --simulate
```

With the same six secured, unloaded, free-shaft motors, A1 on CAN1 and A2..A6
on CAN2, and clearance for the selected window:

```bash
python3 scripts/jog_dm_arm.py --execute --unloaded \
  --plan src/robot_description/config/arm_jog.json \
  --port /dev/ttyACM0 \
  --report build/arm_tests/arm-keyboard-result.json
```

This host feature uses the existing 180-degree routed position-velocity
firmware. No additional firmware change or flash is needed for jogging if
that firmware is already installed. The separately developed cached telemetry
upgrade is not required by this adapter. Ordinary repeat sessions reuse the
existing disabled configuration; faults, ID-session locks and changed routes
still require explicit recovery. Stop other motor-owning processes first.

| Key | Action |
| --- | --- |
| 1..6 | Select motor; a different selection stops and disarms |
| E | Arm selected motor once; no CAN enable yet |
| Hold Right / Up | Move continuously in positive direction |
| Hold Left / Down | Move continuously in negative direction |
| Release arrow | Disable motion; selected motor stays armed |
| Space / X / Escape | Stop and disarm |
| Q / window close | Stop, restore original modes and exit |

Example: `1`, `E`, hold Right, release, hold Left. No second E is needed.
Press `4`, then E to authorize A4. Clicking motor/arm buttons also works.
Speed is 45 degrees/s. Motion stops at the session window (default ±90 degrees
from startup, narrowed with `--window-deg`); holding does not mean unlimited
rotation. Startup does not change hardware zero. Positive is raw drive direction.
Two direction keys stop motion; release all before choosing a direction again.
A window focus loss stops and disarms. Faults lock further motion until recovery.

PyQt5 is required (`python3-pyqt5` on Ubuntu; installed on this NUC). The UI is
a desktop window, and does not require an interactive terminal. Idle positions
are cached and show their age; disabled feedback refreshes before each move.
No IDs, gains, current limits or saved parameters are written.

## API port

`src/motors/arm_position.py` contains:

- `PositionPort`: typed interface shared by hardware and offline adapters.
- `ArmPositionPort`: existing verified CAN routing, preflight, mode preparation,
  acknowledged disable, bounded position targets and cleanup.
- `SimulatedPositionPort`: offline UI adapter; does not represent motor dynamics.
- `JointPosition`: raw position, session origin/bounds, feedback age and enable state.

The API uses raw output-shaft **radians / radians per second**, while the UI
uses degrees. `select(1..6)` determines the physical DOF; the verified config
must use matching command IDs and feedback IDs. `move_to(angle_rad,speed_rad_s)`
sets an absolute raw-shaft target. `move_by(delta_rad,speed_rad_s)` obtains fresh
disabled feedback and computes a relative target. `positions()` returns cached
measurements with age. `release()` disables but preserves authorization on success; `stop()` disables and disarms; `close()` restores modes
and releases the USB port. No background thread refreshes commands.

Example for explicitly confirmed unloaded hardware (same routing/profile):

```python
import json, math, time
from motors.arm_position import ArmPositionPort

with open('src/robot_description/config/arm_jog.json') as stream:
    plan = json.load(stream)
plan['fixture_confirmed'] = True  # only after confirming the unloaded fixture
with ArmPositionPort(plan, port='/dev/ttyACM0') as arm:
    arm.select(2)
    initial = arm.positions()[1]
    arm.arm()  # authorize this selected motor until stop/selection/fault
    arm.move_to(initial.origin_rad + math.radians(30), math.radians(45))
    while arm.poll():
        time.sleep(0.02)
```

Call `poll()` regularly while moving (e.g. every 20 ms). It refreshes only the
explicit bounded target, checks feedback and stops when settled. Stop calling
it and the existing 150 ms MCU command lease expires; telemetry/selection never
keeps an expired movement alive. Each operation must fit a 12-second host
bound; the firmware's 15-second bench cutoff remains. Targets stay within the
session window, with 0.05-rad firmware padding. Software bounds are not measured
mechanical joint stops.

The hardware adapter intentionally reuses `BenchSession`'s tested lifecycle
rather than creating a second competing implementation of enable/stop/restore.
It does not run the benchmark sweep. This single-joint port is suitable for
proving routing and future adapter integration; it is **not** a simultaneous
six-joint trajectory controller or a gravity-hold controller. Nonselected
motors remain disabled. Do not use it to support an assembled loaded arm.

## Validation and next interfaces

Tests exercise all six selections against the real C firmware in a fake plant:
absolute/relative targets, other-motor isolation, positive/negative return,
selection mid-move, invalid targets/speeds, stale poll watchdog and mode cleanup.
Window tests cover held motion, release without re-arming, repeat events,
conflicting arrows, selection, focus loss, stalled UI and fault lockout.
Qt tests use actual key events with its offscreen platform. No motors were
moved by these tests. On 2026-11-09 the operator reported keyboard Up=CCW
on all six unloaded motors and later working Pocket control after receiver
restart. These observations are separate from full stop/failsafe acceptance
and do not establish assembled-arm calibration or holding.

Stable handoff: [cross-layer registry](ARM_LAYER_TODOS.md), especially
ROS-02 (production position-velocity adapter), CAL-01 (coordinates/limits),
BASE-06 (atomic multi-joint commands), PLAN-01 (production jog/trajectory policy)
and SUPPORT-01 (loaded stopping/holding). Add ROS action/service/topic adapters
above this explicit API; do not reimplement drive packets in teleop.

## Pocket display reuse

The keyboard window and Pocket dashboard share `teleop.arm_widgets`' joint
readout. Keyboard selection, E authorization, arrow events and focus-loss stops
are unchanged. The Pocket dashboard consumes the existing Pocket control
owner's snapshots; see [Pocket dashboard](POCKET_ARM_CONTROL.md#pocket-desktop-dashboard).
`cached_positions()` returns already collected control feedback with advancing
age and performs no USB/CAN requests. It is distinct from `positions()`.
