# Assembled arm handoff

Updated 2026-11-09, branch `nathan`. The current Pocket and keyboard tools can
jog the six secured, unloaded DM motors. The MuJoCo study now visualizes
candidate travel around the saved initial pose. The assembled arm still needs
physical calibration, a controller that holds all load-bearing joints, and a
defined response to loss of torque. The operator reports that it falls when
unpowered; the current commissioning adapter disables torque when jogging stops.

## Motor controls delivered

| Control | Current behavior |
| --- | --- |
| Left stick, CH3/CH4 | Chassis forward/reverse and yaw; independent of SE |
| Right stick, CH1/CH2 | Left/up = CW; right/down = CCW on the selected motor |
| SB, CH6 | Top/middle/bottom selects A1/A2/A3 |
| SC, CH7 | Top/middle/bottom selects A4/A5/A6 |
| SE, CH8 | Held primer for arm movement |
| Any right-stick diagonal | Zero motion while preserving an existing SE prime; returning to one axis resumes |
| GUI STOP | Stops chassis and arm commands; requires fresh selection/priming |

The last moved selector wins. Startup selects no motor. Selection changes,
SE release and detected radio loss revoke arm authorization. Select a motor,
release SE and center the right stick for 0.3 seconds, then hold SE to prime.
A diagonal cannot establish the initial prime. The dashboard shows raw inputs,
primer state, an amber diagonal stop, cached motor angles and sample ages.
It does not own another motor connection or send extra CAN register queries.

On 2026-11-09 the operator observed keyboard Up (positive raw shaft rotation)
as CCW on all six motors, viewed facing the output shaft. Pocket CW signs are
therefore all `-1`. The operator also reported working Pocket motor control
after restarting the ER12 receiver. These observations do not establish
physical joint/model signs, encoder offsets, or formal stop/failsafe acceptance.

Both interfaces use the existing real `ArmPositionPort`/DM-MC02 routed
position-velocity path. The default jog window is +/-90 degrees about each
motor's session-start reading, with 45 degrees/s jogging. Selecting a different
motor or re-priming does not recenter the window. A readout such as -89.55 degrees
can be the normal settling tolerance at the -90-degree target.

These windows are commissioning bounds. Neutral, diagonal stop and release
remove drive torque through the current adapter; nonselected motors remain
disabled. Keeping the SE prime preserves authorization, not holding torque.
Use the [Pocket guide](POCKET_ARM_CONTROL.md) and
[keyboard/API guide](ARM_POSITION_JOG.md) for the unloaded-fixture launch commands.
There is no assembled-arm hold launch yet.

## Model and constraint work delivered

`simulation.arm_sweep` scans DOF1–6 independently through the full collision
scene, with the chassis and other five motor coordinates fixed at the saved
`terrain_spawn` pose. It solves D3's passive linkage at every pose and records
the first sampled clearance, collision, closure or authored-range boundary.
The viewer cycles through the motions; optional MP4 and JSON exports retain
the visualization and numerical evidence. It never connects to motors.

The default study uses a 2 mm clearance threshold and 1-degree samples,
refining detected boundaries to brackets no wider than 0.1 degree. Candidate
model-angle intervals are:

| DOF | Saved angle (deg) | Candidate interval (deg) |
| --- | ---: | ---: |
| 1 | 0.00 | -53.62 to +53.31 |
| 2 | +5.40 | -102.04 to +12.15 |
| 3 | -25.21 | -59.34 to +55.98 |
| 4 | 0.00 | -180 to +180, search endpoints only |
| 5 | -10.77 | -12.40 to +148.92 |
| 6 | 0.00 | -155.56 to +155.56 |

These are conditional slices around that initial pose, not six independently
combinable hardware limits. D2's upper boundary includes the steel wall. D4's
search endpoints do not establish mechanical stops. Existing contact filters,
two explicit body exclusions, omitted CAD surfaces and convex collision meshes
remain part of the model's limitations. See the
[constraint study](ARM_COLLISION_STUDY.md) for limiting pairs, reproduction
commands and the full audit.

D3 is now exactly **150 mm between ball/connection centers**. The 140 mm bare
rod mesh was already appropriate with its end fittings. The XML correction
removed only 2.488 micrometres of CAD rounding from the far connection site;
it did not stretch the rod mesh. Flat, curved and bumpy saved poses were
reclosed through passive D3 coordinates. The integration pipeline preserves
the dimension and rejects changes beyond 0.01 mm as requiring a geometry
review. An unchanged initial pose can no longer bypass the closure check.
The six candidate intervals were unchanged at the study's resolution.

## Encoder position, session zero and startup

The manufacturer manuals for J8009P-2EC, J4340P-2EC and J4310P-2EC describe
single-turn absolute output-shaft feedback from dual encoders. The encoder can
identify the current angle within a turn after power returns, including after
off-power movement. This does not establish retained multi-turn counts. The
8009P manual specifically limits the initial reported angle to [-pi, pi].
The motors start disabled; knowing the current angle does not command a return
to the model pose. Sources are the pinned
[manufacturer manuals](DAMIAO_ARM.md#verified-manufacturer-references), sections
"Motor Features", feedback-frame notes and the read/write parameter table.
Actual power-cycle/wrap behavior still needs measurement on all six units.

Three references must remain distinct:

| Reference | Meaning |
| --- | --- |
| Raw motor position | Output-shaft radians reported by the motor |
| Jog session zero | First measured raw position captured by `BenchSession.prepare`; UI angle and +/-90-degree window are relative to it |
| Model joint zero | Mechanical reference requiring a measured sign, offset, any external transmission ratio and wrap branch |

Starting the jog tool does not write the motor's persistent encoder zero.
`ArmPositionPort.move_to` preloads the measured current position before enable,
then sends the requested target after enable is confirmed. It does not use the
MuJoCo initial pose as an automatic startup target. Current production profiles
retain null direction/zero calibration and false confirmation flags.

The manuals distinguish `p_m` (0x50, rotor-derived output position) from `xout`
(0x51, direct output-encoder position). During supported, disabled commissioning,
verify how normal CAN feedback and supported readbacks relate on the installed
firmware. Do not add register polling while enabled. Encoder knowledge does
not provide holding torque or prevent a gravity-driven fall.

## Next steps with the assembled arm

The stable acceptance requirements remain in the
[cross-layer TODO registry](ARM_LAYER_TODOS.md). Perform these stages in order;
none is completed by the simulation study or the unloaded jogging reports.

1. **Support the arm and define stopping — SUPPORT-01.** Establish a repeatable
   supported rest pose and suitable fixture, brake or counterbalance for loss
   of power/drive torque. Define emergency stop, command loss, drive fault,
   controlled stopping and supported disable. Verify this arrangement before
   removing support from an energized arm. MC02 disable is not a brake.
2. **Measure absolute-position mapping — CAL-01, DEPLOY-01.** With the arm
   supported and drives disabled, record each motor's identity/firmware,
   raw angle and known physical/model angle at a repeatable pose. Power-cycle
   without movement, then make a controlled off-power position change and
   compare startup feedback modulo one turn. Determine wrap ambiguity, joint
   sign, offset and any external ratio; save a versioned calibration and
   verify it at additional known poses. Do not let the arm fall as a test or
   silently declare each new startup pose to be the model zero.
3. **Qualify joint and path constraints — CAL-01, PLAN-01.** Compare the actual
   assembly with CAD, inspect excluded contact pairs and check cables, hard
   stops and clearances. Test combined configurations and whole motion paths,
   then derive conservative operating limits with stopping/tolerance margins.
   Approach boundaries under support at low speed; deliberate impacts or a
   reactive torque threshold are not the normal limit-calibration method.
4. **Implement assembled-arm holding — ROS-02, BASE-06, SUPPORT-01.** Add a
   separate lifecycle that reads fresh valid angles for all six joints,
   preloads current-position targets, requires explicit arming and holds all
   load-bearing joints together. Define neutral/SE release as stopping a jog
   while maintaining the authorized hold, separately from supported drive
   disable. Preserve the unloaded adapter's existing release-disable behavior.
   A return to a saved/home pose must be a separate requested, path-checked
   motion. Preserve one transport owner, fault latches and explicit recovery;
   never automatically re-enable after a fault or reconnect.
5. **Qualify load and failures — PROTECT-01, BASE-04, BASE-07, DEPLOY-01.**
   Verify motor-side current/protection settings and persistence. With support
   retained, measure holding capacity, temperature, feedback age, bus timing,
   stop latency and behavior on host/USB/CAN/command loss. Progress from small
   supported motions to coordinated motion only with attributable test reports.
   Release production confirmation flags only after their acceptance criteria
   have measured evidence.

## Evidence recorded on 2026-11-09

- Fresh push checks: 43 model/geometry/sweep tests and 30 control/GUI/native/ROS
  tests passed (73 total); no physical motor ports were opened.
- Software: Pocket policy/worker and Qt tests cover diagonal stop/resume,
  fresh priming, selection, stale inputs and GUI STOP. Native C-backed tests
  cover position bounds and authorization withdrawal during enable. ROS/PTY
  integration uses an offline arm and no physical motor ports.
- Simulation: model/geometry/sweep regressions cover all active scenes,
  the exact center spacing, closure and playback. The dynamic preview stayed
  within its 1 mm closure tolerance (maximum observed gap 0.7234 mm). A
  48-second video rendered 1,152 checked poses; the native viewer launched.
- Hardware: measured radio channels, operator-reported shaft directions and
  successful Pocket motion after receiver restart are recorded separately.
  Full radio stop/failsafe acceptance, assembled calibration, holding and
  loaded operation remain pending.
