# Arm session handoff

Updated 2026-11-11, based on branch `nathan` in `NSSRS/FORGE`. This is the current status
for operators and future agents. Keep open requirements in
[the cross-layer registry](ARM_LAYER_TODOS.md), run instructions in
[the rig guide](ARM_RIG_TEST.md), and detailed measurements in dated reports.

## Verified milestone

**The operator confirmed that all five active motors A2, A3, A4, A5 and A6 can
be moved with the manual controls.** This completes the current five-motor
basic-motion debugging milestone. It is operator-observed motion, not a new
automated five-axis acceptance report or calibrated/loaded-arm qualification.

A1 is temporarily disconnected/omitted because the current assembly needs a
base that lets that axis rotate. The operator will build that base next. A1's
earlier unloaded test is recorded in [the ID inventory](ARM_MOTOR_IDS.md) and
[historical commissioning evidence](DAMIAO_ARM.md#current-status--2026-11-05);
it does not qualify A1 in the new mounted assembly.

## Next work, in order

1. **Base and A1 — SUPPORT-01, DEPLOY-01.** The operator builds a stable base
   with clearance for motor 1 to spin. Record the new fixture/topology, then
   verify A1 identity, disabled feedback and controlled motion on that fixture.
   Restore it through an explicit six-joint profile after reconnecting it;
   do not silently change the current A2–A6 subset.
2. **One-pose teaching — CAL-01.** Run the [tester procedure](ARM_POSE_TEST.md)
   to capture the six references, teach or compute each raw goal, and verify
   repeated approaches against the saved MuJoCo pose. Check power-cycle and
   off-power repeatability. Production coordinate initialization and both-sided
   travel calibration remain open.
3. **Faster movement and A6 response — SPEED-01.** Follow the same procedure
   for separate 5, 7.5 and 10 deg/s runs. Compare A6 with A4/A5 and return
   tracking and stationary-stop evidence. Higher rates await fixture testing.

Current protection, coordinated holding, bus timing and fault-injection work
remain open in the registry; they were not completed by basic motor movement.

## Current configuration

| Item | Current state |
| --- | --- |
| Checkout / controller | `/home/forge2/forge2_ws`, DM-MC02 at `/dev/ttyACM0` |
| Active profile | `src/robot_description/config/arm_rig_a2_a6.json`, physical joints 2–6 |
| Routing | A2–A6: CAN2, command IDs 2–6, feedback IDs 18–22; A1 inventory: CAN1/1/17 |
| Operator interface | Keyboard, no radio needed; compact 880 × 420 initial window |
| Motion | Position–velocity mode 2; 5 deg/s; explicit manual travel without a startup-angle window |
| Reactive speed trip | Same 1.5 rad/s on all active A2–A6 motors |
| Reactive torque trips | A2/A3: 12 Nm; A4/A5/A6: 3.5 Nm |
| Remaining bounds | Raw protocol position range, target-distance guard, 12 s host / 15 s MCU duration, command/feedback leases, temperature/fault checks |
| Stop / recovery | Release disables torque; R performs guarded software recovery and stays disarmed |
| Deployed firmware | Read back on 2026-11-10: build `156f20c8`, capabilities `0xFF8F`; manual travel and disabled control-register readback available |
| Production calibration | Unset; saved fixture/protection/production confirmation flags remain unchanged |

The deployed image is locally recorded at
`build/dm_mc02/manual-travel/forge_arm.elf`. Older images in other build folders
are historical; do not flash one based on its filename alone. No additional
flash is needed for the current profile, recovery key or compact window.

## Resume manual controls

With all links mechanically supported throughout movement and torque disable,
and the previous controller owner closed:

```bash
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
python3 scripts/jog_dm_arm.py --execute --supported \
  --plan src/robot_description/config/arm_rig_a2_a6.json \
  --port /dev/ttyACM0 --manual-travel --speed-deg-s 5 \
  --report "build/arm_tests/manual-keyboard-$(date +%Y%m%d-%H%M%S).json"
```

Select 2–6, press E, then hold an arrow. Release stops; Space/focus loss disarm.
After a trip, release arrows, address its cause, press R, wait for **Recovered
and disarmed**, then use fresh E + arrow. R is also available for a retained
startup fault. See [recovery details](ARM_POSITION_JOG.md#recover-a-trip-inside-the-keyboard-window).
Pocket remains optional and requires its ESP32 radio CSV stream.

## Findings to retain

- No hidden 2 Nm command clamp was found in the active jog path. Mode 2 sends
  position and speed, not a torque/current request; displayed `Feedback Nm`
  is a drive estimate and `Trip Nm` is a reactive stop threshold.
- The earlier +/-3-degree target-completion stops were removed by explicit
  manual travel. Session zero is now only a display reference for that mode.
- Earlier A6 trips at approximately +/-0.300365 rad/s were new speed faults
  after recovery at the former 0.3 rad/s threshold. Feedback quantization
  explains the repeated number; physical tracking/overshoot remains to be
  measured during SPEED-01. The operator requested the common 1.5 rad/s trip.
- Disabled control/protection register readback completed for A2–A6 at 19:56.
  A6's relevant loop settings match A4/A5. No current limits or gains were
  changed. The earlier nearly unchanged PSU current observation still does
  not establish phase current or delivered shaft torque; see
  [the current/torque audit](DAMIAO_TORQUE_LIMITS.md).
- Reports retain enabled motion samples, per-joint peaks, session bounds and
  fault evidence. Stale GUI rows now age visibly and blank dynamic readings.

## Evidence and reproduction

On 2026-11-11 the prepared teach/trim workflow and speed trials passed 177
Ubuntu tests covering the native C fake-motor transport, Qt controls and ROS
launch configuration. These include release during enable, A6 trials in both
directions and a stalled shaft. Physical pose agreement, A6 response and the
7.5/10 deg/s rates await the fixture session described above.

Local evidence is ignored by Git; the source/procedure and this summary travel
with the branch. The five-motor milestone above comes from the operator's
2026-11-10 confirmation. Earlier saved evidence includes:

- `build/arm_tests/rig-a2-a6-check.json`: disabled five-motor check.
- `build/arm_tests/recovery-control-20261110-195636.json`: A2–A6 control readback.
- `build/arm_tests/current-debug-20261110-190750.json`: earlier bounded A2 run,
  11 completed moves and peak reported torque 2.468376 Nm.
- `logs/arm_tests/2026-11-10-*`: archived incident, deployment, recovery,
  torque/readback, manual-travel and speed-trip verification with provenance.

[Tests and commands](../tests/README.md#a2a6-supported-rig-a1-disconnected)
cover native C/fake motors, Qt and ROS/virtual-serial integration. Those checks
do not replace physical calibration, current/torque or stopping measurements.

Pre-push verification on 2026-11-10: **169 control/native-backed/Qt/radio/launch
tests, 4 ROS/virtual-serial integration tests and 2 native CTest targets passed**.
The teleop, bringup and robot_description packages built; the STM32 build
reproduced fingerprint `156f20c8`. Local documentation links and whitespace
checks passed. These checks opened no physical motor ports and did not flash
hardware. Logs are retained locally under `logs/arm_tests/2026-11-10-cleanup/`.
