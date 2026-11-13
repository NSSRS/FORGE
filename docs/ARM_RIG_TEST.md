# A2–A6 rig test, A1 temporarily disconnected

The operator verified manual movement of all five A2–A6 motors on 2026-11-10.
For reference capture, model-pose replay, the new six-axis fixture and
opt-in speed trials, use the [next bench test](ARM_POSE_TEST.md).
The inventory remains A1/CAN1 and A2–A6/CAN2 with command
IDs 1–6 and feedback IDs 17–22. `arm_rig_a2_a6.json` selects physical joints
2–6 without renumbering IDs or MCU slots. A1 is unavailable. Another missing
motor fails the check; the code never silently drops a joint or clears faults.

This is a disabled connection/encoder check followed by optional slow manual
jogging. It is not gravity holding, homing, joint calibration, or an automatic
collision-limit test. Model sweep angles are not sent to the motors.

The [source and artifact map](SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map)
lists the exact owning files and report/log folders. The canonical source checkout
on this machine is `/home/forge2/forge2_ws`.

Manual-only six-axis controls: `bash scripts/control_arm.sh`. Position recording
is a separate application (`bash scripts/record_arm_positions.sh`); model-guided
calibration and obstacle retuning are in [ARM_MODEL_CALIBRATION.md](ARM_MODEL_CALIBRATION.md).

## Current keyboard workflow

The current operator path uses the keyboard, without a radio.

```bash
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
python3 scripts/jog_dm_arm.py --execute --supported \
  --plan src/robot_description/config/arm_rig_a2_a6.json \
  --port /dev/ttyACM0 --manual-travel --speed-deg-s 5 \
  --report "build/arm_tests/manual-keyboard-$(date +%Y%m%d-%H%M%S).json"
```

Select 2–6, press E once, then briefly hold an arrow. Up/right is raw-positive
(previously observed CCW); down/left is raw-negative. Release stops. Space,
focus loss, selection change or close disarm. A1's button is disabled and its
keyboard key stops/disarms with an unavailable message. Simulation accepts the
same plan/manual-travel/speed with `--simulate` instead of `--execute --supported`.

The compact window starts at 880 × 420 pixels and grows if needed for fault text.
After a trip, release arrows, address the cause and press **R / Recover fault**.
It saves evidence, checks every retained motor is disabled/stationary, then clears
the software latch. Wait for **Recovered and disarmed**, then use fresh **E +
arrow**. A retained startup fault opens in lockout with R available. See
[keyboard recovery](ARM_POSITION_JOG.md#recover-a-trip-inside-the-keyboard-window).
The keyboard uses no radio. Current firmware already supports it; normal
sessions need no reboot or flash.

## Optional selected-joint position hold

The default jogger is a short bench test: a held jog expires after 12 seconds
on the host, with a separate 15-second MCU enable cutoff. Release disables
torque, so a gravity-loaded joint can fall. These duration bounds are separate
from the 150 ms command lease and 100 ms MCU feedback watchdog.

`--hold-position` explicitly selects a different keyboard policy for supported
fixture trials. It requires matching MC02 firmware with capability **0x10000**;
the previously deployed `156f20c8` image cannot run it. Follow the
[build/flash procedure](DAMIAO_ARM.md#build-flash-run), with motor PSU off and
all links mechanically supported, then verify the new build identity before use.

After a disabled six-axis check, launch from this checkout:

```bash
python3 scripts/jog_dm_arm.py --execute --supported --manual-travel \
  --hold-position --speed-deg-s 5 \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 \
  --report "build/arm_tests/hold-$(date +%Y%m%d-%H%M%S).json"
```

- Select a joint while disabled, press **E**, then hold an arrow. E alone
  authorizes motion; the first arrow performs current-position preload and enable.
- **Release** captures the fresh measured position once and keeps the selected
  joint enabled at that fixed mode-2 target. A further arrow resumes jogging
  without disabling/re-enabling. Drive position control supplies holding torque;
  no guessed constant torque, current limit or gain is written.
- Jogging starts with 100 ms of target lead, then advances at the selected
  rate with up to 5 degrees of forward lead over feedback. This accommodates
  position-loop lag: a 0.5-degree lead cap reduced recorded A2 motion from
  approximately 5 to 0.6 deg/s. At the cap, missed travel is discarded;
  release still replaces the moving target with the measured release position.
  This host correction needs no firmware reflash. Its physical retest is pending.
- **Space / Escape / focus loss** stop travel and retain the hold, while
  revoking arrow authorization. Use fresh E and arrow to resume. The UI shows
  HOLDING separately from motion authorization.
- **D / Disable torque / Q / closing the window** disable torque. Mechanically
  support the load before these actions. Selection is locked while the selected
  joint is enabled; explicitly disable before changing joints. Other joints stay
  disabled. This is not coordinated whole-arm holding.
- This mode has no 12/15-second bench cutoff. Its fixed hold target is refreshed
  by the existing control loop; telemetry reads do not renew the lease. Command
  loss, stale feedback, stalled UI, USB/CAN faults, drive faults and configured
  speed/torque/temperature trips still disable and require explicit recovery.
  A cancelled initial enable also disables. Power loss cannot provide holding.

First acceptance: under mechanical support, raise A2 only a small distance and
release. Measure hold drift, tracking error, torque feedback and temperatures;
test Space/focus loss, fresh E resume, and a supported D disable. Cross 15 seconds
only after the initial short hold behaves correctly. Save the report and physical
observations. Software/fake-motor checks do not qualify loaded holding or thermal
capacity; keep production confirmation flags false.

## Hold all connected joints

For group holding, use the explicit mode below; the original `--hold-position`
selection lock remains appropriate for a single energized joint.

`--hold-all` adds an explicit **H / Hold all connected joints** activation.
It implies `--hold-position` and still requires supported manual travel. A new
firmware image with capability **0x20000** is required; the earlier selected-joint
hold firmware intentionally rejects group enable. The prepared build is
`82794a1b` / capabilities `0x3FF8F`, saved under `build/dm_mc02/group-hold/`.
Do not flash, close a holding session or turn off power while the load depends
on motor torque. Park/rest or restrain the complete arm first. These commands
do not qualify unsupported operation or replace a brake/restraint on torque loss.

After flashing with motor PSU off, reconnect motor power and complete the
disabled six-joint check. Then launch from the task checkout:

```bash
python3 scripts/jog_dm_arm.py --execute --supported --manual-travel \
  --hold-all --speed-deg-s 5 \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 \
  --report "build/arm_tests/group-hold-$(date +%Y%m%d-%H%M%S).json"
```

The window starts disabled. **H** captures fresh stationary positions for all
configured joints, preloads each target, validates the complete enable mask and
enables the group. It stays disarmed for jogging. Use **1–6**, **E**, then an
arrow; release captures the selected joint's measured position once. Other
targets remain fixed. Selecting another joint pauses travel and disarms jogging
without disabling any joint; use fresh E and arrow after selection. Repeated H
does not adopt passive drift as new targets. Space/focus loss hold the whole
group and disarm movement. The UI shows fresh feedback for every held joint.

**D / Q / closing** disable the complete group. Any joint's trip, missing
feedback or expired command lease also disables the group. Recovery remains
explicit and disabled; another H is required. The firmware rejects partial
group enable, mixed group/single-axis hold policies and individual enable on a
group-configured slot. No background owner or motor parameter polling is added.

Firmware and fake-motor/Qt checks passed, and an operator-run group session is
recorded. A1/A4/A5/A6 moving jitter and an A4 hold overspeed need investigation;
see the current handoff. Physical loaded holding, CAN timing and thermal behavior
remain unqualified. Do not interpret average 5 deg/s travel as smoothness acceptance.

## Optional smooth jog trial

The operator reports no improvement from the 5 deg/s smoothing trial and requested
doubling all six jog speeds. Build `d61f6d16` / capabilities `0xFFF8F` enables
10 deg/s `--smooth-jog`; holding-mode defaults are now 10 deg/s. It advances
moving targets at the existing 2 ms CAN cadence. Release/Space/focus loss still capture a fixed hold target
without applying the jog ramp to that stop. It changes no drive gains or trips.
The forward-lead limit scales with speed (5 degrees at 5 deg/s, 10 at 10) to
avoid slowing the lagging A2/A3 position loops. This trial has passed software
checks; physical jitter reduction is unverified.

Park/rest or restrain the arm so it cannot fall when torque is removed. Close
the previous owner and turn motor PSU **off** before flashing. From this checkout:

```bash
sudo /home/forge2/Documents/Codex/2026-10-30/read/tools/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
  -c port=SWD freq=240 mode=NORMAL reset=SWrst \
  -w "$PWD/build/dm_mc02/jog-10/forge_arm.elf" -v -rst
```

Only after programming/verification succeed, turn motor PSU on. With the fixture
able to carry the links on torque loss, run:

```bash
run="build/arm_tests/jog-10-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$run"
python3 scripts/test_dm_arm.py --all --check --feedback --supported \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 --report "$run/disabled.json" &&
python3 scripts/jog_dm_arm.py --execute --supported --manual-travel \
  --hold-all --smooth-jog --speed-deg-s 10 \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 --report "$run/holding.json"
```

The UI should say **Smooth jog (2 ms)**. H holds all, then 1 → E → arrow tests
A1. Start with a brief press each direction within known clearance, releasing
between directions; compare moving smoothness and steady hold. Repeat for A4,
A5 and A6, then use A2 as a comparison. Space pauses/holds; D/Q/close/fault
removes all torque. Park/rest before closing to save the report.

For an A/B session on the same firmware, omit `--smooth-jog` and use a different
report path. Holding reports include firmware identity, target policy and every
existing host sample in an 18,000-row ring. At six joints and 50 Hz that is
roughly the most recent minute; sample timing depends on actual owner cadence.
Targets in this report are host endpoints, not a capture of every smoothed CAN
target. Feedback/command batch durations and whole owner-poll time are recorded
without extra control traffic; cached MCU timing/error counters are read after
disabled cleanup. A physical CAN capture is still needed to resolve faster oscillations.

## Fixture and first startup

All attached links, including the unpowered A1 axis, must be mechanically
supported throughout the test and on torque loss. Support must carry the
weight while allowing the selected manual motion; do not drive a joint against
a rigid restraint. `--supported` / `arm_supported:=true` is the operator's
confirmation of that arrangement. If the arm can fall when disabled, arrange
support before running either the feedback check or physical jogging.

Stop the previous Pocket/keyboard launch and any other MC02 owner. The commands
below assume the existing temporary CAN wiring and MC02 at `/dev/ttyACM0`;
substitute its actual path if enumeration changed. They do not manage the PSU.

If MC02 still retains a six-motor session from before A1 was disconnected, its
disabled-feedback check will fail waiting for A1. With the arm supported and
motor power off, restart MC02 once to clear the old volatile routes, then restore
motor power for the check. This is explicit recovery after a topology change,
not a routine reset between successful runs. Do not automatically clear a fault
or reset MC02 while the arm relies on motor torque. The updated manual-travel profile requires firmware capability **0x8000**.
Build **156f20c8** / mask **0xFF8F** was verified deployed by passive
readback on 2026-11-10. The corresponding image is
`build/dm_mc02/manual-travel/forge_arm.elf`; it also includes disabled control
readback (0x4000). No additional flash is required. Older images refuse this manual launch before motor setup.

## Connection and encoder check

From the workspace:

```bash
cd /home/forge2/forge2_ws
python3 scripts/test_dm_arm.py \
  --plan src/robot_description/config/arm_rig_a2_a6.json \
  --all --check --feedback --supported \
  --port /dev/ttyACM0 \
  --report "build/arm_tests/rig-a2-a6-check-$(date +%Y%m%d-%H%M%S).json"
```

Here `--all` validates the complete six-motor inventory; `active_joints` in the
plan limits this check to A2–A6. The output lists their raw angles, feedback
ages and CAN addresses, and explicitly lists A1 as omitted. Expect `PASS` and
five fresh, stationary, disabled rows. Investigate any failure before jogging.

This checks the existing ID/range/mode/damping registers, configures volatile
MC02 routes for slots 1–5 and requests fresh normal feedback with disable
frames. It never enables a motor, sends a movement target, writes motor
parameters, saves a zero or changes the motor's mode. It does remove torque.
The JSON retains raw radians/degrees and routing; angles are not model angles.
The current firmware does not expose direct `p_m`/`xout` register reads, so this
test does not claim to verify those readbacks or power-cycle/wrap behavior.

For offline plan validation, omit `--check --feedback --supported`; no serial
device opens. For register checks without a new feedback capture, use `--check`
alone; an existing session can still require disabled-feedback refresh.

## Recover a latched fault without unplugging MC02

Stop the previous Pocket/keyboard process. Check the cause of the trip (for
A2, support, clearance and binding) with all links supported. Then run:

```bash
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
python3 scripts/test_dm_arm.py \
  --plan src/robot_description/config/arm_rig_a2_a6.json \
  --all --check --feedback --supported --recover \
  --port /dev/ttyACM0 \
  --report build/arm_tests/rig-a2-a6-recovery.json
```

`--recover` is explicit authorization to clear the controller fault after the
cause has been addressed. It requires the disabled check, support confirmation
and a report path. Normal checks and Pocket launches still leave faults latched.
The existing firmware supports this command; no reflash or USB unplug is needed
for a recoverable fault such as the recorded A2 torque trip.

Recovery captures the original fault, verifies every retained motor has fresh,
stationary disabled feedback, and saves a unique companion file
`build/arm_tests/rig-a2-a6-recovery.fault-<unique>.json` before issuing reset.
The summary JSON records that path, reset attempt/acknowledgement/verification,
pre/post states, and the subsequent normal ID/encoder check. Companion evidence
survives reuse of the summary filename. If saving evidence fails, no reset is
sent. Reset invalidates the MCU range cache, so recovery re-probes the existing
routes and confirms fresh disabled feedback again. Motor parameters, modes,
thresholds and encoder zeros are unchanged; no enable or movement target is sent.

Expect `Recovery complete; motors remain disabled`, five A2–A6 encoder rows and
`PASS`. Reopen the keyboard, select a motor and explicitly arm again.
Recovery never resumes a jog. If no fault is latched, the option verifies the
retained disabled state and runs the check without sending a reset.

Missing retained motors (including A1 from an old six-motor session), ongoing
motion, drive errors, invalid/hot feedback or lost communications prevent a
verified recovery. A missing motor in the requested new plan fails the normal
check even if recovery itself succeeded. Resolve the reported cause rather
than repeatedly clearing faults. The option does not remove old routes, reboot
a hung USB device or unlock an ID-commissioning session; topology/ID-session
restarts described above remain separate. A reset acknowledgement timeout is
reported as uncertain, never silently retried; the preserved evidence remains
available for the next explicit check.

## Optional Pocket jogging

Build the changed launch/config packages once:

```bash
source scripts/ros2_env.sh
colcon --log-base logs/colcon build --base-paths src \
  --build-base build/ros2 --install-base install/ros2 --symlink-install \
  --packages-select teleop bringup robot_description
source scripts/ros2_env.sh
```

After the check passes, and while all links remain supported:

```bash
ROS_DOMAIN_ID=93 ros2 launch bringup pocket.launch.py \
  simulate:=true radio_port:=auto arm_execute:=true arm_supported:=true \
  arm_port:=/dev/ttyACM0 \
  arm_plan:="$PWD/src/robot_description/config/arm_rig_a2_a6.json" \
  radio_config:="$PWD/src/bringup/config/pocket_controls_rig.json" \
  arm_report:="$PWD/build/arm_tests/rig-a2-a6-pocket-$(date +%Y%m%d-%H%M%S).json"
```

Radio discovery follows the ESP32 when it changes USB sockets. Turn on the
transmitter/receiver and close other serial readers before launch. It receives
five fresh CSV rows from exactly one eligible adapter before starting nodes;
no match, multiple matches or busy ports stop startup. Explicit
`radio_port:=/dev/serial/by-path/ACTUAL_RADIO` still overrides discovery. See
[USB identification and exclusions](POCKET_ARM_CONTROL.md#usb-radio-discovery).

Chassis remains simulated. Right-stick left/up=CW, right/down=CCW. SB middle
and bottom select A2/A3; SC selects A4/A5/A6. SB top shows A1 unavailable and
revokes motion authorization. Select a connected joint, release SE, center the
right stick for 0.3 seconds, then hold SE and briefly deflect one axis.

This profile requests **5 degrees/s with operator-controlled manual travel**
(`window_deg: null`). The operator requested removal of startup-angle constraints
for this commissioning stage. All four diagonals stop without revoking an
existing SE prime. Centering, diagonals and SE release disable torque; other
joints remain disabled. The 12-second host / 15-second MCU cutoffs, measured
speed/torque/temperature trips and communication watchdogs remain. Finite raw
protocol bounds (+/-min(12.5, PMAX) rad) remain; these are not mechanical limits.

For offline rehearsal with the same five-joint selection, use
`arm_execute:=false` and omit `arm_supported:=true` and `arm_port`.
Do not run keyboard and Pocket hardware owners together.

The Pocket summary is written to the timestamped `arm_report` path during
session shutdown. Use a different filename for each retained run; the same
destination is overwritten. Omitting `arm_report` leaves summary-file output
disabled. Console/ROS logs are separate, normally under `~/.ros/log/`.

## Disabled current/loop-parameter inspection (2026-11-10)

Disabled readback completed for A2–A6 at 19:56; see
[the register values and interpretation](DAMIAO_TORQUE_LIMITS.md). The deployed
156f20c8 firmware supports capability 0x4000. With the previous controller owner
closed and all links supported, a new read can be recorded with:

```bash
python3 /home/forge2/forge2_ws/scripts/test_dm_arm.py \
  --plan /home/forge2/forge2_ws/src/robot_description/config/arm_rig_a2_a6.json \
  --all --check --feedback --supported --inspect-control \
  --port /dev/ttyACM0 \
  --report "/home/forge2/forge2_ws/build/arm_tests/rig-control-registers-$(date +%Y%m%d-%H%M%S).json"
```

This disables and verifies stationary feedback before each register read; it
never enables, commands movement, writes gains/limits or saves parameters.
Normal checks and launches never automatically inspect these registers. It
records raw bits, decoded types and read status per motor. A no-reply/nonfinite
value makes the inspection incomplete; do not substitute zero or keep retrying
to hide it. A wrong reply identity aborts. The 8009P's undocumented Imax/VBus
are explicitly skipped. `PASS` means the documented reads completed, not that
factory limits, current/torque calibration or persistence are validated.
Fault recovery remains separate and explicit; inspection does not clear faults.

## Limits and evidence

The supported baseline is **5 degrees/s**. Explicit manual travel removes
startup-angle windows; relative zero is only a session display reference.
Bounded mode remains separately available with a window of at most +/-3 degrees.
All active A2–A6 measured-speed trips are **1.5 rad/s**, at the operator's request.
Torque trips are **12 Nm for A2/A3**, **3.5 Nm for A4/A5/A6**. These are reactive
stops, not current commands or measured load capacity. Existing 12 s host / 15 s
MCU cutoffs, protocol range/target-distance guards, leases, temperature stops
and fault latches remain. Continuous holding and calibrated paths are separate
work; see [open requirements](ARM_LAYER_TODOS.md).

The shared table separates `Feedback Nm` from `Trip Nm`, shows feedback age,
and blanks stale dynamics. Shutdown reports retain enabled motion samples,
sampled peaks, session origins/bounds, startup registers and faults. Peaks are
host-sampled, not every CAN frame or an independent torque/current measurement.
Use unique report paths; close the window to write the full session report.

The operator's five-motor motion confirmation is recorded in the
[current handoff](ARM_SESSION_HANDOFF.md). Earlier incidents/readback and software
verification remain in ignored local `build/arm_tests/` and `logs/arm_tests/`.
Reproduce fake-motor/Qt/ROS checks using [tests/README.md](../tests/README.md).
Basic motion does not complete calibrated startup, faster-speed qualification,
current protection, loaded holding or the physical failure matrix.
