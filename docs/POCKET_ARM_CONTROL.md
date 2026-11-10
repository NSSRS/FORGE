# Pocket chassis and arm controls

Updated 2026-11-10. Uses the existing Pocket -> ER12 -> ESP32 USB CSV setup. The combined
system launch now uses `pocket_teleop` and `pocket_controls.json`. The old
`keyboard_teleop` radio mode and `radio_pocket.yaml` remain legacy right-stick
chassis tools and must not run beside the new reader. No receiver rewiring,
binding or ESP32/MC02 flash is introduced by this host adapter.

| Physical control | Action |
| --- | --- |
| Left stick up/down | Proportional chassis forward/reverse |
| Left stick left/right | Proportional chassis yaw left/right |
| Right stick left or up | Selected arm motor CW, only while SE is held |
| Right stick right or down | Selected arm motor CCW, only while SE is held |
| SB top/middle/bottom | Select A1/A2/A3 |
| SC top/middle/bottom | Select A4/A5/A6 |
| SE released | Stop/disarm arm motion; chassis remains independent |

The last selector moved wins. Startup selects nothing because both switches
always occupy a position. Move one to select a motor. If both change in one
sample, selection clears; move just one again. Selection changes stop/disarm
even with SE held. After selecting, release SE and center the right stick for
0.3 s, then hold SE and deflect the right stick. Centering stops motion; another
deflection with SE still held can jog again. All four diagonals command zero
whenever both right-stick axes are outside their deadzones. This stops motion
without revoking an existing SE prime: returning to one axis resumes while SE
remains held. A diagonal cannot establish the initial prime. SE release,
selection changes, GUI STOP and radio interruption still revoke authorization;
a radio interruption also requires new selection.

Jog speed is fixed at the existing 45 degrees/s, not proportional. The default
session window is +/-90 degrees from startup. Reaching a boundary does not
restart a held command. Every motor uses the same window relative to its own session-start angle;
selecting another motor or re-priming does not recenter it. The target is
considered reached within 0.5 degrees with low velocity, so a displayed angle
such as -89.55 degrees can be the -90-degree boundary. These are unloaded shaft
commissioning limits, not mechanical joint limits. Nonselected motors are disabled and release removes
torque: this adapter does not hold a loaded arm.

The displayed startup zero is a session reference; it does not reset the
motor encoder or identify the model zero. See the
[assembled-arm handoff](ASSEMBLED_ARM_BRINGUP.md) for absolute encoders,
power-cycle calibration, gravity support and the future holding controller.

## Measured radio mapping

Saved right-stick measurements are horizontal CH1 and vertical CH2
(174/992/1811, measured 2026-10-09). Operator readings on 2026-11-06 add:

| Axis | Channel | Minimum | Center | Maximum | Positive command |
| --- | --- | --- | --- | --- | --- |
| Left vertical | CH3 | 174 (down) | 996 | 1811 (up) | Up: forward, sign +1 |
| Left horizontal | CH4 | 175 (left) | 996 | 1811 (right) | Left: positive yaw, sign -1 |

SB/SC/SE were measured on 2026-11-06 after adding the missing EdgeTX mixes:

| Control | Channel | Top / released | Middle | Bottom / held |
| --- | --- | --- | --- | --- |
| SB (A1/A2/A3) | CH6 | 191 | 1004 | 1792 |
| SC (A4/A5/A6) | CH7 | 191 | 1004 | 1792 |
| SE (primer) | CH8 | 191 | - | 1792 |

All radio channels are now recorded in `src/bringup/config/pocket_controls.json`.
On 2026-11-09 the operator reported keyboard Up (positive raw rotation) as CCW
for each A1..A6 using the documented output-shaft viewpoint. `arm.cw_raw_signs`
is therefore `[-1, -1, -1, -1, -1, -1]`: Pocket left/up requests negative raw
rotation for CW. These observations remove the unknown-sign startup block;
live Pocket acceptance remains pending. The left vertical axis must physically
self-center for release-to-zero chassis control; this has not yet been confirmed.

### If switches disappear from receiver readings

The existing ESP32 sketch decodes all 16 CRSF channels and `radio_monitor`
prints all of them. Empty mixes on CH5..CH8 explained the initial missing
switch readings; CH6..CH8 now have direct SB/SC/SE mixes. For future diagnosis:

1. On the Pocket, inspect the current model's channel monitor while moving
   SB/SC and holding/releasing SE. Physical switch indicators alone do not
   establish that the switches are assigned to output channels.
2. Inspect the model's MIXES page. If CH6/CH7/CH8 are unused, add one mix each:
   CH6 source SB, CH7 source SC, CH8 source SE. Use weight 100%, offset 0,
   no conditional Switch, and zero delay/slow. These are the measured
   assignments for this model. Preserve other existing assignments.
   Direct SE input must return immediately on release; do not use a toggle.
3. Verify CH6/CH7 have three distinct positions and CH8 changes only while SE
   is held, first on the Pocket's channel monitor and then in the PC monitor.
   Record actual raw values in physical top/middle/bottom or released/held order.
4. If the Pocket output channels change but the PC readings do not, inspect
   the ExpressLRS packet rate and switch mode before changing the receiver or
   ESP32. CH6..CH8 support these controls in Hybrid, Wide and Full Resolution
   modes. CH5 has special arming/two-position behavior in Hybrid/Wide; keep
   the selectors off it. No RF-mode change is needed merely to assign mixes.

References: [EdgeTX mixes](https://manual.edgetx.org/bw-radios/model-select/inputs-mixes-and-outputs/mixes),
[EdgeTX channel monitor](https://manual.edgetx.org/bw-radios/main-view),
[ExpressLRS switch modes](https://www.expresslrs.org/software/switch-config/).

### Optional recapture after changing the radio model

The guided tool below captures all four axes as well as the switches. The
recorded stick values can instead be retained when entering switch readings
from the existing read-only `radio_monitor`.

Stop other radio readers, then capture from the existing ESP32 port. The tool
opens only that port: no ROS command publisher or motor connection.

```bash
cd /home/forge2/forge2_ws
python3 scripts/calibrate_pocket.py \
  --port /dev/serial/by-path/pci-0000:00:14.0-usb-0:6.3:1.0-port0 \
  --template src/bringup/config/pocket_controls.json \
  --output build/pocket-controls.local.json
```

Use the actual radio path if it changed. Hold only the prompted control; others
must remain stationary. Switch values are saved in physical top/middle/bottom
order, even if raw values decrease; SE is released/held. Distinct channels,
endpoints and position bands are validated before replacing the output file.

`arm.cw_raw_signs` contains six separately measured motor directions. CW is
viewed looking directly at the output shaft: +1 means increasing raw angle is
CW, -1 means decreasing raw angle is CW. Establish this from observed unloaded
jogging; radio capture cannot determine it. Zero means unknown. Offline
simulation uses +1 for unknown signs; physical arm execution rejects zeros.
Production joint calibration and hardware-confirmation flags remain unchanged.

## USB radio discovery

`pocket.launch.py` and `system_test.launch.py` default to `radio_port:=auto`.
Before starting any nodes, bringup enumerates the observed CH340 adapter type
(USB 1a86:7523), excludes the configured IMU and arm paths, then receives at
115200 baud with an exclusive open. Five consecutive valid fresh 16-channel
CSV rows identify the radio. Each candidate has a 1.5-second deadline; probing
sends no serial command bytes and requests DTR/RTS deasserted before opening.
Probe samples are discarded, each probe closes before teleop opens, and normal
neutral/selection/SE/freshness requirements still apply. MC02/STM32 adapters
are never discovery candidates. Unknown bridge types require an explicit path.

Keep the transmitter and receiver on and close other serial readers before
launch. Zero matches, multiple matches or a busy/inaccessible candidate stop
startup with a diagnostic. There is no fallback to the first ttyUSB device or
a shared by-id link, and no automatic reconnect/re-arm during a running session.
After USB removal/reconnection, restart the stopped launch and reselect/prime.

An explicit `radio_port:=/dev/serial/by-path/ACTUAL_RADIO` bypasses discovery.
The IMU retains its configured socket path; if it moves, update `imu_port` too.
Both configured IMU/arm paths are excluded even if those components are disabled.
On 2026-11-10, the moved radio was verified by received CSV at `/dev/ttyUSB1`,
with by-path `pci-0000:00:14.0-usb-0:6.3:1.0-port0`. The IMU and radio share the
same CH340 identity without unique serials, so their by-id link is ambiguous.

To identify the radio alone, with no ROS nodes or motor connection:

```bash
source scripts/ros2_env.sh
python3 -m bringup.radio_port \
  --exclude /dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0 \
  --exclude /dev/ttyACM0
```

Implementation: `src/bringup/bringup/radio_port.py`; regression tests:
`tests/test_radio_port.py` and `tests/test_system_test_launch.py`.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
source install/ros2/setup.bash
colcon --log-base logs/colcon build --base-paths src \
  --build-base build/ros2 --install-base install/ros2 --symlink-install \
  --packages-select teleop bringup
source scripts/ros2_env.sh

# Real Pocket input, simulated chassis and arm; dedicated test ROS domain.
ROS_DOMAIN_ID=93 ros2 launch bringup pocket.launch.py \
  simulate:=true arm_execute:=false \
  radio_config:=$PWD/src/bringup/config/pocket_controls.json
```

If you recaptured the controls, use `build/pocket-controls.local.json` instead.
Keep test domain 93 free of physical drivers.
Selection/direction/primer status is logged and published on `/arm/teleop_status`.
Chassis requests use `/cmd_vel`. Run only the intended driver in this ROS domain
and stop other keyboard/radio command publishers.

After channel and CW measurements, for the explicitly confirmed secured,
unloaded six-motor fixture (chassis remains simulated):

```bash
ros2 launch bringup pocket.launch.py \
  radio_config:=$PWD/src/bringup/config/pocket_controls.json \
  arm_execute:=true arm_unloaded:=true arm_port:=/dev/ttyACM0
```

The default plan is `src/robot_description/config/arm_jog.json`; `arm_plan` can
override it. Arm physical execution is independent of chassis `simulate` and
`execute`. `use_arm:=false` runs left-stick chassis only. The full sensor launch
uses the same mapping:

```bash
bash scripts/run_system_test.sh simulate:=true \
  radio_config:=$PWD/src/bringup/config/pocket_controls.json
```

Without `simulate:=true`, that wrapper retains its physical-chassis default,
now on the left stick. Arm execution remains opt-in.

## Hardware commissioning next step

The real hardware adapter and firmware path exist. All six `arm.cw_raw_signs`
were recorded as -1 from the operator's keyboard Up=CCW observations on
2026-11-09; the unknown-sign startup block is removed. Resolve the reported
right-stick neutral reading using the dashboard. The operator reported working
Pocket motor control after restarting the ER12 on 2026-11-09; full stop/failsafe
acceptance remains pending. Do not
infer Pocket stop/failsafe acceptance from these direction observations or
from the simulator. See [remaining acceptance items](ARM_LAYER_TODOS.md#plan-01--paths-trajectories-and-input-acceptance).

For the same secured, unloaded six-motor bench (A1 on CAN1, A2..A6 on CAN2), the
existing keyboard window can determine raw rotation signs with a narrower
10-degree session window. Stop the Pocket launch and any other MC02 reader
first. `/dev/ttyACM0` is the previously used MC02 path; substitute its actual
port if enumeration changed.

```bash
cd /home/forge2/forge2_ws
python3 scripts/jog_dm_arm.py --execute --unloaded \
  --plan src/robot_description/config/arm_jog.json \
  --port /dev/ttyACM0 --window-deg 10 \
  --report build/arm_tests/pocket-direction-check.json
```

Select 1..6, press E, then briefly hold Up or Right for positive raw rotation.
Release stops; Space disarms. Viewed facing the output shaft, record whether
that positive rotation is CW (+1) or CCW (-1), separately for each motor. These
keyboard keys use raw positive/negative directions; the Pocket's CW/CCW mapping
uses the measured signs afterward. This test does not require a new firmware
flash if the existing routed position-velocity jogging firmware is installed.
The 10-degree window is relative to each motor's session-start angle: holding
Up stops at +10 degrees and does not extend the boundary. Release and reverse
to move back. The operator reported all six positive rotations as CCW on
2026-11-09; live Pocket acceptance still needs its own evidence.

## Pocket desktop dashboard

`pocket.launch.py` now opens a PyQt desktop window by default (`show_pocket:=true`).
It reuses the keyboard jog window's joint-position table and adds six animated
shaft-angle dials, left/right stick plots, raw readings and neutral indicators,
SB/SC/SE indicators, chassis commands, and precise primer/diagonal messages.
These are shaft readouts, not an articulated robot model. Simulation angles do
not establish the physical CW signs.

Stop the previous Pocket launch and radio monitor before restarting:

```bash
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
ROS_DOMAIN_ID=93 ros2 launch bringup pocket.launch.py \
  simulate:=true arm_execute:=false
```

The right-stick plot shows both axes independently. Any diagonal (both CH1 and
CH2 outside their deadzones) displays an amber stopped state. Once primed, it
says **diagonal: stopped; SE still primed**; returning to one axis resumes
without releasing SE. Single-axis left/up is CW and right/down is CCW. An
off-center status lists the active raw channel. No center/deadzone change is
inferred from a log.
Release SE, center the right stick until **ready to prime**, hold SE until
**ready, SE held**, then deflect it. SB/SC selection still requires moving the
switch; startup does not choose a motor from two simultaneous switch positions.

**Space** or the **STOP CHASSIS + ARM** button requests zero chassis command and
revokes the arm intent. Restarting requires neutral chassis acquisition and
fresh motor selection/SE priming. Closing the window ends this launched session.
Focus loss does not change physical Pocket input. Keyboard number/E/arrow jog
commands are not accepted in the Pocket window; the original keyboard tool
remains separate and retains its own focus-loss stop behavior.

The controller publishes `/pocket/telemetry` at approximately 10 Hz; the GUI
subscribes and sends only `/pocket/stop` (Empty) on stop/close. It never opens
ESP32/MC02 or publishes `/cmd_vel`. One radio reader and one motor owner remain.
The owner copies existing position samples for display without new USB or CAN
requests. Hardware rows can initially be unknown and show increasing ages while
idle; they are updated by existing control/stop feedback. Missing GUI telemetry
is marked stale rather than presented as live motor motion.

`show_pocket:=false` retains the headless launch. The Pocket-only launch now
uses `log_motor_targets:=false` to avoid repetitive RPM output; the full-system
launch retains its previous defaults. Enable that flag explicitly if needed.

Set `arm_report:=$PWD/build/arm_tests/<run-name>.json` on the launch to save the
arm worker's session summary during shutdown. The default is no summary file;
choose a distinct name to retain each run. This forwards the existing node
`report` parameter, without a second owner or live-feedback polling. See the
[file map](SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map).

## Ownership and freshness

One reader owns the ESP32; one separate worker owns MC02. Slow arm preparation
or acknowledgement cannot stall chassis publication. The roughly 10 Hz CSV
and 115200 baud transport are unchanged; host service is approximately 50 Hz.
The 150 ms input deadline subtracts reported ESP32 age. Polling stalls and
oversized queues discard input; publishing cannot renew the original deadline.
The latest-only arm mailbox also carries a generation for stop/selection/
direction changes. Intent is checked inside enable and before every target,
including after slow preparation. SE release during enable cancels the move.

Existing MCU watchdogs, cached feedback, faults and mode cleanup remain. There
is no automatic recovery or re-enable, no new CAN register polling, and no
production loaded-arm lifecycle. Shutdown revokes intent before cleanup even
if ROS publishing has already stopped. The diagnostic CSV still cannot prove
RF health or fully measure USB buffering; transmitter-loss testing remains.

## Verification

Policy tests cover six selections, independent chassis control, SE, directions,
all four diagonal stops and resume with SE held, input expiry, invalid data,
calibration, worker cancellation and
serial stalls. Native C-backed tests check SE withdrawal during enable and
before another command, including disable and mode restoration. A real ROS/PTY
test covers shared input, simulated arm motion, diagonal stop/resume with SE
held, chassis messages, silence and
shutdown. Measured-input replay covers the recorded CH3/CH4/CH6/CH7/CH8
values, all six selections, all four right-stick directions and SE release.
The operator measured the radio inputs; automated tests do not establish
physical motor behavior or RF-loss handling.

This advances PLAN-01 manual input; ROS-02, CAL-01, BASE-06, PROTECT-01 and
SUPPORT-01 remain open in [the registry](ARM_LAYER_TODOS.md).

Initial dashboard validation: 60 Damiao/native-backed/keyboard tests, 23 Pocket policy
and Qt tests, 6 launch checks, and the ROS/PTY telemetry/STOP integration test
passed. An installed-launch smoke test on isolated domain 94 also opened the
GUI, fed measured input values through a PTY, observed simulated joint angles,
verified SE release, and confirmed all nodes exit when the GUI closes. No real
motor ports were opened for these checks.

2026-11-09 push verification: 26 Pocket policy/worker/Qt tests, three native
position-port tests and one real ROS/PTY integration test passed (30 total).
The updated tests include all four diagonal stops/resume with SE held and
retain cancellation checks during enable. All motor feedback in these tests
comes from offline adapters or the native fake plant.

## A1-disconnected, mechanically supported rig test

See [the A2–A6 procedure](ARM_RIG_TEST.md). Its separate plan and radio profile
use 5 degrees/s and operator-controlled manual travel (`window_deg: null`),
with no startup-angle window. This requires MC02 capability 0x8000; image
156f20c8 / capabilities 0xFF8F was verified deployed on 2026-11-10. Each continuous hold retains a 12-second host
timeout (15 seconds independently on MCU). `arm_supported:=true` requires
mechanical support throughout travel and torque loss. A1 is unavailable;
A2–A6 retain the existing SB/SC positions. This is a supported commissioning
mode, not an assembled-arm holding controller. The default six-motor unloaded
profile remains unchanged.

## Controller fault reports

Faults still stop the whole launched session. The position port disables first
and captures cached MCU first-fault evidence; Pocket prints the joint/reason
and measured value/limit when available, then exits nonzero after cleanup.
The `arm_report` JSON retains the original exception, traceback, fault snapshot,
cached states and cleanup outcome. Command-4 rejection is a rejected motion
command, not the firmware's numbered fault reason. The retained fault identifies
which guard triggered. Diagnostics do not clear the latch, extend deadlines,
change thresholds or re-enable motors. See the
[fault findings](DAMIAO_TORQUE_LIMITS.md#earlier-faults-and-current-draw-findings)
and [explicit recovery](ARM_RIG_TEST.md#recover-a-latched-fault-without-unplugging-mc02).
