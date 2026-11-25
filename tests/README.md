# Controls, sensors, and robot-model tests

ENCOS motor-4 commissioning: CTest builds socket-free versions of the actual
query and ID-assignment CLIs. `test_encos_cli` checks both buses, empty passages,
one-shot assignment, OLD_ID mismatch, receive-entry reordering and the shared
owner lock. Native fake-CAN tests require all four IDs in slots 0/1/3/4 and
cover W/S/A/D plus stop. Python tests cover bounded direction tests, report/fault
cleanup and profile forwarding. ROS tests use simulated motors only. Run:

```bash
ctest --test-dir build/encos_driver --output-on-failure
source scripts/ros2_env.sh
ROS_DOMAIN_ID=96 PYTHONPATH=tests:$PYTHONPATH .venv/bin/python -m unittest \
  test_encos_commissioning test_motor_test_session test_differential_drive \
  test_unified_controls test_ros_chassis test_motor_mirror
```

`test_weld_response.py` checks the restored v7 baseline against held-out
recorded command/shaft data,
closed-loop second-order fake motors, response mismatch, irregular cadence,
startup/preview/journal agreement, all-sample orientation scoring, periodic-lag
startup exclusion, pause and tracking guards. The panel suite also checks
profile delivery through the worker, planner and journal. The v7 fixture keeps
its original profile and raw recorded cadence/commands/feedback. An exact
561-command regression pins the active v7 profile and startup. Explicit
experimental v8 checks use a test-only profile for the finite startup inverse,
continuous command bounds and fingerprints,
and recorded-residual replay with per-cycle stroke scoring. The accuracy suite
distinguishes center drift across cycles from actual cycle stroke and excludes
incomplete/gapped cycles. These are offline tests, not physical qualification.
Run without motors:

```bash
QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl PYTHONPATH=tests .venv/bin/python -m unittest \
  test_weld_response test_weld_command_sampling test_weld_tracking \
  test_weld_accuracy test_weld_panel test_weld_preview
```

`test_weld_command_sampling.py` replays the Nov 24 command inflation with saved
timestamps and the checked periodic cubic. It checks exact nominal output at
5-100 ms fixed periods and alternating periods, interval-midpoint acceleration
bounds, active correction/projection/saturation, reset, continuous wrap, finite
arrival and fixed pause holds. Run without motors:

```bash
QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl PYTHONPATH=tests .venv/bin/python -m unittest test_weld_command_sampling test_weld_tracking
```

`test_weld_spatial_policy.py` replays the Nov 24 floor-blocked hardware start,
checks advisory space throughout planning/runtime/preview, and retains explicit
mechanical stops, cable bounds and D3 closure. Run without motors:

```bash
QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl PYTHONPATH=tests .venv/bin/python -m unittest test_weld_spatial_policy
```

Follow the [Ubuntu build guide](../docs/ENCOS_WORKSPACE.md#build-on-ubuntu);
`./scripts/build.sh` builds both native drivers and runs CTest: ENCOS codec,
Python MotorBus/native fake-bridge tests, and Damiao firmware tests. The ENCOS
CTest target explicitly selects `test_python_motors` and `test_native_driver`;
it does not discover unrelated ROS/Qt/MuJoCo tests with the system interpreter.
Run the complete suite separately in the configured environment below. A passing
native build alone does not establish ROS/model coverage; report optional skips.

- `encos_protocol_test.c`: query/position/velocity codec vectors and malformed frames.
- `test_weave_reference.py` checks the shared trajectory with NumPy. `test_weave_xy_z.py` checks the fixed CAD gun, 1–3 Hz wheel and wrist motion, and repeat playback. Run both with `.venv/bin/python -m unittest discover -s tests -p 'test_weave*.py'`.
- `test_imu_viewer.py`: orientation rotation, invalid/replayed/stale sample rejection.
- `test_system_test_wrapper.py`: hardware-default elevation, explicit simulation, and CLI override checks using stub commands (no motor access).
- `test_system_test_launch.py`: physical-motor confirmation, separate Ethernet and serial-port guards.
- `test_stereo_cam_network.py`: camera interface selection and non-destructive subnet setup.
- `test_stereo_cam.py`: camera launch argument forwarding and CLI override checks (requires ROS launch).
- `test_imu_transport.py`: Modbus frames, checksums, signed readings and IMU unit conversions.
- `test_motor_test_session.py`: pane/session lifecycle behavior.
- `source_paths.py`: adds the reorganized source packages for test imports.
- `test_python_motors.py`: simulation, independent commands, deadlines, validation and cleanup.
- `test_differential_drive.py`: simulation-derived wheel signs, chassis arcs, RPM
  saturation, motor ordering, timestamp validation and keyboard dead-man stopping.
- `test_radio_input.py`: raw CSV validation, measured calibration, one-stick mixing,
  proportional speed, neutral acquisition, source-age expiry and recovery.
- `test_unified_controls.py`: actual node callbacks with a ROS facade, including
  chassis-only teleop, calibration rejection and publisher-free radio monitoring.
- `test_ros_radio.py`: real pseudo-terminal serial input through the radio teleop
  and ROS chassis simulator, including silence, neutral recovery and USB disconnect.
  Requires ROS 2 and pyserial; never connects to motor hardware.
- `test_ros_chassis.py`: real ROS topics through the existing simulated motor
  driver, including upstream publisher loss; skips without ROS 2. Source the ROS
  environment and run on an isolated ROS domain with no hardware or other publishers.
- `test_native_driver.py`: real C pthread and ctypes ABI against a fake SOEM bridge;
  transport/feedback failures, motor faults, per-motor leases, ownership, shutdown
  and differential forward/left/right/stop commands through all four PDO slots.
  Split-bus tests require ID 3 on CAN2, verify slots `[0,1,3]`, reject invalid
  slot maps, and check that the vacant passage carries no control commands.

The fake library has no raw-socket transport and cannot access motor hardware.
Native integration cases skip unless `ENCOS_TEST_LIBRARY` names that library;
CTest sets it automatically. The normal loader uses `libencos_driver.so`, not
`libencos_driver_test.so`.

The [motor mirror](../docs/MOTOR_MIRROR.md) adds `test_motor_mirror.py` for
feedback mapping and optional MuJoCo model checks, and `test_ros_motor_mirror.py`
for the read-only ROS-to-UDP relay. The latter requires ROS and loopback sockets;
run it on an isolated domain with no hardware publishers.

Portable motor API tests:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_python_motors.py' -v
```

On PowerShell:

```powershell
$env:PYTHONPATH = 'src'
python -m unittest discover -s tests -p 'test_python_motors.py' -v
```

These checks do not validate physical stopping or bridge feedback freshness.

Run the ROS radio integration test on an isolated domain (no hardware processes):

```bash
source scripts/ros2_env.sh
ROS_DOMAIN_ID=84 python3 -m unittest discover -s tests -p test_ros_radio.py -v
```

For the complete native, ROS, Qt and model suite, install the simulation,
calibration and controls search requirements in `.venv`, with system site
packages available for ROS/PyQt5, then build/source the ROS packages:

```bash
source scripts/ros2_env.sh
QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ENCOS_TEST_LIBRARY="$PWD/build/encos_driver/libencos_driver_test.so" \
  .venv/bin/python -m unittest discover -s tests -p 'test_*.py'
```

Keep this ROS domain free of hardware publishers. Optional MuJoCo checks can be
run with the viewer environment after `./scripts/setup_motor_viewer.sh`:

```bash
build/motor_viewer/venv/bin/python -m unittest discover -s tests -p test_motor_mirror.py
./scripts/run_motor_viewer.sh --check
```

`test_camera_frame_buffer.py` checks latest-image replacement and bounded telemetry
when the GUI falls behind. Live ROS callback and display rates are shown in the
camera window; see `docs/STEREO_CAM_SETUP.md` for the measurement boundaries.

Seam edge detection uses synthetic geometry tests and an isolated ROS image test:

```bash
source scripts/ros2_env.sh
source scripts/camera_env.sh
ROS_DOMAIN_ID=91 python3 -m unittest discover -s tests -p 'test_seam_edges*.py' -v
```

The ROS test requires the installed `seam_detection` package. It starts only the
detector and synthetic publisher, never camera acquisition or motor control.
These tests do not establish real-weld recognition or millimeter accuracy.

## Passive ENCOS capture analysis

Run the offline analyzer regression against synthetic Ethernet pcaps and
SocketCAN logs. The tests cover identical CAN replies, retention and DLC
clearing on one motor while the others report, capture clock uncertainty,
and continued velocity commands after EtherCAT input disappears.

```bash
PYTHONPATH=tests .venv/bin/python -m unittest test_encos_capture -v
```

## Motor telemetry for both families

`test_motor_telemetry.py` covers per-field freshness, missing/nonfinite feedback,
snapshot generation replay/wrap, conservative timestamps, units and passive
consumers. `test_native_driver.py` also verifies ENCOS snapshot ABI, independent
field times, transport-fault counters and unchanged command leases.
`test_ros_motor_telemetry.py` exercises both ROS drivers together with simulated
ENCOS and a fake Damiao owner, including command callback latency and stale
feedback. [Topic and timing semantics](../docs/MOTOR_TELEMETRY.md) distinguish
ENCOS PDO observation time from Damiao MCU feedback age.

After the native build, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src:$PWD/tests:$PWD/src/motor_control${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=93 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ENCOS_TEST_LIBRARY="$PWD/build/encos_driver/libencos_driver_test.so"
python3 -m unittest test_motor_telemetry test_native_driver test_python_motors test_unified_controls
python3 -m unittest test_ros_motor_telemetry test_ros_chassis
python3 -m unittest test_damiao_arm test_damiao_bench
```

Keep the ROS domain isolated from hardware nodes. These tests open no physical
motor transport and do not establish real bus timing or loaded acceptance.

## Damiao arm

BASE-05 tests verify typed USB rejections, active MCU route and limit readback,
legacy host/firmware compatibility, valid feedback CAN ID zero, and the
absence of CAN sends or lease updates from rejected validation and readback.
Run CTest and `test_damiao_arm` for the new wire and C-backed Python checks.

`damiao_arm_test.c` verifies the portable firmware codec, leases, feedback loss,
fault reset, rollover, CAN failure and framed USB protocol. `test_damiao_arm.py`
checks the host profile/protocol and drives the real C firmware through fake CAN
for six joints. `bash scripts/build.sh` includes both. No test opens motor hardware.
See [arm implementation](../docs/DAMIAO_ARM.md) for the separate STM32 build.

The native Damiao round-trip test covers both selected bus maps and checks 50
control frames per joint over 100 ms of simulated MCU time (500 Hz). This does
not measure wire utilization, CAN arbitration latency or physical motor response.

## Current chassis and arm MJCF

Use the root .venv with src/simulation/requirements.txt for simulation tests. The dedicated
model tests skip when optional simulation dependencies are unavailable.
Generate terrain scenes before the collision broad-phase tests:

```bash
.venv/bin/python scripts/preview_sim.py --terrain curved --check
.venv/bin/python scripts/preview_sim.py --terrain bumpy --check
.venv/bin/python -m unittest discover -s tests -p test_model_compatibility.py -v
.venv/bin/python -m unittest discover -s tests -p test_magnetic.py -v
.venv/bin/python -m unittest discover -s tests -p test_velocity_control.py -v
.venv/bin/python -m unittest discover -s tests -p test_terrain.py -v
```

The full model has 20 joints, ten actuators and one D3 connect closure.
Model compatibility tests cover default joint angles/targets, the rebased D1,
static D3 solving, and wheel-feedback projection without changing arm/base pose.
See [model guide](../docs/ONSHAPE_ARM_PREVIEW.md) for remaining physical limitations.

### Current Damiao bench regressions

Position–velocity is the only bench motion mode. Python tests cover
host/firmware round trips, wrong mode readback, speed/gain rejection, simulated
target completion and stalls, delayed enable, restoration, watchdogs and both
six-joint bus maps. Two CTest targets cover codec/state machine/USB framing
and CAN completion/abort. The general MIT codec remains for the separate
production interface; obsolete MIT bench variants are removed.

```bash
cmake -S src/damiao_firmware -B build/damiao_tests
cmake --build build/damiao_tests
ctest --test-dir build/damiao_tests --output-on-failure
PYTHONPATH=src python3 -m unittest discover -s tests -p test_damiao_arm.py -v
```

Hardware evidence and remaining model/unit tests are in
[arm commissioning](../docs/DAMIAO_ARM.md) and the
[description handoff](../src/robot_description/ARM_COMMISSIONING.md).

### Routed P-series and six-motor bench

`test_damiao_bench.py` adds an independent six-motor CAN plant, exercising the
real C firmware with the shared host runner. Run both Damiao suites with
`PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_damiao*.py' -v`.
Coverage includes both bus topologies, signed excursions/returns, factory reply
ID 0, model ranges, group enable, wrong identity/mode, stalls, missing feedback,
speed/torque/lease trips, restoration and immutable unconfirmed templates.
See [the test guide](../docs/DAMIAO_MOTOR_TESTS.md) for physical commissioning.

`test_arm_a2_range_setup.py` covers the disabled-only 28-to-40 TMAX transaction
through real native C firmware and fake motors: original-setting evidence before
write, unchanged current/gain settings, independent readback before save, missing
ACK/no retry, stale/moving/subset rejection, persistence across a fake power
cycle, restart-only motion lock, required CLI flags and report preservation.
Changing live VBus readings remain recorded without blocking write/save checks;
genuine gain changes still fail both before and after save.
These checks do not constitute physical drive or flash-persistence acceptance.

`test_arm_a2_load.py` checks the separate 30 Nm A2 load profile with real native
firmware and fake motors: exact slot/bus/ID scoping, older-firmware rejection,
unchanged default and older-profile limits, rejection of 28/30 Nm feedback ranges for
a 30 Nm trip, correct decoding after matched TMAX=40 readback, 30 simulated
seconds above rated torque without an
added overload timer, positive/negative torque trips, both 80 C temperature
trips and whole-group disable. `test_weld_hold_analysis.py` checks preserved
profile evidence, readable temperature peaks/changes and missing readings.
`test_arm_gun_load.py` extends these checks to A3=30 and A4-A6=10, including
per-axis signed trips, 80 C stops, saturation rejection, capability gating and
cached torque/temperature recording without a new overload timer.
`test_arm_load_range_setup.py` covers all five range changes in one disabled
session, skipping already-configured A2, fixed joint/value whitelists, sequencing,
readback-before-save, no uncertain retries, complete configuration comparison,
restart-only motion lock and persistence across a fake power cycle.
These are software tests, not sustained-load or thermal qualification.

```bash
PYTHONPATH=src:tests QT_QPA_PLATFORM=offscreen .venv/bin/python -m unittest \
  test_damiao_arm test_damiao_bench test_damiao_keyboard test_arm_group_hold \
  test_arm_a2_load test_arm_a2_range_setup test_arm_gun_load test_arm_load_range_setup
```

## Arm clearance study

`test_arm_geometry.py` checks exact 150 mm D3 connection-center spacing across
all active scenes, preserved CAD meshes, geometry-review rejection and closure
repair when no independent joint command changed. Large D3 edits are compared
with a dense continuous reference, including repeated straight/standby changes;
failed intermediate closure must restore the entire previous pose.

`test_arm_sweep.py` checks the full-scene clearance scan, first-boundary brackets,
D3 closure, fixed independent coordinates/chassis, contact filters, and rejection
of invalid reference poses. It never commands hardware or advances physics. Run:

```bash
.venv/bin/python -m unittest discover -s tests -p test_arm_sweep.py -v
```

## Full-robot modeling compatibility

With src/simulation/requirements.txt installed, run model_compatibility, motor_overview_models,
body_colors, magnetic, terrain, velocity_control and motor_mirror test modules.
The overview test uses synthetic named feedback across flat, curved and bumpy
full-robot scenes; it verifies status overlays and frozen arm/base coordinates.
No motor hardware or ROS runtime is needed for those tests.

### A2–A6 supported rig (A1 disconnected)

The suites also cover the six-axis profile and optional keyboard position hold:
fixed release targets, >15-second hold/jog execution, requested speed with a
lagging position-loop plant, bounded target lead during a stall in both directions,
uninterrupted resume, selection locking, focus-loss pause, explicit
disable, capability/mode gating, single-axis firmware authorization, retained
command/feedback watchdogs, torque trips and disarmed recovery. These use the
native firmware and fake motors; they are not physical load/thermal acceptance.

`test_arm_group_hold.py` adds complete/sparse group activation at current positions,
selection without disabling, fresh jog authorization, fixed nonselected targets,
position-loop lag, whole-group shutdown on an unselected joint's torque/feedback
fault or missing command lease, old-firmware/partial-mask/mixed-policy rejection,
activation cancellation, display reads without lease refresh, disarmed recovery,
Qt focus/close and UI-stall handling. All motor access uses the fake native plant.

`test_arm_capture.py` checks eight-position recording without mandatory notes:
A1/A4/A5/A6 initial and both CW/CCW ends for each of A2/A3, with no invented model
mapping for the ends. Completion and Qt Next require both endpoint saves;
failed second saves preserve the first. It also checks optional endpoint/initial recording in both raw-angle
orders, complete-group context, stationary/freshness/wrap validation, partial
progress, file reuse rejection, atomic save failures and slow background writes.
Native-backed capture adds no bus requests or disables; Qt/CLI tests cover the
capture button and finishing an outstanding save on normal shutdown. These are
software checks, not physical calibration evidence.
Startup tests inject A1 drive status 9 and missing A4 feedback: diagnostics
identify the joint, the GUI stays visible and locked, and recovery never enables
motors or clears a nonexistent controller fault. Existing active-owner rejection
and stationary/feedback interlocks remain covered.

`test_arm_smooth_jog.py` exercises the new command through the native firmware
and fake CAN plant: capability rejection before motor operations, selected-only
smoothing, immediate release/pause/selection targets, all-six 10 deg/s travel,
lagging A2 at 5 and 10 deg/s in both directions, old-firmware speed gating,
holding-mode CLI defaults, expired command leases and disarmed recovery.
`damiao_smooth_jog_test.c` checks individual 2 ms CAN targets, endpoint saturation,
immediate hold bypass, direction changes, delayed ticks, rollover, invalid
commands, feedback loss and transmit failure at 5 and 10 deg/s. CTest runs it as `damiao_smooth_jog`.
No test result establishes physical jitter improvement.

The [operator guide](../docs/ARM_RIG_TEST.md) and
[current milestone](../docs/ARM_SESSION_HANDOFF.md) distinguish physical
observations from these hardware-free regressions. Install `python3-pyqt5` for
Qt checks; source the ROS environment for launch/PTY tests.

```bash
source scripts/ros2_env.sh
QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH="tests:src:$PYTHONPATH" python3 -m unittest -v \
  test_arm_capture test_arm_smooth_jog test_arm_group_hold test_arm_pose_teach test_damiao_arm test_damiao_bench test_damiao_keyboard \
  test_pocket_controls test_pocket_gui test_radio_port test_system_test_launch
ROS_DOMAIN_ID=94 PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH="tests:src:$PYTHONPATH" python3 -m unittest -v test_ros_pocket
```

Keep domain 94 free of physical drivers. The tests use native C firmware/fake
CAN motors, Qt offscreen events and virtual serial devices; none opens physical
motor hardware. Do not count skipped optional ROS/Qt cases as passed coverage.

| Boundary | Coverage |
| --- | --- |
| Routing / feedback | Physical A2–A6 slots with absent A1, another missing motor, retained A1 routes, disabled capture without enables or parameter writes |
| Manual travel | Beyond former startup window, current-position preload, release/intent expiry, duration/protocol guards, old-firmware rejection |
| Profile / limits | Baseline 4340P 12 Nm ceiling, scoped load-profile extensions and legacy rejection; independent lower torque trips; all active joints accept the old speed-trip region and fault above 1.5 rad/s in both directions |
| Recovery | Evidence saved before reset, retained disabled/stationary checks, range-cache repair, lost ACK, disk/readback/reset failures, no automatic enable; session bounds/original modes survive keyboard R |
| Readback / diagnostics | Disabled-only model register whitelist, wrong identities, missing/nonfinite values, original fault, enabled sample/peak retention, Feedback Nm versus Trip Nm |
| Keyboard / display | Selection, E/held arrows/release, conflicts, focus loss, UI stall, startup fault lockout, threaded recovery, fresh input after recovery, aging stale rows |
| Pocket | SE priming, diagonals, deadlines, input withdrawal during enable, unavailable A1, worker startup/motion/cleanup faults and saved errors |
| Radio / launch | USB relocation/ambiguity/exclusions, receive-only probe, stale/malformed CSV, busy ports, virtual-serial cleanup, authorization before discovery |
| ROS / PTY | Real node topics with offline motors, telemetry/STOP, supported sparse selection/manual travel, silence/shutdown and fault-report exit |

The existing native firmware CTest targets are listed above. Build the target
STM32 firmware separately using `scripts/build_dm_mc02.py` and the toolchain/SDK
instructions in [DAMIAO_ARM.md](../docs/DAMIAO_ARM.md); this build never flashes.
Physical bus timing, current/torque accuracy, position calibration, higher speeds,
loaded holding and fault-stop latency remain acceptance work in
[the open registry](../docs/ARM_LAYER_TODOS.md).

### Model-guided arm calibration

Install `src/simulation/calibration-requirements.txt` in the model environment.
`test_arm_calibration.py` covers path authorization/cancellation, one-degree
waypoints through the fake C firmware, complete fresh-pose validation, constraint
identity/backoff, disk failures, Space with text-field focus, and actual model
closure/adjacent-stop checks. No hardware is opened.

```bash
QT_QPA_PLATFORM=offscreen PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -p test_arm_calibration.py -v
QT_QPA_PLATFORM=offscreen PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -p test_arm_capture.py -v
```
### Independent arm pose mirror

`test_arm_pose_mirror.py` exercises raw position decoding through the native C
fake motors/MC02, physical slot/ID ordering, disabled manual-motion feedback,
active/fault/missing-axis rejection and absence of enable/target/parameter writes.
It also checks stale passive cache behavior, direct affine mapping without wrap
or gearing, view-only edits/full-precision reset, separate observation snapshots,
and actual MuJoCo drive coordinates/D3 closure without physics or collision gating.
All tests and GUI rendering are offline; encoder alignment remains a physical test.

`test_arm_endpoint.py` checks live/disabled/settled gates, cable ends independent
of model zero, straight-pose offset fitting, rejection of swapped/invalid captures,
invalidation of old opposite endpoints and preservation of other axes. Mirror
tests exercise the three buttons with a fake source and the actual CAD shaft-axis
reference. `test_arm_cable_limits.py` checks fixed A4/A6 +/-270-degree bounds across
session restarts, rejection of extra revolutions and exact cubic overshoot.
The native fake-motor group test checks holding at the edge and reversing without
wrapping or dropping the other holds. These are software/model checks, not physical
cable or alignment qualification.

```bash
QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python -m unittest discover -s tests -p test_arm_pose_mirror.py -v
PYTHONPATH=tests .venv/bin/python -m unittest test_arm_endpoint test_arm_cable_limits -v
```

## Supported weld-motion controller

Nov 19 publication validation: **200 weld tests, 183 motor/commissioning tests,
49 shared-model/calibration tests and three native CTests**, plus the offline
weld audit and ARM firmware build `f78f6b0d`. Two initial weld model-fingerprint
rejections coincided with terrain regeneration; restoring generated scenes and
running those cases sequentially passed. The 64 endpoint/hardware tests were
also rerun after the A3 startup helper cleanup. These are software/model results;
physical acceptance remains in the [handoff](../docs/ARM_SESSION_HANDOFF.md).

Weld dashboard verification (Nov 20): 166 tests passed across `test_weld_dashboard`,
`test_weld_panel`, `test_weld_hardware`, `test_weld_recording`,
`test_weld_preview`, `test_damiao_keyboard`, `test_arm_group_hold`,
`test_damiao_arm`, `test_arm_capture` and `test_pocket_gui`. Offline state
renders fit 1366×768; manual telemetry was checked at 1000×768. UI preview
screenshots use synthetic readings and do not establish hardware acceptance.

Run all weld tests without hardware:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl \
  .venv/bin/python -m unittest discover -s tests -p 'test_weld_*.py' -v
.venv/bin/python scripts/control_weld.py --audit
```

For additional shared-model coverage, run this **after** the weld suite; terrain
tests rewrite model inputs and must not run concurrently with planning tests:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl \
  PYTHONPATH=tests .venv/bin/python -m unittest test_arm_geometry \
  test_model_compatibility test_motor_overview_models test_body_colors \
  test_magnetic test_terrain test_velocity_control test_motor_mirror \
  test_arm_calibration -v
```

Terrain tests regenerate scene XML; preserve any pre-existing edits, then review
or restore only test-generated scene changes after the suite completes. Physical
evidence is in the [Nov 19 review](../docs/measurements/arm-gun-load-review-2026-11-19.json)
and [Nov 17 archive](../docs/measurements/weld/2026-11-17/README.md).

| Suite | Contract |
| --- | --- |
| `test_weld_measured_start.py` | Hardware measured-start acceptance, unchanged simulation limits, collision warnings versus checked moving paths, fixed A6 cable bounds, full-model recovery and strict handoff before approach; recorded Nov 23 start, bounded route search and fake-shaft replay |
| `test_weld_accuracy.py` | Cached joint diagnostics, full tool-orientation error with separate target/maximum, rotation-frame invariance, scene/mapping identity, synchronized workpiece-relative tip data, and incomplete evidence rejection |
| `test_weld_hold_analysis.py` | Offline HOLD-01 timing and torque/speed summaries; separate interrupted attempts, reject stale/faulted samples, deduplicate cached feedback, preserve gaps and shutdown uncertainty, and flag changed comparison paths/settings |
| `test_weld_constraints.py` | Calibration identifiability, output-shaft units, physical/host limits and exact retiming |
| `test_weld_encoder_branch.py` | Explicit simulation branches, nearest equivalent hardware A1 targets in both directions, native mapping synchronization without transport, unchanged other-axis/cable limits, and Nov 24 starts including the post-reset CCW recovery through full model checks and fake shafts |
| `test_weld_hardware.py` | Shared state machine, native C fake-CAN arm, ENCOS simulator, continuous periodic execution/pause, fixed hold, retained ready plan and same-owner fault reset without automatic enable |
| `test_weld_preparation.py` | Signed interval acceleration bounds, fast-route A5 lag regression, pacing/stall/timeouts, speed allowances and unchanged motion guards |
| `test_weld_tracking.py` | PID with a bounded preparation clock under unequal/delayed response, recorded A6 overshoot/two-cycle regressions, tiny-axis correction isolation under full clearance checks, finite-leg braking at variable tick intervals, ordered recovery stops, command derivative limits, anti-windup, fixed endpoint and pause holding, stall/displacement guards, full recorded-route geometry, repeated welding, measured Hz and insufficient-motion rejection |
| `test_weld_preparation_guard.py` | Nov 24 A3 motion-guard feedback replay, unchanged nominal clock under healthy/ahead feedback, current-tick lag detection, and the failed plan through the unfiltered model with ideal/lagged/delayed/quantized shafts and recorded/variable ticks; original tracking/correction/arrival and command derivative bounds remain checked |
| `test_weld_hold_activation.py` | Nov 24 fresh-preload/older-plan mismatch through the full model, signed 0.5-degree boundaries across plan loading/start/preload/activation, fixed-target drift rejection, checked-start settling deadline, reused hold without recapture, and cached six-slot target access without transport |
| `test_weld_preparation_search.py` | Nov 16 <=5 s ideal replay; Nov 17 raised approach with unequal lag and full geometry; Nov 19 long-wrist start at explicit 30 deg/s under 10 s with ideal shafts; final lowering minimum preserves selected staging, earlier timings, geometric cubics and limits, including the recorded loaded start with synthetic lag; independent stopped-leg timing, required lift, acceleration reserve, hardware opt-in and legacy offline-plan rejection |
| `test_weld_recovery.py` | Bounded depth-first route search and backtracking, start/preload validation, waypoint arrival and cancelled activation |
| `test_weld_planner_performance.py` | Interval sampling bounds and retiming invariance, exact joined endpoints, contact-mask equivalence against the full model, cache invalidation/copy isolation and optimistic stopped-motion bounds; latency itself belongs in the seeded benchmark |
| `test_weld_endpoint_recovery.py` | Recorded A3 contact separation, 0.25 deg startup boundary, unchanged other limits, explicit inward-only activation, pause/replan, stale/outward rejection, strict handoff, preview and full optimized model replay; fake-CAN command guards are also in `test_weld_hardware.py` |
| `test_weld_model.py` | Actual D3/triangle-clearance/inertia checks, complete-route fake-shaft replay and repeated weld boundaries |
| `test_weld_dashboard.py` | Cached temperatures, signed torque/trip display, age rejection including the missing-feedback sentinel, bounded stage/run peaks, preparation clocks and distinct state/pause causes |
| `test_weld_panel.py` | Worker ownership, ready hold across focus/heartbeat loss, active-motion cancellation, startup fault reset/R, queued-action rejection, reset failure/close, preview isolation, pause-cause journals, compact layouts with retained first-fault evidence, and stable viewer/plot geometry during live text/frame updates at narrow, desktop and 4K sizes |
| `test_weld_preview.py` | Saved-start validation, nominal settling intervals, actual MuJoCo rendering and contact diagnostics; out-of-range measured-pose display without clamping or weakening target/command limits; planner exception serialization |
| `test_weld_recording.py` | Bounded asynchronous JSONL, cached snapshots, overflow and disk errors; automatic report paths, retained interrupted runs and isolation of writers across relaunches |
| `test_weld_search.py` | CMA-ES mobility semantics, objective constraints and stale-identity rejection |

The default audit uses the measured-bound standby. `--plan-only --cycles 1`
also checks the full approach. Native/model tests require the existing compiler
and optional simulation dependencies; the full suite may take several minutes.
Search dependencies are in `src/controls/search-requirements.txt`.

Use the [seeded planner benchmark](../docs/PLANNER_BENCHMARK.md) for planning
latency, random-start success and the historical five-second motion goal
(retired after the operator accepted 8.22 s on Nov 23). Keep
timing runs separate from this suite and model-mutating tests.

Post-arrival tests impose six-axis disturbances, verify that target/allowance
remain fixed, check cached drift/recovery events, and reject welding start when
the arm leaves its arrival tolerance. Endpoint pause preserves the settled target;
interrupted travel still captures a measured hold. These fake plants do not
identify physical stiffness, friction, backlash or load capacity.
Native regression also counts one read/command per axis per moving tick, rejects
samples expired during calculation, and verifies ten seconds of fixed holding
after repeated pauses and an injected displacement. A minimum-allowance plant
checks that welding does not throttle itself near arrival or latch a collapsed
speed limit into the final hold. Fast preparation tests cover 20–100 ms ticks.

The [exported animation](../docs/WELD_MOTION.md#render-the-actual-optimized-plan)
checks model/mapping/constraint identity, continuity and derivative limits before
rendering. Preview and fake-shaft replay do not establish physical tracking,
loaded holding, stopping or five-second hardware preparation.

## Welding machine Modbus

From the repository root, install the optional welder dependency and run the
loopback TCP and POSIX pseudo-terminal RTU checks.

```bash
.venv/bin/python -m pip install -e '.[welder]'
PYTHONPATH=tests .venv/bin/python -m unittest test_welding_machine -v
```

See the [welder bring-up procedure](../src/welding_machine/README.md).
