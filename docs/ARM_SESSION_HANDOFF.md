# Arm session handoff

Updated 2026-11-12, based on branch `nathan` in `NSSRS/FORGE`. This is the current status
for operators and future agents. Keep open requirements in
[the cross-layer registry](ARM_LAYER_TODOS.md), run instructions in
[the rig guide](ARM_RIG_TEST.md), and detailed measurements in dated reports.

## Verified milestone

**The operator confirmed that all five active motors A2, A3, A4, A5 and A6 can
be moved with the manual controls.** This completes the current five-motor
basic-motion debugging milestone. It is operator-observed motion, not a new
automated five-axis acceptance report or calibrated/loaded-arm qualification.

A1 is now reconnected. The operator confirmed base rotation/cable clearance
and mechanical support. All six motors passed disabled identity/feedback checks
on 2026-11-12 using `arm_rig_a1_a6.json`. A1's measured-speed trip now matches
A2–A6 at 1.5 rad/s; the command remains 5 deg/s. Physical tracking, stopping and
loaded-hold acceptance remain open. The A2–A6 subset is still available when A1
is deliberately disconnected.

## Recorded — initial positions and both A2/A3 travel ends

All eight requested positions were recorded on 2026-11-12. The
[operator reference record](../src/robot_description/config/arm_reference_positions_2026-11-12.json)
preserves the supplied degree values and derived radians. The original
[full-precision capture](measurements/arm-reference-2026-11-12.capture.json)
contains matching samples (all eight agree to three decimals), timestamps,
group poses and firmware build `d61f6d16` / capabilities `0xFFF8F`.
The input `-174,692` is interpreted as A1 = -174.692 degrees.

| Joint | Initial raw angle | CW raw angle | CCW raw angle | Raw span |
| --- | ---: | ---: | ---: | ---: |
| A1 | -174.692° | — | — | — |
| A2 | — | -141.731° | -3.115° | 138.616° |
| A3 | — | +152.922° | +72.139° | 80.783° |
| A4 | +173.183° | — | — | — |
| A5 | +8.754° | — | — | — |
| A6 | +64.161° | — | — | — |

A2 increases from CW to CCW; A3 decreases. These labels do not establish a
model-coordinate sign or linkage ratio. A2/A3 initial poses and the other four
joints' two-sided bounds remain unknown. No runtime limits, encoder zero or
automatic homing were installed. Repeatability and production qualification
remain open.

The captures passed the recorder's stationary/freshness gates: selected feedback
ages 6–11 ms, 26–27 samples over 0.500–0.521 s. This does not establish loaded
shutdown acceptance. The same run exited 2 after cleanup: all six final rows
reported disabled status 0, but A2 still reported -0.168498 rad/s, failing the
0.03 rad/s stationary check. `disable_acknowledged` and
`mode_restoration_verified` stayed false. Preserve this unresolved stop/restore
result in [the session summary](measurements/arm-reference-2026-11-12.session-summary.json).
The agent only read these files; no new motor commands were issued.

### Recording workflow

The operator deferred jitter investigation under SPEED-01 and returned to
CAL-01 on 2026-11-12. The current task is A1 front/initial, A2 CW then CCW ends,
A3 CW then CCW ends, and A4/A5/A6 initial. Eight saves; no notes questionnaire.
This supersedes the earlier one-position-per-motor interpretation.
`scripts/jog_dm_arm.py --hold-all --capture-file ...` now defaults to this
reference mode. The portable `scripts/record_arm_positions.sh` launcher writes `initial-positions.json` and prints
the physical instructions. `--capture-mode bounds` retains the longer two-end
plus initial sequence for later. Both use cached feedback while all connected
joints hold. C saves after
release/Space and 0.5 s stationary feedback. Saves run off the control thread.
Each capture includes all joint angles to preserve collision/configuration
context. A new file is required per run; no zero, production limit or model
mapping is installed. Version 2 files declare per-joint capture steps; A2/A3
each keep labeled CW/CCW raw readings and a sorted raw interval after both saves.
Completion and Next require both ends. No A2/A3 initial pose or model mapping is
inferred. Old files remain unchanged and are not resumed across sessions. A5
uses initial, superseding its older CW-stop teaching reference. See
[the procedure](ARM_POSE_TEST.md#current-procedure-initial-positions-and-a2a3-travel-ends).
This is a host-only addition; no new firmware flash is required. The agent has
not launched, stopped or commanded hardware for this change. Existing group
hold still drops torque on D, close, fault or power loss; a powered arm must be
rested/restrained before changing applications.

The operator now confirms the undervoltage issue is resolved. The active
`positions-20261112-182417-O8mGD2` capture file remained empty when both keyboard C
and the capture button appeared to return to ordinary controls. The screenshot
shows the same capture window still holding, not evidence of a process crash.
Offline reproduction found that focus loss replaces capture results with a
generic pause message; key C with empty Qt text was also ignored, but that does
not explain the button failure. The original rejection reason was not retained,
so the exact physical capture blocker is unknown. The UI now keeps a dedicated
capture result, live readiness with per-joint rejection reasons, saved raw angles,
and an explicit Next selection button. C uses its Qt key code. Capture attempts
and write results are retained in `capture_events` in the session report. Freshness,
settling checks and group holding are unchanged. Offline keyboard/button/focus
and fake-controller checks pass; the running hardware process was not restarted.

### Capture startup failure

`positions-20261112-181021-HtBywT/session.json` contains exit 2 and `fresh
stationary disable acknowledgement`; the old constructor exited before opening
the window and did not retain per-motor startup evidence. The now-available
`jog-10-20261112-173834/holding.json` verifies build `d61f6d16` / capabilities
`0xFFF8F` and ended with failed disable cleanup. Its last A1 feedback has drive
status 9 (undervoltage per the 8009P manual), stopping=1 and invalid feedback age;
A2–A6 have disabled status 0 and fresh feedback. This is preceding-run evidence,
not a fresh hardware measurement or proof that undervoltage persists.

Disabled acknowledgement timeouts now name the failing joints and preserve the
last states. With the GUI's explicit `allow_faulted_start`, these failures open
a fault-locked window instead of exiting before display. No H/E/jog is accepted
until the operator resolves the physical cause and presses R for a successful
disabled recovery. The normal timeout/disable checks remain unchanged; active
owners are still rejected. The agent did not open the hardware for this diagnosis.

The next attempt, `positions-20261112-181508-Tab0zA/session.json`, now preserves
the startup states and confirms A1 alone blocked the check: status 9,
stopping=1, invalid normal-feedback age (`UINT32_MAX`). A2–A6 are disabled with
10–11 ms feedback ages. No recovery was attempted and no reference was captured.
All rows later looking stale in the locked window is expected: it displays
cached startup samples rather than polling while faulted. A1's status does not
have a separate error-frame receipt timestamp, so this evidence cannot separate
continuing undervoltage, a retained motor fault, or subsequent loss of A1 CAN1
feedback. The operator confirms a 24 V PSU shared by A1–A6; voltage at A1's
connector and its power-branch integrity have not been measured. 24 V is within
the 8009P manual's 24–48 V operating range. GUI R
uses existing controller recovery and does not send the motor Clear Faults
frame; do not promise it clears a latched drive error.

## Deferred motion issue — A1/A4/A5/A6 jitter while jogging

The original scratch analysis is now versioned as `motors.arm_jitter_report`,
with the portable entry point `scripts/analyze_arm_jitter.py`. To reproduce its
JSON/CSV from the original local session report, run:

```bash
python3 scripts/analyze_arm_jitter.py \
  build/arm_tests/group-hold-20261112-165434/holding.json \
  --output-dir build/arm_tests/jitter-analysis
```

This is a historical analysis, including that trial's operator observations and
hypotheses. It verifies the original report hash before applying those notes;
it does not diagnose other runs or access hardware. Full motion traces remain
local under ignored build directories.

The operator ran group hold and reports steady stationary holding, smooth A2/A3
travel, and jitter on A1/A4/A5/A6 during continuous jogging. Saved physical run:
`build/arm_tests/group-hold-20261112-165434/holding.json`. It ended with disable
and mode restoration acknowledged; a prior fault and explicit recovery remain
preserved inside `recoveries` and `holding.fault-c8qrj882.json`.

Retained jogs average approximately 5 deg/s, but A1/A4/A5/A6 reported speed
ripple is substantially larger. Host sample intervals remain about 20 ms,
feedback age is 0–2 ms, and the forward target lead cap never engages in those
jogs. A1 ripple was also present in the earlier single-joint runs. The leading
hypothesis is stepped position targets interacting with motor/drive dynamics:
the baseline host advances about 0.1 degree per 20 ms and the 2 ms MCU tick repeats the
latest target without interpolation. A2/A3 are 40:1 4340P units; A1 is 9:1 8009P
and A4–A6 are 10:1 4310P. Observed following error is about 1 degree on the
jittering joints versus 3.7–3.8 degrees on A2/A3. These differences support a
model-dependent response but do not prove which loop or mechanism causes it.

Separately, A4 reported **2.4542 rad/s (140.6 deg/s)** against its 1.5 rad/s trip
while A4 was holding and A2 was jogging. Command/feedback ages were 6/0 ms;
the fault target was 0.743 degrees behind measured position. This event is
unexplained. The operator disputes that physical speed; this is one reported
feedback value, not independently verified motion. Correlate encoder changes
and raw feedback before attributing it to real motion or a reading error;
do not raise the trip to hide it.

The report retained only the last ~55 seconds of ~9 Hz trace samples; 16,500
earlier samples were evicted. It cannot resolve 50 Hz excitation or the lead-up
to that A4 trip. Next: retain high-rate command/feedback evidence and isolate
MCU-side target interpolation versus drive/load response under support. Keep
release holding and fault leases intact; read current tuning only disabled.
No motor commands or gain/current changes were made during the investigation.
Analysis outputs are in the task's `outputs/motor-jitter-evidence.json`
and `outputs/motor-jitter-comparison.csv` (outside this checkout).

### Current requested trial — 10 deg/s on all six joints

The operator reports that the 5 deg/s smoothed run still jitters, including
vibration felt by hand, and explicitly requested a 100% speed increase.
`--hold-all` / `--hold-position` now default to 10 deg/s for every selected
joint. The forward-lead bound scales from 5 degrees at 5 deg/s to 10 degrees
at 10 deg/s so A2/A3 do not hit the old bound and slow down. Missed travel at
the bound is still discarded; release/pause bypasses the ramp. Teaching and
non-holding defaults remain 5 deg/s, all measured-speed trips remain 1.5 rad/s,
and torque trips/leases/current and drive gains are unchanged.

Prepared firmware **d61f6d16**, capabilities **0xFFF8F**, is pinned in
`build/dm_mc02/jog-10/`. Capability bit 19 (`0x80000`) authorizes smooth commands
above 5 through 10 deg/s. The host rejects the old 5 deg/s smoothing firmware
before H sends any preload/enable at 10 deg/s; explicit 5 deg/s remains usable.
The agent has not flashed or launched the new build. 212 Python/native-backed/Qt
tests and 3 native CTest targets passed, including every joint at 10 deg/s in
both directions, the lagging A2 plant at 5 and 10, immediate holds, and leases.

The saved `build/arm_tests/smooth-jog-20261112-172602/holding.json` now verifies
build `d6057996`, capabilities `0x7FF8F`, smoothing enabled, no latched fault,
exit 0 and acknowledged disable/restoration. It contains 27 jog/hold pairs,
but its last-minute trace evicted all A1/A4/A5/A6 jogging (44,600 samples dropped).
Only an A2 jog remains in the retained motion trace: median host interval
20.008 ms, maximum 26.197 ms, feedback age 0 ms and no lead limiting. Whole-run
summaries record an approximately 124 ms maximum host sample gap, without enough
context to attribute it to moving jitter. Do not claim timing is cleared.
`build/arm_tests/jog-10-20261112-173834/disabled.json` is also present. Its motion
report is now saved; the A1 undervoltage/cleanup evidence is summarized under
Capture startup failure above. Do not attach another serial owner for diagnostics.

Code audit found no periodic safety pause/resume: trips latch and disable.
The owner performs six sequential feedback requests and six command requests;
the 20 ms serial timeout is a maximum wait, not a mandatory delay. CAN keeps
running independently at the configured 2 ms cadence. CAN sends wait for actual
transmit completion (2 ms failure budget), and the smooth ramp discards missed
tick time; actual tick lateness could reduce its rate and remains unmeasured.
The earlier group report had ~20 ms host cadence, 0–2 ms feedback age and no
lead limiting in the retained jogs. That does not establish the latest run's
timing. Low-speed drive/load response remains possible; doubling speed is a
comparison trial, not a proven jitter fix.

New reports add `owner_poll_ms`, `feedback_batch_ms`, `command_batch_ms` to
existing owner samples without extra active I/O, plus cached MCU bus-health
counters read after disabled cleanup. These allow the next saved run to expose
host latency and late MCU ticks. MCU counters are cumulative since boot.

### Research: physical torque ripple versus drive/timing effects

Manufacturer sources describe position-dependent cogging torque, energized
torque ripple, friction/stiction and servo tuning as possible causes of uneven
low-speed motion. This is a plausible mechanism, not a diagnosis or an excuse
to accept the current jitter. The gearing split is relevant: at a 5 deg/s output
rate, A1's 9:1 rotor turns 7.5 rpm; A4–A6's 10:1 rotors turn 8.33 rpm; A2/A3's
40:1 rotors turn 33.33 rpm. At 10 deg/s these become 15, 16.67 and 66.67 rpm.
Ratios are from the installed models' Damiao manuals; these calculations do not
establish a minimum smooth speed. Maxon's low-speed examples are not Damiao
specifications. Steady holding and failed host-smoothing improvement raise the
priority of motor/drive/load investigation but do not exclude command timing.

The saved smoothing session reads Deta (register 31) as 4.0 on all six motors,
matching the manual's recommended starting value. Do not assume an incorrect
Deta or change it blindly. Mode 2 uses internal position/velocity/current loops;
MIT command kp/kd guidance does not apply to the zero MIT fields in mode 2.
Next distinguish repeatable bumps per angle (motor/gear/encoder periodicity)
from oscillations at a fixed time frequency or command gaps, using short saved
traces/video at both requested speeds. Compare current loop/filter settings
only during a supported disabled session; no tuning/calibration was performed.

Primary references:
- [Maxon: cogging and torque ripple](https://support.maxongroup.com/hc/en-us/articles/6726327972252-Meaning-and-impact-of-Cogging-torque-and-Ripple-torque)
- [TI: cogging and torque-ripple effects](https://www.ti.com/document-viewer/lit/html/SSZTBL1/GUID-9F1C872B-7B49-4C59-A9AB-FDF04E4E61CA)
- [Kollmorgen: motor properties and control mitigation](https://www.kollmorgen.com/en-us/blogs/everywhere-cogging-torque-and-torque-ripple-what-you-need-to-know)
- [ADVANCED Motion Controls: stiction and hunting](https://www.a-m-c.com/experience/technologies/motor-characteristics/friction/)
- [Damiao 4310P manual](https://github.com/dmBots/DM-J4310P-2EC/blob/9e728c9a42b498380ae43964636b3d66d01f42c7/manual/DM-J4310P-2EC%20Geared%20Motor%20User%20Manual%20V1.3%202026-09-14.pdf)

### Earlier smoothing trial — operator reports no improvement

The operator tried smoother targets at the same 5 deg/s. Build **d6057996**, capabilities
**0x7FF8F**, is pinned in `build/dm_mc02/smooth-jog/`; the agent has not flashed
or launched it. Logs and hashes are in `logs/arm_tests/smooth-jog/`.

USB type 25 slews from the last transmitted position toward the host endpoint
by at most `speed × 2 ms` per existing CAN tick. It never extrapolates beyond
that endpoint, catches up missed ticks in one jump or refreshes the host lease.
It requires confirmed enabled mode-2 manual holding and speed <=5 deg/s.
Normal type 4 remains immediate: release/pause captures the measured position
and cancels the ramp; other joints keep fixed holds. Removing `--smooth-jog`
selects the original target steps on the same firmware for comparison.

209 Python/native-backed/Qt tests and 3 CTest targets pass, including individual
2 ms CAN targets, endpoint saturation, direction changes, immediate release,
late ticks, rollover, stale feedback/commands, selection and simulated lagging
A2 at 5 deg/s in both directions. These validate software behavior, not jitter
reduction or physical stop distance. The operator subsequently reported no
jitter improvement. Holding reports record firmware identity
and every existing host sample in an 18,000-row trace (roughly 60 seconds for six
joints at 50 Hz). The logged target is the host endpoint, not every MCU target;
raw CAN timing and the separate A4 feedback spike remain unresolved.
See [smoothing trial instructions](ARM_RIG_TEST.md#optional-smooth-jog-trial).

## Group holding implementation and observed run

The operator requested keeping all joints powered at their current positions
while switching the selected jog joint. `--hold-all` now provides explicit H
activation, current-angle preloads for the complete configured group, fixed
nonselected targets, selection without torque disable and fresh E authorization
after selection. Release/Space/focus loss retain the group hold. D/close/fault,
any expired command lease and power loss still remove torque; this is not a brake.

Firmware build **82794a1b**, capabilities **0x3FF8F**, is built and saved as
`build/dm_mc02/group-hold/forge_arm.elf`, with hashes and verification evidence
under `logs/arm_tests/group-hold/`. The operator's subsequent physical group run
passed the new capability gate; the exact deployed build has not been independently
read back. No secure rest/restraint for torque loss is confirmed. Do not close
a holding owner, disable, reset or flash while the arm relies on its torque.
Park/rest or restrain the arm before those transitions and disabled checks.

204 Python/native-backed/Qt tests and 2 CTest targets passed. Tests cover all-six
and sparse-group preloads, selection/jog/release without disable, lagging A2 jog
response, fixed nonselected targets, single-joint faults stopping the whole group,
partial/mixed-group rejection, cancellation, UI stalls and disarmed recovery.
These tests do not qualify physical CAN timing, loaded holding or thermal limits.
See [group controls](ARM_RIG_TEST.md#hold-all-connected-joints) for operator steps.

## Selected-joint hold and A2 speed correction

The operator requested hold-on-release after A2 dropped when jogging disabled
torque. The optional keyboard `--hold-position` path and matching firmware build
`21a6e20a` / capabilities `0x1FF8F` were built and software-tested. The operator
was given DIY flash/run commands and subsequently recorded an enabled hold-mode
session, so the hold capability gate passed. The exact deployed build identity
has not been read back independently.
It holds the selected joint at a fixed measured release position; Space/focus
loss retain holding and disarm motion. D/close/fault disables. It removes the
12/15-second bench cutoff only for this explicitly selected mode, preserving
command/feedback leases. See [the complete procedure](ARM_RIG_TEST.md#optional-selected-joint-position-hold).

The saved A2 run `build/arm_tests/hold-20261112-163446/holding.json` showed only
0.59–0.64 deg/s sustained travel for a 5 deg/s request. The host's 0.5-degree
lead cap was active almost continuously; the earlier ramp achieved 4.99 deg/s
with about 3.8 degrees of following error. Hold-mode jogging now allows the
5 deg/s ramp to build up to 5 degrees of forward lead, discarding missed travel
at that cap. Initial preload and fixed release holding are unchanged. This is
a Python-only correction; the firmware image does not need rebuilding/flashing.
A lagging fake-motor regression reproduced 0.62 deg/s before the correction and
requires 5 +/-0.15 deg/s in both directions after it. Physical retest is pending.
All 190 Python/native-backed/Qt regression tests passed after the correction;
none accessed physical motor hardware.

The recorded hold session ended with **disable acknowledgement and mode
restoration unconfirmed**. The operator closed that window, then later restarted
it (see above), and says the arm cannot currently be supported. Restore mechanical support before any further
physical check or jog; then require a passing disabled six-axis check before
relaunch. Do not use the shutdown error as proof that the arm is safely held.
Original build/test details remain in `logs/arm_tests/position-hold/`; speed-fix
evidence is in `logs/arm_tests/hold-jog-speed/`. Loaded holding remains unqualified.

## Next work, in order

1. **A1 and support — SUPPORT-01, DEPLOY-01.** Save the fixture/topology record
   and complete controlled-motion/stopping acceptance on the new base. The
   six-joint disabled identity/feedback check has passed. Qualify the requested
   selected-joint/group holding modes and corrected jog rate under mechanical support.
2. **Reference validation — CAL-01.** The eight requested raw positions are now
   recorded above. Verify power-cycle/off-power repeatability, measure the
   remaining bounds and A2/A3 initial poses, and establish model sign/offset/ratio
   before deriving startup goals. Production coordinate initialization and
   physical travel qualification remain open.
3. **Faster movement and A6 response — SPEED-01.** Follow the same procedure
   for separate 5, 7.5 and 10 deg/s runs. Compare A6 with A4/A5 and return
   tracking and stationary-stop evidence. Higher rates await fixture testing.

Current protection, coordinated holding, bus timing and fault-injection work
remain open in the registry; they were not completed by basic motor movement.

## Current configuration

| Item | Current state |
| --- | --- |
| Checkout / controller | Task checkout `/home/forge2/Documents/Codex/2026-11-12/pu/work/FORGE`; DM-MC02 at `/dev/ttyACM0` |
| Active profile | `src/robot_description/config/arm_rig_a1_a6.json`, physical joints 1–6 |
| Routing | A2–A6: CAN2, command IDs 2–6, feedback IDs 18–22; A1: CAN1/1/17 |
| Operator interface | Keyboard, no radio needed; persistent capture results and guided Next steps |
| Motion | Position–velocity mode 2; holding-mode jogs 10 deg/s by operator request, non-holding/teaching 5 deg/s; explicit manual travel |
| Reactive speed trip | Same 1.5 rad/s on all active A1–A6 motors |
| Reactive torque trips | A1: 20 Nm; A2/A3: 12 Nm; A4/A5/A6: 3.5 Nm |
| Remaining bounds | Raw protocol position range, target-distance guard, leases, temperature/fault checks; default bench 12/15 s cutoffs, optional hold mode has bounded forward target lead and no duration cutoff |
| Stop / recovery | Default release disables; optional holds retain torque on release/Space/focus loss, D/close/fault disables; physical group run recorded, load/fault qualification pending; R stays disarmed |
| Deployed firmware | Saved 2026-11-12 capture verifies `d61f6d16` / `0xFFF8F`; no new agent flash |
| Production calibration | Unset; saved fixture/protection/production confirmation flags remain unchanged |

The deployed 10 deg/s image is pinned at `build/dm_mc02/jog-10/forge_arm.elf`.
Older build manifests are historical; the root `build/dm_mc02/forge_arm.elf`
is overwritten by builds. Reference recording and UI changes need no new flash.

## Resume manual controls

With all links mechanically supported throughout movement and torque disable,
and the previous controller owner closed:

```bash
cd /home/forge2/Documents/Codex/2026-11-12/pu/work/FORGE
python3 scripts/jog_dm_arm.py --execute --supported \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 --manual-travel --speed-deg-s 5 \
  --report "build/arm_tests/manual-keyboard-$(date +%Y%m%d-%H%M%S).json"
```

This command uses the original release-disable policy. Select 1–6, press E,
then hold an arrow. Release stops; Space/focus loss disarm.
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
