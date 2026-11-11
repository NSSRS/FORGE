# Physical arm motor IDs and connection order

The user confirmed this physical mapping on 2026-11-05. IDs below have been written and read back on all six units.
A1 persistence was verified after power cycle; A2..A6 persistence remains pending.
Command IDs are decimal; feedback IDs show hexadecimal and decimal.

| DOF | Physical motor | Command ID | Feedback ID | Unloaded motion evidence |
| --- | --- | --- | --- | --- |
| 1 | Previously tested 8009P | 1 | 0x11 (17) | Passed outward/return |
| 2 | Previously tested 4340P | 2 | 0x12 (18) | Passed in 14-stage six-motor report |
| 3 | Additional 4340P | 3 | 0x13 (19) | Passed in 14-stage six-motor report |
| 4 | Previously tested 4310P | 4 | 0x14 (20) | Passed outward/return |
| 5 | First additional 4310P, label DOF5 | 5 | 0x15 (21) | Passed in 14-stage six-motor report |
| 6 | Second additional 4310P, label DOF6 | 6 | 0x16 (22) | Passed in 14-stage six-motor report |

All six passed the saved group/individual unloaded sequence. Later dated notes
below record the commissioning history, not remaining motion-test requirements.
See [current status](DAMIAO_ARM.md#current-status--2026-11-05).

## Physical connection sequence (initial ID assignment)

Change wiring with motor power OFF. Keep DOF1 alone on CAN1. For initial
assignment, connect only one other motor to CAN2, in order DOF2, DOF3, DOF4,
DOF5, DOF6. Label the two new 4310Ps before swapping them. Do not daisy-chain
until all participating units have distinct IDs verified after power cycling.
Current/final bus layouts remain as in arm_motors.json/arm_motors_final.json.
IDs stay attached to physical DOFs when the 4310Ps later move to CAN3.

The new MC02 ID-commissioning extension uses capability 0x400 (HELLO mask
0x78f), USB type 20 and `scripts/set_dm_motor_ids.py`. Rebuild/flash before use.
The operator must confirm which physical unit is isolated. Only one slot is
configured. The ID session requires fresh stationary disabled feedback and
locks all motion/configuration/mode writes until MCU reset. Only feedback IDs
17..22, command IDs 1..6 and an explicit parameter save are accepted. Save
requires matching command/feedback readbacks and a pair of command ID N and
feedback ID 16+N. No enable, zeroing, gain, mode or timeout writes are sent.
An ID write can succeed even if its ACK is lost; inspect the report and discover
addresses before any retry. Never blindly repeat a parameter save.

Example for DOF1 alone on CAN1 after motor power cycle and fresh MCU reset:

```bash
python3 scripts/set_dm_motor_ids.py --dof 1 --execute --isolated \
  --fresh-power-cycle --report build/arm_tests/dof1-id-assignment.json
```

Then turn motor power off, reset MC02, turn motor power on, and verify:

```bash
python3 scripts/set_dm_motor_ids.py --dof 1 --verify --isolated \
  --fresh-power-cycle --report build/arm_tests/dof1-id-verification.json
```

Use `--dof 2` through `--dof 6` for subsequent isolated CAN2 units. `--bus 3`
is available if a non-DOF1 unit is physically isolated there. Defaults for an
unassigned motor are command ID 1 and feedback ID 0; use discovered
`--current-id`/`--current-feedback` if different. Running without an action
flag prints the planned IDs offline without opening a port. Verification is
read-only and requires a real motor power cycle; a fresh process alone does
not prove persistence. Update inventory and bench plans only from successful
hardware assignment/verification reports.

The P-series manuals define MST_ID=register 7, ESC_ID=register 8. Writes take
effect immediately, but persistent assignment also requires Save Parameters
while disabled, followed by power-cycle/readback verification. Save Parameters
stores ALL current motor parameters, not only IDs; perform assignment after a
fresh motor power cycle, before any bench mode/timeout changes in that session.
Do not claim persistence from a write ACK alone.

All six IDs and unloaded tests are recorded in the table above. Physical
signs, offsets, travel and loaded limits still require measurement. For the
next fixture session, see the [pose test](ARM_POSE_TEST.md).

## Historical observation before DOF1 persistence was verified

DOF1 accepted MST_ID=17 while ESC_ID remained 1; both registers read back
correctly before saving, and Save Parameters was acknowledged. Subsequent
register reads timed out, including after the operator-confirmed motor power
cycle and MCU reset. A read-only CAN1 scan of command IDs 0..255 returned no
replies. DOF2 still responds on CAN2 at command ID 1 / feedback ID 0 and has
not been modified. DOF1 is reported powered with a red indicator on CAN1.
Persistence is not verified; do not blindly repeat the save or run old-ID
motion plans. Reports are under build/arm_tests/dof1-id-*.json and
build/arm_tests/dof1-post-cycle-address-scan.json.

Subsequent disabled-only check on the unchanged CAN1 wiring received a valid
DOF1 status reply at feedback ID 17 to command ID 1 after the power cycle
(MOS 29C, rotor 28C). This confirms address persistence and CAN connectivity;
register-read verification remains unresolved. Evidence:
`build/arm_tests/dof1-disabled-route-check.json`.

The diagnostic firmware adds capability 0x800 (total 0xF8F) and passive USB
command 21: per-bus receive count, last frame timestamp, identifier, length,
and bytes before register-payload matching. `scripts/diagnose_dm_can.py`
issues only register reads and records the receive evidence. It does not
configure, enable, disable or write motor parameters. The save path now
suppresses MCU CAN traffic for 50 ms while an already-disabled motor writes
flash. Previously it scheduled disable traffic immediately after save; this
is a corrected timing defect, not a proven explanation for the read failures.

### Follow-up: delayed DOF1 register replies identified

Raw capture showed DOF1 replying two requests behind, rather than being absent
from CAN1. Repeating the same read three times retrieves the matching register.
Unconfigured read-only discovery now retries up to three times on timeout;
configured-motor reads and all writes keep their existing single-attempt rules.
The underlying reason for the queued replies is not established. This is not
validation of motion-feedback latency at the new IDs.

DOF1 power-cycle verification now passes: command 1, feedback 17, mode 1,
timeout 0, ranges 12.5/45/54 and damping 4 unchanged. DOF2 was then assigned
command 2, feedback 18; save ACK and all post-save register readbacks passed,
with mode 1, timeout 0, ranges 12.5/10/28 and damping 4 unchanged. DOF2 physical
power-cycle verification remains pending. No motor was enabled in these steps.
Reports: `build/arm_tests/dof1-id-verification.json`,
`dof1-raw-register-diagnostic.json`, `dof1-repeated-read-diagnostic.json`,
`dof2-raw-register-diagnostic.json`, and `dof2-id-assignment.json`.

Operator preference: avoid routine motor-power cycling for disabled register
checks or MCU resets. Power-cycle verification is a separate persistence check
and can be deferred until the next physical power cycle.

A3/DOF3 assignment completed: command ID 3, feedback ID 19 (0x13). Save
acknowledgement, post-save register readback and disabled feedback passed.
Mode 1, timeout 0, ranges 12.5/10/28 and damping 4 are unchanged. No movement
was commanded; power-cycle persistence and individual motion remain pending.
Evidence: `build/arm_tests/dof3-id-assignment.json`.

A4/DOF4 assignment completed after the operator turned on its PSU: command
ID 4, feedback ID 20 (0x14). Save acknowledgement, post-save register
readback and disabled feedback passed. Mode 1, timeout 0, ranges 12.5/30/10
and damping 4 are unchanged. No movement was commanded; power-cycle
persistence remains pending. Evidence: `build/arm_tests/dof4-id-assignment.json`.

A5/DOF5 assignment completed: command ID 5, feedback ID 21 (0x15). Save
acknowledgement, post-save register readback and disabled feedback passed.
Mode 1, timeout 0, ranges 12.5/30/10 and damping 4 are unchanged. No movement
was commanded; power-cycle persistence and individual motion remain pending.
Evidence: `build/arm_tests/dof5-id-assignment.json`.

A6/DOF6 assignment completed: command ID 6, feedback ID 22 (0x16). Save
acknowledgement, post-save register readback and disabled feedback passed.
Mode 1, timeout 0, ranges 12.5/30/10 and damping 4 are unchanged. No movement
was commanded; power-cycle persistence and individual motion remain pending.
Evidence: `build/arm_tests/dof6-id-assignment.json`.

All six physical units now have their requested IDs written and save
acknowledged. A1 has passed power-cycle verification. A2 through A6 still
require post-power-cycle readback; no full-arm or simultaneous motion is
validated. A3, A5 and A6 still need their individual unloaded motion tests.

The operator requested the next sequence: read-only six-motor ID verification,
then all six together, then A1..A6 individually. The runner now supports
`--all --group-first` explicitly for this order, without marking untested
units as individually passed. The existing default remains individual-first.
Plan: `build/arm_tests/all-six-45deg-3s.json`; fixture confirmation is pending
completion of the new wiring. No motion has been run with this plan.

The operator requested 3x commanded speed for the six-motor sequence. The new
plan `build/arm_tests/all-six-45deg-1s.json` commands pi/4 rad/s (45 deg/s),
nominally one second each direction plus settling. The earlier 3-second plan
is preserved. 4340P host/MCU feedback-speed envelope increased from 0.5 to
1.5 rad/s for this unloaded test; its 3 Nm torque stop is unchanged. This
is a bench envelope, not a motor rating. Firmware rebuilt, not flashed by
the agent. Software validation: 45 Python tests and 2 native tests passed.
Hardware execution of the new plan is pending.

The operator next requested 180 degrees out and back at the same pi/4 rad/s
(45 deg/s), nominally four seconds per direction plus settling. New plan:
`build/arm_tests/all-six-180deg-4s.json`. Routed bench travel validation now
allows up to 180 degrees; firmware allows a 3.2-rad target difference and a
3.25-rad enabled corridor. Legacy single-4340 bench limits remain unchanged.
The runner still checks start/target bounds with 0.05-rad padding, stationary
start, enabled feedback, speed/torque/temperature and watchdogs. This requires
flashing the rebuilt MCU firmware. Software checks passed: 46 Python and two
native tests, including simulated group-first then individual 180-degree
outward/return for all six motors and rejection of wider enabled bounds.
No hardware motion was commanded during preparation.
