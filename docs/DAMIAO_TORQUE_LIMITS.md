# Damiao torque ratings and the A2 rig trips

Reviewed 2026-11-10. This separates manufacturer ratings, FORGE configuration
ceilings, per-test trip thresholds and motor-side current protection.

## Manufacturer manuals

The cached manufacturer PDFs were re-read, and each specification table on
printed/PDF page 7 was rendered and visually checked. Exact versions and pinned
manufacturer URLs are in [DAMIAO_ARM.md](DAMIAO_ARM.md#verified-manufacturer-references).
Local reference copies are filed under `build/reference/damiao/`:

| Joints | Model / manual | Rated torque | Peak torque | Rated phase current |
| --- | --- | ---: | ---: | ---: |
| A1 | J8009P-2EC V1.1 / manual V1.0, 2026-09-10 (`8009P.pdf`) | 20 Nm | 40 Nm | 20 A |
| A2, A3 | J4340P-2EC V1.1 / manual V1.4, 2026-09-14 (`4340P.pdf`) | 12 Nm | 40 Nm | 4.11 A |
| A4, A5, A6 | J4310P-2EC / manual V1.3, 2026-09-14 (`4310P.pdf`) | 3.5 Nm | 12.5 Nm | 4.9 A at 24 V; 4.8 A at 48 V |

These are output-shaft torque specifications. The 4340P V1.1 table supersedes
older 9/27 Nm figures. The 4310P hardware V1.1 label has not been independently
verified; there is no separately titled V1.1 manual in the recorded references.
Phase current is not PSU/supply current. Rated torque does not by itself establish
safe continuous stationary holding in this installation. Peak torque is not a
permitted sustained jog setting or a measured stall torque for this rig.

Page 8 describes the configurable current/overcurrent percentage and its 0.8
default / <=0.98 recommendation. The parameter-help text also uses 'rated current'
where page 8 describes a percentage of the measurable maximum. Do not infer a
safe numerical current clamp from those descriptions without unit-specific
readback and verification. TMAX is protocol scaling, not the allowed torque.
The 4340P/4310P register tables list Imax (0x3B), firmware version, KT_Value,
GREF and OC_Value. Disabled read-only support is implemented; application
protection and unit/scaling verification remain PROTECT-01 work.
No current, temperature, persistent zero or calibration register was changed.

## Current profile and command path

The operator confirmed A2–A6 manual motion on 2026-11-10. The current rig
commands **5 deg/s** in position–velocity mode 2, with explicit manual travel
instead of the former +/-3-degree window. All five active measured-speed trips
are **1.5 rad/s**; torque trips are **12 Nm for A2/A3**, **3.5 Nm for A4–A6**.
A1 is omitted. These trips are reactive feedback stops, not drive-current clamps.
Faster commanded speed and physical position initialization are the next work.

The mode-2 CAN payload contains position and maximum speed float32 values at
CAN ID + 0x100. MIT kp/kd/torque arguments are not transmitted. Drive loops
determine torque/current; `Feedback Nm` is a motor estimate decoded with the
verified TMAX, not independently measured shaft torque or PSU current.
See the pinned 4340P manual pp. 12/16 for the mode-2 command/feedback format.

Python/routed firmware permit torque configuration ceilings of 20/12/3.5 Nm
for 8009P/4340P/4310P. Profiles can request less. Capability 0x2000 gates
4340P trips above the old 3 Nm ceiling. Deployed build **156f20c8 / 0xFF8F**
also supports disabled control reads (0x4000) and manual travel (0x8000).
The separate legacy CAN2/ID1 bench retains its 3 Nm trip and is not the rig path.

| Layer | Current behavior / remaining bound |
| --- | --- |
| Keyboard/Pocket | Fresh held intent; release/focus/neutral/selection stop policies; supported command <=5 deg/s |
| Position port | Rolling manual target; current-position preload; 12 s host hold cutoff |
| Host feedback | Profile speed/torque trips, 80 ms freshness and temperature checks |
| USB/MCU mode 2 | Position/speed payload; no torque or motor-current command |
| Routed MCU | Protocol position range, near-position preload and 3.2 rad target-distance guard; 15 s enable cutoff, 150 ms command lease, 100 ms feedback timeout, speed/torque/temperature trips |
| Board transport | CAN completion/abort, bus-off, USB loss and hardware watchdog stops |
| Motor drive | Existing current thresholds, ACC/DEC/MAX_SPD and loop gains; reads below, no gain/current writes |

Mode setup writes volatile TIMEOUT=2000 and CTRL_MODE=2, with original mode
restored on cleanup. No OC_Value, Imax, gain or persistent-zero change was made.
Saved production/protection confirmation flags remain unchanged.

## Disabled control readback — 2026-11-10 19:56

The operator's `build/arm_tests/recovery-control-20261110-195636.json`
completed A2–A6 control-register inspection. Selected decoded values:

| Register | A2/A3 (4340P) | A4/A5/A6 (4310P) |
| --- | --- | --- |
| KP_ASR | approximately 0.00384 | approximately 0.00372 |
| KI_ASR | 0.002 | 0.002 |
| KP_APR | 54 | 54 |
| ACC / DEC | 2 / -2 | 2 / -2 |
| Deta / V_BW | 4 / 40 | 4 / 40 |
| OC_Value | 0.8 | 0.8 |
| Imax | approximately 20.5224 | approximately 20.5224 |
| KT_Value | 0 | 0 |
| VBus | approximately 24 V | approximately 24 V |

These are register values, not verified engineering-unit current limits.
In particular, a zero KT_Value must not be used to infer zero delivered torque.
A6's relevant settings match A4/A5; the physical source of earlier speed peaks
still needs measurement. The full local report retains raw bits, register
addresses, types and per-read status. See
[disabled inspection procedure](ARM_RIG_TEST.md#disabled-currentloop-parameter-inspection-2026-11-10).

## Earlier faults and current-draw findings

The operator observed roughly 0.234 to 0.235 A at the 24 V PSU during attempted
motion. The code audit found no hidden 2 Nm clamp in the active path. The earlier
bounded A2 run (`current-debug-20261110-190750.json`) recorded 11 normal target
completions and peak reported torque 2.468376 Nm; arrival at the small target
caused disable. Manual travel subsequently removed that completion boundary.
Basic movement now works, but phase current/delivered torque and the unchanged
PSU observation remain unqualified under PROTECT-01.

Earlier A2 faults reached 3.001709 Nm against the old 3 Nm trip; a later A3
fault reached 12.0821 Nm against 12 Nm. Earlier A6 faults reached approximately
0.300365/0.315017 rad/s against the old 0.3 limit while commanding 5 deg/s.
These were reactive stops; repeated A6 values follow its 12-bit speed bins.
The operator requested the current common 1.5 rad/s trip. This does not resolve
tracking/overshoot/noise or establish faster commanded-motion performance.

The former A4 disabled-preload rejection was consistent with drift outside its
startup corridor; the old report lacked the exact origin, so that cause was
inferred. Current manual travel uses current-position preload and retains raw
protocol guards. Nonselected joints remain disabled and require support.

Reports now retain session origins/bounds, startup registers, enabled samples
and sampled peaks before cleanup replaces feedback. `motion_samples` is a capped
10 Hz trace (3000 rows, oldest removed in blocks of 100 with a count); summary
peaks use host samples, not every CAN frame. `Feedback Nm` and `Trip Nm` are
separate in the GUI; stale dynamics are blanked. No extra telemetry owner or
enabled CAN register polling was added.

## Ownership and evidence

- Host limits/compatibility: `src/motors/arm.py`; lifecycle/readback/recovery:
  `arm_bench.py`; motion samples/manual targets: `arm_position.py`.
- Firmware: `src/damiao_firmware/src/arm.c`, `include/arm.h`; rig settings:
  `src/robot_description/config/arm_rig_a2_a6.json`.
- Tests: `tests/test_damiao_bench.py`, keyboard/Pocket suites and native CTest;
  reproduction commands are in [tests/README.md](../tests/README.md).
- Ignored local evidence: `build/reference/damiao/`, `build/arm_tests/` and
  `logs/arm_tests/2026-11-10-*`. The original second A2 report was overwritten
  by a later launch; the torque-review folder preserves an extracted observation
  with provenance, not a claimed byte-identical original.

Current status and next work: [session handoff](ARM_SESSION_HANDOFF.md),
[open registry](ARM_LAYER_TODOS.md), [protection checklist](MOTOR_PROTECTION_TODO.md).
