# Damiao torque ratings and rig trips

Manufacturer/incident review: 2026-11-10; expanded gun-load profile added 2026-11-19. This separates manufacturer ratings, FORGE configuration
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

## Gun-load recording profile (2026-11-19)

The operator confirmed the approximately 1.96 kg gun was attached during the
12.0137 Nm A2 trip despite the journal's `no-ee/` directory. The A2-only 30 Nm
profile was subsequently deployed, with physical TMAX=40 persistence verified
in `readback-20261119-175312.json`. After A5 later faulted at 3.5140 Nm against
3.5 Nm, the operator confirmed **A3 matches A2; A4, A5 and A6 use 10 Nm**.

The explicit `arm_rig_gun_a2_a6_load.json` requests A2/A3 **30 Nm**, A4-A6
**10 Nm** and unchanged A1 **20 Nm**. These are commissioning thresholds,
not manufacturer-approved continuous ratings. No overload-duration cutoff is
added, as requested. Temperature logging does not establish permissible peak
duration or duty cycle; these are not specified in the reviewed manuals.
A2/A3 remain rated 12 Nm, A4-A6 3.5 Nm. The estimated payload mass is not a
measured torque calculation; required torque changes with pose and acceleration.

MC02 capability `0x800000` extends the exact known A3-A6 routes; A2 retains
`0x200000`. Both host and firmware apply the configured absolute torque trip.
The original profiles remain unchanged. The new profile requires actual drive
TMAX **40 on A2/A3** and **12.5 on A4-A6**. Their prior ranges, 28/10, cannot
observe a crossing above the requested 30/10 Nm trips. Changing TMAX changes
feedback encoding; changing only host JSON mis-scales it. Host readback and MCU
probing reject mismatches and unobservable extended trips before enable.

The disabled `scripts/setup_arm_load_ranges.py` requires capability `0x1000000`
and the complete freshly disabled/stationary rig. It skips ranges already at
target, writes only fixed TMAX values, independently reads back before one save,
and compares all captured configuration exactly except live VBus register 60.
Original values are durably recorded before any write. No current/gain/thermal,
mode, encoder-zero or ID setting is changed. The drive saves all its parameters
together, so unexpected unrelated changes block progress. An uncertain save is
never retried; global motion lock persists until MCU restart. A3-A6 setup/readback is now physically recorded in the operator's Nov 19
reports: the first setup writes/saves all four, a subsequent fresh-MCU check
finds retained values, and full-profile disabled readback passes. Firmware
f78f6b0d was deployed by the operator. See the
[physical review](measurements/arm-gun-load-review-2026-11-19.json); the agent
performed offline analysis only.

Normal cached feedback records signed estimated torque, MOS/drive temperature
and motor `rotor_c` temperature throughout preparation, holding and welding.
No second owner or enabled register polling is added. Both **80 C** temperature
trips, speed/tracking/freshness/lease checks and latched whole-arm fault disable
remain. Follow the [operator commands](ARM_RIG_TEST.md#expanded-gun-load-profile-a2a3-30-nm-a4-a6-10-nm).

## Standard profile and command path

The current supported rig includes all six axes. Manual mode-2 non-holding/
teaching uses 5 deg/s; holding-mode jogs default to 10 deg/s. The separate
[weld controller](WELD_MOTION.md) uses 5 deg/s initial/approach and equal
68.7549 deg/s weld caps. All measured-speed trips remain **1.5 rad/s**;
torque trips are **20 Nm A1, 12 Nm A2/A3, 3.5 Nm A4-A6**. These are reactive
feedback stops, not drive-current clamps or physical operating qualifications.
The candidate pose mapping and derived standby still need independent validation.

The mode-2 CAN payload contains position and maximum speed float32 values at
CAN ID + 0x100. MIT kp/kd/torque arguments are not transmitted. Drive loops
determine torque/current; `Feedback Nm` is a motor estimate decoded with the
verified TMAX, not independently measured shaft torque or PSU current.
See the pinned 4340P manual pp. 12/16 for the mode-2 command/feedback format.

Standard torque configuration ceilings are 20/12/3.5 Nm
for 8009P/4340P/4310P, with the explicit A2 trial exception above. Profiles can request less. Capability 0x2000 gates
4340P trips above the old 3 Nm ceiling. The Nov 12 saved deployed build
**d61f6d16 / 0xFFF8F** retains disabled control reads (0x4000) and manual travel
(0x8000), with additional explicit holding and optional jog-smoothing capabilities.
The separate legacy CAN2/ID1 bench retains its 3 Nm trip and is not the rig path.

| Layer | Current behavior / remaining bound |
| --- | --- |
| Keyboard/Pocket | Fresh held intent; explicit stop policies; non-holding/Pocket 5 deg/s, opt-in keyboard hold defaults to 10 deg/s |
| Position port | Rolling manual target and current-position preload; 12 s host cutoff in default bench mode, command leases retained in explicit hold mode |
| Host feedback | Profile speed/torque trips, 80 ms freshness and temperature checks |
| USB/MCU mode 2 | Position/speed payload; no torque or motor-current command |
| Routed MCU | Protocol range, preload and 3.2 rad target-distance guard; default bench 15 s cutoff, explicit holding without duration cutoff; 150 ms command lease, 100 ms feedback timeout and speed/torque/temperature trips remain |
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

VBus (register 60 / 0x3C) is live read-only bus voltage, not a saved setting.
The A2 range setup records it but excludes it from configuration equality
before and after parameter save; other captured settings remain checked exactly.

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
