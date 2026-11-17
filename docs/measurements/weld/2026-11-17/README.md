# Saved welding reports — November 17, 2026

The five newest completed physical-session JSON reports from `build/weld/`, copied unchanged at
the operator’s request before shutdown. Selection uses source modification time before the final offline audit.
These are physical session records, not qualification certificates. JSONL journals
and other generated files remain local; the JSON reports retain their source paths.

| Report | Created (UTC) | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| [weld-1794913974470083980.json](weld-1794913974470083980.json) | 2026-11-17T11:20:23.852762+00:00 | 11634886 | `1c025e2b235194e9dd063b5533e9ea9a3db005b1e92e1e22ef5db4bee89474a2` |
| [weld-1794912939805911647.json](weld-1794912939805911647.json) | 2026-11-17T11:12:52.558225+00:00 | 13773579 | `d6184de5468726dee35646b8ed1d6ce2a811d13b2cfc932056631e598a318c58` |
| [weld-1794911796228349602.json](weld-1794911796228349602.json) | 2026-11-17T10:40:55.625840+00:00 | 15356502 | `c3816de6c4d6a571336f0f5187d318f6b4265d1cd65c00316c592aa800e5966b` |
| [weld-1794911117564541682.json](weld-1794911117564541682.json) | 2026-11-17T10:27:00.060552+00:00 | 14079727 | `f3eb6dde31886ba4727ab2b972997759bc8368fce950371031b12aa60e6b4c15` |
| [weld-1794911106948300856.json](weld-1794911106948300856.json) | 2026-11-17T10:25:11.983209+00:00 | 7427 | `f37d7e571f9e793fd487687a71eec0c8dffab50b7185526ce4b1d0a6a7624654` |

The newest report (`weld-1794913974470083980.json`, tracker version 3) reaches
ready and starts continuous welding. Its final cleanup reports missing feedback
on all six joints, with `disable_acknowledged=false` and
`mode_restoration_verified=false`; do not count it as a verified shutdown.
See [the current handoff](../../../ARM_SESSION_HANDOFF.md) for interpretation
and [open acceptance work](../../../ARM_LAYER_TODOS.md).

## Latest physical motion result

Tracker version 3 reaches ready in 10.540 s and records 15.580 s of continuous
welding before operator-intent interruption. Replaying the report's cached
feedback through `MotionFrequency` gives the final readout below (eight-second
window, updated every 0.5 s; no extra hardware reads). Planned travel is the exact
cubic path excursion, rounded for this table.

| Axis | Measured pattern Hz | Measured travel (deg peak-to-peak) | Planned travel (deg peak-to-peak) |
| --- | ---: | ---: | ---: |
| A1 | 1.730 | 3.694 | 2.260 |
| A2 | insufficient motion | 0.000 | 0.063 |
| A3 | insufficient motion | 0.022 | 0.132 |
| A4 | insufficient motion | 0.022 | 0.098 |
| A5 | insufficient motion | 0.000 | 0.072 |
| A6 | 1.731 | 0.634 | 0.427 |

The plan's weave rate is 1.731 Hz (requested 2 Hz). All 739 welding samples admit
100% of the proposed host correction; correction axes are A1/A6. Their measured
frequency matches the weave in this window, but measured travel exceeds the
plan. This does not qualify amplitude, phase, loaded holding or shutdown. The
operator accepts the tiny A2–A5 nominal motions; their trajectories are retained.
