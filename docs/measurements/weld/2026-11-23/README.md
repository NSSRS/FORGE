# Offline control-development handoff — November 23, 2026

Historical baseline: the operator subsequently selected [v7 on Nov 24](../2026-11-24/README.md).
The findings and verification below describe the Nov 23 state; current operation
and remaining work are in [the session handoff](../../../ARM_SESSION_HANDOFF.md).

This archive is the operator-requested copy of session
`20261123T114316.007261Z-cnyzuo1w`, with the **1.96 kg end effector** installed.
It contains the complete original [JSONL journal](session.jsonl), final
[session report](session.json), [load context](session.load-context.json),
[mapping](mapping.json), [standby](standby.json), [arm profile](arm-profile.json)
and extracted [accepted plan](accepted-plan.json). Raw copies are unchanged;
[manifest.json](manifest.json) records their SHA-256 hashes and provenance.
The journal embeds calibration, settings, cached feedback and dispatched targets.
Local absolute paths inside original reports are provenance, not dependencies.
All commands below run without motors or a transport owner.

## Accepted scope and remaining issues

The operator accepted the **8.22 s preparation time** and retired the earlier
five-second goal. HOLD-01, PLAN-01 and BASE-04 are closed by operator confirmation.
Do not reopen them from historical failure records. Current work is ACCURACY-01
and SPEED-01, prioritizing **end-effector orientation relative to the planned
tool frame: 0.5 deg target, 1 deg maximum during welding**. Individual joint
errors are diagnostic, not per-axis acceptance limits. Position error is reported
separately in mm. Compare at the recorded time without shifting phase or wrapping
angles. Full orientation error is the smallest 3D rotation between the planned
and reconstructed `wire_tip` frames, including roll. This is an accuracy criterion;
the 3-degree runtime tracking trip and other motor protections are unchanged.

**Current end-effector result: maximum NOT MET.** Encoder reconstruction gives
**1.408 deg RMS / 2.633 deg peak**, with **21.8%** of ticks within the 0.5-degree
target and **40.1%** within the 1-degree maximum. This remains model-based until
independently measured on the physical tool. The target and maximum are reported
separately; `passed` uses the maximum, while `orientation_target_met` and sample
percentages show progress toward the preferred target.

Joint diagnostics explain which motions need investigation:

| Axis | Nominal tracking RMS (deg) | Peak (deg) |
| --- | ---: | ---: |
| A1 | 1.138 | 2.098 |
| A2 | 0.030 | 0.043 |
| A3 | 0.061 | 0.107 |
| A4 | 0.076 | 0.193 |
| A5 | 0.040 | 0.078 |
| A6 | 0.379 | 0.750 |

The [end-effector report](accuracy-end-effector.json) records the orientation result.
The original [unthresholded report](accuracy-summary.json),
[hold summary](hold-summary.json) and full
[accuracy/speed review](../../arm-accuracy-speed-review-2026-11-23.json) are retained.

1. **A1/A6 overswing and phase lag:** planned/measured peak-to-peak travel is
   2.261/3.891 deg and 0.428/1.333 deg respectively. Actual travel is about
   1.72 and 3.12 times planned. Approximate fundamental phase lag is 101/112 ms.
   Measured frequency is 1.734/1.728 Hz versus the checked 1.730 Hz, so matching
   frequency alone does not establish tracking accuracy. The original request
   was 2 Hz; the plan was retimed before execution.
2. **Investigate host lead and command shaping:** dispatched travel was
   7.693/2.203 deg on A1/A6. Inspect requested, dispatched and measured positions
   together before attributing the discrepancy to native drive gains. The
   current tracker freezes preparation-adapted lead during welding and enables
   host correction on A1/A6. Preserve the accepted tiny A2-A5 nominal paths.
3. **Tool pose is not independently measured:** encoder reconstruction with
   the matching fixed-base model gives 10.059 mm RMS / 18.575 mm peak error,
   predominantly transverse. The orientation criterion is not a tip-mm tolerance.
   Independent synchronized workpiece-relative tool-pose measurements and a tip tolerance
   remain missing. The chassis output was off in this session.
4. **Accuracy-analyzer pause reporting:** the existing analyzer treats any pause
   as an evidence issue and exits 2. The journal shows intentional
   `operator_input` during continuous welding, no fault, complete coverage and
   verified final disable/mode restoration. Preserve manual-pause semantics;
   do not add `--single-pass` merely to obtain a successful analyzer exit.
5. **Known offline regression:**
   `test_tiny_joint_corrections_cannot_throttle_a6_clearance_projection` also
   failed on the pre-change `2dd6caa` sources: the masked target had 1.538 mm
   floor clearance against the unchanged 1.8 mm requirement. Keep this visible
   when working on tracking; do not lower clearance to make the test pass.

The baseline contains 202 usable weld ticks over 5.951 s (about ten cycles),
maximum tick gap 63.7 ms, no rejected samples and no dropped records. Ready held
fixed targets for 25.19 s. These records are enough to reproduce the current
failure without another motor run.

Publication checks passed 257/258 weld tests, all 191 motor/cable/calibration/
keyboard/GUI tests, and all ten accuracy-analyzer tests. The sole weld failure
is the pre-existing clearance-projection regression listed above. These are
offline checks, not new motor acceptance. The offline CLI audit passed. A clean
source export, without local build files or model configuration, reproduced
the archived orientation, position, joint, coverage and verdict fields exactly;
the hold analyzer also completed there. All archived source bytes and SHA-256
hashes were verified.

## Reproduce offline

From the repository root with the simulation/scientific Python environment
installed (see [test setup](../../../../tests/README.md)):

```bash
.venv/bin/python scripts/analyze_weld_accuracy.py \
  docs/measurements/weld/2026-11-23/session.jsonl \
  --orientation-target-deg 0.5 --max-orientation-deg 1.0

.venv/bin/python scripts/analyze_weld_hold.py \
  docs/measurements/weld/2026-11-23/session.jsonl \
  --label with-ee --ee-mass-kg 1.96

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl \
  PYTHONPATH=tests .venv/bin/python -m unittest \
  test_weld_accuracy test_weld_tracking test_weld_preparation -v
```

The first command should report **FAIL / exit 2** because tool orientation exceeds 1 deg;
it also prints the known manual-pause evidence issue. It needs no output path.
Use the archived plan rather than recalculating a different route for baseline
comparisons. The model and mapping fingerprints must match the report to
reproduce the encoder-tip estimate.

## Control-code entry points

- [PositionTracker](../../../../src/controls/controls/controller/tracking.py):
  lead/PID, filtered velocity, anti-windup and command shaping.
- [WeldController](../../../../src/controls/controls/hardware/weld_control.py):
  nominal clock, stage boundaries, correction mask, clearance projection and
  unchanged tracking guard.
- [MotionFrequency](../../../../src/controls/controls/controller/motion_feedback.py):
  cached encoder autocorrelation; a frequency estimate is not an amplitude score.
- [Accuracy analyzer](../../../../src/controls/controls/hardware/weld_accuracy.py):
  same-tick joint diagnostics and model-reconstructed tool orientation/position error.
- [Tracking regressions](../../../../tests/test_weld_tracking.py): synthetic
  delayed/unequal response, recorded-motion regressions and full-model guards.

First reproduce the baseline, then evaluate bounded changes to A1/A6 lead and
shaping in the existing synthetic-response tests. Replaying recorded feedback
can compare candidate command calculations but cannot prove the physical
response to a changed command. Keep the current thresholds, cable limits,
collision checks, continuous-until-manual-pause behavior and single-owner policy.
Future physical acceptance must measure the changed controller against the
0.5-degree orientation target / 1-degree maximum; no physical improvement is
claimed by this archive.
