# Controls open work

The canonical acceptance criteria are in [ARM_LAYER_TODOS.md](../../docs/ARM_LAYER_TODOS.md).
Current implementation and reproduction: [WELD_MOTION.md](../../docs/WELD_MOTION.md).

- **GUI-01 — operator review:** the dashboard implementation is complete;
  review the telemetry, targets/response and expandable diagnostics on the
  actual NUC screen. Offline layout and cached-data regression checks do not
  replace operator acceptance. See the
  [remaining checklist](../../docs/ARM_LAYER_TODOS.md#gui-01--operator-review-of-the-telemetry-dashboard).
- **HOLD-01:** diagnose the Nov 19 untouched A5 fixed-hold speed fault; the
  successful second attempt does not resolve it. Qualify 15-second Ready and
  post-weld holds, stiffness and temperatures with the installed load. Complete
  matched no-EE / approximately 1.96 kg EE trials with identical path/settings,
  recording actual speed, time, torque, temperatures and fixed-target error.
  Qualify the separate A3 endpoint retreat and keep its timing separate. Use
  measured disabled readbacks before proposing drive tuning; no guessed gains.
  The expanded trips remain trial settings under PROTECT-01, not continuous
  torque ratings. See the [physical review](../../docs/measurements/arm-gun-load-review-2026-11-19.json)
  and [matched-run procedure](../../docs/ARM_RIG_TEST.md#matched-runs-with-and-without-the-end-effector).
- **WEAVE-01:** qualify repeatable recovery -> raised approach/lowering -> weld ->
  hold with both output switches and physical clearance. The 2.3 s final-lowering
  trial reached Ready, but latest total preparation is 12.53 s versus the current
  five-second goal. Qualify response and the supported starting region without
  widening guards. Track infeasible starts explicitly; faster calculation does
  not make a long physical move meet the deadline. See the
  [offline planner benchmark](../../docs/PLANNER_BENCHMARK.md).
- **PLAN-01:** reduce remaining worst-case joint-order recovery/staging latency
  while retaining checked candidates and explicit failure. Extend random-start
  coverage and qualify the starting region that meets the five-second goal;
  the broad benchmark does not establish that deadline for arbitrary poses.
- **SPEED-01:** qualify PID/velocity lead under unequal load, reversals and
  saturation using actual timing, error, phase, measured Hz/travel and thermal
  evidence. Nov 19 A1/A6 frequency matches the ~1.73 Hz plan but travel exceeds
  it (3.869/2.260 and 1.180/0.427 deg). Retain the accepted tiny A2-A5 nominal
  paths; synthetic response and geometry tests do not establish physical tuning.
- **CAL-01 / PLAN-01:** independently qualify the candidate sign/offset mapping,
  derived standby and complete path against the assembly and stopping margins.
  Qualify supported startup, current-position hold pickup and recovery waypoints;
  resolve the Nov 16 pose/model mismatch and audit remaining collision exclusions.
- **PLAN-01:** add a visible six-axis encoder-drift check after path calculation
  and before recovery, including a final check on Start recovery. Compare with
  the planning-start pose; block the old plan and require recalculation if any
  joint exceeds the threshold. **2 degrees or 5 degrees: final value undecided.**
  See the canonical PLAN-01 entry for acceptance behavior.
- **PLAN-01 / SUPPORT-01:** complete the requested automatic recovery from a
  settled, gravity-hanging arm on a secured base. Qualify taking the actual load
  at measured angles, then lifting into compact standby without manual joint
  placement. Model/check the passive return and swing after torque loss as well
  as the powered route; rate-limited fake shafts provide no gravity/load evidence.
- **LOAD-01, after the first arm weld:** identify mass/COM/inertia, reconcile
  model acceleration bounds and add qualified weight/gravity compensation.
- **FEEDBACK-01, after the first arm weld:** baseline tracking logs, tool/seam
  sensing, bounded outer feedback and a staged observation-only ESO evaluation.
  Qualify mode-2 input/delay response and the new bounded host tracker; it does
  not identify loaded dynamics or implement gravity/Cartesian compensation.

Keep `hardware/` beside `simulation/`; shared planning belongs in `controller/`.
Independent arm/chassis switches only gate outputs, with no switch compensation.
