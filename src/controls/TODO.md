# Controls open work

The canonical acceptance criteria are in [ARM_LAYER_TODOS.md](../../docs/ARM_LAYER_TODOS.md).
Current implementation and reproduction: [WELD_MOTION.md](../../docs/WELD_MOTION.md).

- **HOLD-01 (November 17):** qualify post-prep joint stiffness with the installed
  supported load and after welding; tune native drive loops from measured
  readbacks only if needed. The Nov 17 operator journal provides 12.76 s of
  enabled fixed-target hold within 0.177 deg and recovery from operator pushes.
  Fixed endpoint hold, steady stage allowances, one-capture pause and cached
  error monitoring are implemented; physical
  stiffness and the reported sag cause remain unqualified. Follow the canonical
  HOLD-01 acceptance criteria; no guessed gains or weaker trips.
- **WEAVE-01:** after stable holding, qualify recovery -> approach -> weld -> hold,
  both output switches and physical tracking/timing. Target remains 2 Hz;
  historical provisional-model CMA-ES reached 1.815288 Hz. Explicit hardware
  staging is available; the Nov 16 comparison at 10 deg/s reaches ready in
  4.62 s with ideal fake shafts. Qualify the
  five-second preparation goal with actual position-loop response before promotion.
  Corrected reference braking does not qualify the route with unequal-axis lag:
  the direct-route full-model lag replay reaches the gun/floor guard. The explicit
  raised approach preserves the target and existing guards; qualify its physical
  tracking and clearance, then resolve five-second timing without weakening guards.
- **SPEED-01:** qualify the Nov 17 host PID/velocity-lead tracker against the
  previous paced controller using actual timing, per-axis error, measured Hz and
  travel, command timing, temperature and physical jitter. The local lag estimate
  and PID gains are provisional; check unequal load and reversal behavior before
  treating synthetic response/geometry tests as motor qualification. The latest
  tracker-v3 report matches A1/A6 frequency but overshoots planned travel; amplitude
  and phase acceptance remain open. Retain the operator-accepted tiny A2–A5 paths.
  See the [saved reports](../../docs/measurements/weld/2026-11-17/README.md).
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
