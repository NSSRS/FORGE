# Controls open work

The canonical acceptance criteria are in [ARM_LAYER_TODOS.md](../../docs/ARM_LAYER_TODOS.md).
Current implementation and reproduction: [WELD_MOTION.md](../../docs/WELD_MOTION.md).

- **HOLD-01 (November 17):** qualify post-prep joint stiffness with the installed
  supported load. Verify fixed targets/enable/allowances, record six-axis drift
  for >=10 s, and tune native drive loops from measured readbacks if needed.
  Fixed endpoint hold and cached error monitoring are implemented; physical
  stiffness and the reported sag cause remain unqualified. Follow the canonical
  HOLD-01 acceptance criteria; no guessed gains or weaker trips.
- **WEAVE-01:** after stable holding, qualify recovery -> approach -> weld -> hold,
  both output switches and physical tracking/timing. Target remains 2 Hz;
  historical provisional-model CMA-ES reached 1.815288 Hz. The explicit offline
  staging comparison reaches ready in 4.62 s with ideal fake shafts; qualify the
  five-second preparation goal with actual position-loop response before promotion.
- **CAL-01 / PLAN-01:** independently qualify the candidate sign/offset mapping,
  derived standby and complete path against the assembly and stopping margins.
  Qualify supported startup, current-position hold pickup and recovery waypoints;
  resolve the Nov 16 pose/model mismatch and audit remaining collision exclusions.
- **PLAN-01 / SUPPORT-01:** complete the requested automatic recovery from a
  settled, gravity-hanging arm on a secured base. Qualify taking the actual load
  at measured angles, then lifting into compact standby without manual joint
  placement. Model/check the passive return and swing after torque loss as well
  as the powered route; rate-limited fake shafts provide no gravity/load evidence.
- **LOAD-01, after the first arm weld:** identify mass/COM/inertia, reconcile
  model acceleration bounds and add qualified weight/gravity compensation.
- **FEEDBACK-01, after the first arm weld:** baseline tracking logs, tool/seam
  sensing, bounded outer feedback and a staged observation-only ESO evaluation.
  Mode-2 input/delay identification is required before applying compensation.

Keep `hardware/` beside `simulation/`; shared planning belongs in `controller/`.
Independent arm/chassis switches only gate outputs, with no switch compensation.
