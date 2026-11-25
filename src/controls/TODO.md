# Controls open work

The canonical acceptance criteria are in [ARM_LAYER_TODOS.md](../../docs/ARM_LAYER_TODOS.md).
Current implementation and reproduction: [WELD_MOTION.md](../../docs/WELD_MOTION.md).

- **ACCURACY-01, physical test when available:** run the existing v7 arm-only
  motion check and save the journal for cycle-by-cycle stroke and startup
  review. The encoder-based analysis is ready to use. See the
  [operator test](../../docs/ARM_RIG_TEST.md#weld-motion-accuracy-test).
- **SPEED-01, further weld tuning deferred:** preserve the operator-selected
  v7 baseline and its exact 561-command regression. Future work remains
  startup accuracy (1.095 deg whole-pass
  orientation peak) and cycle-stroke consistency (median 18.350 mm, range
  15.723-19.611 mm against 20 mm planned). Preserve its improved steady phase,
  same-time tip RMS and separate preparation behavior. Use the
  [v7 baseline and reusable scheme](../../docs/measurements/arm-weld-v7-baseline-2026-11-24.json)
  before changing response parameters, startup or residual feedback. The v8
  experiment did not establish its predicted startup improvement; retain it
  as offline evidence only. Qualify unequal load, timing, reversals, thermal
  behavior and stopping independently without raising trips. The accepted
  preparation goal remains 10 s; the older five-second goal is retired.
- **CAL-01:** independently qualify the candidate sign/offset mapping,
  derived standby and complete path against the assembly and stopping margins.
  Qualify supported startup, current-position hold pickup and recovery waypoints;
  resolve the Nov 16 pose/model mismatch and audit remaining collision exclusions.
- **LOAD-01, after the first arm weld:** identify mass/COM/inertia, reconcile
  model acceleration bounds and add qualified weight/gravity compensation.
- **FEEDBACK-01, after the first arm weld:** baseline tracking logs, tool/seam
  sensing, bounded outer feedback and a staged observation-only ESO evaluation.
  Qualify mode-2 input/delay response and the new bounded host tracker; it does
  not identify loaded dynamics or implement gravity/Cartesian compensation.
- **ILC-01, later future work:** evaluate bounded iterative learning only after
  command sampling/timing and the loaded tracking baseline are qualified.
  Learn repeatable weld-cycle errors offline first, with explicit path/load/rate
  identity, filtering, convergence checks and command derivative bounds. Do not
  add online learning to the current command fix. See the canonical registry.

Keep `hardware/` beside `simulation/`; shared planning belongs in `controller/`.
Independent arm/chassis switches only gate outputs, with no switch compensation.
