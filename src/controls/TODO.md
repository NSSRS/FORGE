# Controls open work

The canonical acceptance criteria are in [ARM_LAYER_TODOS.md](../../docs/ARM_LAYER_TODOS.md).
Current implementation and reproduction: [WELD_MOTION.md](../../docs/WELD_MOTION.md).

- **ACCURACY-01:** meet end-effector orientation error of 0.5 deg target / 1 deg
  maximum relative to the planned tool frame during welding. The
  [recorded baseline](../../docs/measurements/arm-accuracy-speed-review-2026-11-23.json)
  estimates 1.408 deg RMS / 2.633 deg peak. Joint errors are diagnostic.
  Add synchronized independent workpiece-relative tool-pose measurements and
  define the separate tip-mm tolerance.
  Use the [operator test](../../docs/ARM_RIG_TEST.md#weld-motion-accuracy-test)
  for matched comparisons after tracking changes; end continuous runs manually.
- **SPEED-01:** qualify PID/velocity lead under unequal load, reversals and
  saturation using actual timing, error, phase, measured Hz/travel and thermal
  evidence. Nov 23 A1/A6 frequency matches the ~1.73 Hz plan but travel exceeds
  it (3.891/2.261 and 1.333/0.428 deg measured/planned). Prioritize host lead,
  command shaping and phase lag. Retain the accepted tiny A2-A5 nominal
  paths; synthetic response and geometry tests do not establish physical tuning.
  The operator accepted the 8.22 s preparation and retired the five-second goal.
  Preserve that preparation and existing trips while correcting welding motion.
  [Full logs, calibration and offline commands](../../docs/measurements/weld/2026-11-23/README.md)
  support development without motors.
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

Keep `hardware/` beside `simulation/`; shared planning belongs in `controller/`.
Independent arm/chassis switches only gate outputs, with no switch compensation.
