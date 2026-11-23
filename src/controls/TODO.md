# Controls open work

The canonical acceptance criteria are in [ARM_LAYER_TODOS.md](../../docs/ARM_LAYER_TODOS.md).
Current implementation and reproduction: [WELD_MOTION.md](../../docs/WELD_MOTION.md).

- **ACCURACY-01:** prioritize 3D wire-tip position and both transverse stroke
  edges. For continuous arm-only passes, report one fixed lag and the remaining
  spatial error. Use the 1.730 Hz, 20 mm matched A1 shaper-only trial and
  synchronized independent tip measurements before setting a position
  tolerance. Retain the 0.5 deg target and 1 deg maximum for same-time
  orientation as a secondary criterion. See the
  [operator test](../../docs/ARM_RIG_TEST.md#weld-motion-accuracy-test).
- **SPEED-01:** qualify PID/velocity lead under unequal load, reversals and
  saturation using actual timing, error, phase, measured Hz/travel and thermal
  evidence. Nov 23 A1/A6 frequency matches the ~1.73 Hz plan but travel exceeds
  it (3.891/2.261 and 1.333/0.428 deg measured/planned). Compare A1 host
  correction and command shaping to reduce overswing. Keep phase lag as a
  diagnostic. Retain the accepted tiny A2-A5 nominal
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
