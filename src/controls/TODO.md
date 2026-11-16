# Controls open work

The canonical acceptance criteria are in [ARM_LAYER_TODOS.md](../../docs/ARM_LAYER_TODOS.md).
Current implementation and reproduction: [WELD_MOTION.md](../../docs/WELD_MOTION.md).

- **WEAVE-01:** qualify actual initial -> approach -> weld -> hold execution,
  both independent output switches, tracking and timing on the supported arm.
  Target 2 Hz remains configured; the checked provisional-model CMA-ES plan
  reaches 1.815288 Hz. Animation/fake motors do not complete physical acceptance.
  Resolve the Nov 16 A5 physical drop/alignment report and qualify actual tracking
  before another complete recovery trial. The nominal-timeout defect is fixed
  offline; this does not establish successful hardware recovery.
  Meet the requested five-second total preparation goal through a checked route
  and measured position-loop response, without widening tracking/arrival trips.
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
