# Controls open work

Status and acceptance scope live in the [consolidated registry](../../docs/ARM_LAYER_TODOS.md).
Implementation and reproduction live in [WELD_MOTION.md](../../docs/WELD_MOTION.md).

- **ROS-02 (deferred by the operator on Nov 26):** integrate the production lifecycle using the atomic target API;
  reuse the coordinated owner's existing stop/recovery tests. Weld trajectory
  dispatch already selects the atomic API when firmware supports it.
- **CAL-01:** integrate the reviewed mapping for the next required operating
  range; investigate concrete model/physical discrepancies.
- **SPEED-01:** weld tuning and broader manual-jog investigation are deferred.
  Preserve v7; use the existing measurement review if this work resumes.
- **LOAD-01 / FEEDBACK-01 / ILC-01:** future control features, not current bench
  acceptance requirements. Gun-off/gun-on comparison is removed; ILC is deferred.

PROTECT-01 is deferred. Hardware evidence and production flags keep their
recorded scope. Shared planning belongs in `controller/`; `hardware/` and
`simulation/` remain siblings. Output switches do not enable compensation.
