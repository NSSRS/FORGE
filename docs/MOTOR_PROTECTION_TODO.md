# Motor protection verification — deferred

Updated 2026-11-26. **Deferred by the operator; no test is currently selected.**
This is reference material for PROTECT-01 in the
[consolidated registry](ARM_LAYER_TODOS.md), including former COMMS-06.

Guarded, disabled-only readback is implemented. The operator's
[six-motor inspection](measurements/arm-control-readback-review-2026-11-17.json)
completed on Nov 17, including A1. The earlier A2–A6 values and interpretation
are in [DAMIAO_TORQUE_LIMITS.md](DAMIAO_TORQUE_LIMITS.md).
No gains or current limits were written. Questions to address when resumed:

- Verify engineering units/scaling and supported protection settings for the
  installed Damiao and ENCOS models, using the existing identity/readback evidence.
  Do not assume the 4310P Imax/VBus registers exist on the 8009P; A1's
  undocumented fields remain unknown.
- Distinguish current-command clamping, overcurrent fault shutdown and
  FORGE's reactive measured-torque trips. Verify drive torque/current reporting
  against independent load/phase-current measurements; PSU current is different.
- Establish useful operating limits from independent current/load and thermal
  measurements. Reuse BASE-07 for stop/recovery and BASE-06 for configuration
  identity rather than creating duplicate test campaigns here.

Keep production calibration and protection-confirmation flags unchanged until
their separate acceptance is complete. A successful register read is evidence
of a value, not validation of its suitability for this arm.
