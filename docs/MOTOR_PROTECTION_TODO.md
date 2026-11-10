# Motor protection verification

Updated 2026-11-10. Open work under **PROTECT-01** in the
[cross-layer registry](ARM_LAYER_TODOS.md). A2–A6 basic manual movement is verified
by the operator; that does not establish current protection or loaded operation.

Guarded, disabled-only readback is implemented. The operator's A2–A6 inspection
completed at 19:56 on 2026-11-10; see [values and interpretation](DAMIAO_TORQUE_LIMITS.md).
No gains or current limits were written. Remaining work:

- [ ] Record and verify model/revision, firmware, bus/IDs, current/protection
  settings, units/scaling and timeout behavior for every Damiao and ENCOS unit.
  Extend the current A2–A6 record to A1 when its fixture is ready; do not assume
  the 4310P Imax/VBus registers exist on the 8009P.
- [ ] Distinguish current-command clamping, overcurrent fault shutdown and
  FORGE's reactive measured-torque trips. Verify drive torque/current reporting
  against independent load/phase-current measurements; PSU current is different.
- [ ] Select application-specific operating limits and verify persistence,
  temperature behavior and measured fault/stop response before loaded testing.
- [ ] Complete ENCOS-specific verification with its documentation and tooling.

Keep production calibration and protection-confirmation flags unchanged until
their separate acceptance is complete. A successful register read is evidence
of a value, not validation of its suitability for this arm.
