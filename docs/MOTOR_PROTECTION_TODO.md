# Motor protection verification TODO

Recorded 2026-11-05. User deferred protection-setting verification for the
current secured, unloaded, free-shaft 8009P/4310P basic function tests.
Passing these tests does not verify current protection or loaded operation.

- [ ] Verify protection settings for every physical motor: all Damiao arm
  units (one 8009P, two 4340P, three 4310P) and all ENCOS chassis units.
- [ ] Record each unit's model/revision, firmware, bus/IDs, existing current
  limit and overcurrent fault threshold, units/scaling, and timeout behavior.
- [ ] Distinguish a current-command clamp from an overcurrent fault shutdown
  and from FORGE's reactive measured-torque stop.
- [ ] Add guarded read-only MC02 diagnostics for documented protection and
  version registers. Confirm model-specific support; do not assume the 4310P
  Imax register exists on the 8009P.
- [ ] Select and verify application-specific limits before loaded testing;
  record readback, persistence across power cycles, and fault-stop results.
- [ ] Use ENCOS-specific documentation and tooling for chassis motors.

Factory defaults remain unverified. Preserve production calibration and
hardware-confirmation flags until their separate commissioning is complete.
