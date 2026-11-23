# Welding machine open work

- **WELDER-01:** obtain the exact machine model, firmware, connector pinout,
  transport settings, unit ID and register-map revision. Use the
  [bring-up procedure](README.md) to verify a documented status point against
  the machine panel and record the observed raw and scaled values.
- Add named status fields and machine commands once the map establishes their
  units, word order, permitted values and readback. Verify command handling
  against a simulated endpoint before separate physical commissioning of
  arc enable, stop, interlocks and communication loss.
