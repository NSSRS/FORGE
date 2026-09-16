"""Simulation only: independently command two motors through the public API."""
import time
from forge_motors import MotorBus, MotorConfig

configs = [MotorConfig(i, -180, 180, 5, 1) for i in (1, 2)]
with MotorBus(configs, simulate=True) as bus:
    bus.wait_ready()
    start = time.monotonic()
    while time.monotonic() - start < 2:
        bus.velocity(1, 2.0)
        bus.position(2, 15.0)
        print(bus.state(1), bus.state(2))
        time.sleep(0.02)
    # Continue zero-speed commands while the velocity ramp settles.
    start = time.monotonic()
    while time.monotonic() - start < 1:
        bus.stop()
        time.sleep(0.02)
