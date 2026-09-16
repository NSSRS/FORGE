# Python motor development

## Architecture

`forge_motors.MotorBus` uses standard-library `ctypes` to load the C11
`libencos_driver.so`. A C pthread owns SOEM exchanges at a target 100 Hz on a
monotonic schedule; Python updates command buffers. This is not hard real time.

One session owns one bridge and 1-4 distinct motor IDs in contiguous PDO slots
0-3. Configuration order determines slots: entries 1-3 use CAN1, and entry 4 uses
CAN2. Motor IDs do not select the CAN channel. There is no broadcast discovery.
Arbitrary slot layouts, multiple bridges, hybrid mode, torque, brake release and
configuration writes are not exposed. Four motors do not mean four slots on CAN1.

Opening sends only position queries. A motor's first command activates it;
the others remain query-only. Initial position must be available and within
configured absolute limits before activation.

For four motors, supply four `MotorConfig` entries and command each ID separately.
The fourth motor must be connected to CAN2 with that bus correctly terminated.
Hardware verification of the fourth passage is still required; software tests
exercise all four slots through the fake bridge.

## Build and install

Use native Ubuntu 24.04 with a dedicated EtherCAT NIC and the ENCOS 86-byte OUT /
92-byte IN mapping. WSL supports compilation/fake-bridge tests here, not the
supported physical motor connection.

First complete the [native build](ENCOS_WORKSPACE.md#build-on-ubuntu), then install
the Python package from the repository root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e .
```

No third-party runtime Python dependencies are needed. Build the `.so` separately;
it is not bundled in a wheel. For a different build/install location, set
`FORGE_MOTORS_LIBRARY` to its absolute path or pass `library="/path/libencos_driver.so"`.
Simulation also works on Windows/macOS and never loads the native library.

## Independent commands

These values are simulation examples, not hardware-approved settings:

```python
import time
from forge_motors import MotorBus, MotorConfig

configs = [
    MotorConfig(motor_id=1, min_position_deg=-180, max_position_deg=180,
                max_velocity_rpm=5, max_current_a=1),
    MotorConfig(motor_id=2, min_position_deg=-180, max_position_deg=180,
                max_velocity_rpm=5, max_current_a=1),
]

with MotorBus(configs, simulate=True) as bus:
    bus.wait_ready()
    end = time.monotonic() + 2
    while time.monotonic() < end:
        bus.velocity(1, 2.0)   # signed output RPM
        bus.position(2, 15.0)  # absolute output degrees, independently
        print(bus.state(1), bus.state(2))
        time.sleep(0.02)       # renew BOTH active motors at 50 Hz

    end = time.monotonic() + 1
    while time.monotonic() < end:
        bus.stop()            # renew zero-speed targets
        time.sleep(0.02)
```

For hardware, complete the [equipment record](../docs/bench-equipment.example.md)
and [bench procedure](ENCOS_TEST_BENCH.md). Supply actual IDs, approved limits, and
an absolute position range containing the measured startup angle. Substitute
`MotorBus(configs, interface="enp86s0", execute=True)` using the actual NIC name.
Raw EtherCAT normally needs root: run `sudo .venv/bin/python your_script.py` from
this checkout. `execute=True` permits commands but does not release a holding brake.

Do not run another master or the existing CLI concurrently. The new library holds
a process-wide ownership flag and a per-interface file lock; older CLI tools and
unrelated masters do not honor that lock.

## API behavior

| Method | Behavior |
|---|---|
| `velocity(id, rpm)` | Mode 2; signed output speed with a fixed 60 RPM/s protective slew |
| `position(id, degrees)` | Mode 1; absolute target with configured speed/current ceilings |
| `state(id)` | Read `MotorState`, including after a fault; unknown position/velocity are `None` |
| `wait_ready(timeout=2)` | Wait for initial position observations; no activation or command refresh |
| `check()` | Raise `MotorError` on a latched session fault |
| `stop()` | Request zero velocity on active motors |
| `close()` | Best-effort immediate zero speed for 500 ms, then empty PDOs and release |

Context-manager exit calls `close()`. Normal `stop()` is nonblocking and does not
disarm, engage a brake, or hold chassis position. Neither a zero-speed command nor
close/fault handling promises mechanical standstill within 500 ms. The fixed
host-side slew is not exposed as a tuning parameter. The API does not change the
motor's firmware acceleration configuration or generate synchronized wheel
trajectories. Setters are serialized but are not an atomic multi-motor batch.
CAN transmission is sequential too.

Configured position bounds are enforced for Mode 1 position control. Mode 2
velocity control treats output rotation as continuous and does not fault solely
because the powered multi-turn encoder count crosses those angular bounds.
Velocity, current, temperature, motor-error, feedback-age, command-age, and
EtherCAT checks remain active in both modes. The motor's multi-turn count resets
after power loss.

Feedback types 2 and 3 alternate during motion, so position and velocity can come
from different cycles. Current is phase A, not supply current or calibrated chassis
force. Encoder angles/speeds are already gearbox-output quantities; do not divide
by the motor's internal ratio again. Application code accounts for external gearing,
wheel radius and direction. Multi-turn counts reset on power-up; choose an explicit
finite travel range and a startup reference procedure for wheel operation.

**Renew every active motor within 250 ms**, even a stationary position-holding motor.
Use approximately 50 Hz with margin. Do not issue one command and sleep for seconds.
Commands for motor 1 never renew motor 2. Invalid commands do not renew a lease.
Query-only motors have no command lease. There is no implicit Python heartbeat.

## Faults and hardware limitations

The C thread latches a global fault for command expiry, EtherCAT exchange failure,
a scheduling gap over 100 ms, missing decodable control feedback for 250 ms, motor
errors, position/speed/current limits or a temperature over 70 C. Current checking
allows max(0.5 A, 25% of the ceiling) margin, and speed checking allows 1 RPM.
These checks supplement transmitted motor-side limits; they are not model-specific
thermal or traction protection. A fault attempts zero speed on all active motors
for 500 ms, then clears outputs. No automatic rearming occurs after reconnection.
Close and investigate before explicitly reopening.

**Cached feedback:** healthy EtherCAT and a decodable PDO do not prove a fresh CAN
reply. The bridge may retain frames. `observed_monotonic_s` is host observation time,
not verified CAN reception time. Alternating reply types obtains both measurements
but does not solve cache freshness. Bridge status bytes are uninterpreted pending
a verified firmware definition. Limit checks can miss conditions hidden by stale
telemetry. `state()` values remain available after a fault but can be stale;
call `check()` in application loops.

Before chassis use, verify bridge receive validity/counters, host-loss output
invalidation, motor timeout settings, stopping/braking under load, and actual
timing. Killing the Python process also kills its C thread; its watchdog cannot
run after that. A hung Python application with a live C thread is a different
case and is covered by the command lease. Physical stopping and complete host-loss
behavior must be provided and tested independently.

The simulator is ideal kinematics, not a model of gravity, slip, current, motor
dynamics, braking or EtherCAT timing. It evaluates elapsed time when the API is
called. Native watchdog tests instead use a fake bridge with the actual C thread.

## Next development layers

Both manual and automatic operation can feed `velocity()`:

1. Keys/joystick -> chassis speed/yaw rate -> wheel RPM.
2. Reference path/speed plus estimated chassis state -> tracking corrections -> wheel RPM.

Kinematics, keyboard mapping, localization, trajectory tracking and camera/IMU
fusion are not implemented by this interface. Position mode remains available
for independent motor-angle experiments.

## Verification

See [Motor tests](../tests/README.md) for native/fake-bridge coverage and portable
Python test commands. Software tests do not establish physical motion or stopping
behavior; dated hardware evidence is in the
[bench commissioning record](ENCOS_TEST_BENCH.md#commissioning-record-2026-09-16).

Protocol source: supplied ENCOS V1.19EAP printed sections 4.1-4.5, 9.1.2-9.1.3,
10.2-10.3. The supplier PDF remains outside the public repository.
