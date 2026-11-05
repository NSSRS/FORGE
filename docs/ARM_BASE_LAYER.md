# Arm base-layer contract and implementation plan

Status: 2026-11-05, `nathan`. This is the base transport/execution layer, not
an arm planner. All project code/builds live in `forge2_ws`; deployment tools
may be installed elsewhere. Cross-layer work: [ARM_LAYER_TODOS.md](ARM_LAYER_TODOS.md).

## Layer boundaries

| Layer | Owns | Must not infer or do implicitly |
| --- | --- | --- |
| Motor drive | Internal position/velocity/current loops, drive protections | Assume factory current protection is application-safe |
| MC02 base firmware | CAN routing/codec, bounded command execution, command/feedback leases, enable/disable confirmation, latched faults, cached feedback and bus health | IK, ROS joint conventions, gravity compensation, automatic fault clearing/re-enable |
| Python base API | Framing, capability/version checks, raw shaft SI values, cached snapshots and health access | Background replay of old motion commands, hidden CAN register polling |
| Commissioning runner | Explicit test plans, ID/mode readback, out-and-back tests, cleanup | Mark loaded operation or calibration validated from a free-shaft test |
| ROS control adapter | One device owner, coordinate conversion, publication freshness, command arbitration, lifecycle policy | Treat the current production MIT path as validated by position-velocity bench tests |
| Planning / teleop | Trajectories, acceleration/jerk, synchronization, collision avoidance and operator intent | Bypass joint/firmware limits or continually renew a stale command |
| Bringup / safety integration | Physical stop chain, gravity support/brakes, power sequencing, deployment identity | Assume torque disable safely holds a loaded arm |

## Work applied in this change

- Cached six-slot snapshot with one MCU capture timestamp and generation, returned
  in three USB pages. Each motor retains its own feedback age and validity.
- Per-bus completed TX/failure and RX/feedback counters; observed feedback gaps,
  control service delays, bus-off transitions and controller error state.
- API revision, capability mask, timing constants, configured mask, ID-session
  lock state, and a build fingerprint available without CAN register reads.
- Python APIs and an opt-in passive inspection CLI. Older firmware fails with
  a clear missing-capability error; there is no fallback to CAN diagnostics.
- Native-backed tests for no extra CAN traffic, immutable snapshot pages,
  stale generation rejection, invalid data, lease expiry and fault preservation.
- No ROS control-mode conversion, trajectory generator, current-limit write,
  firmware flash or motor motion is performed as part of this change.

This extends the existing firmware. It requires one deployment flash to use
these new APIs; normal motion parameter changes within the supported envelope
and successful repeated tests still do not require reflashing. Existing
commands remain compatible. The new build has not been hardware-qualified.

## CAN capacity and information policy

The recorded temporary topology is **A1 alone on CAN1; A2..A6 on CAN2**.
The planned final topology is A1/CAN1, A2..A3/CAN2, A4..A6/CAN3. If a speaker
calls the five-motor bus CAN1, confirm physical mapping before changing config.

For 11-bit classic CAN, eight data bytes, one command plus one response per
motor/update, including intermission: estimate 111..135 bits/frame.
`load = motors * 2 * update_hz * bits_per_frame / bitrate`.

| Motors on bus | Update rate | Estimated data-frame load at 1 Mbps |
| --- | --- | --- |
| 1 | 500 Hz | 11.1–13.5% |
| 3 | 500 Hz | 33.3–40.5% |
| 5 | 500 Hz | 55.5–67.5% |
| 5 | 1000 Hz | 111–135%: cannot fit |

Sources: [Vector standard CAN stuffing explanation](https://certification.vector.com/mod/page/view.php?id=349)
(132-bit maximum frame plus three intermission bits),
[MicroControl CAN introduction](https://www.microcontrol.net/service/grundlagen/can-bus/)
(111-bit eight-byte frame including intermission). The estimates exclude error
frames, retransmissions, other nodes, diagnostics and scheduler delays; they
are not measured utilization or proof of timing headroom. Existing synchronous
CAN send waits also affect scheduling even if the theoretical bus load fits.

Position/velocity/torque/status and drive temperatures already arrive in normal
feedback. Expose cached data to upper layers instead of adding CAN queries.
Start upper-layer snapshot publication at 20–50 Hz and health at 1–5 Hz;
measure host command latency before raising these rates. These are proposed
consumer rates, not new CAN rates or automatic background threads.

Registers (identity, mode, ranges, version/current protection) belong in
startup/disabled commissioning, not steady-state polling. The five-motor bus
may be workable at 500 Hz, but must pass BASE-04 measurement before loaded use.
Splitting to the final topology adds headroom. Do not silently lower refresh
rates, stretch watchdogs, or enable CAN FD to hide congestion.

## Python API

Use one owning `ArmBus`; do not open a second serial client while another
process controls the motors. HELLO rejects an actively enabled controller.
During operation, call these methods on the existing owner's instance:

```python
info = bus.base_info()        # require api_version == 1
sample = bus.snapshot()       # six motor slots; three USB request/reply pairs
health = bus.bus_health(2)    # one USB request/reply, no CAN frame
fault = bus.fault_snapshot()  # existing first-fault detail
```

`base_info()` returns API version, capabilities, control period, command lease,
feedback/enable timeouts, CAN bitrate, configured-slot mask, ID-session lock,
and build ID. The build ID is a 32-bit fingerprint of build-script/source/header
inputs and flags, not a security signature. Zero identifies untagged/native builds.
A snapshot is a frozen MCU cache, **not simultaneous physical sampling**. Ages
are measured at capture time, in milliseconds. For conservative freshness,
add `acquisition_duration_ms` plus elapsed host time since
`host_received_monotonic_s`; the acquisition duration bounds the extra age
introduced while assembling three USB pages. Reject stale or invalid data; do not use a
snapshot as a motion acknowledgement for a command issued after capture.
`feedback_valid=false` means position/velocity/torque/age are `None`, not zero.
Age saturates at 65535 ms. Slots are mapped by the owner's verified profile.
Temperature/status bytes without valid feedback are not evidence of a sample.

Snapshot flags expose configured/enabled/stopping/enable-confirmed state;
the top level contains fault/reason/slot. Reading any telemetry never renews a
motion lease, clears a fault, configures a motor or sends CAN traffic. Existing
`state(slot)` remains available with detailed register and fault fields.

Counter fields are unsigned 32-bit and wrap; they reset on MCU boot. Do not
compare samples across a reboot. `tx_frames` counts completed driver sends,
not arbitration retries. `tx_failed` counts unsuccessful send callbacks.
`rx_frames` counts standard received data frames up to eight bytes passed to
core firmware. `feedback_frames` counts accepted feedback with decoded ranges.
Max feedback gaps are recorded while a joint is enabled; max control tick gaps
and late ticks are recorded when any joint is enabled. These are millisecond
observations, not an oscilloscope measurement or a latency percentile.
Status flag bits: 0 bus off, 1 error passive, 2 warning, 3 HAL status/counter-read
failure, 4 sticky RX FIFO0 message loss, 8..10 sampled last-error-code. Readings
do not clear these counters or flags. Controller error counts are snapshots.

Offline capacity estimate (no device opened):

```bash
python3 scripts/inspect_dm_base.py
```

After deploying the new firmware, passive inspection of an idle controller:

```bash
python3 scripts/inspect_dm_base.py --port /dev/ttyACM0 --samples 50 \
  --interval 0.1 --report build/arm_tests/base-telemetry.json
```

The CLI does not send enable, parameter writes, register reads or keepalives.
It does not silently configure motors; unconfigured slots remain invalid.

## Wire extension (little endian, existing 64-byte CRC frame)

Capability bit 12 (`0x1000`); total current mask `0x1F8F`. API version 1.
Existing type-7 state and type-19 first-fault replies are unchanged.
Offsets below are within the response payload, whose byte 0 is result status.

| Request type | Request payload | Response payload from byte 1 |
| --- | --- | --- |
| 22 | page u8, generation u32 | generation u32, capture_ms u32, fault/reason/slot u8 each, two motor records |
| 23 | bus u8 (1..3) | 12 u32 values in `ArmBus.bus_health()` key order |
| 24 | empty | 10 u32 values in `ArmBus.base_info()` key order |

Page 0 creates a new snapshot; pages 1/2 require its generation. A different
capture invalidates old generations; reject the whole sample if pages differ.
Motor record: position/velocity/torque float32, age u16, flags/status/MOS/rotor
u8 each (18 bytes). Flags: configured=1, enabled=2, valid=4, stopping=8,
enable-confirmed=16. No motor-specific register requests are generated.

## Validation and remaining boundaries

Run Python `test_damiao*.py`, CMake/CTest, and the MC02 build. The current
software tests use a native C-backed fake plant, not real bus contention.
No upper-layer production/calibration confirmation flags are set by this work.
See stable TODO IDs in the cross-layer handoff for hardware acceptance,
telemetry consumers, batching, protection, calibration and loaded stopping.

Validation for this implementation: **53 Python tests and 2 native C tests
passed**, local documentation links and `git diff --check` passed, and the
MC02 firmware build completed with build ID `4f73a8b2`. Only existing vendor
compiler warnings were emitted. Hardware was not flashed or enabled during
this base-layer change; BASE-04 and BASE-07 acceptance remain pending.
