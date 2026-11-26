# Arm base-layer contract and implementation plan

Status: 2026-11-26, `nathan`. This is the base transport/execution layer, not
an arm planner. Code/build paths are relative to the FORGE checkout; deployment tools
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

## Implemented APIs

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
The current rig build `cd93f9b3 / 0x7FFFF8F` provides these APIs, disabled
control-register readback, explicit manual travel and atomic mode-2 groups. Normal profile changes within the supported
envelope and successful repeat sessions do not require reflashing. Production
ROS mode, trajectories, current-limit writes and physical bus/load qualification
remain separate work; see [current status](ARM_SESSION_HANDOFF.md).

## BASE-06 atomic group targets

Implemented offline on Nov 26. Capability bit 26 (`0x04000000`) enables USB
type 30 and `ArmBus.command_group(targets, velocities, smooth_slots=())`.
Both mappings use physical slots 0–5 and must cover the complete configured
routed mode-2 group. A subset keeps its original slots. MIT mode and torque/gain
batches are outside this API. The existing configuration, enable and stop APIs
are unchanged.

One ordinary 64-byte CRC-protected frame contains the entire request. Payload
offsets exclude the eight-byte frame header; all numbers are little endian:

| Offset | Format | Meaning |
| --- | --- | --- |
| 0 | u8 | Complete configured slot mask, bits 0–5 |
| 1 | u8 | Smooth-jog slot mask, a subset of byte 0; zero for trajectories |
| 2–3 | two u8 | Reserved, must be zero |
| 4 + 8 × slot | float32 | Raw output-shaft position, rad |
| 8 + 8 × slot | float32 | Positive speed allowance, rad/s |

Unused slot fields are ignored; the host zeroes them. Smooth slots retain the
existing confirmed manual-hold state and speed checks. Successful response
payload byte 0 is zero, bytes 1–4 contain the common MCU commit timestamp in
milliseconds, and the rest are zero. The outer response echoes the request
sequence. Rejections use the existing typed reason/first-failing-slot format;
group-wide errors use slot `0xff`.

The parser waits for the whole valid frame. Validation checks the exact group,
every slot's state, readback, bounds, finite values and fresh feedback before
changing any target. Enabled slots must still have an unexpired 150 ms command
lease, even when USB is serviced just before the watchdog tick. A frame whose
assembly took at least 150 ms is rejected with `STALE_PRELOAD`; this measures
time since the parser first saw its header, not host-to-device transit latency.
Repeated/older sequences are rejected within the existing USB session.
Rejected or incomplete batches change no target, lease or smoothing state and
send no CAN frames. Existing watchdogs continue independently and can fault/stop
while an incomplete frame is pending. No separate staging/commit exchange exists.

After validation, all selected commands and lease timestamps change together
in the main loop, before `arm_tick` runs. USB interrupts only enqueue bytes;
they do not run this commit or the control tick. This provides atomic controller
state, not simultaneous CAN transmissions or mechanically synchronized arrival.
A CAN execution failure still follows the existing whole-group fault path.

The sole group-hold owner uses this API for complete refreshes, including
preload. Selected-joint manual start/jog operations retain their existing
single-axis semantics. The trajectory owner uses it for every complete path or
hold update and checks operator intent around dispatch. A rejected request,
short write or lost ACK stops through the existing owner; no motion retry or
fallback is attempted after an uncertain commit. Successful command delivery
followed by intent loss retains the existing pause/hold policy.

Firmware without bit 26 retains the established sequential command path.
`group_command_transport` records `atomic_mode2` or `legacy_sequential`; weld
reports also record `usb_command_type` as 30 or 4. The operator deployed
`cd93f9b3 / 0x7FFFF8F` on Nov 26. USB identity, typed readback rejection,
selected-profile configuration comparisons and 7.12 seconds of supported atomic
group holding passed, with final disable/mode restoration verified. BASE-06 is
complete within that stationary scope. The prior `f78f6b0d` image does not support type 30. See
[deployment evidence](measurements/arm-base06-deployment-2026-11-26.json).

## CAN capacity and information policy

Capability bit 21 (`0x200000`) permits the explicit A2 gun-load trip up to
30 Nm at slot 1 / CAN2 / ID2 / feedback18 / 4340P. Bit 20 (`0x100000`) retains
the earlier 13 Nm compatibility. Bit 23 (`0x800000`) adds **A3 up to 30 Nm**
and **A4-A6 up to 10 Nm**, restricted to their known physical slots, CAN2,
IDs 3-6 / feedback19-22 and matching 4340P/4310P models. Other routes retain
rated configuration ceilings. Defaults and planning budgets stay unchanged.
Host preflight rejects missing capabilities before motor operations. Host and
MCU both reject extended trips at or above actual TMAX; the expanded profile
requires A2/A3=40 and A4-A6=12.5. A2's TMAX=40 persistence was physically
verified Nov 19; A3-A6 setup and subsequent fresh-MCU/readback verification
are recorded in the [gun-load review](measurements/arm-gun-load-review-2026-11-19.json). No overload timer is added.
Cached signed torque and both temperature channels use the existing owner.
These are reactive trips, not commanded current or qualified continuous ratings.

Capability bit 22 (`0x400000`) retains USB type 27 `[op]` for fixed A2 TMAX=40.
Bit 24 (`0x1000000`) adds type 28 `[joint, op]`, where joint is strictly 3-6:
A3 writes TMAX=40, A4-A6 write TMAX=12.5. Operation 0 writes the fixed register
23 value, operation 1 saves once after independent readback. The complete known
rig must have fresh stationary disabled feedback and verified ranges/modes.
The setup uses rated-reference trips and accepts only the known original range
(28 or 10). No arbitrary register/value/route is writable. All motion and
route/mode changes lock until MCU restart; fault reset cannot clear the lock.
Another joint can be configured only after the preceding save acknowledgement;
each joint can be written/saved at most once per boot. An uncertain operation
blocks progression without retries. The 50 ms save quiet interval remains.

Base-info retains its original ten uint32 fields and legacy flags at payload
byte 41: bit 0 globally locked, bit 1 latest target readback verified, bit 2
latest save attempted. Older hosts therefore also see the lock during A3-A6
setup. Under bit 24, payload byte 45 contains a uint32 physical joint number
(0 if no setup), and bytes 49/50/51 contain written/save-attempted/save-acknowledged
slot masks. All offsets include the leading status byte; packet size is unchanged.
The Nov 19 build advertised **0x1FFFF8F**; the Nov 26 image retains these
capabilities. Range persistence and readback are complete on this rig. A new
range-setup session requires the documented power cycle and readback. See the
[setup procedure](ARM_RIG_TEST.md#expanded-gun-load-profile-a2a3-30-nm-a4-a6-10-nm).

Optional `--smooth-jog` requires capability bit 18 (`0x40000`) and USB type 25,
using the same slot + five-float payload as type 4. Only confirmed enabled,
manual mode-2 holding accepts it. Capability bit 19 (`0x80000`) extends its
ceiling from 5 to 10 deg/s for the requested holding-mode trial. The host checks
this before H preloads/enables at 10 deg/s. It slews from the last
transmitted target toward the host endpoint by at most `v × 2 ms` each tick;
late ticks drop missed advancement and endpoints are never extrapolated.
This keeps the existing 500 Hz CAN rate. Each accepted host command renews its
150 ms lease; interpolation and telemetry never do. Type 4 cancels smoothing
and applies the new target on the next tick, preserving immediate release/hold
semantics. Stop/fault clears the ramp. This is a commissioning experiment,
not acceleration planning or physical jitter qualification.

The optional keyboard selected-joint hold path uses capability bit 16
(`0x10000`) and routed configure mode flag `0x40` together with manual-travel
flag `0x80` and mode 2. Those policy bits never reach the drive's CTRL_MODE.
Only that explicitly configured path omits the 15-second bench cutoff; command
and feedback leases, enable confirmation and all trip checks remain unchanged.
Firmware rejects group enable or another enabled joint while this hold policy
is active. The host holds one fixed measured release target and never treats
telemetry as a command refresh. See [operator behavior](ARM_RIG_TEST.md#optional-selected-joint-position-hold).

Explicit group hold adds capability bit 17 (`0x20000`) and policy flag `0x20`
alongside those existing flags. Firmware permits only the complete configured
group, with every preload/freshness/enable check passing before the first enable;
individual enable and mixed policies are rejected. The host group adapter owns
all fixed targets and refreshes all leases, moving only the selected target.
Selection preserves torque and revokes jog authorization. A fault on any joint
still stops all joints. This is powered mode-2 holding, not a mechanical brake
or the production ROS lifecycle; physical bus/load/thermal acceptance is pending.

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
The ROS arm consumer publishes snapshots at 25 Hz and reads health at 2 Hz on
the existing owner. [Dual-family ROS telemetry](MOTOR_TELEMETRY.md) documents
ages, diagnostics, ENCOS observation limits and offline validation. Measure
physical host command latency before raising these rates; the CAN rate is unchanged.

Registers (identity, mode, ranges, version/current protection) belong in
startup/disabled commissioning, not steady-state polling. BASE-04 was
[resolved by operator confirmation on Nov 23](ARM_SESSION_HANDOFF.md#plan-01-and-base-04-resolution).
Retain the final split-bus topology. Do not silently lower refresh
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

Passive inspection of an idle controller with no competing serial owner:

```bash
python3 scripts/inspect_dm_base.py --port /dev/ttyACM0 --samples 50 \
  --interval 0.1 --report build/arm_tests/base-telemetry.json
```

The CLI does not send enable, parameter writes, register reads or keepalives.
It does not silently configure motors; unconfigured slots remain invalid.

## Wire extension (little endian, existing 64-byte CRC frame)

Cached telemetry uses capability bit 12 (`0x1000`). API version 1.
The 2026-11-10 torque-ceiling extension adds bit 13 (`0x2000`) for explicit
4340P trip thresholds above 3 Nm and up to the manual-rated 12 Nm. Current
source mask after manual travel is `0xFF8F`: bit 14 (`0x4000`) permits guarded
disabled control readback; bit 15 (`0x8000`) permits explicit manual travel.
Build `156f20c8` / mask `0xFF8F` was verified deployed on 2026-11-10; see the rig guide. Type-16
mode byte bit 7 requests manual travel with mode 2 (0x82); the flag stays on the
MCU and motor mode readback remains 2. Only these slots bypass the narrow
startup-corridor width check at enable; raw protocol bounds, near-position
preload, 3.2-rad command-distance check, trips and watchdogs remain. No wire
layout changes; old firmware remains usable with <=3 Nm 4340P bounded profiles.
Loaded/bus qualification remains pending. See
[DAMIAO_TORQUE_LIMITS.md](DAMIAO_TORQUE_LIMITS.md).
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

## BASE-05 specification

Implemented and verified offline on Damiao DM-MC02 firmware and the existing
Python ArmBus owner. Verify the readback against the rig during bench commissioning.

### Typed request rejection

Retain the 64-byte version-1 frame, CRC, request types and success payloads.
Advertise `CAP_ARM_CONFIG_DIAGNOSTICS` (bit 25, `0x02000000`) in HELLO and
base info when both BASE-05 features are available. Keep response payload
byte 0 at `1` for rejected requests. For supported diagnostic replies use
byte 1 for a reason and byte 2 for the attempted or first failing zero-based
joint slot. Use `0xff` when there is no meaningful slot. Reason `0` denotes a
generic rejection. Legacy hosts continue to use the existing response status.

Cover configure (2, 16), probe (3), setpoint (4, 25), enable (5, 18), and active
configuration readback (29) in the first implementation. Return the first
failing selected slot for a per-slot group-enable validation error, in slot
order. Group-wide failures use the no-slot sentinel.

| Code | Reason | Meaning |
| --- | --- | --- |
| 1 | `INVALID_ARGUMENT` | Invalid slot, mask, mode encoding or non-finite numeric input |
| 2 | `INVALID_ROUTE` | Invalid CAN address or conflicting route |
| 3 | `INVALID_STATE` | Configuration, probe, enable, stop, preload or commissioning state prevents the operation |
| 4 | `STALE_FEEDBACK` | Feedback is absent or has expired at an existing freshness gate |
| 5 | `STALE_PRELOAD` | A required preload is absent or its command lease expired |
| 6 | `LIMIT_EXCEEDED` | Finite value outside an enforced bound, or a numeric setpoint with a reserved command encoding |
| 7 | `FAULT_LATCHED` | An existing fault blocks the requested operation |
| 8 | `UNSUPPORTED` | Unknown request or unavailable feature |
| 9 | `CAN_TX_FAILED` | A required CAN transmission failed during an otherwise eligible operation |
| 10 | `SESSION_REQUIRED` | Request requires an established USB session |
| 11 | `STALE_SEQUENCE` | Request sequence is repeated or behind the active session |

Return the reason from the existing validation path. Preserve its acceptance
criteria and error precedence. Keep STOP accepted on replay and discard
malformed or CRC-invalid frames as before. A validation rejection leaves
setpoints, leases and the fault latch unchanged and emits no CAN traffic.
An execution failure follows the existing fault and stop path. Keep typed
rejection separate from the asynchronous first-fault record.

Expose valid rejection responses as `ArmRequestRejected(RuntimeError)` with
`request_type`, `reason`, and `slot` attributes. Map legacy generic
rejections and unrecognized reason codes to `UNKNOWN`. Translate a missing
slot to `None`. Keep local host validation and USB timeout errors distinct.

### Active configuration readback

`ArmBus.active_config(slot)` uses USB request type 29 and one unsigned
8-bit slot (0–5). Read the active MCU slot record in a single response. Return
`configured=False` for an unused slot. Return `None` for fields without
valid readback, including ranges whose probe-validity bits have not been set.
Readback only uses MCU state and the existing USB owner.

Success payload, little endian. Offsets start at the response status
byte. Reserved bytes are zero.

| Offset | Format | Fields |
| --- | --- | --- |
| 0 | u8 | Success status |
| 1 | u8 | Slot flags. Bits 0–3 are configured, manual travel, position hold and group hold |
| 2–5 | four u8 | CAN bus, selected mode, model code, verified register mask |
| 6–11 | three u16 | Motor CAN ID, effective setpoint CAN ID, configured feedback CAN ID |
| 12–35 | six float32 | Profile lower/upper bounds, speed trip, torque trip, Kp and Kd ceilings |
| 36–47 | three float32 | Last verified `PMAX`, `VMAX` and `TMAX` |
| 48–51 | four bytes | Reserved |

Position, velocity and torque use raw output-shaft rad, rad/s and Nm. Return
the exact configured feedback ID, including valid CAN ID zero on factory
bench routes. Read mode and setpoint ID from the firmware's existing logic. Keep profile limits and probed protocol
ranges distinct. Physical limits and calibration provenance belong to the
measured profile. The active record describes one slot at request time.

For firmware lacking the capability, `active_config()` reports unsupported
before sending type 29. Legacy commands and telemetry keep their current
behavior. Readback neither probes CAN registers nor refreshes a motion lease.

### Offline acceptance

- Exercise legacy and new host/firmware combinations with the native bridge.
  Verify byte-for-byte existing success payloads, generic legacy rejection,
  unknown reason handling and the capability check before type 29.
- Exercise non-finite and out-of-bounds values, conflicting routes, active
  reconfiguration, missing and stale feedback, expired preloads, faulted
  state, group rejection and CAN TX failure. Check command age and CAN sends
  before and after rejected validation. Verify STOP replay and fault latching.
- Read an unconfigured slot, then valid configuration, changed routes and
  post-probe ranges. Compare against MCU state and verify that missing range
  validity yields `None`. Check that readback leaves CAN TX counts and
  lease timestamps unchanged through the existing owner.
- Run CTest and the C-backed Damiao Python suite. The STM32 build produces
  an offline image for later deployment.
- On the bench, compare the readback with the selected profile, documented
  CAN routes and disabled motor range readback using the existing owner.

## Validation and remaining boundaries

Run Python `test_damiao*.py`, CMake/CTest, and the MC02 build. The current
software tests use a native C-backed fake plant, not real bus contention.
No upper-layer production/calibration confirmation flags are set by this work.
See stable TODO IDs in the cross-layer handoff for hardware acceptance,
telemetry consumers, batching, protection, calibration and loaded stopping.

Current reproduction commands are in [tests/README.md](../tests/README.md).
