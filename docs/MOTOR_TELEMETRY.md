# ENCOS and Damiao ROS telemetry

The existing motor drivers own their transports and publish passive cached
telemetry. No extra serial owner, EtherCAT master, CAN register polling or
motion-command heartbeat is created by these readers.

| Family | Joint feedback | Normal rate | Diagnostics |
| --- | --- | --- | --- |
| Damiao / DM-MC02 | `/arm/joint_states`, raw-shaft rad, rad/s, reported Nm | 25 Hz | `/diagnostics`: `damiao/controller`, `damiao/can_N`, `damiao/arm_joint_N` |
| ENCOS / EtherCAT bridge | `/joint_states`, output-shaft rad and rad/s; effort empty | 50 Hz | `/diagnostics`: `encos/bridge`, `encos/motor_ID` |

Diagnostics publish at 2 Hz and immediately when status changes. Their header
is publication time; values carry explicit observation/feedback ages. Existing
joint names and command topics remain unchanged. ENCOS phase current stays in
diagnostics as amperes, not JointState effort/torque. Damiao torque is the
drive's estimate, not an independently calibrated shaft-torque measurement.

## Damiao snapshot and bus health

The ROS driver uses the existing owner's `base_info()`, `snapshot()` and
`bus_health()` methods. Firmware must support cached telemetry API version 1;
there is no fallback to enabled register polling. Base metadata includes build
identity, capabilities, configured slots and deadlines. Health reads occur at
2 Hz for the buses in the explicit profile and expose TX/RX counters, feedback
gaps, late ticks, bus-off and error flags.

Each snapshot freezes six cache slots at one MCU time and generation. It is not
simultaneous motor sampling. The host rejects repeated/reversed generations and
inconsistent or partial slot lists. The Python transport also rejects mixed
snapshot pages. Unsigned generation wrap is supported.

Effective sample age includes the motor's age at MCU capture, the complete
three-page USB acquisition duration and elapsed host time since receipt.
JointState requires six configured, valid, finite, fault-free samples younger
than 100 ms. Missing or stale joints remain visible in diagnostics; no zeros
are substituted and no incomplete arm state is published. The joint header is
the conservative time of the oldest included sample, not a new timestamp for
old feedback. Bus-health age includes its USB acquisition time as an upper bound.

## ENCOS observation limits

`MotorBus.snapshot()` copies all configured motors and transport counters under
the native driver's existing mutex. It performs no EtherCAT exchange, sends no
command, changes no lease and remains readable after a fault. The original
`state()` C/Python ABI remains available. The snapshot function is an additive,
size-checked API; rebuild the native library before using the updated ROS node.

Position, velocity and current/temperature carry separate observation times.
Position replies and alternating ACK types must not refresh each other's ages.
A vector is published only when every included motor has finite observations
younger than 100 ms for that quantity; otherwise that JointState vector is
empty. If neither position nor velocity is available, or a fault is latched,
no JointState is published. Diagnostics still report missing/old fields and
the fault before the existing shutdown path closes the driver.

These timestamps record when the host observed matching PDO contents. The
bridge may retain CAN frames, so they do **not** prove fresh physical CAN
reception. Hardware diagnostics explicitly report
`timestamp_basis=host_pdo_observation`, `can_freshness_verified=false` and WARN
for this unresolved boundary. EtherCAT WKC, exchange/failure counts and maximum
native loop gap are observed host metrics; they are not CAN utilization or
per-bus error counters. The simulator identifies itself and leaves unavailable
transport counters unset. Current, temperature and each field's validity are
reported separately.

### ENCOS vendor evidence

The original vendor materials were checked on 2026-11-15: `ENCOS+调试技术手册+V1.19EAP.pdf`,
`EtherCat-CAN转接板说明文档.pdf`, `EtherCAT数据传输字典.xlsx`, `SSC-IO.xml`,
and the SOEM C/C++ and IgH example archives. Vendor files were not modified.

- Motor manual sections 10.1-10.3 define position, velocity, phase current,
  motor temperature and error feedback; type 1 also defines MOS temperature.
  Types 2 and 3 carry position in degrees and velocity in rpm respectively.
  The documented feedback frames do not include a sample timestamp or receive
  sequence. CAN command timeout protection (section 9.2.10) is a separate
  motor-side function, not a host-visible feedback-age measurement.
- The bridge dictionary (`Sheet1`, rows 3-70) describes motor count, CAN ID
  format, and six records containing ID, RTR, DLC and eight data bytes. It
  specifies no per-motor receive timestamp, sequence or new-frame flag.
- The ESI maps this 86-byte motor payload to TxPDO `0x1a00`. A separate
  6-byte TxPDO `0x1a02`, named `AI Inputs`, contains `0x1802:07` TxPDO State
  and `0x1802:09` TxPDO Toggle, plus analog/demo fields including `mytest`.
  For this exact ESI, State and Toggle occupy byte 87 bits 6 and 7 of the
  input image (zero-based), i.e. `encos_bridge_inputs_t.status[1]`.
  Their names alone do not establish that either tracks new CAN replies,
  much less replies from each of the six motors.
- SOEM `app/transmit.c:EtherCAT_Data_Get()` copies the input image and calls
  `RV_can_data_repack()`. The decoder skips zero-DLC slots but has no receive
  sequence/timestamp check. IgH `main.c` maps the additional PDO, while
  `ethercat_data_get()` reads only the motor records. Neither example
  demonstrates CAN freshness using the status/toggle bits. The supplied
  bridge documentation does not say whether a slot's DLC is cleared when
  no new reply arrives; that behavior also remains unverified.

The [ENCOS product page](https://www.encos.cn/productinfo/55395.html?templateId=6217)
confirms transparent transport, two CAN buses, six motors and SOEM/IgH host
examples; it does not document receive-freshness metadata.
[Beckhoff's description of the same object names](https://infosys.beckhoff.com/content/1033/el3255/1784398347.html)
ties the toggle to the corresponding input-data update on its own terminal.
That is not evidence of the ENCOS bridge firmware's CAN receive behavior.

Consequently basic ENCOS telemetry is supported, but per-motor CAN freshness
through this bridge is **unverified**, not established as unsupported. Resolve
the boundary with firmware-version-specific vendor semantics or bridge CAN RX
source, then qualify each motor independently while other motors keep replying.
Required evidence includes behavior for identical successive replies, missing
replies, cache retention and restart. Do not promote the global toggle, WKC,
payload changes or motor command timeout to per-motor freshness without that
evidence. No physical test or firmware change was performed for this review.

### ENCOS offline capture analysis

Use the [paired CAN and EtherCAT capture procedure](CAN2_BENCH.md#combined-encos-commissioning)
to record per-motor reply gaps, repeated identical replies, and the PDO status
and DLCs during independently observed CAN silence. The tool reads saved files.
Retain the bridge revision and supplier RX semantics with the captures before
promoting CAN freshness in the runtime diagnostics.

## Viewing and deployment

With the existing driver owners already running and the ROS environment sourced:

```bash
ros2 topic echo /diagnostics
ros2 topic echo /arm/joint_states
ros2 topic echo /joint_states
```

Do not launch another driver just to view telemetry. The normal hardware opt-in,
profile confirmation, explicit recovery and command deadlines still apply.
This change requires rebuilding the ENCOS host library and `motor_control` ROS
package. It requires no Damiao firmware change when cached telemetry is already
supported. Runtime profiles and production-confirmation flags are unchanged.

## Offline validation and remaining qualification

`test_motor_telemetry.py` checks stale/missing/nonfinite data, per-field ages,
receipt-time accounting, generation replay/wrap, units and publication policy.
`test_native_driver.py` exercises the real ENCOS C thread with fake SOEM,
including query-only snapshots, latched transport faults and command expiry
while telemetry continues. Existing Damiao native-backed tests verify that
snapshot/health reads add no CAN traffic and do not renew command leases.
`test_ros_motor_telemetry.py` runs both ROS drivers with simulated/fake owners,
checks both diagnostic namespaces, stale-state suppression and command callback
latency. `test_ros_chassis.py` covers full and explicitly omitted-wheel profiles.
See [test commands](../tests/README.md#motor-telemetry-for-both-families).

The 25/50 Hz rates and 2 Hz diagnostics are software defaults. BASE-04 was
[resolved by operator confirmation on Nov 23](ARM_SESSION_HANDOFF.md#plan-01-and-base-04-resolution).
ENCOS-specific freshness remains in COMMS-01; fault response, protection and
loaded stopping remain under BASE-07, PROTECT-01 and SUPPORT-01. Production
ROS lifecycle and coordinated holding remain ROS-02.
