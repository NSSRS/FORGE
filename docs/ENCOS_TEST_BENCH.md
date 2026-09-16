# ENCOS test bench: NUC, Ubuntu, Windows and EtherCAT-CAN bridge

This guide uses the ENCOS EtherCAT-CAN board identified in the conversation. Start with one bridge and one motor, with no robot link or external load attached. No URDF is needed yet. The procedure has now been exercised on the Ubuntu NUC with three secured, unloaded motors: query-only telemetry, isolated and simultaneous bounded position tests, native Python velocity control, automatic ROS command/feedback, and dead-man keyboard control all passed. See the [commissioning record](#commissioning-record-2026-09-16) for dated results and remaining limitations, and [ROS 2 motor control](ROS2_MOTOR_CONTROL_STATUS.md) for build/run instructions. Motor models, brake behavior, bridge firmware, loaded stopping, and host-loss behavior still require verification before robot use.

## 1. Assign each device a job

| Item | Job | Needed for first bench? |
|---|---|---|
| Intel NUC, native Ubuntu 24.04 | Runs the EtherCAT master and motor control program | Yes |
| NUC wired Ethernet | Dedicated direct connection to the bridge | Yes |
| Wi-Fi or separate network port | Internet, downloads, optional remote access | Useful |
| ENCOS EtherCAT-CAN bridge | Exchanges CAN packets with the motors | Yes |
| Regulated 5 V supply | Powers bridge; provision at least 500 mA per board | Yes |
| Correct motor supply | Powers motor electronics and motor phases | Yes, after board-only test |
| Windows partition | Stores vendor tools and optionally runs supported commissioning tools | Optional |
| USB-CAN / CANalyst-II | Alternative communication path | No |
| ROS 2 / MoveIt / URDF | Later robot integration | No for first motor |
| ST-LINK | Vendor-directed bridge firmware service | No for normal operation |

Dual boot means Windows and Ubuntu are alternatives on this NUC, not simultaneously running controllers. Use Ubuntu directly for the bench; this procedure does not use WSL or a virtual machine.

## 2. Prepare the bench and record the equipment

Obtain a multimeter, insulated mounting standoffs, the manufacturer-supplied Ethernet adapter cable, the correct CAN harness, suitable termination, and a rigid motor fixture. Fit correctly rated power wiring, fuse and a reachable DC-rated disconnect/stop arrangement. A qualified bench builder should size the power protection for the selected motor and supply; a software stop is not a substitute.

Fill these in before applying motor power:

```text
Motor model / serial:
Motor count for initial test: 1
Holding brake fitted? Supported release sequence:
Motor nominal voltage / continuous and peak limits:
Chosen supply voltage / current capacity:
Regenerative energy handling:
Bridge revision / firmware:
5 V supply rating:
NUC Ethernet interface / driver:
CAN baud rate: classic CAN 1 Mbps, verify actual configuration
Motor CAN ID: discover before assuming 1
Permitted first-test phase current:
Permitted output travel / speed:
```

The family manual says 20-57 V operating range, but that is not a model-specific supply prescription. Regenerated energy needs a suitable return/absorption path; an ordinary bench PSU may not sink current. Motor phase current and PSU current are different. The board receives 5 V, never the motor supply voltage.

Mount the motor by its intended mounting features. Keep its output clear, remove loose couplings/tools, and add a visible reference mark. Do not use a long lever arm for the first test. Do not attach the full robot until the fault and shutdown behavior is understood.

## 3. Wire everything with power off

```text
NUC Ethernet
   |
   | supplied Ethernet-to-board harness
   v
Bridge EtherCAT IN       EtherCAT OUT: unused for one board
   |
   +-- CAN1 H/L ---------------- motor CAN H/L
   |
   +-- CAN2: unused initially

5 V supply -------------------- bridge power input
Motor supply -- protection ---- motor power connector
```

In the upper diagram on manual printed p. 45: left connectors are 5 V/GND; right upper is Ethernet IN, right lower OUT; top row is CAN1, bottom row CAN2. Locate the labels on the actual board rather than relying on its orientation. Use the supplied pinout/harness, not inferred wire colors. The CAN connectors marked L H H L contain repeated CAN signals, not motor supply pins.

The board has TWO CAN buses, not eight: the multiple connectors on each row share a bus. Its six software slots are passages 1-3 on CAN1 and 4-6 on CAN2. Begin with passage 1 and one motor on CAN1. A passage is not the same thing as the motor's CAN ID or a physical socket number.

Check the built-in termination on the bridge revision. Each used CAN bus needs one 120-ohm termination at each physical end. For one motor, normally the bridge supplies one and the motor end needs the other. With power off, a conventional two-terminator bus reads about 60 ohms H-to-L; unexpected resistance must be investigated. Never measure resistance on an energized bus. Follow the supplied grounding/isolation instructions; do not invent additional ground connections.

Use one documented bridge power source at a time; do not connect USB power and external 5 V together unless ENCOS confirms power sharing is supported. USB on the bridge is documented for power/debugging, not as the ordinary host CAN interface. Leave programming ports alone.

## 4. Organize the vendor files on Windows and Ubuntu

Keep an untouched copy of the original manual and the entire vendor folder. Transfer a copy through a USB drive or normal file transfer to Ubuntu; use the Ubuntu home filesystem for builds.

| Supplied material | How we use it |
|---|---|
| ENCOS V1.19EAP PDF | Motor command units, responses, startup and limits |
| EtherCat-CAN bridge PDF | Wiring, power and passage mapping |
| EtherCAT data dictionary XLSX and SSC-IO.xml | Reference for validating the bridge's process-data layout |
| soem_demo_c.zip | Initial C master source and bundled SOEM library |
| soem_demo_cpp.zip | Optional alternative; do not mix versions during first bring-up |
| IgH example ZIPs | Alternative master, defer initially |
| MotorTool executable | Optional Windows commissioning with its supported adapter |
| VESC tool | Optional vendor-supported motor configuration |
| ST-LINK packages | Firmware maintenance only when required by ENCOS |
| STEP/CAD files | Mounting/clearance reference |
| Videos | Supplementary demonstrations, not verified startup scripts |

MotorTool is not assumed to work through the EtherCAT bridge. Do not buy USB-CAN just to satisfy a Windows step: the intended commissioning route can be implemented through EtherCAT. Preserve factory calibration and firmware. Install Windows drivers only for hardware actually connected and documented as supported.

## 5. Ubuntu workspace preparation

This workspace is now on the Ubuntu 24.04 NUC at `/home/forge2/forge2_ws`.
Original vendor files are preserved in `vendor/encos/`; the C example has already been extracted into `src/soem_demo_c/`. Do not extract over the working copy.

Install dependencies using the [workspace build guide](ENCOS_WORKSPACE.md#build-on-ubuntu),
then confirm the host and available interfaces:

```bash
cd /home/forge2/forge2_ws
uname -a
cat /etc/os-release
ip -br link
```

The active build fetches pinned upstream SOEM v1.4.0; the bundled supplier copy
is reference material. SOEM is a userspace EtherCAT master; installing the IgH
kernel master is not part of this path. See the [workspace layout](ENCOS_WORKSPACE.md#layout)
and `docs/bench-equipment.md` for the local equipment record.

## 6. Dedicate the Ethernet interface

Identify the wired port from `ip -br link`, then inspect it. Replace the example name below with YOUR actual interface; `enp3s0` is a placeholder:

```bash
export ENCOS_IFACE=enp3s0
sudo ethtool -i "$ENCOS_IFACE"
sudo ethtool "$ENCOS_IFACE"
```

Disconnect that port from office/home networking and connect it directly to bridge IN. Keep internet on Wi-Fi or a separate interface. EtherCAT uses Ethernet frames, not an IP connection: do not assign an IP to the board or expect ping to discover it. Do not insert a router or ordinary network switch.

If NetworkManager manages the dedicated port, temporarily release only that port, from the local NUC console:

```bash
sudo nmcli device set "$ENCOS_IFACE" managed no
sudo ip link set dev "$ENCOS_IFACE" up
```

If `nmcli` is not present, inspect the active network manager instead; do not change unrelated interfaces. To return the port to NetworkManager after testing:

```bash
sudo nmcli device set "$ENCOS_IFACE" managed yes
```

Keep the NUC awake during tests; disable automatic suspend in Ubuntu Power settings. For first testing, no real-time kernel is required. Measure timing before selecting higher rates or changing kernel/scheduler configuration.

## 7. Build the project-owned C utilities without running hardware

Follow [Build on Ubuntu](ENCOS_WORKSPACE.md#build-on-ubuntu). The build produces
`encos_query`, `encos_assign_id`, `encos_motion`, and `libencos_driver.so` under
`build/encos_query/`, then runs the hardware-free tests. The interface is supplied
at runtime; no supplier source edits are needed.

The [historical supplier-demo inspection](ENCOS_BRINGUP.md#supplier-soem-c-demo-inspection-historical)
preserves its build target, hardcoded interface, and known defects.

## 8. Perform the board-only test

Keep MOTOR POWER OFF and the motor CAN harness disconnected. Connect NUC Ethernet
to bridge IN, power the bridge at 5 V, and check the link with
`sudo ethtool "$ENCOS_IFACE"`. From the workspace, run:

```bash
sudo ./build/encos_query/encos_query "$ENCOS_IFACE"
```

The current query-only utility validates one slave and the 86-byte output /
92-byte input PDO layout. With no motor connected, expect the bridge-operational
message followed by a motor-ID query timeout and a nonzero exit. That timeout
does not indicate a failed bridge test. It sends no motion commands and clears
outputs before exiting; it is not a sustained one-minute bridge soak test.

Pass for this initial check: one bridge reaches OP with the expected working
counter and correct PDO sizes. LEDs do not prove motor communication. Sustained
link/process-data stability still needs verification. Power down before
connecting motor wiring.

## 9. Prepare the powered-motor diagnostic program

The supplied vendor example is not a finished bench controller; do not simply
uncomment its 180-degree movement or zero-setting examples. Project-owned
utilities now provide query-only telemetry (`encos_query`), bounded position
motion (`encos_motion`), and guarded Python/ROS velocity control. The checklist
below remains the acceptance standard rather than a description of missing
software.

The adapted utility must have:

1. A default DISARMED state and explicit separate query/arm/move/stop actions.
2. Verified slave count and per-slave input/output sizes before copying process data; exact board PDO layout matched to firmware and vendor dictionary.
3. One configured passage and motor ID, with unused slots explicitly inactive according to bridge protocol.
4. A query-only path containing ENCOS query packets, not accidental zero-filled hybrid commands. Confirm how the bridge retransmits stored commands and how a slot becomes inactive.
5. Checked receive ID/DLC/type, signed current, float endianness, fault codes and timestamps.
6. A monotonic periodic loop, initially 100 Hz for a single motor, with the bridge watchdog requirements checked separately. If bridge cyclic timing must be faster, schedule motor commands deliberately within that loop.
7. Target initialized from fresh measured angle; finite-value checks, travel/speed/current limits and explicit feedback-age limits.
8. Latched disarm on bad working counter, stale motor data or motor fault; no automatic resumption of motion on reconnection.
9. A model-appropriate controlled stop and brake sequence; a hardware stop independent of Linux.
10. CSV logging outside the critical loop, including raw frames and evidence of fresh CAN reception where provided by the bridge.

The current implementation covers guarded entry, PDO-size validation, explicit
IDs and slots, checked protocol decoding, measured-position initialization,
bounded commands, feedback/current/temperature checks, command deadlines, and
latched fault handling. Still unresolved are authoritative bridge receive-age
metadata, the model-specific brake/disarm sequence, independent hardware-stop
behavior, and complete structured timing/raw-frame logs.

Important: valid EtherCAT working counters only prove exchange with the bridge. They do not prove that its cached motor feedback is new. Determine whether bridge firmware supplies receive counters, timestamps or validity flags; otherwise an unchanged cached position can be mistaken for a healthy stationary motor.

Important: determine what the bridge does when the NUC or EtherCAT connection disappears. If it keeps replaying the last CAN control frame, the motor's own CAN timeout may never expire. Test the bridge's watchdog/output-invalidation behavior with the motor secured and a hardware stop available; do not assume the motor's 500 ms watchdog covers host failure.

## 10. Power one motor and establish telemetry

With the project-owned utilities built and the equipment sheet complete:

1. With all power off, connect the one motor to CAN1 and verify power polarity and termination.
2. Boot Ubuntu and verify the motor application is disarmed, with no background master running.
3. Power the bridge, then the correctly configured motor supply. Wait at least 5 seconds for the motor to boot. Keep the mechanism secured; do not assume a brake has released.
4. Through the query-only EtherCAT path, discover the motor ID. Use standard CAN ID `0x7FF`, DLC 4, payload `FF FF 00 82`; this is a CAN packet inside bridge process data, not a `cansend` command on the NUC.
5. On the discovered motor ID, query position (`E0 01`), versions (`E0 1E`) and timeout (`E0 1F`). Record replies. The manual also illustrates `E1 01`; validate the reserved-bit convention against firmware as described in ENCOS_BRINGUP.md.
6. Query configured hybrid ranges before any future hybrid use; retain factory gains for now.
7. Record no-motion angle for about 30 seconds, checking faults and actual fresh responses. Do not reset zero simply to make the displayed angle look convenient.

Pass: correct motor identified, plausible angle, repeatable fresh telemetry and no faults. If motion/brake/temperature state cannot be interpreted, resolve it before arming.

ID changes and zero/communication-mode configuration need over 500 ms between transmitted CAN commands per the manual. This includes bridge-generated retransmissions: do not repeat a configuration frame every EtherCAT cycle. Configure one motor at a time. Preserve the initial ID unless there is a reason to change it.

## 11. First controlled movement

Use servo position mode 1 with feedback type 2. Choose the phase-current ceiling from the actual motor and fixture; no universal A setting is appropriate without the model. A low PSU current limit alone is not a substitute for the motor's phase-current ceiling.

1. Read a fresh initial angle q0.
2. Set the target to q0, not zero degrees. Verify the holding-brake sequence if a brake is fitted; support any load before release. Firmware-specific brake commands are described in the manual; do not test both variants blindly.
3. Explicitly arm the controller with bounded output limits.
4. For a verified unloaded, unobstructed shaft, an illustrative first trajectory is q0 to q0 + 1 degree over 2 seconds, hold briefly, then return over 2 seconds. Use only if this travel is clear for the actual fixture.
5. Use a small speed ceiling compatible with the planned trajectory; e.g. 1 rpm is 6 degrees/second, while this trajectory asks about 0.5 degree/second. Current must be high enough for controlled motion but within the approved bench limit. Do not increase it blindly if the shaft does not move; check brake, faults and units first.
6. Compare actual movement direction, amplitude and returned degrees to the reference mark. Stop on unexpected movement, rising current, vibration, faults or lost feedback.
7. Repeat a small negative excursion only after the first succeeds.

Servo speed/current ceilings are encoded in 0.1 rpm / 0.1 A units. Type-2 feedback current uses 0.01 A units. Its angle is output-shaft degrees. ENCOS_BRINGUP.md contains the exact packet layout.

Pass: bounded motion follows the target, stops and returns predictably, with current and feedback within configured limits. Encoder resolution is not a guarantee of angular accuracy.

## 12. Stop behavior and fault tests

First establish a deliberate normal stop: decelerate to a stationary target, confirm near-zero speed, then use the documented brake/disarm sequence for this motor. If no brake exists, removing torque does not hold a load. After the mechanism is secure, turn off motor power, wait for indicators/discharge, then remove bridge power if desired. Rewire only when unpowered.

With a secured unloaded setup, exercise one fault at a time: controller stop, host process termination, EtherCAT link loss and loss of CAN feedback. CAN wiring changes must follow the vendor prohibition on hot-plugging; simulate missing replies in software first, and alter physical CAN wiring with power off. Check both motor and bridge watchdogs and confirm that reconnection does not restart motion. Record whether the shaft coasts, brakes or holds and the measured response time.

Neither 'ESTOP' console text, a healthy EtherCAT working counter, nor the default CAN timeout alone establishes a safe robot stop.

## 13. Logging and expanding the bench

Log run ID, software version, motor/bridge firmware, bus/passage/ID, limits, monotonic time, commanded/measured angle, velocity, phase current, temperatures, motor errors, EtherCAT state/working counter, confirmed motor feedback age, cycle period and missed deadlines. Separate measured values from estimates. Include winding and MOS temperatures when the selected reply provides them; use additional queries/reply scheduling where necessary.

Once one motor passes, add a second only with power off and a unique ID on that bus. Map every motor to (bridge index, passage, CAN ID). Each bridge supports passages 1-3 on CAN1 and 4-6 on CAN2. One board is sufficient for up to six motor slots, but throughput still needs measurement. Start conservatively and preserve timing headroom.

A temporary ROS 2 Jazzy commissioning layer now publishes direct motor RPM
commands and `/joint_states`; it is not yet a chassis controller or a
`ros2_control` hardware interface. Add the production `ros2_control` interface,
joint-state/trajectory controllers, and URDF/YAML only after the chassis mapping
and safety requirements are defined. Those files must include joint axes,
origins, signs, zero references, travel/velocity/effort limits, and external
transmissions. Dynamic feedforward also needs mass, center of mass, inertia, and
payload. Evaluate hybrid mode only when the task requires it.

## Troubleshooting order

| Symptom | First checks |
|---|---|
| No Ethernet link | 5 V supply, correct IN harness, NIC up, cable continuity |
| Cannot open EtherCAT socket | Correct interface, local sudo permission, another master using NIC |
| No slave found | Dedicated direct cable, bridge IN rather than OUT, board power |
| Slave found but not OP | AL status code, PDO sizes/mapping, bridge firmware; do not blame motor ID yet |
| OP but no motor response | Motor power and boot delay, CAN H/L, termination, CAN1 passage, ID/baud, enabled transmit slot |
| Feedback looks frozen | Confirm new CAN reception instead of repeatedly reading bridge cache |
| Position wrong by about 57.3 | Degrees/radians conversion |
| Position wrong by gear ratio | Internal gearbox mistakenly applied to output-shaft feedback |
| Motor energized but no movement | Brake status, faults, current ceiling and mechanical obstruction |
| Works, then resets on braking | Supply/regeneration handling and voltage logs |
| ESTOP printed but loop continues | Known vendor demo behavior; requires actual latched stop implementation |

Sources: supplied ENCOS V1.19EAP, bridge instructions, and inspected soem_demo_c.zip CMakeLists.txt, app/main.c and app/transmit.c. Upstream reference: [SOEM project](https://github.com/OpenEtherCATsociety/SOEM). Original protocol analysis: ENCOS_BRINGUP.md. The workbook, videos and Windows executables are listed as reference materials; this guide does not claim to have executed the binaries, watched the videos or verified the workbook against physical board firmware.

## Commissioning record (2026-09-16)

### First ROS hardware run (2026-09-16)

The driver reached hardware-ready state on `enp86s0` for IDs 1, 2, and 3. When
the command path became active, the motors shook and the native driver latched
fault class 3 (`feedback/limit failure`). The session closed without automatic
re-arm. At that point, powered testing was paused pending read-only telemetry and
identification of the specific motor/error/current/feedback condition.

The next read-only query returned fault-free position, hardware, version, and
500 ms timeout replies from all three motors. Positions were -151.493652 degrees
(ID 1), 69.675499 degrees (ID 2), and 2.895467 degrees (ID 3). Brake replies were
unknown. All three reported software 1.0.0, which conflicts with the first
preflight's 1.0.8 and 1.0.32 values for IDs 1 and 3. The discrepancy may indicate
cached or otherwise unverified bridge replies and must not be treated as proof
of firmware identity.

Motors 1, 2, and 3 then each passed the native isolated motion test
independently: +1 degree over two seconds, return over two seconds, a 2 A
phase-current ceiling, and final position near the measured start. This
establishes individual low-speed position control but does not yet clear
simultaneous three-motor velocity control.

All three motors subsequently passed the same +1 degree / return test
simultaneously with a 2 A phase-current ceiling per motor. At that stage, ROS DDS
was forced to loopback with `ROS_LOCALHOST_ONLY=1` so discovery and topic traffic
did not use the dedicated EtherCAT interface. The native driver was also updated
to print the exact motor and measured condition before latching a fault.

The following ROS keyboard run reported the exact cause: motor 2 reached
720.049 degrees, outside the configured [-720, 720] degree commissioning range.
This was a software position-limit stop, not a reported current, temperature,
EtherCAT, or motor fault. Output-shaft position accumulates across turns while
the motor remains powered, so continuous wheel velocity control requires a
continuous-joint policy instead of the bounded-position policy used for initial
bench motion.

The driver now implements that policy: Mode 1 position control retains angular
bounds, while Mode 2 velocity control permits continuous accumulated output
rotation. Speed, phase-current, temperature, motor-error, feedback-age,
command-age, and EtherCAT protections remain enabled. ROS discovery is limited
to loopback with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` rather than the
deprecated `ROS_LOCALHOST_ONLY` variable.

Fast DDS may report the root driver's endpoints as `_NODE_NAME_UNKNOWN_`, so
`ros2 node list` is not a reliable liveness check for this temporary
commissioning arrangement. `scripts/ros2_env.sh` loads
`src/forge_motor_control/config/fastdds-udp-loopback.xml`, which restricts DDS to loopback and disables
data sharing, but this did not make mixed-account payload delivery reliable.

The guarded native Python test subsequently passed with motors 1, 2, and 3 at a
+30 RPM target for two seconds and a 2 A phase-current ceiling per motor. It
reported all three below 0.5 RPM during the stop window. The automatic ROS test
then passed at +5 RPM for two seconds with fresh `/joint_states` feedback and a
verified reported stop. Root-to-root keyboard commissioning also worked at
both +5 RPM and -5 RPM with dead-man stopping. These results validate the
current unloaded bench path; they do not establish loaded robot stopping or
host-loss behavior.

### Confirmed bench values (2026-09-16)

| Item | Value |
|---|---|
| EtherCAT interface | `enp86s0`, confirmed as the direct bridge connection |
| CAN motor IDs | 1, 2, 3 |
| Initial bus | CAN1; the three configured Python slots map to CAN1 |
| Mechanical condition | Motors are secured and unloaded; operator approved roughly one output revolution for a bench test |
| Bench PSU | 5 A maximum supply current |
| Requested provisional command ceiling | 2 A phase-current ceiling per motor; this is not a 6 A aggregate supply-current claim |

### Query-only hardware result (2026-09-16)

The bridge reached OPERATIONAL with the required 86-byte output / 92-byte input
PDO layout and WKC 3. No motion command was sent.

| CAN ID | Position | Hardware | Software | CAN timeout |
|---:|---:|---:|---:|---:|
| 1 | 119.436943 deg | 1.0.1 | 1.0.8 | 500 ms |
| 2 | -19.084883 deg | 1.0.1 | 1.0.0 | 500 ms |
| 3 | -85.408943 deg | 1.0.1 | 1.0.32 | 500 ms |

The PSU current and a motor's phase current are different quantities. The 2 A
value is a provisional command ceiling, not a model-derived safe operating
limit.

### Powered-motion status

Holding-brake presence and its documented release sequence are unknown. A
holding brake is an electromagnetic mechanism that can prevent shaft rotation;
the ENCOS manual says it should be opened before motor control. On 2026-09-16,
the operator explicitly instructed the bench test to proceed without treating
brake status as a motion gate. Stop immediately if a motor does not rotate,
draws unexpected current, heats, vibrates, or reports a fault.

`scripts/run_three_motor_velocity.py` implements the completed bounded
three-motor check: a +30 RPM target for a two-second command window, a 2 A
phase-current ceiling per motor, explicit zero-speed commands, and shutdown. A
fixed 60 RPM/s native slew limits abrupt velocity steps; it is intentionally not
another ROS/Python tuning parameter. The driver retains feedback, command-age,
current, temperature, overspeed, and EtherCAT fault checks.
