# Damiao arm and ENCOS chassis

FORGE now has two separate motor paths:

- **Chassis:** existing ENCOS motors → ENCOS EtherCAT-CAN bridge → `encos_driver`
  and `motors.MotorBus`. Existing chassis profiles, IDs, units and commands stay valid.
- **Arm:** 1 × DM-J8009P-2EC V1.1, 2 × DM-J4340P-2EC V1.1,
  3 × DM-J4310P-2EC (user label V1.1) → DM-MC02 → USB CDC →
  `motors.arm.ArmBus` or the separate ROS `arm_driver`.

This implementation is built and tested without physical motor motion. It is not
flashed or commissioned. The user opened the STM32 USB port with the diagnostic
script, but the FORGE handshake timed out. Do not assume the board is already
running this project's USB protocol. See the dated handoff below.

## Verified manufacturer references

Retrieved 2026-10-30 from Damiao's `dmBots` repositories:

| Device | Manufacturer source | Finding |
| --- | --- | --- |
| J8009P V1.1 | [English user manual V1.0, 2026-09-10](https://github.com/dmBots/DM-J8009P-2EC/blob/42eaaf7f05cccba18d182b51504ee0349e16cff6/manual/DM-J8009P-2EC%20V1.1%20Gear%20Motor%20User%20Manual%20V1.0%202026-09-10.pdf) | Exact hardware-revision manual found; rated 20 Nm, peak 40 Nm |
| J4340P V1.1 | [English user manual V1.4, 2026-09-14](https://github.com/dmBots/DM-J4340P-2EC/blob/98a29a9df67c83512795f8bb9ca2bdd2423f66dd/manual/DM-J4340P-2EC%20V1.1%20Geared%20Motor%20User%20Manual%20V1.4%202026-09-14.pdf) | Exact revision found; rated 12 Nm, peak 40 Nm; older 9/27 Nm data does not apply |
| J4310P | [English user manual V1.3, 2026-09-14](https://github.com/dmBots/DM-J4310P-2EC/blob/9e728c9a42b498380ae43964636b3d66d01f42c7/manual/DM-J4310P-2EC%20Geared%20Motor%20User%20Manual%20V1.3%202026-09-14.pdf) | Rated 3.5 Nm, peak 12.5 Nm; no separately titled hardware V1.1 manual found |
| MC02 | [Manufacturer repository at pinned revision](https://github.com/dmBots/DM-MC02/tree/d65a3688981405dc205f628f1a026bde8cf8303e) | `manual/` contains board manual V1.1; USB/CAN examples supply STM32H723 support |

The motor manuals' CAN communication and register-map chapters define the
codec. In the J4340P manual these are printed pages 13–21; timeout units are
explained in the configuration section. CAN is standard 11-bit classic CAN,
nominally 1 Mbps. MIT packets carry p16/v12/kp12/kd12/t12; gains map to
0–500 and 0–5. Position/velocity/torque ranges are configurable, not inferred
from rated or peak torque. Feedback has a status nibble, position, velocity,
torque, MOS temperature and rotor temperature. MIT mode is CTRL_MODE=1.

Firmware reads PMAX/VMAX/TMAX (registers 21/22/23), CTRL_MODE (10), and TIMEOUT
(9). It does not write motor parameters, clear faults, change IDs, save zeros,
calibrate encoders or update motor firmware. TIMEOUT is 50 microseconds/count;
a value in 1–3000 is required (up to 150 ms). Configure that with the vendor
tool before commissioning. The driver limits command IDs to 1–15 so the ID fits
the feedback nibble, and requires distinct feedback IDs 16–2046.

## Hardware mapping and power

The user has selected these wiring plans:

| Profile in `src/robot_description/config/` | CAN1 | CAN2 | CAN3 |
| --- | --- | --- | --- |
| `arm_motors.json` (current temporary plan) | 8009P alone | Two 4340Ps + three 4310Ps, daisy-chained | Unused pending cable |
| `arm_motors_final.json` (final plan) | 8009P alone | Two 4340Ps, daisy-chained | Three 4310Ps, daisy-chained |

Both profiles target classic 11-bit CAN at 1 Mbps and **500 Hz per enabled
motor**, including the single motor on CAN1. The final profile changes only
the three 4310P bus assignments; no firmware change is needed to use it.
The host rejects unsupported bitrate, frame-format and update-rate fields;
changing JSON does not reconfigure the CAN peripheral or activate CAN-FD.

These are confirmed wiring plans, not measured hardware maps. Proposed command
IDs remain 1–6 and feedback IDs 0x11–0x16; actual ID configuration is unverified.
Joint names/order are placeholders. All operating limits remain unset and
`hardware_tested` is false. `hardware_confirmed` and `joint_order_confirmed`
must both be true before the host accepts an operational profile. Set limits
in **raw motor output-shaft coordinates**: radians, radians/second, Nm, Kp, Kd.
No automatic sign/zero transformation is implemented. Direction and offset
fields are reserved commissioning records, not applied transformations.

### Connector routing

Motor manuals, printed page 10: the 8009P V1.1 has a separate XT30 power
socket, one two-pin H/L socket and a G/R/T UART debugging socket. It has no
second CAN socket for direct pass-through. Both smaller P-series models have
two XT30(2+2) sockets combining power and CAN, supporting daisy-chaining.
G/R/T is GND/RX/TX, not another CAN connector. Electrically, CAN devices share
H-to-H and L-to-L; the selected arrangement avoids an external branch harness
for the 8009P.

The MC02 manual (printed pages 4–5, 12–13 and appendix on page 15) identifies
three independent CAN channels. CAN1 and CAN2 have two-pin H/L connectors.
CAN3's four-pin connector is 1=5V, 2=GND, 3=H, 4=L; its 5V is not motor power.
With the main power connector at the top, CAN3 is on the left, below CAN2 and
above UART10. The two switched XT30(2+2) outputs carry CAN1 and CAN2, not two
additional channels. They are rated 5 A each; size motor power separately.
Use twisted H/L, short branches, appropriate ground references and termination
at the two physical ends of each bus, accounting for built-in termination.

### Calculated CAN budget (not measured)

Assume one standard eight-byte command and one eight-byte request-response
feedback frame per motor per update, with 111–135 wire bits per frame,
including framing/inter-frame spacing and variable bit stuffing:

`load = motors × 2 × update_hz × bits_per_frame / 1,000,000`

| Motors on one bus | Update rate per motor | Estimated load |
| --- | --- | --- |
| 5 | 500 Hz | 55.5–67.5% |
| 5 | 1 kHz | 111–135%: exceeds capacity |
| 3 | 500 Hz | 33.3–40.5% |
| 3 | 1 kHz | 66.6–81% |

These estimates exclude diagnostic traffic and retransmissions. The firmware
does not allow enabling while any joint is still probing registers. The MCU
queues one update per enabled joint every 2 ms without catch-up bursts after a
late tick; CAN arbitration serializes the frames. Actual achieved rates and
jitter still need measurement with the connected motors. CAN-FD support in the
hardware does not mean it has been configured or tested here.

`chassis_motors.json` classifies the original ENCOS motors as chassis motors.
It records the existing three-motor bench: IDs 1/2 on CAN1, ID3 on CAN2, PDO
slots 0/1/3. The four-wheel mapping remains the existing chassis profile.
Damiao CAN IDs can repeat chassis IDs because they use a different controller.

MC02 pins from the vendor example:

| Logical port | RX | TX |
| --- | --- | --- |
| CAN1 | PD0 | PD1 |
| CAN2 | PB5 | PB6 |
| CAN3 | PD12 | PD13 |

The board manual specifies 12–24 V input (6S supported) and two switched
outputs rated at 5 A each. This is not a suitable assumption for distributing
all six motors' peak power. Verify motor voltage variants and supply them via
appropriately sized distribution. Firmware leaves the board's switched power
outputs and IMU heater untouched. USB powers CAN transceivers according to the
manual; verify the actual wiring, common reference and bus-end termination.

## Firmware build

The embedded code is C11 in `src/damiao_firmware/`. It runs on STM32H723VGT6,
uses a 2 ms control tick, polling FDCAN reception, a USB receive ring, and an
independent hardware watchdog. Each CAN bus has a separate message-RAM region.
The vendor USB example's PLL is **120 MHz at PLL1Q**, so the firmware uses
120 time quanta for 1 Mbps; copying the standalone CAN example's 80-tq timing
into the USB project would give the wrong bitrate.

Install Ubuntu build prerequisites if needed:

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
python3 scripts/fetch_mc02_sdk.py
python3 scripts/build_dm_mc02.py
```

The SDK fetch pins revision `d65a3688981405dc205f628f1a026bde8cf8303e` and checks
Git blob hashes. Only required vendor files are cached under ignored
`build/dm_mc02/sdk/`; vendor contents are unchanged. Build-time copies adapt
the example's main loop and CDC receive callback. ST source notices remain
in those copies. The fetch does not download motor firmware or flash hardware.

Outputs: `build/dm_mc02/forge_arm.elf`, `forge_arm.bin`, and `forge_arm.map`.
The linker targets internal flash at 0x08000000 and DTCM RAM. USB DMA and CPU
data cache are not enabled. A full SDK installation can use the default
compiler name; a relocated toolchain can be supplied via `--cc`.

Flashing is a separate commissioning step requiring an identified SWD probe or
confirmed bootloader procedure. No flash command is run automatically. Keep
the arm supported: disabling torque is **not** a mechanical brake or a
validated gravity hold. Firmware restart/host loss is designed to remove
commands; the motor's own configured timeout is essential if the MCU hangs.

## Host and ROS interfaces

Install the optional host dependency with `pip install -e '.[arm]'` or the
system `python3-serial` package. `ArmBus(port=...)` performs a version/rate handshake
only; the host rejects older firmware that does not report 500 Hz. It uses an exclusive serial port and never refreshes commands in a
background thread. Each command applies to one of six zero-based slots.

1. Load a confirmed profile with `configure(profile_path)`.
2. Call `probe(slot)` on each slot; poll `state(slot)` until `registers == 31`.
   Probing only reads registers. A missing bit means the range, mode, timeout
   or communication configuration is unsuitable.
3. Explicitly call `stop()` with the arm supported. It sends disable frames
   until each configured motor acknowledges disabled status. Obtain fresh
   position feedback from `state()`. There is no automatic motor-fault clear.
4. Within 100 ms of disabled feedback, call `command(slot, measured_position,
   velocity=0, kp=..., kd=..., torque=0)` then `enable(slot)`. Position must be
   within 0.05 rad of feedback. Prepare/enable each slot promptly; stale data
   requires another explicit disabled-feedback request.
5. Supply fresh bounded commands for every enabled slot, typically at 50 Hz.
   Firmware targets 500 Hz CAN transmission while their 150 ms leases remain valid.
   This is repeated transmission of fresh host targets, not 500 Hz host trajectory
   generation or USB feedback delivery.
6. `stop()` or `close()` requests torque disable. Verify status acknowledgements;
   USB ACK means accepted by the board, not mechanically stopped.

No operational gain/travel example is supplied because the fixture and arm
geometry have not been established. `torque_nm` bounds feedforward torque and
sets a measured-torque trip threshold (120%); it is not a hard clamp on the
combined Kp/Kd/torque controller output. Set motor-side current limits and
validate gains for the actual mechanism. Feedback position, speed, torque and
80°C temperature checks trip the whole arm. Timeout, CAN enqueue failure,
bus-off or USB overflow/disconnect likewise disable the arm and latch a fault.
A reset requires disabled acknowledgements and then fresh parameter probing.
There is no automatic restart after faults.

ROS build:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base logs/colcon build --base-paths src --build-base build/ros2 --install-base install/ros2 --symlink-install --packages-select robot_description motor_control bringup
source scripts/ros2_env.sh
ros2 launch bringup arm.launch.py execute:=true port:=/dev/serial/by-id/YOUR_MC02 profile:=/absolute/path/to/confirmed_arm.json
```

The launch defaults to refusing hardware execution. It does not participate in
`system_test.launch.py`, so existing chassis teleoperation does not enable the arm.

- `/arm/command`: `sensor_msgs/JointState`, exactly the configured six names,
  all position/velocity/effort arrays present, fresh increasing stamp (<100 ms).
  Units are raw shaft rad/rad/s/Nm; gains come from the confirmed profile.
- `/arm/joint_states`: fresh measured shaft state, distinct from chassis state.
  Samples are polled sequentially and checked for freshness; they are not a
  simultaneous six-axis hardware snapshot or a calibrated robot-frame estimate.
- `/arm_driver/enable`: `std_srvs/Trigger`, requires a fresh complete command
  and fresh disabled feedback. It never synthesizes a target or restarts a fault.
- `/arm_driver/disable`: explicit torque-disable request; also obtains disabled
  feedback during commissioning.
- `/arm_driver/reset`: reset board latch after disable acknowledgements and
  re-read all motor parameters. Does not clear motor error codes.

ROS message expiry is checked in the host; the independent MCU lease still
expires if ROS or USB stalls. This is a basic bounded command interface, not
MoveIt, trajectory planning, inverse kinematics or a gravity-compensated controller.

## Robot description boundaries

The new `robot_description` package installs both motor inventories and
`damiao_motor.xacro` / `encos_chassis.xacro` macros. Damiao visuals use documented
casing dimensions. Origins, axes and limits are required macro arguments.
No link lengths, collision geometry, inertias or full arm kinematic chain are
invented. Supply actual CAD/joint transforms to assemble a calibrated robot
URDF. Existing chassis MuJoCo visualization remains unchanged.

## Tests and commissioning still needed

`bash scripts/build.sh` now includes the portable arm C tests. Source
`scripts/ros2_env.sh` first for the existing ROS integration tests; set
`ROS_DOMAIN_ID` to an isolated test domain. CTest now preserves the sourced
Python paths instead of hiding ROS modules. Targeted checks:

```bash
cmake -S src/damiao_firmware -B build/damiao_tests
cmake --build build/damiao_tests
ctest --test-dir build/damiao_tests --output-on-failure
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_damiao_arm.py' -v
```

Tests include a Python-to-real-C-firmware round trip through fake CAN, all six
motors, USB CRC/fragmentation/replay rejection, invalid limits, feedback faults,
command expiry, timer rollover and fault reset. These do not establish electrical
wiring, physical CAN timing, actual USB enumeration, mechanical limits, loaded
stopping, or MCU reset behavior on this board. A board flash, register discovery,
and supported/unloaded commissioning remain necessary.

Before that step, identify the connected port, CAN wiring, actual motor IDs and
feedback IDs, motor firmware/hardware/voltage variants, joint order and fixture.
The J4310P manual V1.3 is a document revision, not proof of hardware V1.1.
No non-P J4310 manual is substituted.

## Bench handoff — 2026-10-30

An ST-LINK/V2 has been purchased and is awaiting delivery. No board flash,
motor enable, test spin, or physical CAN timing measurement has been completed.
Development is on the `nathan` branch; `main` is left unchanged.

Actual bench wiring currently differs from the six-motor plans above:

- One J4340P on MC02 CAN2 through the GH1.25 H/L connector.
- Motor powered independently by a 24 V PSU through XT30.
- MC02 powered/connected through USB-C; no SWD probe connected yet.

The user ran `python3 scripts/check_dm_mc02.py --port /dev/ttyACM0` with and
without the USB dongle. Both attempts enumerated an STM32 Virtual COM Port and
ended with `MC02 response timeout; firmware command leases are not refreshed`.
`lsusb -d 0483:` showed `0483:5740`, the virtual COM device, not the USB DFU
bootloader. Removing the dongle did not resolve the handshake. This confirms
USB enumeration and port opening, but does not identify the installed firmware.
The script sends identity/status requests only; neither attempt enabled a motor.

The reported beep from a button is consistent with the manufacturer's ALL demo:
its USER-button task toggles the buzzer. That observation is not a firmware
version check. The board has separate USER and RESET buttons. The manual does
not prescribe a first-use activation or a button sequence to enable our USB
protocol. No BOOT0 pads have been bridged; that is not part of the selected SWD
procedure.

### When the ST-LINK/V2 arrives

The MC02 manual V1.1, printed page 7, documents ST-Link/J-Link SWD programming.
A genuine ST-LINK/V2 supports STM32 SWD. Small third-party sticks sold under
that name can have different pin layouts: use signal labels, not a guessed
connector orientation.

1. Turn off the motor PSU and initially disconnect motor CAN2. Connect wiring
   with power removed. Keep this first step limited to the controller.
2. Connect the probe to the MC02 **SWD** connector using the table below.
   Power the MC02 through USB-C and connect the ST-LINK to the PC by USB.
3. In STM32CubeProgrammer, select ST-LINK, SWD, initially 400 kHz, then Connect.
   Confirm identification as STM32H723 and a target reference near 3.3 V.
4. If readable, save the existing internal flash from `0x08000000`, length
   `0x100000` (1 MiB), as a local backup. Do not change read protection or option
   bytes just to force a backup. Keep device backups out of Git.
5. Build with the commands above. Program `build/dm_mc02/forge_arm.elf`, enable
   verification, and reset the board after programming. The ELF contains its
   load addresses; a raw `.bin` instead requires start address `0x08000000`.
6. Run the diagnostic below. Require a successful FORGE version/rate handshake
   reporting 500 Hz before moving on to CAN commissioning.

| MC02 SWD pin | Signal | Probe connection |
| --- | --- | --- |
| 1 | VCC 3.3 V | Genuine probe target-voltage reference (VTref) |
| 2 | GND | GND |
| 3 | SWCLK / PA14 | SWCLK |
| 4 | SWDIO / PA13 | SWDIO |

A clone's pin labelled `3.3V` may be a **power output**, not VTref. Do not connect
that output to the already USB-powered board. Verify the actual probe pinout;
connect SWDIO, SWCLK and common ground, with voltage-reference wiring according
to that probe's documentation. The MC02 four-pin SWD connector has no NRST pin.

```bash
python3 scripts/check_dm_mc02.py --port /dev/ttyACM0
```

Use the actual enumerated port if its name changes. This script is the next
script to run after flashing; it does not spin the motor. Flashing replaces
MC02 controller firmware, not firmware inside the Damiao motor.

### Remaining motor commissioning

Before a first spin, establish the single 4340P's actual command and feedback
IDs, MIT mode, PMAX/VMAX/TMAX, timeout, current limits, secure unloaded mounting,
permitted travel and gains. Check CAN termination and ground reference. The
checked-in profiles deliberately have unset operating limits and unconfirmed
IDs/order; do not set confirmation flags merely to bypass validation.

The current public `ArmBus.configure()` and ROS command interface require a
complete six-joint profile. A dedicated single-motor motion runner is **not yet
provided**; do not use the six-joint launch as a one-motor spin test. Firmware
can configure individual slots, but a bounded bench runner and confirmed limits
are still needed before enabling the one connected motor. The identity/status
check above is ready for the controller-only step while awaiting that setup.

References: [MC02 manual directory](https://github.com/dmBots/DM-MC02/tree/d65a3688981405dc205f628f1a026bde8cf8303e/manual),
[manufacturer USER button task](https://github.com/dmBots/DM-MC02/blob/d65a3688981405dc205f628f1a026bde8cf8303e/examples/CtrBoard-H7_ALL/App/key_task.c),
[ST-LINK/V2](https://www.st.com/en/development-tools/st-link-v2.html),
and [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html).

### Validation at handoff

On 2026-10-30, `scripts/build.sh` passed both ENCOS CTest targets (the Python
suite ran 111 tests, with one optional test skipped) and the portable Damiao C
suite. `motor_control`, `robot_description`, and `bringup` built with ROS Jazzy.
The STM32H723 ELF/bin rebuilt successfully with Arm GCC 13.2.1; vendor USB code
emits unused-parameter warnings. These are software checks only. Generated SDK,
build products and local test logs are excluded from Git; rebuild the ELF from
this branch before flashing.
