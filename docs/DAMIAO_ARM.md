# Damiao arm commissioning

ENCOS motors are chassis actuators. Damiao motors are arm actuators on a
separate DM-MC02 controller: Python on the NUC → USB CDC → C11 firmware on
STM32H723 → classic CAN at 1 Mbps. We implement the CAN protocol in FORGE;
the official Python USB2CAN SDK is a reference, not a library loaded by MC02.

## Verified hardware result — 2026-11-03

One secured, unloaded **DM-J4340P-2EC V1.1**, alone on **CAN2**, command ID 1,
feedback ID 0, completed the position–velocity test:

| Item | Observed result |
| --- | --- |
| Requested / final relative angle | +45° / +44.83° (0.17° error) |
| Movement onset | Visible by first 0.25 s log sample |
| Commanded maximum speed | 0.1 rad/s (5.73°/s) |
| Final settling | Within1° and <=0.03 rad/s for 0.3 s, around 9.5 s |
| Largest printed torque sample | 0.554 Nm; sampled output is not a peak capture |
| Exit | Disable acknowledged; original CTRL_MODE=1 restored and verified |
| Measured protocol ranges | PMAX12.5 rad, VMAX10 rad/s, TMAX28 Nm |
| Existing damping factor | 4; internal motor gains unchanged |

This validates one bench unit/move, not loaded-arm operation, all six motors,
bus loading, joint calibration or the ROS arm driver. Hardware revision V1.1
is distinct from manual V1.4. The motor's internal firmware version has not
been recorded. The 4310P hardware V1.1 label remains unverified.

## Current bench test

Only `scripts/test_dm_4340_bench.py` is the supported motion commissioning
entry point. It is fixed to the discovered CAN2/ID 1/feedback0 4340P and its
measured ranges. Do not run it unchanged for an 8009P or 4310P.

The superseded MIT bench ramps and their firmware enable path have been
removed. General MIT codec/profile support remains for the separate,
hardware-unvalidated six-joint interface; it is not the validated bench mode.
The initial disabled probe can still read a motor currently in MIT mode.

The script verifies identity, ranges and positive damping, writes/reads back
volatile TIMEOUT=2000 (100 ms), obtains fresh disabled feedback, writes
CTRL_MODE=2 while disabled/stationary, independently reads it back, and probes
again. It preloads the current position before enable confirmation, then sends
the target directly using CAN ID 0x101 and two little-endian float32 values:
position radians and maximum speed rad/s. There is no host ten-second ramp or
MIT gain command. Success requires the target and settling criteria above.
Original mode is restored after acknowledged disable; no zeroing, gain tuning,
parameter-save, ID changes or motor firmware updates occur.

Limits: +45° target; host travel window(-0.14,target+0.14) rad relative to start;
MCU window[-0.15,target+0.15]; commanded speed 0.1 rad/s; measured speed stop
0.3 rad/s host and0.36 rad/s MCU; absolute measured torque stop 3 Nm in both;
80°C MCU temperature stop; host deadline 12 s and MCU deadline 15 s.
The 3 Nm value is a **reactive feedback stop, not a hard torque/current clamp**.
Motor timeout 100 ms, host command lease 150 ms, firmware feedback timeout 100 ms
and host feedback-age check 80 ms are independent. MCU targets 500 Hz CAN updates.
CAN transmit completion/abort, bus-off, USB loss and hardware watchdog protect
against stale commands. No automatic fault clear, re-enable or gravity hold.

### Build, flash, run

From the repository root, fetch/build as described below. Keep motor PSU off
while flashing or resetting MC02. Keep the motor secured and output travel free.

```bash
# Set this to your installed STM32CubeProgrammer CLI:
STM32CLI="/path/to/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
sudo "$STM32CLI" -c port=SWD freq=240 mode=NORMAL reset=SWrst \
  -w "$PWD/build/dm_mc02/forge_arm.elf" -v -rst
```

On the commissioning NUC, the installed CLI is
`/home/forge2/Documents/Codex/2026-10-30/read/tools/STM32CubeProgrammer/bin/STM32_Programmer_CLI`.
The original 1 MiB flash backup is locally stored under
`build/dm_mc02/backups/mc02-before-forge-20261103-170007.bin`; do not commit
backups, SDK caches or build artifacts.

Turn motor PSU on, then:

```bash
python3 scripts/check_dm_mc02.py --port /dev/ttyACM0
python3 scripts/test_dm_4340_bench.py --port /dev/ttyACM0 --execute
```

The optional `--mode pos-vel --degrees 45` flags remain accepted for previous
commands. `--mode mit` is rejected. Default is the validated +45° position–velocity
test. Exit 0 means target reached and cleanup completed; exit 1 means incomplete
motion/settling; exit 2 means a failure. Inspect disable/mode-restoration messages.
If restoration fails, turn motor PSU off and power-cycle before retrying.

**Reset MC02 between tests:** previous configuration remains in MCU RAM even
when Python exits. With motor PSU off:

```bash
sudo "$STM32CLI" -c port=SWD freq=240 mode=NORMAL reset=SWrst -rst
```

Then power the motor and run again. Command 9 rejection before enable can mean
previous configuration or a latched fault, not motor failure. A reset requires
no reflash. Script/logging changes within existing firmware capabilities need
no flash; firmware/guard/protocol changes do. This cleanup removes the obsolete
MIT bench firmware entry point, so flash the rebuilt image to remove it on MCU.

### Read-only discovery

After a fresh MCU reset, with only the intended motor on the selected powered
bus, run (does not enable or write parameters):

```bash
python3 scripts/discover_dm_motor.py --port /dev/ttyACM0 --bus 2
```

USB handshake alone does not verify CAN. Solid green motor LED means enabled;
solid red means disabled; flashing red indicates a fault. Preserve the exact
ID, reply ID, mode, timeout and PMAX/VMAX/TMAX results before adapting any test.

## Next development work

See [robot-description commissioning handoff](../src/robot_description/ARM_COMMISSIONING.md).
Next: individually discover and test the 8009P on CAN1, a 4310P, the remaining 4340P
and other 4310Ps. Generalize the bench transport only after recording each
motor's ranges/IDs and validating model-specific limits. Do not infer ranges
from torque ratings, reuse3 Nm for every model, or mark six-axis profiles as
confirmed from one successful bench move.

## Why the old test was removed

Experimental MIT Kp2/Kd1 position ramps showed about 4 s delay before breakaway
near 0.6 Nm and final angles around32° instead of45°. Raising the feedback trip
threshold to 3 Nm did not fix low-stiffness tracking error. The position–velocity
run above moved promptly and reached the target using the motor's existing
loops. Historical code is available in Git; old operational instructions are
intentionally replaced by this single supported workflow.

## Verified manufacturer references

Retrieved 2026-10-30 from Damiao's `dmBots` repositories:

| Device | Manufacturer source | Finding |
| --- | --- | --- |
| J8009P V1.1 | [English user manual V1.0, 2026-09-10](https://github.com/dmBots/DM-J8009P-2EC/blob/42eaaf7f05cccba18d182b51504ee0349e16cff6/manual/DM-J8009P-2EC%20V1.1%20Gear%20Motor%20User%20Manual%20V1.0%202026-09-10.pdf) | Exact hardware-revision manual found; rated 20 Nm, peak 40 Nm |
| J4340P V1.1 | [English user manual V1.4, 2026-09-14](https://github.com/dmBots/DM-J4340P-2EC/blob/98a29a9df67c83512795f8bb9ca2bdd2423f66dd/manual/DM-J4340P-2EC%20V1.1%20Geared%20Motor%20User%20Manual%20V1.4%202026-09-14.pdf) | Exact revision found; rated 12 Nm, peak 40 Nm; older 9/27 Nm data does not apply |
| J4310P | [English user manual V1.3, 2026-09-14](https://github.com/dmBots/DM-J4310P-2EC/blob/9e728c9a42b498380ae43964636b3d66d01f42c7/manual/DM-J4310P-2EC%20Geared%20Motor%20User%20Manual%20V1.3%202026-09-14.pdf) | Rated 3.5 Nm, peak 12.5 Nm; no separately titled hardware V1.1 manual found |
| MC02 | [Manufacturer repository at pinned revision](https://github.com/dmBots/DM-MC02/tree/d65a3688981405dc205f628f1a026bde8cf8303e) | `manual/` contains board manual V1.1; USB/CAN examples supply STM32H723 support |


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

## Production arm interface and simulation boundary

The six-slot `ArmBus.configure()` / ROS `arm_driver` path remains MIT-based and
has software coverage only. It requires explicit `hardware_confirmed` and
`joint_order_confirmed`, populated limits, unique IDs and suitable timeouts.
Do not use the bench result to enable those template profiles. Position–velocity
is currently a fixed single-motor bench feature, not multi-axis ROS support.
Raw shaft radians/rad/s/Nm are used. Simulation joint names/default poses and
DOF1 zero rebasing are not measured physical signs/zeros or joint mapping.
See [model documentation](ONSHAPE_ARM_PREVIEW.md). No arm coordinate adapter
or synchronized feedback implementation is implied by this bench test.

## Firmware protocol and validation

64-byte CRC-framed USB v1; HELLO identity FORGE-MC02/1 and update rate 500 Hz.
Capability bits0–3 cover diagnostics/bench/fault details/confirmed enable;
bit 7 covers position–velocity. Bits4–6 and type13 were retired with experimental
MIT bench motion. Type12 is disabled MIT-mode probing only; type14 prepares
position–velocity bench; type15 writes mode 1/2 only with fresh stationary
acknowledged-disabled bench feedback. Mode writes invalidate old commands and
mode verification; enable requires a matching fresh probe. Normal profile
commands cannot replace bench configuration or enable its MIT probe path.

```bash
cmake -S src/damiao_firmware -B build/damiao_tests
cmake --build build/damiao_tests
ctest --test-dir build/damiao_tests --output-on-failure
PYTHONPATH=src python3 -m unittest discover -s tests -p test_damiao_arm.py -v
```

Coverage includes CAN packing, mode write/readback mismatch, delayed enable,
wrong-mode/gain/speed rejection, simulated target completion and stall,
restoration, leases, feedback loss, torque stops, framing and transmit abort.
Tests do not substitute for hardware commissioning of the remaining motors.
