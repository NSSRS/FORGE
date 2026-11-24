# Split-bus ENCOS chassis bench

The operator's four-motor 5 RPM test passed on Nov 24; use
`bash scripts/run_motor_test.sh --four-motors` for this bench. See the
[physical result](measurements/encos-four-motor-physical-review-2026-11-24.json)
and [four-motor procedure](#motor-4-identification-and-four-motor-test).

The retained default three-motor profile is `src/bringup/config/chassis_bench.yaml`:
IDs 1/2 on CAN1, ID 3 on CAN2, and motor 4 explicitly omitted. Use W/S/A/D or
one proportional radio stick through the same chassis command path.

- [Driver and W/S/A/D commands](DIFFERENTIAL_DRIVE.md#three-motor-differential-bench-motor-4-absent)
- [ESP32 radio input and calibration](RADIO_CONTROL.md)
- [Four-pane motor test and MuJoCo feedback](MOTOR_MIRROR.md)

That three-motor profile requests 0.08 m/s forward/reverse and 0.28 rad/s yaw, with a
20 RPM per-motor ceiling and the provisional 2 A phase-current ceiling. Initial
position feedback from all three present motors is required at startup. Motor 4
is never synthesized or silently removed after a fault.

`can2_bench.yaml` remains a legacy direct-RPM driver profile with a 5 RPM ceiling.
It was used for the operator-confirmed 2026-09-29 CAN2 test; the teleop no longer
accepts it. The historical result remains in the
[bench record](ENCOS_TEST_BENCH.md#can2-bench-confirmation-2026-09-29).
Stop previous driver and teleop processes before changing profiles. Power off
before moving CAN wiring and verify termination on each separate bus.

## Slot routing

`motor_slots: [0, 1, 3]` explicitly maps the three motors to zero-based PDO slots.
Slots 0-2 route to CAN1, slots 3-5 to CAN2, independent of motor ID. The native
mapped-open API keeps the original ABI and contiguous defaults available.
The output span reaches slot 3 (`Motor_Num=4`); vacant slot 2 remains empty
(ID=0, DLC=0, no payload). It does not contain a query or an eight-byte motor
control packet. This layout was used for the operator-confirmed CAN2 test.
The fake bridge verifies software routing, forward/reverse/stop, and empty gaps;
it models replies for the same CAN ID replacing each other to catch accidental
query traffic masking control feedback. It does not establish bridge firmware behavior.

The initial split-bus implementation filled slot 2 with position queries for ID 1.
That can mask motor 1's control acknowledgements and reproduce the reported
250 ms feedback timeout. Removing that filler fixes the reproduced failure
without a startup delay or a longer watchdog. The subsequent operator test
confirmed motor 3 working on CAN2; no raw CAN capture was supplied to establish
the exact firmware reply-replacement behavior.

`encos_query` defaults to slot 0/CAN1. The Nov 24 update adds `--can-bus 2`
for read-only queries through slot 3/CAN2. Query and ID-assignment utilities
now share the native driver's interface lock; do not run a second master.

## Motor 4 identification and four-motor test

Physical test completed 2026-11-24: report `20261124T110254.496409Z.json`
contains all four IDs, both signed motion legs and successful stop checks.
All 115 cached samples have fault/error zero; all observed WKC values are 3/3,
and the final counters show 532 exchanges with zero failures. The owner closed.
The operator then launched the four-motor pane/viewer workflow and reported
normal visual behavior. Identification and the bounded four-motor test do not
need repeating; the commands below remain the setup/reference procedure.
Explicit wheel location/sign/gearing and loaded/host-loss qualification remain
open under CHASSIS-02 and the communication tasks. See the
[physical record](measurements/encos-four-motor-physical-review-2026-11-24.json).

The `src/bringup/config/chassis_four_motor_bench.yaml` profile requires all four motors,
with a 5 output-shaft RPM ceiling and the existing provisional 2 A phase-current
ceiling per motor. The commissioned three-motor profile is unchanged.

### 1. Connect and identify the isolated fourth motor

Close the existing ENCOS driver/test normally before these commands. Change
wiring with ENCOS power off. Secure the motors or support the chassis with
all drive wheels clear; these are unloaded direction tests, not loaded driving.
Retain the existing motor supply and bridge 5 V supply/harness. Connect the
NUC's `enp86s0` to EtherCAT **IN**, and motor 4's CAN H/L to bridge **CAN2**.
Disconnect or power off the other three ENCOS motors for discovery and any ID
change. Duplicate default IDs must never receive a shared-bus ID-change write.

CAN2's sockets share one bus; a PDO slot is a software passage, not a physical
socket number. Verify end termination with power off using the existing
[wiring procedure](ENCOS_TEST_BENCH.md#3-wire-everything-with-power-off).
Use the existing harness/pinout, not guessed colors or motor power on CAN pins.

The updated native utilities are already built in this checkout. For a clean
checkout run `bash scripts/build.sh` first. With only motor 4 powered:

```bash
cd /home/forge2/forge2_ws
sudo ./build/encos_driver/encos_query enp86s0 --can-bus 2
```

Expected: `Query-only mode; CAN2, PDO slot 3`, `Discovered motor CAN ID: N`,
then position, hardware/software versions, CAN timeout and brake status.
This command must not rotate the motor. Do not assume it is already ID 4.

If the discovered ID is already **4**, skip assignment. Otherwise enter the
actual discovered ID below, keeping every other motor disconnected/off:

```bash
read -r -p "Discovered CAN ID: " encos_old_id
sudo ./build/encos_driver/encos_assign_id enp86s0 "$encos_old_id" 4 --can-bus 2 --execute
```

The tool checks isolated discovery against OLD_ID, sends one persistent
ID-change frame, requires the matching acknowledgement, and clears output.
On `PASS`, power-cycle only that motor and verify:

```bash
sudo ./build/encos_driver/encos_query enp86s0 --can-bus 2 --motor-ids 4
```

Expected: `Motor 4: ...` with no motor error. If identification or assignment
fails, retain the output; do not reconnect possible duplicate IDs or proceed
to motion. No CAN timeout, brake, zero, gains or firmware settings are changed.

### 2. Test motor 4 alone

Keep the other motors off and run:

```bash
sudo .venv/bin/python scripts/test_encos_motors.py --motor4-only --execute
```

After a three-second countdown, only ID 4 is commanded: **+5 RPM for one
second -> below 0.5 RPM -> -5 RPM for one second -> below 0.5 RPM -> close**.
Each leg is roughly 30 output-shaft degrees; slew, load and timing change the
actual travel, so exact return-to-start is not an acceptance claim. Observe
which physical wheel/shaft turns and its direction for positive RPM. The sign
is protocol-relative; do not infer physical CW/CCW from the ID.

The test retains ID 4's full-rig slot **4** on CAN2 even when it is the only
selected motor. It requires initial feedback and signed motion evidence,
checks the existing fault latch, stops between directions and records the
owner's cached telemetry. Ctrl+C closes that owner; no auto-reset/re-enable or
extra reader is added. Expected final line: `PASS: every selected motor reported
both commanded motion and stop.` The `Report:` line names a JSON under
`build/encos/sessions/`, also written on failure. PDO observation age is not
independently verified CAN age.

### 3. Reconnect and test all four

Power off before restoring the complete split bus:

| Motor CAN ID | Bridge bus | Zero-based PDO slot |
| --- | --- | --- |
| 1 | CAN1 | 0 |
| 2 | CAN1 | 1 |
| 3 | CAN2 | 3 |
| 4 | CAN2 | 4 |

Leave slots 2 and 5 unused. Motors 3/4 share CAN2 with unique IDs; preserve
end termination after adding the fourth motor. With all outputs secured/clear:

```bash
sudo .venv/bin/python scripts/test_encos_motors.py --four-motors --execute
```

All four IDs must provide initial feedback before any motion. This bounded
motor test commands the **same protocol sign** on all four (+5 then -5 RPM),
not chassis-forward mixing. It repeats the one-second legs and stop checks,
and writes one report. A missing ID fails; it never silently omits motor 4.

### 4. W/S/A/D together

After the bounded test exits, start the existing four-pane workflow as your
normal desktop user (the launcher requests sudo once):

```bash
bash scripts/run_motor_test.sh --four-motors
```

Driver, keyboard and read-only MuJoCo viewer receive the same four-motor
profile. Wait for `Ready` with `motor IDs=(1, 2, 3, 4)`. With provisional
wheel signs confirmed by the physical direction checks:

| Key | Requested movement | ID 1/2/3/4 protocol signs |
| --- | --- | --- |
| W | forward | - / - / + / + |
| S | reverse | + / + / - / - |
| A | left pivot | + / + / + / + |
| D | right pivot | - / - / - / - |
| Space or X | request zero speed | 0 / 0 / 0 / 0 |

Hold the key: a second matching event starts motion, and key repeat refreshes
the 0.15 s keyboard lease. W/S requests 0.02 m/s (about 4.77 wheel RPM);
pivots are scaled to the 5 RPM motor ceiling. Native 60 RPM/s slew and 0.25 s
lease remain. Release requests zero after the keyboard lease; this is not a
mechanical brake. Stop motion before closing the session. Closing only the
viewer does not stop the driver. Do not run another `/cmd_vel` publisher
alongside this keyboard.

Send the discovery/readback output, both JSON report paths, the physical
location and positive-RPM direction of motor 4, and whether all four turn and
stop as expected under W/S/A/D. Record any error exactly. Wheel sign/gearing,
loaded traction/stopping and host-loss acceptance remain open.

Hardware-free rehearsal uses the same paths:

```bash
.venv/bin/python scripts/test_encos_motors.py --four-motors --simulate
bash scripts/run_motor_test.sh --four-motors --simulate
```
