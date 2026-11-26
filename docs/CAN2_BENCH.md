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
The operator confirmed the [wheel mapping](DIFFERENTIAL_DRIVE.md#model-source-and-hardware-mapping)
on Nov 25. The operator closed [COMMS-01](ARM_SESSION_HANDOFF.md#comms-01-closure)
after the Nov 26 reply-loss tests on both CAN ports. Loaded stopping and
host-loss qualification remain open. The [Nov 24 physical record](measurements/encos-four-motor-physical-review-2026-11-24.json)
preserves the bench results.

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
profile. Wait for `Ready` with `motor IDs=(1, 2, 3, 4)`. The confirmed mapping
uses these command signs.

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

For a changed setup, record discovery/readback output, the JSON report paths,
any revised wheel locations and signs, and whether all four wheels turn and
stop as expected under W/S/A/D. Loaded traction, stopping and host-loss
acceptance remain open.

Hardware-free rehearsal uses the same paths:

```bash
.venv/bin/python scripts/test_encos_motors.py --four-motors --simulate
bash scripts/run_motor_test.sh --four-motors --simulate
```

## Combined ENCOS commissioning

Use the confirmed four-motor profile and the
[secured, unloaded fixture](ENCOS_TEST_BENCH.md#12-stop-behavior-and-fault-tests).
Keep the 5 output-shaft RPM and 2 A bench limits for the bounded motion already
documented above. The operator runs the physical stages. Record the exact
bridge board/firmware and ESI revision, each motor's model/firmware and
configured CAN timeout, and the software commit and profile. The supplier
question for that bridge revision is when each input slot's ID/DLC/data is
cleared or replaced, and what event drives the State/Toggle bits.

| Stage | Capture and evaluate | Status or TODO |
| --- | --- | --- |
| Healthy four-motor baseline | All four motors supplied recurring stationary feedback. For protection measurements, record motor current, temperature and the independent motion reference during bounded motion and deliberate stop. | COMMS-01 closed. Protection work is deferred under PROTECT-01. |
| Individual reply loss | The [guided reply-loss test](#individual-reply-loss-with-continuous-polling) passed for ID 1 on CAN1 and ID 3 on CAN2 with all four polled. | [COMMS-01 closed](ARM_SESSION_HANDOFF.md#comms-01-closure). |
| Owner and link loss | On the secured, unloaded fixture, record graceful stop as a reference. Run publisher loss, host-owner termination and EtherCAT link loss as separate events with an agreed independent means of removing motor power. Keep the independent CAN logger and motion reference running through the loss and reconnection. Compare the last outgoing nonzero command with the physical stop time and verify that motion requires explicit re-arm. | Deferred COMMS-02; protection questions remain under deferred PROTECT-01 |
| Loaded chassis | After unloaded stop behavior and current/thermal limits are established, use a separate supported loaded run. Measure travel and yaw independently for effective radius/track and slip. Record current, temperature, load and stopping distance with the same evidence format. | Deferred CHASSIS-03 and PROTECT-01 |

When PROTECT-01 resumes, verify phase-current scaling against independent phase-current
measurement and use the motor's documented temperature limits. Record the
configured command ceiling separately from measured current and stop behavior.
See the [protection TODO](MOTOR_PROTECTION_TODO.md).

### Individual reply loss with continuous polling

Start with all four stationary motors online and the previous owner closed.
Use an installed, independently switchable motor supply branch or a fixture
that can suppress that motor's replies while preserving the other three and
CAN termination. Keep bridge power connected throughout. Leave the CAN and
motor connectors seated. Make any wiring changes with the equipment unpowered.

```bash
sudo ./build/encos_driver/encos_feedback_probe enp86s0 \
  "build/encos-loss-$(date -u +%Y%m%dT%H%M%SZ).jsonl" --reply-loss all
```

The probe names one ID at a time. Suppress that ID, then press Enter.
When prompted, restore it and press Enter. To test a single branch,
replace `all` with the motor ID. Every exchange polls all four motors on
their commissioned routes, including while the probe waits for Enter.
It alternates two read-only queries (position and firmware version) every
50 ms so a stationary motor can demonstrate progress even at a constant angle.

The report stores both operator markers, time-stamped raw input images and
each output change. The probe evaluates the second from 250 to 1,250 ms
after the suppression marker. It reports how often a target record remains
visible and fails if the target continues changing reply type. Each peer
must continue changing type within 250 ms. After the restore marker, two new
target reply-type changes must arrive within five seconds. EtherCAT errors,
sampling gaps over 250 ms and missing markers fail the run.
Markers time the keypress. For actual CAN loss timing and qualification,
start the [independent CAN capture](#passive-capture-on-the-bench) before the test.

### Stationary feedback probe with the existing bridge

To check the normal native driver's idle position updates on the secured
four-motor bench, close the previous chassis owner normally, then run:

```bash
sudo .venv/bin/python scripts/test_encos_motors.py --four-motors --query-only
```

This selects a two-second position-only observation through the regular
driver. Each motor must supply recurring timestamp updates and stay within
the 250 ms idle activation age. The terminal prints updates seen and peak
sampled age per motor plus the saved JSON path. The sampled count is a lower
bound on driver updates. This mode issues no position or velocity setpoints.
Use the separate probe below to inspect raw process images.

The operator can test the host-visible response with only the ENCOS bridge
and four stationary motors. Use the commissioned split bus and a secured,
unloaded fixture. Close the existing chassis owner normally. Run from the
workspace root with the bridge on its dedicated interface:

```bash
cmake --build build/encos_driver --target encos_feedback_probe
sudo ./build/encos_driver/encos_feedback_probe enp86s0 \
  "build/encos-feedback-$(date -u +%Y%m%dT%H%M%SZ).jsonl"
```

On a fresh checkout, configure the native build with `bash scripts/build.sh`
first. The probe takes about eight seconds. It uses the existing EtherCAT
interface lock, reads the bridge identity and optional version strings, and
issues only position and version queries to IDs 1/2/3/4 in slots 0/1/3/4.
After observing positions for all four, it leaves one motor's output slot
empty for 800 ms while querying the other three. It then requests that
motor's version followed by its position. This tests whether the returned
record changes type when polling resumes. After a complete baseline, a
missing or ambiguous reply is recorded and the remaining IDs are examined.
A failed exchange, motor error, or interrupt ends acquisition and triggers
an empty output exchange. Each run requires a new report path.

For each pause the terminal prints how many returned process images still
contained the target's position after 250 ms, how many matched its previous
payload, and how often the global Toggle bit changed. For example, 100/100
means the position stayed visible throughout those 100 observations. 0/100
means it was absent from every sampled image in that interval. JSONL retains
the raw output for each stage, timestamped input images and WKC, zero-DLC
observations, State/Toggle edges, and the final completion flag. An
interrupted or failed experiment exits nonzero. Withholding a host query
does not establish when the last physical CAN reply arrived. The paired
capture below can measure physical CAN timing.

**Optional USB-C debug capture.** Page 2 of the supplied bridge manual
identifies USB-C as a power input and a firmware-debug serial port.
Use a vendor-confirmed power arrangement before connecting it to USB while
external 5 V is present. Identify the bridge's own idle tty and choose a
baud rate to test. Append `--debug-port "$BRIDGE_TTY" "$DEBUG_BAUD"` to
the probe command. The probe opens only the named tty, read-only, in raw
8N1 mode, and restores its termios settings on exit. It writes received
bytes as hex in `debug_rx` records with host read times on the same monotonic
clock as the PDOs. A zero byte count means silence at that setting during
the run. Bytes received establish serial activity. Identifying independent
CAN RX events requires decoding the trace and establishing its source.

Inspect the journal without hardware:

```bash
.venv/bin/python scripts/inspect_encos_debug.py "$REPORT"
```

Set `REPORT` to the printed JSONL path. The inspector checks for exact
position and version payload bytes in binary, contiguous hex or spaced hex
within the same host polling stage. It distinguishes a quiet port, other
debug bytes and recognizable payloads. A match can come from a cached copy
or a payload shared by several motors. The reported target ID identifies the
test stage. The serial protocol, motor identity and physical CAN RX time
require separate confirmation.

### Passive capture on the bench

Collect both CAN buses with independent SocketCAN adapters in listen-only
mode, using the existing bit timing and termination. Use an Ethernet capture
on the dedicated ENCOS interface or a passive capture point that sees returned
EtherCAT frames. Capture software only records traffic and runs alongside the
existing owner. Prefer both captures on the same clock and record a bound on
relative timestamp error, including adapter delays. If the host is being
powered off, use an independent logger so the capture survives. Retain
logger drop counts and reject capture windows with drops or logger outage.

Example commands in separate capture terminals with an empty run directory
and the approved interfaces set in each terminal:

```bash
sudo tcpdump -i "$ETHERCAT_IF" -s 0 -w "$RUN_DIR/ethercat.pcap" 'ether proto 0x88a4'
candump -d -L "$CAN1_MONITOR" "$CAN2_MONITOR" > "$RUN_DIR/can.log"
```

Use a classic Ethernet pcap from the single-bridge SOEM process image and
absolute timestamps in the candump log. Capture the healthy baseline before
each analysis window and continue logging until after the last event. Save the
window start/end and event times in absolute epoch seconds. The analyzer
requires pre-window CAN replies from all four IDs and at least one valid
returned process image. The passive capture commands use
[tcpdump](https://github.com/the-tcpdump-group/tcpdump/blob/master/tcpdump.1.in)
and [can-utils candump](https://github.com/linux-can/can-utils/blob/master/candump.c).

```bash
.venv/bin/python scripts/analyze_encos_capture.py \
  "$RUN_DIR/ethercat.pcap" "$RUN_DIR/can.log" \
  --can1 "$CAN1_MONITOR" --can2 "$CAN2_MONITOR" \
  --start "$START_EPOCH_S" --end "$END_EPOCH_S" \
  --clock-skew-ms "$MEASURED_SKEW_BOUND_MS" > "$RUN_DIR/capture-analysis.json"
```

Review per-motor CAN reply counts and gaps, identical replies of the same
type, and valid PDOs that still report data after the CAN silence threshold.
The default 250 ms matches the driver's observation timeout. Use one window
per event and check that the other motors continue reporting during the
single-motor loss case. During CAN silence, the report distinguishes matching
retained payloads from cleared or missing feedback and counts Toggle edges.
Review the raw bytes and supplier semantics before using a bit as a freshness
signal. An observed Toggle edge during one motor's silence can occur while
other motors keep reporting.

For COMMS-02, set the window start to the marked loss event. Compare nonzero
velocity command counts and the last observed command time with the
independent shaft or wheel motion trace. A zero-velocity command is a request.
Use the motion trace to establish physical stopping and check for movement
after reconnection. The capture report is evidence for review. Acceptance
requires repeatable physical results with the identified bridge firmware.
