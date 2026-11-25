import io
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

import source_paths
from motors.encos_capture import analyze, pcap_frames, timestamp_ns


POSITION = bytes.fromhex("4000000000000064")
VELOCITY = bytes.fromhex("6000000000000064")
QUERY = bytes.fromhex("a00100000000")
MOVING = bytes.fromhex("4240a000000014")
STOP = bytes.fromhex("43000000000014")
START = timestamp_ns("1700000001")
END = timestamp_ns("1700000002")


def can(milliseconds, interface, motor_id, payload):
    stamp = (START + milliseconds * 1_000_000) / 1_000_000_000
    return f"({stamp:.6f}) {interface} {motor_id:03X}#{payload.hex()}"


def incoming(records, *, toggle=False, wkc=3, separate_read=False):
    pdo = bytearray(92)
    pdo[0] = len(records)
    pdo[87] = 0x80 if toggle else 0
    for slot, identifier, payload in records:
        struct.pack_into("<IBB8s", pdo, 2 + slot * 14, identifier, 0, len(payload), payload)
    command, address, data = (0x0A, 86, pdo) if separate_read else (0x0C, 0, bytes(86) + pdo)
    flags = 0x8000 | len(data)
    datagram = struct.pack("<BBIHH", command, 1, address, flags, 0) + data + struct.pack("<H", wkc)
    dc_datagram = struct.pack("<BBIHH8sH", 0x0E, 1, 0, 8, 0, bytes(8), 1)
    ethernet = bytes(12) + bytes.fromhex("88a4")
    return ethernet + struct.pack("<H", 0x1000 | (len(datagram) + len(dc_datagram))) + datagram + dc_datagram


def pcap(packets, *, order="<", nanos=False):
    magic = 0xa1b23c4d if nanos else 0xa1b2c3d4
    output = bytearray(struct.pack(order + "IHHIIII", magic, 2, 4, 0, 0, 65535, 1))
    for milliseconds, frame in packets:
        seconds, fraction = divmod(START + milliseconds * 1_000_000, 1_000_000_000)
        output.extend(struct.pack(order + "IIII", seconds, fraction if nanos else fraction // 1000,
                                  len(frame), len(frame)))
        output.extend(frame)
    return bytes(output)


class EncosCaptureTests(unittest.TestCase):
    def setUp(self):
        self.can_baseline = [can(-100, "can0" if motor_id < 3 else "can1", motor_id, QUERY)
                             for motor_id in (1, 2, 3, 4)]
        self.baseline = incoming([(0, 1, QUERY), (1, 2, QUERY), (3, 3, QUERY), (4, 4, QUERY)])

    def analyze(self, packets, log):
        return analyze(pcap_frames(io.BytesIO(pcap(packets))), log, can1="can0", can2="can1",
                       start_ns=START, end_ns=END, silence_ns=250_000_000, skew_ns=2_000_000)

    def test_identical_replies_and_other_motors_do_not_hide_retained_reply(self):
        log = self.can_baseline + [
            can(10, "can0", 1, POSITION),
            can(15, "can0", 1, VELOCITY),
            can(20, "can0", 1, POSITION),
            can(110, "can1", 4, VELOCITY),
            can(119, "can0", 4, POSITION),  # Wrong bus, even with the expected ID.
            "(1700000001.130000) can1 00000004#4000000000000064",  # Extended ID.
            "(1700000001.150000) can1 004#R8",
            can(365, "can0", 1, POSITION),
        ]
        packets = [
            (-80, self.baseline),
            (200, incoming([(1, 4, VELOCITY), (5, 1, POSITION)])),  # Reply slots can move.
            (361, incoming([(1, 4, VELOCITY), (5, 1, POSITION)], toggle=True)),
            (362, incoming([(1, 4, VELOCITY), (5, 1, POSITION)], toggle=True)),
            (369, incoming([(1, 4, b""), (5, 1, POSITION)])),
            (400, incoming([(1, 4, VELOCITY)], wkc=0)),
        ]
        report = self.analyze(packets, log)
        self.assertEqual(report["pdo_frames"], 4)
        self.assertEqual(report["pdo_status_byte_87"], {"00": 2, "80": 2})
        one, four = report["motors"][1], report["motors"][4]
        self.assertEqual(one["identical_same_type_replies"], 2)
        self.assertEqual(one["pdo_silence_with_old_feedback"], 2)
        self.assertEqual(four["can_feedback_frames"], 1)
        self.assertEqual(four["pdo_silence_observations"], 2)  # 361 ms is within clock uncertainty.
        self.assertEqual(four["pdo_silence_matching_last_can_reply"], 1)
        self.assertEqual(four["pdo_silence_without_feedback"], 1)
        self.assertEqual(four["pdo_toggle_edges_during_silence"], 1)
        self.assertEqual(four["pdo_dlcs_by_id"], {8: 3, 0: 1})

    def test_link_loss_window_can_still_show_commands_on_can(self):
        report = self.analyze([(-80, self.baseline)], self.can_baseline + [
            can(70, "can1", 4, MOVING), can(120, "can1", 4, MOVING), can(150, "can1", 4, STOP),
        ])
        self.assertEqual(report["pdo_frames"], 0)
        four = report["motors"][4]
        self.assertEqual(four["nonzero_velocity_can_commands"], 2)
        self.assertEqual(four["last_nonzero_velocity_ms_from_start"], 120)
        self.assertEqual(four["zero_velocity_can_commands"], 1)

    def test_missing_baselines_and_truncated_capture_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "all four routed"):
            self.analyze([(-80, self.baseline)], self.can_baseline[:-1])
        with self.assertRaisesRegex(ValueError, "valid ENCOS input PDO"):
            self.analyze([(10, self.baseline)], self.can_baseline)
        with self.assertRaisesRegex(ValueError, "dropped frames"):
            self.analyze([(-80, self.baseline)], self.can_baseline + ["DROPCOUNT: dropped 1 CAN frame"])
        with self.assertRaisesRegex(ValueError, "invalid classic CAN payload"):
            self.analyze([(-80, self.baseline)], self.can_baseline + ["(1700000001.1) can1 004#600"])
        with self.assertRaisesRegex(ValueError, "invalid pcap packet data"):
            list(pcap_frames(io.BytesIO(pcap([(-80, self.baseline)])[:-1])))

    def test_cli_reads_nanosecond_pcap_and_separate_logical_read(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            capture, log = Path(temp) / "bridge.pcap", Path(temp) / "can.log"
            separate = incoming([(0, 1, QUERY)], wkc=1, separate_read=True)
            capture.write_bytes(pcap([(-80, separate), (300, separate)], order=">", nanos=True))
            log.write_text("\n".join(self.can_baseline), encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(root / "scripts/analyze_encos_capture.py"), str(capture), str(log),
                 "--can1", "can0", "--can2", "can1", "--start", "1700000001", "--end", "1700000002",
                 "--clock-skew-ms", "1"],
                check=True, capture_output=True, text=True,
            )
            report = json.loads(result.stdout)
            self.assertEqual(report["pdo_frames"], 1)
            self.assertEqual(report["motors"]["1"]["pdo_silence_matching_last_can_reply"], 1)
