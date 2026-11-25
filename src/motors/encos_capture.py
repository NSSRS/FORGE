"""Compare passive CAN and EtherCAT captures of the four-motor ENCOS bench."""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
from decimal import Decimal, InvalidOperation
import json
from math import isfinite
from pathlib import Path
import re
import struct

from motors.commissioning import BenchPlan


NS_PER_SECOND = 1_000_000_000
NS_PER_MS = 1_000_000
CAN_LOG = re.compile(r"\((\d+\.\d{1,9})\)\s+(\S+)\s+([0-9a-fA-F]{3}|[0-9a-fA-F]{8})#(\S*)")
PCAP_FORMATS = {
    b"\xd4\xc3\xb2\xa1": ("<", 1000),
    b"\xa1\xb2\xc3\xd4": (">", 1000),
    b"\x4d\x3c\xb2\xa1": ("<", 1),
    b"\xa1\xb2\x3c\x4d": (">", 1),
}
CAN_RECORD = struct.Struct("<IBB8s")
OUTPUT_BYTES = 86
INPUT_BYTES = 92
# Group 0 in the native driver's single-slave SOEM map. A blocked LRW
# uses a separate logical read with its own working counter.
INPUT_DATAGRAMS = {(0x0C, 0, OUTPUT_BYTES + INPUT_BYTES): (OUTPUT_BYTES, 3),
                   (0x0A, OUTPUT_BYTES, INPUT_BYTES): (0, 1)}


def pcap_frames(stream):
    header = stream.read(24)
    fmt = PCAP_FORMATS.get(header[:4])
    if len(header) != 24 or fmt is None:
        raise ValueError("expected a classic Ethernet pcap file")
    order, fraction_to_ns = fmt
    major, minor, _, _, snaplen, linktype = struct.unpack_from(order + "HHIIII", header, 4)
    if (major, minor) != (2, 4) or linktype & 0xFFFF != 1:
        raise ValueError("expected Ethernet link type in a pcap 2.4 file")
    while record := stream.read(16):
        if len(record) != 16:
            raise ValueError("truncated pcap packet header")
        seconds, fraction, captured, original = struct.unpack(order + "IIII", record)
        if captured != original or captured > min(snaplen, 262144):
            raise ValueError("truncated or invalid pcap packet; capture with full snap length")
        frame = stream.read(captured)
        if len(frame) != captured or fraction * fraction_to_ns >= NS_PER_SECOND:
            raise ValueError("invalid pcap packet data or timestamp")
        yield seconds * NS_PER_SECOND + fraction * fraction_to_ns, frame


def bridge_inputs(frame):
    ethercat = 14
    while frame[ethercat - 2:ethercat] in (b"\x81\x00", b"\x88\xa8"):
        ethercat += 4
    if frame[ethercat - 2:ethercat] != b"\x88\xa4":
        return
    if len(frame) < ethercat + 2:
        raise ValueError("missing EtherCAT header")
    header, = struct.unpack_from("<H", frame, ethercat)
    if header >> 12 != 1:
        return
    cursor = ethercat + 2
    end = cursor + (header & 0x7FF)
    if end > len(frame):
        raise ValueError("truncated EtherCAT frame")
    while cursor + 12 <= end:
        command, _, address, flags = struct.unpack_from("<BBIH", frame, cursor)
        size = flags & 0x7FF
        data_start = cursor + 10
        data_end = data_start + size
        if data_end + 2 > end:
            raise ValueError("truncated EtherCAT datagram")
        layout = INPUT_DATAGRAMS.get((command, address, size))
        if layout is not None:
            offset, expected_wkc = layout
            wkc, = struct.unpack_from("<H", frame, data_end)
            if wkc == expected_wkc:
                yield frame[data_start + offset:data_end]
        if not flags & 0x8000:
            break
        cursor = data_end + 2


def feedback_type(payload):
    """Recognize feedback used by the native owner on this velocity/query bench."""
    if len(payload) == 8 and payload[0] >> 5 in (2, 3):
        if isfinite(struct.unpack_from(">f", payload, 1)[0]):
            return payload[0] >> 5
    if len(payload) == 6 and payload[0] >> 5 == 5 and payload[1] == 1:
        if isfinite(struct.unpack_from(">f", payload, 2)[0]):
            return 5
    return None


def can_log(lines, routes):
    replies = {motor_id: [] for _, motor_id in routes}
    commands = {motor_id: [] for _, motor_id in routes}
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith("DROPCOUNT:"):
            raise ValueError("candump reports dropped frames; recapture before assessing silence")
        match = CAN_LOG.fullmatch(line)
        if match is None:
            raise ValueError(f"expected a candump -L record, got {line!r}")
        stamp, interface, identifier, data = match.groups()
        motor_id = int(identifier, 16)
        if len(identifier) != 3 or (interface, motor_id) not in routes or re.fullmatch(r"R[0-8]?", data):
            continue
        if len(data) % 2 or not re.fullmatch(r"[0-9a-fA-F]{0,16}", data):
            raise ValueError(f"invalid classic CAN payload in {line!r}")
        payload = bytes.fromhex(data)
        when_ns = int(Decimal(stamp) * NS_PER_SECOND)
        kind = feedback_type(payload)
        if kind is not None:
            replies[motor_id].append((when_ns, kind, payload))
        if len(payload) == 7 and payload[0] in (0x42, 0x43):
            rpm, = struct.unpack_from(">f", payload, 1)
            if isfinite(rpm):
                commands[motor_id].append((when_ns, rpm))
    for records in (*replies.values(), *commands.values()):
        records.sort(key=lambda item: item[0])
    return replies, commands


def analyze(packets, lines, *, can1, can2, start_ns, end_ns, silence_ns, skew_ns):
    if can1 == can2 or start_ns >= end_ns or not 0 <= skew_ns < silence_ns:
        raise ValueError("require distinct interfaces, an increasing window, and skew below silence threshold")

    plan = BenchPlan.fourth()
    routes = {(can1 if slot < 3 else can2, motor_id) for motor_id, slot in
              zip(plan.motor_ids, plan.motor_slots)}
    replies, commands = can_log(lines, routes)
    times = {motor_id: [item[0] for item in records] for motor_id, records in replies.items()}
    if any(not records or records[0] > start_ns - skew_ns for records in times.values()):
        raise ValueError("need CAN feedback from all four routed motors before the analysis window")

    by_motor = {}
    for motor_id, slot in zip(plan.motor_ids, plan.motor_slots):
        previous_payload = {}
        identical = 0
        feedback = [(when, kind, data) for when, kind, data in replies[motor_id]
                    if start_ns <= when <= end_ns]
        for _, kind, data in feedback:
            if data == previous_payload.get(kind):
                identical += 1
            previous_payload[kind] = data
        velocity = [(when, rpm) for when, rpm in commands[motor_id] if start_ns <= when <= end_ns]
        nonzero = [when for when, rpm in velocity if rpm != 0.0]
        gaps = [start_ns, *(when for when, _, _ in feedback), end_ns]
        by_motor[motor_id] = dict(
            can_bus=1 if slot < 3 else 2, output_slot=slot,
            can_feedback_frames=len(feedback), identical_same_type_replies=identical,
            longest_gap_in_window_ms=max(b - a for a, b in zip(gaps, gaps[1:])) / NS_PER_MS,
            zero_velocity_can_commands=len(velocity) - len(nonzero),
            nonzero_velocity_can_commands=len(nonzero),
            last_nonzero_velocity_ms_from_start=(nonzero[-1] - start_ns) / NS_PER_MS if nonzero else None,
            pdo_feedback_frames=0, pdo_silence_observations=0, pdo_silence_without_feedback=0,
            pdo_silence_with_old_feedback=0, pdo_silence_matching_last_can_reply=0,
            pdo_toggle_edges_during_silence=0, pdo_dlcs_by_id=Counter(),
        )

    baseline_pdo = False
    pdo_frames = 0
    status_bytes = Counter()
    ide_values = Counter()
    previous_toggle = None
    for when, frame in packets:
        if when > end_ns:
            continue
        for pdo in bridge_inputs(frame):
            if pdo[0] > 6:
                raise ValueError("expected six-record ENCOS input layout")
            if when < start_ns:
                baseline_pdo = True
                continue
            pdo_frames += 1
            status_bytes[f"{pdo[87]:02x}"] += 1
            ide_values[pdo[1]] += 1
            toggle = bool(pdo[87] & 0x80)
            edge = previous_toggle is not None and toggle != previous_toggle
            previous_toggle = toggle
            records = list(CAN_RECORD.iter_unpack(pdo[2:OUTPUT_BYTES]))
            for motor_id, evidence in by_motor.items():
                payloads = []
                for identifier, rtr, dlc, data in records:
                    if identifier != motor_id:
                        continue
                    evidence["pdo_dlcs_by_id"][dlc] += 1
                    payload = data[:dlc]
                    if rtr == 0 and dlc <= len(data) and feedback_type(payload) is not None:
                        payloads.append(payload)
                if payloads:
                    evidence["pdo_feedback_frames"] += 1
                # Allow for capture clock error in either direction. Even the
                # newest possible reply must predate the silence threshold.
                index = bisect_right(times[motor_id], when + skew_ns) - 1
                if index < 0 or when - times[motor_id][index] - skew_ns < silence_ns:
                    continue
                evidence["pdo_silence_observations"] += 1
                if edge:
                    evidence["pdo_toggle_edges_during_silence"] += 1
                if not payloads:
                    evidence["pdo_silence_without_feedback"] += 1
                    continue
                evidence["pdo_silence_with_old_feedback"] += 1
                if replies[motor_id][index][2] in payloads:
                    evidence["pdo_silence_matching_last_can_reply"] += 1
    if not baseline_pdo:
        raise ValueError("need a valid ENCOS input PDO before the analysis window")

    return dict(schema="forge.encos_passive_capture", version=1, start_epoch_ns=start_ns,
                end_epoch_ns=end_ns, silence_threshold_ns=silence_ns, clock_skew_bound_ns=skew_ns,
                can1=can1, can2=can2, pdo_frames=pdo_frames, pdo_status_byte_87=dict(status_bytes),
                pdo_can_ide_values=dict(ide_values), motors=by_motor)


def timestamp_ns(value):
    if not re.fullmatch(r"\d+(\.\d{1,9})?", value):
        raise argparse.ArgumentTypeError("expected absolute epoch seconds with up to nine decimal places")
    return int(Decimal(value) * NS_PER_SECOND)


def duration_ns(value):
    try:
        nanoseconds = Decimal(value) * NS_PER_MS
        if not nanoseconds.is_finite() or nanoseconds < 0 or nanoseconds != int(nanoseconds):
            raise ValueError
        return int(nanoseconds)
    except (InvalidOperation, ValueError, OverflowError) as exc:
        raise argparse.ArgumentTypeError("expected nonnegative milliseconds at nanosecond resolution") from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcap", type=Path, help="classic Ethernet pcap from the single ENCOS EtherCAT link")
    parser.add_argument("can_log", type=Path, help="candump -L log from both listen-only CAN interfaces")
    parser.add_argument("--can1", required=True, help="candump interface on bridge CAN1")
    parser.add_argument("--can2", required=True, help="candump interface on bridge CAN2")
    parser.add_argument("--start", required=True, type=timestamp_ns, help="window start in absolute epoch seconds")
    parser.add_argument("--end", required=True, type=timestamp_ns, help="window end in absolute epoch seconds")
    parser.add_argument("--clock-skew-ms", required=True, type=duration_ns, help="bound on relative timestamp error")
    parser.add_argument("--silence-ms", type=duration_ns, default=250 * NS_PER_MS)
    args = parser.parse_args(argv)
    try:
        with args.pcap.open("rb") as capture, args.can_log.open(encoding="utf-8") as can:
            report = analyze(pcap_frames(capture), can, can1=args.can1, can2=args.can2,
                             start_ns=args.start, end_ns=args.end,
                             skew_ns=args.clock_skew_ms, silence_ns=args.silence_ms)
    except (OSError, ValueError) as exc:
        parser.exit(2, f"{parser.prog}: {exc}\n")
    print(json.dumps(report, indent=2))
    return 0
