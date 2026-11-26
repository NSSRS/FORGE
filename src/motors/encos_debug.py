"""Compare optional bridge debug bytes with reply payloads in a saved probe run."""
import argparse
from dataclasses import dataclass, field
import json
from pathlib import Path

from motors.encos_capture import CAN_RECORD, INPUT_BYTES, OUTPUT_BYTES


@dataclass
class Stage:
    name: str
    target: int
    received: bytearray = field(default_factory=bytearray)
    payloads: set = field(default_factory=set)


def inspect_debug(report):
    """Search within each host polling stage. This does not infer the serial framing."""
    events = (json.loads(line) for line in report)
    start = next(events, {})
    if (not isinstance(start, dict) or start.get("schema") != "forge.encos_feedback_probe"
            or start.get("version") != 1):
        raise ValueError("expected a version-1 ENCOS feedback probe journal")
    stages = []
    serial_bytes = 0
    completed = False
    expected_wkc = None
    stage = None
    closed = False
    for event in events:
        if closed:
            raise ValueError("unexpected data after the end of the probe")
        kind = event["event"]
        if kind == "bridge":
            expected_wkc = event["expected_wkc"]
        elif kind == "stage":
            stage = Stage(event["stage"], event["id"])
            stages.append(stage)
        elif kind == "pdo" and stage and expected_wkc and event["wkc"] == expected_wkc:
            image = bytes.fromhex(event["input_hex"])
            if len(image) != INPUT_BYTES:
                raise ValueError("invalid input image length")
            for motor, rtr, dlc, data in CAN_RECORD.iter_unpack(image[2:OUTPUT_BYTES]):
                if rtr or not 1 <= motor <= 4 or data[0] != 0xa0:
                    continue
                if (dlc, data[1]) in ((6, 1), (8, 30)):
                    stage.payloads.add(data[:dlc])
        elif kind == "debug_rx":
            data = bytes.fromhex(event["hex"])
            serial_bytes += len(data)
            if stage:
                stage.received.extend(data)
        elif kind == "end":
            if serial_bytes != event["debug_rx_bytes"]:
                raise ValueError("debug byte count disagrees with the journal footer")
            completed = event["completed"]
            closed = True

    matches = []
    for stage in stages:
        text = stage.received.lower()
        for payload in sorted(stage.payloads):
            formats = []
            if payload in stage.received:
                formats.append("binary")
            if payload.hex().encode() in text:
                formats.append("hex")
            if payload.hex(" ").encode() in text:
                formats.append("spaced_hex")
            if formats:
                matches.append(dict(stage=stage.name, target_id=stage.target,
                                    payload_hex=payload.hex(), formats=formats))
    if not start["serial_requested"]:
        outcome = "debug_port_not_selected"
    elif not serial_bytes:
        outcome = "no_bytes_observed"
    elif matches:
        outcome = "matching_reply_payloads_observed"
    else:
        outcome = "bytes_observed_without_recognized_payloads"
    return dict(outcome=outcome, completed=completed, footer_present=closed, debug_rx_bytes=serial_bytes,
                same_stage_matches=matches, can_freshness_verified=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("journal", type=Path)
    args = parser.parse_args(argv)
    try:
        with args.journal.open() as stream:
            result = inspect_debug(stream)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, f"{parser.prog}: {error}\n")
    print(json.dumps(result, indent=2))
    return 0 if result["completed"] else 1
