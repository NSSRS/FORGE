"""Offline HOLD-01 summaries from the existing cached-feedback JSONL journal."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stats(values):
    values = [v for v in values if finite(v)]
    if not values:
        return None
    return dict(mean=statistics.fmean(values), minimum=min(values), maximum=max(values),
                peak_abs=max(map(abs, values)), rms=math.sqrt(statistics.fmean(v*v for v in values)))


def joint_summary(samples):
    def field(name, degrees=False):
        return [math.degrees(s[name]) if degrees else s[name]
                for s in samples if finite(s.get(name))]

    times = field("at_monotonic_s")
    errors = [math.degrees(s["target_rad"]-s["position_rad"]) for s in samples
              if finite(s.get("target_rad")) and finite(s.get("position_rad"))]
    targets = field("target_rad", True)
    speeds = field("requested_speed_rad_s", True)
    def temperature_change(name):
        rows = [s for s in samples if finite(s.get(name)) and finite(s.get("at_monotonic_s"))]
        if not rows:
            return None
        span = rows[-1]["at_monotonic_s"]-rows[0]["at_monotonic_s"]
        return dict(first_c=rows[0][name], last_c=rows[-1][name], observed_span_s=span,
                    change_c=rows[-1][name]-rows[0][name] if span > 0 else None)
    return dict(
        samples=len(samples), observed_span_s=times[-1]-times[0] if len(times) > 1 else 0,
        max_sample_gap_s=max((b-a for a, b in zip(times, times[1:])), default=None),
        velocity_deg_s=stats(field("velocity_rad_s", True)),
        absolute_speed_deg_s=stats([abs(v) for v in field("velocity_rad_s", True)]),
        torque_feedback_nm=stats(field("torque_feedback_nm")),
        command_error_deg=stats(errors),
        target_span_deg=max(targets)-min(targets) if targets else None,
        speed_allowance_span_deg_s=max(speeds)-min(speeds) if speeds else None,
        mos_c=stats(field("mos_c")), rotor_c=stats(field("rotor_c")),
        temperature_change_c={name: temperature_change(name) for name in ("mos_c", "rotor_c")},
        effective_feedback_age_ms=stats(field("effective_age_ms")),
    )


def summarize(path, *, label=None, ee_mass_kg=None):
    """Keep each attempt and contiguous stage separate; never infer hardware acceptance."""
    path = Path(path)
    session = None
    arm_configuration = None
    plan = None
    attempts, segments, faults = [], [], []
    attempt = segment = end = None
    seen = set()
    incomplete_tail = False
    dropped = 0
    last_time = None
    with path.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                # A live writer may not have completed its final line yet.
                if not line.endswith("\n") and stream.read() == "":
                    incomplete_tail = True
                    break
                raise ValueError(f"Invalid JSONL at line {number}") from None
            kind = row.get("kind")
            if kind == "session":
                if session is not None or row.get("schema") != "forge.weld_motion_journal":
                    raise ValueError("Expected one weld-motion session per journal")
                session = row
            elif kind == "accepted_plan":
                plan = row.get("plan")
            elif kind == "arm_configuration":
                arm_configuration = row.get("profile")
            elif kind == "session_end":
                end = row
            dropped = max(dropped, row.get("dropped_records", 0))
            if row.get("fault") and row["fault"] not in faults:
                faults.append(row["fault"])
            t = row.get("time_s")
            if not finite(t):
                continue
            if last_time is not None and t < last_time:
                raise ValueError("Journal sample times are not monotonic")
            last_time = t
            for event in row.get("events", []):
                name, at = event.get("event"), event.get("time_s")
                if name == "recovery_requested":
                    attempt = dict(number=len(attempts)+1, requested_s=at, ready_s=None,
                                   time_to_ready_s=None, endpoint_recovery_s=None,
                                   time_to_ready_after_endpoint_s=None,
                                   normal_preparation_to_ready_s=None, outcome="incomplete", plan=plan)
                    attempts.append(attempt)
                elif attempt and attempt["outcome"] == "incomplete":
                    if name == "hold_verified" and event.get("state") == "initial":
                        attempt["normal_preparation_started_s"] = at
                    if name == "endpoint_recovery_completed":
                        attempt["endpoint_recovery_s"] = event.get("duration_s")
                        attempt["endpoint_completed_s"] = at
                        attempt["normal_preparation_started_s"] = at
                    if name == "stage_arrived" and event.get("state") == "ready":
                        attempt.update(ready_s=at, time_to_ready_s=at-attempt["requested_s"],
                                       outcome="ready")
                        if attempt.get("endpoint_completed_s") is not None:
                            attempt["time_to_ready_after_endpoint_s"] = at-attempt["endpoint_completed_s"]
                        if attempt.get("normal_preparation_started_s") is not None:
                            attempt["normal_preparation_to_ready_s"] = at-attempt["normal_preparation_started_s"]
                    elif name in ("pause", "fault", "arm_disabled"):
                        attempt["outcome"] = "interrupted"
            arm = row.get("arm") or {}
            state = row.get("state")
            active = kind == "sample" and arm.get("holding") is True
            key = (len(attempts), state, arm.get("endpoint_hold")) if active else None
            if segment is not None and segment["key"] != key:
                segment = None
            if not active:
                continue
            if segment is None:
                segment = dict(key=key, attempt=len(attempts), state=state,
                               endpoint_hold=arm.get("endpoint_hold"),
                               rows={str(i): [] for i in range(1, 7)}, rejected=0, duplicates=0)
                segments.append(segment)
            for joint in map(str, range(1, 7)):
                sample = (arm.get("last_samples") or {}).get(joint)
                if not sample:
                    segment["rejected"] += 1
                    continue
                at = sample.get("at_monotonic_s")
                age = sample.get("age_ms")
                if not finite(at) or not finite(age):
                    segment["rejected"] += 1
                    continue
                identity = (joint, at)
                if identity in seen:
                    segment["duplicates"] += 1
                    continue
                seen.add(identity)
                effective_age = age + max(0, t-at)*1000
                if (at > t or age < 0 or effective_age >= 80 or sample.get("enabled") is not True
                        or sample.get("enable_confirmed") is not True or sample.get("status") != 1):
                    segment["rejected"] += 1
                    continue
                segment["rows"][joint].append(dict(sample, effective_age_ms=effective_age))
    if session is None:
        raise ValueError("Missing weld-motion session header")
    for item in segments:
        item.pop("key")
        item["joints"] = {joint: joint_summary(rows) for joint, rows in item.pop("rows").items()}
        item["minimum_axis_span_s"] = min(j["observed_span_s"] for j in item["joints"].values())
        # Report evidence, not a blanket pass: support, physical motion and load are operator observations.
        item["all_axes_within_025_deg"] = all(
            j["command_error_deg"] is not None and j["command_error_deg"]["peak_abs"] <= .25
            for j in item["joints"].values())
    for item in attempts:
        p = item.pop("plan") or {}
        item["plan"] = dict(
            mapping_fingerprint=p.get("mapping_fingerprint"),
            scene_fingerprint=p.get("scene_fingerprint"), current_raw_deg=p.get("current_raw_deg"),
            initial_raw_deg=p.get("initial_raw_deg"), preparation_timing=p.get("preparation_timing"),
            path_fingerprint=fingerprint({k: p.get(k) for k in ("endpoint_recovery", "home", "recovery", "approach", "weld", "stage_limits")}) if p else None,
        )
    arm_end = (end or {}).get("arm_session") or {}
    return dict(
        schema="forge.hold_comparison", version=1, journal=str(path.resolve()),
        label=label, end_effector_mass_kg=ee_mass_kg, mass_basis="operator estimate" if ee_mass_kg else None,
        simulated=session.get("simulated"), settings=session.get("settings"), tracker=session.get("tracker"),
        arm_configuration=arm_configuration,
        mapping_fingerprint=fingerprint(session.get("mapping")),
        initial_raw_deg=session.get("initial_raw_deg"), attempts=attempts, segments=segments,
        faults=faults, dropped_records=dropped, incomplete_final_line=incomplete_tail,
        session_closed=end is not None,
        disable_acknowledged=arm_end.get("disable_acknowledged"),
        mode_restoration_verified=arm_end.get("mode_restoration_verified"),
        hardware_qualified=False,
        limitations=["Torque is drive-reported feedback, not independently calibrated shaft torque.",
                     "Statistics are sample-weighted; gaps and rejected samples remain explicit.",
                     "Match physical start, route, settings, support contact and temperatures before attributing changes to payload.",
                     "An open journal has no final shutdown result; summary does not stop or contact the motor owner."],
    )


def compare(before, after):
    checks = {k: before[k] == after[k] for k in ("settings", "tracker", "mapping_fingerprint", "initial_raw_deg", "simulated")}
    checks["same_arm_configuration"] = (before.get("arm_configuration") == after.get("arm_configuration")
                                        if before.get("arm_configuration") is not None and after.get("arm_configuration") is not None
                                        else None)
    paths = lambda r: [a["plan"]["path_fingerprint"] for a in r["attempts"]]
    checks["identical_planned_paths"] = bool(paths(before)) and paths(before) == paths(after) and all(paths(before))
    checks["same_scene"] = bool(before["attempts"] and after["attempts"]) and [
        a["plan"]["scene_fingerprint"] for a in before["attempts"]] == [
        a["plan"]["scene_fingerprint"] for a in after["attempts"]] and all(
            a["plan"]["scene_fingerprint"] for a in before["attempts"])
    return dict(checks=checks, interpretation="Review stage tables and physical setup; matching metadata alone does not isolate the payload effect.")


def print_summary(result):
    print(f"\n{result['label'] or result['journal']} | EE mass: {result['end_effector_mass_kg']} kg")
    for attempt in result["attempts"]:
        duration = attempt["time_to_ready_s"]
        print(f"Attempt {attempt['number']}: {attempt['outcome']}; recovery request to Ready: {duration if duration is not None else 'unavailable'} s")
    def value(stat, name):
        return "—" if stat is None else f"{stat[name]:.3f}"
    for segment in result["segments"]:
        print(f"  Attempt {segment['attempt']} / {segment['state']}: minimum axis span {segment['minimum_axis_span_s']:.2f} s; rejected {segment['rejected']}; duplicate cached samples {segment['duplicates']}")
        print("  Axis  Mean |speed|  Peak |speed|  Mean torque  RMS torque  Peak |torque|  Peak |error|  Peak MOS  Peak motor")
        print("             deg/s         deg/s         Nm          Nm             Nm           deg       degC        degC")
        for joint, s in segment["joints"].items():
            numbers = [value(s["absolute_speed_deg_s"], "mean"), value(s["velocity_deg_s"], "peak_abs"),
                       value(s["torque_feedback_nm"], "mean"), value(s["torque_feedback_nm"], "rms"),
                       value(s["torque_feedback_nm"], "peak_abs"), value(s["command_error_deg"], "peak_abs"),
                       value(s["mos_c"], "maximum"), value(s["rotor_c"], "maximum")]
            print(f"  A{joint} " + " ".join(f"{n:>12}" for n in numbers))
            for name, title in (("mos_c", "MOS"), ("rotor_c", "motor")):
                change = s["temperature_change_c"][name]
                if change is not None and change["change_c"] is not None:
                    print(f"     {title}: {change['first_c']:g} -> {change['last_c']:g} degC "
                          f"({change['change_c']:+g} degC over {change['observed_span_s']:.2f} s)")
    print(f"Closed: {result['session_closed']}; disable acknowledged: {result['disable_acknowledged']}; mode restored: {result['mode_restoration_verified']}; dropped records: {result['dropped_records']}; incomplete tail: {result['incomplete_final_line']}")
    for fault in result["faults"]:
        print(f"Fault: {fault}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("journal", type=Path)
    parser.add_argument("--label")
    parser.add_argument("--ee-mass-kg", type=float)
    parser.add_argument("--compare", type=Path)
    parser.add_argument("--compare-label")
    parser.add_argument("--compare-ee-mass-kg", type=float)
    parser.add_argument("--output", type=Path, help="New JSON summary; existing files are never overwritten")
    args = parser.parse_args(argv)
    for mass in (args.ee_mass_kg, args.compare_ee_mass_kg):
        if mass is not None and (not finite(mass) or mass < 0):
            parser.error("EE mass must be finite and nonnegative")
    first = summarize(args.journal, label=args.label, ee_mass_kg=args.ee_mass_kg)
    results = dict(runs=[first])
    print_summary(first)
    if args.compare:
        second = summarize(args.compare, label=args.compare_label, ee_mass_kg=args.compare_ee_mass_kg)
        results["runs"].append(second)
        results["comparison"] = compare(first, second)
        print_summary(second)
        print("Comparison checks:", json.dumps(results["comparison"]["checks"], sort_keys=True))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as stream:
            json.dump(results, stream, indent=2, allow_nan=False)
            stream.write("\n")
    return 0
