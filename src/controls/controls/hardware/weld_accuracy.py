"""Measure weld tracking offline from the controller's cached journal."""

import argparse
import csv
from dataclasses import dataclass, field
import json
from pathlib import Path

import numpy as np

from controls.controller.weld_plan import JointPath, finite_vector, positive
from motors.arm_calibration import CalibrationMap, DRIVES
from .hold_analysis import finite, stats


# Match the existing controller's feedback and tick deadlines.
MAX_FEEDBACK_AGE_MS = 80.0
MAX_GAP_S = 0.100


@dataclass
class TraceSample:
    elapsed_s: float
    measured_deg: np.ndarray
    dispatched_deg: np.ndarray
    reference_deg: np.ndarray
    age_ms: np.ndarray


@dataclass
class WeldPass:
    plan: dict
    started_s: float
    continuous: bool
    commands: list = field(default_factory=list)
    outcome: str = "incomplete"

    def collect(self, row):
        trace = row.get("last_command") or {}
        at = trace.get("time_s")
        if trace.get("stage") != "welding" or (finite(at) and at < self.started_s):
            return
        # The journal repeats its last command while waiting, pausing and closing.
        if self.commands and finite(at) and at == self.commands[-1].get("time_s"):
            return
        self.commands.append(trace)


def read_journal(path):
    session = plan = current = None
    passes = []
    closed = partial = False
    dropped = 0
    fault = None
    with path.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                if not line.endswith("\n"):
                    partial = True
                    break
                raise ValueError(f"Invalid JSONL at line {number}") from None
            if not isinstance(row, dict):
                raise ValueError(f"Expected a journal object at line {number}")
            kind = row.get("kind")
            count = row.get("dropped_records", 0)
            if type(count) is not int or count < 0:
                raise ValueError(f"Invalid dropped-record count at line {number}")
            dropped = max(dropped, count)
            if kind == "session":
                if session is not None or (
                    row.get("schema"), row.get("version")
                ) != ("forge.weld_motion_journal", 1):
                    raise ValueError("Expected one version-1 weld motion journal")
                if type(row.get("simulated")) is not bool:
                    raise ValueError("Journal must identify its feedback as simulated or hardware")
                session = row
            elif kind == "accepted_plan":
                plan = row.get("plan")
            elif kind == "session_end":
                closed = True
                fault = row.get("fault")
            elif kind in ("sample", "before_close"):
                events = row.get("events", [])
                for event in events:
                    if event.get("event") == "weld_started":
                        started = event.get("time_s")
                        continuous = event.get("continuous")
                        if not isinstance(plan, dict) or not finite(started) or type(continuous) is not bool:
                            raise ValueError("Weld start is missing its plan, clock or repeat mode")
                        current = WeldPass(plan, started, continuous)
                        passes.append(current)
                if current is None:
                    continue
                current.collect(row)
                if row.get("fault"):
                    current.outcome = "fault"
                    current = None
                    continue
                for event in events:
                    name = event.get("event")
                    if name == "stage_arrived" and event.get("from_stage") == "weld" and event.get("state") == "complete":
                        current.outcome = "complete"
                        current = None
                        break
                    if name in ("pause", "fault", "arm_disabled"):
                        current.outcome = name
                        current = None
                        break
    if session is None:
        raise ValueError("Missing weld journal session header")
    return session, passes, closed, partial, dropped, fault


def sample_commands(run, path):
    """Use unique dispatched ticks; reject stale or unpaired feedback."""
    samples = []
    rejected = 0
    last_clock = -1.0
    last_elapsed = -1.0
    entry = JointPath.from_document(run.plan["weld_entry"]) if run.plan.get("weld_entry") else None
    for trace in run.commands:
        try:
            at = trace.get("time_s")
            elapsed = trace.get("elapsed_s")
            if not finite(at) or not finite(elapsed) or elapsed < 0:
                raise ValueError("Invalid weld trace clock")
            if at <= last_clock or elapsed <= last_elapsed or abs(at - elapsed - run.started_s) > MAX_GAP_S:
                raise ValueError("Trace time does not belong to this pass")
            last_clock, last_elapsed = at, elapsed
            # End-of-pass settling uses the final hold command and is outside
            # the moving interval when computing accuracy.
            if not run.continuous and elapsed > path.duration:
                continue
            age = finite_vector(trace.get("age_ms"), "age")
            measured = finite_vector(trace.get("measured_raw_deg"), "measured pose")
            commanded = finite_vector(trace.get("command_raw_deg"), "dispatched pose")
            nominal = finite_vector(trace.get("requested_raw_deg"), "reference pose")
            phase = elapsed % path.duration if run.continuous else elapsed
            nominal_path = entry if entry is not None and elapsed < entry.duration else path
            if (
                trace.get("dispatch_state") != "sent"
                or np.any((age < 0) | (age >= MAX_FEEDBACK_AGE_MS))
                or not np.allclose(nominal, nominal_path.sample(phase)[0], atol=1e-6, rtol=0)
            ):
                raise ValueError("Invalid or stale weld feedback")
        except (TypeError, ValueError):
            rejected += 1
            continue
        samples.append(TraceSample(elapsed, measured, commanded, nominal, age))
    return samples, rejected


def coverage(times, duration):
    intervals = np.diff([0.0, *times, duration])
    maximum = float(np.max(intervals))
    return dict(samples=len(times), first_s=times[0] if times else None,
                last_s=times[-1] if times else None, max_gap_ms=maximum * 1000,
                complete=len(times) >= 2 and maximum <= MAX_GAP_S + 1e-9)


class TipModel:
    """Reconstruct the tool in the planner's fixed-base frame with D3 closed."""

    def __init__(self, mapping, identity):
        from controls.controller.planning_cache import scene_copy
        from simulation.arm_closure import ArmClosure

        self.mapping = mapping
        self.model, self.data = scene_copy(identity)
        self.closure = ArmClosure(self.model, self.data, forward=False)
        self.addresses = [int(self.model.joint(name).qposadr[0]) for name in DRIVES]

    def tip_pose(self, raw):
        angles = [self.mapping.model(joint, angle) for joint, angle in enumerate(raw.tolist(), 1)]
        self.data.qpos[self.addresses] = np.radians(angles)
        if not self.closure.apply(self.data):
            raise ValueError("Cannot reconstruct the D3-closed pose")
        site = self.data.site("wire_tip")
        return site.xpos.copy() * 1000, site.xmat.copy().reshape(3, 3)

    def tip_mm(self, raw):
        return self.tip_pose(raw)[0]


def orientation_error_deg(reference, measured):
    """Smallest full 3D rotation from planned to measured tool orientation."""
    relative = np.asarray(reference).reshape(3, 3).T @ np.asarray(measured).reshape(3, 3)
    skew = np.array([relative[2, 1] - relative[1, 2],
                     relative[0, 2] - relative[2, 0],
                     relative[1, 0] - relative[0, 1]])
    return float(np.degrees(np.arctan2(np.linalg.norm(skew) / 2,
                                      (np.trace(relative) - 1) / 2)))


def tip_statistics(errors):
    points = np.asarray(errors, dtype=float).reshape(-1, 3)
    return dict(samples=len(points), along_mm=stats(points[:, 0]),
                transverse_mm=stats(points[:, 1]), height_mm=stats(points[:, 2]),
                distance_mm=stats(np.linalg.norm(points, axis=1)))


def read_tip_csv(csv_path, duration):
    times, positions = [], []
    with csv_path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = ["elapsed_s", "x_mm", "y_mm", "z_mm"]
        if reader.fieldnames != fields:
            raise ValueError("Tip CSV requires elapsed_s,x_mm,y_mm,z_mm in that order")
        for row in reader:
            try:
                elapsed, *position = [float(row[name]) for name in fields]
                if (
                    None in row or not np.isfinite([elapsed, *position]).all()
                    or not 0 <= elapsed <= duration
                    or (times and elapsed <= times[-1])
                ):
                    raise ValueError
            except (ValueError, TypeError):
                raise ValueError(f"Invalid synchronized tip data at CSV line {reader.line_num}") from None
            times.append(elapsed)
            positions.append(position)
    return times, np.asarray(positions, dtype=float).reshape(-1, 3)


def measured_tip_errors(times, positions, run, path, model, duration, feed_mm_s):
    origin = model.tip_mm(path.positions[0])
    errors = []
    entry = JointPath.from_document(run.plan["weld_entry"]) if run.plan.get("weld_entry") else None
    for elapsed, position in zip(times, positions):
        phase = elapsed % path.duration if run.continuous else elapsed
        nominal_path = entry if entry is not None and elapsed < entry.duration else path
        reference = model.tip_mm(nominal_path.sample(phase)[0]) - origin
        reference[0] += feed_mm_s * elapsed
        errors.append(position - reference)
    return dict(coverage=coverage(times, duration), **tip_statistics(errors))


def tip_cycle_strokes(period, times, transverse, planned_low, planned_high):
    """Sampled extrema in complete periods, distinct from the run envelope."""
    times, transverse = np.asarray(times), np.asarray(transverse)
    if len(times) < 2:
        return dict(status="insufficient complete cycles", cycles=[])
    # Permit a boundary sample offset of at most one normal coverage gap.
    # Do not include the partial final cycle or recover missing interior data.
    margin = min(MAX_GAP_S, period / 4)
    first = max(0, int(np.ceil((times[0] - margin) / period)))
    last = int(np.floor((times[-1] + margin) / period))
    cycles = []
    for index in range(first, last):
        mask = (times >= index*period) & (times < (index+1)*period)
        t, values = times[mask], transverse[mask]
        if (len(t) < 4 or t[0] - index*period > margin
                or (index+1)*period - t[-1] > margin or np.max(np.diff(t)) > MAX_GAP_S):
            continue
        low, high = float(min(values)), float(max(values))
        cycles.append(dict(cycle=index+1, samples=len(values), measured_width_mm=high-low,
                           lower_edge_outward_error_mm=planned_low-low,
                           upper_edge_outward_error_mm=high-planned_high))
    widths = [row["measured_width_mm"] for row in cycles]
    return dict(status="available" if cycles else "insufficient complete cycles",
                basis="Sampled extrema per complete cycle; between-sample peaks are not measured.",
                planned_width_mm=planned_high-planned_low, count=len(cycles),
                width_mm=stats(widths), median_width_mm=float(np.median(widths)) if widths else None,
                cycles=cycles)


def tip_lag_report(path, model, period, times, positions, same_time, source):
    from scipy.optimize import minimize_scalar

    if not finite(period) or not 0 < period <= path.duration + 1e-9:
        return dict(status="a valid weave period is required")
    if times[-1] - times[0] < period:
        return dict(status="at least one weave period of tip samples is required")

    # Evaluate the accepted periodic path in the same workpiece frame as the
    # CSV. One time shift applies to all three axes, with no spatial fit.
    times = np.asarray(times)
    origin = model.tip_mm(path.positions[0])
    grid = np.linspace(0, path.duration, max(2049, len(path.times)))
    planned = np.array([model.tip_mm(path.sample(t)[0]) - origin for t in grid])
    planned_low, planned_high = float(planned[:, 1].min()), float(planned[:, 1].max())
    if planned_high - planned_low <= 1e-6:
        return dict(status="the planned tip has no transverse stroke")

    def errors(lag):
        phase = (times - lag) % path.duration
        shifted = np.column_stack([np.interp(phase, grid, axis) for axis in planned.T])
        return positions - shifted

    def objective(lag):
        return float(np.mean(np.sum(errors(lag) ** 2, axis=1)))

    # A weave can have dwell and multiple local minima. Find a coarse minimum
    # over one fundamental period and refine inside the neighboring interval.
    candidates = np.linspace(-period / 2, period / 2, 129)
    best = int(np.argmin([objective(lag) for lag in candidates]))
    bounds = candidates[max(0, best - 1)], candidates[min(len(candidates) - 1, best + 1)]
    refined = minimize_scalar(objective, bounds=bounds, method="bounded")
    lag = min((candidates[best], refined.x), key=objective)
    measured_low, measured_high = float(positions[:, 1].min()), float(positions[:, 1].max())
    return dict(
        status="available", source=source, lag_ms=float(1000 * lag),
        same_time_error=same_time, aligned_error=tip_statistics(errors(lag)),
        stroke=dict(
            planned_width_mm=planned_high - planned_low,
            measured_width_mm=measured_high - measured_low,
            lower_edge_outward_error_mm=planned_low - measured_low,
            upper_edge_outward_error_mm=measured_high - planned_high,
        ),
        cycle_stroke=tip_cycle_strokes(period, times, positions[:, 1], planned_low, planned_high),
    )


def analyze(journal, *, run_number=None, tip_csv=None, peak_joint_deg=None, peak_tip_mm=None,
            orientation_target_deg=None, max_orientation_deg=None, fit_tip_lag=False):
    from simulation.arm_calibration import scene_fingerprint
    from simulation.paths import MODEL_DIR

    for label, limit in (("Peak joint limit", peak_joint_deg), ("Peak tip limit", peak_tip_mm),
                         ("Orientation target", orientation_target_deg),
                         ("Maximum orientation error", max_orientation_deg)):
        if limit is not None:
            positive(limit, label)
    if (orientation_target_deg is not None and max_orientation_deg is not None
            and orientation_target_deg > max_orientation_deg):
        raise ValueError("Orientation target must not exceed maximum orientation error")
    if peak_tip_mm is not None and not tip_csv:
        raise ValueError("--peak-tip-mm requires independent --tip-csv measurements")
    journal = Path(journal)
    session, runs, closed, partial, dropped, fault = read_journal(journal)
    if not runs:
        raise ValueError("The journal contains no weld start")
    if run_number is None:
        if len(runs) != 1:
            raise ValueError(f"The journal contains {len(runs)} weld starts; choose --run-number")
        run_number = 1
    if not 1 <= run_number <= len(runs):
        raise ValueError(f"Run number must be in 1..{len(runs)}")

    run = runs[run_number - 1]
    plan = run.plan
    if (plan.get("schema"), plan.get("version")) != ("forge.weld_motion_plan", 1) or plan.get("arm_enabled") is not True:
        raise ValueError("The pass needs an accepted version-1 six-axis weld plan")
    chassis_selected = plan.get("chassis_enabled")
    feed = plan.get("feed_m_s")
    if type(chassis_selected) is not bool or not finite(feed) or feed < 0:
        raise ValueError("Invalid planned chassis feed")
    feed_mm_s = feed * 1000 if chassis_selected else 0.0
    mapping = CalibrationMap(plan.get("mapping", session["mapping"]))
    if plan.get("mapping_fingerprint") != mapping.fingerprint:
        raise ValueError("The weld plan and journal use different encoder mappings")
    path = JointPath.from_document(plan["weld"])
    samples, rejected = sample_commands(run, path)
    times = [sample.elapsed_s for sample in samples]
    duration = times[-1] if run.continuous and times else path.duration
    gaps = coverage(times, duration)
    issues = []
    if run.outcome != "complete":
        issues.append(f"Pass outcome: {run.outcome}")
    if not closed or partial or dropped or fault:
        issues.append("Journal is unfinished, has dropped data or records a final fault")
    if rejected or not gaps["complete"]:
        issues.append("Weld trace has rejected samples or gaps over 100 ms")

    joints = {}
    for axis in range(6):
        measured = np.array([sample.measured_deg[axis] for sample in samples])
        reference = np.array([sample.reference_deg[axis] for sample in samples])
        dispatched = np.array([sample.dispatched_deg[axis] for sample in samples])
        joints[f"A{axis + 1}"] = dict(
            tracking_error_deg=stats(measured - reference),
            dispatch_error_deg=stats(measured - dispatched),
            measured_travel_deg=float(np.ptp(measured)) if samples else None,
            reference_travel_deg=float(np.ptp(reference)) if samples else None,
            feedback_age_ms=stats([sample.age_ms[axis] for sample in samples]),
        )

    encoder_tip = independent_tip = encoder_orientation = None
    lag_report = dict(status="a continuous arm-only run is required")
    lag_eligible = fit_tip_lag and run.continuous and not chassis_selected
    model_status = "scene fingerprint differs from the accepted plan"
    if lag_eligible:
        lag_report = dict(status=model_status)
    identity = scene_fingerprint(MODEL_DIR / "robot.xml")
    if identity == plan.get("scene_fingerprint"):
        model = TipModel(mapping, identity)
        errors, angles, encoder_positions = [], [], []
        for sample in samples:
            # The model holds the base fixed, including when chassis feed ran.
            try:
                measured_position, measured_orientation = model.tip_pose(sample.measured_deg)
                reference_position, reference_orientation = model.tip_pose(sample.reference_deg)
                errors.append(measured_position - reference_position)
                angles.append(orientation_error_deg(reference_orientation, measured_orientation))
                if lag_eligible and not tip_csv:
                    encoder_positions.append(measured_position)
            except ValueError:
                continue
        encoder_tip = dict(rejected=len(samples) - len(errors), **tip_statistics(errors))
        encoder_orientation = dict(
            samples=len(angles), rejected=len(samples) - len(angles), angle_deg=stats(angles),
            definition="smallest full 3D rotation: planned wire_tip frame to encoder-reconstructed frame",
            within_target_percent=(100 * sum(angle <= orientation_target_deg for angle in angles) / len(angles)
                                   if angles and orientation_target_deg is not None else None),
            within_maximum_percent=(100 * sum(angle <= max_orientation_deg for angle in angles) / len(angles)
                                    if angles and max_orientation_deg is not None else None),
        )
        model_status = "encoder-reconstructed, fixed base"
        if encoder_tip["rejected"]:
            issues.append("Some encoder poses could not be reconstructed")
        if tip_csv:
            tip_times, tip_positions = read_tip_csv(Path(tip_csv), duration)
            independent_tip = measured_tip_errors(
                tip_times, tip_positions, run, path, model, duration, feed_mm_s,
            )
            if not independent_tip["coverage"]["complete"]:
                issues.append("Independent tip data has gaps over 100 ms or insufficient samples")
        if lag_eligible:
            lag_report = dict(status="complete, fault-free tip and journal coverage is required")
            source = ("independent" if tip_csv else "encoder_reconstructed")
            if tip_csv:
                trace_complete = independent_tip["coverage"]["complete"]
                same_time = independent_tip
            else:
                trace_complete = not encoder_tip["rejected"]
                same_time = encoder_tip
            if (
                trace_complete and gaps["complete"] and closed
                and not any((partial, dropped, fault, rejected))
                and run.outcome in ("complete", "pause")
            ):
                if not tip_csv:
                    tip_times = times
                    tip_positions = np.asarray(encoder_positions) - model.tip_mm(path.positions[0])
                excluded_startup = 0
                if plan.get("weld_entry"):
                    entry_duration = float(plan["weld_entry"]["times_s"][-1])
                    keep = np.asarray(tip_times) >= entry_duration
                    excluded_startup = int(np.sum(~keep))
                    tip_times = np.asarray(tip_times)[keep].tolist()
                    tip_positions = np.asarray(tip_positions)[keep]
                    origin = model.tip_mm(path.positions[0])
                    same_time = tip_statistics([
                        p - (model.tip_mm(path.sample(t % path.duration)[0]) - origin)
                        for t, p in zip(tip_times, tip_positions)
                    ])
                hz = plan.get("effective_hz")
                lag_report = tip_lag_report(
                    path, model, 1 / hz if finite(hz) and hz > 0 else None,
                    tip_times, tip_positions, same_time, source,
                ) if len(tip_times) >= 2 else dict(status="insufficient steady cycles after startup")
                lag_report["excluded_startup_samples"] = excluded_startup
    elif tip_csv:
        raise ValueError("Independent tip analysis requires the exact model used to plan the pass")

    joint_peaks = [item["tracking_error_deg"]["peak_abs"] for item in joints.values() if item["tracking_error_deg"]]
    joint_check = None if peak_joint_deg is None else bool(joint_peaks and max(joint_peaks) <= peak_joint_deg)
    tip_check = None if peak_tip_mm is None else bool(
        independent_tip and independent_tip["distance_mm"]
        and independent_tip["distance_mm"]["peak_abs"] <= peak_tip_mm
    )
    orientation_peak = (encoder_orientation["angle_deg"]["peak_abs"]
                        if encoder_orientation and encoder_orientation["angle_deg"] else None)
    orientation_goal = (None if orientation_target_deg is None else
                        bool(orientation_peak is not None and orientation_peak <= orientation_target_deg))
    orientation_check = (None if max_orientation_deg is None else
                         bool(orientation_peak is not None and orientation_peak <= max_orientation_deg))
    # The preferred target is reported separately; maximum is the acceptance cap.
    checks = [value for value in (joint_check, tip_check, orientation_check) if value is not None]
    report = dict(
        schema="forge.weld_motion_accuracy", version=1, journal=str(journal.resolve()),
        simulated=session.get("simulated"), run_number=run_number, outcome=run.outcome,
        duration_s=duration, effective_hz=plan.get("effective_hz"),
        chassis_feed_selected=chassis_selected, nominal_chassis_feed_mm_s=feed_mm_s,
        coverage=gaps, rejected_samples=rejected, dropped_records=dropped, session_closed=closed,
        mapping_fingerprint=mapping.fingerprint, scene_fingerprint=plan.get("scene_fingerprint"),
        joints=joints, encoder_tip_status=model_status, encoder_tip=encoder_tip,
        encoder_orientation=encoder_orientation,
        independent_tip=independent_tip, tip_csv=str(Path(tip_csv).resolve()) if tip_csv else None,
        issues=issues, hardware_qualified=False,
        peak_joint_limit_deg=peak_joint_deg, peak_tip_limit_mm=peak_tip_mm,
        joint_limit_met=joint_check, independent_tip_limit_met=tip_check,
        orientation_target_deg=orientation_target_deg, max_orientation_deg=max_orientation_deg,
        orientation_target_met=orientation_goal, orientation_maximum_met=orientation_check,
        passed=None if not checks else not issues and all(checks),
    )
    if fit_tip_lag:
        report["tip_lag"] = lag_report
    return report


def print_summary(report):
    source = "simulation" if report["simulated"] else "hardware journal"
    evidence = report["coverage"]
    print(f"Weld run {report['run_number']} ({source}): {report['outcome']}, "
          f"{report['duration_s']:.3f} s, {evidence['samples']} usable ticks, "
          f"max gap {evidence['max_gap_ms']:.1f} ms")
    for axis, row in report["joints"].items():
        error = row["tracking_error_deg"]
        if error:
            print(f"  {axis}: reference tracking RMS {error['rms']:.3f}°, peak {error['peak_abs']:.3f}°")
    for name, label in (("encoder_tip", "Encoder-estimated tip, fixed base"),
                        ("independent_tip", "Independently measured tip, workpiece frame")):
        item = report[name]
        if item and item["distance_mm"]:
            error = item["distance_mm"]
            print(f"  {label}: 3D RMS {error['rms']:.3f} mm, peak {error['peak_abs']:.3f} mm")
    if report["encoder_tip"] is None:
        print(f"  Tip estimate unavailable: {report['encoder_tip_status']}")
    lag = report.get("tip_lag")
    if lag and lag["status"] == "available":
        error = lag["aligned_error"]["distance_mm"]
        stroke = lag["stroke"]
        source_label = ("independent measurements" if lag["source"] == "independent" else "encoder estimate")
        print(f"  Tip with one fixed lag ({source_label}): {lag['lag_ms']:+.1f} ms, "
              f"3D RMS {error['rms']:.3f} mm, peak {error['peak_abs']:.3f} mm")
        print(f"    Full-window transverse envelope: {stroke['measured_width_mm']:.3f} mm measured, "
              f"{stroke['planned_width_mm']:.3f} mm planned")
        print(f"    Edge errors (positive is overshoot): "
              f"lower {stroke['lower_edge_outward_error_mm']:+.3f} mm, "
              f"upper {stroke['upper_edge_outward_error_mm']:+.3f} mm")
        cycles = lag.get("cycle_stroke", {})
        if cycles.get("status") == "available":
            widths = cycles["width_mm"]
            print(f"    Sampled stroke across {cycles['count']} complete cycles: "
                  f"median {cycles['median_width_mm']:.3f} mm, "
                  f"range {widths['minimum']:.3f}-{widths['maximum']:.3f} mm")
    elif lag:
        print(f"  Tip lag unavailable: {lag['status']}")
    orientation = report.get("encoder_orientation")
    if orientation and orientation["angle_deg"]:
        error = orientation["angle_deg"]
        print(f"  Encoder-estimated full tool orientation: RMS {error['rms']:.3f}°, "
              f"peak {error['peak_abs']:.3f}°")
        for value, limit, label in (("within_target_percent", "orientation_target_deg", "target"),
                                    ("within_maximum_percent", "max_orientation_deg", "maximum")):
            if report[limit] is not None:
                print(f"    Within {report[limit]:g}° {label}: {orientation[value]:.1f}% of usable ticks")
    for issue in report["issues"]:
        print(f"  Evidence: {issue}")
    if report["passed"] is not None:
        print("PASS" if report["passed"] else "FAIL")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("journal", type=Path, help="The completed weld session .jsonl")
    parser.add_argument("--run-number", type=int, help="One-based weld start in a multi-run journal")
    parser.add_argument("--tip-csv", type=Path, help="Synchronized independent tip measurements")
    parser.add_argument("--fit-tip-lag", action="store_true",
                        help="Report one fixed wire-tip lag and stroke errors for a continuous arm-only run")
    parser.add_argument("--peak-joint-deg", type=float, help="Allowed peak nominal tracking error on every axis")
    parser.add_argument("--peak-tip-mm", type=float, help="Allowed measured 3D tip error; requires --tip-csv")
    parser.add_argument("--orientation-target-deg", type=float,
                        help="Preferred full tool-orientation error; report coverage separately from maximum")
    parser.add_argument("--max-orientation-deg", type=float,
                        help="Maximum encoder-estimated full tool-orientation error allowed for acceptance")
    parser.add_argument("--output", type=Path, help="Save the JSON report to a new file")
    args = parser.parse_args(argv)
    try:
        report = analyze(
            args.journal, run_number=args.run_number, tip_csv=args.tip_csv,
            peak_joint_deg=args.peak_joint_deg, peak_tip_mm=args.peak_tip_mm,
            orientation_target_deg=args.orientation_target_deg, max_orientation_deg=args.max_orientation_deg,
            fit_tip_lag=args.fit_tip_lag,
        )
        if args.output:
            payload = json.dumps(report, indent=2, allow_nan=False)
            with args.output.open("x") as stream:
                stream.write(payload + "\n")
        print_summary(report)
        return 2 if report["issues"] or report["passed"] is False else 0
    except (OSError, KeyError, TypeError, ValueError) as exc:
        parser.exit(2, f"Accuracy analysis failed: {exc}\n")
