"""Seeded, hardware-free planner experiments against the actual arm model."""

import argparse
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import asdict
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import platform
import signal
import threading
import time

import numpy as np

from motors.arm_calibration import CalibrationMap, digest
from controls.controller.weld_plan import Settings, WeldGeometry, build_plan, JointPath
from controls.controller.recovery import recovery_path


def save(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def generate(mapping, standby, *, count=48, seed=20261120):
    """Mix nearby and broad joint samples; preserve rejected samples separately.

    These are model-valid configurations, not measured or approved hardware
    targets. Rejection sampling does not claim uniform free-space coverage.
    Each check starts on the same saved D3 assembly branch.
    """
    geometry = WeldGeometry(mapping)
    geometry.check(standby, clearance_mm=2.0)
    closed = geometry.data.qpos.copy()
    rng = np.random.default_rng(seed)
    bounds = np.array([mapping.raw_limits(i) for i in range(1, 7)])
    cases, rejected = [], []
    attempts = 0
    for category in ("nearby", "broad"):
        accepted = 0
        target = count // 2 if category == "nearby" else count - count // 2
        while accepted < target and attempts < count * 500:
            attempts += 1
            if category == "nearby":
                raw = np.asarray(standby) + rng.uniform(-1, 1, 6) * [40, 20, 12, 80, 40, 80]
            else:
                raw = rng.uniform(bounds[:, 0], bounds[:, 1])
            geometry.data.qpos[:] = closed
            geometry.closure.last = closed.copy()
            try:
                geometry.check(raw, clearance_mm=2.0)
            except ValueError as exc:
                if len(rejected) < 12:
                    rejected.append(dict(raw_deg=raw.tolist(), reason=str(exc)))
                continue
            cases.append(dict(id=f"{category}-{accepted:03d}", category=category, raw_deg=raw.tolist()))
            accepted += 1
        if accepted != target:
            raise ValueError(f"Only generated {accepted}/{target} {category} configurations")
    from simulation.arm_calibration import scene_fingerprint
    from simulation.paths import MODEL_DIR
    return dict(schema="forge.planner_benchmark_corpus", version=1, seed=seed,
                mapping=mapping.document, mapping_fingerprint=mapping.fingerprint,
                scene_fingerprint=scene_fingerprint(MODEL_DIR / "robot.xml"),
                standby_raw_deg=list(standby), cases=cases, rejected_samples=rejected,
                sampling_attempts=attempts, hardware_qualified=False)


def path_metrics(path):
    # Sample every spline interval; this is a quality metric, not clearance proof.
    t = np.unique(np.r_[path.times, (path.times[:-1] + path.times[1:]) / 2])
    q = path.curve(t)
    return dict(duration_s=path.duration,
                joint_travel_deg=np.sum(abs(np.diff(q, axis=0)), axis=0).tolist(),
                peak_speed_deg_s=path.peaks()[0].tolist(),
                peak_acceleration_deg_s2=path.peaks()[1].tolist())


def verify_plan(plan, mapping):
    """Replay old time-based clearance checks and the real controller on fake shafts."""
    from controls.hardware.weld_control import WeldController
    from controls.hardware.weld_ports import SimulatedArm
    from controls.controller.tracking import TrackingSettings
    geometry = WeldGeometry(mapping, arm_contacts_only=False)
    samples = 0
    for name in ("home", "approach", "weld"):
        path = JointPath.from_document(plan[name])
        path.check_raw_bounds([mapping.raw_limits(i) for i in range(1, 7)])
        limits = plan["stage_limits"][name]
        for peak, limit in zip(path.peaks(), (limits["speed_deg_s"], limits["acceleration_deg_s2"])):
            if np.any(peak > np.asarray(limit) + 1e-7):
                raise ValueError(f"Independent verification: {name} derivative limit exceeded")
        dt = min(.02, .1 / max(max(path.peaks()[0]), 1e-9))
        times = np.linspace(0, path.duration, max(2, int(np.ceil(path.duration / dt)) + 1))
        for raw in path.curve(times):
            geometry.check(raw, clearance_mm=2.0 if name == "home" else .5)
        samples += len(times)
    class Clock:
        now = 0.0
        def __call__(self):
            return self.now
    clock = Clock()
    arm = SimulatedArm(plan["current_raw_deg"], clock=clock)
    control = WeldController(arm, clock=clock, pose_check=geometry.check,
                             allow_optimized_preparation=True,
                             tracking_settings=TrackingSettings())
    ready_s = None
    try:
        control.select(True, False)
        control.load_plan(plan)
        control.go_initial()
        budget = plan["preparation_timing"]["nominal_total_s"] + plan["duration_s"] + 30
        while clock.now < budget:
            clock.now += .02
            control.tick()
            if control.state in ("paused", "fault"):
                raise ValueError(f"Independent fake-shaft replay: {control.state}: {control.message}")
            if control.state == "ready":
                ready_s = clock.now
                control.start_weld()
            if control.state == "complete":
                return dict(clearance_samples=samples, ideal_ready_s=ready_s,
                            within_five_seconds=ready_s <= 5, fake_shaft_replay=True,
                            physical_qualification=False)
        raise ValueError("Independent fake-shaft replay did not complete")
    finally:
        control.close()


def run_case(job):
    corpus, case, settings_doc, stage, output, verify, timeout_s = job
    mapping = CalibrationMap(corpus["mapping"])
    settings = Settings(**settings_doc)
    row = dict(id=case["id"], category=case["category"], success=False)
    started = time.perf_counter()
    timer = hasattr(signal, "setitimer") and threading.current_thread() is threading.main_thread()
    row["planning_deadline_enforced"] = timer
    def deadline(signum, frame):
        raise TimeoutError(f"Planning exceeded {timeout_s:g}s benchmark budget")
    previous_handler = None
    if timer:
        previous_handler = signal.signal(signal.SIGALRM, deadline)
        signal.setitimer(signal.ITIMER_REAL, timeout_s)
    try:
        if stage == "recovery":
            geometry = WeldGeometry(mapping)
            path, route = recovery_path(geometry, case["raw_deg"], corpus["standby_raw_deg"], settings)
            result = dict(path=path.document(), recovery=route)
            row.update(paths=dict(home=path_metrics(path)), route=route["route"],
                       checked_edges=route["checked_edges"])
        else:
            result = build_plan(dict(mapping=corpus["mapping"],
                current_raw_deg=case["raw_deg"], initial_raw_deg=corpus["standby_raw_deg"],
                scene_fingerprint=corpus["scene_fingerprint"], settings=settings_doc,
                arm_enabled=True, chassis_enabled=False))
            row.update(paths={name: path_metrics(JointPath.from_document(result[name]))
                              for name in ("home", "approach", "weld")},
                       preparation_s=result["preparation_timing"]["nominal_total_s"],
                       planning_phases=result.get("planning_timing_s"),
                       planning_reuse=result.get("planning_reuse"),
                       effective_hz=result["effective_hz"], route=result["recovery"]["route"],
                       checked_edges=result["recovery"]["checked_edges"])
            distance = abs(np.asarray(result["weld"]["raw_deg"][0]) - case["raw_deg"])
            speed = np.minimum(result["raw_speed_limits_deg_s"], settings.max_speed_deg_s)
            acceleration = settings.max_accel_deg_s2
            bound = np.where(distance <= speed**2 / acceleration,
                             2 * np.sqrt(distance / acceleration), distance / speed + speed / acceleration)
            # Optimistic any-route bound: simultaneous shafts, no extra stops,
            # no path detour, no lag and only initial/final 0.2 s settling.
            row["optimistic_motion_lower_bound_s"] = float(max(bound)) + .4
            row["within_five_seconds"] = row["preparation_s"] <= 5
        row["success"] = True
        row["planning_s"] = time.perf_counter() - started
        if timer:
            signal.setitimer(signal.ITIMER_REAL, 0)
        save(Path(output) / "plans" / f"{case['id']}.json", result)
        if verify and stage == "full":
            verification_started = time.perf_counter()
            row["verification"] = verify_plan(result, mapping)
            row["verification_s"] = time.perf_counter() - verification_started
    except (ValueError, TimeoutError) as exc:
        if row["success"]:
            row["verification_error"] = str(exc)
        else:
            row.update(planning_s=time.perf_counter() - started, reason=str(exc))
            row["timed_out"] = isinstance(exc, TimeoutError)
    finally:
        if timer:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous_handler)
    if platform.system() == "Linux":
        import resource
        row["worker_peak_rss_mib"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    save(Path(output) / "cases" / f"{case['id']}.json", row)
    return row


def summarize(rows):
    elapsed = np.array([r["planning_s"] for r in rows])
    accepted = [r for r in rows if r["success"]]
    preparation = [r["preparation_s"] for r in accepted if "preparation_s" in r]
    return dict(cases=len(rows), successful=len(accepted), failed=len(rows) - len(accepted),
                latency_s=dict(median=float(np.median(elapsed)), p95=float(np.percentile(elapsed, 95)),
                               maximum=float(max(elapsed)), total=float(sum(elapsed))),
                successful_median_s=(float(np.median([r["planning_s"] for r in accepted])) if accepted else None),
                preparation=(dict(median_s=float(np.median(preparation)), minimum_s=float(min(preparation)),
                                  maximum_s=float(max(preparation)), within_five_seconds=sum(t <= 5 for t in preparation))
                             if preparation else None),
                verification_errors=sum("verification_error" in r for r in rows))


def main(argv=None):
    root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corpus", type=Path, required=True)
    parser.add_argument("--generate", action="store_true")
    parser.add_argument("--mapping", type=Path, default=root / "src/robot_description/config/arm_calibration_measured.json")
    parser.add_argument("--standby", type=Path, default=root / "src/robot_description/config/arm_standby_compact.json")
    parser.add_argument("--count", type=int, default=48)
    parser.add_argument("--seed", type=int, default=20261120)
    parser.add_argument("--stage", choices=("recovery", "full"), default="full")
    parser.add_argument("--executor", choices=("serial", "threads", "processes"), default="serial")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--label", default="working-tree")
    parser.add_argument("--max-speed-deg-s", type=float, default=100,
                        help="offline preparation request; existing motor/acceleration caps still apply")
    parser.add_argument("--prep-seconds", type=float, default=5)
    parser.add_argument("--lowering-seconds", type=float, default=2.3)
    parser.add_argument("--preparation-planner", choices=("direct-first", "staged"), default="direct-first")
    parser.add_argument("--verify", action="store_true", help="independently recheck accepted plans and replay fake shafts")
    parser.add_argument("--resume", action="store_true", help="reuse completed per-case checkpoints with matching inputs/code")
    parser.add_argument("--timeout-seconds", type=float, default=180)
    args = parser.parse_args(argv)
    if (args.count < 2 or not 1 <= args.workers <= 8 or (args.limit is not None and args.limit < 1)
            or not np.isfinite(args.timeout_seconds) or args.timeout_seconds <= 0):
        parser.error("Require count >=2, workers 1..8 and a positive limit")
    if args.generate:
        if args.corpus.exists():
            parser.error("Corpus already exists; use it unchanged for paired comparisons")
        mapping = CalibrationMap(json.loads(args.mapping.read_text()))
        standby = json.loads(args.standby.read_text())
        if standby.get("mapping_fingerprint", mapping.fingerprint) != mapping.fingerprint:
            parser.error("Standby and mapping fingerprints differ")
        from controls.controller.weld_plan import initial_from_model
        corpus = generate(mapping, initial_from_model(standby, mapping), count=args.count, seed=args.seed)
        save(args.corpus, corpus)
        print(f"Generated {len(corpus['cases'])} valid model poses in {corpus['sampling_attempts']} attempts", flush=True)
        if args.output is None:
            return 0
    corpus = json.loads(args.corpus.read_text())
    if args.output is None:
        parser.error("--output is required for a benchmark run")
    if args.output.exists() and not args.resume:
        parser.error("Output already exists; preserve earlier measurements")
    from simulation.arm_calibration import scene_fingerprint
    from simulation.paths import MODEL_DIR
    if corpus["scene_fingerprint"] != scene_fingerprint(MODEL_DIR / "robot.xml"):
        parser.error("Model changed since corpus generation")
    settings = asdict(Settings(cycles=1, limit_profile="weld", max_speed_deg_s=args.max_speed_deg_s,
        optimize_preparation=True, approach_lift_m=.03, lowering_min_s=args.lowering_seconds,
        preparation_target_s=args.prep_seconds, preparation_planner=args.preparation_planner))
    # Interleave categories so a prefix includes local and broad cases.
    cases = sorted(corpus["cases"], key=lambda c: (int(c["id"].split("-")[-1]), c["category"]))[:args.limit]
    sources = [p for folder in ("src/controls/controls/controller", "src/simulation/simulation")
               for p in sorted((root / folder).glob("*.py"))]
    sources.extend(root / name for name in (
        "src/controls/controls/simulation/planner_benchmark.py",
        "src/controls/controls/hardware/weld_control.py",
        "src/controls/controls/hardware/weld_ports.py", "scripts/benchmark_weld_planner.py"))
    source_sha256 = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    identity = dict(corpus_fingerprint=digest(corpus), settings=settings, stage=args.stage,
                    source_sha256=source_sha256, executor=args.executor, workers=args.workers,
                    verify=args.verify, timeout_seconds=args.timeout_seconds)
    manifest = args.output / "inputs.json"
    if args.resume and (not manifest.exists() or json.loads(manifest.read_text()) != identity):
        parser.error("Cannot resume: benchmark inputs/code changed or checkpoint identity is missing")
    args.output.mkdir(parents=True, exist_ok=True)
    save(manifest, identity)
    completed = {p.stem: json.loads(p.read_text()) for p in (args.output / "cases").glob("*.json")} if args.resume else {}
    jobs = [(corpus, case, settings, args.stage, str(args.output), args.verify, args.timeout_seconds)
            for case in cases if case["id"] not in completed]
    rows = [completed[case["id"]] for case in cases if case["id"] in completed]
    started = time.perf_counter()
    def collect(results):
        for row in results:
            rows.append(row)
            print(f"{row['id']}: {'PASS' if row['success'] else 'BLOCKED'} {row['planning_s']:.3f}s", flush=True)
            save(args.output / "partial.json", rows)
    if args.executor == "serial":
        collect(map(run_case, jobs))
    else:
        executor = ThreadPoolExecutor if args.executor == "threads" else ProcessPoolExecutor
        kwargs = dict(max_workers=args.workers)
        if args.executor == "processes":
            kwargs["mp_context"] = multiprocessing.get_context("spawn")
        with executor(**kwargs) as pool:
            collect(pool.map(run_case, jobs))
    report = dict(schema="forge.planner_benchmark", version=1, label=args.label,
        corpus_fingerprint=digest(corpus), stage=args.stage, settings=settings,
        executor=args.executor, workers=1 if args.executor == "serial" else args.workers,
        wall_s=time.perf_counter() - started, summary=summarize(rows), cases=rows,
        platform=platform.platform(), python=platform.python_version(), cpu_count=os.cpu_count(),
        thread_environment={key: os.environ.get(key) for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS")},
        source_sha256=source_sha256, resumed=args.resume, completed_before_run=len(completed),
        hardware_qualified=False)
    save(args.output / "report.json", report)
    print(json.dumps(report["summary"], indent=2), flush=True)
    return 2 if any("verification_error" in row for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
