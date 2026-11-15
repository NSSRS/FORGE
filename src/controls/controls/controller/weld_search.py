"""Offline multi-start CMA-ES over joint mobility and IK center/branch seeds.

Candidates never command motors. Search uses coarse samples for ranking only;
finalists are regenerated at normal resolution, checked densely and retimed.
Only a candidate with a checked complete approach receives an executable plan.
"""

from dataclasses import asdict, replace
import argparse
import json
import math
import multiprocessing as mp
from pathlib import Path
import time

import numpy as np

from motors.arm_calibration import CalibrationMap, digest
from .limits import motor_constraints, dynamics_constraints
from .weld_plan import (
    Settings,
    WeldGeometry,
    JointPath,
    weave_path,
    validate_weave,
    solve_weave,
    build_plan,
    initial_from_model,
    validate_search_solution,
)


def decode(values):
    """Higher mobility is a reward for using an axis, inverse of IK penalty."""
    x = np.asarray(values, float)
    if x.shape != (7,) or not np.isfinite(x).all() or np.any(abs(x) > 1.0 + 1e-8):
        raise ValueError("Seven normalized bounded search coordinates required")
    return dict(
        mobility_weights=[1.0, *(10.0 ** (1.0 + x[:5])).tolist()],
        center_yaw_deg=float(10 * x[5]),
        wrist_seed_deg=float(75 * x[6]),
    )


def encode(candidate):
    mobility = np.asarray(candidate["mobility_weights"], float)
    if mobility.shape != (6,) or mobility[0] != 1 or np.any(mobility <= 0):
        raise ValueError("Warm start requires A1-normalized mobility weights")
    values = np.r_[
        np.log10(mobility[1:]) - 1,
        candidate["center_yaw_deg"] / 10,
        candidate["wrist_seed_deg"] / 75,
    ]
    decode(values)
    return values


def objective(path, settings, speed_limits, acceleration_limits):
    """Fixed requested clock: slowing down cannot artificially improve rewards."""
    speed, acceleration = path.peaks()
    speed_limits = np.asarray(speed_limits)
    acceleration_limits = np.asarray(acceleration_limits)
    velocity_ratio = speed / speed_limits
    acceleration_ratio = acceleration / acceleration_limits
    ratio = max(1.0, float(max(velocity_ratio)), math.sqrt(float(max(acceleration_ratio))))
    scale = ratio * 1.02 if ratio > 1 else 1.0
    t = np.linspace(0, path.duration / settings.cycles, 801)
    v = path.curve.derivative()(t)
    a = path.curve.derivative(2)(t)
    # A1 motion has the highest cost; shoulders next, wrist lowest.
    effort = np.array([3.0, 1.5, 1.5, 1.0, 1.0, 1.0])
    velocity_rms = float(np.sqrt(np.mean((v / speed_limits) ** 2 * effort)))
    acceleration_rms = float(np.sqrt(np.mean((a / acceleration_limits) ** 2)))
    reversals = []
    for i in range(6):
        signs = np.sign(v[abs(v[:, i]) > 0.01, i])
        count = int(np.count_nonzero(signs != np.roll(signs, 1))) if len(signs) else 0
        reversals.append(count)
    extra_reversals = float(np.mean(np.maximum(0, np.array(reversals) - 2)))
    # Timing dominates while infeasible; once requested timing fits, favor less
    # speed/acceleration and fewer spurious direction changes at that SAME clock.
    score = (
        100 * math.log(scale)
        + 0.5 * float(max(velocity_ratio))
        + 0.25 * velocity_rms
        + 0.05 * acceleration_rms
        + 0.1 * extra_reversals
    )
    return dict(
        score=score,
        time_scale=scale,
        effective_hz=settings.hz / scale,
        feed_mm_s=settings.feed_m_s * 1000 / scale,
        peak_requested_deg_s=speed.tolist(),
        peak_executable_deg_s=(speed / scale).tolist(),
        peak_requested_accel_deg_s2=acceleration.tolist(),
        max_speed_fraction=float(max(velocity_ratio)),
        max_acceleration_fraction=float(max(acceleration_ratio)),
        weighted_velocity_rms=velocity_rms,
        normalized_acceleration_rms=acceleration_rms,
        direction_reversals=reversals,
        extra_reversals=extra_reversals,
    )


_worker = None


def initialize_worker(root, mapping, settings):
    global _worker
    root = Path(root)
    settings = Settings(**settings)
    constraints = motor_constraints(root, settings.max_speed_deg_s, profile=settings.limit_profile)
    _worker = (WeldGeometry(CalibrationMap(mapping)), settings, constraints)


def evaluate(job):
    candidate, dense = job
    geometry, settings, constraints = _worker
    try:
        path = weave_path(
            geometry, settings, candidate=candidate, spatial_steps=20 if dense else 10
        )
        task = validate_weave(geometry, path, settings, samples=401 if dense else 81)
        dynamics = dynamics_constraints(
            geometry,
            path.positions[np.linspace(0, 200, 9, dtype=int)],
            constraints,
            acceleration_policy_deg_s2=(
                None if settings.limit_profile == "weld" else settings.max_accel_deg_s2
            ),
        )
        metrics = objective(
            path,
            settings,
            [r["deployment_deg_s"] for r in constraints["joints"]],
            dynamics["raw_acceleration_deg_s2"],
        )
        return dict(feasible=True, candidate=candidate, dense=dense, task_report=task, **metrics)
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        return dict(feasible=False, candidate=candidate, dense=dense, score=1e6, reason=str(exc))


def solution_document(candidate, geometry, settings, constraints):
    from simulation.arm_calibration import scene_fingerprint
    from simulation.paths import MODEL_DIR

    return dict(
        schema="forge.weld_search_solution",
        version=1,
        candidate=candidate,
        mapping_fingerprint=geometry.mapping.fingerprint,
        scene_fingerprint=scene_fingerprint(MODEL_DIR / "robot.xml"),
        constraints_fingerprint=digest(constraints),
        amplitude_m=settings.amplitude_m,
        hardware_qualified=False,
        requires_fresh_route_validation=True,
    )


def write_json(path, value):
    # A checkpoint replacement does not publish/activate a motor configuration.
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def search(
    root,
    settings,
    mapping,
    initial,
    *,
    directory,
    seeds=(11, 23, 47),
    generations=12,
    population=12,
    workers=4,
    finalists=8,
    warm_start=None,
):
    import cma

    root = Path(root)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    geometry = WeldGeometry(CalibrationMap(mapping))
    constraints = motor_constraints(root, settings.max_speed_deg_s, profile=settings.limit_profile)
    warm = (
        validate_search_solution(warm_start, geometry, settings, constraints)
        if warm_start
        else None
    )
    warm_coordinates = encode(warm) if warm else None
    started = time.monotonic()
    archive = []
    baseline_path, baseline = solve_weave(geometry, settings, root)
    # Undo the baseline's automatic slowdown before comparing the fixed requested clock.
    nominal = JointPath(
        baseline_path.times / baseline["total_time_scale"],
        baseline_path.positions,
        baseline_path.interpolation,
    )
    baseline_metrics = objective(
        nominal,
        settings,
        [r["deployment_deg_s"] for r in constraints["joints"]],
        baseline["dynamics"]["raw_acceleration_deg_s2"],
    )
    report = dict(
        schema="forge.weld_search",
        version=1,
        backend=f"pycma {cma.__version__}",
        seeds=list(seeds),
        generations=generations,
        population=population,
        workers=workers,
        settings=asdict(settings),
        constraints=constraints,
        baseline=baseline_metrics,
        objective="100 log(common time scale) + 0.5 peak normalized speed + 0.25 weighted RMS speed + 0.05 RMS acceleration + 0.1 extra reversals, at unchanged requested timing",
        search_space="five log mobility weights [1,100], center yaw +/-10 deg, wrist IK seed +/-75 deg",
        identity=solution_document(decode([0] * 7), geometry, settings, constraints),
        warm_start=warm_start,
        hardware_accessed=False,
        hardware_qualified=False,
        archive=archive,
    )
    print(json.dumps(dict(event="baseline", **baseline_metrics)), flush=True)
    write_json(directory / "search.json", report)
    context = mp.get_context("spawn")
    with context.Pool(workers, initialize_worker, (str(root), mapping, asdict(settings))) as pool:
        if warm:
            row = pool.map(evaluate, [(warm, False)])[0]
            row.update(restart=-1, seed=0, generation=0, coordinates=warm_coordinates.tolist())
            archive.append(row)
        # Deliberately distinct preferences and IK seeds for independent restarts.
        centers = [
            [-0.4, -0.4, 0.2, 0.2, 0.2, 0, 0],
            [-0.1, -0.1, 0.6, 0.6, 0.6, 0.35, 0.4],
            [-0.6, -0.6, 0.0, 0.0, 0.0, -0.35, -0.4],
        ]
        for restart, seed in enumerate(seeds):
            center = warm_coordinates.tolist() if warm else centers[restart % len(centers)]
            sigma = (0.18 + 0.14 * (restart % 3)) if warm else 0.32
            es = cma.CMAEvolutionStrategy(
                center,
                sigma,
                dict(
                    seed=int(seed),
                    popsize=population,
                    bounds=[-1.0, 1.0],
                    maxiter=generations,
                    verbose=-9,
                    verb_log=0,
                    tolfun=1e-8,
                    tolx=1e-6,
                ),
            )
            while not es.stop():
                xs = es.ask()
                rows = pool.map(evaluate, [(decode(x), False) for x in xs])
                es.tell(xs, [r["score"] for r in rows])
                for x, row in zip(xs, rows):
                    row.update(
                        restart=restart,
                        seed=seed,
                        generation=es.countiter,
                        coordinates=list(map(float, x)),
                    )
                    archive.append(row)
                feasible = [r for r in archive if r["feasible"]]
                best = min(feasible, key=lambda r: r["score"]) if feasible else None
                print(
                    json.dumps(
                        dict(
                            event="generation",
                            restart=restart,
                            generation=es.countiter,
                            evaluations=len(archive),
                            feasible=len(feasible),
                            elapsed_s=time.monotonic() - started,
                            best_hz=best["effective_hz"] if best else None,
                            best_score=best["score"] if best else None,
                        )
                    ),
                    flush=True,
                )
                write_json(directory / "search.json", report)
        feasible = sorted((r for r in archive if r["feasible"]), key=lambda r: r["score"])
        selected = []
        for row in feasible:
            vector = np.array(row["coordinates"])
            if all(
                np.linalg.norm(vector - np.array(other["coordinates"])) > 0.12 for other in selected
            ):
                selected.append(row)
            if len(selected) >= finalists:
                break
        # At least one candidate from each restart gets a dense assessment.
        for restart in range(len(seeds)):
            rows = [r for r in feasible if r["restart"] == restart]
            if rows and rows[0] not in selected:
                selected.append(rows[0])
        dense = pool.map(evaluate, [(r["candidate"], True) for r in selected])
    dense.sort(key=lambda r: r["score"])
    report["finalists"] = dense
    report["route_checks"] = []
    for rank, row in enumerate(dense):
        if not row["feasible"] or row["score"] >= baseline_metrics["score"]:
            continue
        document = solution_document(row["candidate"], geometry, settings, constraints)
        try:
            print(
                json.dumps(
                    dict(
                        event="checking_complete_route", rank=rank, effective_hz=row["effective_hz"]
                    )
                ),
                flush=True,
            )
            plan = build_plan(
                dict(
                    mapping=mapping,
                    current_raw_deg=initial,
                    initial_raw_deg=initial,
                    settings=asdict(replace(settings, search_solution=document)),
                    arm_enabled=True,
                    chassis_enabled=True,
                )
            )
            report["route_checks"].append(dict(rank=rank, passed=True))
            report["winner"] = dict(rank=rank, **row)
            report["complete_route_checked"] = True
            write_json(directory / "solution.json", document)
            write_json(directory / "plan.json", plan)
            break
        except ValueError as exc:
            report["route_checks"].append(dict(rank=rank, passed=False, reason=str(exc)))
    report["elapsed_s"] = time.monotonic() - started
    report["improved"] = bool(report.get("winner"))
    write_json(directory / "search.json", report)
    print(
        json.dumps(
            dict(
                event="complete",
                improved=report["improved"],
                evaluations=len(archive),
                elapsed_s=report["elapsed_s"],
                winner=report.get("winner"),
            ),
            allow_nan=False,
        ),
        flush=True,
    )
    return report


def main(argv=None):
    root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--generations", type=int, default=12)
    parser.add_argument("--population", type=int, default=12)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seeds", type=int, nargs="+", default=[11, 23, 47])
    parser.add_argument("--finalists", type=int, default=8)
    parser.add_argument(
        "--warm-start",
        type=Path,
        help="refine a saved solution with matching model/mapping/constraints",
    )
    args = parser.parse_args(argv)
    if not (
        1 <= args.generations <= 1000
        and 4 <= args.population <= 128
        and 1 <= args.workers <= 16
        and 1 <= len(args.seeds) <= 16
        and all(seed > 0 for seed in args.seeds)
        and 1 <= args.finalists <= 32
    ):
        parser.error("Invalid search budget")
    mapping = CalibrationMap(
        json.loads(
            (root / "src/robot_description/config/arm_calibration_measured.json").read_text()
        )
    )
    pose = json.loads((root / "src/robot_description/config/arm_initial_measured.json").read_text())
    experiment = json.loads((root / "src/controls/config/weld_experiment.json").read_text())
    settings = Settings(
        hz=experiment["frequency_hz"],
        feed_m_s=experiment["feed_mm_s"] / 1000,
        amplitude_m=experiment["amplitude_mm"] / 1000,
        cycles=1,
        limit_profile="weld",
    )
    report = search(
        root,
        settings,
        mapping.document,
        initial_from_model(pose, mapping),
        directory=args.output_dir,
        seeds=args.seeds,
        generations=args.generations,
        population=args.population,
        workers=args.workers,
        finalists=args.finalists,
        warm_start=json.loads(args.warm_start.read_text()) if args.warm_start else None,
    )
    return 0 if report["improved"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
