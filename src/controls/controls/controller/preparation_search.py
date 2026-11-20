"""Bounded staging search; preserves the complete checked weld target."""

from copy import deepcopy

import numpy as np

from .limits import dynamics_constraints
from .preparation import preparation_timing
from .recovery import recovery_path


STAGING_FRACTIONS = (0.0, 0.25, 0.5, 0.75)
TIMING_MARGIN = 1.02
PATH_ACCELERATION_FRACTION = 0.9


def stopped_move_lower_bound(start, goal, speed, acceleration):
    """Optimistic rest-to-rest time; ignores path detours and reduced inertia."""
    distance = abs(np.asarray(goal) - start)
    speed, acceleration = np.asarray(speed), np.asarray(acceleration)
    return float(np.max(np.where(distance <= speed**2 / acceleration,
        2 * np.sqrt(distance / acceleration), distance / speed + speed / acceleration)))


def fit_timing(path, speed, acceleration):
    """Use exact cubic extrema, with 2% timing margin and 10% pacing reserve."""
    from .weld_plan import JointPath

    peak_speed, peak_acceleration = path.peaks()
    factor = max(
        float(np.max(peak_speed / speed)),
        float(
            np.sqrt(
                np.max(peak_acceleration / (PATH_ACCELERATION_FRACTION * acceleration))
            )
        ),
    )
    # Keep stationary stages finite; no zero-duration interpolants.
    factor = max(factor * TIMING_MARGIN, 0.5 / path.duration)
    return JointPath(path.times * factor, path.positions, path.interpolation)


def fit_approach_timing(path, speed, acceleration, *, minimum_final_s=0.0):
    """Time moving legs independently across existing stationary intervals.

    The raised approach contains a stopped transfer followed by a short lowering
    leg. Scaling both by the lowering leg's acceleration peak needlessly slows
    the transfer. PCHIP has zero derivatives beside a constant interval, so each
    moving leg can be scaled without changing any cubic's geometric path. Keep
    the stationary intervals and the same acceleration reserve.
    """
    from .weld_plan import JointPath

    flat = np.all(np.diff(path.positions, axis=0) == 0, axis=1)
    if path.interpolation != "pchip" or not np.any(flat):
        return fit_timing(path, speed, acceleration)
    steps = np.diff(path.times).copy()
    dt = steps[:, None]
    a, b, c = 3 * path.curve.c[0], 2 * path.curve.c[1], path.curve.c[2]
    vertex = np.clip(np.divide(-b, 2 * a, out=np.zeros_like(a), where=a != 0), 0, dt)
    velocity = np.maximum.reduce([
        abs(c), abs((a * dt + b) * dt + c), abs((a * vertex + b) * vertex + c),
    ])
    acceleration_peak = np.maximum(abs(b), abs(2 * a * dt + b))
    start = 0
    while start < len(steps):
        if flat[start]:
            start += 1
            continue
        end = start + 1
        while end < len(steps) and not flat[end]:
            end += 1
        factor = TIMING_MARGIN * max(
            float(np.max(velocity[start:end] / speed)),
            float(np.sqrt(np.max(acceleration_peak[start:end] /
                                (PATH_ACCELERATION_FRACTION * acceleration)))),
        )
        minimum = max(0.5, minimum_final_s if end == len(steps) else 0.0)
        steps[start:end] *= max(factor, minimum / float(np.sum(steps[start:end])))
        start = end
    return JointPath(np.r_[0.0, np.cumsum(steps)], path.positions)


def apply_lowering_minimum(plan, settings):
    """Slow only the final raised-approach leg, after staging selection.

    Scaling time after the last stationary interval preserves each geometric
    cubic, the earlier transfer/dwell and the selected standby. Never shorten
    an already slower lowering leg or relax its derivative limits.
    """
    from .weld_plan import JointPath

    if not settings.lowering_min_s:
        return plan
    path = JointPath.from_document(plan["approach"])
    flat = np.all(np.diff(path.positions, axis=0) == 0, axis=1)
    if path.interpolation != "pchip" or not np.any(flat) or flat[-1]:
        raise ValueError("Lowering duration requires a raised approach with a stationary boundary")
    start = int(np.flatnonzero(flat)[-1] + 1)
    original = path.duration - path.times[start]
    factor = max(1.0, settings.lowering_min_s / original)
    times = path.times.copy()
    times[start:] = times[start] + (times[start:] - times[start]) * factor
    lowered = JointPath(times, path.positions, path.interpolation)
    envelope = plan["stage_limits"]["approach"]
    timing = preparation_timing(
        JointPath.from_document(plan["home"]), lowered,
        plan["recovery"]["waypoint_times_s"],
        np.asarray(envelope["speed_deg_s"]),
        np.asarray(envelope["acceleration_deg_s2"]),
        settings.preparation_target_s,
    )
    return dict(
        plan, approach=lowered.document(), preparation_timing=timing,
        lowering_timing=dict(
            requested_minimum_s=settings.lowering_min_s,
            start_s=float(path.times[start]), original_duration_s=float(original),
            duration_s=float(lowered.duration - lowered.times[start]),
            added_s=float(lowered.duration - path.duration),
        ),
    )


def optimize_preparation(plan, geometry, settings):
    """Compare current pose and three staging poses to the requested standby.

    A staging pose is a raw-coordinate interpolation toward the same weld start,
    not a truncated approach. Check recovery at 2 mm and approach at the normal
    clearance, including fixed cable bounds and reduced-inertia acceleration.
    This small search is not a global optimum or a physical readiness claim.
    """
    from .weld_plan import approach_path, raised_approach_path, APPROACH_LOWERING_SECONDS

    best = deepcopy(plan)
    start = np.asarray(plan["current_raw_deg"])
    end = np.asarray(plan["weld"]["raw_deg"][0])
    speed = np.minimum(plan["raw_speed_limits_deg_s"], settings.max_speed_deg_s)
    constraints = plan["optimization"]["constraints"]
    candidates = []
    def selection_duration(timing, approach):
        extra = 0.0
        if settings.lowering_min_s:
            flat = np.all(np.diff(approach.positions, axis=0) == 0, axis=1)
            if not np.any(flat) or flat[-1]:
                raise ValueError("Lowering duration requires a raised approach with a stationary boundary")
            start_lowering = approach.times[int(np.flatnonzero(flat)[-1]) + 1]
            extra = max(0.0, settings.lowering_min_s - (approach.duration - start_lowering))
        return timing["nominal_total_s"] + extra
    from .weld_plan import JointPath
    best_duration = selection_duration(best["preparation_timing"], JointPath.from_document(best["approach"]))
    for fraction in STAGING_FRACTIONS:
        staging = start + fraction * (end - start)
        row = dict(fraction=fraction, staging_raw_deg=staging.tolist())
        key = (float(settings.approach_lift_m), tuple(float(q) for q in end))
        lower = getattr(geometry, "lowering_cache", {}).get(key)
        if (fraction and settings.preparation_planner == "direct-first"
                and settings.approach_lift_m and settings.search_solution is None
                and isinstance(lower, np.ndarray)):
            # This fixed-height branch has one known approach spline. Time it
            # cheaply at the full policy ceilings, before mesh/inertia checks
            # can reject it or lower those ceilings. Add an ideal rest-to-rest
            # recovery bound (even a detour cannot beat that). This estimates
            # a lower bound, never accepts an unchecked geometric route.
            home_bound = max(.5, stopped_move_lower_bound(start, staging, speed, settings.max_accel_deg_s2))
            approach_bound = fit_approach_timing(
                raised_approach_path(staging, lower, settings), speed,
                np.full(6, settings.max_accel_deg_s2),
                minimum_final_s=max(APPROACH_LOWERING_SECONDS, settings.lowering_min_s))
            bound = home_bound + approach_bound.duration + .6
            if bound >= best_duration + 1e-9:
                row.update(checked=False, pruned=True, duration_lower_bound_s=bound,
                           reason="Optimistic recovery/approach timing cannot improve the best checked route")
                candidates.append(row)
                continue
        try:
            home, recovery = recovery_path(geometry, start, staging, settings)
            approach, route = approach_path(geometry, staging, end, settings)
            dynamics = dynamics_constraints(
                geometry,
                np.vstack(
                    [
                        p.positions[np.linspace(0, len(p.positions) - 1, 5, dtype=int)]
                        for p in (home, approach)
                    ]
                ),
                constraints,
                acceleration_policy_deg_s2=settings.max_accel_deg_s2,
            )
            acceleration = np.asarray(dynamics["raw_acceleration_deg_s2"])
            old_duration = home.duration
            home = fit_timing(home, speed, acceleration)
            # Retain the authored lowering duration when shortening the long
            # transfer. Compressing this leg caused a synthetic A2 tracking
            # pause even though nominal derivative limits were satisfied.
            approach = fit_approach_timing(approach, speed, acceleration,
                                          minimum_final_s=APPROACH_LOWERING_SECONDS)
            recovery["waypoint_times_s"] = [
                t * home.duration / old_duration for t in recovery["waypoint_times_s"]
            ]
            # recovery_path/approach_path checked these complete geometric paths.
            # Uniform home scaling and scaling approach legs across flat stops
            # preserve every cubic position; do not repeat the same mesh work.
            timing = preparation_timing(
                home,
                approach,
                recovery["waypoint_times_s"],
                speed,
                acceleration,
                settings.preparation_target_s,
            )
            duration = selection_duration(timing, approach)
            row.update(checked=True, timing=timing, selection_duration_s=duration)
            if duration < best_duration:
                best_duration = duration
                best.update(
                    initial_raw_deg=staging.tolist(),
                    home=home.document(),
                    approach=approach.document(),
                    recovery=recovery,
                    approach_route=route,
                    preparation_timing=timing,
                    transition_dynamics=dynamics,
                )
                for name in ("home", "approach"):
                    best["stage_limits"][name] = dict(
                        speed_deg_s=speed.tolist(),
                        acceleration_deg_s2=acceleration.tolist(),
                    )
        except ValueError as exc:
            row.update(checked=False, reason=str(exc))
        candidates.append(row)
    best["preparation_search"] = dict(
        requested_standby_raw_deg=plan["initial_raw_deg"],
        original_nominal_total_s=plan["preparation_timing"]["nominal_total_s"],
        candidates=candidates,
        changed_standby=best["initial_raw_deg"] != plan["initial_raw_deg"],
        selection_includes_lowering_minimum=True,
        planner=settings.preparation_planner,
        guarantee="Best checked candidate; pruned fixed-height candidates cannot beat optimistic recovery/approach timing. Physical time unverified.",
    )
    return best
