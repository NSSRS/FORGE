"""Bounded staging search; preserves the complete checked weld target."""

from copy import deepcopy

import numpy as np

from .limits import dynamics_constraints
from .preparation import preparation_timing
from .recovery import recovery_path


STAGING_FRACTIONS = (0.25, 0.5, 0.75)
TIMING_MARGIN = 1.02
PATH_ACCELERATION_FRACTION = 0.9


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


def fit_approach_timing(path, speed, acceleration):
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
        steps[start:end] *= max(factor, 0.5 / float(np.sum(steps[start:end])))
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
    """Compare three staging poses to the requested standby, without new limits.

    A staging pose is a raw-coordinate interpolation toward the same weld start,
    not a truncated approach. Check recovery at 2 mm and approach at the normal
    clearance, including fixed cable bounds and reduced-inertia acceleration.
    This small search is not a global optimum or a physical readiness claim.
    """
    from .weld_plan import approach_path

    best = deepcopy(plan)
    start = np.asarray(plan["current_raw_deg"])
    end = np.asarray(plan["weld"]["raw_deg"][0])
    speed = np.minimum(plan["raw_speed_limits_deg_s"], settings.max_speed_deg_s)
    constraints = plan["optimization"]["constraints"]
    candidates = []
    for fraction in STAGING_FRACTIONS:
        staging = start + fraction * (end - start)
        row = dict(fraction=fraction, staging_raw_deg=staging.tolist())
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
            approach = fit_approach_timing(approach, speed, acceleration)
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
            row.update(checked=True, timing=timing)
            if (
                timing["nominal_total_s"]
                < best["preparation_timing"]["nominal_total_s"]
            ):
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
        guarantee="Best of three checked staging candidates and requested standby; physical time unverified.",
    )
    return best
