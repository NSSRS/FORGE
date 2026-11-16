"""Bounded offline staging search; preserves the complete checked weld target."""

from copy import deepcopy

import numpy as np

from .limits import dynamics_constraints
from .preparation import preparation_timing
from .recovery import RECOVERY_CLEARANCE_MM, recovery_path


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
            approach = fit_timing(approach, speed, acceleration)
            recovery["waypoint_times_s"] = [
                t * home.duration / old_duration for t in recovery["waypoint_times_s"]
            ]
            geometry.validate_path(home, clearance_mm=RECOVERY_CLEARANCE_MM)
            geometry.validate_path(approach)
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
    best["offline_only"] = True
    best["preparation_search"] = dict(
        requested_standby_raw_deg=plan["initial_raw_deg"],
        original_nominal_total_s=plan["preparation_timing"]["nominal_total_s"],
        candidates=candidates,
        changed_standby=best["initial_raw_deg"] != plan["initial_raw_deg"],
        guarantee="Best of three checked staging candidates and requested standby; offline only.",
    )
    return best
