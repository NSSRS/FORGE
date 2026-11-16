"""Feedback-paced preparation on the checked path, without position offsets."""

import math

import numpy as np


def preparation_timing(home, approach, waypoint_times, speed_limits, acceleration_limits, target_s):
    """Report the requested budget separately from achievable/observed timing.

    Per-segment trapezoidal bounds assume perfect tracking and ignore geometry;
    they can prove a budget impossible, never prove physical feasibility.
    """
    speed, acceleration = np.asarray(speed_limits), np.asarray(acceleration_limits)
    bounded = acceleration_only = 0.0
    stages = []
    for name, path, ends in (("home", home, waypoint_times), ("approach", approach, [approach.duration])):
        previous = path.positions[0]
        stage_min = 0.0
        for end in ends:
            following = path.sample(end)[0]
            distance = abs(following-previous)
            triangular = 2*np.sqrt(distance/acceleration)
            trapezoidal = distance/speed + speed/acceleration
            stage_min += float(np.max(np.where(distance <= speed**2/acceleration,
                                               triangular, trapezoidal)))
            acceleration_only += float(np.max(triangular))
            previous = following
        bounded += stage_min
        stages.append(dict(stage=name, nominal_motion_s=path.duration, lower_bound_s=stage_min))
    settling = .2*(len(waypoint_times)+2)  # Initial hold, home stops, approach arrival.
    nominal = home.duration + approach.duration + settling
    return dict(target_s=float(target_s), nominal_total_s=nominal,
                minimum_settling_s=settling, lower_bound_total_s=bounded+settling,
                acceleration_only_lower_bound_s=acceleration_only+settling,
                nominal_meets_target=nominal <= target_s,
                ruled_out_by_limits=bounded+settling > target_s,
                stages=stages, hardware_qualified=False)


class PreparationClock:
    """Slow a common path clock when any axis lags; never skip or reverse it.

    For q(s(t)), acceleration is q'' * s_dot**2 + q' * s_ddot.
    Use the path's exact global extrema to bound the clock slew within the
    remaining stage acceleration budget. This is reference shaping, not torque
    or backlash compensation, and it does not establish physical tracking.
    """

    slow_error_deg = 0.5
    wait_error_deg = 1.5

    def __init__(self, path, acceleration_limits):
        self.path = path
        speed, acceleration = path.peaks()
        limits = np.broadcast_to(np.asarray(acceleration_limits, dtype=float), (6,))
        active = speed > 1e-9
        reserve = limits - acceleration
        if np.any(reserve[active] <= 0):
            raise ValueError("Preparation path needs acceleration reserve; calculate again")
        self.slew_per_s = min(2.0, float(np.min(reserve[active] / speed[active]))) if np.any(active) else 2.0
        self.reset()

    def reset(self):
        self.elapsed = 0.0
        self.scale = 1.0
        self.lag_progress = [None] * 6
        self.arrival_started = None

    def timeout_reason(self, now, measured, desired, *, at_end):
        """Allow paced travel, but bound stalled axes and final settling.

        A nominal path duration is not a wall-time deadline after pacing. Each
        lagging shaft must instead make at least 0.1 degree of net progress
        toward its reference within 10 seconds. Returning over the same encoder
        noise does not count as new progress. Final settling gets its own timer.
        These are failure detectors, not a claim of loaded holding capability.
        """
        if at_end:
            if self.arrival_started is None:
                self.arrival_started = now
            if now - self.arrival_started > 10.0:
                return "Arrival timeout after reaching the final reference; recovery incomplete."
            return None
        for i, error in enumerate(desired - measured):
            if abs(error) < self.slow_error_deg:
                self.lag_progress[i] = None
                continue
            direction = math.copysign(1.0, error)
            progress = direction * measured[i]
            previous = self.lag_progress[i]
            if previous is None or previous[0] != direction or progress - previous[1] >= 0.1:
                self.lag_progress[i] = (direction, progress, now)
            elif now - previous[2] > 10.0:
                return f"Preparation stalled A{i+1}: no 0.1 deg net progress for 10 s; recovery incomplete."
        return None

    def advance(self, dt, measured, offset, end):
        if not math.isfinite(dt) or not 0 <= dt <= 0.1:
            raise ValueError("Invalid preparation clock interval")
        previous, _ = self.path.sample(min(offset + self.elapsed, end))
        error = float(np.max(abs(previous - measured)))
        target_scale = np.clip((self.wait_error_deg - error) /
                               (self.wait_error_deg - self.slow_error_deg), 0.0, 1.0)
        scale = float(np.clip(target_scale, self.scale - self.slew_per_s * dt,
                              self.scale + self.slew_per_s * dt))
        self.elapsed = min(end - offset, self.elapsed + dt * (self.scale + scale) / 2)
        self.scale = scale
        position, rate = self.path.sample(offset + self.elapsed)
        return position, rate * scale
