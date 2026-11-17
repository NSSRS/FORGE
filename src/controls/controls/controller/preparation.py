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
    Use signed cubic derivative bounds over each step to bound clock slew within
    the stage acceleration budget. This is reference shaping, not torque
    or backlash compensation, and it does not establish physical tracking.
    """

    # Soft pacing thresholds leave braking room below the separate 3-degree
    # tracking trip. Arrival still requires 0.25 degree; it is not a travel cap.
    slow_error_deg = 1.0
    wait_error_deg = 2.0
    progress_error_deg = 0.5

    def __init__(self, path, acceleration_limits):
        self.path = path
        self.position_bounds = path.position_bounds()
        self.position_interval = (0.0, path.duration)
        speed, acceleration = path.peaks()
        limits = np.broadcast_to(np.asarray(acceleration_limits, dtype=float), (6,))
        active = speed > 1e-9
        reserve = limits - acceleration
        if np.any(reserve[active] <= 0):
            raise ValueError("Preparation path needs acceleration reserve; calculate again")
        self.acceleration_limits = limits.copy()
        self.slew_per_s = 2.0
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
            if abs(error) < self.progress_error_deg:
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

    def derivative_bounds(self, start, end):
        """Exact signed velocity/acceleration bounds on the traversed cubics."""
        times = self.path.times
        first = min(len(times) - 2, max(0, np.searchsorted(times, start, side="right") - 1))
        last = min(len(times) - 2, np.searchsorted(times, end, side="right") - 1)
        indices = np.arange(first, max(first, last) + 1)
        low = (np.maximum(start, times[indices]) - times[indices])[:, None]
        high = (np.minimum(end, times[indices + 1]) - times[indices])[:, None]
        coefficients = self.path.curve.c[:, indices, :]
        a, b, c = 3 * coefficients[0], 2 * coefficients[1], coefficients[2]
        left = (a * low + b) * low + c
        right = (a * high + b) * high + c
        vertex = np.divide(-b, 2 * a, out=np.zeros_like(a), where=a != 0)
        vertex = np.clip(vertex, low, high)
        middle = (a * vertex + b) * vertex + c
        acceleration_left, acceleration_right = 2 * a * low + b, 2 * a * high + b
        return (
            np.minimum(np.minimum(left, right), middle).min(axis=0),
            np.maximum(np.maximum(left, right), middle).max(axis=0),
            np.minimum(acceleration_left, acceleration_right).min(axis=0),
            np.maximum(acceleration_left, acceleration_right).max(axis=0),
        )

    def acceleration_fits(self, start, end, dt, scale):
        """Conservative interval bound for q'' * scale² + q' * scale_dot.

        Preserve signs: slowing the clock can cancel path acceleration. Using
        global absolute extrema here reserved too little braking authority and
        allowed an accelerating reference to run past lagging shafts.
        """
        stop = min(end, start + dt * (self.scale + scale) / 2)
        v_low, v_high, a_low, a_high = self.derivative_bounds(start, stop)
        low_scale, high_scale = sorted((self.scale**2, scale**2))
        change = (scale - self.scale) / dt
        low = np.minimum(a_low * low_scale, a_low * high_scale) + np.minimum(v_low * change, v_high * change)
        high = np.maximum(a_high * low_scale, a_high * high_scale) + np.maximum(v_low * change, v_high * change)
        return bool(np.all(low >= -self.acceleration_limits) and np.all(high <= self.acceleration_limits))

    def advance(self, dt, measured, offset, end):
        if not math.isfinite(dt) or not 0 <= dt <= 0.1:
            raise ValueError("Invalid preparation clock interval")
        if dt == 0:
            position, rate = self.path.sample(min(offset + self.elapsed, end))
            return position, rate * self.scale
        start = min(offset + self.elapsed, end)
        previous, _ = self.path.sample(start)
        error = float(np.max(abs(previous - measured)))
        target_scale = np.clip((self.wait_error_deg - error) /
                               (self.wait_error_deg - self.slow_error_deg), 0.0, 1.0)
        scale = float(np.clip(target_scale, self.scale - self.slew_per_s * dt,
                              self.scale + self.slew_per_s * dt))
        if not self.acceleration_fits(start, end, dt, scale):
            safe, blocked = self.scale, scale
            for _ in range(24):
                trial = (safe + blocked) / 2
                if self.acceleration_fits(start, end, dt, trial):
                    safe = trial
                else:
                    blocked = trial
            scale = safe
        self.elapsed = min(end - offset, self.elapsed + dt * (self.scale + scale) / 2)
        self.scale = scale
        position, rate = self.path.sample(offset + self.elapsed)
        return position, rate * scale

    def advance_nominal(self, dt, offset, end):
        """Keep planned time for closed-loop tracking; retain independent trips.

        There is no backlog catch-up after a delayed tick. The controller's
        timing-gap, tracking, progress and arrival checks remain responsible
        for stopping motion that cannot follow this reference.
        """
        if not math.isfinite(dt) or not 0 <= dt <= 0.1:
            raise ValueError("Invalid preparation clock interval")
        if self.position_interval != (offset, end):
            # Each mandatory recovery stop is a finite move of its own. Lead
            # may not borrow travel from a later, unchecked-at-this-stage leg.
            self.position_bounds = self.path.position_bounds(offset, end)
            self.position_interval = (offset, end)
        self.scale = 1.0
        self.elapsed = min(end - offset, self.elapsed + dt)
        return self.path.sample(offset + self.elapsed)
