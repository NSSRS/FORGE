"""Bounded host position tracking for the existing native mode-2 drive loop.

All angles use raw output-shaft degrees. The velocity input to mode 2 is a
ceiling, not feedforward: velocity feedforward therefore becomes a position
lead. No drive gains, current parameters, transport or motor modes are changed.
"""

from dataclasses import asdict, dataclass
import math

import numpy as np

from .weld_plan import finite_vector


@dataclass(frozen=True)
class TrackingSettings:
    # Provisional local response estimates from the Nov 17 recorded approaches:
    # median same-sign position error / measured velocity, |v| > 0.7 deg/s and
    # |error| > 0.15 deg. Not a load-independent identified motor model.
    lead_time_s: tuple = (0.16, 0.76, 0.27, 0.14, 0.22, 0.25)
    kp: float = 0.3
    ki_per_s: float = 0.3
    kd_s: float = 0.01
    velocity_filter_s: float = 0.08
    max_correction_deg: float = 12.0
    max_integral_deg: float = 0.5
    # The provisional lag model was identified above 0.15 deg position error.
    # Sub-0.3 deg peak-to-peak weld motions cannot support that model reliably.
    minimum_weld_travel_deg: float = 0.3
    weld_a1_shaper_only: bool = False

    def __post_init__(self):
        if type(self.weld_a1_shaper_only) is not bool:
            raise ValueError("The A1 weld trial switch must be a boolean")
        lead = finite_vector(self.lead_time_s, "tracking lead times")
        if np.any(lead < 0) or np.any(lead > 1):
            raise ValueError("Tracking lead times must be between 0 and 1 second")
        for name, maximum in (("kp", 2), ("ki_per_s", 1), ("kd_s", 0.1),
                              ("velocity_filter_s", 0.5), ("max_correction_deg", 12),
                              ("max_integral_deg", 1), ("minimum_weld_travel_deg", 1)):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or not 0 <= value <= maximum:
                raise ValueError(f"Invalid tracking setting: {name}")
        if self.velocity_filter_s == 0 or self.max_correction_deg == 0:
            raise ValueError("Tracking filter and correction bound must be positive")

    def document(self):
        return dict(
            version=3, **asdict(self), hardware_qualified=False,
            initial_estimate_source="weld-1794907065402842317.jsonl",
            adaptive_lead="bounded 0–1 s during preparation; frozen during welding",
            preparation_braking="command stays inside finite path extrema with acceleration-bounded braking",
            small_weld_motion="nominal targets retained; no host PID lead below minimum_weld_travel_deg",
        )


class PositionTracker:
    """Velocity feedforward + PID correction, followed by command shaping.

    Derivative feedback uses filtered measured velocity, never a finite
    difference of quantized position. Conditional integration prevents windup
    during lead/speed/acceleration saturation. The caller must check corrected
    commands against geometry and raw limits before dispatch.
    """

    def __init__(self, settings=None):
        self.settings = settings or TrackingSettings()
        self.command = None
        self.velocity = np.zeros(6)
        self.filtered_velocity = np.zeros(6)
        self.integral = np.zeros(6)
        self.lead_time = np.asarray(self.settings.lead_time_s, dtype=float).copy()
        self.last = None

    def reset(self, raw, *, reset_model=False):
        self.command = finite_vector(raw, "tracking initial command").copy()
        self.velocity[:] = 0
        self.filtered_velocity[:] = 0
        self.integral[:] = 0
        self.last = None
        if reset_model:
            self.lead_time = np.asarray(self.settings.lead_time_s, dtype=float).copy()

    def step(self, reference, rate, measured, velocity, dt, speed_limits,
             acceleration_limits, *, endpoint=False, project=None, remaining_s=None,
             adapt=True, position_bounds=None, correction_mask=None):
        reference = finite_vector(reference, "tracking reference")
        rate = finite_vector(rate, "tracking reference velocity")
        measured = finite_vector(measured, "tracking measurement")
        velocity = finite_vector(velocity, "tracking measured velocity")
        speed = finite_vector(speed_limits, "tracking speed limits")
        acceleration = finite_vector(acceleration_limits, "tracking acceleration limits")
        if np.any(speed <= 0) or np.any(acceleration <= 0):
            raise ValueError("Tracking limits must be positive")
        if not math.isfinite(dt) or not 0 <= dt <= 0.1:
            raise ValueError("Invalid tracking interval")
        if remaining_s is not None and (not math.isfinite(remaining_s) or remaining_s < 0):
            raise ValueError("Invalid remaining trajectory time")
        if self.command is None:
            raise ValueError("Reset the tracker at a stationary stage boundary first")
        if dt == 0:
            return self.command.copy()
        cfg = self.settings
        error = reference - measured
        # Update a bounded local lag estimate from the PREVIOUS dispatched
        # target and current cached shaft feedback. This reduces unnecessary
        # lead on faster shafts; zero/slow/noisy motion cannot identify lag.
        moving = abs(velocity) >= 0.7
        lag = self.command - measured
        consistent = lag * velocity >= -0.02
        estimate = np.clip(np.divide(lag, velocity, out=np.zeros(6), where=moving) + dt, 0, 1)
        if adapt:
            self.lead_time += np.where(moving & consistent,
                                       dt / (0.2 + dt) * (estimate - self.lead_time), 0)
        self.filtered_velocity += dt / (cfg.velocity_filter_s + dt) * (
            velocity - self.filtered_velocity
        )
        candidate_integral = np.clip(
            self.integral + cfg.ki_per_s * error * dt,
            -cfg.max_integral_deg, cfg.max_integral_deg,
        )
        correction = (self.lead_time * rate + cfg.kp * error
                      + candidate_integral + cfg.kd_s * (rate - self.filtered_velocity))
        if remaining_s is not None:
            # Fade position pre-emphasis before a finite endpoint so the inner
            # loop and command shaper can brake into the exact unshifted hold.
            u = np.clip(remaining_s / max(0.2, 4 * max(self.lead_time)), 0, 1)
            correction *= u * u * (3 - 2 * u)
        if endpoint:
            # No residual PID offset may become a new endpoint hold target.
            correction = np.zeros(6)
            candidate_integral[:] = 0
            self.integral[:] = 0
        if correction_mask is not None:
            mask = np.asarray(correction_mask)
            if mask.shape != (6,) or mask.dtype != bool:
                raise ValueError("Tracking correction mask requires six booleans")
            # Keep the original small-motion reference. An unobservable local
            # lag estimate must not add a larger virtual move, constrain shared
            # clearance projection, or accumulate an offset for later stages.
            correction = np.where(mask, correction, 0.0)
            candidate_integral = np.where(mask, candidate_integral, 0.0)
            self.integral = np.where(mask, self.integral, 0.0)
        limited = np.clip(correction, -cfg.max_correction_deg, cfg.max_correction_deg)
        target = reference + limited
        if position_bounds is not None:
            low = finite_vector(position_bounds[0], "tracking position lower bounds")
            high = finite_vector(position_bounds[1], "tracking position upper bounds")
            if (np.any(low > high) or np.any(self.command < low - 1e-9)
                    or np.any(self.command > high + 1e-9)):
                raise ValueError("Tracking command is outside the finite path envelope")
            # A velocity lead must never ask a finite preparation move to pass
            # its checked extrema and then reverse back into the endpoint.
            target = np.clip(target, low, high)
        if project is not None:
            target = finite_vector(project(reference, target), "projected tracking target")
        distance = target - self.command
        dv = acceleration * dt
        # Follow the moving position demand subject to the command derivative
        # bounds. Brake into a finite endpoint instead of braking at every tick.
        wanted_velocity = np.clip(rate + 10.0 * distance, -speed, speed)
        if endpoint:
            # Reserve this step's travel plus acceleration-bounded stopping
            # distance before the stationary final target.
            stopping_speed = np.sqrt(dv**2 + 2 * acceleration * abs(distance)) - dv
            wanted_velocity = np.sign(distance) * np.minimum(speed, stopping_speed)
            near = (abs(distance) <= dv * dt) & (abs(self.velocity) <= dv)
            wanted_velocity = np.where(near, distance / dt, wanted_velocity)
        next_velocity = np.clip(wanted_velocity, self.velocity - dv, self.velocity + dv)
        next_velocity = np.clip(next_velocity, -speed, speed)
        if position_bounds is not None:
            # Brake BEFORE reaching an extremum, not only after the reference
            # clock reaches its endpoint. For next velocity v, reserve both
            # this step's travel v*dt and stopping distance v²/(2*a). The
            # remaining distance then still permits braking on a longer tick.
            negative = np.sqrt(dv**2 + 2 * acceleration * np.maximum(
                0, self.command - low)) - dv
            positive = np.sqrt(dv**2 + 2 * acceleration * np.maximum(
                0, high - self.command)) - dv
            lower = np.maximum(self.velocity - dv, -np.minimum(speed, negative))
            upper = np.minimum(self.velocity + dv, np.minimum(speed, positive))
            if np.any(lower > upper + 1e-8):
                raise ValueError("Tracking cannot brake within the finite path envelope")
            next_velocity = np.clip(wanted_velocity, lower, np.maximum(lower, upper))
        command = self.command + next_velocity * dt
        # A saturated actuator must not accumulate more integral in the same
        # direction. Permit unwinding and freeze axes that cannot realize it.
        residual = reference + correction - command
        accept = residual * error <= 0
        self.integral = np.where(accept, candidate_integral, self.integral)
        self.command, self.velocity = command, next_velocity
        self.last = dict(
            command_raw_deg=command.tolist(),
            correction_deg=(command - reference).tolist(),
            target_correction_deg=limited.tolist(),
            integral_deg=self.integral.tolist(),
            lead_time_s=self.lead_time.tolist(),
            command_rate_deg_s=next_velocity.tolist(),
            limited=bool(np.any(abs(command - target) > 1e-6)),
            projected=bool(np.any(abs(target - reference - limited) > 1e-6)),
            correction_axes=(list(range(1, 7)) if correction_mask is None
                             else (np.flatnonzero(correction_mask) + 1).tolist()),
        )
        return command.copy()

    def at_endpoint(self, endpoint):
        return bool(np.max(abs(self.command - endpoint)) < 1e-9
                    and np.max(abs(self.velocity)) < 1e-9)
