"""Bounded periodic mode-2 feedforward from an explicit response profile.

The profile describes a provisional command-to-shaft response, not motor
inertia. No online identification, iterative learning or transport is used.
"""

from copy import deepcopy
import math

import numpy as np

from .weld_plan import finite_vector, JointPath
from motors.arm_calibration import digest


class WeldResponse:
    """Compile a small, bounded Fourier inverse of the accepted periodic path."""

    def __init__(self, profile, path, plan):
        self.profile = deepcopy(profile)
        if (not isinstance(profile, dict) or profile.get("schema") != "forge.weld_response"
                or type(profile.get("version")) is not int or profile["version"] not in (1, 2)):
            raise ValueError("Expected a version-1 or version-2 weld response profile")
        self.finite_startup = profile["version"] == 2
        if self.finite_startup and profile.get("startup_policy") != "finite_response_inverse":
            raise ValueError("Version-2 response requires finite_response_inverse startup")
        if profile.get("hardware_qualified") is not False:
            raise ValueError("Weld response profiles remain provisional")
        for key in ("mapping_fingerprint", "scene_fingerprint"):
            if not profile.get(key) or profile[key] != plan.get(key):
                raise ValueError(f"Weld response {key} does not match the checked plan")
        if path.interpolation != "periodic_cubic":
            raise ValueError("Weld response requires a checked periodic cubic")
        hz = self.number(profile, "effective_hz", 0.1, 3)
        if abs(plan["effective_hz"] - hz) > 0.005 or abs(path.duration * hz - 1) > 0.005:
            raise ValueError("Weld response requires the recorded frequency and a one-cycle preview")
        low, high = path.position_bounds()
        self.center = (low + high) / 2
        travel = high - low
        expected_center = finite_vector(profile.get("center_raw_deg"), "response center")
        expected_travel = finite_vector(profile.get("travel_deg"), "response travel")
        if np.any(abs(self.center - expected_center) > 0.5):
            raise ValueError("Weld response does not cover this weld pose")
        if np.any(abs(travel - expected_travel) > np.maximum(0.002, 0.02 * expected_travel)):
            raise ValueError("Weld response does not cover this weld amplitude")
        self.path, self.period = path, path.duration
        self.harmonics = profile.get("harmonics")
        if type(self.harmonics) is not int or not 1 <= self.harmonics <= 3:
            raise ValueError("Weld response supports at most three harmonics")
        self.ramp_s = self.period
        self.kp = self.number(profile, "residual_kp", 0, 0.2)
        hold = self.number(profile, "command_hold_s", 0.01, 0.04)
        self.axes = np.zeros(6, dtype=bool)
        self.residual_bound = np.zeros(6)
        self.coefficients = np.zeros((self.harmonics, 6), dtype=complex)
        self.offset = self.center.copy()
        self.physical_response = np.zeros_like(self.coefficients)
        self.input_delay = np.zeros(6)
        self.velocity_inverse = np.zeros(6)
        self.acceleration_inverse = np.zeros(6)
        grid = np.arange(2048) * self.period / 2048
        nominal = path.curve(grid)
        spectrum = np.fft.rfft(nominal, axis=0) / len(grid)
        self.omega = 2 * math.pi * np.arange(1, self.harmonics + 1) / self.period
        phases = np.exp(1j * grid[:, None] * self.omega)
        if not isinstance(profile.get("axes"), dict) or set(profile["axes"]) != {"1", "6"}:
            raise ValueError("This response trial requires the recorded A1 and A6 pair")
        for name, parameters in profile["axes"].items():
            axis = int(name) - 1
            self.axes[axis] = True
            wn = self.number(parameters, "natural_frequency_rad_s", 4, 25)
            damping = self.number(parameters, "damping_ratio", 0.2, 2)
            delay = self.number(parameters, "delay_s", 0, 0.1)
            gain_limit = self.number(parameters, "max_inverse_gain", 1, 2)
            self.residual_bound[axis] = self.number(parameters, "max_residual_deg", 0, 0.25)
            w = self.omega
            physical = wn**2 / (wn**2 - w**2 + 2j * damping * wn * w)
            self.physical_response[:, axis] = physical
            self.input_delay[axis] = delay + hold / 2
            self.velocity_inverse[axis] = 2 * damping / wn
            self.acceleration_inverse[axis] = 1 / wn**2
            # Include the average sample-and-hold phase, separately from the
            # fitted input delay. The inverse never cancels high-frequency noise.
            response = (physical
                        * np.exp(-1j * w * (delay + hold / 2))
                        * np.sinc(w * hold / (2 * math.pi)))
            inverse = 1 / response
            inverse *= np.minimum(1, gain_limit / abs(inverse))
            command = spectrum[1:self.harmonics + 1, axis] * inverse
            predicted = 2 * np.real(phases @ (command * response))
            width = float(np.ptp(predicted))
            if width < 1e-6:
                raise ValueError("Weld response cannot identify a stationary axis")
            scale = travel[axis] / width
            if not 0.75 <= scale <= 1.25:
                raise ValueError("Weld response stroke normalization exceeds its bound")
            self.coefficients[:, axis] = command * scale
            self.offset[axis] -= scale * (predicted.max() + predicted.min()) / 2
        # Triangle bounds cover all phases, including between sampled knots.
        self.command_span_bound = 4 * np.sum(abs(self.coefficients), axis=0)
        limits = plan["stage_limits"]["weld"]
        speed = finite_vector(limits["speed_deg_s"], "response speed limits")
        acceleration = finite_vector(limits["acceleration_deg_s2"], "response acceleration limits")
        peak_rate = 2 * np.sum(abs(self.coefficients) * self.omega[:, None], axis=0)
        peak_acceleration = 2 * np.sum(abs(self.coefficients) * self.omega[:, None]**2, axis=0)
        if np.any(peak_rate > speed) or np.any(peak_acceleration > acceleration):
            raise ValueError("Weld response exceeds the checked command derivative limits")
        # A stationary shaft cannot begin in steady periodic motion. The first
        # cycle explicitly ramps the planned stroke, using the same reference
        # in execution, preview and accuracy scoring. Later cycles are unchanged.
        self.startup_command = None
        self.startup_delay = float(max(self.input_delay)) if self.finite_startup else 0.0
        if self.finite_startup:
            blend = self.envelope(path.times - self.startup_delay)[0]
        else:
            u = path.times / self.period
            blend = u**3 * (10 - 15 * u + 6 * u**2)
        entry = path.positions[0] + blend[:, None] * (path.positions - path.positions[0])
        entry[-1] = entry[0]
        self.entry = JointPath(path.times, entry, "periodic_cubic")
        entry_rate, entry_acceleration = self.entry.peaks()
        if np.any(entry_rate > speed + 1e-8) or np.any(entry_acceleration > acceleration + 1e-8):
            raise ValueError("Weld startup stroke exceeds the checked derivative limits")
        if self.finite_startup:
            # Compile once. Exact cubic extrema cover the entire dispatched
            # startup interpolant, not just geometry-check sample times.
            times = np.linspace(0, self.period, 801)
            targets = self.startup_feedforward(times)
            targets[0] = path.positions[0]
            self.startup_command = JointPath(times, targets, "pchip")
            startup_low, startup_high = self.startup_command.position_bounds()
            half_width = self.command_span_bound / 2
            command_low = np.minimum(self.offset - half_width, startup_low)
            command_high = np.maximum(self.offset + half_width, startup_high)
            self.command_span_bound = np.where(self.axes, command_high - command_low, 0)
            startup_rate, startup_acceleration = self.startup_command.peaks()
            if (np.any(startup_rate > speed + 1e-8)
                    or np.any(startup_acceleration > acceleration + 1e-8)):
                raise ValueError("Weld startup command exceeds the checked derivative limits")
        for axis, maximum in ((0, 4.0), (5, 1.0)):
            if self.command_span_bound[axis] + 2 * self.residual_bound[axis] > maximum:
                raise ValueError(f"A{axis+1} response command exceeds the trial envelope")

    def envelope(self, time):
        """C3 expansion, with reference delay leaving the input at rest at t=0."""
        duration = self.period - self.startup_delay
        u = np.clip(np.asarray(time) / duration, 0, 1)
        value = 35*u**4 - 84*u**5 + 70*u**6 - 20*u**7
        rate = (140*u**3 - 420*u**4 + 420*u**5 - 140*u**6) / duration
        acceleration = (420*u**2 - 1680*u**3 + 2100*u**4 - 840*u**5) / duration**2
        return value, rate, acceleration

    def startup_feedforward(self, elapsed):
        """Inverse of an expanding periodic response, including product terms."""
        elapsed = np.asarray(elapsed)
        phase = np.exp(1j * elapsed[..., None] * self.omega)
        periodic = self.offset + 2 * np.real(phase @ self.coefficients)
        # Predict the pole-filtered response before the input delay. The
        # command spectrum already advances the fitted delay and sample hold.
        future = self.offset + 2 * np.real(phase @ (self.coefficients * self.physical_response))
        future_rate = 2 * np.real(phase @ (
            self.coefficients * self.physical_response * (1j*self.omega[:, None])))
        blend, rate, acceleration = self.envelope(
            elapsed[..., None] + self.input_delay - self.startup_delay)
        start = self.path.positions[0]
        target = (start + blend * (periodic - start)
                  + (self.velocity_inverse * rate + self.acceleration_inverse * acceleration)
                  * (future - start)
                  + 2 * self.acceleration_inverse * rate * future_rate)
        return np.where(self.axes, target, self.entry.curve(elapsed))

    def reference(self, elapsed):
        return (self.entry.sample(elapsed) if elapsed < self.period
                else self.path.sample(elapsed % self.period))

    @staticmethod
    def number(document, key, minimum, maximum):
        value = document.get(key) if isinstance(document, dict) else None
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not minimum <= value <= maximum):
            raise ValueError(f"Invalid weld response parameter: {key}")
        return float(value)

    def sample(self, elapsed, nominal, measured):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Invalid weld response time")
        nominal = finite_vector(nominal, "response nominal")
        measured = finite_vector(measured, "response feedback")
        phase = (elapsed % self.period) * self.omega
        periodic = self.offset + 2 * np.real(np.exp(1j * phase) @ self.coefficients)
        # Blend at startup; every subsequent repeat is periodic without a reset.
        u = min(1.0, elapsed / self.ramp_s)
        blend = (float(self.envelope(elapsed - self.startup_delay)[0]) if self.finite_startup
                 else u**3 * (10 - 15 * u + 6 * u**2))
        residual = np.clip(self.kp * (nominal - measured), -self.residual_bound, self.residual_bound)
        start = self.path.positions[0]
        if self.finite_startup:
            feedforward = (self.startup_command.sample(elapsed)[0]
                           if elapsed < self.period else periodic)
            target = np.where(self.axes, feedforward + blend * residual, nominal)
        else:
            feedforward = periodic
            target = np.where(self.axes, start + blend * (periodic + residual - start), nominal)
        return target, dict(
            policy="bounded_periodic_response", axes=(np.flatnonzero(self.axes) + 1).tolist(),
            blend=blend, residual_deg=residual.tolist(),
            startup_policy=("finite_response_inverse" if self.finite_startup else "periodic_blend"),
            feedforward_raw_deg=np.where(self.axes, feedforward, nominal).tolist(),
        )

    def document(self):
        return dict(profile=deepcopy(self.profile),
                    path_fingerprint=digest(self.path.document()),
                    entry_fingerprint=digest(self.entry.document()),
                    startup_stroke_ramp_s=self.period,
                    startup_policy=("finite_response_inverse" if self.finite_startup else "periodic_blend"),
                    startup_reference_delay_s=self.startup_delay,
                    startup_command_fingerprint=(None if self.startup_command is None
                                                 else digest(self.startup_command.document())),
                    command_span_bound_deg=self.command_span_bound.tolist(),
                    steady_path_unchanged=True, online_learning=False,
                    hardware_qualified=False)

    def check_geometry(self, pose_check):
        """Run in the planning process, never on the motor owner's tick thread."""
        for at in np.linspace(0, self.period, 65):
            nominal = self.path.sample(at)[0]
            target, _ = self.sample(at + 10 * self.period, nominal, nominal)
            for a1_sign in (-1, 1):
                for a6_sign in (-1, 1):
                    corner = target.copy()
                    corner[[0, 5]] += self.residual_bound[[0, 5]] * [a1_sign, a6_sign]
                    pose_check(corner)
            entry = self.entry.sample(at)[0]
            pose_check(entry)
            target, _ = self.sample(at, entry, entry)
            for a1_sign in (-1, 1):
                for a6_sign in (-1, 1):
                    corner = target.copy()
                    corner[[0, 5]] += self.residual_bound[[0, 5]] * [a1_sign, a6_sign]
                    pose_check(corner)
        return dict(self.document(), geometry_checked=True)
