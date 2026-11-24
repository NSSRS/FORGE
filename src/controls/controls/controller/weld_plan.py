"""Offline, model-checked weld paths in raw output-shaft degrees.

No motor transport is imported here. The simulation's task-space path and fixed
tool geometry are reused; its PD/gravity-compensation commands are not replayed.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import PchipInterpolator, CubicSpline

from motors.arm_calibration import CalibrationMap, DRIVES, digest
from .limits import (
    MAX_PREPARATION_SPEED_DEG_S,
    motor_constraints,
    dynamics_constraints,
    trajectory_feasibility,
)
from .recovery import recovery_path, START_TOLERANCE_DEG

APPROACH_LOWERING_SECONDS = 2.0


def finite_vector(values, label, size=6):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label}: expected {size} finite values")
    return result


def positive(value, label):
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")
    return float(value)


def initial_from_model(pose, mapping):
    """D4/D6 use their authored zero when absent from initial_pose.json."""
    missing = set(DRIVES) - set(pose["joint_angles_rad"]) - {DRIVES[3], DRIVES[5]}
    if missing:
        raise ValueError(f"Missing initial model angles: {sorted(missing)}")
    return [
        mapping.raw(i, math.degrees(pose["joint_angles_rad"].get(name, 0.0)))
        for i, name in enumerate(DRIVES, 1)
    ]


@dataclass(frozen=True)
class Settings:
    hz: float = 2.0
    amplitude_m: float = 0.010
    feed_m_s: float = 0.012
    cycles: int = 6
    time_scale: float = 1.0
    max_speed_deg_s: float = 5.0
    max_accel_deg_s2: float = 30.0
    control_period_s: float = 0.02
    auto_retime: bool = True
    lock_base_yaw: bool = False
    limit_profile: str = "debug"
    search_solution: dict | None = None
    preparation_target_s: float = 5.0
    optimize_preparation: bool = False
    approach_lift_m: float = 0.0
    lowering_min_s: float = 0.0
    preparation_planner: str = "direct-first"

    def __post_init__(self):
        for name in (
            "hz",
            "time_scale",
            "max_speed_deg_s",
            "max_accel_deg_s2",
            "control_period_s",
            "preparation_target_s",
        ):
            positive(getattr(self, name), name)
        if not 0 < self.hz <= 3 or not 1 <= self.time_scale <= 1000:
            raise ValueError("Require 0 < frequency <=3 Hz and 1 <= time scale <=1000")
        if self.max_speed_deg_s > MAX_PREPARATION_SPEED_DEG_S or self.max_accel_deg_s2 > 30:
            raise ValueError(
                f"Preparation request requires speed <={MAX_PREPARATION_SPEED_DEG_S:g} deg/s "
                "and acceleration <=30 deg/s²; motor guards also apply"
            )
        if not 0 <= self.amplitude_m <= 0.01 or not 0 <= self.feed_m_s <= 0.012:
            raise ValueError(
                "This floor-seam experiment supports amplitude <=10 mm and feed <=12 mm/s"
            )
        if type(self.cycles) is not int or not 1 <= self.cycles <= 100:
            raise ValueError("Cycles must be an integer from 1 to 100")
        if not 0.01 <= self.control_period_s <= 0.02:
            raise ValueError("Control period must be 10–20 ms")
        if type(self.auto_retime) is not bool or type(self.lock_base_yaw) is not bool:
            raise ValueError("Solver switches must be booleans")
        if type(self.optimize_preparation) is not bool:
            raise ValueError("Preparation optimization switch must be boolean")
        if self.preparation_planner not in ("direct-first", "staged"):
            raise ValueError("Unknown preparation planner")
        if (
            isinstance(self.approach_lift_m, bool)
            or not math.isfinite(self.approach_lift_m)
            or not 0 <= self.approach_lift_m <= 0.15
        ):
            raise ValueError("Approach lift must be between 0 and 150 mm")
        if (isinstance(self.lowering_min_s, bool)
                or not math.isfinite(self.lowering_min_s) or self.lowering_min_s < 0):
            raise ValueError("Lowering duration must be finite and nonnegative")
        if self.lowering_min_s and not self.approach_lift_m:
            raise ValueError("Lowering duration requires an explicit approach lift")
        if self.limit_profile not in ("debug", "weld"):
            raise ValueError("Unknown constraint profile")
        if self.search_solution is not None and (
            not isinstance(self.search_solution, dict) or self.lock_base_yaw
        ):
            raise ValueError(
                "Search solution must be a document and cannot be combined with locked base yaw"
            )


class JointPath:
    """Read-only sample arrays with reusable interpolant derivatives and extrema."""

    def __init__(self, times, positions, interpolation="pchip"):
        self.times = np.array(times, dtype=float, copy=True)
        self.positions = np.array(positions, dtype=float, copy=True)
        if (
            self.times.ndim != 1
            or len(self.times) < 2
            or self.times[0] != 0
            or self.positions.shape != (len(self.times), 6)
            or not np.all(np.isfinite(self.times))
            or not np.all(np.isfinite(self.positions))
            or np.any(np.diff(self.times) <= 0)
        ):
            raise ValueError("Invalid six-axis time-indexed path")
        self.interpolation = interpolation
        if interpolation == "periodic_cubic":
            self.curve = CubicSpline(
                self.times,
                self.positions,
                bc_type="periodic",
                axis=0,
                extrapolate=False,
            )
        elif interpolation == "pchip":
            self.curve = PchipInterpolator(
                self.times, self.positions, axis=0, extrapolate=False
            )
        else:
            raise ValueError("Unknown trajectory interpolant")
        self.times.setflags(write=False)
        self.positions.setflags(write=False)
        self._velocity = self.curve.derivative()
        self._peaks = None
        self.duration = float(self.times[-1])

    def sample(self, elapsed):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Invalid trajectory clock")
        t = min(elapsed, self.duration)
        return self.curve(t), self._velocity(t)

    def peaks(self):
        if self._peaks is not None:
            return tuple(value.copy() for value in self._peaks)
        # Exact extrema for each piecewise cubic, including either side of knots.
        speed = np.zeros(6)
        accel = np.zeros(6)
        for axis in range(6):
            curve = (
                CubicSpline(
                    self.times,
                    self.positions[:, axis],
                    bc_type="periodic",
                    extrapolate=False,
                )
                if self.interpolation == "periodic_cubic"
                else PchipInterpolator(
                    self.times, self.positions[:, axis], extrapolate=False
                )
            )
            roots = curve.derivative(2).roots(extrapolate=False)
            roots = roots[np.isfinite(roots)]
            times = np.r_[self.times, roots, np.maximum(0, self.times[1:] - 1e-10)]
            speed[axis] = np.max(np.abs(curve.derivative()(times)))
            accel[axis] = np.max(np.abs(curve.derivative(2)(times)))
        self._peaks = speed, accel
        return speed.copy(), accel.copy()

    def document(self):
        return dict(
            times_s=self.times.tolist(),
            raw_deg=self.positions.tolist(),
            interpolation=self.interpolation,
        )

    def position_bounds(self, start=0.0, end=None):
        """Exact cubic extrema within an optional finite path interval."""
        end = self.duration if end is None else end
        if not (math.isfinite(start) and math.isfinite(end)
                and 0 <= start <= end <= self.duration):
            raise ValueError("Invalid trajectory position interval")
        low, high = np.empty(6), np.empty(6)
        all_roots = self._velocity.roots(extrapolate=False)
        for axis in range(6):
            roots = all_roots[axis]
            roots = roots[np.isfinite(roots) & (roots >= start) & (roots <= end)]
            knots = self.times[(self.times >= start) & (self.times <= end)]
            times = np.r_[start, end, knots, roots]
            values = self.curve(times)[:, axis]
            low[axis], high[axis] = np.min(values), np.max(values)
        return low, high

    def check_raw_bounds(self, bounds):
        low, high = self.position_bounds()
        if not np.isfinite([low, high]).all():
            raise ValueError("Nonfinite trajectory position envelope")
        for axis, (lo, hi) in enumerate(bounds):
            if low[axis] < lo or high[axis] > hi:
                raise ValueError(
                    f"A{axis+1}: trajectory exceeds fixed travel envelope; turn back without wrapping"
                )

    @classmethod
    def from_document(cls, value):
        return cls(
            value["times_s"], value["raw_deg"], value.get("interpolation", "pchip")
        )


def move_path(start, goal, settings):
    start, goal = finite_vector(start, "start"), finite_vector(goal, "goal")
    distance = float(np.max(np.abs(goal - start)))
    # Quintic endpoints have zero velocity and acceleration. Margin covers PCHIP.
    duration = max(
        0.5,
        2.1 * distance / settings.max_speed_deg_s,
        math.sqrt(7.0 * distance / settings.max_accel_deg_s2),
    )
    t = np.linspace(0, duration, max(21, math.ceil(duration / 0.02) + 1))
    u = t / duration
    s = 10 * u**3 - 15 * u**4 + 6 * u**5
    positions = start[None, :] + s[:, None] * (goal - start)[None, :]
    # Preserve exact shared endpoints; rounding in start + (goal - start)
    # must not turn a stationary join into a tiny moving cubic.
    positions[0], positions[-1] = start, goal
    return JointPath(t, positions)


class WeldGeometry:
    def __init__(self, mapping, *, clearance_mm=0.5, arm_contacts_only=True,
                 scene_identity=None, enforce_spatial_clearance=True):
        import mujoco as mj
        from controls.simulation.weave_xy_z import SEAM_XY, WIRE_GAP
        from simulation.arm_calibration import ExplicitStops
        from simulation.arm_sweep import descendant_bodies
        from simulation.arm_closure import ArmClosure

        if type(enforce_spatial_clearance) is not bool:
            raise ValueError("Spatial clearance enforcement must be boolean")
        self.enforce_spatial_clearance = enforce_spatial_clearance
        self.mj, self.mapping = mj, mapping
        if scene_identity is None:
            from simulation.arm_calibration import scene_fingerprint
            from simulation.paths import MODEL_DIR
            scene_identity = scene_fingerprint(MODEL_DIR / "robot.xml")
        from .planning_cache import scene_copy
        self.model, self.data = scene_copy(scene_identity)
        self.origin = np.array(
            [*SEAM_XY, self.data.geom("preview_floor").xpos[2] + WIRE_GAP]
        )
        self.addresses = [int(self.model.joint(n).qposadr[0]) for n in DRIVES]
        self.seed = self.data.qpos.copy()
        self.closure = ArmClosure(self.model, self.data, forward=False)
        self.arm_bodies = descendant_bodies(self.model, self.model.body(DRIVES[0]).id)
        self.arm_geoms = np.isin(self.model.geom_bodyid, list(self.arm_bodies))
        from .collision_filter import retain_arm_pairs
        self.arm_contacts_only = arm_contacts_only and retain_arm_pairs(self.model, self.arm_geoms)
        self.clearance = clearance_mm / 1000
        self.model.geom_margin[:] = np.maximum(
            self.model.geom_margin, max(0.005, self.clearance * 2)
        )
        self.model.geom_gap[:] = self.model.geom_margin
        self.stops = ExplicitStops(
            self.model, self.data, mapping.document.get("adjacent_stop_pairs", [])
        )
        # These concave CAD parts overlap only in their convex envelopes at the
        # recorded lowered pose. Refine this reviewed pair with the full triangle
        # surfaces; keep every other contact filter and clearance unchanged.
        self.concave_pair = ("collision_139", "collision_463")
        self.concave = ExplicitStops(self.model, self.data, [self.concave_pair])
        self.contact_report = []
        # Request-local reuse only. All complete approach paths still receive
        # their own clearance validation after joining to the measured start.
        self.lowering_cache = {}

    def inspect_contacts(self, clearance_mm):
        """Report convex candidates and the reviewed pair's mesh narrow phase."""
        candidates = {}
        contacts = self.data.contact
        # Most floor/chassis contacts are irrelevant to the arm. Filter the
        # native arrays once instead of constructing Python rows for each one.
        relevant = ((contacts.dist * 1000 < clearance_mm)
                    & np.any(self.arm_geoms[contacts.geom], axis=1))
        for index in np.flatnonzero(relevant):
            contact = contacts[int(index)]
            ids = tuple(int(g) for g in contact.geom)
            names = tuple(self.model.geom(g).name for g in ids)
            row = dict(
                geom_names=list(names),
                distance_mm=float(contact.dist) * 1000,
                convex_distance_mm=float(contact.dist) * 1000,
                point=np.asarray(contact.pos).tolist(),
                basis="convex hull",
            )
            key = tuple(sorted(names))
            if (
                key not in candidates
                or row["distance_mm"] < candidates[key]["distance_mm"]
            ):
                candidates[key] = row
        key = tuple(sorted(self.concave_pair))
        if key in candidates:
            measured = next(self.concave.measure(nearest_points=True))
            candidates[key].update(measured, basis="triangle mesh")
            if "nearest_points" in measured:
                candidates[key]["point"] = np.mean(
                    measured["nearest_points"], axis=0
                ).tolist()
        for row in candidates.values():
            row["warning"] = row["distance_mm"] < clearance_mm
            row["blocked"] = row["warning"] and self.enforce_spatial_clearance
        return list(candidates.values())

    def check(self, raw, *, clearance_mm=None):
        return self._check(raw, clearance_mm=clearance_mm)

    def observe(self, raw, *, clearance_mm=2.0):
        """Inspect actual encoder coordinates without rejecting a model pose range."""
        return self._check(raw, clearance_mm=clearance_mm, observation=True)

    def check_recovery(self, raw, document, *, clearance_mm=None, measured=False):
        from .measured_start import MeasuredStart

        start = MeasuredStart.from_document(document)
        start.check_raw(self.mapping, raw, measured=measured)
        return self._check(raw, clearance_mm=clearance_mm, measured_start=start,
                           measured=measured)

    def check_endpoint(self, raw, policy):
        from .endpoint_recovery import A3Retreat, STOP_PAIR, MAX_OVERLAP_MM

        policy = A3Retreat.from_document(policy)
        policy.check_mapping(self.mapping)
        q = policy.check_raw(raw)
        self._check(q, clearance_mm=2.0, endpoint=policy)
        distance = self.stops.signed_clearance(STOP_PAIR)
        if distance < -MAX_OVERLAP_MM:
            raise ValueError("A3 modeled overlap exceeds the reviewed shallow-contact envelope")
        if distance < 0.1:
            inward = q.copy(); inward[2] -= 0.01
            self._check(inward, clearance_mm=2.0, endpoint=policy)
            following = self.stops.signed_clearance(STOP_PAIR)
            self._check(q, clearance_mm=2.0, endpoint=policy)
            if following <= distance + 1e-8:
                raise ValueError("A3 inward motion does not separate the modeled stop contact")
        return distance

    def _check(self, raw, *, clearance_mm=None, endpoint=None, measured_start=None,
               observation=False, measured=False):
        self.contact_report = []
        clearance = (
            self.clearance
            if clearance_mm is None
            else positive(clearance_mm, "clearance") / 1000
        )
        # The live guard uses the same contact envelope as offline recovery.
        if clearance * 2 > float(np.min(self.model.geom_margin)):
            raise ValueError(
                "Requested clearance exceeds the configured contact envelope"
            )
        raw = finite_vector(raw, "raw pose")
        if observation or measured_start is not None:
            angles = {str(i): self.mapping.joints[str(i)]['model_per_raw'] * q
                      + self.mapping.joints[str(i)]['model_intercept_deg']
                      for i, q in enumerate(raw, 1)}
        elif endpoint is None:
            angles = self.mapping.pose({str(i): float(q) for i, q in enumerate(raw, 1)})
        else:
            endpoint.check_raw(raw)
            angles = {}
            for i, q in enumerate(raw, 1):
                row = self.mapping.joints[str(i)]
                angles[str(i)] = (row['model_per_raw'] * q + row['model_intercept_deg']
                                  if i == 3 else self.mapping.model(i, float(q)))
        for i, name in enumerate(DRIVES):
            jid = self.model.joint(name).id
            q = math.radians(angles[str(i + 1)])
            low, high = self.model.jnt_range[jid]
            if measured_start is not None:
                row = self.mapping.joints[str(i + 1)]
                observed = math.radians(row['model_per_raw'] * measured_start.raw[i]
                                        + row['model_intercept_deg'])
                tolerance = math.radians(START_TOLERANCE_DEG) if measured else 0
                low = min(low, observed - tolerance if observed < low else observed)
                high = max(high, observed + tolerance if observed > high else observed)
            epsilon = 1e-12 if measured_start is not None else 0
            if (
                not observation and self.model.jnt_limited[jid]
                and not low - epsilon <= q <= high + epsilon
            ):
                raise ValueError(f"A{i+1}: authored model joint limit")
            self.data.qpos[self.addresses[i]] = q
        if not self.closure.apply(self.data):
            raise ValueError("D3 linkage cannot close at the planned pose")
        # Clearance needs transforms and contacts, not the dynamics/constraint
        # solve. Inertia/bias evaluation explicitly calls mj_forward itself.
        self.mj.mj_collision(self.model, self.data)
        self.contact_report = self.inspect_contacts(clearance * 1000)
        warnings = []
        # Hardware model space is advisory throughout motion, including tracker
        # corrections. Simulation still enforces it after the observed knot.
        # Explicit mechanical stops below remain independent of this policy.
        observed_knot = (measured_start is not None
                         and np.max(abs(raw - measured_start.raw)) <= 1e-9)
        for contact in self.contact_report:
            if contact.get("warning", contact["blocked"]):
                detail = (
                    "triangle mesh intersection"
                    if contact["basis"] == "triangle mesh"
                    and contact["distance_mm"] < 0
                    else f"{contact['distance_mm']:.3f} mm ({contact['basis']})"
                )
                message = f"Arm/environment clearance: {contact['geom_names']}, {detail}"
                if self.enforce_spatial_clearance and not (observation or observed_knot):
                    raise ValueError(message)
                if not self.enforce_spatial_clearance:
                    message += " (advisory; does not block hardware motion)"
                warnings.append(message)
        from .endpoint_recovery import STOP_PAIR
        for stop in self.stops.measure():
            if endpoint is not None and tuple(stop['geom_names']) == STOP_PAIR:
                continue
            if stop['distance_mm'] < 0.1:
                message = f"Adjacent mechanical stop: {stop}"
                if not (observation or observed_knot):
                    raise ValueError(message)
                warnings.append(message)
        return warnings if observation or measured_start is not None or not self.enforce_spatial_clearance else None

    def solve(
        self, target, *, free_yaw=False, preference=None, weights=None, fixed_yaw=0.0
    ):
        from scipy.optimize import least_squares
        from controls.simulation.weave_xy_z import TARGET_WIRE
        from simulation.arm_closure import ArmClosure

        fitted = self.mj.MjData(self.model)
        self.mj.mj_copyData(fitted, self.model, self.data)
        closure = ArmClosure(self.model, fitted)
        first = 0 if free_yaw else 1
        coords = np.r_[
            self.addresses[first:], int(self.model.joint("b4_2_arm_d3").qposadr[0])
        ]
        joints = [self.model.joint(n).id for n in DRIVES[first:]] + [closure.bearing]
        bounds = np.array(
            [
                (
                    self.model.jnt_range[j]
                    if self.model.jnt_limited[j]
                    else [-np.pi, np.pi]
                )
                for j in joints
            ]
        ).T
        for i in range(first, 6):
            row = self.mapping.joints[str(i + 1)]
            measured = np.sort(
                np.radians(
                    np.asarray(self.mapping.raw_limits(i + 1)) * row["model_per_raw"]
                    + row["model_intercept_deg"]
                )
            )
            if i == 3 and not self.model.jnt_limited[joints[i - first]]:
                # A4 has a cable envelope, not a periodic +/-pi IK coordinate.
                bounds[:, i - first] = measured
            bounds[0, i - first] = max(
                bounds[0, i - first], measured[0] + math.radians(0.1)
            )
            bounds[1, i - first] = min(
                bounds[1, i - first], measured[1] - math.radians(0.1)
            )
        if not free_yaw:
            fitted.qpos[self.addresses[0]] = fixed_yaw
        anchor = (
            fitted.qpos[self.addresses].copy()
            if preference is None
            else np.asarray(preference)
        )
        weights = np.ones(6) if weights is None else np.asarray(weights)
        rod_length = np.linalg.norm(
            self.data.site_xpos[closure.sites[1]] - self.data.xanchor[closure.rod]
        )

        def residual(q):
            fitted.qpos[coords] = q
            self.mj.mj_kinematics(self.model, fitted)
            wire = fitted.site("wire_tip").xmat.reshape(3, 3)[:, 2]
            rod = fitted.site_xpos[closure.sites[0]] - fitted.xanchor[closure.rod]
            task = np.r_[
                fitted.site("wire_tip").xpos - target,
                0.12 * (wire - TARGET_WIRE),
                np.linalg.norm(rod) - rod_length,
            ]
            return (
                np.r_[task, 0.00005 * weights * (fitted.qpos[self.addresses] - anchor)]
                if free_yaw
                else task
            )

        fit = least_squares(
            residual,
            np.clip(fitted.qpos[coords], *bounds),
            bounds=bounds,
            gtol=1e-12,
            ftol=1e-10,
            xtol=1e-10,
            max_nfev=400,
        )
        fitted.qpos[coords] = fit.x
        if not closure.apply(fitted):
            raise ValueError("IK cannot close the D3 linkage")
        self.mj.mj_forward(self.model, fitted)
        wire = fitted.site("wire_tip").xmat.reshape(3, 3)[:, 2]
        if np.linalg.norm(
            fitted.site("wire_tip").xpos - target
        ) > 0.0006 or wire @ TARGET_WIRE < math.cos(math.radians(0.2)):
            raise ValueError("IK cannot meet the wire position/orientation tolerance")
        self.data.qpos[:] = fitted.qpos
        raw = [
            self.mapping.raw(i, math.degrees(fitted.qpos[address]))
            for i, address in enumerate(self.addresses, 1)
        ]
        self.check(raw)
        return raw

    def validate_path(self, path, *, clearance_mm=None, raw_bounds=None, pose_check=None):
        from .path_sampling import geometry_times

        path.check_raw_bounds([self.mapping.raw_limits(i) for i in range(1, 7)]
                              if raw_bounds is None else raw_bounds)
        check = self.check if pose_check is None else pose_check
        times = geometry_times(path)
        positions = path.curve(times)
        # Reject blocked endpoints/interiors before an expensive full scan.
        # These are a subset of the same final samples, never an acceptance test.
        if len(times) > 32:
            saved_qpos, saved_branch = self.data.qpos.copy(), self.closure.last.copy()
            try:
                for index in np.linspace(0, len(times) - 1, 17, dtype=int):
                    check(positions[index], clearance_mm=clearance_mm)
            finally:
                # Coarse rejection must not choose the starting D3 branch for
                # the authoritative ordered traversal that follows.
                self.data.qpos[:] = saved_qpos
                self.closure.last = saved_branch
        for raw in positions:
            check(raw, clearance_mm=clearance_mm)


def smooth_level(phase):
    """Same stroke and dwell intervals as Shape 1; C2 quintic ramp joins."""
    from controls.weave_xy_z import PHASE, LEVEL

    phase = np.asarray(phase) % 1.0
    i = np.clip(np.searchsorted(PHASE, phase, side="right") - 1, 0, len(PHASE) - 2)
    u = (phase - PHASE[i]) / (PHASE[i + 1] - PHASE[i])
    return LEVEL[i] + (LEVEL[i + 1] - LEVEL[i]) * (10 * u**3 - 15 * u**4 + 6 * u**5)


def candidate_weights(candidate):
    mobility = finite_vector(candidate["mobility_weights"], "mobility weights")
    if np.any(mobility < 0.001) or np.any(mobility > 1000):
        raise ValueError("Unsupported mobility weights")
    for name in ("center_yaw_deg", "wrist_seed_deg"):
        value = candidate[name]
        if isinstance(value, bool) or not math.isfinite(value) or abs(value) > 180:
            raise ValueError("Invalid candidate seed")
    penalty = 1.0 / mobility
    return penalty * np.sqrt(6) / np.linalg.norm(penalty)


def weave_path(
    geometry,
    settings,
    *,
    free_yaw=False,
    weights=None,
    candidate=None,
    spatial_steps=20,
):
    """MuJoCo closed-chain IK, spatial continuation, then smooth timing.

    Solve each transverse location once and retrace the same branch. This avoids
    null-space drift over cycles and avoids over-constraining rotation about the
    wire. Chassis participation never enters this geometric solve.
    """
    geometry.data.qpos[:] = geometry.seed
    yaw = 0.0
    if candidate is not None:
        weights = candidate_weights(candidate)
        free_yaw = True
        yaw = math.radians(candidate["center_yaw_deg"])
        roll = math.radians(candidate["wrist_seed_deg"])
        geometry.data.qpos[geometry.addresses[3]] += roll
        geometry.data.qpos[geometry.addresses[5]] -= roll
    center = geometry.solve(geometry.origin, fixed_yaw=yaw)
    anchor = geometry.data.qpos[geometry.addresses].copy()
    qcenter = geometry.data.qpos.copy()
    if settings.amplitude_m == 0:
        poses = np.tile(center, (201, 1))
    else:
        arms = []
        for direction in (-1, 1):
            geometry.data.qpos[:] = qcenter
            positions = [center]
            for y in np.linspace(
                0, direction * settings.amplitude_m, spatial_steps + 1
            )[1:]:
                positions.append(
                    geometry.solve(
                        geometry.origin + [0, y, 0],
                        free_yaw=free_yaw,
                        preference=anchor,
                        weights=weights,
                    )
                )
            arms.append(positions)
        positions = np.vstack([arms[0][::-1], arms[1][1:]])
        spatial = CubicSpline(
            np.linspace(
                -settings.amplitude_m, settings.amplitude_m, 2 * spatial_steps + 1
            ),
            positions,
            axis=0,
        )
        poses = spatial(settings.amplitude_m * smooth_level(np.linspace(0, 1, 201)))
    poses[-1] = poses[0]
    nominal = np.linspace(0, 1 / settings.hz, 201)
    times = (
        np.concatenate(
            [nominal[:-1] + i / settings.hz for i in range(settings.cycles)]
            + [np.array([settings.cycles / settings.hz])]
        )
        * settings.time_scale
    )
    points = np.vstack([poses[:-1] for _ in range(settings.cycles)] + [poses[-1:]])
    return JointPath(times, points, "periodic_cubic")


def retime(path, speed_limits, accel_limits, *, automatic=True):
    speed, accel = path.peaks()
    ratio = max(
        1.0,
        float(np.max(speed / speed_limits)),
        math.sqrt(float(np.max(accel / accel_limits))),
    )
    if not automatic and ratio > 1.0 + 1e-6:
        raise ValueError(f"Requested timing is infeasible; required scale {ratio:.6f}")
    factor = ratio * 1.02 if ratio > 1 else 1.0
    if factor == 1.0:
        return path, factor
    return JointPath(path.times * factor, path.positions, path.interpolation), factor


def validate_weave(geometry, path, settings, *, samples=401):
    from controls.simulation.weave_xy_z import TARGET_WIRE

    max_position = 0.0
    max_angle = 0.0
    for t in np.linspace(0, path.duration / settings.cycles, samples):
        geometry.check(path.sample(float(t))[0])
        target = geometry.origin + [
            0,
            settings.amplitude_m
            * float(smooth_level(t / settings.time_scale * settings.hz)),
            0,
        ]
        position = float(np.linalg.norm(geometry.data.site("wire_tip").xpos - target))
        wire = geometry.data.site("wire_tip").xmat.reshape(3, 3)[:, 2]
        angle = math.degrees(math.acos(np.clip(wire @ TARGET_WIRE, -1.0, 1.0)))
        if position > 0.0006:
            raise ValueError("Interpolated tool path exceeds 0.6 mm task tolerance")
        if angle > 0.2:
            raise ValueError("Interpolated wire direction exceeds 0.2 degree tolerance")
        max_position = max(max_position, position)
        max_angle = max(max_angle, angle)
    return dict(
        max_tip_error_mm=max_position * 1000,
        max_wire_angle_error_deg=max_angle,
        samples=samples,
    )


def validate_search_solution(document, geometry, settings, constraints):
    from simulation.arm_calibration import scene_fingerprint
    from simulation.paths import MODEL_DIR

    if (
        document.get("schema") != "forge.weld_search_solution"
        or document.get("version") != 1
    ):
        raise ValueError("Unsupported search solution")
    expected = dict(
        mapping_fingerprint=geometry.mapping.fingerprint,
        scene_fingerprint=scene_fingerprint(MODEL_DIR / "robot.xml"),
        constraints_fingerprint=digest(constraints),
        amplitude_m=settings.amplitude_m,
    )
    for key, value in expected.items():
        if document.get(key) != value:
            raise ValueError(f"Search solution {key} changed; rerun search")
    candidate_weights(document["candidate"])
    return document["candidate"]


def solve_weave(geometry, settings, root):
    constraints = motor_constraints(
        root, settings.max_speed_deg_s, profile=settings.limit_profile
    )
    deployment = np.array([r["deployment_deg_s"] for r in constraints["joints"]])
    physical = np.array([r["physical_design_deg_s"] for r in constraints["joints"]])
    candidates = []
    solutions = []
    options = [("fixed_D1", False, np.ones(6))]
    if not settings.lock_base_yaw:
        options.extend(
            [
                ("free_D1_equal_utilization", True, np.ones(6)),
                ("free_D1_motor_weighted", True, max(physical) / physical),
                ("free_D1_speed_weighted", True, max(deployment) / deployment),
            ]
        )
    options = [(name, free, weights, None) for name, free, weights in options]
    if settings.search_solution is not None:
        candidate = validate_search_solution(
            settings.search_solution, geometry, settings, constraints
        )
        options = [("cmaes", True, None, candidate)]
    evaluated = {}
    for name, free, weights, candidate in options:
        key = (free, None if weights is None else tuple(weights),
               None if candidate is None else digest(candidate))
        if key in evaluated:
            from copy import deepcopy
            previous, solution = evaluated[key]
            row = deepcopy(previous)
            row.update(name=name, reused_from=previous["name"])
            candidates.append(row)
            if solution is not None:
                duration, final, _, dynamics, factor = solution
                solutions.append((duration, final, name, dynamics, factor))
            continue
        try:
            path = weave_path(
                geometry, settings, free_yaw=free, weights=weights, candidate=candidate
            )
            # Check the complete transverse stroke, including spline midpoints.
            task_report = validate_weave(geometry, path, settings)
            dynamics = dynamics_constraints(
                geometry,
                path.positions[np.linspace(0, 200, 9, dtype=int)],
                constraints,
                acceleration_policy_deg_s2=(
                    None
                    if settings.limit_profile == "weld"
                    else settings.max_accel_deg_s2
                ),
            )
            acceleration = np.asarray(dynamics["raw_acceleration_deg_s2"])
            final, factor = retime(
                path, deployment, acceleration, automatic=settings.auto_retime
            )
            _, physical_factor = retime(
                path, physical, np.asarray(dynamics["raw_model_acceleration_deg_s2"])
            )
            feasibility = trajectory_feasibility(
                *path.peaks(),
                constraints,
                dynamics,
                requested_hz=settings.hz / settings.time_scale,
                feed_m_s=settings.feed_m_s / settings.time_scale,
            )
            candidates.append(
                dict(
                    name=name,
                    feasible=True,
                    time_scale=settings.time_scale * factor,
                    geometric_path_checked=True,
                    task_report=task_report,
                    search_candidate=candidate,
                    physical_design_time_scale=settings.time_scale * physical_factor,
                    native_mode2_feasibility=feasibility,
                    peak_requested_deg_s=path.peaks()[0].tolist(),
                    peak_executable_deg_s=final.peaks()[0].tolist(),
                )
            )
            solutions.append((final.duration, final, name, dynamics, factor))
            evaluated[key] = (candidates[-1], solutions[-1])
        except ValueError as exc:
            candidates.append(dict(name=name, feasible=False, reason=str(exc)))
            evaluated[key] = (candidates[-1], None)
    if not solutions:
        raise ValueError("No feasible constrained solution: " + json.dumps(candidates))
    _, path, name, dynamics, factor = min(
        solutions, key=lambda row: (row[0], max(row[1].peaks()[0] / deployment))
    )
    return path, dict(
        backend="MuJoCo closed-chain kinematics + SciPy bounded least squares and cubic retiming",
        selected=name,
        candidates=candidates,
        constraints=constraints,
        dynamics=dynamics,
        total_time_scale=settings.time_scale * factor,
        automatic_retime=settings.auto_retime,
        guarantee="best of evaluated candidates; not a global optimum",
        physical_design_scope="provisional reduced inertia and derated rated specs; excludes host debug caps; native guard feasibility is reported separately, not executable authorization",
        switch_policy="output gating only; inactive counterpart assumed to follow nominal trajectory",
    )


def speed_report(path, settings, speed_limits=None, accel_limits=None):
    speed, accel = path.peaks()
    speeds = (
        settings.max_speed_deg_s if speed_limits is None else np.asarray(speed_limits)
    )
    accelerations = (
        settings.max_accel_deg_s2 if accel_limits is None else np.asarray(accel_limits)
    )
    required = max(
        1.0,
        float(np.max(speed / speeds)),
        math.sqrt(float(np.max(accel / accelerations))),
    )
    return dict(
        peak_raw_speed_deg_s=speed.tolist(),
        peak_raw_accel_deg_s2=accel.tolist(),
        additional_time_scale=required,
        within_limits=required <= 1.0 + 1e-6,
    )


def raised_approach_path(initial, lower, settings):
    """Construct a fixed-height candidate; callers must validate its geometry."""
    first = move_path(initial, lower[0], settings)
    times = np.linspace(0, APPROACH_LOWERING_SECONDS, len(lower))
    path = JointPath(
        np.r_[first.times, first.duration + 0.2, first.duration + 0.2 + times[1:]],
        np.vstack([first.positions, lower[0], lower[1:]]),
    )
    factor = speed_report(path, settings)["additional_time_scale"] * 1.02
    return JointPath(path.times * factor, path.positions)


def approach_path(geometry, initial, weld_start, settings):
    failures = []
    if not settings.approach_lift_m:
        direct = move_path(initial, weld_start, settings)
        try:
            geometry.validate_path(direct)
            return direct, "direct checked joint transition"
        except ValueError as exc:
            failures.append(str(exc))
    # An explicit lift must survive staging optimization and must never silently
    # fall back to the direct joint blend. It changes the route, not the weld pose.
    heights = ((settings.approach_lift_m,) if settings.approach_lift_m
               else (0.03, 0.06, 0.10, 0.15))
    # Deterministic clearance waypoints, not an unbounded search or an ignored
    # collision. Each candidate includes the entire initial-to-raised route.
    for height in heights:
        try:
            times = np.linspace(0, APPROACH_LOWERING_SECONDS, 81)
            key = (float(height), tuple(float(q) for q in weld_start))
            cache = getattr(geometry, "lowering_cache", {})
            if key not in cache:
                geometry.data.qpos[:] = geometry.seed
                geometry.closure.apply(geometry.data)
                raised = geometry.solve(geometry.origin + [0, 0, height])
                u = times / times[-1]
                s = 10 * u**3 - 15 * u**4 + 6 * u**5
                lower = [
                    geometry.solve(geometry.origin + [0, 0, height * (1 - float(v))])
                    for v in s
                ]
                lower[0] = raised
                if max(abs(np.asarray(lower[-1]) - weld_start)) > 0.05:
                    raise ValueError(
                        "Approach reached a different inverse-kinematics branch"
                    )
                lower[-1] = list(weld_start)
                cache[key] = np.asarray(lower).copy()
            lower = cache[key]
            path = raised_approach_path(initial, lower, settings)
            geometry.validate_path(path)
            return (
                path,
                f"checked raised-tool approach, {height*1000:g} mm above weld tip",
            )
        except ValueError as exc:
            failures.append(f"{height*1000:g} mm: {exc}")
    # A searched center pose can occupy another redundant branch. Lift from
    # that exact pose and reverse the checked lift, rather than snapping to the
    # old D1=0 branch at the stage boundary.
    if settings.search_solution is not None:
        weights = candidate_weights(settings.search_solution["candidate"])
        for height in heights:
            try:
                geometry.check(weld_start)
                anchor = geometry.data.qpos[geometry.addresses].copy()
                times = np.linspace(0, APPROACH_LOWERING_SECONDS, 81)
                u = times / times[-1]
                fractions = 10 * u**3 - 15 * u**4 + 6 * u**5
                lift = [list(weld_start)]
                for fraction in fractions[1:]:
                    lift.append(
                        geometry.solve(
                            geometry.origin + [0, 0, height * float(fraction)],
                            free_yaw=True,
                            preference=anchor,
                            weights=weights,
                        )
                    )
                raised = lift[-1]
                first = move_path(initial, raised, settings)
                geometry.validate_path(first)
                t = np.r_[
                    first.times, first.duration + 0.2, first.duration + 0.2 + times[1:]
                ]
                q = np.vstack(
                    [
                        first.positions,
                        np.asarray(raised)[None, :],
                        np.asarray(lift[-2::-1]),
                    ]
                )
                path = JointPath(t, q)
                factor = speed_report(path, settings)["additional_time_scale"] * 1.02
                path = JointPath(path.times * factor, path.positions)
                geometry.validate_path(path)
                return (
                    path,
                    f"checked branch-following raised-tool approach, {height*1000:g} mm above weld tip",
                )
            except ValueError as exc:
                failures.append(f"branch {height*1000:g} mm: {exc}")
    raise ValueError("No checked approach route: " + "; ".join(failures))


def build_plan(request):
    """Run in a separate process. A result never enables or starts a motor."""
    started = phase_started = time.perf_counter()
    timings = {}
    def phase(name):
        nonlocal phase_started
        now = time.perf_counter()
        timings[name] = now - phase_started
        phase_started = now
    settings = Settings(**request.get("settings", {}))
    from simulation.arm_calibration import scene_fingerprint
    from simulation.paths import MODEL_DIR
    from .preparation import preparation_timing

    fingerprint = scene_fingerprint(MODEL_DIR / "robot.xml")
    if request.get("scene_fingerprint", fingerprint) != fingerprint:
        raise ValueError(
            "Model changed since application startup; reopen with the reviewed model"
        )
    mapping = CalibrationMap(request["mapping"])
    current = finite_vector(request["current_raw_deg"], "current")
    initial = finite_vector(request["initial_raw_deg"], "initial")
    hardware_start = request.get("measured_hardware_start", False)
    if type(hardware_start) is not bool:
        raise ValueError("Measured hardware start switch must be boolean")
    if hardware_start:
        from .calibration import check_a1_recovery_branch

        check_a1_recovery_branch(mapping, current, initial)
    phase("input_and_fingerprints")
    geometry = WeldGeometry(mapping, scene_identity=fingerprint,
                            enforce_spatial_clearance=not hardware_start)
    phase("model_initialization")
    from .endpoint_recovery import prepare_retreat, A3Retreat
    from .measured_start import MeasuredStart, RecoveryGeometry
    start = MeasuredStart(tuple(current), mapping.fingerprint) if hardware_start else None
    start_warnings = geometry.observe(current) if hardware_start else []
    recovery_geometry = geometry if start is None else RecoveryGeometry(geometry, start)
    try:
        endpoint = prepare_retreat(geometry, current, settings)
    except ValueError:
        if start is None:
            raise
        # Retain the reviewed A3 retreat when applicable. Otherwise let the
        # recovery path decide feasibility from the actual measured pose.
        endpoint = None
    recovered = current if endpoint is None else A3Retreat.from_document(endpoint['policy']).goal
    geometry.check(initial)
    home, recovery = recovery_path(recovery_geometry, recovered, initial, settings)
    phase("requested_recovery")
    root = Path(__file__).resolve().parents[4]
    from .planning_cache import cached_weave
    weld, optimization, weave_reused = cached_weave(geometry, settings, root, fingerprint)
    phase("weave_solve")
    weld_limits = np.array(
        [row["deployment_deg_s"] for row in optimization["constraints"]["joints"]]
    )
    weld_acceleration = np.array(optimization["dynamics"]["raw_acceleration_deg_s2"])
    report = speed_report(weld, settings, weld_limits, weld_acceleration)
    approach, route = approach_path(geometry, initial, weld.positions[0], settings)
    phase("requested_approach")
    # Only one cycle needs geometric checking; all cycles use the same body-relative path.
    one = JointPath(weld.times[:201], weld.positions[:201], weld.interpolation)
    geometry.validate_path(one)
    transition_dynamics = dynamics_constraints(
        recovery_geometry,
        np.vstack(
            [
                home.positions[np.linspace(0, len(home.positions) - 1, 5, dtype=int)],
                approach.positions[
                    np.linspace(0, len(approach.positions) - 1, 5, dtype=int)
                ],
            ]
        ),
        optimization["constraints"],
        acceleration_policy_deg_s2=settings.max_accel_deg_s2,
    )
    limits = np.minimum(weld_limits, settings.max_speed_deg_s)
    acceleration = np.array(transition_dynamics["raw_acceleration_deg_s2"])
    original_home_duration = home.duration
    # Reserve acceleration for feedback pacing of the preparation path clock.
    # Stage limits remain the physical/model envelope, not the smaller reserve.
    home, _ = retime(home, limits, 0.9 * acceleration)
    recovery["waypoint_times_s"] = [
        t * home.duration / original_home_duration for t in recovery["waypoint_times_s"]
    ]
    approach, _ = retime(approach, limits, 0.9 * acceleration)
    for name, path in (("initial", home), ("approach", approach)):
        if not speed_report(path, settings)["within_limits"]:
            raise ValueError(f"{name} transition exceeds the speed/acceleration policy")
    plan = dict(
        schema="forge.weld_motion_plan",
        version=1,
        request_fingerprint=digest(request),
        mapping_fingerprint=mapping.fingerprint,
        mapping=mapping.document,
        scene_fingerprint=fingerprint,
        spatial_clearance_policy="advisory" if hardware_start else "enforced",
        current_raw_deg=recovered.tolist(),
        initial_raw_deg=initial.tolist(),
        home=home.document(),
        recovery=recovery,
        approach=approach.document(),
        weld=weld.document(),
        speed_report=report,
        effective_hz=settings.hz / optimization["total_time_scale"],
        feed_m_s=settings.feed_m_s / optimization["total_time_scale"],
        duration_s=weld.duration,
        max_speed_deg_s=settings.max_speed_deg_s,
        max_accel_deg_s2=settings.max_accel_deg_s2,
        limit_profile=settings.limit_profile,
        raw_speed_limits_deg_s=weld_limits.tolist(),
        stage_limits={
            "home": dict(
                speed_deg_s=limits.tolist(), acceleration_deg_s2=acceleration.tolist()
            ),
            "approach": dict(
                speed_deg_s=limits.tolist(), acceleration_deg_s2=acceleration.tolist()
            ),
            "weld": dict(
                speed_deg_s=weld_limits.tolist(),
                acceleration_deg_s2=weld_acceleration.tolist(),
            ),
        },
        optimization=optimization,
        transition_dynamics=transition_dynamics,
        arm_enabled=bool(request["arm_enabled"]),
        chassis_enabled=bool(request["chassis_enabled"]),
        approach_route=route,
        preparation_timing=preparation_timing(
            home,
            approach,
            recovery["waypoint_times_s"],
            limits,
            acceleration,
            settings.preparation_target_s,
        ),
        hardware_qualified=False,
        world_tracking="nominal counterpart assumed even when its output is OFF; no switch compensation",
    )
    if start is not None:
        plan["measured_start"] = start.document()
        plan["start_pose_warnings"] = start_warnings
    phase("limits_and_timing")

    if settings.optimize_preparation:
        from .preparation_search import optimize_preparation

        plan = optimize_preparation(plan, geometry, settings, recovery_geometry=recovery_geometry)
    phase("preparation_search")
    if settings.lowering_min_s:
        from .preparation_search import apply_lowering_minimum

        plan = apply_lowering_minimum(plan, settings)
    if endpoint is not None:
        from .endpoint_recovery import retreat_path, add_retreat_timing
        policy = A3Retreat.from_document(endpoint['policy'])
        endpoint_path = JointPath.from_document(endpoint['path'])
        endpoint['dynamics'] = dynamics_constraints(
            geometry, endpoint_path.positions[np.linspace(0, len(endpoint_path.positions)-1, 5, dtype=int)],
            optimization['constraints'], acceleration_policy_deg_s2=endpoint['acceleration_deg_s2'],
            pose_check=lambda raw: geometry.check_endpoint(raw, endpoint['policy']))
        endpoint['acceleration_deg_s2'] = min(endpoint['dynamics']['raw_acceleration_deg_s2'])
        endpoint['speed_deg_s'] = min(endpoint['speed_deg_s'], min(limits))
        endpoint['path'] = retreat_path(policy, speed=endpoint['speed_deg_s'],
                                       acceleration=endpoint['acceleration_deg_s2']).document()
        plan['endpoint_recovery'] = endpoint
        plan['current_raw_deg'] = current.tolist()
        add_retreat_timing(plan, endpoint)
    if scene_fingerprint(MODEL_DIR / "robot.xml") != fingerprint:
        raise ValueError("Model changed while planning; no path authorized")
    geometry.check(plan["approach"]["raw_deg"][-1])
    tip = geometry.data.site("wire_tip").xpos.copy()
    plan["weld_start_check"] = dict(
        target_m=geometry.origin.tolist(),
        tip_m=tip.tolist(),
        error_mm=float(np.linalg.norm(tip - geometry.origin) * 1000),
        floor_gap_mm=float(
            (tip[2] - geometry.data.geom("preview_floor").xpos[2]) * 1000
        ),
    )
    phase("final_checks")
    if request.get("weld_response_profile") is not None:
        from .weld_response import WeldResponse

        if not plan["arm_enabled"] or plan["chassis_enabled"]:
            raise ValueError("The provisional weld response requires arm output alone")
        response = WeldResponse(request["weld_response_profile"],
                                JointPath.from_document(plan["weld"]), plan)
        plan["weld_response"] = response.check_geometry(geometry.check)
        plan["weld_entry"] = response.entry.document()
        phase("weld_response_check")
    plan["planning_timing_s"] = dict(timings, total=time.perf_counter() - started)
    plan["planning_reuse"] = dict(weave=weave_reused, recovery=False, approach=False)
    return plan


def audit(request):
    """Report original initial-pose conflicts and speed needs without hiding either."""
    mapping = CalibrationMap(request["mapping"])
    settings = Settings(**request.get("settings", {}))
    result = dict(initial_raw_deg=request["initial_raw_deg"], initial_issues=[])
    for i, q in enumerate(request["initial_raw_deg"], 1):
        try:
            mapping.model(i, q)
        except ValueError as exc:
            result["initial_issues"].append(str(exc))
    geometry = WeldGeometry(mapping)
    try:
        geometry.check(request["initial_raw_deg"])
    except ValueError as exc:
        if str(exc) not in result["initial_issues"]:
            result["initial_issues"].append(str(exc))
    try:
        path, optimization = solve_weave(
            geometry, settings, Path(__file__).resolve().parents[4]
        )
        result["weld_start_raw_deg"] = path.positions[0].tolist()
        result["speed_report"] = speed_report(
            path,
            settings,
            [r["deployment_deg_s"] for r in optimization["constraints"]["joints"]],
            optimization["dynamics"]["raw_acceleration_deg_s2"],
        )
        result["optimization"] = optimization
        result["effective_hz"] = settings.hz / optimization["total_time_scale"]
        result["effective_feed_mm_s"] = (
            settings.feed_m_s * 1000 / optimization["total_time_scale"]
        )
    except ValueError as exc:
        result["weld_issue"] = str(exc)
    result["hardware_qualified"] = False
    return result
