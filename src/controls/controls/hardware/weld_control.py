"""Explicit initial -> approach -> ready -> weld state machine; no implicit owners."""

from dataclasses import dataclass
from collections import deque
from copy import deepcopy
import math
import time
from pathlib import Path

import numpy as np

from controls.controller.weld_plan import (
    JointPath,
    finite_vector,
    positive,
    speed_report,
    Settings,
)
from controls.controller.limits import motor_constraints
from motors.arm_travel import a4_cable_bounds, check_a4
from controls.controller.recovery import RECOVERY_CLEARANCE_MM, START_TOLERANCE_DEG
from controls.controller.preparation import PreparationClock


class TrajectoryPaused(RuntimeError):
    """The existing owner completed a measured hold after intent cancellation."""


@dataclass
class Feedback:
    """Six physical slots in order; angles/rates in degrees, cached ages in ms."""

    raw_deg: np.ndarray
    velocity_deg_s: np.ndarray
    age_ms: np.ndarray

    def validate(self, stationary=False):
        self.raw_deg = finite_vector(self.raw_deg, "feedback position")
        self.velocity_deg_s = finite_vector(self.velocity_deg_s, "feedback velocity")
        self.age_ms = finite_vector(self.age_ms, "feedback age")
        if np.any(self.age_ms < 0) or np.any(self.age_ms >= 80):
            raise RuntimeError("Arm feedback is missing or stale")
        if stationary and max(abs(self.velocity_deg_s)) > math.degrees(0.03):
            raise ValueError("Wait for all six joints to become stationary")
        return self


class WeldController:
    """Single-thread execution over raw output-shaft degree trajectories.

    The owner thread calls ``tick`` at the configured period. Adapters convert
    to bus SI units; the GUI and planner never access transports. Arrival latches
    a fixed endpoint hold, while faults and interrupted travel retain explicit
    stop/recovery semantics. Model and fake-drive checks are not load acceptance.
    """

    ACTIVE = ("verifying_hold", "initial", "approach", "welding")

    def __init__(
        self, arm=None, chassis=None, *, clock=time.monotonic, pose_check=None
    ):
        self.arm, self.chassis, self.clock = arm, chassis, clock
        self.pose_check = pose_check
        self.a4_bounds = a4_cable_bounds()
        self.arm_on = False
        self.chassis_on = False
        self.arm_holding = False
        self.state = "idle"
        self.message = "Select outputs, calculate a checked path, then start recovery."
        self.plan = None
        self.paths = {}
        self.started = None
        self.settled = None
        self.last_tick = None
        self.last_feedback = None
        self.events = []
        self.fault = None
        self.last_accepted_plan = None
        self.trace = deque(maxlen=30000)
        self.trace_dropped = 0
        self.tracking_deg = 3.0
        self.arrival_deg = 0.25
        self.max_gap_s = 0.10
        self.recovery_stops = []
        self.recovery_index = 0
        self.preparation = {}
        self.last_observation = None
        self.hold_deviated = False

    def check_pose(self, raw, *, recovery=False):
        check_a4(float(raw[3]), self.a4_bounds)
        if self.pose_check is None:
            raise ValueError(
                "A model clearance guard is required before arm activation"
            )
        self.pose_check(raw, clearance_mm=RECOVERY_CLEARANCE_MM if recovery else 0.5)

    def joint_goal(self):
        """Current stage endpoint for display; never a new motor command."""
        if not self.arm_on or not self.paths:
            return None
        if self.state == "verifying_hold":
            raw, label = self.paths["home"].positions[0], "Hold preload"
        elif self.state in ("planned", "initial"):
            index = self.recovery_index if self.state == "initial" else 0
            raw = self.paths["home"].sample(self.recovery_stops[index])[0]
            label = "Recovery waypoint"
        elif self.state in ("approach", "ready"):
            raw, label = self.paths["approach"].positions[-1], "Weld start"
        elif self.state in ("welding", "complete"):
            raw, label = self.paths["weld"].positions[-1], "Weld end"
        else:
            return None
        return dict(label=label, raw_deg=raw.tolist())

    def observe_endpoint_hold(self, now):
        """Record fresh cached feedback against the fixed goal after arrival.

        Native poll continues refreshing the same endpoint. Observation must
        never recapture a sagged pose as a new target or issue gain/torque writes.
        """
        goal = self.joint_goal()
        actual = self.feedback()
        error = np.asarray(goal["raw_deg"]) - actual.raw_deg
        self.last_observation = dict(
            time_s=now,
            stage=self.state,
            target_raw_deg=goal["raw_deg"],
            measured_raw_deg=actual.raw_deg.tolist(),
            measured_velocity_deg_s=actual.velocity_deg_s.tolist(),
            tracking_error_deg=error.tolist(),
            age_ms=actual.age_ms.tolist(),
            arrival_tolerance_deg=self.arrival_deg,
        )
        deviated = bool(np.max(abs(error)) > self.arrival_deg)
        if deviated != self.hold_deviated:
            self.hold_deviated = deviated
            self.event(
                "hold_deviation" if deviated else "hold_recovered",
                target_raw_deg=goal["raw_deg"],
                error_deg=error.tolist(),
                arrival_tolerance_deg=self.arrival_deg,
            )

    def event(self, kind, **data):
        self.events.append(
            dict(event=kind, time_s=self.clock(), state=self.state, **data)
        )

    def feedback(self, stationary=False):
        if self.arm is None:
            raise ValueError("No arm owner is connected")
        read = (
            getattr(self.arm, "read_stationary", self.arm.read)
            if stationary and not self.arm_holding
            else self.arm.read
        )
        self.last_feedback = read().validate(stationary)
        return self.last_feedback

    def select(self, arm, chassis):
        if self.fault:
            raise RuntimeError("Fault is latched; resolve it and explicitly reconnect")
        if type(arm) is not bool or type(chassis) is not bool:
            raise ValueError("Participation switches must be booleans")
        if arm and self.arm is None or chassis and self.chassis is None:
            raise ValueError("Connect the selected hardware owner first")
        self.pause()
        if self.fault:
            raise RuntimeError("Stop failed; no component may be enabled")
        if arm and not self.arm_holding:
            self.feedback(stationary=True)
        self.arm_on, self.chassis_on = arm, chassis
        self.state = "idle"
        self.plan = None
        self.event("selection", arm=arm, chassis=chassis)
        self.message = "Selection does not enable motors. Calculate a fresh path; an existing hold is retained."

    def load_plan(self, plan):
        if self.fault or self.state in self.ACTIVE:
            raise ValueError("Pause before accepting a plan")
        if plan.get("schema") != "forge.weld_motion_plan" or plan.get("version") != 1:
            raise ValueError("Unsupported weld plan")
        if plan.get("offline_only") and any(
            enabled and not getattr(port, "offline", False)
            for enabled, port in (
                (self.arm_on, self.arm),
                (self.chassis_on, self.chassis),
            )
        ):
            raise ValueError("Experimental preparation plan is offline-only")
        plan = deepcopy(plan)
        if (plan["arm_enabled"], plan["chassis_enabled"]) != (
            self.arm_on,
            self.chassis_on,
        ):
            raise ValueError("Participation changed during planning")
        if not (self.arm_on or self.chassis_on):
            raise ValueError("Select at least one component")
        feed = plan["feed_m_s"]
        if not math.isfinite(feed) or not 0 <= feed <= 0.012:
            raise ValueError("Invalid chassis feed")
        paths = {}
        preparation = {}
        if self.arm_on:
            settings = Settings(
                max_speed_deg_s=plan["max_speed_deg_s"],
                max_accel_deg_s2=plan["max_accel_deg_s2"],
                limit_profile=plan.get("limit_profile", "debug"),
            )
            hard = motor_constraints(
                Path(__file__).resolve().parents[4],
                settings.max_speed_deg_s,
                profile=settings.limit_profile,
            )
            hard = np.array([r["deployment_deg_s"] for r in hard["joints"]])
            paths = {
                n: JointPath.from_document(plan[n])
                for n in ("home", "approach", "weld")
            }
            for name, path in paths.items():
                cable_bounds = [(-math.inf, math.inf)] * 6
                cable_bounds[3] = self.a4_bounds
                path.check_raw_bounds(cable_bounds)
                envelope = plan.get("stage_limits", {}).get(name, {})
                ceilings = finite_vector(
                    envelope.get(
                        "speed_deg_s",
                        plan.get(
                            "raw_speed_limits_deg_s", [settings.max_speed_deg_s] * 6
                        ),
                    ),
                    "axis speed limits",
                )
                acceleration = finite_vector(
                    envelope.get(
                        "acceleration_deg_s2", [settings.max_accel_deg_s2] * 6
                    ),
                    "axis acceleration limits",
                )
                cap = (
                    hard
                    if name == "weld"
                    else np.minimum(hard, settings.max_speed_deg_s)
                )
                if (
                    np.any(ceilings <= 0)
                    or np.any(ceilings > cap + 1e-9)
                    or np.any(acceleration <= 0)
                ):
                    raise ValueError("Invalid per-axis speed limits")
                acceleration_cap = (
                    finite_vector(
                        plan["optimization"]["dynamics"]["raw_acceleration_deg_s2"],
                        "model acceleration limits",
                    )
                    if name == "weld" and settings.limit_profile == "weld"
                    else settings.max_accel_deg_s2
                )
                if np.any(acceleration > np.asarray(acceleration_cap) + 1e-9):
                    raise ValueError(
                        "Initial/approach acceleration exceeds debug policy"
                    )
                if not speed_report(path, settings, ceilings, acceleration)[
                    "within_limits"
                ]:
                    raise ValueError(
                        "Path exceeds motor-specific speed or acceleration limits"
                    )
                if np.any(path.peaks()[0] > ceilings + 1e-6):
                    raise ValueError("Path exceeds motor-specific speed limits")
                if name in ("home", "approach"):
                    preparation[name] = PreparationClock(path, acceleration)
            for a, b in (("home", "approach"), ("approach", "weld")):
                if max(abs(paths[a].positions[-1] - paths[b].positions[0])) > 1e-6:
                    raise ValueError("Discontinuous stage boundary")
            recovery = plan.get("recovery", {})
            if (
                recovery.get("version") != 1
                or recovery.get("clearance_mm") != RECOVERY_CLEARANCE_MM
            ):
                raise ValueError(
                    "Recalculate with the checked startup/recovery planner"
                )
            stops = np.asarray(recovery.get("waypoint_times_s", []), dtype=float)
            home = paths["home"]
            if (
                stops.ndim != 1
                or not 1 <= len(stops) <= 6
                or not np.all(np.isfinite(stops))
                or stops[0] <= 0
                or np.any(np.diff(stops) <= 0)
                or abs(stops[-1] - home.duration) > 1e-8
            ):
                raise ValueError("Invalid recovery waypoint times")
            for t in stops:
                if (
                    np.min(abs(home.times - t)) > 1e-8
                    or max(abs(home.sample(float(t))[1])) > 0.001
                ):
                    raise ValueError(
                        "Recovery waypoints must be stationary trajectory knots"
                    )
            if (
                max(
                    abs(
                        home.positions[0]
                        - finite_vector(plan["current_raw_deg"], "planned start")
                    )
                )
                > 1e-8
            ):
                raise ValueError("Recovery does not start at the checked measured pose")
            if (
                max(
                    abs(
                        home.positions[-1]
                        - finite_vector(plan["initial_raw_deg"], "planned standby")
                    )
                )
                > 1e-8
            ):
                raise ValueError("Recovery does not end at standby")
            if (
                max(abs(home.positions[0] - self.feedback(stationary=True).raw_deg))
                > START_TOLERANCE_DEG
            ):
                raise ValueError("Arm moved since calculation; calculate again")
            self.recovery_stops = stops.tolist()
        else:
            positive(plan["duration_s"], "duration")
        self.plan = plan
        self.paths = paths
        self.preparation = preparation
        self.state = "planned"
        self.last_accepted_plan = plan
        self.hold_deviated = False
        next_step = (
            "Start recovery to verify hold, reach standby, then approach weld start."
        )
        self.message = "Plan ready. " + next_step
        if self.arm_on:
            order = plan["recovery"].get("joint_order")
            route = (
                " -> ".join(f"A{i}" for i in order)
                if order
                else "direct coordinated move"
            )
            self.message = (
                f"Plan ready: recovery {route}; {len(self.recovery_stops)} waypoint(s). "
                + next_step
            )
        self.event("plan_loaded")

    def go_initial(self):
        if self.fault or self.state != "planned" or self.plan is None:
            raise ValueError("Calculate and inspect a fresh plan first")
        if self.arm_on:
            actual = self.feedback(stationary=True).raw_deg
            if max(abs(actual - self.paths["home"].positions[0])) > START_TOLERANCE_DEG:
                raise ValueError("Start position changed; calculate again")
            self.check_pose(actual, recovery=True)
            if not self.arm_holding:
                try:
                    self.arm.hold(
                        self.paths["home"].positions[0],
                        lambda raw: self.check_pose(raw, recovery=True),
                    )
                    self.arm_holding = True
                except Exception as exc:
                    self.fail(exc)
                    return
            self.recovery_index = 0
            for clock in self.preparation.values():
                clock.reset()
            self.state = "verifying_hold"
        else:
            self.state = "ready"
        self.started = self.clock()
        self.last_tick = self.started
        self.settled = None
        self.event("recovery_requested")
        self.message = (
            "Verifying stationary current-position hold before recovery motion."
            if self.arm_on
            else "Chassis-only pass ready; arm is not commanded."
        )

    def start_weld(self):
        if self.fault or self.state != "ready":
            raise ValueError("Reach the weld start before starting welding motion")
        if self.arm_on:
            if (
                max(
                    abs(
                        self.feedback(stationary=True).raw_deg
                        - self.paths["weld"].positions[0]
                    )
                )
                > self.arrival_deg
            ):
                raise ValueError("Arm left the weld start; calculate a new approach")
        self.state = "welding"
        self.started = self.clock()
        self.last_tick = self.started
        self.settled = None
        self.event("weld_started")
        self.message = (
            "Welding motion active. This application does not energize a welder."
        )

    def pause(self, reason=None):
        # Attempt both actions even when one transport fails.
        errors = []
        if self.chassis is not None:
            try:
                self.chassis.stop()
            except Exception as exc:
                errors.append(str(exc))
        if self.arm_holding:
            try:
                self.arm.pause()
            except Exception as exc:
                errors.append(str(exc))
        self.plan = None
        self.paths = {}
        self.started = None
        self.settled = None
        if self.state != "fault":
            self.state = "paused"
        self.message = (
            "Paused: chassis zero, "
            + ("arm holding. " if self.arm_holding else "arm not enabled. ")
            + "Recalculate before restarting."
        )
        if reason:
            self.message = reason
        self.event("pause", reason=reason)
        if errors:
            self.fail(RuntimeError("; ".join(errors)))

    def disable_arm(self):
        self.pause()
        try:
            if self.arm is not None:
                self.arm.disable()
        finally:
            self.arm_on = False
            self.arm_holding = False
        self.event("arm_disabled")

    def fail(self, error):
        primary = self.fault is None
        if primary:
            self.fault = str(error)
        self.state = "fault"
        self.plan = None
        self.paths = {}
        cleanup = []
        for port in (self.chassis, self.arm):
            if port is not None:
                try:
                    port.stop() if port is self.chassis else port.disable()
                except Exception as exc:
                    cleanup.append(str(exc))
        self.arm_holding = False
        self.arm_on = False
        self.chassis_on = False
        self.message = "FAULT: " + self.fault
        self.event(
            "fault" if primary else "secondary_fault",
            error=str(error),
            primary_fault=self.fault,
            cleanup_errors=cleanup,
        )

    def tick(self):
        if self.fault:
            return
        now = self.clock()
        try:
            dt = 0.0 if self.last_tick is None else now - self.last_tick
            if self.state in self.ACTIVE and self.last_tick is not None:
                if not 0 <= dt <= self.max_gap_s:
                    self.pause(
                        "Control timing gap: paused without catching up missed motion."
                    )
                    self.last_tick = now
                    return
            self.last_tick = now
            if self.arm_holding:
                self.arm.poll()
            if self.chassis is not None:
                self.chassis.check()
                # Once commanded, the native ENCOS owner also leases zero targets.
                # Refresh only zero while idle/paused; never a previous feed value.
                if self.state != "welding":
                    self.chassis.stop()
            if self.state not in self.ACTIVE:
                if (
                    self.arm_on
                    and self.arm_holding
                    and self.state in ("ready", "complete")
                ):
                    self.observe_endpoint_hold(now)
                return
            elapsed = now - self.started
            if not self.arm_on:
                if self.state == "welding" and elapsed < self.plan["duration_s"]:
                    if self.chassis_on:
                        self.chassis.velocity(self.plan["feed_m_s"])
                else:
                    if self.chassis is not None:
                        self.chassis.stop()
                    self.state = "complete"
                    self.event("completed")
                return
            if self.state == "verifying_hold":
                actual = self.feedback()
                self.check_pose(actual.raw_deg, recovery=True)
                if (
                    max(abs(actual.raw_deg - self.paths["home"].positions[0]))
                    > START_TOLERANCE_DEG
                ):
                    self.pause(
                        "Pose changed during hold activation; inspect and recalculate recovery."
                    )
                    return
                stationary = max(abs(actual.velocity_deg_s)) < math.degrees(0.03)
                self.settled = (
                    (now if self.settled is None else self.settled)
                    if stationary
                    else None
                )
                if self.settled is not None and now - self.settled >= 0.2:
                    self.state = "initial"
                    self.started, self.settled = now, None
                    self.event("hold_verified", raw_deg=actual.raw_deg.tolist())
                    self.message = "Hold verified; following checked recovery waypoints to standby."
                elif elapsed > 2.0:
                    self.pause(
                        "Hold did not settle; no recovery motion started. Recalculate."
                    )
                return
            key = {"initial": "home", "approach": "approach", "welding": "weld"}[
                self.state
            ]
            path = self.paths[key]
            offset = (
                self.recovery_stops[self.recovery_index - 1]
                if key == "home" and self.recovery_index
                else 0
            )
            end = (
                self.recovery_stops[self.recovery_index]
                if key == "home"
                else path.duration
            )
            duration = end - offset
            actual = self.feedback()
            self.last_observation = dict(
                time_s=now,
                stage=self.state,
                measured_raw_deg=actual.raw_deg.tolist(),
                measured_velocity_deg_s=actual.velocity_deg_s.tolist(),
                age_ms=actual.age_ms.tolist(),
            )
            self.check_pose(actual.raw_deg, recovery=key == "home")
            preparation = self.preparation.get(key)
            if preparation is not None:
                desired, rate = preparation.advance(dt, actual.raw_deg, offset, end)
                path_elapsed = preparation.elapsed
            else:
                path_elapsed = min(elapsed, duration)
                desired, rate = path.sample(path_elapsed)
            error = desired - actual.raw_deg
            if max(abs(error)) > self.tracking_deg:
                axis = int(np.argmax(abs(error)))
                self.pause(
                    f"Tracking error A{axis+1}: {error[axis]:+.3f} deg; paused both components; inspect and replan."
                )
                return
            envelope = self.plan.get("stage_limits", {}).get(key, {})
            ceiling = np.broadcast_to(
                np.asarray(
                    envelope.get(
                        "speed_deg_s",
                        self.plan.get(
                            "raw_speed_limits_deg_s", self.plan["max_speed_deg_s"]
                        ),
                    ),
                    dtype=float,
                ),
                (6,),
            )
            # Native mode 2 owns position feedback. Keep a steady preparation
            # speed allowance instead of collapsing it near every tiny target.
            # The checked, feedback-paced position path still defines the move.
            speed = (
                np.minimum(ceiling, self.plan["max_speed_deg_s"])
                if preparation is not None
                else np.minimum(ceiling, np.maximum(0.1, abs(rate) + 2 * abs(error)))
            )
            timeout = (
                preparation.timeout_reason(
                    now, actual.raw_deg, desired, at_end=path_elapsed >= duration
                )
                if preparation is not None
                else (
                    "Arrival timeout; no next stage started."
                    if elapsed > duration + 10
                    else None
                )
            )
            if timeout:
                self.event(
                    (
                        "preparation_incomplete"
                        if preparation is not None
                        else "arrival_timeout"
                    ),
                    path_elapsed_s=path_elapsed,
                    path_duration_s=duration,
                    measured_raw_deg=actual.raw_deg.tolist(),
                    target_raw_deg=path.sample(end)[0].tolist(),
                    reason=timeout,
                )
                self.pause(timeout)
                return
            if len(self.trace) == self.trace.maxlen:
                self.trace_dropped += 1
            self.trace.append(
                dict(
                    time_s=now,
                    stage=self.state,
                    elapsed_s=elapsed,
                    path_elapsed_s=path_elapsed,
                    path_clock_scale=1.0 if preparation is None else preparation.scale,
                    tracking_error_deg=error.tolist(),
                    requested_raw_deg=desired.tolist(),
                    measured_raw_deg=actual.raw_deg.tolist(),
                    requested_rate_deg_s=rate.tolist(),
                    measured_velocity_deg_s=actual.velocity_deg_s.tolist(),
                    speed_limit_deg_s=speed.tolist(),
                    age_ms=actual.age_ms.tolist(),
                    dispatch_state="requested",
                )
            )
            try:
                self.arm.command(desired, speed)
                self.trace[-1]["dispatch_state"] = "sent"
            except Exception:
                self.trace[-1]["dispatch_state"] = "incomplete"
                raise
            if self.state == "welding" and elapsed < path.duration and self.chassis_on:
                self.chassis.velocity(self.plan["feed_m_s"])
            elif self.chassis is not None:
                self.chassis.stop()
            if path_elapsed >= duration:
                stationary = max(abs(actual.velocity_deg_s)) < math.degrees(0.03)
                arrived = max(abs(error)) <= self.arrival_deg and stationary
                self.settled = (
                    (now if self.settled is None else self.settled) if arrived else None
                )
                if self.settled is not None and now - self.settled >= 0.2:
                    if self.state == "initial":
                        self.event(
                            "recovery_waypoint_arrived",
                            waypoint=self.recovery_index + 1,
                        )
                        self.recovery_index += 1
                        self.started = now
                        self.settled = None
                        preparation.reset()
                        if self.recovery_index == len(self.recovery_stops):
                            self.state = "approach"
                            self.message = "Standby reached; moving to weld start."
                        else:
                            self.message = "Recovery waypoint settled; continuing the checked route."
                    elif self.state == "approach":
                        self.arm.finish_trajectory()
                        self.state = "ready"
                        self.message = "Weld start reached. Press Start welding motion."
                    else:
                        self.arm.finish_trajectory()
                        self.state = "complete"
                        self.message = "Pass complete; chassis stopped, arm holding."
                    self.event(
                        "stage_arrived",
                        from_stage=key,
                        target_raw_deg=desired.tolist(),
                        measured_raw_deg=actual.raw_deg.tolist(),
                        error_deg=error.tolist(),
                        speed_limit_deg_s=speed.tolist(),
                        arrival_tolerance_deg=self.arrival_deg,
                    )
        except TrajectoryPaused as exc:
            self.pause(str(exc))
        except Exception as exc:
            self.fail(exc)

    def close(self):
        errors = []
        self.pause()
        for port in (self.chassis, self.arm):
            if port is not None:
                try:
                    port.close()
                except Exception as exc:
                    errors.append(str(exc))
        self.arm_holding = False
        if errors:
            self.fail(RuntimeError("; ".join(errors)))


def chassis_only_plan(settings):
    return dict(
        schema="forge.weld_motion_plan",
        version=1,
        arm_enabled=False,
        chassis_enabled=True,
        duration_s=settings.cycles / settings.hz * settings.time_scale,
        feed_m_s=settings.feed_m_s / settings.time_scale,
        effective_hz=settings.hz / settings.time_scale,
        hardware_qualified=False,
    )
