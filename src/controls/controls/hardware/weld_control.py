"""Explicit initial -> approach -> ready -> weld state machine; no implicit owners."""

from dataclasses import dataclass
from collections import deque
import math
import time
from pathlib import Path

import numpy as np

from controls.controller.weld_plan import JointPath, finite_vector, positive, speed_report, Settings
from controls.controller.limits import motor_constraints


@dataclass
class Feedback:
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
    """Ports are called by one control thread. GUI/planning must never call them."""

    ACTIVE = ("initial", "approach", "welding")

    def __init__(self, arm=None, chassis=None, *, clock=time.monotonic):
        self.arm, self.chassis, self.clock = arm, chassis, clock
        self.arm_on = False
        self.chassis_on = False
        self.arm_holding = False
        self.state = "idle"
        self.message = "Select arm/chassis, calculate, then move to initial pose."
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

    def event(self, kind, **data):
        self.events.append(dict(event=kind, time_s=self.clock(), state=self.state, **data))

    def feedback(self, stationary=False):
        if self.arm is None:
            raise ValueError("No arm owner is connected")
        self.last_feedback = self.arm.read().validate(stationary)
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
            self.arm.hold()
            self.arm_holding = True
        self.arm_on, self.chassis_on = arm, chassis
        self.state = "idle"
        self.plan = None
        self.event("selection", arm=arm, chassis=chassis)
        self.message = (
            "Selection changed; calculate a fresh path. Arm OFF retains an existing powered hold."
        )

    def load_plan(self, plan):
        if self.fault or self.state in self.ACTIVE:
            raise ValueError("Pause before accepting a plan")
        if plan.get("schema") != "forge.weld_motion_plan" or plan.get("version") != 1:
            raise ValueError("Unsupported weld plan")
        if (plan["arm_enabled"], plan["chassis_enabled"]) != (self.arm_on, self.chassis_on):
            raise ValueError("Participation changed during planning")
        if not (self.arm_on or self.chassis_on):
            raise ValueError("Select at least one component")
        feed = plan["feed_m_s"]
        if not math.isfinite(feed) or not 0 <= feed <= 0.012:
            raise ValueError("Invalid chassis feed")
        paths = {}
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
            paths = {n: JointPath.from_document(plan[n]) for n in ("home", "approach", "weld")}
            for name, path in paths.items():
                envelope = plan.get("stage_limits", {}).get(name, {})
                ceilings = finite_vector(
                    envelope.get(
                        "speed_deg_s",
                        plan.get("raw_speed_limits_deg_s", [settings.max_speed_deg_s] * 6),
                    ),
                    "axis speed limits",
                )
                acceleration = finite_vector(
                    envelope.get("acceleration_deg_s2", [settings.max_accel_deg_s2] * 6),
                    "axis acceleration limits",
                )
                cap = hard if name == "weld" else np.minimum(hard, settings.max_speed_deg_s)
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
                    raise ValueError("Initial/approach acceleration exceeds debug policy")
                if not speed_report(path, settings, ceilings, acceleration)["within_limits"]:
                    raise ValueError("Path exceeds motor-specific speed or acceleration limits")
                if np.any(path.peaks()[0] > ceilings + 1e-6):
                    raise ValueError("Path exceeds motor-specific speed limits")
            for a, b in (("home", "approach"), ("approach", "weld")):
                if max(abs(paths[a].positions[-1] - paths[b].positions[0])) > 1e-6:
                    raise ValueError("Discontinuous stage boundary")
            if max(abs(paths["home"].positions[0] - self.feedback(stationary=True).raw_deg)) > 0.3:
                raise ValueError("Arm moved since calculation; calculate again")
        else:
            positive(plan["duration_s"], "duration")
        self.plan = plan
        self.paths = paths
        self.state = "planned"
        self.last_accepted_plan = plan
        self.message = "Plan ready. Move to recorded/model-mapped initial, then weld start."
        self.event("plan_loaded")

    def go_initial(self):
        if self.fault or self.state != "planned" or self.plan is None:
            raise ValueError("Calculate and inspect a fresh plan first")
        if self.arm_on:
            if not self.arm_holding:
                raise ValueError("Arm must hold before moving")
            if (
                max(abs(self.feedback(stationary=True).raw_deg - self.paths["home"].positions[0]))
                > 0.3
            ):
                raise ValueError("Start position changed; calculate again")
            self.state = "initial"
        else:
            self.state = "ready"
        self.started = self.clock()
        self.last_tick = self.started
        self.settled = None
        self.event("initial_started")
        self.message = (
            "Moving to initial pose; then approaching weld start."
            if self.arm_on
            else "Chassis-only pass ready; arm is not commanded."
        )

    def start_weld(self):
        if self.fault or self.state != "ready":
            raise ValueError("Reach the weld start before starting welding motion")
        if self.arm_on:
            if (
                max(abs(self.feedback(stationary=True).raw_deg - self.paths["weld"].positions[0]))
                > self.arrival_deg
            ):
                raise ValueError("Arm left the weld start; calculate a new approach")
        self.state = "welding"
        self.started = self.clock()
        self.last_tick = self.started
        self.settled = None
        self.event("weld_started")
        self.message = "Welding motion active. This application does not energize a welder."

    def pause(self):
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
        self.message = "Paused: chassis zero, arm holding. Recalculate before restarting."
        self.event("pause")
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
        self.event("fault", error=self.fault, cleanup_errors=cleanup)

    def tick(self):
        if self.fault:
            return
        now = self.clock()
        try:
            if self.state in self.ACTIVE and self.last_tick is not None:
                if not 0 <= now - self.last_tick <= self.max_gap_s:
                    self.pause()
                    self.message = "Control timing gap: paused without catching up missed motion."
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
            key = {"initial": "home", "approach": "approach", "welding": "weld"}[self.state]
            path = self.paths[key]
            desired, rate = path.sample(elapsed)
            actual = self.feedback()
            error = desired - actual.raw_deg
            if max(abs(error)) > self.tracking_deg:
                self.pause()
                self.message = "Tracking error: paused both components; inspect and replan."
                return
            envelope = self.plan.get("stage_limits", {}).get(key, {})
            speed = np.minimum(
                envelope.get(
                    "speed_deg_s",
                    self.plan.get("raw_speed_limits_deg_s", self.plan["max_speed_deg_s"]),
                ),
                np.maximum(0.1, abs(rate) + 2 * abs(error)),
            )
            if len(self.trace) == self.trace.maxlen:
                self.trace_dropped += 1
            self.trace.append(
                dict(
                    time_s=now,
                    stage=self.state,
                    elapsed_s=elapsed,
                    requested_raw_deg=desired.tolist(),
                    measured_raw_deg=actual.raw_deg.tolist(),
                    requested_rate_deg_s=rate.tolist(),
                    measured_velocity_deg_s=actual.velocity_deg_s.tolist(),
                    speed_limit_deg_s=speed.tolist(),
                    age_ms=actual.age_ms.tolist(),
                )
            )
            self.arm.command(desired, speed)
            if self.state == "welding" and elapsed < path.duration and self.chassis_on:
                self.chassis.velocity(self.plan["feed_m_s"])
            elif self.chassis is not None:
                self.chassis.stop()
            if elapsed >= path.duration:
                stationary = max(abs(actual.velocity_deg_s)) < math.degrees(0.03)
                arrived = max(abs(error)) <= self.arrival_deg and stationary
                self.settled = (now if self.settled is None else self.settled) if arrived else None
                if elapsed > path.duration + 10:
                    self.pause()
                    self.message = "Arrival timeout; no next stage started."
                    return
                if self.settled is not None and now - self.settled >= 0.2:
                    if self.state == "initial":
                        self.state = "approach"
                        self.started = now
                        self.settled = None
                        self.message = "Initial pose reached; moving to weld start."
                    elif self.state == "approach":
                        self.state = "ready"
                        self.message = "Weld start reached. Press Start welding motion."
                    else:
                        self.state = "complete"
                        self.message = "Pass complete; chassis stopped, arm holding."
                    self.event("stage_arrived")
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
