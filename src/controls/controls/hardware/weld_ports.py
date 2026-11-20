"""Single-owner motor adapters for supported weld-motion commissioning."""

import math
import time
from contextlib import contextmanager

import numpy as np

from motors.arm_group_hold import ArmGroupHoldPort
from motors import arm_position as position
from motors.arm_position import MotionCancelled
from motors.arm_calibration import CalibrationMap
from .weld_control import Feedback, TrajectoryPaused
from controls.controller.weld_plan import finite_vector
from controls.controller.limits import joint_speed_policy
from controls.controller.recovery import START_TOLERANCE_DEG
from controls.controller.endpoint_recovery import A3Retreat, ALLOWANCE_DEG, SPEED_DEG_S


def check_hold_start(actual, expected, pose_check):
    actual = finite_vector(actual, "hold preload")
    expected = finite_vector(expected, "checked recovery start")
    if max(abs(actual - expected)) > START_TOLERANCE_DEG:
        raise ValueError(
            "Arm moved before hold activation; calculate a fresh recovery path"
        )
    pose_check(actual)


class TrajectoryArmPort(ArmGroupHoldPort):
    """Stream validated complete targets through existing mode-2 group holding.

    The controller already supplies time-indexed smooth targets. Use native mode 2
    (USB type 4), without the custom type-25 manual-jog slew and its extra lag.
    USB commands remain sequential: this is not firmware-atomic multi-axis motion.
    A failure uses the existing latched fault/whole-group disable policy.
    """

    def __init__(self, *args, mapping, max_speed_deg_s=5.0, **kwargs):
        self.mapping = CalibrationMap(mapping)
        plan = args[0] if args else kwargs["plan"]
        # Keep 20% headroom below the actual configured measured-speed stops.
        native = np.array(
            [0.8 * math.degrees(row["speed_trip_rad_s"]) for row in plan["motors"]]
        )
        if native.shape != (6,) or not np.isfinite(native).all() or np.any(native <= 0):
            raise ValueError("Six finite positive configured speed guards required")
        if not 0 < max_speed_deg_s <= min(max(native), math.degrees(1.2)) + 1e-9:
            raise ValueError(
                "Trajectory command ceiling exceeds native commissioning guards"
            )
        policy = joint_speed_policy()["max_speed_deg_s"]
        self.axis_ceilings = np.minimum(native, max_speed_deg_s)
        for i in range(6):
            cap = policy[f"A{i+1}"]
            if cap is not None:
                self.axis_ceilings[i] = min(self.axis_ceilings[i], cap)
        self.path_speeds = np.radians(np.minimum(self.axis_ceilings, 5.0))
        self.path_active = False
        self.endpoint_policy = None
        self.endpoint_last = None
        self.endpoint_minimum = None
        self.endpoint_hold = False
        self._cycle = None
        self.path_ceiling = max_speed_deg_s
        super().__init__(*args, window_degrees=None, smooth_jog=False, **kwargs)
        self.report.update(
            target_policy="controller_trajectory_native_mode2",
            usb_command_type=4,
            secondary_jog_smoothing=False,
            trajectory_speed_ceilings_deg_s=self.axis_ceilings.tolist(),
        )
        if tuple(self.available_joints) != tuple(range(1, 7)):
            self.close()
            raise ValueError("Welding motion requires all six arm joints")

    def _refresh_targets(self):
        if self.endpoint_policy is not None:
            raw = np.degrees([r.position_rad for r in sorted(self.cached_positions(), key=lambda r: r.joint)])
            self.endpoint_policy.check_raw(raw)
            for joint, q in enumerate(raw, 1):
                if joint != 3:
                    self.mapping.model(joint, float(q))
            self.endpoint_minimum = min(self.endpoint_minimum, float(raw[2]))
            if raw[2] > self.endpoint_minimum + ALLOWANCE_DEG:
                raise ValueError("A3 moved outward during endpoint recovery/hold")
        for i, target in self.hold_targets.items():
            if self.path_active:
                self._check_intent()
            self.bus.command(
                i, target, velocity=float(self.path_speeds[i]), smooth=False
            )

    @contextmanager
    def _guard_motion(self):
        # Match the panel's pause semantics even if focus/intent changes halfway
        # through a six-axis dispatch. A partly enabled group cannot use this
        # exception: it follows the base owner's disable-on-cancel contract.
        try:
            yield
        except TrajectoryPaused:
            raise  # A nested dispatch guard already completed the hold.
        except MotionCancelled as exc:
            if self.holding and self.hold_targets and not self.recovery_required:
                self.pause_trajectory()
                raise TrajectoryPaused(
                    "Operator intent interrupted; existing arm hold retained; recalculate."
                ) from exc
            self.stop()
            raise
        except Exception as exc:
            raise RuntimeError(self.capture_failure(exc)) from exc
        except BaseException:
            self.stop()
            raise

    def set_targets(self, raw_deg, speed_deg_s):
        raw = finite_vector(raw_deg, "targets")
        speed = finite_vector(speed_deg_s, "speeds")
        if not self.holding or not self.hold_targets:
            raise ValueError("Explicit group hold is required")
        if np.any(speed <= 0) or np.any(speed > self.axis_ceilings + 1e-9):
            raise ValueError("Trajectory speed exceeds supported limit")
        if self.endpoint_policy is None:
            self.mapping.pose({str(i): float(q) for i, q in enumerate(raw, 1)})
        else:
            self.endpoint_policy.check_raw(raw, command=True)
            if raw[2] > self.endpoint_last + 1e-9 or np.any(speed > SPEED_DEG_S + 1e-9):
                raise ValueError("A3 recovery commands must remain inward and at most 1 deg/s")
            for i, q in enumerate(raw, 1):
                if i != 3:
                    self.mapping.model(i, float(q))
        targets = np.radians(raw)
        for i, target in enumerate(targets):
            lo, hi = self._limits(i)
            if not lo < target < hi:
                raise ValueError(f"A{i+1}: target exceeds configured protocol bounds")
        # Commit only after validating every axis. Physical dispatch is sequential.
        with self._guard_motion():
            if self._cycle is None:
                self.begin_cycle()
            states, started, feedback_done = self._cycle
            self._cycle = None  # A feedback batch may authorize only one dispatch.
            if any(row.age_ms >= 80 for row in self.cached_positions()):
                raise RuntimeError("Arm feedback became stale before trajectory dispatch")
            self._check_intent()
            if self.endpoint_policy is not None:
                self.endpoint_last = float(raw[2])
            self.hold_targets = dict(enumerate(targets))
            self.path_speeds = np.radians(speed)
            self.path_active = True
            self.endpoint_hold = False
            commands_started = position.time.monotonic()
            self._refresh_targets()
            done = position.time.monotonic()
            self._control_timing = dict(
                owner_poll_ms=(done - started) * 1000,
                feedback_batch_ms=(feedback_done - started) * 1000,
                calculation_ms=(commands_started - feedback_done) * 1000,
                command_batch_ms=(done - commands_started) * 1000,
            )
            for i, sample in states.items():
                self._record_motion_sample(i, sample, hold_target=self.hold_targets[i])

    def begin_cycle(self):
        """Read once before planning this tick; dispatch follows in set_targets.

        Idle holding still uses poll(), which reads and refreshes fixed targets.
        Splitting a moving cycle avoids sending the previous target and then
        reading every joint again before dispatching the new target.
        """
        self._cycle = None
        self._check()
        if not self.holding or not self.hold_targets:
            raise ValueError("Explicit group hold is required")
        with self._guard_motion():
            started = position.time.monotonic()
            states = {i: self._enabled_sample(i) for i in self.slots}
            self._cycle = states, started, position.time.monotonic()

    def poll(self):
        self._cycle = None
        return super().poll()

    def stop(self):
        self._cycle = None
        self.endpoint_hold = False
        self.path_active = False
        super().stop()
        self.endpoint_policy = None

    def configure_endpoint(self, document):
        if document is None:
            if self.endpoint_policy is not None and self.hold_targets:
                feedback = ArmOwner._feedback(self.cached_positions()).validate(stationary=True)
                self.mapping.pose(dict(zip('123456', map(float, feedback.raw_deg))))
                self.mapping.pose({str(i+1): math.degrees(q) for i, q in self.hold_targets.items()})
            self.endpoint_policy = None
            return
        policy = A3Retreat.from_document(document)
        policy.check_mapping(self.mapping)
        minimum = (policy.start[2] if self.endpoint_policy is None
                   else min(self.endpoint_minimum, policy.start[2]))
        self.endpoint_policy = policy
        self.endpoint_last = policy.start[2]
        self.endpoint_minimum = minimum
        self.path_speeds = np.radians(np.minimum(self.axis_ceilings, SPEED_DEG_S))

    def finish_trajectory(self):
        """Latch the already-dispatched endpoint after measured settling.

        Keep mode, position and speed allowance unchanged. The native owner
        continues command refresh and protection checks without motion intent.
        """
        if not self.holding or not self.hold_targets or not self.path_active:
            raise ValueError("No active trajectory to finish")
        with self._guard_motion():
            self._check_intent()
            self.endpoint_hold = True
            self.path_active = False
            self.moving = False
            self.armed = False
            self._event(
                "weld_endpoint_hold",
                targets_rad=dict(self.hold_targets),
                speed_allowance_rad_s=self.path_speeds.tolist(),
            )

    def pause_trajectory(self):
        self._cycle = None
        if not self.hold_targets:
            return
        with self._guard_motion():
            if self.path_active:
                samples = {i: self._enabled_sample(i) for i in self.slots}
                if self.endpoint_policy is not None:
                    self.endpoint_policy.check_raw(np.degrees([samples[i].position for i in range(6)]))
                self.hold_targets = {i: s.position for i, s in samples.items()}
            self.path_active = False
            self.moving = False
            self.holding = True
            self.armed = False
            self._refresh_targets()
            self._event(
                "weld_pause_hold",
                targets_rad=dict(self.hold_targets),
                speed_allowance_rad_s=self.path_speeds.tolist(),
                endpoint_preserved=self.endpoint_hold,
            )


class ArmOwner:
    def __init__(self, plan, mapping, *, port, max_speed_deg_s, motion_allowed):
        self.port = TrajectoryArmPort(
            plan,
            port=port,
            mapping=mapping,
            max_speed_deg_s=max_speed_deg_s,
            motion_allowed=motion_allowed,
            allow_faulted_start=True,
        )
        self.ceiling = max_speed_deg_s

    @property
    def fault(self):
        return self.port.report.get("error") if self.port.recovery_required else None

    def recover(self, report_path):
        self.port.recover(report_path)

    def read(self):
        rows = (
            self.port.cached_positions() if self.port.holding else self.port.positions()
        )
        return self._feedback(rows)

    @staticmethod
    def _feedback(rows):
        rows = sorted(rows, key=lambda row: row.joint)
        if len(rows) != 6 or [r.joint for r in rows] != list(range(1, 7)):
            raise RuntimeError("Missing arm feedback slots")
        return Feedback(
            np.degrees([r.position_rad for r in rows]),
            np.degrees([r.velocity_rad_s for r in rows]),
            np.array([r.age_ms for r in rows], float),
            [dict(joint=r.joint, age_ms=r.age_ms,
                  **{key: getattr(r, key, None) for key in
                     ("enabled", "torque_feedback_nm", "torque_trip_nm", "rotor_c", "mos_c")}) for r in rows],
        )

    def display_feedback(self):
        """Copy the owner's aged samples; display must not request bus traffic."""
        return self._feedback(self.port.cached_positions())

    def read_stationary(self):
        """Explicit startup action: refresh disabled feedback without enabling."""
        if not self.port.holding:
            self.port.stationary_position()
        return self.read()

    def hold(self, expected, pose_check):
        def validate(states):
            check_hold_start(
                np.degrees([states[i].position for i in range(6)]), expected, pose_check
            )

        self.port.path_active = False
        self.port.endpoint_hold = False
        cap = SPEED_DEG_S if self.port.endpoint_policy is not None else 5.0
        self.port.path_speeds = np.radians(np.minimum(self.port.axis_ceilings, cap))
        self.port.hold_all(
            math.radians(min(min(self.port.axis_ceilings), cap)),
            validate_preload=validate,
        )

    def configure_endpoint(self, document):
        self.port.configure_endpoint(document)

    def poll(self):
        self.port.poll()

    def begin_cycle(self):
        self.port.begin_cycle()

    def command(self, raw, speed):
        self.port.set_targets(raw, speed)

    def finish_trajectory(self):
        self.port.finish_trajectory()

    def pause(self):
        self.port.pause_trajectory()

    def disable(self):
        self.port.path_active = False
        self.port.endpoint_hold = False
        self.port.stop()

    def close(self):
        self.port.close()


class SimulatedArm:
    """Rate-limited fake shafts for UI/state-machine tests, never a dynamics claim."""

    offline = True

    def __init__(self, raw, *, clock=time.monotonic):
        self.clock = clock
        self.q = finite_vector(raw, "simulated initial").copy()
        self.target = self.q.copy()
        self.speed = np.zeros(6)
        self.velocity = np.zeros(6)
        self.last = clock()
        self.enabled = False
        self.endpoint_hold = False
        self.path_active = False
        self.closed = False
        self.commands = []
        self.endpoint_policy = None
        self.endpoint_last = None

    def configure_endpoint(self, document):
        self.endpoint_policy = None if document is None else A3Retreat.from_document(document)
        self.endpoint_last = None if document is None else self.endpoint_policy.start[2]

    def poll(self):
        now = self.clock()
        dt = max(0.0, now - self.last)
        self.last = now
        step = (
            np.clip(self.target - self.q, -self.speed * dt, self.speed * dt)
            if self.enabled
            else np.zeros(6)
        )
        self.q += step
        if dt > 0:
            self.velocity = step / dt

    def read(self):
        self.poll()
        return Feedback(self.q.copy(), self.velocity.copy(), np.zeros(6))

    def display_feedback(self):
        return self.read()

    def hold(self, expected, pose_check):
        self.poll()
        check_hold_start(self.q, expected, pose_check)
        self.enabled = True
        self.endpoint_hold = False
        self.path_active = False
        self.target = self.q.copy()
        self.speed = np.zeros(6)

    def command(self, raw, speed):
        if not self.enabled:
            raise RuntimeError("Simulated arm disabled")
        if self.endpoint_policy is not None:
            self.endpoint_policy.check_raw(raw, command=True)
            if raw[2] > self.endpoint_last + 1e-9 or np.max(speed) > SPEED_DEG_S + 1e-9:
                raise ValueError("A3 recovery command must stay inward at <=1 deg/s")
            self.endpoint_last = float(raw[2])
        self.target = finite_vector(raw, "target").copy()
        self.speed = finite_vector(speed, "speed").copy()
        self.endpoint_hold = False
        self.path_active = True
        self.commands.append(self.target.tolist())
        if len(self.commands) > 1000:
            del self.commands[:500]

    def finish_trajectory(self):
        if not self.enabled:
            raise RuntimeError("Simulated arm disabled")
        self.endpoint_hold = True
        self.path_active = False

    def pause(self):
        self.poll()
        if self.path_active:
            self.target = self.q.copy()
        self.path_active = False
        self.velocity[:] = 0

    def disable(self):
        self.pause()
        self.enabled = False
        self.endpoint_hold = False
        self.endpoint_policy = None

    def recover(self, report_path):
        if self.enabled:
            raise RuntimeError("Disable before recovery")
        self.path_active = self.endpoint_hold = False
        self.target = self.q.copy()
        self.speed[:] = 0
        self.velocity[:] = 0

    def close(self):
        self.disable()
        self.closed = True


class ChassisOwner:
    def __init__(self, config, *, simulate, execute, mapping_confirmed):
        from motors import MotorBus, MotorConfig
        from motor_control.differential import DifferentialDrive

        self.offline = bool(simulate)
        p = config["motor_driver"]["ros__parameters"]
        if p.get("bench_omit_motor_id", 0):
            raise ValueError(
                "Weld feed requires the complete four-wheel profile; omitted motor 4 is not accepted"
            )
        if not simulate and not mapping_confirmed:
            raise ValueError(
                "Hardware chassis requires explicit wheel mapping confirmation"
            )
        keys = (
            "motor_ids",
            "wheel_motor_ids",
            "wheel_sides",
            "wheel_directions",
            "external_ratios",
            "wheel_rpm_scales",
            "wheel_radius_m",
            "track_width_m",
            "max_velocity_rpm",
        )
        self.drive = DifferentialDrive(
            **{
                key: tuple(p[key]) if isinstance(p[key], list) else p[key]
                for key in keys
                if key in p
            }
        )
        # No silent ratio-preserving speed clamp in a time-coordinated pass.
        nominal = 0.012 * 60 / (2 * math.pi * self.drive.wheel_radius_m)
        required = max(
            nominal * abs(d) * r * s
            for d, r, s in zip(
                self.drive.wheel_directions,
                self.drive.external_ratios,
                self.drive.wheel_rpm_scales,
            )
        )
        if required > self.drive.max_velocity_rpm + 1e-9:
            raise ValueError(
                "Chassis speed ceiling would clip weld feed; revise the reviewed profile"
            )
        self.bus = MotorBus(
            [
                MotorConfig(
                    i,
                    min_position_deg=-720.0,
                    max_position_deg=720.0,
                    max_velocity_rpm=p["max_velocity_rpm"],
                    max_current_a=p["max_current_a"],
                )
                for i in self.drive.motor_ids
            ],
            interface=p.get("interface"),
            simulate=simulate,
            execute=execute,
            motor_slots=p.get("motor_slots"),
        )
        try:
            self.bus.wait_ready()
        except BaseException:
            self.bus.close()
            raise
        self.last_feed = 0.0
        self.simulated = simulate

    def velocity(self, feed):
        if not math.isfinite(feed) or not 0 <= feed <= 0.012:
            raise ValueError("Invalid weld feed")
        for motor, rpm in zip(self.drive.motor_ids, self.drive.mix(feed, 0)):
            self.bus.velocity(motor, rpm)
        self.last_feed = feed

    def check(self):
        self.bus.check()

    def stop(self):
        self.bus.stop()
        self.last_feed = 0.0

    def close(self):
        self.bus.close()
