"""Single-owner motor adapters for supported weld-motion commissioning."""

import math
import time

import numpy as np

from motors.arm_group_hold import ArmGroupHoldPort
from motors.arm_calibration import CalibrationMap
from .weld_control import Feedback
from controls.controller.weld_plan import finite_vector
from controls.controller.limits import joint_speed_policy


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
        native = np.array([0.8 * math.degrees(row["speed_trip_rad_s"]) for row in plan["motors"]])
        if native.shape != (6,) or not np.isfinite(native).all() or np.any(native <= 0):
            raise ValueError("Six finite positive configured speed guards required")
        if not 0 < max_speed_deg_s <= min(max(native), math.degrees(1.2)) + 1e-9:
            raise ValueError("Trajectory command ceiling exceeds native commissioning guards")
        policy = joint_speed_policy()["max_speed_deg_s"]
        self.axis_ceilings = np.minimum(native, max_speed_deg_s)
        for i in range(6):
            cap = policy[f"A{i+1}"]
            if cap is not None:
                self.axis_ceilings[i] = min(self.axis_ceilings[i], cap)
        self.path_speeds = np.radians(np.minimum(self.axis_ceilings, 5.0))
        self.path_active = False
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
        for i, target in self.hold_targets.items():
            if self.path_active:
                self._check_intent()
            self.bus.command(i, target, velocity=float(self.path_speeds[i]), smooth=False)

    def set_targets(self, raw_deg, speed_deg_s):
        raw = finite_vector(raw_deg, "targets")
        speed = finite_vector(speed_deg_s, "speeds")
        if not self.holding or not self.hold_targets:
            raise ValueError("Explicit group hold is required")
        if np.any(speed <= 0) or np.any(speed > self.axis_ceilings + 1e-9):
            raise ValueError("Trajectory speed exceeds supported limit")
        self.mapping.pose({str(i): float(q) for i, q in enumerate(raw, 1)})
        targets = np.radians(raw)
        for i, target in enumerate(targets):
            lo, hi = self._limits(i)
            if not lo < target < hi:
                raise ValueError(f"A{i+1}: target exceeds configured protocol bounds")
        # Commit only after validating every axis. Physical dispatch is sequential.
        with self._guard_motion():
            self._check_intent()
            self.hold_targets = dict(enumerate(targets))
            self.path_speeds = np.radians(speed)
            self.path_active = True
            self.poll()

    def pause_trajectory(self):
        if not self.hold_targets:
            return
        with self._guard_motion():
            samples = {i: self._enabled_sample(i) for i in self.slots}
            self.hold_targets = {i: s.position for i, s in samples.items()}
            self.path_active = False
            self.moving = False
            self.holding = True
            self.armed = False
            self.path_speeds = np.minimum(self.path_speeds, math.radians(5.0))
            self._refresh_targets()
            self._event("weld_pause_hold", targets_rad=dict(self.hold_targets))


class ArmOwner:
    def __init__(self, plan, mapping, *, port, max_speed_deg_s, motion_allowed):
        self.port = TrajectoryArmPort(
            plan,
            port=port,
            mapping=mapping,
            max_speed_deg_s=max_speed_deg_s,
            motion_allowed=motion_allowed,
        )
        self.ceiling = max_speed_deg_s

    def read(self):
        rows = self.port.cached_positions() if self.port.holding else self.port.positions()
        rows = sorted(rows, key=lambda row: row.joint)
        if len(rows) != 6 or [r.joint for r in rows] != list(range(1, 7)):
            raise RuntimeError("Missing arm feedback slots")
        return Feedback(
            np.degrees([r.position_rad for r in rows]),
            np.degrees([r.velocity_rad_s for r in rows]),
            np.array([r.age_ms for r in rows], float),
        )

    def hold(self):
        self.port.hold_all(math.radians(min(min(self.port.axis_ceilings), 5.0)))

    def poll(self):
        self.port.poll()

    def command(self, raw, speed):
        self.port.set_targets(raw, speed)

    def pause(self):
        self.port.pause_trajectory()

    def disable(self):
        self.port.path_active = False
        self.port.stop()

    def close(self):
        self.port.close()


class SimulatedArm:
    """Rate-limited fake shafts for UI/state-machine tests, never a dynamics claim."""

    def __init__(self, raw, *, clock=time.monotonic):
        self.clock = clock
        self.q = finite_vector(raw, "simulated initial").copy()
        self.target = self.q.copy()
        self.speed = np.zeros(6)
        self.velocity = np.zeros(6)
        self.last = clock()
        self.enabled = False
        self.closed = False
        self.commands = []

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

    def hold(self):
        self.poll()
        self.enabled = True
        self.target = self.q.copy()
        self.speed = np.zeros(6)

    def command(self, raw, speed):
        if not self.enabled:
            raise RuntimeError("Simulated arm disabled")
        self.target = finite_vector(raw, "target").copy()
        self.speed = finite_vector(speed, "speed").copy()
        self.commands.append(self.target.tolist())
        if len(self.commands) > 1000:
            del self.commands[:500]

    def pause(self):
        self.poll()
        self.target = self.q.copy()
        self.velocity[:] = 0

    def disable(self):
        self.pause()
        self.enabled = False

    def close(self):
        self.disable()
        self.closed = True


class ChassisOwner:
    def __init__(self, config, *, simulate, execute, mapping_confirmed):
        from motors import MotorBus, MotorConfig
        from motor_control.differential import DifferentialDrive

        p = config["motor_driver"]["ros__parameters"]
        if p.get("bench_omit_motor_id", 0):
            raise ValueError(
                "Weld feed requires the complete four-wheel profile; omitted motor 4 is not accepted"
            )
        if not simulate and not mapping_confirmed:
            raise ValueError("Hardware chassis requires explicit wheel mapping confirmation")
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
                self.drive.wheel_directions, self.drive.external_ratios, self.drive.wheel_rpm_scales
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
