"""Passive ROS telemetry for Damiao raw shafts and ENCOS output shafts.

No controller ownership or motion operations live here. ENCOS ages describe
host PDO observations; the bridge does not expose proven CAN receive times.
"""

import math
import time


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def age_seconds(now, observed):
    if not finite(observed) or not finite(now) or observed > now:
        return None
    return now - observed


def fresh(value, age):
    return finite(value) and age is not None and 0 <= age < 0.1


def status(name, level, message, values):
    return dict(name=name, level=level, message=message, values=values)


def damiao_frame(sample, names, now):
    """Normalize a complete snapshot, retaining invalid/stale slots in diagnostics."""
    rows = sample["motors"]
    if len(names) != 6 or len(rows) != 6 or [r["slot"] for r in rows] != list(range(6)):
        raise ValueError("Damiao snapshot must contain all six ordered slots")
    elapsed = age_seconds(now, sample["host_received_monotonic_s"])
    acquisition = sample["acquisition_duration_ms"]
    valid_time = elapsed is not None and finite(acquisition) and acquisition >= 0
    ages, good, diagnostics = [], [], []
    for name, row in zip(names, rows):
        raw_age = row["age_ms"]
        age = (
            (raw_age + acquisition) / 1000 + elapsed
            if valid_time and finite(raw_age) and raw_age >= 0
            else None
        )
        valid = (
            row["configured"]
            and row["feedback_valid"]
            and all(fresh(row[key], age) for key in ("position", "velocity", "torque"))
        )
        fault = bool(sample["fault"] or row["status"] not in (0, 1))
        good.append(valid and not fault)
        ages.append(age)
        values = dict(
            row,
            feedback_age_ms=None if age is None else age * 1000,
            fresh=bool(valid),
            generation=sample["generation"],
            capture_ms=sample["at_ms"],
            acquisition_duration_ms=acquisition,
        )
        diagnostics.append(
            status(
                "damiao/" + name,
                2 if fault else 0 if valid else 3,
                (
                    "Motor fault"
                    if fault
                    else "Fresh cached feedback" if valid else "Missing, invalid or stale feedback"
                ),
                values,
            )
        )
    complete = all(good)
    joint = (
        dict(
            names=list(names),
            position=[r["position"] for r in rows],
            velocity=[r["velocity"] for r in rows],
            effort=[r["torque"] for r in rows],
            age_s=max(ages),
            frame_id="arm_motor_shafts",
        )
        if complete
        else None
    )
    return joint, diagnostics


def encos_frame(sample, motor_ids, now):
    rows = sample["motors"]
    if sample["api_version"] != 1 or [r["motor_id"] for r in rows] != list(motor_ids):
        raise ValueError("ENCOS snapshot identity/order mismatch")
    captured_age = age_seconds(now, sample["captured_monotonic_s"])
    field_values = {"position": [], "velocity": []}
    field_ages = {"position": [], "velocity": []}
    diagnostics = []
    for row in rows:
        ages = {
            name: age_seconds(now, row[name + "_observed_s"])
            for name in ("position", "velocity", "electrical")
        }
        values = dict(
            row,
            timestamp_basis=sample["timestamp_basis"],
            can_freshness_verified=sample["can_freshness_verified"],
        )
        for name, age in ages.items():
            values[name + "_observation_age_ms"] = None if age is None else age * 1000
        for field, raw, factor in (
            ("position", "position_deg", math.pi / 180),
            ("velocity", "velocity_rpm", math.tau / 60),
        ):
            valid = captured_age is not None and captured_age < 0.1 and fresh(row[raw], ages[field])
            values[field + "_valid"] = valid
            field_values[field].append(row[raw] * factor if valid else None)
            field_ages[field].append(ages[field])
        values["electrical_valid"] = (
            captured_age is not None
            and captured_age < 0.1
            and all(fresh(row[k], ages["electrical"]) for k in ("current_a", "temperature_c"))
        )
        fault = bool(sample["fault"] or row["error"])
        available = values["position_valid"] or values["velocity_valid"]
        diagnostics.append(
            status(
                f"encos/motor_{row['motor_id']}",
                2 if fault else 3 if not available else 0 if sample["simulated"] else 1,
                (
                    "Motor/bridge fault"
                    if fault
                    else (
                        "Missing or stale PDO observation"
                        if not available
                        else (
                            "Simulated feedback"
                            if sample["simulated"]
                            else "PDO observed; CAN freshness unverified"
                        )
                    )
                ),
                values,
            )
        )
    # JointState permits an empty vector for an unavailable quantity. Never
    # substitute zeros or stamp a stale position fresh after a velocity update.
    selected_ages = []
    for field in field_values:
        if all(v is not None for v in field_values[field]):
            selected_ages.extend(field_ages[field])
        else:
            field_values[field] = []
    usable = selected_ages and not sample["fault"] and not any(r["error"] for r in rows)
    joint = (
        dict(
            names=[f"motor_{i}_output" for i in motor_ids],
            **field_values,
            effort=[],
            age_s=max(selected_ages),
            frame_id="encos_motor_outputs",
        )
        if usable
        else None
    )
    transport = {k: v for k, v in sample.items() if k != "motors"}
    diagnostics.insert(
        0,
        status(
            "encos/bridge",
            2 if sample["fault"] else 0 if sample["simulated"] else 1,
            (
                "Latched bridge fault"
                if sample["fault"]
                else (
                    "Simulated transport"
                    if sample["simulated"]
                    else "CAN receive timing unavailable"
                )
            ),
            transport,
        ),
    )
    return joint, diagnostics


class TelemetryPublisher:
    """Existing JointState topic plus 2 Hz diagnostics, immediate status changes."""

    def __init__(self, node, joint_publisher, hardware_id):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from sensor_msgs.msg import JointState

        self.node, self.joint_publisher, self.hardware_id = node, joint_publisher, hardware_id
        self.Array, self.Status, self.KeyValue, self.JointState = (
            DiagnosticArray,
            DiagnosticStatus,
            KeyValue,
            JointState,
        )
        self.publisher = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.last_diagnostics = -math.inf
        self.last_levels = None

    def publish(self, joint, statuses, *, force=False):
        now = time.monotonic()
        ros_now = self.node.get_clock().now()
        if joint:
            # Conservative oldest included sample time, not publication time.
            stamp_ns = ros_now.nanoseconds - math.ceil(joint["age_s"] * 1e9)
            if stamp_ns >= 0:
                message = self.JointState()
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                message.header.frame_id = joint["frame_id"]
                message.name = joint["names"]
                message.position, message.velocity, message.effort = (
                    joint["position"],
                    joint["velocity"],
                    joint["effort"],
                )
                self.joint_publisher.publish(message)
        levels = tuple((s["name"], s["level"], s["message"]) for s in statuses)
        if not force and now - self.last_diagnostics < 0.5 and levels == self.last_levels:
            return
        message = self.Array()
        message.header.stamp = ros_now.to_msg()  # Diagnostic publication time; ages explicit below.
        for s in statuses:
            entry = self.Status()
            entry.name, entry.level, entry.message = s["name"], bytes([s["level"]]), s["message"]
            entry.hardware_id = self.hardware_id
            for key, value in s["values"].items():
                pair = self.KeyValue()
                pair.key, pair.value = key, (
                    "unavailable"
                    if value is None
                    else str(value).lower() if isinstance(value, bool) else str(value)
                )
                entry.values.append(pair)
            message.status.append(entry)
        self.publisher.publish(message)
        self.last_diagnostics, self.last_levels = now, levels

    def error(self, family, error):
        self.publish(
            None,
            [status(family + "/transport", 2, "Telemetry unavailable", {"error": str(error)})],
            force=True,
        )


class DamiaoTelemetry:
    """Reuse the driver's sole ArmBus; never send commands or poll CAN registers."""

    def __init__(self, bus, publisher, names, buses):
        self.bus, self.publisher, self.names = bus, publisher, names
        self.buses = tuple(sorted(set(buses)))
        self.info = bus.base_info()
        if self.info["api_version"] != 1:
            raise RuntimeError("Unsupported Damiao base telemetry API")
        self.health = {}
        self.health_started = {}
        self.health_at = None
        self.last_generation = None

    def poll(self):
        # Query health first so the motion snapshot is the most recent read.
        if self.health_at is None or time.monotonic() - self.health_at >= 0.5:
            for bus in self.buses:
                self.health_started[bus] = time.monotonic()
                self.health[bus] = self.bus.bus_health(bus)
            self.health_at = time.monotonic()
        sample = self.bus.snapshot()
        generation = sample["generation"]
        if self.last_generation is not None:
            advance = (generation - self.last_generation) & 0xFFFFFFFF
            if not 0 < advance < 0x80000000:
                raise RuntimeError("Repeated or reversed Damiao snapshot generation")
        self.last_generation = generation
        now = time.monotonic()
        joint, statuses = damiao_frame(sample, self.names, now)
        statuses.insert(
            0,
            status(
                "damiao/controller",
                2 if sample["fault"] else 0,
                "Latched controller fault" if sample["fault"] else "Cached controller metadata",
                dict(
                    self.info,
                    fault=sample["fault"],
                    fault_reason=sample["fault_reason"],
                    fault_slot=sample["fault_slot"],
                    generation=sample["generation"],
                    capture_ms=sample["at_ms"],
                    acquisition_duration_ms=sample["acquisition_duration_ms"],
                ),
            ),
        )
        for bus, health in self.health.items():
            flags = health["status_flags"]
            level = (
                2
                if flags & 1
                else 1 if flags & 0x1E or health["tx_failed"] or health["bus_off_events"] else 0
            )
            statuses.append(
                status(
                    f"damiao/can_{bus}",
                    level,
                    (
                        "Bus off"
                        if flags & 1
                        else "Bus errors recorded" if level else "Cached bus counters"
                    ),
                    dict(
                        health,
                        host_health_age_upper_bound_ms=(now - self.health_started[bus]) * 1000,
                    ),
                )
            )
        self.publisher.publish(joint, statuses)
        return sample
