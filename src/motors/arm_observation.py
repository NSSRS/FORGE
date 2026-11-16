"""Six-axis observation with optional, explicitly selected disabled feedback refresh.

No enable, target, fault reset, zero, mode write or motor parameter write exists
in this adapter. Disabled refresh sends disable frames to obtain new feedback.
"""

import math
import time

from .arm import ArmBus
from .arm_bench import BenchSession, active_joints, preflight, validate_plan


class ArmObservation:
    def __init__(self, plan, *, port=None, refresh_disabled=False, supported=False, bus=None):
        self.motors = validate_plan(plan, all_motors=True)
        if active_joints(plan) != tuple(range(1, 7)):
            raise ValueError("The mirror requires all six physical slots A1-A6")
        if refresh_disabled and not supported:
            raise ValueError("Disabled refresh requires --supported; it sends torque-disable frames")
        self.refresh_disabled = refresh_disabled
        self.bus = bus if bus is not None else ArmBus(port)
        self.evidence = dict(motor_parameter_writes=False, motor_targets_sent=False,
                             disable_refresh=refresh_disabled, identity_verified=False)
        try:
            self.evidence["base_info"] = self.bus.base_info()
            if self.evidence["base_info"]["api_version"] != 1:
                raise ValueError("Unsupported cached telemetry API")
            if refresh_disabled:
                # Preflight rejects any enabled/faulted slot before sending disable.
                observed = dict(enumerate(preflight(self.bus, self.motors)))
                session = BenchSession(self.bus, self.motors, observed, self.evidence)
                session.probe_disabled()
                self.evidence.update(identity_verified=True, registers=observed)
        except BaseException:
            self.bus.close()
            raise

    @staticmethod
    def require_disabled(snapshot):
        if snapshot["fault"]:
            raise RuntimeError("Controller fault is latched; mirror will not reset it")
        if any(row["enabled"] or row["status"] != 0 for row in snapshot["motors"]):
            raise RuntimeError("Mirror requires six already-disabled motors; no takeover is permitted")
        if not all(row["configured"] for row in snapshot["motors"]):
            raise RuntimeError("Missing configured motor slot; no partial arm pose")

    def sample(self):
        snapshot = self.bus.snapshot()
        if self.refresh_disabled:
            self.require_disabled(snapshot)
            self.bus.stop()
            deadline = time.monotonic() + 0.4
            while True:
                snapshot = self.bus.snapshot()
                self.require_disabled(snapshot)
                if all(row["feedback_valid"] and not row["stopping"] and
                       row["age_ms"] + snapshot["acquisition_duration_ms"] < 80
                       for row in snapshot["motors"]):
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("No fresh disabled feedback from all six motors")
                time.sleep(0.005)
        rows = []
        for row, motor in zip(snapshot["motors"], self.motors):
            valid = row["feedback_valid"] and row["configured"]
            if valid and not all(math.isfinite(row[key]) for key in ("position", "velocity", "torque")):
                raise ValueError("Nonfinite encoder feedback")
            rows.append(dict(joint=row["slot"] + 1,
                bus=motor["bus"] if self.refresh_disabled else None,
                can_id=motor["can_id"] if self.refresh_disabled else None,
                feedback_id=motor["feedback_id"] if self.refresh_disabled else None,
                raw_position_rad=row["position"] if valid else None,
                raw_position_deg=math.degrees(row["position"]) if valid else None,
                velocity_rad_s=row["velocity"] if valid else None,
                age_ms=row["age_ms"] + snapshot["acquisition_duration_ms"] if valid else None,
                enabled=row["enabled"], status=row["status"], feedback_valid=valid))
        return dict(feedback=rows, host_received_monotonic_s=snapshot["host_received_monotonic_s"],
                    generation=snapshot["generation"], fault=snapshot["fault"],
                    source="disabled-refresh" if self.refresh_disabled else "passive-cache")

    def close(self):
        self.bus.close()
