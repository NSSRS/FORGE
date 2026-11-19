"""Damiao output-shaft envelopes, distinct from protocol and commissioning caps."""

import json
import math
from pathlib import Path
import numpy as np


MAX_PREPARATION_SPEED_DEG_S = 100.0


def joint_speed_policy(root=None):
    root = Path(root) if root is not None else Path(__file__).resolve().parents[4]
    policy = json.loads((root / "src/controls/config/arm_motion_policy.json").read_text())
    if policy.get("schema") != "forge.arm_motion_policy" or policy.get("version") != 1:
        raise ValueError("Unsupported arm motion policy")
    speeds = policy["max_speed_deg_s"]
    if set(speeds) != {f"A{i}" for i in range(1, 7)}:
        raise ValueError("Motion policy requires all six axes")
    if any(
        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0
        for v in speeds.values()
        if v is not None
    ):
        raise ValueError(
            "Motion policy speeds must be finite and positive, or null for no extra cap"
        )
    return policy


def motor_constraints(
    root, command_ceiling_deg_s=5.0, *, speed_fraction=0.5, torque_fraction=0.5, profile="debug"
):
    if profile not in ("debug", "weld"):
        raise ValueError("Unknown constraint profile")
    if (
        not 0 < command_ceiling_deg_s <= MAX_PREPARATION_SPEED_DEG_S
        or not 0 < speed_fraction <= 1
        or not 0 < torque_fraction <= 1
    ):
        raise ValueError("Invalid commissioning or motor derating policy")
    root = Path(root)
    policy = joint_speed_policy(root)
    specs = json.loads((root / "src/controls/config/damiao_physics.json").read_text())
    rig = json.loads((root / "src/robot_description/config/arm_rig_a1_a6.json").read_text())
    rows = []
    for motor in rig["motors"]:
        spec = specs["models"][motor["model"]]
        values = [
            spec["rated_output_rpm_24v"],
            spec["rated_output_torque_nm"],
            spec["reduction_ratio"],
            motor["ranges"]["vmax"],
            motor["speed_trip_rad_s"],
            motor["torque_trip_nm"],
        ]
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("Motor physical and protocol limits must be finite and positive")
        rated = spec["rated_output_rpm_24v"] * 6.0
        physical = min(rated * speed_fraction, math.degrees(motor["ranges"]["vmax"]))
        # This is the host commissioning policy, not a native mode-2 firmware cap.
        # Welding uses type 4; type 25's independent 10 deg/s jog cap is bypassed.
        joint_cap = policy["max_speed_deg_s"][motor["unit"]]
        native = min(0.8 * math.degrees(motor["speed_trip_rad_s"]), physical)
        if joint_cap is not None:
            native = min(native, joint_cap)
        deployment = min(command_ceiling_deg_s, native) if profile == "debug" else native
        torque = min(
            spec["rated_output_torque_nm"] * torque_fraction, motor["torque_trip_nm"] * 0.8
        )
        rows.append(
            dict(
                joint=motor["unit"],
                model=motor["model"],
                rated_output_deg_s=rated,
                physical_design_deg_s=physical,
                deployment_deg_s=deployment,
                joint_policy_speed_deg_s=joint_cap,
                native_guarded_speed_deg_s=native,
                torque_budget_nm=torque,
                internal_reduction=spec["reduction_ratio"],
                protocol_vmax_rad_s=motor["ranges"]["vmax"],
                reactive_trip_rad_s=motor["speed_trip_rad_s"],
                source=spec["source"],
                manual_page=7,
            )
        )
    if [r["joint"] for r in rows] != [f"A{i}" for i in range(1, 7)]:
        raise ValueError("Six ordered motors required")
    return dict(
        joints=rows,
        speed_fraction=speed_fraction,
        torque_fraction=torque_fraction,
        limit_profile=profile,
        joint_motion_policy=policy,
        command_transport="native position-velocity mode 2 via USB type 4",
        secondary_jog_smoothing=False,
        host_commissioning_ceiling_deg_s=command_ceiling_deg_s,
        voltage_basis="24 V specification, conservative across listed variants; not a measured supply voltage",
        policy="Rated continuous output, not peak torque or no-load speed. Fractions are design margins.",
        dynamics_qualified=False,
        hardware_qualified=False,
    )


def trajectory_feasibility(
    peak_speed, peak_acceleration, constraints, dynamics, *, requested_hz, feed_m_s
):
    """Separate a valid geometric path from debug and guarded model timing.

    This report does not authorize faster hardware commands. It retains the
    configured measured-speed trips and exposes exactly which modeled constraint
    rejects the requested timing; failure of this sufficient box is not proof
    that no other trajectory or torque allocation can work.
    """
    speed = np.asarray(peak_speed, float)
    accel = np.asarray(peak_acceleration, float)
    if speed.shape != (6,) or accel.shape != (6,) or not np.isfinite([speed, accel]).all():
        raise ValueError("Six finite speed and acceleration peaks required")
    if (
        np.any(speed < 0)
        or np.any(accel < 0)
        or not math.isfinite(requested_hz)
        or requested_hz <= 0
    ):
        raise ValueError("Invalid trajectory timing")
    if not math.isfinite(feed_m_s) or feed_m_s < 0:
        raise ValueError("Invalid nominal feed")
    guarded = np.array([r["native_guarded_speed_deg_s"] for r in constraints["joints"]])
    acceleration = np.asarray(dynamics["raw_model_acceleration_deg_s2"])
    speed_ratio = speed / guarded
    accel_ratio = accel / acceleration
    minimum = max(1.0, float(max(speed_ratio)), math.sqrt(float(max(accel_ratio))))
    factor = minimum * 1.02 if minimum > 1 else 1.0
    return dict(
        requested_hz=requested_hz,
        requested_timing_passes_guarded_model=minimum <= 1.0 + 1e-6,
        native_guarded_speed_deg_s=guarded.tolist(),
        model_acceleration_deg_s2=acceleration.tolist(),
        requested_speed_fraction=speed_ratio.tolist(),
        requested_acceleration_fraction=accel_ratio.tolist(),
        limiting_speed_joint=int(np.argmax(speed_ratio)) + 1,
        limiting_acceleration_joint=int(np.argmax(accel_ratio)) + 1,
        guarded_model_time_scale=factor,
        guarded_model_hz=requested_hz / factor,
        guarded_model_feed_mm_s=feed_m_s * 1000 / factor,
        hardware_qualified=False,
        changes_debug_command_limits=False,
        scope="sampled provisional inertia acceleration box; other solutions may exist; no measured tracking or host/CAN timing qualification",
    )


def reduced_dynamics(geometry, raw_deg, *, pose_check=None):
    """MuJoCo mass/bias projected through the closed D3 linkage to six raw shafts.

    No motor torque is sent. CAD inertias are provisional: this is a model-derived
    planning envelope, never a claim that rated torque guarantees loaded motion.
    """
    mj = geometry.mj
    m = geometry.model
    (geometry.check if pose_check is None else pose_check)(raw_deg)
    q = geometry.data.qpos.copy()
    nv = m.nv
    geometry.data.qvel[:] = 0
    mj.mj_forward(m, geometry.data)
    mass = np.zeros((nv, nv))
    mj.mj_fullM(m, geometry.data, mass)
    bias = geometry.data.qfrc_bias.copy()
    basis = np.zeros((nv, 6))
    eps = 1e-5
    from simulation.arm_closure import ArmClosure

    for i in range(6):
        row = geometry.mapping.joints[str(i + 1)]
        variants = []
        for sign in (-1, 1):
            data = mj.MjData(m)
            data.qpos[:] = q
            closure = ArmClosure(m, data)
            data.qpos[geometry.addresses[i]] += sign * eps * row["model_per_raw"]
            if not closure.apply(data):
                raise ValueError("Cannot differentiate D3 closure")
            variants.append(data.qpos.copy())
        tangent = np.zeros(nv)
        mj.mj_differentiatePos(m, tangent, 2 * eps, *variants)
        basis[:, i] = tangent
    return basis.T @ mass @ basis, basis.T @ bias


def dynamics_constraints(geometry, raw_samples, constraints, *, acceleration_policy_deg_s2=30.0, pose_check=None):
    """Conservative simultaneous-acceleration box from sampled reduced inertia.

    Reserve half the remaining static torque for acceleration. Coriolis, friction,
    cables and incomplete CAD mass still need qualification; no certified torque
    curve or fabricated rotor inertia is substituted for missing data.
    """
    budget = np.array([r["torque_budget_nm"] for r in constraints["joints"]])
    max_bias = np.zeros(6)
    mass_envelope = np.zeros((6, 6))
    for raw in raw_samples:
        inertia, bias = reduced_dynamics(geometry, raw, pose_check=pose_check)
        max_bias = np.maximum(max_bias, abs(bias))
        mass_envelope = np.maximum(mass_envelope, abs(inertia))
    reserve = budget - max_bias
    if np.any(reserve <= 0):
        raise ValueError(
            "Model static torque exceeds derated budget: "
            + str(dict(bias=max_bias.tolist(), budget=budget.tolist()))
        )
    weights = np.array([row["physical_design_deg_s"] for row in constraints["joints"]])
    weights /= max(weights)
    # A shared factor enforces every coupled torque inequality M_abs @ a <= reserve/2.
    # Independent row-wise acceleration caps would not enforce cross-axis coupling.
    factor = float(np.min(0.5 * reserve / np.maximum(mass_envelope @ weights, 1e-8)))
    model_acceleration = np.degrees(weights * factor)
    acceleration = (
        model_acceleration
        if acceleration_policy_deg_s2 is None
        else np.minimum(acceleration_policy_deg_s2, model_acceleration)
    )
    return dict(
        raw_acceleration_deg_s2=acceleration.tolist(),
        max_static_torque_nm=max_bias.tolist(),
        raw_model_acceleration_deg_s2=model_acceleration.tolist(),
        commissioning_acceleration_deg_s2=acceleration_policy_deg_s2,
        max_reduced_mass_row_sum=mass_envelope.sum(axis=1).tolist(),
        reduced_mass_absolute_envelope=mass_envelope.tolist(),
        torque_budget_nm=budget.tolist(),
        model_sample_count=len(raw_samples),
        qualification="provisional CAD inertia; bounded acceleration design estimate",
    )
