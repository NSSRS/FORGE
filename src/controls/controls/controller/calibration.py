"""Fit encoder alignment without fitting an unmeasured transmission ratio."""

from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
from motors.arm_calibration import CalibrationMap, DRIVES, digest
from motors.arm_travel import a4_cable_bounds


def select_a1_encoder_branch(mapping, turns=0):
    """Translate a verified A1 restart branch, keeping raw bus coordinates.

    This seeds the initial map and stays explicit in simulation. Hardware may
    choose a nearer equivalent target on a fresh stationary planning request;
    feedback is never reduced modulo 360. A4's winding envelope is not translated.
    """
    if type(turns) is not int or turns not in (-1, 0, 1):
        raise ValueError("A1 encoder branch must be -1, 0 or +1 whole turns")
    if not turns:
        return mapping
    if mapping.document.get("encoder_branch_selection"):
        raise ValueError("Select the A1 branch from the original saved mapping, not an already translated session")
    document = deepcopy(mapping.document)
    row = document["joints"]["1"]
    if abs(row["model_per_raw"]) != 1.0:
        raise ValueError("A1 encoder branch selection requires unit output-shaft gain")
    shift = 360.0 * turns
    row["model_intercept_deg"] -= row["model_per_raw"] * shift
    row["raw_branch_deg"] = [q + shift for q in row["raw_branch_deg"]]
    document["encoder_branch_selection"] = dict(
        joint=1,
        turns=turns,
        raw_shift_deg=shift,
        source_mapping_fingerprint=mapping.fingerprint,
        basis="Explicit operator selection of A1 encoder restart branch; physical pose must be verified",
    )
    document["production_qualified"] = False
    return CalibrationMap(document)


def check_a1_recovery_branch(mapping, current, goal):
    """Reject a stale long target if a caller skipped A1 map synchronization."""
    low, high = mapping.raw_limits(1)
    distance = abs(float(goal[0]) - float(current[0]))
    if distance > 180.0:
        raise ValueError(
            f"A1 recovery would move {distance:.2f} deg from {current[0]:+.4f} "
            f"to {goal[0]:+.4f} deg with encoder branch "
            f"[{low:+.4f}, {high:+.4f}] deg. Synchronize the nearest equivalent "
            "A1 target and mapping before planning. No motion was authorized."
        )


def nearest_a1_recovery_branch(mapping, current, goal):
    """Choose the nearest equivalent A1 target in the measured raw coordinates.

    Only the A1 unit-gain, full-turn model branch is translated. Feedback is
    never normalized and A2-A6, including cable envelopes, stay unchanged.
    Call only for a fresh stationary planning request, never during execution.
    """
    current, goal = np.asarray(current, float), np.asarray(goal, float)
    if current.shape != (6,) or goal.shape != (6,) or not np.isfinite([current, goal]).all():
        raise ValueError("A1 branch selection requires six finite start and goal angles")
    delta = float(goal[0] - current[0])
    if abs(delta) <= 180.0:
        return mapping, goal.tolist(), None
    row = mapping.joints["1"]
    low, high = mapping.raw_limits(1)
    if abs(row["model_per_raw"]) != 1.0 or not math.isclose(high - low, 360.0, abs_tol=1e-8):
        raise ValueError("Nearest A1 target requires a unit-gain full-turn branch")
    turns = int(math.floor((current[0] - goal[0]) / 360.0 + 0.5))
    if abs(turns) > 1:
        raise ValueError("A1 recovery requires more than one adjacent encoder branch; inspect the raw start")
    shift = 360.0 * turns
    document = deepcopy(mapping.document)
    changed = document["joints"]["1"]
    changed["model_intercept_deg"] -= changed["model_per_raw"] * shift
    changed["raw_branch_deg"] = [q + shift for q in changed["raw_branch_deg"]]
    prior = document.get("encoder_branch_selection", {})
    total_turns = prior.get("turns", 0) + turns
    document["encoder_branch_selection"] = dict(
        joint=1, turns=total_turns, raw_shift_deg=360.0 * total_turns,
        source_mapping_fingerprint=prior.get("source_mapping_fingerprint", mapping.fingerprint),
        basis="Nearest equivalent A1 recovery target selected from fresh stationary raw feedback",
    )
    document["production_qualified"] = False
    selected = CalibrationMap(document)
    target = goal.copy()
    target[0] += shift
    decision = dict(
        current_raw_deg=float(current[0]), previous_goal_raw_deg=float(goal[0]),
        goal_raw_deg=float(target[0]), delta_deg=float(target[0] - current[0]),
        previous_delta_deg=delta, relative_turns=turns, turns=total_turns,
        direction="CCW" if target[0] > current[0] else "CW",
        previous_mapping_fingerprint=mapping.fingerprint,
        mapping_fingerprint=selected.fingerprint,
    )
    return selected, target.tolist(), decision


def check_a1_branch_translation(previous, selected):
    """Reject any runtime mapping change other than an exact A1 whole turn."""
    before, after = deepcopy(previous.document), deepcopy(selected.document)
    for document in (before, after):
        document.pop("encoder_branch_selection", None)
        document.pop("production_qualified", None)
    old, new = before["joints"]["1"], after["joints"]["1"]
    gain = old["model_per_raw"]
    if abs(gain) != 1 or new["model_per_raw"] != gain:
        raise ValueError("A1 branch translation requires unchanged unit gain")
    shift = (old["model_intercept_deg"] - new["model_intercept_deg"]) / gain
    if not math.isclose(shift / 360, round(shift / 360), abs_tol=1e-10) or abs(shift) > 360 + 1e-8:
        raise ValueError("A1 mapping change must be one adjacent whole turn")
    if not np.allclose(np.subtract(new["raw_branch_deg"], old["raw_branch_deg"]), shift, atol=1e-8, rtol=0):
        raise ValueError("A1 branch bounds must translate with its target")
    new["model_intercept_deg"] = old["model_intercept_deg"]
    new["raw_branch_deg"] = old["raw_branch_deg"]
    if before != after or selected.document.get("production_qualified") is not False:
        raise ValueError("A1 branch selection cannot change other calibration or qualification fields")


def fit_alignment(raw_deg, model_deg, *, direction=-1.0, known_ratio=1.0, allow_scale=False):
    """Offset is identifiable from one anchor; scale requires independent evidence.

    Two endpoint readings alone have no residual degrees of freedom for an affine
    fit. Report that fit for comparison, but do not mistake CAD/contact tolerance
    for a physical gear ratio. All coordinates are already output-shaft angles.
    """
    x = np.asarray(raw_deg, float)
    y = np.asarray(model_deg, float)
    if x.ndim != 1 or not len(x) or x.shape != y.shape or not np.isfinite([x, y]).all():
        raise ValueError("Paired finite raw/model references required")
    if direction not in (-1.0, 1.0) or not math.isfinite(known_ratio) or known_ratio <= 0:
        raise ValueError("Known sign and positive external ratio required")
    gain = direction / known_ratio
    offset = float(np.mean(y - gain * x))
    comparison = None
    if len(x) >= 2 and np.ptp(x) > 1e-6:
        a, b = np.polyfit(x, y, 1)
        comparison = dict(gain=float(a), offset_deg=float(b), residual_deg=(y - a * x - b).tolist())
    if allow_scale:
        if len(x) < 3 or comparison is None:
            raise ValueError("Scale fitting needs at least three independent matched poses")
        if comparison["gain"] * direction <= 0:
            raise ValueError("Fitted sign contradicts known direction")
        gain, offset = comparison["gain"], comparison["offset_deg"]
    return dict(
        method="affine" if allow_scale else "sign_plus_offset",
        gain=gain,
        offset_deg=offset,
        residual_deg=(y - gain * x - offset).tolist(),
        affine_comparison=comparison,
    )


def derive_measured_configuration(root, *, backoff_deg=2.0):
    """Reproducible candidate; never re-label a calculated standby as a measurement."""
    root = Path(root)
    if not math.isfinite(backoff_deg) or not 0 < backoff_deg <= 10:
        raise ValueError("Positive measured-endpoint backoff <=10 degrees required")
    prior = json.loads(
        (root / "src/robot_description/config/arm_calibration_candidate.json").read_text()
    )
    capture = json.loads(
        (root / "docs/measurements/arm-reference-2026-11-12.capture.json").read_text()
    )
    pose = json.loads((root / "src/robot_description/mujoco/initial_pose.json").read_text())
    old = CalibrationMap(prior)
    document = deepcopy(prior)
    fits = {}
    initial = []
    adjustments = []
    for i, name in enumerate(DRIVES, 1):
        key = str(i)
        row = document["joints"][key]
        old_model = math.degrees(pose["joint_angles_rad"].get(name, 0.0))
        requested_raw = old.raw(i, old_model)
        if i in (2, 3):
            # Latest corrected A3 values already belong to the reviewed candidate.
            raw = row["raw_branch_deg"]
            model = [old.model(i, q) for q in raw]
            fit = fit_alignment(raw, model)
            fit["reference_basis"] = (
                "measured endpoints paired with prior CAD stop estimates; contact not independently verified"
            )
        else:
            raw = capture["joints"][key]["reference"]["raw_deg"]
            model = math.degrees(capture["joints"][key]["model_initial_rad"])
            fit = fit_alignment([raw], [model])
            requested_raw = raw
            fit["reference_basis"] = "Nov 12 full-precision physical initial anchor"
        fits[key] = fit
        row.update(
            model_per_raw=fit["gain"],
            model_intercept_deg=fit["offset_deg"],
            basis="output-shaft sign plus fitted offset; no empirical gear scaling",
        )
        if i == 4:
            row['raw_branch_deg'] = list(a4_cable_bounds())
        lo, hi = row["raw_branch_deg"]
        target = float(np.clip(requested_raw, lo + backoff_deg, hi - backoff_deg))
        initial.append(target)
        adjustments.append(
            dict(
                joint=i,
                requested_raw_deg=requested_raw,
                standby_raw_deg=target,
                measured_initial=i in (1, 4, 5, 6),
                adjusted=abs(target - requested_raw) > 1e-9,
            )
        )
    document.update(
        equipment_revision="forge-arm-2026-11-15-measured-offset-candidate",
        basis="Full-precision initial anchors and corrected measured endpoint spans; unit output-shaft gain.",
        initial_pose_target_authorized=False,
        notes=[
            "Generated from preserved source measurements; no new hardware evidence.",
            "A2 initial is inherited model intent; A3 standby is projected inside measured bounds with explicit backoff.",
            "No external gear ratio was identified. Dynamics and physical calibration remain unqualified.",
            "A4 cable travel is fixed at CW/CCW 270 degrees from the preserved initial anchor, per operator on Nov 16; startup does not rebase it.",
        ],
    )
    mapping = CalibrationMap(document)
    model_angles = {
        name: math.radians(mapping.model(i, q))
        for i, (name, q) in enumerate(zip(DRIVES, initial), 1)
    }
    document["simulation_model_deg"] = {
        str(i): mapping.model(i, q) for i, q in enumerate(initial, 1)
    }
    document["simulation_pose_note"] = (
        "Measured-bound controller standby; calculated candidate, not a six-axis captured pose."
    )
    report = dict(
        schema="forge.measured_alignment",
        version=1,
        fit=fits,
        initial=adjustments,
        endpoint_backoff_deg=backoff_deg,
        source_fingerprints={
            "prior_mapping": digest(prior),
            "capture": digest(capture),
            "authored_pose": digest(pose),
            "a4_cable_policy": digest(json.loads((root / "src/robot_description/config/arm_cable_limits.json").read_text())),
        },
        production_qualified=False,
    )
    initial_pose = dict(
        joint_angles_rad=model_angles,
        raw_output_shaft_deg=initial,
        description="Controller model standby dictated by measured raw bounds and anchors.",
        mapping_fingerprint=CalibrationMap(document).fingerprint,
        derivation=report,
        hardware_qualified=False,
    )
    return document, initial_pose, report


def save_configuration(root, destination):
    mapping, pose, report = derive_measured_configuration(root)
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    for name, data in (
        ("arm_calibration_measured.json", mapping),
        ("arm_initial_measured.json", pose),
        ("arm_alignment_report.json", report),
    ):
        (destination / name).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    return report


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Explicitly write/regenerate the three candidate JSON files here",
    )
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[4]
    result = (
        save_configuration(root, args.output_dir)
        if args.output_dir
        else derive_measured_configuration(root)[2]
    )
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0
