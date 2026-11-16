"""Disabled cable-bound and independent straight-pose observations; no motion."""

from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from .arm_calibration import CalibrationMap, number


def settled_endpoint(samples, now):
    if len(samples) < 5:
        raise ValueError("Wait for at least five fresh disabled samples")
    times = [number(s.get("host_received_monotonic_s"), "sample time") for s in samples]
    if times[-1]-times[0] < .5 or any(b <= a for a,b in zip(times,times[1:])):
        raise ValueError("Hold the pose still for 0.5 seconds")
    values = []
    for sample in samples:
        if sample.get("source") != "disabled-refresh" or sample.get("historical") or sample.get("fault"):
            raise ValueError("Endpoint capture requires live, fault-free disabled feedback")
        rows = sample["feedback"]
        if len(rows) != 6 or [r["joint"] for r in rows] != list(range(1,7)):
            raise ValueError("Endpoint capture needs all six physical axes")
        for row in rows:
            if not row.get("feedback_valid") or row.get("enabled") or row.get("status") != 0:
                raise ValueError("All six motors must remain disabled")
            if not 0 <= number(row.get("age_ms"), "feedback age") < 80:
                raise ValueError("Stale endpoint feedback")
            if abs(number(row.get("velocity_rad_s"), "shaft velocity")) > .03:
                raise ValueError("Let the arm settle, then capture again")
        values.append([number(r["raw_position_deg"], "raw angle") for r in rows])
    if not 0 <= now-times[-1] < .08 or any(
            r["age_ms"]+(now-times[-1])*1000 >= 80 for r in samples[-1]["feedback"]):
        raise ValueError("Latest endpoint sample is stale")
    if any(max(q)-min(q) > .2 for q in zip(*values)):
        raise ValueError("The arm moved during capture; hold still and try again")
    return [sum(q)/len(q) for q in zip(*values)]


class A5Capture:
    """One live owner's recording round. Cable endpoints never define model zero."""

    def __init__(self, directory, mapping, evidence):
        if not evidence.get("identity_verified"):
            raise ValueError("Verified motor routing is required")
        if mapping.joints["5"]["model_per_raw"] != -1:
            raise ValueError("A5 capture requires the existing raw-positive CCW convention (gain -1)")
        if mapping.document.get("a5_cw_alignment"):
            raise ValueError("Obsolete mechanical-stop fit: load the original measured mapping")
        self.directory, self.mapping, self.evidence = Path(directory), mapping, deepcopy(evidence)
        self.captures = {}

    def record(self, kind, samples, now, *, reference=None):
        if kind not in ("CW", "CCW", "straight"):
            raise ValueError("Choose CW, CCW or straight")
        raw = settled_endpoint(samples, now)
        if kind == "straight":
            if not reference or reference.get("kind") != "a5_straight_axes":
                raise ValueError("Independent straight-pose model reference required")
            number(reference["model_deg"], "straight model angle")
        captures = deepcopy(self.captures)
        # A changed cable end starts a new pair; never reuse the opposite old end.
        if kind in ("CW", "CCW") and all(k in captures for k in ("CW", "CCW")):
            captures.pop("CW"); captures.pop("CCW")
        captures[kind] = dict(raw_output_shaft_deg=raw, samples=samples,
                              model_reference=reference if kind == "straight" else None)
        document, bounds, error = None, None, None
        if all(k in captures for k in ("CW", "CCW")):
            cw, ccw = (captures[k]["raw_output_shaft_deg"][4] for k in ("CW", "CCW"))
            if not 4 < ccw-cw < 360:
                error = "CW must be below CCW in raw degrees, with more than 4 and less than 360 degrees of travel. Re-record both ends; no wrapping applied."
            else:
                bounds = [cw+2., ccw-2.]
                if "straight" in captures:
                    straight = captures["straight"]
                    raw_straight = straight["raw_output_shaft_deg"][4]
                    if not bounds[0] < raw_straight < bounds[1]:
                        error = "Straight pose is outside the backed-off CW/CCW interval; recheck the observations."
                    else:
                        document = deepcopy(self.mapping.document)
                        document["joints"]["5"].update(
                            raw_branch_deg=bounds,
                            model_intercept_deg=straight["model_reference"]["model_deg"]+raw_straight,
                            basis="Independent operator-declared straight pose; cable ends restrict raw travel only")
                        document.update(production_qualified=False, initial_pose_target_authorized=False,
                            equipment_revision=f"{document['equipment_revision']}-a5-observed-{time.time_ns()}")
                        document["a5_observations"] = dict(cw_raw_deg=cw, ccw_raw_deg=ccw,
                            straight_raw_deg=raw_straight, backoff_deg=2.,
                            original_mapping_fingerprint=self.mapping.fingerprint,
                            physical_alignment_verified=False, cable_envelope_qualified=False,
                            note="Candidate for review. Independent poses and loaded clearance remain unqualified. Regenerate prior motion plans and standby raw targets.")
                        CalibrationMap(document)
        folder = self.directory/f"a5-capture-{time.time_ns()}"
        folder.mkdir(parents=True, exist_ok=False)
        record = dict(schema="forge.arm_a5_observations", version=2,
            created_utc=datetime.now(timezone.utc).isoformat(), captures=captures,
            source_evidence=self.evidence, candidate_complete=document is not None,
            raw_command_bounds_deg=bounds, backoff_deg=2., error=error,
            original_mapping_fingerprint=self.mapping.fingerprint,
            motor_targets_sent=False, motor_parameter_writes=False, production_qualified=False)
        (folder/"observation.json").write_text(json.dumps(record, indent=2, allow_nan=False)+"\n")
        if document is not None:
            (folder/"mapping.json").write_text(json.dumps(document, indent=2, allow_nan=False)+"\n")
        self.captures = captures
        return dict(path=folder/"observation.json", mapping=document, raw=raw,
                    kind=kind, captured=list(captures), bounds=bounds, validation_error=error)
