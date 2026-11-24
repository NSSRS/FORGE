"""Session-local recovery from an observed hardware pose, without rebasing limits."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MeasuredStart:
    raw: tuple
    mapping_fingerprint: str

    def __post_init__(self):
        q = np.asarray(self.raw, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("Measured hardware start requires six finite angles")
        if not isinstance(self.mapping_fingerprint, str) or not self.mapping_fingerprint:
            raise ValueError("Measured hardware start requires a mapping fingerprint")
        object.__setattr__(self, "raw", tuple(float(x) for x in q))

    def document(self):
        return dict(version=1, raw_deg=list(self.raw), mapping_fingerprint=self.mapping_fingerprint)

    @classmethod
    def from_document(cls, document):
        if document.get("version") != 1:
            raise ValueError("Unsupported measured hardware start policy")
        return cls(tuple(document["raw_deg"]), document["mapping_fingerprint"])

    def bounds(self, mapping):
        if mapping.fingerprint != self.mapping_fingerprint:
            raise ValueError("Measured hardware start belongs to a different mapping")
        bounds = np.array([mapping.raw_limits(i) for i in range(1, 7)], dtype=float)
        # A4 and A5 carry recorded cable envelopes, rather than simulator pose
        # windows. A6's saved model window can be narrower than its fixed cable
        # envelope: extend the former to the observation, never the latter.
        for i in (0, 1, 2, 5):
            bounds[i, 0] = min(bounds[i, 0], self.raw[i])
            bounds[i, 1] = max(bounds[i, 1], self.raw[i])
        bounds[5, 0] = max(bounds[5, 0], mapping.a6_bounds[0])
        bounds[5, 1] = min(bounds[5, 1], mapping.a6_bounds[1])
        return bounds

    def check_raw(self, mapping, raw, *, measured=False):
        from .weld_plan import finite_vector
        from .recovery import START_TOLERANCE_DEG

        q = finite_vector(raw, "hardware recovery pose")
        bounds = self.bounds(mapping)
        if measured:
            # Reuse the existing planning/preload tolerance for feedback at the
            # observed outer extent. Command bounds and cable limits stay exact.
            for i in (0, 1, 2, 5):
                low, high = mapping.raw_limits(i+1)
                if self.raw[i] < low:
                    bounds[i, 0] -= START_TOLERANCE_DEG
                if self.raw[i] > high:
                    bounds[i, 1] += START_TOLERANCE_DEG
            bounds[5, 0] = max(bounds[5, 0], mapping.a6_bounds[0])
            bounds[5, 1] = min(bounds[5, 1], mapping.a6_bounds[1])
        invalid = np.flatnonzero((q < bounds[:, 0] - 1e-9) | (q > bounds[:, 1] + 1e-9))
        if len(invalid):
            axis = int(invalid[0])
            cable_excess = axis in (3, 4) or (axis == 5 and not mapping.a6_bounds[0] <= q[5] <= mapping.a6_bounds[1])
            reason = "fixed cable bounds" if cable_excess else "measured-start recovery bounds"
            raise ValueError(f"A{axis+1}: pose exceeds {reason}")
        return q


class RecoveryGeometry:
    """Use observed angle extents only for recovery with the selected space policy."""

    def __init__(self, geometry, start):
        self.geometry, self.start = geometry, start
        start.bounds(geometry.mapping)

    def __getattr__(self, name):
        return getattr(self.geometry, name)

    def check(self, raw, *, clearance_mm=None):
        return self.geometry.check_recovery(raw, self.start.document(), clearance_mm=clearance_mm)

    def validate_path(self, path, *, clearance_mm=None):
        return self.geometry.validate_path(path, clearance_mm=clearance_mm,
                                          raw_bounds=self.start.bounds(self.mapping), pose_check=self.check)
