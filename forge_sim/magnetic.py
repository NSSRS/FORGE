"""Distance-based demo attraction between tagged wheel bodies and designated steel geoms.

Uses the existing convex collision meshes as a wheel-surface approximation.
It is not a magnetic field/material solver and does not magnetize other contacts.
"""
import mujoco
import numpy as np


class MagneticAttraction:
    def __init__(self, model):
        self.model = model
        self.pairs = []
        self.last_forces = {}
        self.force = np.zeros(model.nv)
        tag = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, "magnetic_bodies")
        if tag < 0:
            return
        def text(name):
            idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, name)
            if idx < 0:
                raise ValueError(f"Missing scene text: {name}")
            start = int(model.text_adr[idx])
            return bytes(model.text_data[start:start + int(model.text_size[idx])]).rstrip(b"\0").decode()
        self.surfaces = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                                  for name in text("magnetic_surface").split()], dtype=int)
        self.surface = int(self.surfaces[0]) if len(self.surfaces) else -1
        if self.surface < 0 or np.any(self.surfaces < 0):
            raise ValueError("Magnetic surface geom is missing")
        self.maximum, self.cutoff, self.decay = model.numeric("magnetic_parameters").data
        if self.maximum < 0 or self.cutoff <= 0 or self.decay <= 0:
            raise ValueError("Invalid magnetic force/distance parameters")
        for name in text("magnetic_bodies").split():
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body < 0:
                raise ValueError(f"Missing magnetic body: {name}")
            geoms = np.flatnonzero(model.geom_bodyid == body)
            # One merged visual mesh remains the magnetic wheel envelope;
            # physical collisions use the independent original part geoms.
            visual = geoms[model.geom_group[geoms] == 2]
            if len(visual) == 1:
                geoms = visual
            if len(geoms) != 1:
                raise ValueError(f"Expected one merged geom for magnetic body: {name}")
            self.pairs.append((name, body, int(geoms[0])))

    def _surface_bounds(self, data):
        # Cache only world-fixed geometry. Moving bodies use fresh bounds.
        ids = self.surfaces
        cached = getattr(self, "_bounds_cache", None)
        if cached is not None and np.array_equal(cached[0], ids):
            return cached[1], cached[2]
        rotation = data.geom_xmat[ids].reshape(-1, 3, 3)
        centers = data.geom_xpos[ids] + np.einsum(
            "nij,nj->ni", rotation, self.model.geom_aabb[ids, :3])
        halves = np.einsum("nij,nj->ni", np.abs(rotation),
                           self.model.geom_aabb[ids, 3:])
        if np.all(self.model.geom_bodyid[ids] == 0):
            self._bounds_cache = (ids.copy(), centers, halves)
        return centers, halves

    def compute(self, data):
        """Return magnetic generalized force without changing external/user forces."""
        self.force.fill(0)
        self.last_forces = {}
        if not self.pairs:
            return self.force
        centers, halves = self._surface_bounds(data)
        for name, body, geom in self.pairs:
            segment = np.zeros(6)
            distance = self.cutoff
            surface = self.surface
            # Vectorized conservative broad phase; exact mesh distance only nearby.
            outside = np.maximum(np.abs(data.geom_xpos[geom] - centers) - halves, 0)
            reach = self.model.geom_rbound[geom] + self.cutoff
            nearby = self.surfaces[np.einsum("ij,ij->i", outside, outside) <= reach*reach]
            for candidate in nearby:
                trial = np.zeros(6)
                gap = mujoco.mj_geomDistance(self.model, data, geom, int(candidate), self.cutoff, trial)
                if gap < distance:
                    distance, surface = gap, int(candidate)
                    segment[:] = trial
            self.last_forces[name] = 0.0
            if distance >= self.cutoff:
                continue
            direction = segment[3:] - segment[:3]
            # For penetration, the signed-distance segment reverses its direction.
            if distance < 0:
                direction *= -1
            norm = np.linalg.norm(direction)
            if norm < 1e-10:
                direction = data.geom_xpos[surface] - data.geom_xpos[geom]
                norm = np.linalg.norm(direction)
            if norm < 1e-10:
                continue
            gap = max(0.0, distance)
            magnitude = self.maximum * (1 - gap / self.cutoff)**2 / (1 + gap / self.decay)**2
            mujoco.mj_applyFT(self.model, data, magnitude * direction / norm,
                             np.zeros(3), segment[:3], body, self.force)
            self.last_forces[name] = magnitude
        return self.force

    def step(self, data):
        # Refresh geometry before distance queries (also after viewer joint edits).
        mujoco.mj_forward(self.model, data)
        force = self.compute(data)
        data.qfrc_applied[:] += force
        try:
            mujoco.mj_step(self.model, data)
        finally:
            data.qfrc_applied[:] -= force
