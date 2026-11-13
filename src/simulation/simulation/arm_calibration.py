"""Model-derived calibration paths at the measured six-axis pose; no hardware.

Normal model contacts plus explicitly named triangle-mesh stop pairs are checked.
The latter include adjacent parts omitted by MuJoCo's parent-body contact filter.
Planning is sampled and conservative; it does not certify unmodeled equipment.
"""
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from motors.arm_calibration import CalibrationMap, constraint_limits, number
from simulation.arm_sweep import ArmSweepStudy, DRIVES


def scene_fingerprint(scene):
    scene = Path(scene).resolve()
    # Include XML and geometry, never ignored credentials/config.json.
    files = sorted(set(scene.parent.glob('*.xml')) | {scene} |
                   {p for p in (scene.parent/'assets').rglob('*')
                    if p.is_file() and p.suffix.lower() in ('.obj', '.stl', '.msh')})
    if (scene.parent/'initial_pose.json').is_file():files.append(scene.parent/'initial_pose.json')
    h = hashlib.sha256()
    for path in files:
        h.update(str(path.relative_to(scene.parent)).encode())
        h.update(hashlib.sha256(path.read_bytes()).digest())
    h.update(scene.name.encode())
    return h.hexdigest()


class ExplicitStops:
    def __init__(self, model, data, pairs):
        self.model, self.data = model, data
        self.pairs, self.objects = [], {}
        if not pairs:
            return
        import fcl
        self.fcl = fcl
        shapes = {}
        for names in pairs:
            if len(names) != 2:
                raise ValueError('Each adjacent_stop_pairs entry needs two model geom names')
            ids = tuple(model.geom(name).id for name in names)
            self.pairs.append(ids)
            for geom in ids:
                if geom in self.objects:
                    continue
                mesh = int(model.geom_dataid[geom])
                if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH or mesh < 0:
                    raise ValueError('Explicit stop pairs must be triangle meshes')
                if mesh not in shapes:
                    va, vn = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                    fa, fn = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
                    vertices = np.array(model.mesh_vert[va:va+vn], dtype=np.float64)
                    faces = np.array(model.mesh_face[fa:fa+fn], dtype=np.int32)
                    shape = fcl.BVHModel()
                    shape.beginModel(len(vertices), len(faces))
                    shape.addSubModel(vertices, faces);shape.endModel()
                    shapes[mesh] = shape
                self.objects[geom] = fcl.CollisionObject(shapes[mesh])

    def check(self, clearance_mm):
        for geom, obj in self.objects.items():
            obj.setTransform(self.fcl.Transform(
                np.array(self.data.geom_xmat[geom].reshape(3, 3)), np.array(self.data.geom_xpos[geom])))
        for a, b in self.pairs:
            value = self.fcl.distance(self.objects[a], self.objects[b],
                self.fcl.DistanceRequest(), self.fcl.DistanceResult())
            if value * 1000 < clearance_mm:
                return dict(clear=False, reason='adjacent mechanical stop', distance_mm=float(value)*1000,
                            geom_names=[self.model.geom(a).name, self.model.geom(b).name])
        return None


class ModelCalibrationPlanner:
    def __init__(self, scene, mapping, *, clearance_mm=2., stop_clearance_mm=.1,
                 step_deg=.25, extent_deg=90., backoff_deg=2.):
        for name, value in locals().copy().items():
            if name in ('clearance_mm', 'stop_clearance_mm', 'step_deg', 'extent_deg', 'backoff_deg'):
                if number(value, name) <= 0:
                    raise ValueError(f'{name} must be positive')
        if not .05 <= step_deg <= .5 or not step_deg <= extent_deg <= 180:
            raise ValueError('Require .05 <= step <= .5 and step <= extent <=180 degrees')
        if not .2 <= backoff_deg <= 20:
            raise ValueError('Require .2 <= angular backoff <=20 degrees')
        self.scene = Path(scene).resolve()
        self.mapping = CalibrationMap(mapping)
        self.fingerprint = scene_fingerprint(self.scene)
        self.step, self.extent, self.backoff = step_deg, extent_deg, backoff_deg
        self.stop_clearance_mm = stop_clearance_mm
        self.study = ArmSweepStudy(self.scene, clearance_mm=clearance_mm,
                                   step_deg=step_deg, refinement_deg=.05, extent_deg=extent_deg)
        pose_file = self.scene.parent/'initial_pose.json'
        if pose_file.exists():
            for name, angle in json.loads(pose_file.read_text())['joint_angles_rad'].items():
                self.study.data.qpos[int(self.study.model.joint(name).qposadr[0])] = angle
        if not self.study.closure.apply(self.study.data):
            raise ValueError('Model seed cannot close the D3 linkage')
        self.seed = self.study.data.qpos.copy()
        self.stops = ExplicitStops(self.study.model, self.study.data, mapping.get('adjacent_stop_pairs', []))

    def set_pose(self, raw):
        angles = self.mapping.pose(raw)
        self.study.reset(self.seed)
        for joint, address in enumerate(self.study.addresses, 1):
            self.study.data.qpos[address] = math.radians(angles[str(joint)])
        if not self.study.closure.apply(self.study.data):
            raise ValueError('Measured pose cannot close the model linkage; inspect mapping/branch')
        self.study.home = self.study.data.qpos.copy()
        result = self.inspect()
        if not result['clear']:
            raise ValueError('Current measured pose fails model clearance; no path authorized: '+json.dumps(result))

    def inspect(self):
        result = self.study.inspect()
        if not result['clear']:
            return result
        return self.stops.check(self.stop_clearance_mm) or result

    def sample(self, joint, raw):
        angle = self.mapping.model(joint, raw)
        jid = self.study.model.joint(DRIVES[joint-1]).id
        if self.study.model.jnt_limited[jid]:
            lo, hi = np.degrees(self.study.model.jnt_range[jid])
            if not lo <= angle <= hi:
                return dict(clear=False, reason='authored joint limit')
        result = self.study.pose(joint, angle)
        if not result['clear']:
            return result
        return self.stops.check(self.stop_clearance_mm) or result

    def plan(self, raw, joint, constraints=()):
        if type(joint) is not int or not 1 <= joint <= 6:
            raise ValueError('Joint must be 1..6')
        self.set_pose(raw)
        lower, upper = constraint_limits(self.mapping, joint, constraints, self.fingerprint)
        center = raw[str(joint)]
        if not lower <= center <= upper:
            raise ValueError('Current position is outside the recorded constraint backoff; reposition manually before planning')
        ends = {}
        for name, sign, cap in (('lower', -1, lower), ('upper', 1, upper)):
            self.study.reset(self.study.home)
            end = max(cap, center-self.extent) if sign < 0 else min(cap, center+self.extent)
            reason = 'configured encoder branch / recorded constraint' if end == cap else 'search horizon (not a physical stop)'
            last, boundary, count = center, None, 0
            raw_step = self.step / max(1., abs(self.mapping.joints[str(joint)]['model_per_raw']))
            for offset in np.arange(raw_step, abs(end-center)+raw_step, raw_step):
                angle = center + sign*min(float(offset), abs(end-center))
                sample = self.sample(joint, angle);count += 1
                if not sample['clear']:
                    reason, boundary = sample['reason'], dict(raw_deg=angle, sample=sample)
                    break
                last = angle
            # Back off from the last clear sample, never advance beyond it.
            goal = center + sign*max(0., sign*(last-center)-self.backoff)
            ends[name] = dict(raw_deg=goal, model_deg=self.mapping.model(joint, goal),
                              last_clear_raw_deg=last, boundary=boundary, reason=reason, samples=count)
        return dict(schema='forge.model_calibration_path', version=1, joint=joint,
                    scene=str(self.scene), scene_fingerprint=self.fingerprint,
                    mapping_fingerprint=self.mapping.fingerprint, start_raw_deg=dict(raw),
                    start_model_deg=self.mapping.pose(raw), ends=ends,
                    step_deg=self.step, clearance_mm=self.study.clearance*1000,
                    stop_clearance_mm=self.stop_clearance_mm, backoff_deg=self.backoff,
                    adjacent_stop_pairs=self.mapping.document.get('adjacent_stop_pairs', []),
                    hardware_limits=False, production_qualified=False,
                    limitations=['Only this six-axis pose and selected single-axis path were sampled.',
                                 'Other axes/chassis must remain at the planned pose; added gear must be included in the model.',
                                 'Unmodeled cables and obstacles require operator stop and constraint recording.',
                                 'Existing contact filters remain except the explicitly checked adjacent stop pairs.',
                                 'Discrete samples, configured margins and speed are not measured stopping-distance certification.'])


def calculate_path(request):
    """Spawn-process entry point: receives values only, never a port or Qt object."""
    planner = ModelCalibrationPlanner(request['scene'], request['mapping'], **request.get('options', {}))
    return planner.plan(request['raw'], request['joint'], request.get('constraints', []))
