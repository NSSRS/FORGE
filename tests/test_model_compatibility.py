import source_paths  # source-checkout package imports
from robot_description.paths import model_directory
MODEL_DIR = model_directory()
"""Regression checks for consumers of the current arm/chassis model."""
import importlib.util
import unittest
from pathlib import Path
if any(importlib.util.find_spec(n) is None for n in ('mujoco','numpy','scipy')):
    raise unittest.SkipTest('requires src/simulation/requirements.txt')
import json
import numpy as np
import mujoco
from simulation.spawn_fit import reset_to_spawn
from simulation.arm_closure import ArmClosure
ROOT=Path(__file__).resolve().parents[1]

class ModelCompatibilityTests(unittest.TestCase):
    def load(self):
        m=mujoco.MjModel.from_xml_path(str(MODEL_DIR/'scene.xml'))
        d=mujoco.MjData(m);reset_to_spawn(m,d)
        return m,d
    def test_default_pose_and_closure(self):
        m,d=self.load()
        self.assertEqual((m.njnt,m.nu,m.neq),(20,10,1))
        config=json.loads((MODEL_DIR/'initial_pose.json').read_text())
        self.assertEqual(set(config['joint_zero_offsets_rad']),{'b2_arm_d1'})
        for name,value in config['joint_angles_rad'].items():
            self.assertAlmostEqual(d.qpos[m.jnt_qposadr[m.joint(name).id]],value)
            self.assertAlmostEqual(d.ctrl[m.actuator(name+'_target').id],value)
        a=m.site('closing_b4_2_arm_rod_d3_1').id;b=m.site('closing_b4_2_arm_rod_d3_2').id
        self.assertLess(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b]),1e-6)
        self.assertFalse(any(c.dist<-.00005 for c in d.contact))
    def test_static_d3_preserves_other_commands(self):
        m,d=self.load();solver=ArmClosure(m,d)
        adr=int(m.jnt_qposadr[m.joint('b4_arm_d3').id])
        other=int(m.jnt_qposadr[m.joint('b3_arm_d2').id])
        for value in (-.4,-.5,-.44):
            d.qpos[adr]=value
            self.assertTrue(solver.apply(d))
            self.assertAlmostEqual(d.qpos[adr],value)
            self.assertAlmostEqual(d.qpos[other],.0942)
            self.assertEqual(d.time,0)
            self.assertLess(np.linalg.norm(d.site_xpos[solver.sites[0]]-d.site_xpos[solver.sites[1]]),1e-6)
    @unittest.skipUnless(importlib.util.find_spec("yaml"), "requires PyYAML to read chassis.yaml")
    def test_driven_wheel_geometry_matches_chassis_config(self):
        import yaml

        config_path = ROOT / "src/bringup/config/chassis.yaml"
        config = yaml.safe_load(config_path.read_text())["motor_driver"]["ros__parameters"]
        model = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "scene.xml"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        joints = [model.joint(f"wheel_{number}").id for number in range(1, 5)]
        centers = data.xanchor[joints]
        lateral = data.xaxis[joints[0]]
        track = abs(np.dot(centers[:2].mean(axis=0) - centers[2:].mean(axis=0), lateral))
        tolerance_m = 0.00005  # Allow mesh tessellation and CAD export rounding.
        self.assertAlmostEqual(track, config["track_width_m"], delta=tolerance_m)

        for number, joint in enumerate(joints, start=1):
            anchor = data.xanchor[joint]
            axis = data.xaxis[joint]
            wheel_geoms = np.flatnonzero(model.geom_bodyid == model.jnt_bodyid[joint])
            treads = {
                int(model.geom_group[geom]): geom
                for geom in wheel_geoms
                if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH
                and (
                    model.geom_group[geom] == 2
                    or model.mesh(int(model.geom_dataid[geom])).name == "sus_ref8_磁铁轮_80mm_1"
                )
            }
            with self.subTest(wheel=number):
                self.assertEqual(set(treads), {2, 3})
                collision_center = data.geom_xpos[treads[3]]
                self.assertLess(np.linalg.norm(np.cross(collision_center - anchor, axis)), tolerance_m)
            for group, geom in treads.items():
                with self.subTest(wheel=number, geom_group=group):
                    mesh = model.geom_dataid[geom]
                    first = model.mesh_vertadr[mesh]
                    points = model.mesh_vert[first:first + model.mesh_vertnum[mesh]]
                    rotation = data.geom_xmat[geom].reshape(3, 3)
                    world = points @ rotation.T + data.geom_xpos[geom]
                    radius = np.linalg.norm(np.cross(world - anchor, axis), axis=1).max()
                    self.assertAlmostEqual(radius, config["wheel_radius_m"], delta=tolerance_m)

    def test_wheel_projection_preserves_full_spawn(self):
        spec=importlib.util.spec_from_file_location('model_mirror',ROOT/'scripts/mujoco_motor_viewer.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        projection=module.Projection(MODEL_DIR/'scene.xml',dict(wheel_motor_ids=[1,2,3,4],wheel_directions=[-1,-1,1,1],external_ratios=[1,1,1,1],bench_omit_motor_id=4))
        m,d=self.load()
        np.testing.assert_allclose(projection.data.qpos,d.qpos)
        for i,state in enumerate(projection.mirror.wheels):
            state.angle=.1*(i+1)
        projection.apply(1.)
        expected=d.qpos.copy()
        for i,j in enumerate(projection.joints):
            expected[m.jnt_qposadr[j]]+=.1*(i+1)
        np.testing.assert_allclose(projection.data.qpos,expected)
        # Feedback status colors may override driven wheels, never arm/link colors.
        driven_bodies = m.jnt_bodyid[projection.joints]
        other_geoms = ~np.isin(m.geom_bodyid, driven_bodies)
        np.testing.assert_array_equal(projection.model.geom_rgba[other_geoms], m.geom_rgba[other_geoms])
        np.testing.assert_array_equal(projection.model.geom_matid[other_geoms], m.geom_matid[other_geoms])
        self.assertEqual(projection.data.time,0)
if __name__=='__main__':unittest.main()
