import unittest
import mujoco
import numpy as np
from forge_sim.magnetic import MagneticAttraction

class MagneticTests(unittest.TestCase):
    def fixture(self, x):
        m = mujoco.MjModel.from_xml_string(f'''<mujoco>
        <option gravity="0 0 0"/>
        <custom><text name="magnetic_bodies" data="wheel"/>
        <text name="magnetic_surface" data="steel"/>
        <numeric name="magnetic_parameters" data="100 .01 .002"/></custom>
        <worldbody><geom name="steel" type="box" size=".01 1 1"/>
        <body name="wheel" pos="{x} 0 0"><freejoint/>
        <geom type="sphere" size=".05" mass="1"/></body>
        <body name="ordinary" pos=".065 .3 0"><freejoint/>
        <geom type="sphere" size=".05" mass="1"/></body></worldbody></mujoco>''')
        d = mujoco.MjData(m); mujoco.mj_forward(m,d)
        return m,d,MagneticAttraction(m)

    def test_attraction_points_toward_steel_and_ignores_other_body(self):
        m,d,a = self.fixture(.065)
        f=a.compute(d).copy()
        self.assertLess(f[0],0)
        np.testing.assert_allclose(f[1:],0,atol=1e-10)
        self.assertGreater(a.last_forces['wheel'],0)
        d.qfrc_applied[6]=3
        old=d.qfrc_applied.copy();a.step(d)
        np.testing.assert_allclose(d.qfrc_applied,old)
        self.assertLess(d.qvel[0],0)

    def test_far_wheel_has_no_force(self):
        m,d,a=self.fixture(.08)
        np.testing.assert_allclose(a.compute(d),0)

    def test_contact_force_is_bounded_and_attractive(self):
        m,d,a=self.fixture(.059)
        f=a.compute(d)
        self.assertLess(f[0],0)
        self.assertAlmostEqual(a.last_forces['wheel'],100)

class WallSpawnTests(unittest.TestCase):
    def test_scene_spawns_six_magnets_near_wall_with_free_base(self):
        from pathlib import Path
        m = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1] / "model/scene.xml"))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        a = MagneticAttraction(m)
        self.assertEqual(len(a.pairs), 6)
        self.assertEqual(m.joint("floating_base").type[0], mujoco.mjtJoint.mjJNT_FREE)
        for name, body, geom in a.pairs:
            gap = mujoco.mj_geomDistance(m,d,geom,a.surface,1,None)
            self.assertGreater(gap, 0, name)
            self.assertLess(gap, a.cutoff, name)
        a.compute(d)
        self.assertTrue(all(f > 0 for f in a.last_forces.values()))

if __name__ == '__main__':
    unittest.main()


class MagneticBroadPhaseTests(unittest.TestCase):
    def test_matches_exhaustive_surface_search(self):
        from pathlib import Path
        from forge_sim.spawn_fit import reset_to_spawn
        for scene in ("scene.xml", "scene_curved.xml", "scene_bumpy.xml"):
            path = Path(__file__).resolve().parents[1] / "model" / scene
            m = mujoco.MjModel.from_xml_path(str(path))
            d = mujoco.MjData(m)
            reset_to_spawn(m, d)
            fast, reference = MagneticAttraction(m), MagneticAttraction(m)
            # Infinite bounds disable only broad-phase rejection, querying every geom.
            reference._surface_bounds = lambda data: (
                np.zeros((len(reference.surfaces), 3)),
                np.full((len(reference.surfaces), 3), np.inf))
            for offset in (0, .005, .025):
                reset_to_spawn(m, d)
                d.qpos[1] += offset
                mujoco.mj_forward(m, d)
                np.testing.assert_allclose(fast.compute(d), reference.compute(d),
                                           rtol=1e-10, atol=1e-10)
                self.assertEqual(fast.last_forces, reference.last_forces)
