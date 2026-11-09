"""Connection dimensions and closure after a CAD rounding correction."""
import source_paths
import importlib.util
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

if any(importlib.util.find_spec(n) is None for n in ('mujoco', 'numpy', 'scipy')):
    raise unittest.SkipTest('requires src/simulation/requirements.txt')
import mujoco
import numpy as np
from simulation.arm_geometry import normalize_d3_rod_centers, D3_ROD_BODY, D3_ROD_SITE
from simulation.arm_closure import ArmClosure
from simulation.paths import MODEL_DIR
from simulation.spawn_fit import reset_to_spawn, apply_initial_arm_pose


class ArmGeometryTests(unittest.TestCase):
    def test_cad_normalization_only_changes_connection_site(self):
        root = ET.parse(MODEL_DIR/'cad_robot.xml').getroot()
        before = ET.tostring(root)
        site = root.find(f".//site[@name='{D3_ROD_SITE}']")
        old = site.get('pos')
        self.assertAlmostEqual(normalize_d3_rod_centers(root)*1000, 150.002488146, places=8)
        point = np.fromstring(site.get('pos'), sep=' ')
        self.assertAlmostEqual(np.linalg.norm(point), .150, places=14)
        np.testing.assert_allclose(point/np.linalg.norm(point),
                                   np.fromstring(old, sep=' ')/np.linalg.norm(np.fromstring(old, sep=' ')), atol=1e-14)
        normalize_d3_rod_centers(root)
        np.testing.assert_allclose(np.fromstring(site.get('pos'), sep=' '), point, atol=1e-16)
        site.set('pos', old)
        self.assertEqual(ET.tostring(root), before)

    def test_rejects_real_geometry_changes_and_invalid_coordinates(self):
        for value in ('0 0 .160', '0 0 0', 'nan 0 .15', '0 .15'):
            root = ET.fromstring(f'<mujoco><worldbody><body name="{D3_ROD_BODY}">'
                                f'<joint name="{D3_ROD_BODY}" type="ball"/>'
                                f'<site name="{D3_ROD_SITE}" pos="{value}"/>'
                                '</body></worldbody></mujoco>')
            before = ET.tostring(root)
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize_d3_rod_centers(root)
            self.assertEqual(ET.tostring(root), before)

    def test_all_active_scenes_use_150mm_and_close_at_saved_pose(self):
        for scene in ('scene.xml', 'scene_curved.xml', 'scene_bumpy.xml', 'arm_preview.xml'):
            with self.subTest(scene=scene):
                m = mujoco.MjModel.from_xml_path(str(MODEL_DIR/scene))
                d = mujoco.MjData(m)
                if scene == 'arm_preview.xml':
                    apply_initial_arm_pose(m, d)
                else:
                    reset_to_spawn(m, d)
                    self.assertEqual((m.njnt, m.nu, m.neq), (20, 10, 1))
                joint = m.joint(D3_ROD_BODY).id
                site = m.site(D3_ROD_SITE).id
                self.assertAlmostEqual(np.linalg.norm(m.site_pos[site]-m.jnt_pos[joint]), .150, places=14)
                a, b = [m.site(f'closing_b4_2_arm_rod_d3_{i}').id for i in (1, 2)]
                self.assertLess(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b]), 1e-6)
                self.assertFalse(any(c.dist < -0.00005 for c in d.contact))

    def test_unchanged_qpos_does_not_bypass_broken_connection(self):
        m = mujoco.MjModel.from_xml_path(str(MODEL_DIR/'scene.xml'))
        d = mujoco.MjData(m)
        reset_to_spawn(m, d)
        initial = d.qpos.copy()
        m.site_pos[m.site(D3_ROD_SITE).id] *= (1-1e-5/.150)
        mujoco.mj_forward(m, d)
        solver = ArmClosure(m, d)
        gap = lambda: np.linalg.norm(d.site_xpos[solver.sites[0]]-d.site_xpos[solver.sites[1]])
        self.assertGreater(gap(), 5e-6)
        self.assertTrue(solver.apply(d))
        self.assertLess(gap(), 1e-6)
        fixed = np.ones(m.nq, dtype=bool)
        fixed[solver.qadr:solver.qadr+4] = False
        fixed[solver.badr] = False
        np.testing.assert_array_equal(d.qpos[fixed], initial[fixed])
        self.assertEqual(d.time, 0)


if __name__ == '__main__':
    unittest.main()
