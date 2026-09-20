from pathlib import Path
import sys
import tempfile
import unittest

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from merge_mjcf import body_mesh, merge_model


class MergeTests(unittest.TestCase):
    def test_transformed_meshes_keep_body_coordinates_and_joint_tree(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "assets").mkdir()
            (folder / "assets/old.obj").write_text(
                "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
                "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n")
            (folder / "scene.xml").write_text('<mujoco><include file="robot.xml"/></mujoco>')
            (folder / "robot.xml").write_text('''<mujoco>
              <compiler angle="radian" meshdir="assets"/>
              <asset><mesh name="part" file="old.obj" scale=".1 .2 .3"/></asset>
              <worldbody><body name="base" pos="1 2 3" euler=".2 .4 .6">
                <inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/>
                <geom type="mesh" mesh="part" group="2" pos=".5 0 0" euler="0 .3 0"/>
                <geom type="mesh" mesh="part" group="2" pos="0 .6 0" euler=".4 0 .2"/>
                <body name="wheel" pos=".4 .5 .6">
                  <joint name="wheel_joint" type="hinge" axis="0 1 0"/>
                  <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
                  <geom type="mesh" mesh="part" group="2" pos="0 0 .7"/>
                </body>
              </body></worldbody></mujoco>''')
            before = mujoco.MjModel.from_xml_path(str(folder / "scene.xml"))
            original, _ = body_mesh(before, [0, 1])
            report = merge_model(folder, target_faces=0)
            after = mujoco.MjModel.from_xml_path(str(folder / "scene.xml"))
            merged, _ = body_mesh(after, [0])
            # Every transformed source vertex must survive unchanged, regardless of reindexing.
            distances = np.linalg.norm(original[:, None, :] - merged[None, :, :], axis=2)
            self.assertLess(distances.min(axis=1).max(), 1e-6)
            self.assertLess(distances.min(axis=0).max(), 1e-6)
            self.assertEqual(after.njnt, 1)
            self.assertEqual(after.ngeom, 2)
            self.assertEqual(len(list((folder / "assets").iterdir())), 2)
            self.assertFalse((folder / "assets/old.obj").exists())
            self.assertEqual(sum(r['original_faces'] for r in report),
                             sum(r['merged_faces'] for r in report))

    def test_preserve_collision_leaves_gap_between_parts(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "assets").mkdir()
            (folder / "assets/part.obj").write_text(
                "v 0 0 0\nv .1 0 0\nv 0 .1 0\nv 0 0 .1\n"
                "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n")
            (folder / "scene.xml").write_text('<mujoco><include file="robot.xml"/></mujoco>')
            (folder / "robot.xml").write_text('''<mujoco>
              <compiler meshdir="assets"/><asset><mesh name="part" file="part.obj"/></asset>
              <worldbody><body name="base"><freejoint/>
              <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
              <geom name="left" type="mesh" mesh="part" pos="-.3 0 0" group="2"/>
              <geom name="right" type="mesh" mesh="part" pos=".3 0 0" group="2"/>
              </body><geom name="probe" type="sphere" pos="0 .02 .02" size=".01"/>
              </worldbody></mujoco>''')
            merge_model(folder, 0, preserve_collision=True)
            m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
            d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            self.assertEqual(d.ncon,0)
            for name in ('left','right'):
                self.assertEqual(m.geom(name).group[0],3)
                self.assertEqual(m.geom(name).contype[0],1)
            visual=np.flatnonzero(m.geom_group==2)
            self.assertEqual(len(visual),1)
            self.assertEqual(m.geom_contype[visual[0]],0)
            self.assertEqual(m.geom_conaffinity[visual[0]],0)
            self.assertTrue((folder/'assets/part.obj').exists())



if __name__ == "__main__":
    unittest.main()
