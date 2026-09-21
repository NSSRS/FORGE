import unittest
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from terrain import generate, MODEL
from forge_sim.magnetic import MagneticAttraction
from forge_sim.spawn_fit import reset_to_spawn

class TerrainTests(unittest.TestCase):
    def test_presets_preserve_robot_and_spawn_without_overlap(self):
        original=(MODEL/"robot.xml").read_bytes()
        for preset in ("curved","bumpy"):
            path=generate(preset)
            m=mujoco.MjModel.from_xml_path(str(path))
            d=mujoco.MjData(m);reset_to_spawn(m,d)
            a=MagneticAttraction(m)
            self.assertEqual(m.njnt,12)
            self.assertEqual(m.nu,4)
            self.assertEqual(len(a.surfaces),100 if preset=="curved" else 110)
            self.assertEqual(d.ncon,0)
            for _,_,g in a.pairs:
                gap=min(mujoco.mj_geomDistance(m,d,g,int(s),1,None) for s in a.surfaces)
                self.assertGreater(gap,0)
                self.assertLess(gap,a.cutoff)
            # Collision surface actually departs from its tangent, not a flat proxy.
            g0=m.geom("steel_panel_000").id;gc=m.geom("steel_panel_050").id
            self.assertGreater(abs(d.geom_xpos[g0,1]-d.geom_xpos[gc,1]),.5)
        self.assertEqual((MODEL/"robot.xml").read_bytes(),original)

    def test_seams_do_not_multiply_magnetic_force(self):
        xml='''<mujoco><custom><text name="magnetic_bodies" data="wheel"/>
        <text name="magnetic_surface" data="a b"/>
        <numeric name="magnetic_parameters" data="100 .01 .002"/></custom>
        <worldbody><geom name="a" type="box" size=".01 1 1"/>
        <geom name="b" type="box" size=".01 1 1"/>
        <body name="wheel" pos=".0601 0 0"><freejoint/>
        <geom type="sphere" size=".05" mass="1"/></body></worldbody></mujoco>'''
        m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);reset_to_spawn(m,d)
        a=MagneticAttraction(m);f=a.compute(d).copy()
        a.surfaces=a.surfaces[:1]
        np.testing.assert_allclose(a.compute(d),f)
        self.assertLessEqual(abs(f[0]),100)

    def test_bad_parameters_rejected(self):
        for params in ({"angle":0},{"length":float('nan')},{"bump_height":.2}):
            with self.assertRaises(ValueError):generate(**params)

if __name__=="__main__":unittest.main()


class MenuTests(unittest.TestCase):
    def test_mouse_controls_and_bounds(self):
        from forge_sim.terrain_menu import TerrainMenu
        menu=TerrainMenu(preset="flat",length=10.,angle=30.,width=4.,
                         bump_diameter=.1,bump_height=.005,tilt=0.,inward=False)
        left,width,row,top=menu.layout(1280,900)
        menu.click(left+width-30,top-55-row/2,1280,900)
        self.assertEqual(menu.values['preset'],'curved')
        menu.selected=6
        for _ in range(100):menu.adjust(1)
        self.assertEqual(menu.values['tilt'],90)
        menu.click(left+30,top-55-(len(menu.fields)+.5)*row,1280,900)
        self.assertTrue(menu.pending)


class SpawnFitTests(unittest.TestCase):
    def test_tight_arc_and_reset(self):
        path=generate("curved",length=3,angle=60)
        m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m)
        reset_to_spawn(m,d);a=MagneticAttraction(m)
        fitted=d.qpos.copy()
        self.assertGreater(np.linalg.norm(fitted-m.qpos0),.01)
        for _,_,g in a.pairs:
            gap=min(mujoco.mj_geomDistance(m,d,g,int(s),1,None) for s in a.surfaces)
            self.assertGreaterEqual(gap,-.00005)
            self.assertLess(gap,.004)
        # Check the full robot against the steel, not only wheels.
        for g in np.flatnonzero((m.geom_bodyid!=0) & ((m.geom_contype!=0) | (m.geom_conaffinity!=0))):
            gap=min(mujoco.mj_geomDistance(m,d,int(g),int(s),1,None) for s in a.surfaces)
            self.assertGreaterEqual(gap,-.00005)
        for _ in range(500):a.step(d)
        self.assertTrue(np.isfinite(d.qpos).all())
        self.assertFalse(any(w.number for w in d.warning))
        reset_to_spawn(m,d)
        np.testing.assert_allclose(d.qpos,fitted)
        np.testing.assert_allclose(d.qvel,0)
        np.testing.assert_allclose(d.ctrl,0)
        self.assertEqual(d.time,0)
