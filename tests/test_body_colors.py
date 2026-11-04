"""Visual identity must not change the full robot's physics or closure."""
import copy
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from forge_sim.body_colors import BODY_COLORS, apply_body_colors, rgba

ROOT = Path(__file__).resolve().parents[1]


class BodyColorTests(unittest.TestCase):
    def test_palette_covers_all_visual_bodies_and_is_stable(self):
        for filename in ('robot.xml', 'arm_preview.xml'):
            root = ET.parse(ROOT / 'model' / filename).getroot()
            bodies = [b for b in root.findall('.//worldbody//body')
                      if any(g.get('group') == '2' for g in b.findall('geom'))]
            self.assertEqual({b.get('name') for b in bodies}, set(BODY_COLORS))
            self.assertEqual(len({c for _, c in BODY_COLORS.values()}), len(BODY_COLORS))
            for b in bodies:
                for g in b.findall('geom'):
                    if g.get('group') == '2':
                        self.assertEqual(g.get('rgba'), rgba(BODY_COLORS[b.get('name')][1]))
                        if b.get('name').startswith('linkage_wheel_'):
                            mat = root.find(f"asset/material[@name='{g.get('material')}']")
                            self.assertEqual(mat.get('rgba'), g.get('rgba'))
                            self.assertEqual(mat.get('texture'), 'wheel_checker')
            before = ET.tostring(root)
            apply_body_colors(root)
            # Compare after normalizing whitespace introduced by XML indentation.
            ET.indent(root)
            self.assertEqual(ET.tostring(root), before)

    def test_only_visuals_and_visual_materials_change(self):
        root = ET.parse(ROOT / 'model/robot.xml').getroot()
        def physical_xml(tree):
            tree = copy.deepcopy(tree)
            asset = tree.find('asset')
            for e in list(asset):
                if e.tag in ('texture', 'material'):
                    asset.remove(e)
            for geom in tree.findall('.//geom'):
                if geom.get('group') == '2':
                    geom.attrib.pop('rgba', None)
                    geom.attrib.pop('material', None)
            return ET.tostring(tree)
        before = physical_xml(root)
        apply_body_colors(root)
        self.assertEqual(before, physical_xml(root))

    def test_unknown_visual_body_rejected_before_mutation(self):
        root = ET.fromstring('<mujoco><worldbody><body name="new_link"><geom group="2"/></body></worldbody></mujoco>')
        before = ET.tostring(root)
        with self.assertRaisesRegex(ValueError, 'new_link'):
            apply_body_colors(root)
        self.assertEqual(before, ET.tostring(root))


if __name__ == '__main__':
    unittest.main()
