import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from forge_sim.onshape_compat import add_joint_compat
from onshape_to_robot.exporter_mujoco import ExporterMuJoCo
from onshape_to_robot.robot import Joint


class ContinuousJointTests(unittest.TestCase):
    def test_continuous_is_unlimited_hinge_without_mutating_source(self):
        exporter = ExporterMuJoCo()
        exporter.config = SimpleNamespace(round=lambda value: value)
        exporter.xml = ""
        joint = SimpleNamespace(
            parent=SimpleNamespace(name="base"), child=SimpleNamespace(name="wheel"),
            name="wheel", axis=np.array([0, 0, 1]), joint_type=Joint.CONTINUOUS,
            limits=(-1, 1), properties={"limits": [-2, 2]},
        )
        add_joint_compat(ExporterMuJoCo.add_joint, exporter, joint)
        element = ET.fromstring("<mujoco>" + exporter.xml + "</mujoco>").find("joint")
        self.assertEqual(element.get("type"), "hinge")
        self.assertIsNone(element.get("range"))
        self.assertEqual(joint.joint_type, Joint.CONTINUOUS)
        self.assertEqual(joint.limits, (-1, 1))


if __name__ == "__main__":
    unittest.main()
