"""Fixed A4 winding range, exact spline extrema, and restart invariance."""
import source_paths
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from motors.arm import CAP_MANUAL_TRAVEL
from motors.arm_bench import BenchSession
from motors.arm_calibration import CalibrationMap
from motors.arm_travel import a4_cable_bounds, check_a4
from controls.controller.weld_plan import JointPath
from test_weld_hardware import simple_plan, Clock
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm

ROOT = Path(__file__).resolve().parents[1]


class CableLimitTests(unittest.TestCase):
    def test_fixed_reference_allows_270_each_way_but_no_extra_revolution(self):
        bounds = a4_cable_bounds();center = sum(bounds)/2
        self.assertAlmostEqual(center, 173.1834352361486)
        self.assertAlmostEqual(bounds[1]-bounds[0], 540.)
        mapping = CalibrationMap(json.loads((ROOT/'src/robot_description/config/arm_calibration_measured.json').read_text()))
        for relative in (-269.9, 0, 269.9, 100, -100, 0):
            self.assertAlmostEqual(mapping.model(4,center+relative),-relative)
        for angle in (center+360,center-360,bounds[0]-.001,bounds[1]+.001):
            with self.assertRaises(ValueError):mapping.model(4,angle)
            with self.assertRaises(ValueError):check_a4(angle,bounds)
        # A custom mapping may not reopen cable travel.
        document = deepcopy(mapping.document);document['joints']['4']['raw_branch_deg']=[-720,720]
        with self.assertRaises(ValueError):CalibrationMap(document).model(4,center+360)

    def test_firmware_bounds_do_not_rebase_at_new_session_origins(self):
        motors = json.loads((ROOT/'src/robot_description/config/arm_rig_a1_a6.json').read_text())['motors']
        expected = tuple(map(math.radians,a4_cable_bounds()))
        for origin in (0.,2.,5.):
            session = BenchSession(SimpleNamespace(capabilities=CAP_MANUAL_TRAVEL),motors,{}, {},slots=(3,))
            with patch.object(session,'probe_disabled',return_value={3:SimpleNamespace(position=origin)}), patch.object(session,'prepare_motion'):
                session.prepare(manual_travel=True)
            self.assertEqual(session.bounds[3],expected)
        with patch.object(session,'probe_disabled',return_value={3:SimpleNamespace(position=expected[1]+.01)}), patch.object(session,'prepare_motion') as prepare:
            with self.assertRaisesRegex(RuntimeError,'fixed cable'):session.prepare(manual_travel=True)
            prepare.assert_not_called()

    def test_periodic_spline_overshoot_is_rejected_even_when_every_knot_fits(self):
        bounds=a4_cable_bounds();center=sum(bounds)/2
        q=np.zeros((5,6));q[:,3]=center+np.array([0,269,269,0,0])
        path=JointPath([0,1,2,3,4],q,'periodic_cubic')
        self.assertLess(q[:,3].max(),bounds[1])
        self.assertGreater(path.position_bounds()[1][3],bounds[1])
        checks=[(-math.inf,math.inf)]*6;checks[3]=bounds
        with self.assertRaisesRegex(ValueError,'A4'):path.check_raw_bounds(checks)
        q[:,3]=center+np.array([0,260,0,-260,0])
        JointPath([0,10,20,30,40],q).check_raw_bounds(checks)
        clock=Clock();control=WeldController(SimulatedArm(np.zeros(6),clock=clock),clock=clock,pose_check=lambda *a,**k:None)
        control.select(True,False)
        plan=simple_plan(chassis=False);plan['weld']=path.document()
        with self.assertRaisesRegex(ValueError,'A4'):control.load_plan(plan)


if __name__ == '__main__':unittest.main()
