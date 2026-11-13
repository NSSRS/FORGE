"""Offline calibration policy, cancellation, persistence and full-model evidence."""
from concurrent.futures import Future
from copy import deepcopy
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import source_paths
from motors.arm_calibration import (CalibrationMap, CalibrationRecord, SimulatedCalibrationPort,
    CalibrationGroupHoldPort, DRIVES, snapshot, validate_start, constraint_limits)
from teleop.arm_calibration import CalibrationKeys, calibration_panel
from teleop.arm_keyboard import make_window

ROOT = Path(__file__).resolve().parents[1]


def identity_mapping():
    return CalibrationMap(dict(version=1, equipment_revision='test-rig',
        joints={str(j):dict(model_joint=name,model_per_raw=1.,model_intercept_deg=0.,
                           raw_branch_deg=[-360.,360.]) for j,name in enumerate(DRIVES,1)},
        simulation_model_deg={str(j):0. for j in range(1,7)}))


class CalibrationPolicyTests(unittest.TestCase):
    def setUp(self):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        self.mapping=identity_mapping()
        self.record=CalibrationRecord(Path(directory.name)/'run.json',self.mapping,simulated=True)
        self.addCleanup(self.record.close)
        self.port=SimulatedCalibrationPort(window_degrees=None);self.addCleanup(self.port.close)
        self.executor=Mock()
        self.keys=CalibrationKeys(self.port,2,mapping=self.mapping,record=self.record,
            scene=ROOT/'src/robot_description/mujoco/robot.xml',scene_fingerprint='scene',
            executor=self.executor,options=dict(backoff_deg=2.))

    def path(self, endpoint=3.):
        return dict(joint=1,scene_fingerprint='scene',mapping_fingerprint=self.mapping.fingerprint,
                    start_raw_deg=snapshot(self.port.positions()),
                    ends=dict(lower=dict(raw_deg=-endpoint),upper=dict(raw_deg=endpoint)))

    def ready(self):
        self.keys.press('h');self.keys.path=self.path()

    def test_map_inverse_branch_and_nonfinite_rejection(self):
        document=deepcopy(self.mapping.document)
        document['joints']['1'].update(model_per_raw=-2.,model_intercept_deg=13.)
        mapping=CalibrationMap(document)
        for raw in (-188.79,-174.67,192.85):
            self.assertAlmostEqual(mapping.raw(1,mapping.model(1,raw)),raw)
        for raw in (math.nan,math.inf,361):
            with self.assertRaises(ValueError):mapping.model(1,raw)
        document['joints']['1']['model_per_raw']=0
        with self.assertRaises(ValueError):CalibrationMap(document)

    def test_complete_fresh_pose_and_changed_other_joint_required(self):
        rows=self.port.positions()
        for bad in (rows[:-1],[replace(r,age_ms=100) if r.joint==3 else r for r in rows]):
            with self.assertRaises(ValueError):snapshot(bad)
        current=snapshot(rows);plan=self.path();current['4']=.5
        with self.assertRaisesRegex(ValueError,'A4 changed'):validate_start(plan,current)

    def test_no_motion_before_hold_calculation_and_explicit_start(self):
        self.keys.press(']');self.assertFalse(self.port.moving)
        self.ready();self.assertFalse(self.port.moving)
        self.keys.press('right');self.assertFalse(self.port.moving)
        self.keys.path=self.path();self.keys.press(']')
        self.assertTrue(self.port.moving)
        self.assertAlmostEqual(math.degrees(self.port.target),1.)
        self.keys.press(' ',repeat=False)
        self.assertFalse(self.port.moving or self.port.armed)
        self.assertTrue(self.port.holding)
        self.assertIsNone(self.keys.run)
        self.keys.press(']',repeat=True);self.assertFalse(self.port.moving)

    def test_cancelled_planning_result_cannot_start_or_replace_path(self):
        self.ready()
        future=Future();future.set_running_or_notify_cancel()
        self.keys.future=future
        self.keys.press(' ')
        future.set_result(self.path())
        self.keys.tick()
        self.assertFalse(self.port.moving)
        self.assertIsNone(self.keys.future)

    def test_changed_plan_snapshot_does_not_arm(self):
        self.ready();self.port.values[2]=math.radians(1.)
        self.keys.press(']')
        self.assertFalse(self.port.moving or self.port.armed)
        self.assertIn('changed',self.keys.message)

    def test_waypoint_completion_and_no_automatic_reverse(self):
        self.ready();self.keys.press(']')
        targets=[]
        for _ in range(5):
            if self.keys.run is None:break
            targets.append(math.degrees(self.port.target))
            self.port.values[0]=self.port.target
            self.keys.tick()
        self.assertEqual(len(targets),3)
        for actual,expected in zip(targets,[1.,2.,3.]):self.assertAlmostEqual(actual,expected)
        self.assertFalse(self.port.moving or self.port.armed)
        self.assertTrue(self.port.holding)
        self.assertIn('boundary reached',self.keys.message)

    def test_drift_timeout_and_disable_cancel_without_resume(self):
        for kind in ('drift','timeout','disable'):
            with self.subTest(kind=kind):
                self.port.stop();self.port.values=[0.]*6;self.ready();self.keys.press(']')
                if kind=='drift':self.port.values[3]=math.radians(1.)
                elif kind=='timeout':self.keys.run['segment_deadline']=0
                else:self.keys.press('d')
                self.keys.tick()
                self.assertFalse(self.port.moving or self.port.armed)
                self.assertIsNone(self.keys.run)

    def test_constraints_require_disabled_feedback_and_keep_raw_branch(self):
        self.ready();self.keys.constraint_reason='tool bracket'
        with self.assertRaisesRegex(ValueError,'press D'):self.keys.record_constraint()
        self.keys.press('d');self.port.values[0]=math.radians(-188.79)
        self.keys.record_constraint()
        item=self.record.data['constraints'][0]
        self.assertAlmostEqual(item['raw_deg'],-188.79)
        lo,hi=constraint_limits(self.mapping,1,[item],'scene')
        self.assertAlmostEqual(lo,-186.79);self.assertEqual(hi,360.)
        with self.assertRaisesRegex(ValueError,'different model'):constraint_limits(self.mapping,1,[item],'new scene')
        changed=deepcopy(self.mapping.document);changed['equipment_revision']='new gear'
        with self.assertRaises(ValueError):constraint_limits(CalibrationMap(changed),1,[item],'scene')
        self.assertFalse(self.port.moving or self.port.armed)

    def test_record_refuses_overwrite_and_disk_failure_is_visible(self):
        with self.assertRaises(FileExistsError):CalibrationRecord(self.record.path,self.mapping,simulated=True)
        with patch('motors.arm_calibration._write',side_effect=OSError('disk full')):
            self.record.save()
            self.record.pending[0].exception(timeout=2)
            with self.assertRaisesRegex(OSError,'disk full'):self.record.check()
            self.record.pending.clear()

    def test_space_stops_even_when_constraint_text_has_focus(self):
        try:from PyQt5 import QtCore,QtTest,QtWidgets
        except ImportError:self.skipTest('requires PyQt5')
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window=make_window(self.port,'SIMULATION',speed_deg_s=2,
            keys_factory=lambda *a,**kw:self.keys,panel_factory=calibration_panel)
        window.timer.stop();window.show();app.processEvents()
        window.constraint_reason_input.setFocus();app.processEvents()
        self.ready();self.keys.press(']');self.assertTrue(self.port.moving)
        QtTest.QTest.keyClick(window.constraint_reason_input,QtCore.Qt.Key_Space)
        self.assertFalse(self.port.moving or self.port.armed)
        self.assertIsNone(self.keys.run)
        window.close();app.processEvents()

    def test_manual_window_has_no_capture_workflow(self):
        try:from PyQt5 import QtWidgets
        except ImportError:self.skipTest('requires PyQt5')
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window=make_window(self.port,'SIMULATION',speed_deg_s=10)
        window.timer.stop()
        self.assertFalse(hasattr(window,'capture_button'))
        self.assertFalse(any('Position recording'==g.title() for g in window.findChildren(QtWidgets.QGroupBox)))
        window.close();app.processEvents()


class CalibrationFirmwareAdapterTests(unittest.TestCase):
    import test_damiao_bench as routed
    setUpClass=classmethod(routed.RoutedRoundTrip.setUpClass.__func__)
    tearDownClass=classmethod(routed.RoutedRoundTrip.tearDownClass.__func__)
    setUp=routed.RoutedRoundTrip.setUp
    tearDown=routed.RoutedRoundTrip.tearDown

    def test_bounded_waypoint_keeps_group_holding_and_pause_disarms(self):
        from motors import arm_position as pos, arm_bench as bench
        plan=json.loads((ROOT/'src/robot_description/config/arm_rig_a1_a6.json').read_text())
        plan.update(fixture_confirmed=True,supported_confirmed=True)
        with patch.object(pos,'time',bench.time), CalibrationGroupHoldPort(plan,bus=self.bus,window_degrees=None,smooth_jog=True) as port:
            port.hold_all(math.radians(2));port.select(2);port.arm()
            before=port.hold_targets.copy()
            current=snapshot(port.cached_positions(),enabled=True)['2']
            with self.assertRaises(ValueError):port.start_waypoint(math.radians(current+2),math.radians(2))
            port.start_waypoint(math.radians(current+.8),math.radians(2))
            for _ in range(80):self.lib.fake_advance(20);port.poll()
            port.pause()
            self.assertTrue(port.holding)
            self.assertFalse(port.moving or port.armed)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[1]*6)
            for i in (0,2,3,4,5):self.assertEqual(port.hold_targets[i],before[i])
            self.assertAlmostEqual(math.degrees(self.bus.state(1).position),current+.8,delta=.1)
            port.stop()
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)


@unittest.skipUnless(all(importlib.util.find_spec(n) for n in ('mujoco','numpy','scipy','fcl')), 'model dependencies required')
class ModelCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from simulation.arm_calibration import ModelCalibrationPlanner
        cls.mapping=json.loads((ROOT/'src/robot_description/config/arm_calibration_candidate.json').read_text())
        cls.planner=ModelCalibrationPlanner(ROOT/'src/robot_description/mujoco/robot.xml',cls.mapping,extent_deg=10)
        cls.raw=CalibrationMap(cls.mapping).simulation_pose()

    def test_model_bounds_and_ignored_adjacent_stop_are_checked(self):
        with patch('mujoco.mj_step',side_effect=AssertionError('no physics stepping')):
            plan=self.planner.plan(self.raw,3)
        self.assertEqual(plan['start_raw_deg'],self.raw)
        self.assertFalse(plan['hardware_limits'])
        self.assertEqual(plan['ends']['upper']['reason'],'adjacent mechanical stop')
        self.assertLess(plan['ends']['upper']['raw_deg'],151.39)
        self.assertEqual(self.planner.study.data.time,0.)
        for other in ('1','2','4','5','6'):
            address=self.planner.study.addresses[int(other)-1]
            self.assertAlmostEqual(math.degrees(self.planner.study.home[address]),plan['start_model_deg'][other])

    def test_model_initial_overlap_is_rejected_not_sent_as_home(self):
        raw=dict(self.raw);raw['3']=self.planner.mapping.raw(3,-25.21014298575622)
        with self.assertRaises(ValueError):self.planner.plan(raw,3)

    def test_search_horizon_not_claimed_as_physical_limit(self):
        plan=self.planner.plan(self.raw,1)
        self.assertIn('search horizon',plan['ends']['lower']['reason'])
        self.assertAlmostEqual(plan['ends']['lower']['raw_deg'],self.raw['1']-8)


if __name__=='__main__':unittest.main()
