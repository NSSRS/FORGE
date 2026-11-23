"""A3 endpoint uncertainty, separating contact and explicit retreat; offline only."""
import source_paths
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from motors.arm_calibration import CalibrationMap, RawAngleLimitError
from controls.controller.endpoint_recovery import A3Retreat, prepare_retreat, retreat_path, add_retreat_timing
from controls.controller.preparation import preparation_timing
from controls.controller.weld_plan import Settings, JointPath, WeldGeometry, move_path, build_plan
from controls.controller.tracking import TrackingSettings
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from simulation.weld_preview import PlanPlayback
from test_weld_hardware import Clock, simple_plan

ROOT = Path(__file__).resolve().parents[1]
START = np.array([-136.7479277125, -102.8696976556, 151.5013548858,
                  175.4784334388, -35.7033112446, 155.8072154216])


def candidate_mapping():
    doc = json.loads((ROOT / 'src/robot_description/config/arm_calibration_measured.json').read_text())
    a5 = json.loads((ROOT / 'docs/measurements/arm-a5-alignment-2026-11-16.json').read_text())['recorded_candidate']
    doc['joints']['5'].update(model_intercept_deg=a5['model_intercept_deg'],
                            raw_branch_deg=a5['raw_command_bounds_deg'])
    return CalibrationMap(doc)


def endpoint_plan(mapping, start=START):
    policy = A3Retreat(tuple(start), mapping.fingerprint)
    path = retreat_path(policy)
    goal = policy.goal
    standby = goal.copy(); standby[0] += .3
    weld_start = standby.copy(); weld_start[0] += .3
    apex = weld_start.copy(); apex[0] += .1
    home = move_path(goal, standby, Settings())
    plan = simple_plan(chassis=False)
    plan.update(current_raw_deg=list(start), initial_raw_deg=standby.tolist(),
                home=home.document(), approach=move_path(standby, weld_start, Settings()).document(),
                weld=JointPath([0,.5,1], [weld_start,apex,weld_start], 'periodic_cubic').document(),
                recovery=dict(version=1,clearance_mm=2.,waypoint_times_s=[home.duration]),
                endpoint_recovery=dict(policy=policy.document(), path=path.document(),
                                       speed_deg_s=1., acceleration_deg_s2=5.))
    return plan


class EndpointControlTests(unittest.TestCase):
    def setUp(self):
        self.mapping = candidate_mapping()
        self.clock = Clock()
        self.arm = SimulatedArm(START, clock=self.clock)
        def strict(q, **kwargs):
            self.mapping.pose(dict(zip('123456', map(float,q))))
            if q[2] > 150.76:
                raise ValueError('Adjacent mechanical stop')
        def endpoint(q, spec):
            policy = A3Retreat.from_document(spec)
            policy.check_mapping(self.mapping)
            policy.check_raw(q)
        self.control = WeldController(self.arm, clock=self.clock, pose_check=strict,
            endpoint_check=endpoint, tracking_settings=TrackingSettings())
        self.plan = endpoint_plan(self.mapping)
        self.control.select(True, False)
        self.control.load_plan(self.plan)

    def tick(self, count=1):
        for _ in range(count):
            self.clock.advance(); self.control.tick()

    def test_explicit_start_monotone_retreat_and_strict_handoff(self):
        self.assertFalse(self.arm.enabled)
        self.control.go_initial()
        self.tick(5)
        self.assertEqual(self.control.state, 'verifying_hold')
        self.assertEqual(self.arm.commands, [])
        for _ in range(1000):
            self.tick()
            if self.control.state in ('ready','fault','paused'):
                break
        self.assertEqual(self.control.state, 'ready', self.control.message)
        rows = [r for r in self.control.trace if r['stage']=='endpoint_recovery']
        commands = np.array([r['command_raw_deg'] for r in rows])
        self.assertGreater(len(rows), 10)
        self.assertTrue(np.all(np.diff(commands[:,2]) <= 1e-9))
        np.testing.assert_allclose(commands[:,[0,1,3,4,5]], np.tile(START[[0,1,3,4,5]],(len(rows),1)))
        self.assertTrue(all(max(r['speed_limit_deg_s'])<=1 and r['tracker'] is None for r in rows))
        velocity = np.diff(commands[:,2])/.02
        self.assertLessEqual(max(abs(velocity)),1)
        self.assertLessEqual(max(abs(np.diff(velocity)))/.02,5)
        self.assertFalse(self.control.endpoint_active)
        self.assertIsNone(self.arm.endpoint_policy)
        with self.assertRaises(RawAngleLimitError):
            self.control.check_pose(START, recovery=True)
        self.assertTrue(any(e['event']=='endpoint_recovery_completed' for e in self.control.events))

    def test_preload_drift_does_not_command_back_to_old_start(self):
        self.arm.q[2] -= .1
        self.control.go_initial()
        self.arm.q[2] -= .02
        self.arm.target = self.arm.q.copy()
        self.tick(30)
        self.assertEqual(self.control.state,'endpoint_recovery',self.control.message)
        self.assertLessEqual(max(q[2] for q in self.arm.commands), START[2]-.12+1e-8)

    def test_pause_holds_and_never_resumes_without_new_plan(self):
        self.control.go_initial();self.tick(40)
        self.control.pause()
        q=self.arm.target.copy();n=len(self.arm.commands)
        self.tick(100)
        self.assertEqual(self.control.state,'paused')
        self.assertIsNone(self.control.plan)
        np.testing.assert_allclose(self.arm.target,q)
        self.assertEqual(len(self.arm.commands),n)
        with self.assertRaises(ValueError):self.control.go_initial()
        plan=endpoint_plan(self.mapping,self.arm.q)
        self.control.load_plan(plan);self.control.go_initial();self.tick(30)
        self.assertEqual(self.control.state,'endpoint_recovery',self.control.message)

    def test_large_drift_before_enable_and_stale_during_retreat_are_blocked(self):
        self.arm.q[2]=151.641
        with self.assertRaises(ValueError):self.control.go_initial()
        self.assertFalse(self.arm.enabled)
        self.arm.q=START.copy();self.control.go_initial();self.tick(30)
        read=self.arm.read
        def stale():
            value=read();value.age_ms[2]=80;return value
        self.arm.read=stale
        before=len(self.arm.commands);self.tick()
        self.assertEqual(self.control.state,'fault')
        self.assertFalse(self.arm.enabled)
        self.assertEqual(len(self.arm.commands),before)

    def test_outward_measurement_after_progress_faults(self):
        self.control.go_initial();self.tick(100)
        self.arm.q[2] += .3;self.arm.target=self.arm.q.copy()
        self.tick()
        self.assertEqual(self.control.state,'fault')
        self.assertIn('outward',self.control.message)

    def test_unchecked_or_outward_paths_and_high_caps_are_rejected(self):
        for mutation in (
            lambda p:p['endpoint_recovery'].update(speed_deg_s=2),
            lambda p:p['endpoint_recovery'].update(acceleration_deg_s2=6),
            lambda p:p['endpoint_recovery']['path']['raw_deg'][1].__setitem__(2,152),
            lambda p:p['endpoint_recovery']['path']['raw_deg'][1].__setitem__(0,START[0]+.1),
        ):
            p=deepcopy(self.plan);mutation(p)
            with self.assertRaises(ValueError):self.control.load_plan(p)
            self.assertFalse(self.arm.enabled)

    def test_measured_start_policy_expires_after_the_separate_a3_retreat(self):
        from controls.controller.measured_start import MeasuredStart

        self.plan['mapping_fingerprint'] = self.mapping.fingerprint
        self.plan['measured_start'] = MeasuredStart(tuple(START), self.mapping.fingerprint).document()
        self.control.recovery_check = lambda raw, document, **kwargs: self.control.pose_check(raw)
        self.control.load_plan(self.plan)
        self.control.go_initial()
        for _ in range(1000):
            self.tick()
            if self.control.state in ('ready', 'fault', 'paused'):
                break
        self.assertEqual(self.control.state, 'ready', self.control.message)
        self.assertFalse(self.control.endpoint_active)
        self.assertIsNone(self.control.measured_start)
        self.assertIsNone(self.arm.measured_start)
        events = [row['event'] for row in self.control.events]
        self.assertLess(events.index('endpoint_recovery_completed'),
                        events.index('measured_start_recovery_completed'))

    def test_preview_includes_backoff_and_settling_before_home(self):
        play=PlanPlayback(self.plan)
        self.assertEqual(play.sample(0)[0],'endpoint_recovery')
        self.assertAlmostEqual(play.sample(0)[1][2],START[2])
        t=JointPath.from_document(self.plan['endpoint_recovery']['path']).duration
        self.assertEqual(play.sample(t+.21)[0],'endpoint_recovery')
        self.assertEqual(play.sample(t+.41)[0],'home')

    def test_timing_adds_retreat_once_to_aliased_search_candidates(self):
        plan = deepcopy(self.plan)
        timing = preparation_timing(JointPath.from_document(plan['home']),
            JointPath.from_document(plan['approach']),plan['recovery']['waypoint_times_s'],
            np.full(6,5.),np.full(6,30.),5.)
        original = deepcopy(timing)
        plan['preparation_timing'] = timing
        plan['preparation_search'] = dict(original_nominal_total_s=timing['nominal_total_s'],
            candidates=[dict(checked=True,timing=timing),dict(checked=True,timing=deepcopy(timing)),
                        dict(checked=False,reason='blocked')])
        add_retreat_timing(plan,plan['endpoint_recovery'])
        extra=JointPath.from_document(plan['endpoint_recovery']['path']).duration+.2
        self.assertEqual(timing,original)
        self.assertAlmostEqual(plan['preparation_search']['original_nominal_total_s'],original['nominal_total_s']+extra)
        for value in [plan['preparation_timing']]+[r['timing'] for r in plan['preparation_search']['candidates'] if r['checked']]:
            self.assertAlmostEqual(value['nominal_total_s'],original['nominal_total_s']+extra)
            self.assertAlmostEqual(value['nominal_total_s'],sum(s['nominal_motion_s'] for s in value['stages'])+value['minimum_settling_s'])


class EndpointGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mapping=candidate_mapping()
        cls.geometry=WeldGeometry(cls.mapping)

    def test_recorded_pose_separates_without_widening_mapping(self):
        g=self.geometry
        before=deepcopy(g.mapping.document)
        result=prepare_retreat(g,START,Settings())
        samples=result['separation_samples']
        self.assertAlmostEqual(samples[0]['signed_clearance_mm'],-.075812,places=4)
        self.assertGreater(samples[-1]['signed_clearance_mm'],.37)
        self.assertTrue(np.all(np.diff([r['signed_clearance_mm'] for r in samples])>=-1e-6))
        self.assertEqual(g.mapping.document,before)
        with self.assertRaises(RawAngleLimitError):g.check(START)
        self.assertIsNone(prepare_retreat(g,A3Retreat.from_document(result['policy']).goal,Settings()))

    def test_tolerance_edge_and_unreviewed_axes_are_not_widened(self):
        edge=START.copy();edge[2]=151.64
        result=prepare_retreat(self.geometry,edge,Settings())
        self.assertIsNotNone(result)
        for axis,value in ((2,151.641),(0,START[0]+360),(3,444),(4,115),(2,float('nan'))):
            bad=START.copy();bad[axis]=value
            with self.subTest(axis=axis,value=value),self.assertRaises(ValueError):
                prepare_retreat(self.geometry,bad,Settings())

    def test_unknown_collision_linkage_and_deepening_contact_remain_blocked(self):
        spec=A3Retreat(tuple(START),self.mapping.fingerprint).document()
        with patch.object(self.geometry,'inspect_contacts',return_value=[dict(blocked=True,basis='convex hull',distance_mm=-1.,geom_names=['other','obstacle'])]):
            with self.assertRaisesRegex(ValueError,'Arm/environment'):
                self.geometry.check_endpoint(START,spec)
        with patch.object(self.geometry.closure,'apply',return_value=False):
            with self.assertRaisesRegex(ValueError,'linkage'):
                self.geometry.check_endpoint(START,spec)
        with patch.object(self.geometry.stops,'measure',return_value=[
                dict(geom_names=['collision_411','collision_463'],distance_mm=0.)]):
            with self.assertRaisesRegex(ValueError,'Adjacent mechanical stop'):
                self.geometry.check_endpoint(START,spec)
        with patch.object(self.geometry.stops.fcl,'collide',return_value=10000):
            with self.assertRaisesRegex(ValueError,'Incomplete'):
                self.geometry.check_endpoint(START,spec)
        for distances in ((-.08,-.09),(-.16,), (0.,0.)):
            with patch.object(self.geometry.stops,'signed_clearance',side_effect=distances):
                with self.assertRaises(ValueError):self.geometry.check_endpoint(START,spec)

    def test_full_plan_keeps_exception_ahead_of_optimized_staging(self):
        goal=json.loads((ROOT/'docs/measurements/arm-recovery-review-2026-11-16.json').read_text())['standby_goal_raw_deg']
        plan=build_plan(dict(mapping=self.mapping.document,current_raw_deg=START.tolist(),
            initial_raw_deg=goal,arm_enabled=True,chassis_enabled=False,
            settings=asdict(Settings(limit_profile='weld',cycles=1,optimize_preparation=True,approach_lift_m=.03))))
        endpoint=plan['endpoint_recovery']
        np.testing.assert_allclose(plan['current_raw_deg'],START)
        np.testing.assert_allclose(plan['home']['raw_deg'][0],A3Retreat.from_document(endpoint['policy']).goal)
        self.assertFalse(plan['hardware_qualified'])
        self.assertIn('endpoint_recovery_s',plan['preparation_timing'])
        clock=Clock();arm=SimulatedArm(START,clock=clock)
        control=WeldController(arm,clock=clock,pose_check=self.geometry.check,
            endpoint_check=self.geometry.check_endpoint,allow_optimized_preparation=True,
            tracking_settings=TrackingSettings())
        control.select(True,False);control.load_plan(plan);control.go_initial()
        for _ in range(4000):
            clock.advance();control.tick()
            if control.state in ('ready','fault','paused'):break
        self.assertEqual(control.state,'ready',control.message)
        self.assertFalse(control.endpoint_active)


if __name__=='__main__':unittest.main()
