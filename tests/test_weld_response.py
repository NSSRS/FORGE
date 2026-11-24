"""Recorded response holdouts and closed-loop fake plants; no motor transport."""

import source_paths
from copy import deepcopy
from collections import deque
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy import signal

from controls.controller.tracking import PositionTracker, TrackingSettings
from controls.controller.weld_plan import JointPath
from controls.controller.weld_response import WeldResponse
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock, FakeChassis


ROOT = Path(__file__).resolve().parents[1]
PROFILE = json.loads((ROOT / 'src/controls/config/weld_response_gun_20261124.json').read_text())
FIXTURE = json.loads((ROOT / 'tests/fixtures/weld_response_20261124.json').read_text())
V7_FIXTURE = json.loads((ROOT / 'tests/fixtures/weld_response_v7_20261124.json').read_text())
LEGACY_PROFILE = V7_FIXTURE['legacy_profile']
V8_PROFILE = json.loads((ROOT / 'tests/fixtures/weld_response_v8_profile_20261124.json').read_text())


class ResponseArm(SimulatedArm):
    """Second-order A1/A6 with input delay; ordinary fake preparation first."""

    factor = 1.0
    response_on = False
    profile = PROFILE
    disturbance = None
    response_started = 0.0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.history = deque(maxlen=200)
        self.initial = self.q.copy()

    def command(self, raw, speed):
        super().command(raw, speed)
        self.history.append((self.clock(), self.target.copy()))

    def poll(self):
        if not self.response_on:
            return super().poll()
        end = self.clock()
        while self.last < end - 1e-10:
            dt = min(0.001, end-self.last)
            self.last += dt
            for axis in range(6):
                parameters = self.profile['axes'].get(str(axis+1))
                if parameters is None:
                    v = (self.target[axis]-self.q[axis])/0.03
                    self.velocity[axis] = v if self.enabled else 0
                    self.q[axis] += self.velocity[axis]*dt
                    continue
                target = self.initial[axis]
                for at, raw in reversed(self.history):
                    if at <= self.last-parameters['delay_s']:
                        target = raw[axis]
                        break
                wn = parameters['natural_frequency_rad_s']*self.factor
                acceleration = wn*wn*(target-self.q[axis])-2*parameters['damping_ratio']*wn*self.velocity[axis]
                if self.enabled:
                    self.q[axis] += self.velocity[axis]*dt + 0.5*acceleration*dt*dt
                    self.velocity[axis] += acceleration*dt
                else:
                    self.velocity[axis] = 0

    def read(self):
        result = super().read()
        if self.response_on and self.disturbance is not None:
            result.raw_deg += self.disturbance(self.clock() - self.response_started)
        return result


def run_response(*, enabled=True, factor=1., intervals=None, duration=8.,
                 profile=PROFILE, plant_profile=None, disturbance=None):
    plan = deepcopy(FIXTURE['plan'])
    if enabled:
        response = WeldResponse(profile, JointPath.from_document(plan['weld']), plan)
        plan['weld_response'] = response.check_geometry(lambda *a, **k: None)
        plan['weld_entry'] = response.entry.document()
    clock = Clock()
    arm = ResponseArm(plan['current_raw_deg'], clock=clock)
    arm.factor = factor
    arm.profile = profile if plant_profile is None else plant_profile
    control = WeldController(
        arm, FakeChassis(), clock=clock, pose_check=lambda *a, **k: None,
        continuous_weld=True, tracking_settings=TrackingSettings(weld_a1_shaper_only=not enabled),
        weld_response_profile=profile if enabled else None,
    )
    control.select(True, False)
    control.load_plan(plan)
    control.go_initial()
    for _ in range(1500):
        clock.advance()
        control.tick()
        if control.state in ('ready','paused','fault'):
            break
    if control.state != 'ready':
        raise AssertionError(control.message)
    prep = deepcopy(list(control.trace))
    arm.initial = arm.q.copy()
    arm.history.clear()
    arm.history.append((clock(), arm.target.copy()))
    arm.response_on = True
    arm.response_started = clock()
    arm.disturbance = disturbance
    arm.velocity[:] = 0
    control.start_weld()
    start = clock()
    schedule = iter(intervals) if intervals is not None else None
    while clock()-start < duration:
        dt = next(schedule, 0.026) if schedule is not None else 0.026
        clock.advance(dt)
        control.tick()
        if control.state != 'welding':
            break
    return clock, arm, control, prep


def recorded_disturbance():
    """Unexplained v7 output replay; a sensitivity case, not a causal model."""
    times = np.array(V7_FIXTURE['elapsed_s'])
    commands = np.array(V7_FIXTURE['command_raw_deg'])
    measured = np.array(V7_FIXTURE['measured_raw_deg'])
    reference = np.array(V7_FIXTURE['reference_raw_deg'])
    start = np.array(FIXTURE['plan']['weld']['raw_deg'][0])
    grid = np.arange(0, times[-1] + .002, .002)
    residual = measured - reference
    for axis in (0, 5):
        p = V8_PROFILE['axes'][str(axis+1)]
        wn, damping = p['natural_frequency_rad_s'], p['damping_ratio']
        held = np.r_[start[axis], commands[:, axis]][np.searchsorted(times, grid, side='right')] - start[axis]
        delayed = np.interp(grid-p['delay_s'], grid, held, left=0)
        b, a, _ = signal.cont2discrete(([wn*wn], [1, 2*damping*wn, wn*wn]), .002)
        residual[:, axis] = measured[:, axis] - start[axis] - np.interp(
            times, grid, signal.lfilter(b.ravel(), a, delayed))
    return lambda at: np.array([np.interp(at, times, residual[:, axis]) for axis in range(6)])


class WeldResponseTests(unittest.TestCase):
    def test_restored_profile_reproduces_all_physical_v7_commands(self):
        self.assertEqual(PROFILE, LEGACY_PROFILE)
        self.assertEqual(TrackingSettings().document()['version'], 7)
        path = JointPath.from_document(V7_FIXTURE['plan']['weld'])
        response = WeldResponse(PROFILE, path, V7_FIXTURE['plan'])
        self.assertFalse(response.finite_startup)
        self.assertEqual(response.document()['startup_policy'], 'periodic_blend')
        np.testing.assert_allclose(response.residual_bound[[0, 5]], [.2, .05])
        for t, reference, measured, recorded in zip(
                V7_FIXTURE['elapsed_s'], V7_FIXTURE['reference_raw_deg'],
                V7_FIXTURE['measured_raw_deg'], V7_FIXTURE['command_raw_deg']):
            nominal = response.reference(t)[0]
            np.testing.assert_allclose(nominal, reference, atol=1e-10, rtol=0)
            target, _ = response.sample(t, nominal, measured)
            np.testing.assert_allclose(target, recorded, atol=1e-10, rtol=0)

    def test_model_predicts_unfitted_second_halves_of_two_recorded_runs(self):
        # Fit used only 1.2-5.5 seconds. These held-out ticks are independent
        # recorded output, not generated by the response implementation.
        for run in FIXTURE['runs']:
            times = np.array(run['elapsed_s'])
            grid = np.arange(0,times[-1]+0.002,0.002)
            use = times>5.5
            center = np.array(LEGACY_PROFILE['center_raw_deg'])[[0,5]]
            for column, axis in enumerate((1,6)):
                parameters = LEGACY_PROFILE['axes'][str(axis)]
                wn, damping = parameters['natural_frequency_rad_s'], parameters['damping_ratio']
                command = np.array(run['command_a1_a6_deg'])[:,column]-center[column]
                measured = np.array(run['measured_a1_a6_deg'])[:,column]-center[column]
                held = np.r_[0,command][np.searchsorted(times,grid,side='right')]
                delayed = np.interp(grid-parameters['delay_s'],grid,held,left=0)
                b,a,_=signal.cont2discrete(([wn*wn],[1,2*damping*wn,wn*wn]),0.002)
                predicted=np.interp(times,grid,signal.lfilter(b.ravel(),a,delayed))
                error=(predicted-measured)[use]
                self.assertLess(np.sqrt(np.mean(error**2)),0.10,(run['source'],axis))

    def test_updated_operating_point_predicts_untouched_v7_second_half(self):
        times = np.array(V7_FIXTURE['elapsed_s'])
        grid = np.arange(0, times[-1] + .002, .002)
        train, heldout = (times > 1.2) & (times < 8), times >= 8
        for axis in (0, 5):
            center = V8_PROFILE['center_raw_deg'][axis]
            commands = np.array(V7_FIXTURE['command_raw_deg'])[:, axis] - center
            measured = np.array(V7_FIXTURE['measured_raw_deg'])[:, axis] - center
            rms = []
            for profile in (LEGACY_PROFILE, V8_PROFILE):
                p = profile['axes'][str(axis+1)]
                wn, damping = p['natural_frequency_rad_s'], p['damping_ratio']
                held = np.r_[0, commands][np.searchsorted(times, grid, side='right')]
                delayed = np.interp(grid-p['delay_s'], grid, held, left=0)
                b, a, _ = signal.cont2discrete(([wn*wn], [1, 2*damping*wn, wn*wn]), .002)
                error = np.interp(times, grid, signal.lfilter(b.ravel(), a, delayed)) - measured
                # Bias is estimated on training data only, never on holdouts.
                error -= error[train].mean()
                rms.append(np.sqrt(np.mean(error[heldout]**2)))
            self.assertLess(rms[1], .09, (axis, rms))
            self.assertLess(rms[1], .9*rms[0], (axis, rms))

    def test_finite_startup_inverse_reduces_transient_at_recorded_cadence(self):
        from controls.hardware.weld_accuracy import TipModel, orientation_error_deg
        from motors.arm_calibration import CalibrationMap

        legacy = dict(deepcopy(V8_PROFILE), version=1)
        legacy.pop('startup_policy')
        intervals = np.diff([0, *V7_FIXTURE['elapsed_s']]).tolist()
        model = TipModel(CalibrationMap(FIXTURE['plan']['mapping']), FIXTURE['plan']['scene_fingerprint'])
        peaks = []
        for profile in (legacy, V8_PROFILE):
            _, _, c, _ = run_response(profile=profile, plant_profile=V8_PROFILE,
                                      intervals=intervals, duration=2.)
            self.addCleanup(c.close)
            self.assertEqual(c.state, 'welding', c.message)
            rows = [r for r in c.trace if r['stage']=='welding']
            angles = [orientation_error_deg(model.tip_pose(np.array(r['requested_raw_deg']))[1],
                                           model.tip_pose(np.array(r['measured_raw_deg']))[1]) for r in rows]
            peaks.append(max(angles))
        self.assertLess(peaks[1], .7, peaks)
        self.assertLess(peaks[1], .65*peaks[0], peaks)

    def test_startup_commands_are_continuous_bounded_and_fingerprinted(self):
        path = JointPath.from_document(FIXTURE['plan']['weld'])
        response = WeldResponse(V8_PROFILE, path, FIXTURE['plan'])
        self.assertEqual(response.period, path.duration)
        self.assertLess(response.startup_delay, .04)
        np.testing.assert_array_equal(response.startup_command.positions[0], path.positions[0])
        bounds = response.startup_command.position_bounds()
        for axis, maximum in ((0, 4), (5, 1)):
            self.assertLessEqual(bounds[1][axis]-bounds[0][axis]+2*response.residual_bound[axis], maximum)
        for value, key in zip(response.startup_command.peaks(), ('speed_deg_s', 'acceleration_deg_s2')):
            self.assertTrue(np.all(value <= FIXTURE['plan']['stage_limits']['weld'][key]))
        nominal = path.positions[0]
        left = response.sample(path.duration-1e-7, nominal, nominal)[0]
        right = response.sample(path.duration+1e-7, nominal, nominal)[0]
        self.assertLess(max(abs(left-right)), 1e-4)
        document = response.document()
        self.assertEqual(document['startup_policy'], 'finite_response_inverse')
        self.assertTrue(document['startup_command_fingerprint'])
        bad = deepcopy(V8_PROFILE); bad['startup_policy'] = 'unchecked'
        with self.assertRaisesRegex(ValueError, 'startup'):
            WeldResponse(bad, path, FIXTURE['plan'])

    def test_recorded_residual_replay_improves_startup_and_per_cycle_stroke(self):
        from controls.hardware.weld_accuracy import TipModel, orientation_error_deg, tip_cycle_strokes
        from motors.arm_calibration import CalibrationMap

        model = TipModel(CalibrationMap(FIXTURE['plan']['mapping']), FIXTURE['plan']['scene_fingerprint'])
        period = FIXTURE['plan']['weld']['times_s'][-1]
        origin = model.tip_mm(np.array(FIXTURE['plan']['weld']['raw_deg'][0]))
        results = []
        for profile in (LEGACY_PROFILE, V8_PROFILE):
            _, _, c, _ = run_response(profile=profile, plant_profile=V8_PROFILE,
                                      intervals=np.diff([0, *V7_FIXTURE['elapsed_s']]),
                                      duration=V7_FIXTURE['elapsed_s'][-1]-.0001,
                                      disturbance=recorded_disturbance())
            self.addCleanup(c.close)
            self.assertEqual(c.state, 'welding', c.message)
            rows = [r for r in c.trace if r['stage']=='welding']
            times = np.array([r['elapsed_s'] for r in rows])
            poses = [model.tip_pose(np.array(r['measured_raw_deg'])) for r in rows]
            nominal = [model.tip_pose(np.array(r['requested_raw_deg'])) for r in rows]
            angles = [orientation_error_deg(r[1], m[1]) for r, m in zip(nominal, poses)]
            transverse = np.array([p[0][1]-origin[1] for p in poses])
            keep = times >= 2*period
            cycles = tip_cycle_strokes(period, times[keep], transverse[keep], -10, 10)
            results.append((max(angles), cycles['median_width_mm']))
        self.assertGreater(results[0][0], 1.)  # Reproduce the recorded startup failure.
        self.assertLess(results[1][0], .85)
        self.assertLess(abs(results[1][1]-20), .5)
        self.assertLess(abs(results[1][1]-20), .4*abs(results[0][1]-20))

    def test_profile_rejects_wrong_identity_pose_frequency_amplitude_and_parameters(self):
        for key, value in [('mapping_fingerprint','wrong'),('scene_fingerprint','wrong'),
                           ('effective_hz',2),('harmonics',8),
                           ('residual_kp',10),('command_hold_s',float('nan'))]:
            profile=deepcopy(PROFILE);profile[key]=value
            with self.subTest(key=key), self.assertRaises(ValueError):
                WeldResponse(profile,JointPath.from_document(FIXTURE['plan']['weld']),FIXTURE['plan'])
        for key in ('center_raw_deg','travel_deg'):
            profile=deepcopy(PROFILE);profile[key][0]+=2
            with self.assertRaises(ValueError):
                WeldResponse(profile,JointPath.from_document(FIXTURE['plan']['weld']),FIXTURE['plan'])

    def test_periodic_targets_preserve_small_axes_and_do_not_reset_at_seams(self):
        path=JointPath.from_document(FIXTURE['plan']['weld'])
        response=WeldResponse(PROFILE,path,FIXTURE['plan'])
        for t in (0.,0.01,0.1,1.,3.,6.):
            nominal=path.sample(t%path.duration)[0]
            target,trace=response.sample(t,nominal,nominal+10)
            np.testing.assert_array_equal(target[1:5],nominal[1:5])
            self.assertLessEqual(max(abs(np.array(trace['residual_deg']))),.2)
            if t==0:np.testing.assert_array_equal(target,nominal)
        for boundary in (4*path.duration,10*path.duration):
            left=response.sample(boundary-1e-7,path.sample(path.duration-1e-7)[0],path.positions[0])[0]
            right=response.sample(boundary+1e-7,path.sample(1e-7)[0],path.positions[0])[0]
            self.assertLess(max(abs(left-right)),1e-4)

    def test_closed_loop_reduces_same_time_error_and_preserves_preparation(self):
        controls=[]; errors=[]; preparations=[]
        for enabled in (False,True):
            clock,arm,c,prep=run_response(enabled=enabled)
            self.addCleanup(c.close);controls.append(c);preparations.append(prep)
            self.assertEqual(c.state,'welding',c.message)
            rows=[r for r in c.trace if r['stage']=='welding' and r['elapsed_s']>2]
            errors.append(np.array([r['tracking_error_deg'] for r in rows]))
            if enabled:
                commands=np.array([r['command_raw_deg'] for r in rows])
                nominal=np.array([r['requested_raw_deg'] for r in rows])
                np.testing.assert_allclose(commands[:,1:5],nominal[:,1:5],atol=1e-10)
                self.assertTrue(all(r['tracker']['weld_response']['policy']=='bounded_periodic_response' for r in rows))
                self.assertEqual(c.tracking_deg,3)
                self.assertEqual(c.arrival_deg,.25)
                self.assertFalse(c.plan['weld_response']['online_learning'])
                self.assertEqual(c.plan['weld'],FIXTURE['plan']['weld'])
        np.testing.assert_array_equal([r['command_raw_deg'] for r in preparations[0]],
                                      [r['command_raw_deg'] for r in preparations[1]])
        for axis in (0,5):
            self.assertLess(np.sqrt(np.mean(errors[1][:,axis]**2)),.3*np.sqrt(np.mean(errors[0][:,axis]**2)))
        self.assertLess(np.max(abs(errors[1][:,[0,5]])),.65)

    def test_response_mismatch_irregular_ticks_pause_and_tracking_guard(self):
        for factor in (.8,1.2):
            clock,arm,c,_=run_response(factor=factor,intervals=np.resize([.01,.065,.02,.04],300))
            self.addCleanup(c.close)
            self.assertEqual(c.state,'welding',c.message)
            c.pause()
            held=arm.target.copy()
            for _ in range(20):clock.advance();c.tick()
            c.pause();np.testing.assert_array_equal(arm.target,held)
            self.assertIsNone(c.plan)
        clock,arm,c,_=run_response(duration=2.)
        self.addCleanup(c.close)
        arm.response_on=False
        arm.q[0]+=4;arm.target=arm.q.copy()
        clock.advance();c.tick()
        self.assertEqual(c.state,'paused')
        self.assertIn('Tracking error A1',c.message)

    def test_preflight_geometry_failure_does_not_enable(self):
        plan=deepcopy(FIXTURE['plan']);clock=Clock();arm=SimulatedArm(plan['current_raw_deg'],clock=clock)
        def reject(*a,**k):raise ValueError('explicit mechanical stop')
        c=WeldController(arm,clock=clock,pose_check=reject,continuous_weld=True,
                         tracking_settings=TrackingSettings(),weld_response_profile=PROFILE)
        self.addCleanup(c.close);c.select(True,False)
        response=WeldResponse(PROFILE,JointPath.from_document(plan['weld']),plan)
        with self.assertRaisesRegex(ValueError,'mechanical stop'):response.check_geometry(reject)
        with self.assertRaisesRegex(ValueError,'Recalculate'):c.load_plan(plan)
        self.assertFalse(arm.enabled)

    def test_cli_rejects_mixed_trial_modes_before_opening_any_owner(self):
        for extra in (['--weld-a1-shaper-only'],['--single-pass'],['--chassis'],['--tracker','paced']):
            result=subprocess.run([sys.executable,str(ROOT/'scripts/control_weld.py'),
                                   '--arm','--weld-response-profile',str(ROOT/'src/controls/config/weld_response_gun_20261124.json'),*extra],
                                  cwd=ROOT,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('requires continuous --arm alone',result.stderr)

    def test_feedforward_cannot_stack_on_velocity_lead_or_pid(self):
        t=PositionTracker();t.reset(np.zeros(6))
        with self.assertRaisesRegex(ValueError,'additive PID disabled'):
            t.step([0]*6,[0]*6,[0]*6,[0]*6,.02,[5]*6,[30]*6,
                   sampled_reference=True,feedforward_reference=[1]*6,correction_mask=np.ones(6,dtype=bool))

    def test_startup_reference_matches_preview_and_accuracy_without_hiding_error(self):
        from controls.hardware.weld_accuracy import analyze, sample_commands, WeldPass
        from simulation.weld_preview import PlanPlayback

        clock,arm,c,_=run_response(duration=3.)
        self.addCleanup(c.close)
        rows=[r for r in c.trace if r['stage']=='welding']
        run=WeldPass(c.plan,c.started,True,commands=deepcopy(rows),outcome='pause')
        path=c.paths['weld']
        samples,rejected=sample_commands(run,path)
        self.assertEqual(rejected,0)
        playback=PlanPlayback(c.plan)
        weld_start=playback.duration-2*path.duration
        elapsed=rows[3]['elapsed_s']
        stage,raw=playback.sample(weld_start+elapsed)
        self.assertEqual(stage,'weld_entry')
        np.testing.assert_allclose(raw,rows[3]['requested_raw_deg'],atol=1e-10)
        self.assertGreater(max(abs(path.sample(elapsed)[0]-np.array(raw))),.1)
        run.commands[3]['requested_raw_deg']=path.sample(elapsed)[0].tolist()
        self.assertEqual(sample_commands(run,path)[1],1)
        journal=[dict(kind='session',schema='forge.weld_motion_journal',version=1,
                      simulated=True,mapping=FIXTURE['plan']['mapping']),
                 dict(kind='accepted_plan',plan=c.plan),
                 dict(kind='sample',events=[dict(event='weld_started',time_s=c.started,continuous=True)])]
        journal.extend(dict(kind='sample',last_command=r) for r in rows)
        journal.extend([dict(kind='sample',events=[dict(event='pause')]),
                        dict(kind='session_end',fault=None,dropped_records=0)])
        with tempfile.TemporaryDirectory() as directory:
            file=Path(directory)/'session.jsonl'
            file.write_text(''.join(json.dumps(r)+'\n' for r in journal))
            report=analyze(file,fit_tip_lag=True,max_orientation_deg=1.)
        self.assertEqual(report['rejected_samples'],0)
        self.assertEqual(report['encoder_orientation']['samples'],len(rows))
        self.assertLess(report['encoder_orientation']['angle_deg']['peak_abs'],1.)
        self.assertGreater(report['tip_lag']['excluded_startup_samples'],0)
        self.assertEqual(report['issues'],['Pass outcome: pause'])
        self.assertFalse(report['passed'])


if __name__=='__main__':
    unittest.main()
