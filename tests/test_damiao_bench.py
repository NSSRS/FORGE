import contextlib
import ctypes
import io
import json
from pathlib import Path
import struct
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from motors.arm import ArmBus
from motors import arm_bench as bench
from test_damiao_arm import NativeTransport

ROOT = Path(__file__).resolve().parents[1]


def plan(final=False):
    motors = []
    for i in range(6):
        motors.append(dict(unit=f'unit-{i}', model='8009P' if i == 0 else '4340P' if i < 3 else '4310P',
            bus=1 if i == 0 else 3 if final and i >= 3 else 2, can_id=i+1, feedback_id=17+i,
            identity_confirmed=True, individual_test_passed=True,
            ranges=dict(pmax=12.5, vmax=45 if i == 0 else 10 if i < 3 else 30,
                        tmax=54 if i == 0 else 28 if i < 3 else 10),
            lower_rad=-1, upper_rad=1, speed_rad_s=.1, speed_trip_rad_s=.3,
            torque_trip_nm=.5, delta_degrees=5))
    return dict(fixture_confirmed=True, motors=motors)


class PlanTests(unittest.TestCase):
    def test_templates_are_not_hardware_confirmation(self):
        for path in (ROOT/'src/robot_description/config').glob('bench_*.json'):
            p = json.loads(path.read_text())
            with self.subTest(path=path.name), self.assertRaises(ValueError):
                bench.validate_plan(p, execute=True, all_motors=len(p['motors']) == 6)

    def test_full_validation_and_bus_isolation(self):
        bench.validate_plan(plan(), execute=True, all_motors=True)
        for key, value in [('bus', 1), ('can_id', 2), ('feedback_id', 18),
                           ('torque_trip_nm', float('nan')), ('individual_test_passed', False)]:
            p = plan(); p['motors'][3][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                bench.validate_plan(p, execute=True, all_motors=True)

    def test_unconfirmed_plan_never_opens_port(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'plan.json'; p=plan(); p['fixture_confirmed']=False
            path.write_text(json.dumps(p))
            with patch.object(bench, 'ArmBus') as factory, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(bench.main(['--plan', str(path), '--all', '--execute']), 2)
            factory.assert_not_called()


class RoutedRoundTrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        so = Path(cls.temp.name)/'routed.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-std=c11', '-Wall', '-Wextra',
            '-I'+str(ROOT/'src/damiao_firmware/include'), str(ROOT/'src/damiao_firmware/src/arm.c'),
            str(ROOT/'src/damiao_firmware/src/wire.c'), str(ROOT/'tests/damiao_routed_bridge.c'),
            '-lm', '-o', str(so)], check=True)
        cls.lib = ctypes.CDLL(str(so))
        cls.lib.fake_exchange.argtypes = [ctypes.c_char_p, ctypes.c_void_p]
        cls.lib.fake_torque.argtypes = [ctypes.c_uint, ctypes.c_float]
        cls.lib.fake_speed.argtypes = [ctypes.c_uint, ctypes.c_float]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.bus = ArmBus(transport=NativeTransport(self.lib))
        clock = SimpleNamespace(monotonic=lambda: self.lib.fake_now()/1000.,
                                sleep=lambda s: self.lib.fake_advance(round(s*1000)))
        self.clock_patch = patch.object(bench, 'time', clock); self.clock_patch.start()
        self.output = contextlib.redirect_stdout(io.StringIO()); self.output.__enter__()

    def tearDown(self):
        self.bus.close(); self.clock_patch.stop(); self.output.__exit__(None, None, None)

    def session(self, motors):
        observed = bench.preflight(self.bus, motors)
        return bench.BenchSession(self.bus, motors, observed, {'stages': []})

    def test_each_model_and_factory_feedback_zero(self):
        for index in (0, 1, 3):
            with self.subTest(model=index):
                self.bus.close(); self.bus = ArmBus(transport=NativeTransport(self.lib))
                m=plan()['motors'][index]; m['feedback_id']=0
                self.lib.fake_route(index, m['bus'], m['can_id'], 0)
                s=self.session([m])
                try: s.run()
                finally: s.cleanup()
                self.assertEqual(len(s.report['stages']), 2)
                self.assertEqual(self.lib.fake_mode(index), 1)
                self.assertEqual(self.lib.fake_status(index), 0)
                self.assertEqual(self.lib.fake_invalid(), 0)

    def test_all_six_both_topologies_and_negative_travel(self):
        for final in (False, True):
            self.bus.close(); self.bus=ArmBus(transport=NativeTransport(self.lib))
            motors=plan(final)['motors']
            for i,m in enumerate(motors):
                self.lib.fake_route(i, m['bus'], m['can_id'], m['feedback_id'])
                if i % 2: m['delta_degrees']=-5
            s=self.session(motors)
            try: s.run(simultaneous=True)
            finally: s.cleanup()
            self.assertEqual(len(s.report['stages']), 14)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)
            self.assertEqual([self.lib.fake_mode(i) for i in range(6)], [1]*6)
            self.assertEqual(self.lib.fake_invalid(), 0)

    def test_stall_fails_and_restores(self):
        s=self.session([plan()['motors'][0]]); self.lib.fake_stall(0)
        with self.assertRaisesRegex(RuntimeError, 'not reached'):
            try: s.run()
            finally: s.cleanup()
        self.assertEqual(self.lib.fake_mode(0), 1)
        self.assertEqual(self.lib.fake_status(0), 0)

    def test_wrong_identity_ranges_and_missing_reply_no_commands(self):
        for field,value in [('feedback_id', 99), ('can_id', 15), ('ranges', dict(pmax=12.5,vmax=30,tmax=10))]:
            m=plan()['motors'][0]; m[field]=value
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                self.session([m])
        self.assertEqual(self.lib.fake_commands(0), 0)

    def test_wrong_mode_readback_prevents_enable(self):
        s=self.session([plan()['motors'][0]]); self.lib.fake_ignore_mode(0)
        with self.assertRaisesRegex(RuntimeError, 'readback mismatch'):
            try: s.prepare()
            finally: s.cleanup()
        self.assertEqual(self.lib.fake_commands(0), 0)

    def enable_all(self):
        s=self.session(plan()['motors']); s.prepare(); states=s.stop()
        for i in range(6):
            s.command(i, states[i].position)
        self.bus.enable_bench_group(range(6))
        return s

    def test_multibus_rate_and_diagnostics_interlock(self):
        s=self.enable_all()
        for i in range(6): s.command(i, s.origins[i])
        before=[self.lib.fake_commands(i) for i in range(6)]
        self.lib.fake_advance(50)
        self.assertEqual([self.lib.fake_commands(i)-before[i] for i in range(6)], [25]*6)
        with self.assertRaises(RuntimeError): self.bus.read_register(1,1,10)
        with self.assertRaises(RuntimeError): self.bus.write_routed_bench(0,10,1)
        s.cleanup()

    def test_one_missing_feedback_stops_every_motor(self):
        s=self.enable_all(); self.lib.fake_missing(3)
        # Keep commands fresh to isolate the feedback watchdog.
        for _ in range(4):
            for i in range(6):
                if not self.bus.state(i).fault: s.command(i, s.origins[i])
        state=self.bus.state(0)
        self.assertTrue(state.fault)
        self.assertEqual(state.fault_reason, 2)
        self.assertFalse(any(self.bus.state(i).enabled for i in range(6)))
        with self.assertRaisesRegex(RuntimeError, 'Disable not confirmed'): s.cleanup()

    def test_torque_speed_and_lease_trip_all_and_restore(self):
        for failure in ('torque', 'speed', 'lease'):
            self.bus.close(); self.bus=ArmBus(transport=NativeTransport(self.lib))
            s=self.enable_all()
            if failure=='torque': self.lib.fake_torque(3, 1.)
            elif failure=='speed': self.lib.fake_speed(3, .5)
            self.lib.fake_advance(160 if failure=='lease' else 2)
            state=self.bus.state(0)
            self.assertTrue(state.fault)
            self.assertEqual(state.fault_reason, {'torque':9,'speed':8,'lease':1}[failure])
            self.assertFalse(any(self.bus.state(i).enabled for i in range(6)))
            s.cleanup()
            self.assertEqual([self.lib.fake_mode(i) for i in range(6)], [1]*6)

    def test_group_enable_checks_all_slots_before_any_enable(self):
        s=self.session(plan()['motors']); s.prepare(); states=s.stop()
        for i in range(5): s.command(i, states[i].position)
        with self.assertRaises(RuntimeError): self.bus.enable_bench_group(range(6))
        self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)], [0]*6)
        s.cleanup()

    def test_mcu_duration_cutoff_with_fresh_commands(self):
        s=self.session([plan()['motors'][0]]); s.prepare(); state=s.stop()[0]
        s.command(0,state.position); self.bus.enable_bench_group([0])
        for _ in range(320):
            if self.bus.state(0).fault: break
            s.command(0,state.position); self.lib.fake_advance(45)
        self.assertEqual(self.bus.state(0).fault_reason,10)
        s.cleanup()

    def test_cli_read_only_and_motion_reports(self):
        for action in ('--check', '--execute'):
            self.bus.close(); self.bus=ArmBus(transport=NativeTransport(self.lib))
            with tempfile.TemporaryDirectory() as d:
                path=Path(d)/'plan.json'; report=Path(d)/'result.json'
                p=plan(); p['motors']=p['motors'][:1]; path.write_text(json.dumps(p))
                with patch.object(bench,'ArmBus',lambda _: self.bus):
                    code=bench.main(['--plan',str(path),action,'--report',str(report)])
                self.assertEqual(code,0)
                result=json.loads(report.read_text()); self.assertTrue(result['passed'])
                self.assertEqual(result['disable_acknowledged'],action=='--execute')
                self.assertEqual(self.lib.fake_status(0),0)
                self.assertEqual(self.lib.fake_mode(0),1)
                if action=='--check': self.assertEqual(self.lib.fake_commands(0),0)

    def test_interrupt_runs_cleanup_and_reports_failure(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'plan.json'; report=Path(d)/'result.json'
            p=plan(); p['motors']=p['motors'][:1]; path.write_text(json.dumps(p))
            def interrupt(session, simultaneous=False):
                session.prepare()
                state=session.stop()[0]; session.command(0,state.position)
                session.bus.enable_bench_group([0])
                raise KeyboardInterrupt
            with patch.object(bench,'ArmBus',lambda _: self.bus), \
                 patch.object(bench.BenchSession,'run',interrupt), contextlib.redirect_stderr(io.StringIO()):
                code=bench.main(['--plan',str(path),'--execute','--report',str(report)])
            self.assertEqual(code,130)
            result=json.loads(report.read_text())
            self.assertFalse(result['passed']); self.assertTrue(result['disable_acknowledged'])
            self.assertTrue(result['mode_restoration_verified'])
            self.assertEqual(self.lib.fake_mode(0),1)
            self.assertEqual(self.lib.fake_status(0),0)

    def test_firmware_rejects_wrong_bus_wide_enable_and_route_change(self):
        m=plan()['motors'][0]
        # Exercise firmware validation without the Python guard.
        with self.assertRaises(RuntimeError):
            self.bus._request(16, struct.pack('<BBHHBB4f',0,2,1,17,1,2,-1,1,.3,.5))
        s=self.session([m]); s.prepare()
        with self.assertRaises(RuntimeError):
            self.bus._request(16, struct.pack('<BBHHBB4f',0,1,2,17,1,2,-.1,.1,.3,.5))
        self.bus.prepare_routed_bench(0,m,-1,1,2); self.bus.probe(0); self.lib.fake_advance(20)
        state=s.stop()[0]; self.bus.command(0,state.position,velocity=.1)
        with self.assertRaises(RuntimeError): self.bus.enable(0)
        s.cleanup()


if __name__ == '__main__':
    unittest.main()
