import contextlib
import ctypes
import io
import json
import math
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

    def test_group_first_keeps_individual_history_honest(self):
        p=plan(); p['motors'][2]['individual_test_passed']=False
        bench.validate_plan(p,execute=True,all_motors=True,group_first=True)
        self.assertFalse(p['motors'][2]['individual_test_passed'])
        with self.assertRaises(ValueError):
            bench.validate_plan(p,execute=True,group_first=True)
        p['fixture_confirmed']=False
        with self.assertRaises(ValueError):
            bench.validate_plan(p,execute=True,all_motors=True,group_first=True)

    def test_unconfirmed_plan_never_opens_port(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'plan.json'; p=plan(); p['fixture_confirmed']=False
            path.write_text(json.dumps(p))
            with patch.object(bench, 'ArmBus') as factory, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(bench.main(['--plan', str(path), '--all', '--execute']), 2)
            factory.assert_not_called()

    def test_configurable_speed_still_requires_feedback_envelope(self):
        p=plan(); p['motors']=p['motors'][:1]
        m=p['motors'][0]; m.update(delta_degrees=45, speed_rad_s=math.radians(45)/3)
        bench.validate_plan(p, execute=True)
        for speed in (.31, float('nan'), float('inf'), 0):
            m['speed_rad_s']=speed
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                bench.validate_plan(p, execute=True)


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

    def test_unloaded_speed_margin_still_trips_above_configured_limit(self):
        m=plan()['motors'][0]
        m.update(speed_rad_s=math.radians(45)/3, speed_trip_rad_s=.5)
        bench.validate_plan(dict(fixture_confirmed=True, motors=[m]), execute=True)
        s=self.session([m]); s.prepare(); state=s.stop()[0]
        s.command(0, state.position); self.bus.enable_bench_group([0])
        self.lib.fake_speed(0, .319); self.lib.fake_advance(2)
        self.assertFalse(self.bus.state(0).fault)
        self.lib.fake_speed(0, .6); self.lib.fake_advance(2)
        self.assertEqual(self.bus.state(0).fault_reason, 8)
        self.assertGreater(self.bus.fault_snapshot()['velocity_rad_s'], .5)
        s.cleanup()
        self.assertTrue(s.report['disable_acknowledged'])
        self.assertTrue(s.report['mode_restoration_verified'])

    def test_manufacturer_rated_envelopes_and_faults(self):
        for index, rpm, torque in ((0, 100, 20.), (3, 120, 3.5)):
            for scenario in ('motion', 'speed', 'torque'):
                with self.subTest(model=index, scenario=scenario):
                    self.bus.close(); self.bus=ArmBus(transport=NativeTransport(self.lib))
                    m=plan()['motors'][index]
                    rated_speed=rpm*2*math.pi/60
                    m.update(delta_degrees=45, speed_rad_s=math.radians(45)/3,
                             speed_trip_rad_s=rated_speed, torque_trip_nm=torque)
                    bench.validate_plan(dict(fixture_confirmed=True, motors=[m]), execute=True)
                    for key, value in (('speed_trip_rad_s', rated_speed+.1), ('torque_trip_nm', torque+.1)):
                        bad=dict(m); bad[key]=value
                        with self.assertRaises(ValueError):
                            bench.validate_plan(dict(fixture_confirmed=True, motors=[bad]), execute=True)
                        with self.assertRaises(ValueError):
                            self.bus.prepare_routed_bench(0,bad,-1,1,2)
                        with self.assertRaises(RuntimeError):
                            self.bus._request(16, struct.pack('<BBHHBB4f',0,m['bus'],m['can_id'],
                                m['feedback_id'],1 if index==0 else 3,2,-1,1,
                                bad['speed_trip_rad_s'],bad['torque_trip_nm']))
                    s=self.session([m])
                    try:
                        if scenario=='motion':
                            s.run()
                            self.assertEqual(len(s.report['stages']),2)
                        else:
                            s.prepare(); state=s.stop()[0]
                            s.command(0,state.position); self.bus.enable_bench_group([0])
                            if scenario=='speed': self.lib.fake_speed(index,rated_speed+1)
                            else: self.lib.fake_torque(index,torque+1)
                            self.lib.fake_advance(2)
                            self.assertEqual(self.bus.state(0).fault_reason,8 if scenario=='speed' else 9)
                    finally:
                        s.cleanup()
                    self.assertTrue(s.report['disable_acknowledged'])
                    self.assertTrue(s.report['mode_restoration_verified'])
                    self.assertEqual(self.lib.fake_status(index),0)

    def id_fixture(self, index):
        from motors import arm as arm_module, arm_ids
        # Other physical units are disconnected from the selected bus.
        for i in range(6):
            if i!=index: self.lib.fake_missing(i)
        bus=1 if index==0 else 2
        self.lib.fake_route(index,bus,1,0)
        clock=SimpleNamespace(monotonic=lambda:self.lib.fake_now()/1000.,
                              sleep=lambda seconds:self.lib.fake_advance(round(seconds*1000)))
        return arm_module, arm_ids, clock, bus

    def test_ids_assign_all_dofs_without_motion_and_verify_after_cycle(self):
        for index in range(6):
            with self.subTest(dof=index+1):
                self.bus.close(); self.bus=ArmBus(transport=NativeTransport(self.lib))
                arm_module, ids, clock, channel=self.id_fixture(index)
                report={}
                with patch.object(arm_module,'time',clock), patch.object(ids,'time',clock):
                    ids.assign(self.bus,dof=index+1,channel=channel,current_id=1,current_feedback=0,report=report)
                    self.assertEqual(self.lib.fake_id(index),index+1)
                    self.assertEqual(self.lib.fake_rx(index),17+index)
                    self.assertEqual(self.lib.fake_saves(index),1)
                    self.assertEqual(self.lib.fake_commands(index),0)
                    self.assertEqual(self.lib.fake_mode(index),1)
                    self.assertEqual(self.lib.fake_timeout(index),0)
                    self.assertTrue(report['disable_acknowledged'])
                    with self.assertRaises(RuntimeError): self.bus.enable_bench_group([0])
                    with self.assertRaises(RuntimeError): self.bus.command(0,0,velocity=.1)
                    with self.assertRaises(RuntimeError): self.bus._request(17,struct.pack('<BBI',0,10,2))
                    with self.assertRaises(ValueError): self.bus.id_operation(10,2)
                    self.lib.fake_power_cycle(); self.bus._request(1)
                    verification={}
                    ids.assign(self.bus,dof=index+1,channel=channel,current_id=1,current_feedback=0,
                               report=verification,verify=True)
                    self.assertTrue(verification['power_cycle_verified'])
                    self.assertEqual(self.lib.fake_saves(index),1)
                    self.assertEqual(self.lib.fake_invalid(),0)

    def test_passive_rx_trace_does_not_send_can(self):
        before=self.bus.can_rx_trace(1)
        self.assertEqual(before['count'],0)
        self.assertEqual(self.bus.read_register(1,1,8),(17,1))
        trace=self.bus.can_rx_trace(1)
        self.assertEqual(trace['can_id'],17)
        self.assertEqual(trace['length'],8)
        self.assertEqual(trace['data'],'0100330801000000')
        self.assertEqual(self.bus.can_rx_trace(1),trace)
        self.assertEqual(self.bus.can_rx_trace(2)['count'],0)
        self.assertEqual(self.lib.fake_commands(0),0)

    def test_id_assignment_fails_closed_on_write_or_save_failure(self):
        for fault in ('write', 'save'):
            self.bus.close(); self.bus=ArmBus(transport=NativeTransport(self.lib))
            arm_module, ids, clock, channel=self.id_fixture(1)
            if fault=='write': self.lib.fake_ignore_ids(1)
            else: self.lib.fake_drop_save(1)
            report={}
            with patch.object(arm_module,'time',clock), patch.object(ids,'time',clock), \
                 self.assertRaises(RuntimeError):
                ids.assign(self.bus,dof=2,channel=channel,current_id=1,current_feedback=0,report=report)
            self.assertFalse(report.get('ids_written',False))
            self.assertEqual(self.lib.fake_saves(1),0 if fault=='write' else 1)
            self.assertEqual(self.lib.fake_commands(1),0)

    def test_id_operations_require_disabled_lock_and_readback(self):
        with self.assertRaises(RuntimeError): self.bus.id_operation(7,17)
        s=self.session([plan()['motors'][0]]); s.prepare()
        state=s.stop()[0]; s.command(0,state.position); self.bus.enable_bench_group([0])
        with self.assertRaises(RuntimeError): self.bus.id_operation(0)
        s.stop(); self.bus.id_operation(0)
        with self.assertRaises(RuntimeError): self.bus.id_operation(0xaa)
        for op,value in ((7,0),(8,0),(8,7),(7,23),(9,2000)):
            with self.assertRaises(RuntimeError):
                self.bus._request(20,struct.pack('<BI',op,value))

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

    def test_group_first_runs_group_then_each_unit_and_restores_modes(self):
        s=self.session(plan()['motors'])
        try: s.run(simultaneous=True,group_first=True)
        finally: s.cleanup()
        labels=[stage['stage'] for stage in s.report['stages']]
        self.assertEqual(labels[:2],['all motors outward','all motors return'])
        self.assertEqual(labels[2:],[f'unit-{i} {direction}' for i in range(6)
                                   for direction in ('outward','return')])
        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
        self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_group_first_at_triple_speed_all_models(self):
        motors=plan()['motors']
        for m in motors:
            m.update(delta_degrees=45, speed_rad_s=math.pi/4, speed_trip_rad_s=1.5)
        s=self.session(motors)
        try:
            s.run(simultaneous=True,group_first=True)
            self.assertEqual(len(s.report['stages']),14)
            for stage in s.report['stages']:
                self.assertLess(stage['elapsed_s'],2)
                for sample in stage['motors']:
                    self.assertLess(abs(sample['error_rad']),math.radians(1))
            # The configured feedback envelope remains enforced by firmware.
            with self.assertRaises(RuntimeError):
                self.bus.command(1,s.origins[1],velocity=1.51)
        finally: s.cleanup()
        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
        self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_cached_base_telemetry_adds_no_can_traffic(self):
        info=self.bus.base_info()
        self.assertEqual(info['api_version'],1)
        self.assertEqual(info['can_bitrate'],1000000)
        self.assertEqual(info['command_lease_ms'],150)
        snap=self.bus.snapshot()
        self.assertEqual(len(snap['motors']),6)
        self.assertGreaterEqual(snap['acquisition_duration_ms'],0)
        self.assertGreater(snap['host_received_monotonic_s'],0)
        self.assertTrue(all(m['position'] is None and m['age_ms'] is None
                            for m in snap['motors']))
        before=self.bus.bus_health(1)
        self.bus.snapshot(); self.bus.base_info(); self.bus.fault_snapshot()
        after=self.bus.bus_health(1)
        for key in ('tx_frames','tx_failed','rx_frames','feedback_frames'):
            self.assertEqual(before[key],after[key])
        self.bus.read_register(1,1,8)
        after=self.bus.bus_health(1)
        self.assertEqual(after['tx_frames'],before['tx_frames']+1)
        self.assertEqual(after['rx_frames'],before['rx_frames']+1)

    def test_snapshot_pages_are_frozen_and_generation_checked(self):
        first=self.bus._request(22,struct.pack('<BI',0,0))
        generation,at=struct.unpack_from('<II',first,1)
        self.lib.fake_advance(50)
        second=self.bus._request(22,struct.pack('<BI',1,generation))
        self.assertEqual(struct.unpack_from('<II',second,1),(generation,at))
        self.bus.snapshot()  # starts a different generation
        with self.assertRaises(RuntimeError):
            self.bus._request(22,struct.pack('<BI',2,generation))

    def test_telemetry_does_not_renew_motion_lease_or_clear_fault(self):
        s=self.session([plan()['motors'][0]]); s.prepare()
        state=s.stop()[0];s.command(0,state.position);self.bus.enable_bench_group([0])
        for _ in range(12): self.bus.snapshot()
        snap=self.bus.snapshot()
        self.assertTrue(snap['fault'])
        self.assertEqual(snap['fault_reason'],1)
        self.assertFalse(snap['motors'][0]['enabled'])
        health=self.bus.bus_health(1)
        self.assertGreater(health['feedback_frames'],0)
        self.assertGreaterEqual(health['max_tick_gap_ms'],2)
        self.assertTrue(self.bus.state(0).fault)
        s.cleanup()

    def test_repeated_group_runs_without_reset_after_feedback_expires(self):
        motors=plan()['motors']
        for attempt in range(2):
            # Reopen-like host flags; controller retains its previous slots.
            self.bus.configured=False; self.bus.bench=False
            observed=bench.preflight(self.bus,motors)
            s=bench.BenchSession(self.bus,motors,observed,{'stages':[]})
            try: s.run(simultaneous=True,group_first=True)
            finally: s.cleanup()
            self.assertEqual(len(s.report['stages']),14)
            self.lib.fake_advance(1500)
        before=[self.lib.fake_commands(i) for i in range(6)]
        bench.preflight(self.bus,motors)  # --check on retained session
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)],before)
        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
        self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_preflight_does_not_clear_retained_fault(self):
        s=self.session([plan()['motors'][0]]); s.prepare()
        state=s.stop()[0]; s.command(0,state.position); self.bus.enable_bench_group([0])
        self.lib.fake_advance(200)
        with self.assertRaisesRegex(RuntimeError,'explicit recovery'):
            bench.preflight(self.bus,[plan()['motors'][0]])
        self.assertTrue(self.bus.state(0).fault)
        s.cleanup()

    def test_position_port_selects_each_joint_without_cross_motion(self):
        from motors import arm_position as pos
        clock=SimpleNamespace(monotonic=lambda:self.lib.fake_now()/1000.,
                              sleep=lambda seconds:self.lib.fake_advance(round(seconds*1000)))
        p=plan()
        for m in p['motors']:
            m.update(lower_rad=-12.5,upper_rad=12.5,speed_trip_rad_s=1.5)
        with patch.object(pos,'time',clock):
            port=pos.ArmPositionPort(p,bus=self.bus)
            try:
                for joint in range(1,7):
                    before=[self.lib.fake_commands(i) for i in range(6)]
                    port.select(joint)
                    self.assertTrue(all(self.lib.fake_status(i)==0 for i in range(6)))
                    with self.assertRaises(RuntimeError):port.move_by(.1,.5)
                    port.arm();port.move_by(.1,.5)
                    for _ in range(300):
                        self.lib.fake_advance(10)
                        if not port.poll():break
                    self.assertFalse(port.moving)
                    self.assertTrue(port.armed)
                    self.assertAlmostEqual(port.positions()[joint-1].position_rad,.1,delta=.01)
                    for i in range(6):
                        if i!=joint-1:self.assertEqual(self.lib.fake_commands(i),before[i])
                    port.move_to(0.,.5)
                    self.lib.fake_advance(20);port.poll();port.release()
                    self.assertTrue(port.armed);self.assertFalse(port.moving)
                    self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
                    port.move_to(0.,.5)
                    for _ in range(300):
                        self.lib.fake_advance(10)
                        if not port.poll():break
                    self.assertFalse(port.moving)
            finally:port.close()
            self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            self.assertEqual(self.lib.fake_invalid(),0)

    def test_position_port_window_selection_and_stale_poll_fail_closed(self):
        from motors import arm_position as pos
        clock=SimpleNamespace(monotonic=lambda:self.lib.fake_now()/1000.,
                              sleep=lambda seconds:self.lib.fake_advance(round(seconds*1000)))
        p=plan()
        for m in p['motors']:m.update(lower_rad=-12.5,upper_rad=12.5,speed_trip_rad_s=1.5)
        with patch.object(pos,'time',clock):
            port=pos.ArmPositionPort(p,bus=self.bus)
            try:
                port.arm()
                with self.assertRaises(ValueError):port.move_to(2.,.5)
                with self.assertRaises(ValueError):port.move_to(float('nan'),.5)
                with self.assertRaises(ValueError):port.move_to(.1,2.)
                port.move_by(.2,.5)
                port.select(2)
                self.assertFalse(port.armed);self.assertFalse(port.moving)
                self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
                port.arm();port.move_by(.2,.5);self.lib.fake_advance(200)
                with self.assertRaises(RuntimeError):port.poll()
                self.assertTrue(self.bus.state(0).fault)
                self.assertFalse(port.armed)
            finally:port.close()

    def test_position_port_checks_primer_during_enable_and_before_poll(self):
        from motors import arm_position as pos
        clock=SimpleNamespace(monotonic=lambda:self.lib.fake_now()/1000.,
                              sleep=lambda seconds:self.lib.fake_advance(round(seconds*1000)))
        p=plan()
        for m in p['motors']:m.update(lower_rad=-12.5,upper_rad=12.5,speed_trip_rad_s=1.5)
        allowed=[True]
        with patch.object(pos,'time',clock):
            port=pos.ArmPositionPort(p,bus=self.bus,motion_allowed=lambda:allowed[0])
            try:
                port.select(2);port.arm()
                original=self.bus.enable_bench_group
                def release_during_enable(slots):
                    original(slots);allowed[0]=False
                with patch.object(self.bus,'enable_bench_group',side_effect=release_during_enable), \
                     patch.object(self.bus,'command',wraps=self.bus.command) as command:
                    with self.assertRaises(pos.MotionCancelled):port.move_to(.2,.5)
                    self.assertFalse(any(call.args[1] == .2 for call in command.call_args_list))
                self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
                self.assertFalse(port.armed)
                allowed[0]=True;port.arm();port.move_to(.2,.5)
                self.lib.fake_advance(10)
                before=[self.lib.fake_commands(i) for i in range(6)]
                allowed[0]=False
                with self.assertRaises(pos.MotionCancelled):port.poll()
                self.assertEqual([self.lib.fake_commands(i) for i in range(6)],before)
                self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            finally:port.close()
            self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
            self.assertEqual(self.lib.fake_invalid(),0)

    def test_180_degree_group_then_individuals_at_same_speed(self):
        p=plan()
        for m in p['motors']:
            m.update(delta_degrees=180,speed_rad_s=math.pi/4,
                     speed_trip_rad_s=1.5,lower_rad=-12.5,upper_rad=12.5)
        bench.validate_plan(p,execute=True,all_motors=True,group_first=True)
        s=self.session(p['motors'])
        try:
            s.run(simultaneous=True,group_first=True)
            self.assertEqual(len(s.report['stages']),14)
            for stage in s.report['stages']:
                self.assertGreater(stage['elapsed_s'],3.5)
                self.assertLess(stage['elapsed_s'],5)
                for sample in stage['motors']:
                    self.assertLess(abs(sample['error_rad']),math.radians(1))
        finally: s.cleanup()
        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
        self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
        self.assertEqual(self.lib.fake_invalid(),0)
        p['motors'][0]['delta_degrees']=181
        with self.assertRaises(ValueError):
            bench.validate_plan(p,execute=True,all_motors=True,group_first=True)

    def test_45_degree_three_second_speed_and_firmware_envelope(self):
        for index in (0, 3):
            with self.subTest(model=index):
                self.bus.close(); self.bus=ArmBus(transport=NativeTransport(self.lib))
                m=plan()['motors'][index]
                m.update(delta_degrees=45, speed_rad_s=math.radians(45)/3)
                s=self.session([m])
                try:
                    s.run()
                    for stage in s.report['stages']:
                        self.assertLess(stage['elapsed_s'], 4)
                        self.assertLess(abs(stage['motors'][0]['error_rad']), math.radians(1))
                        self.assertGreaterEqual(stage['enable_confirmation_s'], 0)
                        sample=stage['motors'][0]
                        self.assertIsNotNone(sample['first_motion_observed_s'])
                        self.assertLess(sample['first_motion_observed_s'], .25)
                        self.assertLess(sample['first_motion_observed_s'], sample['first_within_one_degree_s'])
                        self.assertLessEqual(sample['first_within_one_degree_s'], stage['elapsed_s'])
                    # Direct host call must still be rejected by MCU above the
                    # configured .3 rad/s feedback envelope.
                    with self.assertRaises(RuntimeError):
                        self.bus.command(0, s.origins[0], velocity=.31)
                finally:
                    s.cleanup()
                self.assertEqual(len(s.report['stages']), 2)
                self.assertEqual(self.lib.fake_invalid(), 0)
                self.assertEqual(self.lib.fake_status(index), 0)
                self.assertEqual(self.lib.fake_mode(index), 1)

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

    def test_cleanup_refreshes_disabled_feedback_until_coasting_stops(self):
        s=self.session([plan()['motors'][0]]); s.prepare()
        commands=self.lib.fake_commands(0)
        before=self.lib.fake_disables(0)
        self.lib.fake_coast(0, 4)
        s.cleanup()
        self.assertGreaterEqual(self.lib.fake_disables(0)-before, 5)
        self.assertEqual(self.lib.fake_commands(0), commands)
        self.assertTrue(s.report['disable_acknowledged'])
        self.assertTrue(s.report['mode_restoration_verified'])
        self.assertEqual(self.lib.fake_mode(0), 1)

    def test_cleanup_does_not_accept_persistent_coasting(self):
        s=self.session([plan()['motors'][0]]); s.prepare()
        self.lib.fake_coast(0, 10000)
        start=self.lib.fake_now()
        with self.assertRaisesRegex(RuntimeError, 'Disable not confirmed'):
            s.cleanup()
        self.assertLess(self.lib.fake_now()-start, 1100)
        self.assertEqual(self.lib.fake_mode(0), 2)
        self.assertFalse(s.report.get('disable_acknowledged', False))

    def test_fault_sample_survives_disable_feedback(self):
        s=self.enable_all()
        self.lib.fake_speed(3, .5); self.lib.fake_advance(2)
        s.capture_failure()
        snapshot=s.report['fault_snapshot']
        self.assertTrue(snapshot['fault'])
        self.assertEqual(snapshot['reason'], 8)
        self.assertEqual(snapshot['slot'], 3)
        self.assertEqual(snapshot['sample_flags'], 3)
        self.assertGreater(snapshot['velocity_rad_s'], snapshot['speed_limit_rad_s'])
        self.assertAlmostEqual(snapshot['command_velocity_rad_s'], .1, places=5)
        self.assertLess(abs(self.bus.state(3).velocity), .03)
        s.cleanup()
        self.assertEqual(self.bus.fault_snapshot(), snapshot)
        self.bus.reset_fault()
        self.assertFalse(self.bus.fault_snapshot()['fault'])
        self.assertEqual(self.bus.fault_snapshot()['sample_flags'], 0)

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
            def interrupt(session, simultaneous=False, group_first=False):
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
        self.bus.prepare_routed_bench(0,m,-2,2,2); self.bus.probe(0); self.lib.fake_advance(20)
        state=s.stop()[0]; self.bus.command(0,state.position,velocity=.1)
        with self.assertRaises(RuntimeError): self.bus.enable(0)
        s.cleanup()


if __name__ == '__main__':
    unittest.main()
