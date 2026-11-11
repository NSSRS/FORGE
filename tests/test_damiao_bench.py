import contextlib
import ctypes
from dataclasses import replace
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

from motors.arm import ArmBus, CAP_RATED_4340_TORQUE, CAP_CONTROL_READBACK, CONTROL_REGISTERS
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
    def test_control_inspection_requires_disabled_check_and_saved_report(self):
        required = ['--check', '--feedback', '--supported', '--report', 'result.json']
        for args in [required[:i]+required[i+1:] for i in range(3)] + [required[:3]]:
            with self.subTest(args=args), patch.object(bench,'ArmBus') as factory, \
                 contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                bench.main(['--plan','unused.json','--inspect-control']+args)
            factory.assert_not_called()

    def test_subset_plan_validation_and_no_automatic_sweep(self):
        for joints in ([], [2,2], [0,2], [True,2], [7], '2,3'):
            p=plan();p['active_joints']=joints
            with self.subTest(joints=joints), self.assertRaises(ValueError):
                bench.validate_plan(p,all_motors=True)
        p=plan();p['active_joints']=[2,3,4,5,6]
        self.assertEqual(bench.active_joints(p),(2,3,4,5,6))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'plan.json';path.write_text(json.dumps(p))
            with patch.object(bench,'ArmBus') as factory, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(bench.main(['--plan',str(path),'--all','--execute']),2)
            factory.assert_not_called()
        from motors.arm_position import ArmPositionPort
        p.update(fixture_mode='supported',supported_confirmed=False)
        with patch('motors.arm_position.ArmBus') as factory:
            with self.assertRaisesRegex(ValueError,'support confirmation'):
                ArmPositionPort(p,window_degrees=3)
            p['supported_confirmed']=True
            with self.assertRaisesRegex(ValueError,'window'):
                ArmPositionPort(p,window_degrees=90)
            factory.assert_not_called()

    def test_recovery_cli_requires_disabled_check_support_and_report(self):
        required = ['--check', '--feedback', '--supported', '--report', 'result.json']
        cases = [required[:i] + required[i+1:] for i in range(3)]
        cases += [required[:3], ['--execute', '--supported', '--report', 'result.json']]
        for args in cases:
            with self.subTest(args=args), patch.object(bench, 'ArmBus') as factory, \
                 contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                bench.main(['--plan', 'unused.json', '--recover'] + args)
            factory.assert_not_called()

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
        cls.lib.fake_position.argtypes = [ctypes.c_uint, ctypes.c_float]
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

    def test_control_inspection_typed_values_sparse_routes_and_no_writes(self):
        p=plan();p['active_joints']=[2,3,4,5,6]
        self.lib.fake_missing(0)
        self.lib.fake_register(1,14,0x1020304)
        self.lib.fake_register(1,5,struct.unpack('<I',struct.pack('<f',-12.5))[0])
        self.lib.fake_register(1,59,struct.unpack('<I',struct.pack('<f',19.85))[0])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'plan.json';path.write_text(json.dumps(p))
            report=Path(directory)/'report.json'
            with patch.object(bench,'ArmBus',return_value=self.bus), \
                 patch.object(self.bus,'_request',wraps=self.bus._request) as request:
                self.assertEqual(bench.main(['--plan',str(path),'--all','--check','--feedback',
                    '--supported','--inspect-control','--report',str(report)]),0)
            result=json.loads(report.read_text())
            self.assertTrue(result['control_readback_complete'])
            self.assertTrue(result['disable_acknowledged'])
            self.assertFalse(result['motor_parameter_writes'])
            self.assertEqual([r['joint'] for r in result['control_registers']],[2,3,4,5,6])
            regs=result['control_registers'][0]['registers']
            self.assertEqual(regs['sw_ver']['value'],0x1020304)
            self.assertEqual(regs['DEC']['value'],-12.5)
            self.assertAlmostEqual(regs['Imax']['value'],19.85,places=4)
            self.assertEqual(regs['Imax']['reply_can_id'],18)
            self.assertEqual(regs['Imax']['raw_hex'],'419ecccd')
            self.assertTrue({c.args[0] for c in request.call_args_list}.isdisjoint({4,5,8,11,15,17,18,20}))
        self.assertEqual(self.lib.fake_parameter_writes(),0)
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)],[0]*6)
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_control_inspection_skips_undocumented_8009_registers(self):
        s=self.session([plan()['motors'][0]]);s.probe_disabled()
        try:
            s.inspect_control()
            regs=s.report['control_registers'][0]['registers']
            for reg,name in ((59,'Imax'),(60,'VBus')):
                self.assertEqual(regs[name]['status'],'not_documented_for_model')
                self.assertIsNone(regs[name]['value'])
                self.assertEqual(self.lib.fake_register_reads(0,reg),0)
                s.stop()
                with self.assertRaisesRegex(RuntimeError,'rejected command 9'):
                    self.bus._request(9,struct.pack('<BHB',1,1,reg))
        finally:s.cleanup()
        self.assertEqual(self.lib.fake_parameter_writes(),0)

    def test_control_reads_require_new_capability_and_configured_route(self):
        self.bus.capabilities &= ~CAP_CONTROL_READBACK
        with patch.object(self.bus,'_request',wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError,'0x4000'):self.bus.read_register(2,2,59)
            request.assert_not_called()
        self.bus.capabilities |= CAP_CONTROL_READBACK
        with self.assertRaisesRegex(RuntimeError,'configured disabled'):self.bus.read_register(2,2,59)
        for reg in CONTROL_REGISTERS:
            if reg==31:continue
            with self.subTest(register=reg), self.assertRaisesRegex(RuntimeError,'rejected command 9'):
                self.bus._request(9,struct.pack('<BHB',2,2,reg))
        self.assertEqual(self.lib.fake_register_reads(1,59),0)
        s=self.session(plan()['motors'])
        with self.assertRaisesRegex(RuntimeError,'all selected disabled routes'):s.inspect_control()

    def test_control_reads_reject_active_or_stale_other_routes_without_can_read(self):
        s=self.enable_all()
        try:
            with self.assertRaisesRegex(RuntimeError,'rejected command 9'):
                self.bus.read_register(2,2,59)
            self.assertEqual(self.lib.fake_register_reads(1,59),0)
            s.stop();self.lib.fake_advance(101)
            with self.assertRaisesRegex(RuntimeError,'rejected command 9'):
                self.bus.read_register(2,2,59)
            self.assertEqual(self.lib.fake_register_reads(1,59),0)
            s.stop()
            self.assertIsNotNone(self.bus.read_register(2,2,59))
            for reg in (1,3,25,27,59):
                with self.assertRaises(ValueError):self.bus.write_routed_bench(1,reg,1)
                with self.assertRaisesRegex(RuntimeError,'rejected command 17'):
                    self.bus._request(17,struct.pack('<BBI',1,reg,1))
        finally:s.cleanup()

    def test_control_inspection_missing_and_nonfinite_remain_unavailable(self):
        s=self.session(plan()['motors']);s.probe_disabled()
        self.lib.fake_drop_register(1,59)
        self.lib.fake_register(2,25,0x7fc00000)
        try:
            with self.assertRaisesRegex(RuntimeError,'readback incomplete'):s.inspect_control()
            self.assertEqual(len(s.report['control_registers']),6)
            missing=s.report['control_registers'][1]['registers']['Imax']
            invalid=s.report['control_registers'][2]['registers']['KP_ASR']
            self.assertEqual(missing['status'],'no_reply');self.assertIsNone(missing['value'])
            self.assertEqual(invalid['status'],'nonfinite');self.assertIsNone(invalid['value'])
            self.assertEqual(invalid['raw_hex'],'7fc00000')
            json.dumps(s.report,allow_nan=False)
            self.assertFalse(s.report['control_readback_complete'])
        finally:s.cleanup()
        self.assertEqual(self.lib.fake_parameter_writes(),0)

    def test_control_read_preserves_fault_latch(self):
        s=self.enable_all();self.lib.fake_advance(200);s.stop()
        try:
            before=self.bus.fault_snapshot()
            self.assertTrue(before['fault'])
            self.assertIsNotNone(self.bus.read_register(2,2,59))
            self.assertEqual(self.bus.fault_snapshot(),before)
        finally:s.cleanup()

    def test_control_read_rejects_id_session_and_coasting_other_motor(self):
        s=self.session(plan()['motors']);s.probe_disabled()
        self.lib.fake_coast(0,20);self.bus.stop()
        with self.assertRaisesRegex(RuntimeError,'rejected command 9'):
            self.bus.read_register(2,2,59)
        self.assertEqual(self.lib.fake_register_reads(1,59),0)
        self.lib.fake_coast(0,0);s.cleanup()
        # Separate explicit ID-only session, with one motor configured.
        self.bus.close();self.bus=ArmBus(transport=NativeTransport(self.lib))
        s=self.session([plan()['motors'][0]]);s.probe_disabled()
        try:
            self.bus.id_operation(0)
            with self.assertRaisesRegex(RuntimeError,'rejected command 9'):
                self.bus.read_register(1,1,3)
            self.assertEqual(self.lib.fake_register_reads(0,3),0)
        finally:s.cleanup()

    def test_control_inspection_wrong_identity_preserves_partial_evidence(self):
        s=self.session(plan()['motors']);s.probe_disabled()
        original=self.bus.read_register
        def read(bus,mid,reg):
            return (999,0) if mid==2 and reg==59 else original(bus,mid,reg)
        try:
            with patch.object(self.bus,'read_register',side_effect=read):
                with self.assertRaisesRegex(RuntimeError,'wrong feedback ID for Imax'):s.inspect_control()
            self.assertFalse(s.report['control_readback_complete'])
            self.assertEqual(s.report['control_registers'][1]['registers']['Imax']['status'],'wrong_feedback_id')
        finally:s.cleanup()

    def test_control_cli_old_firmware_fails_before_configuring_or_reading_motors(self):
        self.bus.capabilities &= ~CAP_CONTROL_READBACK
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'plan.json';path.write_text(json.dumps(plan()))
            report=Path(directory)/'report.json'
            with patch.object(bench,'ArmBus',return_value=self.bus), \
                 patch.object(self.bus,'_request',wraps=self.bus._request) as request, \
                 contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(bench.main(['--plan',str(path),'--all','--check','--feedback',
                    '--supported','--inspect-control','--report',str(report)]),2)
                request.assert_not_called()
            self.assertIn('0x4000',json.loads(report.read_text())['error'])

    def manual_port(self, *, speed_trip=None):
        from motors import arm_position as pos
        p=json.loads((ROOT/'src/robot_description/config/arm_rig_a2_a6.json').read_text())
        p.update(fixture_confirmed=True,supported_confirmed=True)
        if speed_trip is not None:
            for joint in p['active_joints']:p['motors'][joint-1]['speed_trip_rad_s']=speed_trip
        self.lib.fake_missing(0)
        return pos.ArmPositionPort(p,bus=self.bus,window_degrees=None)

    def check_a6_speed_quantization(self, injected, expected):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            # Preserve reproduction of the earlier 0.3 rad/s incident.
            port=self.manual_port(speed_trip=.3)
            try:
                port.select(6);port.arm();port.jog(1 if injected>0 else -1,math.radians(5))
                self.lib.fake_speed(5,.28 if injected>0 else -.28);self.lib.fake_advance(2)
                port.poll();self.assertTrue(port.moving)
                self.lib.fake_speed(5,injected);self.lib.fake_advance(2)
                with self.assertRaisesRegex(RuntimeError,'commanded 0.0873 rad/s'):port.poll()
                f=port.report['fault_snapshot']
                self.assertEqual((f['slot'],f['reason']),(5,8))
                self.assertAlmostEqual(f['velocity_rad_s'],expected,places=6)
                self.assertAlmostEqual(f['command_velocity_rad_s'],math.radians(5),places=7)
                self.assertAlmostEqual(f['speed_limit_rad_s'],.3,places=7)
                self.assertEqual(self.lib.fake_status(5),0)
                self.assertFalse(port.moving or port.armed)
            finally:port.close()

    def test_a6_positive_speed_trip_uses_verified_vmax_and_first_bin_above_limit(self):
        self.check_a6_speed_quantization(.31,.3003654479980469)

    def test_a6_negative_speed_trip_uses_verified_vmax_and_first_bin_above_limit(self):
        self.check_a6_speed_quantization(-.30,-.3003654479980469)

    def test_rig_all_active_joints_share_speed_trip_across_host_and_firmware(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time), tempfile.TemporaryDirectory() as directory:
            port=self.manual_port()
            try:
                self.assertEqual(port.available_joints,(2,3,4,5,6))
                for joint in port.available_joints:
                    slot=joint-1
                    for direction in (1,-1):
                        with self.subTest(joint=joint,direction=direction):
                            port.select(joint);port.arm();port.jog(direction,math.radians(5))
                            # Firmware and host must both accept the old A6
                            # incident values, without increasing commanded speed.
                            for speed in (.3151,.5,1.45):
                                self.lib.fake_speed(slot,direction*speed);self.lib.fake_advance(2)
                                self.assertTrue(port.poll());self.assertFalse(self.bus.state(slot).fault)
                                self.assertAlmostEqual(port.speed,math.radians(5),places=7)
                            self.lib.fake_speed(slot,direction*1.6);self.lib.fake_advance(2)
                            with self.assertRaisesRegex(RuntimeError,'speed limit'):port.poll()
                            fault=port.report['fault_snapshot']
                            self.assertEqual((fault['slot'],fault['reason']),(slot,8))
                            self.assertAlmostEqual(fault['speed_limit_rad_s'],1.5,places=7)
                            self.assertAlmostEqual(fault['command_velocity_rad_s'],math.radians(5),places=7)
                            self.assertEqual(self.lib.fake_status(slot),0)
                            self.lib.fake_speed(slot,0)
                            port.recover(Path(directory)/'speed-trip.json')
                            self.assertFalse(port.armed or port.moving)
                self.assertEqual(self.lib.fake_commands(0),0)
            finally:port.close()

    def trip_position_port(self, port, reason=9):
        port.select(3);port.arm();port.jog(1,math.radians(5))
        if reason==9:self.lib.fake_torque(2,12.1)
        else:self.lib.fake_speed(2,port.motors[2]['speed_trip_rad_s']+.2)
        self.lib.fake_advance(2)
        with self.assertRaises(RuntimeError):port.poll()
        self.assertEqual(port.report['fault_snapshot']['reason'],reason)
        self.lib.fake_torque(2,0);self.lib.fake_speed(2,0)

    def test_position_recovery_repeated_torque_speed_faults_keep_modes_and_origins(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time), tempfile.TemporaryDirectory() as directory:
            port=self.manual_port()
            origins=dict(port.session.origins);bounds=dict(port.session.bounds)
            try:
                for reason in (9,8):
                    self.trip_position_port(port,reason)
                    original=port.report['fault_snapshot'].copy()
                    before=[self.lib.fake_commands(i) for i in range(6)]
                    with patch.object(self.bus,'enable_bench_group',wraps=self.bus.enable_bench_group) as enable:
                        port.recover(Path(directory)/'session.json')
                        enable.assert_not_called()
                    self.assertEqual([self.lib.fake_commands(i) for i in range(6)],before)
                    self.assertEqual(port.session.origins,origins);self.assertEqual(port.session.bounds,bounds)
                    self.assertEqual(port.selected,3);self.assertFalse(port.armed or port.moving or port.recovery_required)
                    self.assertIsNone(port.target);self.assertNotIn('error',port.report)
                    attempt=port.report['recoveries'][-1]
                    self.assertTrue(attempt['success']);self.assertTrue(attempt['recovery']['reset_verified'])
                    evidence=json.loads(Path(attempt['recovery']['evidence_path']).read_text())
                    self.assertEqual(evidence['failure_before']['fault_snapshot'],original)
                    self.assertEqual(evidence['recovery']['fault_before'],original)
                    self.assertEqual([self.bus.state(i).registers for i in port.slots],[31]*5)
                    with self.assertRaisesRegex(RuntimeError,'Arm selected'):port.jog(1,math.radians(5))
                self.assertEqual(len(list(Path(directory).glob('*.fault-*.json'))),2)
                port.arm();port.jog(-1,math.radians(5));self.assertTrue(port.moving);port.release()
            finally:port.close()
            self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
            self.assertTrue(port.report['mode_restoration_verified'])

    def test_position_startup_fault_is_explicit_opt_in_and_recovers_disabled(self):
        from motors import arm_position as pos
        p=self.latch_recovery_fault()
        original=self.bus.fault_snapshot()
        with patch.object(pos,'time',bench.time), tempfile.TemporaryDirectory() as directory:
            with patch.object(self.bus,'close') as close:
                with self.assertRaises(bench.LatchedControllerFault):
                    pos.ArmPositionPort(p,bus=self.bus,window_degrees=None)
                close.assert_called_once()
            port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=None,allow_faulted_start=True)
            try:
                self.assertTrue(port.recovery_required);self.assertIsNone(port.session)
                self.assertEqual(port.report['fault_snapshot'],original)
                with self.assertRaisesRegex(RuntimeError,'Recovery required'):port.arm()
                with patch.object(self.bus,'enable_bench_group') as enable:
                    port.recover(Path(directory)/'startup.json');enable.assert_not_called()
                self.assertFalse(port.armed or port.moving or port.recovery_required)
                self.assertEqual([row.joint for row in port.positions()],[2,3,4,5,6])
                port.arm();port.jog(1,.05);self.assertTrue(port.moving);port.release()
            finally:port.close()
            self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)

    def test_position_recovery_failure_preserves_latch_then_allows_explicit_retry(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time), tempfile.TemporaryDirectory() as directory:
            port=self.manual_port()
            try:
                self.trip_position_port(port)
                with patch.object(bench.os,'fsync',side_effect=OSError('disk failure')), \
                     patch.object(self.bus,'reset_fault',wraps=self.bus.reset_fault) as reset:
                    with self.assertRaisesRegex(RuntimeError,'disk failure'):port.recover(Path(directory)/'session.json')
                    reset.assert_not_called()
                self.assertTrue(port.recovery_required);self.assertTrue(self.bus.state(2).fault)
                self.assertFalse(port.armed or port.moving)
                with self.assertRaisesRegex(RuntimeError,'Recovery required'):port.arm()
                port.recover(Path(directory)/'session.json')
                self.assertEqual([a['success'] for a in port.report['recoveries']],[False,True])
            finally:port.close()

    def test_position_startup_recovery_retries_partial_mode_preparation(self):
        from motors import arm_position as pos
        p=self.latch_recovery_fault()
        with patch.object(pos,'time',bench.time), tempfile.TemporaryDirectory() as directory:
            port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=None,allow_faulted_start=True)
            write=self.bus.write_routed_bench
            def lost_mode_ack(slot,reg,value):
                write(slot,reg,value)
                if reg==10:raise TimeoutError('lost mode ACK')
            try:
                with patch.object(self.bus,'write_routed_bench',side_effect=lost_mode_ack):
                    with self.assertRaisesRegex(RuntimeError,'lost mode ACK'):port.recover(Path(directory)/'session.json')
                self.assertTrue(port.recovery_required);self.assertFalse(port.armed or port.moving)
                port.recover(Path(directory)/'session.json')
                self.assertFalse(port.recovery_required)
                self.assertEqual([self.lib.fake_mode(i) for i in port.slots],[2]*5)
            finally:port.close()
            self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)

    def test_faulted_start_option_still_rejects_an_active_owner(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            port=self.manual_port()
            try:
                port.arm();port.jog(1,math.radians(5))
                p=json.loads((ROOT/'src/robot_description/config/arm_rig_a2_a6.json').read_text())
                p.update(fixture_confirmed=True,supported_confirmed=True)
                with patch.object(self.bus,'close'), patch.object(self.bus,'reset_fault') as reset:
                    with self.assertRaisesRegex(RuntimeError,'active or faulted') as caught:
                        pos.ArmPositionPort(p,bus=self.bus,window_degrees=None,allow_faulted_start=True)
                    self.assertNotIsInstance(caught.exception,bench.LatchedControllerFault)
                    reset.assert_not_called()
            finally:port.close()

    def test_position_recovery_lost_reset_ack_can_retry_with_invalidated_cache(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time), tempfile.TemporaryDirectory() as directory:
            port=self.manual_port()
            try:
                self.trip_position_port(port)
                reset=self.bus.reset_fault
                def lost_ack():
                    reset();raise TimeoutError('lost reset ACK')
                with patch.object(self.bus,'reset_fault',side_effect=lost_ack) as reset_call:
                    with self.assertRaisesRegex(RuntimeError,'lost reset ACK'):port.recover(Path(directory)/'session.json')
                    reset_call.assert_called_once()
                self.assertTrue(port.recovery_required)
                with patch.object(self.bus,'reset_fault',wraps=reset) as reset_call:
                    port.recover(Path(directory)/'session.json');reset_call.assert_not_called()
                self.assertFalse(port.recovery_required or port.armed or port.moving)
                self.assertEqual([self.bus.state(i).registers for i in port.slots],[31]*5)
            finally:port.close()

    def test_position_recovery_preserves_bounded_window_after_disabled_drift(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time), tempfile.TemporaryDirectory() as directory:
            p=plan();port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=3)
            try:
                origins=dict(port.session.origins);bounds=dict(port.session.bounds)
                port.capture_failure(RuntimeError('host interruption'))
                self.lib.fake_position(0,origins[0]+.3)
                with patch.object(self.bus,'reset_fault') as reset:
                    port.recover(Path(directory)/'session.json');reset.assert_not_called()
                self.assertEqual(port.session.origins,origins);self.assertEqual(port.session.bounds,bounds)
                port.arm()
                with patch.object(self.bus,'enable_bench_group') as enable:
                    with self.assertRaisesRegex(RuntimeError,'outside the startup corridor'):
                        port.move_to(origins[0],.1)
                    enable.assert_not_called()
            finally:port.close()

    def test_position_recovery_rejects_moving_and_missing_retained_motor(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time), tempfile.TemporaryDirectory() as directory:
            port=self.manual_port()
            try:
                port.arm();port.jog(1,math.radians(5))
                with patch.object(self.bus,'reset_fault') as reset:
                    with self.assertRaisesRegex(RuntimeError,'Release motion'):port.recover(Path(directory)/'session.json')
                    reset.assert_not_called()
                port.release();self.trip_position_port(port)
                self.lib.fake_missing(5)
                with patch.object(self.bus,'reset_fault') as reset:
                    with self.assertRaisesRegex(RuntimeError,'disable acknowledgement'):port.recover(Path(directory)/'session.json')
                    reset.assert_not_called()
                self.assertTrue(port.recovery_required);self.assertTrue(self.bus.state(2).fault)
                with self.assertRaisesRegex(RuntimeError,'Disable not confirmed'):port.close()
            finally:
                if not port.closed:port.close()

    def test_manual_travel_passes_old_bounds_both_directions_and_uses_current_preload(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            port=self.manual_port()
            try:
                for joint in (2,3,4):
                    slot=joint-1;origin=port.session.origins[slot]
                    self.lib.fake_position(slot,origin-math.radians(8.8))
                    for direction in (1,-1):
                        port.select(joint);port.arm()
                        before=port.positions()[slot-1].position_rad
                        commands=[self.lib.fake_commands(i) for i in range(6)]
                        port.jog(direction,math.radians(5))
                        start=bench.time.monotonic()
                        while bench.time.monotonic()-start<4:
                            self.lib.fake_advance(20);self.assertTrue(port.poll())
                        after=self.bus.state(slot).position
                        self.assertGreater(direction*(after-before),math.radians(18))
                        self.assertFalse(self.bus.state(slot).fault)
                        for i in range(6):
                            if i!=slot:self.assertEqual(self.lib.fake_commands(i),commands[i])
                        port.release();self.assertFalse(port.moving);self.assertTrue(port.armed)
                        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
                    self.assertEqual(port.session.origins[slot],origin)
                    self.assertEqual(port.session.bounds[slot],(-12.5,12.5))
                self.assertEqual(port.report['travel_policy'],'manual')
                self.assertFalse(any(e['event']=='reached' for e in port.report['events']))
            finally:port.close()
        self.assertTrue(port.report['mode_restoration_verified'])
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_manual_stall_keeps_advancing_target_then_stops_at_twelve_seconds(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            port=self.manual_port()
            try:
                port.arm();self.lib.fake_stall(1);self.lib.fake_torque(1,2.)
                port.jog(1,math.radians(5));origin=port.jog_origin;start=bench.time.monotonic()
                with self.assertRaisesRegex(RuntimeError,'Manual jog host duration limit reached'):
                    for _ in range(650):
                        self.lib.fake_advance(20);port.poll()
                self.assertGreater(bench.time.monotonic()-start,12)
                self.assertLess(bench.time.monotonic()-start,13)
                self.assertGreater(port.report['motion_summary']['2']['last_sample']['target_rad']-origin,math.radians(55))
                self.assertFalse(port.report['fault_snapshot']['fault'])
                self.assertFalse(port.armed);self.assertFalse(port.moving)
                self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            finally:port.close()

    def test_manual_old_firmware_fails_before_motor_operations(self):
        from motors.arm import CAP_MANUAL_TRAVEL
        self.bus.capabilities &= ~CAP_MANUAL_TRAVEL
        with patch.object(self.bus,'_request',wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError,'0x8000'):self.manual_port()
            request.assert_not_called()
        self.assertEqual(self.lib.fake_parameter_writes(),0)
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)],[0]*6)

    def test_manual_intent_cancellation_during_enable_and_poll_stops(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            port=self.manual_port();allowed=[True];port.motion_allowed=lambda:allowed[0]
            try:
                port.arm()
                enable=self.bus.enable_bench_group
                def cancel(slots):
                    enable(slots);allowed[0]=False
                with patch.object(self.bus,'enable_bench_group',side_effect=cancel):
                    with self.assertRaises(pos.MotionCancelled):port.jog(1,math.radians(5))
                self.assertFalse(port.armed);self.assertEqual(self.lib.fake_status(1),0)
                allowed[0]=True;port.arm();port.jog(-1,math.radians(5))
                allowed[0]=False
                with self.assertRaises(pos.MotionCancelled):port.poll()
                self.assertFalse(port.armed);self.assertEqual(self.lib.fake_status(1),0)
            finally:port.close()

    def test_manual_keeps_torque_trip_and_fault_evidence(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            port=self.manual_port()
            try:
                port.arm();port.jog(1,math.radians(5))
                self.lib.fake_torque(1,12.1);self.lib.fake_advance(2)
                with self.assertRaisesRegex(RuntimeError,'measured torque limit'):port.poll()
                self.assertEqual(port.report['fault_snapshot']['reason'],9)
                self.assertEqual(self.lib.fake_status(1),0)
            finally:port.close()

    def test_manual_protocol_boundary_is_not_silently_clipped(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            port=self.manual_port()
            try:
                self.lib.fake_position(1,12.45);port.arm();port.jog(1,math.radians(5))
                with self.assertRaisesRegex(RuntimeError,'protocol position range'):
                    for _ in range(40):self.lib.fake_advance(20);port.poll()
                self.assertFalse(port.moving);self.assertFalse(port.armed)
                self.assertEqual(self.lib.fake_status(1),0)
            finally:port.close()

    def test_firmware_manual_flag_keeps_mode_validation_command_lease_and_duration(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            port=self.manual_port()
            try:
                # The flag is a USB policy bit; it must never select motor mode 129.
                with self.assertRaises(RuntimeError):
                    self.bus._request(16,struct.pack('<BBHHBB4f',1,2,2,18,2,0x81,-12.5,12.5,.3,12.))
                port.arm();port.jog(1,math.radians(5))
                # Exercise firmware's independent 15 s stop while renewing the lease.
                with self.assertRaises(RuntimeError):
                    for _ in range(1600):
                        self.lib.fake_advance(10)
                        self.bus.command(1,port.target,velocity=math.radians(5))
                self.assertEqual(self.bus.fault_snapshot()['reason'],10)
                self.assertEqual(self.lib.fake_status(1),0)
            finally:port.close()

    def test_manual_lost_host_lease_still_latches_fault(self):
        from motors import arm_position as pos
        with patch.object(pos,'time',bench.time):
            port=self.manual_port()
            try:
                port.arm();port.jog(1,math.radians(5));self.lib.fake_advance(200)
                with self.assertRaises(RuntimeError):port.poll()
                self.assertEqual(port.report['fault_snapshot']['reason'],1)
                self.assertEqual(self.lib.fake_status(1),0)
            finally:port.close()

    def test_rig_has_no_hidden_two_or_three_nm_stop(self):
        from motors import arm_position as pos
        p=json.loads((ROOT/'src/robot_description/config/arm_rig_a2_a6.json').read_text())
        p.update(fixture_confirmed=True,supported_confirmed=True)
        self.lib.fake_missing(0)
        self.lib.fake_torque_limit.restype=ctypes.c_float
        with patch.object(pos,'time',bench.time):
            port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=3)
            try:
                for joint in (2,3,4):
                    slot=joint-1
                    self.assertEqual(self.lib.fake_torque_limit(slot),12. if joint<4 else 3.5)
                    port.select(joint);port.arm()
                    port.move_to(port.session.origins[slot]+math.radians(3),math.radians(5))
                    self.assertAlmostEqual(port.duration,2.6,delta=.01)
                    for torque in ((2.,3.1,6.,11.5) if joint<4 else (2.,3.4)):
                        self.lib.fake_torque(slot,torque);self.lib.fake_advance(2);port.poll()
                        self.assertTrue(port.moving);self.assertFalse(self.bus.state(slot).fault)
                    port.release();self.lib.fake_torque(slot,0)
                self.assertEqual(self.lib.fake_torque_limit(3),3.5)
            finally:port.close()
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_rig_two_nm_stall_ends_at_duration_not_torque_trip(self):
        from motors import arm_position as pos
        p=json.loads((ROOT/'src/robot_description/config/arm_rig_a2_a6.json').read_text())
        p.update(fixture_confirmed=True,supported_confirmed=True)
        self.lib.fake_missing(0)
        with patch.object(pos,'time',bench.time):
            port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=3)
            try:
                port.select(2);port.arm();self.lib.fake_stall(1);self.lib.fake_torque(1,2.)
                port.move_to(port.session.origins[1]+math.radians(3),math.radians(5))
                event=next(e for e in port.report['events'] if e['event']=='move')
                self.assertAlmostEqual(event['duration_limit_s'],2.6,delta=.01)
                with self.assertRaisesRegex(RuntimeError,'host duration limit reached'):
                    for _ in range(160):
                        self.lib.fake_advance(20);port.poll()
                self.assertFalse(port.report['fault_snapshot']['fault'])
                self.assertAlmostEqual(port.report['motion_summary']['2']['max_abs_torque_feedback_nm'],2.,delta=.02)
                self.assertFalse(port.moving)
            finally:port.close()
        self.assertTrue(port.report['disable_acknowledged'])

    def test_rig_check_with_a1_absent_reads_fresh_disabled_feedback_without_writes(self):
        p=plan();p.update(active_joints=[2,3,4,5,6],fixture_confirmed=False)
        self.lib.fake_missing(0)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'plan.json';path.write_text(json.dumps(p))
            report=Path(directory)/'result.json'
            with patch.object(bench,'ArmBus',return_value=self.bus), \
                 patch.object(self.bus,'read_register',wraps=self.bus.read_register) as read, \
                 patch.object(self.bus,'write_routed_bench',wraps=self.bus.write_routed_bench) as write, \
                 patch.object(self.bus,'enable_bench_group',wraps=self.bus.enable_bench_group) as enable:
                self.assertEqual(bench.main(['--plan',str(path),'--all','--check','--feedback',
                                             '--supported','--report',str(report)]),0)
                self.assertEqual({c.args[1] for c in read.call_args_list},{2,3,4,5,6})
                write.assert_not_called();enable.assert_not_called()
            result=json.loads(report.read_text())
            self.assertTrue(result['passed']);self.assertTrue(result['disable_acknowledged'])
            self.assertFalse(result['motor_parameter_writes'])
            self.assertEqual([r['joint'] for r in result['feedback']],[2,3,4,5,6])
            self.assertTrue(all(not r['enabled'] and r['age_ms']<80 for r in result['feedback']))
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)],[0]*6)
        self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_supported_trial_uses_existing_firmware_and_records_a6_response(self):
        from motors import arm_position as pos
        p=json.loads((ROOT/'src/robot_description/config/arm_rig_a2_a6.json').read_text())
        p.update(fixture_confirmed=True,supported_confirmed=True)
        self.lib.fake_missing(0)
        with patch.object(pos,'time',bench.time), pos.ArmPositionPort(
                p,bus=self.bus,window_degrees=None,speed_trial=True) as port:
            port.select(6)
            self.assertFalse(port.stationary_position().enabled)
            port.arm()
            with self.assertRaisesRegex(ValueError,'5 degrees'):port.move_to(.05,math.radians(10))
            with self.assertRaisesRegex(ValueError,'10 degrees'):port.jog(1,math.radians(11))
            for direction in (1,-1):
                for speed in (5,7.5,10):
                    port.jog(direction,math.radians(speed))
                    for _ in range(15):
                        self.lib.fake_advance(20)
                        self.assertTrue(port.poll())
                    port.release()
                    self.assertFalse(port.stationary_position().enabled)
            self.assertEqual([self.lib.fake_commands(i) for i in range(5)],[0]*5)
            summary=port.report['motion_summary']['6']
            self.assertGreater(summary['max_sample_gap_ms'],0)
            self.assertGreater(summary['max_abs_tracking_error_rad'],0)
            self.assertEqual(len([e for e in port.report['events'] if e['event']=='motion_stop']),6)
            self.lib.fake_stall(5)
            for direction in (1,-1):
                port.jog(direction,math.radians(10))
                initial=port.target
                for _ in range(30):
                    self.lib.fake_advance(20)
                    port.poll()
                self.assertAlmostEqual(port.target,initial,delta=.001)
                self.assertTrue(summary['last_sample']['lead_limited'])
                port.release()
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_teach_window_release_during_enable_cancels_before_goal_command(self):
        from PyQt5 import QtCore,QtTest,QtWidgets
        from motors import arm_position as pos
        from motors.arm_pose_teach import PoseTeach
        from teleop.arm_keyboard import make_window
        model=ROOT/'src/robot_description/mujoco/initial_pose.json'
        p=json.loads((ROOT/'src/robot_description/config/arm_rig_a2_a6.json').read_text())
        p.update(fixture_confirmed=True,supported_confirmed=True)
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as directory, patch.object(pos,'time',bench.time), \
             pos.ArmPositionPort(p,bus=self.bus,window_degrees=None) as port:
            teach=PoseTeach(Path(directory)/'teach.json',model,p,False,[])
            window=make_window(port,'FAKE CAN',speed_deg_s=5,teach=teach)
            try:
                window.show();app.processEvents()
                window.keys.press('6');window.keys.press('c')
                self.lib.fake_position(5,.16)
                self.lib.fake_advance(100)
                window.keys.press('t')
                self.assertAlmostEqual(teach.target(6),.16,delta=.001)
                self.lib.fake_position(5,.02);self.lib.fake_advance(100)
                window.keys.press('e')
                enable=self.bus.enable_bench_group
                def release_during_enable(slots):
                    enable(slots)
                    QtCore.QTimer.singleShot(0,lambda:QtTest.QTest.keyRelease(window,QtCore.Qt.Key_H))
                with patch.object(self.bus,'enable_bench_group',side_effect=release_during_enable), \
                     patch.object(self.bus,'command',wraps=self.bus.command) as commands:
                    QtTest.QTest.keyPress(window,QtCore.Qt.Key_H)
                    self.assertTrue(commands.called)
                    self.assertTrue(all(abs(c.args[1]-.02)<.001 for c in commands.call_args_list))
                self.assertFalse(port.moving or port.armed or window.keys.failed)
                self.assertFalse(self.bus.state(5).enabled)
                window.keys.press('e')
                QtTest.QTest.keyPress(window,QtCore.Qt.Key_H)
                self.assertTrue(port.moving)
                self.assertAlmostEqual(port.target,.02+math.radians(5),delta=.001)
                with patch.object(port,'positions',side_effect=AssertionError('extra display read')):
                    window.next_display=0;window.update_state()
                self.assertFalse(window.keys.failed)
                QtTest.QTest.keyRelease(window,QtCore.Qt.Key_H)
                self.assertFalse(self.bus.state(5).enabled)
            finally:
                window.close()

    def test_subset_jog_preserves_physical_slots_and_cleans_up(self):
        from motors import arm_position as pos
        p=plan();p.update(active_joints=[2,3,4,5,6],fixture_mode='supported',supported_confirmed=True)
        self.lib.fake_missing(0)
        with patch.object(pos,'time',bench.time):
            port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=3)
            try:
                self.assertEqual(port.selected,2)
                self.assertEqual([r.joint for r in port.positions()],[2,3,4,5,6])
                self.assertEqual(self.bus.state(0).registers,0)
                for joint in range(2,7):
                    before=[self.lib.fake_commands(i) for i in range(6)]
                    port.select(joint);port.arm()
                    with self.assertRaisesRegex(ValueError,'5 degrees'):
                        port.move_by(.01,.1)
                    port.move_by(.02,math.radians(5))
                    for _ in range(200):
                        self.lib.fake_advance(10)
                        if not port.poll():break
                    self.assertFalse(port.moving)
                    row=next(r for r in port.positions() if r.joint==joint)
                    self.assertAlmostEqual(row.position_rad,.02,delta=.01)
                    for i in range(6):
                        if i!=joint-1:self.assertEqual(self.lib.fake_commands(i),before[i])
                port.select(2);port.arm();port.move_by(.01,math.radians(5))
                with self.assertRaisesRegex(ValueError,'unavailable'):port.select(1)
                self.assertFalse(port.moving);self.assertFalse(port.armed)
            finally:port.close()
        self.assertEqual([self.lib.fake_mode(i) for i in range(6)],[1]*6)
        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
        self.assertEqual(self.lib.fake_invalid(),0)

    def test_subset_does_not_silently_skip_another_missing_motor(self):
        p=plan();p.update(active_joints=[2,3,4,5,6],fixture_confirmed=False)
        self.lib.fake_missing(0);self.lib.fake_missing(3)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'plan.json';path.write_text(json.dumps(p))
            report=Path(directory)/'result.json'
            with patch.object(bench,'ArmBus',return_value=self.bus), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(bench.main(['--plan',str(path),'--all','--check','--feedback',
                                             '--supported','--report',str(report)]),2)
            result=json.loads(report.read_text())
            self.assertFalse(result['passed']);self.assertIn('missing/wrong feedback',result['error'])
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)],[0]*6)

    def test_prior_six_motor_session_does_not_hide_missing_a1_or_reset(self):
        p=plan();session=self.session(p['motors'])
        session.prepare();session.cleanup()
        self.lib.fake_missing(0)
        with patch.object(self.bus,'reset_fault',wraps=self.bus.reset_fault) as reset:
            with self.assertRaisesRegex(TimeoutError,'disable acknowledgement'):
                bench.preflight(self.bus,p['motors'][1:])
            reset.assert_not_called()
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)],[0]*6)

    def latch_recovery_fault(self, *, full=False):
        p = plan()
        slots = list(range(6)) if full else list(range(1, 6))
        if not full:
            p['active_joints'] = [2, 3, 4, 5, 6]
            self.lib.fake_missing(0)
        observed = dict(zip(slots, bench.preflight(self.bus, [p['motors'][i] for i in slots])))
        session = bench.BenchSession(self.bus, p['motors'], observed, {}, slots=slots)
        session.prepare()
        session.command(1, session.stop()[1].position)
        self.bus.enable_bench_group([1])
        self.lib.fake_torque(1, .6)
        self.lib.fake_advance(2)
        self.assertEqual(self.bus.state(1).fault_reason, 9)
        session.cleanup()  # Restores actual mode 1; retained MCU routes still expect 2.
        self.lib.fake_torque(1, 0)
        return p

    def test_recovery_cli_preserves_fault_before_reset_without_enables_or_writes(self):
        p = self.latch_recovery_fault()
        self.bus.capabilities = 3983  # Deployed rig: no base_info or rated-torque extension.
        before = [self.lib.fake_commands(i) for i in range(6)]
        original = self.bus.fault_snapshot()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'plan.json'; path.write_text(json.dumps(p))
            report = Path(directory)/'result.json'
            reset = self.bus.reset_fault
            def checked_reset():
                files = list(Path(directory).glob('result.fault-*.json'))
                self.assertEqual(len(files), 1)
                evidence = json.loads(files[0].read_text())['recovery']
                self.assertEqual(evidence['fault_before'], original)
                self.assertFalse(evidence['reset_attempted'])
                self.assertEqual(len(evidence['disabled_before']), 5)
                self.assertTrue(all(not s['enabled'] and s['age_ms'] < 80
                                    for s in evidence['disabled_before'].values()))
                reset()
            with patch.object(bench, 'ArmBus', return_value=self.bus), \
                 patch.object(self.bus, 'reset_fault', side_effect=checked_reset) as reset_call, \
                 patch.object(self.bus, 'enable_bench_group', wraps=self.bus.enable_bench_group) as enable, \
                 patch.object(self.bus, 'write_routed_bench', wraps=self.bus.write_routed_bench) as write, \
                 patch.object(self.bus, 'base_info', wraps=self.bus.base_info) as info:
                result = bench.main(['--plan', str(path), '--all', '--check', '--feedback',
                                     '--supported', '--recover', '--report', str(report)])
                self.assertEqual(result, 0)
                reset_call.assert_called_once(); enable.assert_not_called()
                write.assert_not_called(); info.assert_not_called()
            result = json.loads(report.read_text())
            self.assertTrue(result['passed']); self.assertTrue(result['disable_acknowledged'])
            self.assertTrue(result['recovery']['reset_verified'])
            self.assertEqual(result['recovery']['fault_before'], original)
            self.assertEqual([r['joint'] for r in result['feedback']], [2,3,4,5,6])
            self.assertTrue(all(not r['enabled'] and r['age_ms'] < 80 for r in result['feedback']))
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)], before)
        self.assertEqual([self.lib.fake_mode(i) for i in range(6)], [1]*6)
        self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)
        self.assertEqual(self.lib.fake_invalid(), 0)

    def test_recovery_reprobes_reset_cache_and_repeated_checks_need_no_reset(self):
        p = self.latch_recovery_fault()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(self.bus, 'reset_fault', wraps=self.bus.reset_fault) as reset:
            report = {}; path = Path(directory)/'recovery.json'
            bench.recover_fault(self.bus, report, path)
            self.assertTrue(report['recovery']['reset_verified'])
            for _ in range(2):
                self.assertEqual(len(bench.preflight(self.bus, p['motors'][1:])), 5)
            second = {}; bench.recover_fault(self.bus, second, path)
            self.assertEqual(second['recovery']['outcome'], 'no_latched_fault')
            reset.assert_called_once()
            self.assertEqual(len(list(Path(directory).glob('*.fault-*.json'))), 1)

    def test_recovery_missing_retained_motor_never_clears_fault(self):
        self.latch_recovery_fault(full=True)
        self.lib.fake_missing(0)  # Next plan omits A1; old MCU session still owns it.
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(self.bus, 'reset_fault', wraps=self.bus.reset_fault) as reset:
            with self.assertRaisesRegex(TimeoutError, 'disable acknowledgement'):
                bench.recover_fault(self.bus, {}, Path(directory)/'result.json')
            reset.assert_not_called()
        self.assertTrue(self.bus.state(1).fault)

    def test_recovery_coasting_or_drive_error_never_clears_fault(self):
        self.latch_recovery_fault()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(self.bus, 'reset_fault', wraps=self.bus.reset_fault) as reset:
            self.lib.fake_coast(1, 10000)
            with self.assertRaisesRegex(TimeoutError, 'stationary disable acknowledgement'):
                bench.recover_fault(self.bus, {}, Path(directory)/'result.json')
            self.lib.fake_coast(1, 0)
            state = self.bus.state
            def drive_error(i):
                s = state(i)
                return replace(s, status=8) if i == 1 else s
            with patch.object(self.bus, 'state', side_effect=drive_error), \
                 self.assertRaisesRegex(TimeoutError, 'disable acknowledgement'):
                bench.recover_fault(self.bus, {}, Path(directory)/'result.json')
            reset.assert_not_called()
        self.assertTrue(self.bus.state(1).fault)

    def test_recovery_diagnostic_or_evidence_failure_never_clears_fault(self):
        self.latch_recovery_fault()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(self.bus, 'reset_fault', wraps=self.bus.reset_fault) as reset:
            with patch.object(self.bus, 'capabilities', self.bus.capabilities & ~512), \
                 self.assertRaisesRegex(RuntimeError, 'requires routed bench and first-fault'):
                bench.recover_fault(self.bus, {}, Path(directory)/'result.json')
            with patch.object(self.bus, 'fault_snapshot', return_value=None), \
                 self.assertRaisesRegex(RuntimeError, 'consistent retained fault'):
                bench.recover_fault(self.bus, {}, Path(directory)/'result.json')
            with patch.object(bench.os, 'fsync', side_effect=OSError('disk failure')), \
                 self.assertRaisesRegex(OSError, 'disk failure'):
                bench.recover_fault(self.bus, {}, Path(directory)/'result.json')
            reset.assert_not_called()
        self.assertTrue(self.bus.state(1).fault)

    def test_recovery_reset_rejection_preserves_latch_and_does_not_claim_success(self):
        self.latch_recovery_fault()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(self.bus, 'reset_fault', side_effect=RuntimeError('reset rejected')) as reset:
            report = {}
            with self.assertRaisesRegex(RuntimeError, 'reset rejected'):
                bench.recover_fault(self.bus, report, Path(directory)/'result.json')
            reset.assert_called_once()
            self.assertTrue(report['recovery']['reset_attempted'])
            self.assertFalse(report['recovery']['reset_acknowledged'])
            self.assertFalse(report['recovery']['reset_verified'])
            self.assertTrue(self.bus.state(1).fault)

    def test_recovery_lost_reset_ack_does_not_retry_and_next_explicit_check_repairs_cache(self):
        p = self.latch_recovery_fault()
        self.bus.capabilities = 3983
        reset = self.bus.reset_fault
        def lost_ack():
            reset()
            raise TimeoutError('lost reset ACK')
        with tempfile.TemporaryDirectory() as directory:
            report = {}; path = Path(directory)/'result.json'
            with patch.object(self.bus, 'reset_fault', side_effect=lost_ack) as reset_call, \
                 self.assertRaisesRegex(TimeoutError, 'lost reset ACK'):
                bench.recover_fault(self.bus, report, path)
            reset_call.assert_called_once()
            self.assertTrue(report['recovery']['reset_attempted'])
            self.assertFalse(report['recovery']['reset_acknowledged'])
            self.assertFalse(report['recovery']['reset_verified'])
            with patch.object(self.bus, 'reset_fault', wraps=reset) as second_reset:
                next_report = {}; bench.recover_fault(self.bus, next_report, path)
                self.assertEqual(next_report['recovery']['outcome'], 'no_latched_fault')
                self.assertEqual(len(bench.preflight(self.bus, p['motors'][1:])), 5)
                second_reset.assert_not_called()

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
        # 4340P torque is rated; its existing 1.5 rad/s speed ceiling stays lower.
        for index, rpm, torque in ((0, 100, 20.), (1, 1.5*60/(2*math.pi), 12.), (3, 120, 3.5)):
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
                                m['feedback_id'],1 if index==0 else 2 if index==1 else 3,2,-1,1,
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

    def test_4340_rated_threshold_requires_new_firmware_before_config_write(self):
        m = dict(plan()['motors'][1], torque_trip_nm=12)
        self.assertTrue(self.bus.capabilities & CAP_RATED_4340_TORQUE)
        self.bus.capabilities = 3983
        with patch.object(self.bus, '_request', wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError, 'old 3 Nm.*0x2000'):
                self.bus.prepare_routed_bench(1, m, -1, 1, 2)
            request.assert_not_called()
            self.assertFalse(self.bus.configured)
        m['torque_trip_nm'] = 3
        self.bus.prepare_routed_bench(1, m, -1, 1, 2)
        self.bus.stop()

    def test_4340_configured_threshold_is_enforced_below_rated_ceiling(self):
        for threshold in (3., 6., 12.):
            with self.subTest(threshold=threshold):
                self.bus.close(); self.bus = ArmBus(transport=NativeTransport(self.lib))
                m = dict(plan()['motors'][1], torque_trip_nm=threshold)
                session = self.session([m])
                try:
                    session.prepare(); state = session.stop()[0]
                    session.command(0, state.position); self.bus.enable_bench_group([0])
                    self.lib.fake_torque(1, threshold-.1); self.lib.fake_advance(2)
                    self.assertFalse(self.bus.state(0).fault)
                    self.lib.fake_torque(1, threshold+.1); self.lib.fake_advance(2)
                    self.assertEqual(self.bus.state(0).fault_reason, 9)
                    self.assertAlmostEqual(self.bus.fault_snapshot()['torque_limit_nm'], threshold)
                finally:
                    session.cleanup()
                self.assertEqual(self.lib.fake_status(1), 0)
                self.assertEqual(self.lib.fake_mode(1), 1)

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

    def test_disabled_drift_rejected_before_preload_or_enable_without_rebasing(self):
        from motors import arm_position as pos
        p=json.loads((ROOT/'src/robot_description/config/arm_rig_a2_a6.json').read_text())
        p.update(fixture_confirmed=True,supported_confirmed=True)
        self.lib.fake_missing(0)
        with patch.object(pos,'time',bench.time):
            port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=3)
            try:
                port.select(4);port.arm()
                origin=port.session.origins[3];bounds=port.session.bounds[3]
                self.lib.fake_position(3,origin-math.radians(8.8))
                with patch.object(self.bus,'command',wraps=self.bus.command) as command, \
                     patch.object(self.bus,'enable_bench_group',wraps=self.bus.enable_bench_group) as enable:
                    with self.assertRaisesRegex(RuntimeError,'A4: disabled joint shifted.*no enable sent'):
                        port.move_to(origin,math.radians(5))
                    command.assert_not_called();enable.assert_not_called()
                self.assertEqual(port.session.origins[3],origin)
                self.assertEqual(port.session.bounds[3],bounds)
                self.assertFalse(port.report['start_rejection']['enable_sent'])
                self.assertFalse(port.armed);self.assertFalse(self.bus.state(3).fault)
            finally:port.close()
        self.assertTrue(port.report['disable_acknowledged'])
        self.assertTrue(port.report['mode_restoration_verified'])

    def test_position_port_reports_a2_torque_trip_after_command_rejection(self):
        from motors import arm_position as pos
        p=json.loads((ROOT/'src/robot_description/config/arm_rig_a2_a6.json').read_text())
        p.update(fixture_confirmed=True,supported_confirmed=True)
        self.lib.fake_missing(0)
        with patch.object(pos,'time',bench.time):
            port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=3)
            try:
                port.select(2);port.arm()
                port.move_to(port.positions()[0].upper_rad,math.radians(5))
                trip=p['motors'][1]['torque_trip_nm']
                # Exercise the actual rig profile above the former 3 Nm trip,
                # then verify its configured rated threshold still latches.
                self.lib.fake_torque(1,trip/2);self.lib.fake_advance(2)
                port.poll()
                self.assertTrue(port.moving)
                self.assertFalse(self.bus.state(1).fault)
                self.assertAlmostEqual(port.report['motion_summary']['2']['max_abs_torque_feedback_nm'],trip/2,delta=.02)
                self.assertEqual(port.report['control_mode'],'position_velocity')
                self.assertIsNone(port.report['commanded_torque_nm'])
                self.assertEqual(port.report['session_joints'][0]['verified_motion_mode'],2)
                last=port.report['motion_summary']['2']['last_sample']
                self.assertTrue(last['enabled']);self.assertTrue(last['enable_confirmed'])
                self.assertEqual(last['torque_trip_nm'],trip)
                self.lib.fake_torque(1,trip+.05);self.lib.fake_advance(2)
                with patch.object(self.bus,'_request',wraps=self.bus._request) as request:
                    with self.assertRaisesRegex(RuntimeError,'A2: measured torque limit'):
                        port.poll()
                    kinds=[c.args[0] for c in request.call_args_list]
                self.assertEqual(kinds[0],4)  # Original rejection reproduced.
                self.assertLess(kinds.index(6),kinds.index(19))  # Stop before diagnostics.
                self.assertTrue(set(kinds)<= {4,6,7,19})  # No reset, enable, probe or parameter writes.
                self.assertEqual(kinds.count(4),1)
                fault=port.report['fault_snapshot']
                self.assertEqual((fault['reason'],fault['slot']),(9,1))
                self.assertGreater(fault['torque_nm'],trip)
                self.assertLess(fault['torque_nm'],trip+.07)
                self.assertAlmostEqual(fault['torque_limit_nm'],trip)
                self.assertIn('MC02 rejected command 4',port.report['error_original'])
                self.assertIn(f'limit {trip:.4f} Nm',port.report['error'])
                self.assertIn('explicit recovery',port.report['error'])
                self.assertEqual(set(port.report['failure_states']),{1,2,3,4,5})
                self.assertFalse(port.armed);self.assertFalse(port.moving)
                # Cleanup/callback reporting must not overwrite first-fault evidence.
                first=port.report['error']
                self.assertEqual(port.capture_failure(RuntimeError('secondary')),first)
                self.assertEqual(self.bus.fault_snapshot(),fault)
                self.assertTrue(self.bus.state(1).fault)
            finally:port.close()
        self.assertTrue(port.report['disable_acknowledged'])
        self.assertTrue(port.report['mode_restoration_verified'])
        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
        self.assertEqual([self.lib.fake_mode(i) for i in range(1,6)],[1]*5)
        self.assertGreater(port.report['motion_summary']['2']['max_abs_torque_feedback_nm'],trip/2-.02)
        self.assertTrue(port.report['motion_samples'][0]['enabled'])
        json.dumps(port.report)  # Shutdown report remains serializable.

    def test_position_port_diagnostic_failure_preserves_original_and_stops(self):
        from motors import arm_position as pos
        p=plan()
        with patch.object(pos,'time',bench.time):
            port=pos.ArmPositionPort(p,bus=self.bus,window_degrees=3)
            try:
                port.select(2);port.arm();port.move_to(port.positions()[1].upper_rad,.05)
                with patch.object(self.bus,'fault_snapshot',side_effect=OSError('USB read failed')):
                    message=port.capture_failure(RuntimeError('original transport failure'))
                self.assertIn('original transport failure',message)
                self.assertIn('USB read failed',message)
                self.assertEqual(port.report['error_original'],'original transport failure')
                self.assertTrue(port.report['failure_capture_errors'])
                self.assertFalse(port.armed);self.assertFalse(port.moving)
                self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            finally:port.close()

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
                self.assertNotIn('error',port.report)
                self.assertNotIn('fault_snapshot',port.report)
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
