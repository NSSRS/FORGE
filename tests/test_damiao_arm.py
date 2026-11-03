import copy
import json
from pathlib import Path
import struct
import unittest
import source_paths  # noqa
from motors.arm import ArmBus, decode, frame


class FakeUSB:
    def __init__(self): self.rx=bytearray(); self.sent=[]; self.closed=False
    def write(self,data):
        kind,seq,p=decode(data); self.sent.append((kind,p))
        reply=bytearray(52)
        if kind==1:
            reply[1:13]=b'FORGE-MC02/1'
            struct.pack_into('<I',reply,13,500)
        if kind==7: struct.pack_into('<7BI6f',reply,1,0,0,31,0,25,26,0,5,.1,.2,.3,12.5,30,40)
        self.rx.extend(frame(kind|0x80,seq,reply)); return len(data)
    def read(self,n):
        n=min(n,7)  # fragmented serial reads
        out=bytes(self.rx[:n]); del self.rx[:n]; return out
    def close(self): self.closed=True


class ArmTests(unittest.TestCase):
    def profile(self):
        return json.loads((Path(__file__).resolve().parents[1]/'src/robot_description/config/arm_motors.json').read_text())
    def test_template_cannot_enable(self):
        usb=FakeUSB(); bus=ArmBus(transport=usb)
        with self.assertRaises(ValueError): bus.configure(self.profile())
        self.assertEqual([x[0] for x in usb.sent],[1]); bus.close()
        self.assertTrue(usb.closed)
    def test_full_validation_before_write(self):
        usb=FakeUSB(); bus=ArmBus(transport=usb); p=self.profile(); p['hardware_confirmed']=True; p['joint_order_confirmed']=True
        for j in p['joints']:
            j['limits']=dict(lower_rad=-1,upper_rad=1,speed_rad_s=1,torque_nm=2,kp=5,kd=.2)
        bad=copy.deepcopy(p); bad['joints'][-1]['limits']['kp']=float('nan')
        with self.assertRaises(ValueError): bus.configure(bad)
        self.assertEqual(len(usb.sent),1)
        bad=copy.deepcopy(p); bad['joints'][-1]['can_id']=2
        with self.assertRaises(ValueError): bus.configure(bad)
        self.assertEqual(len(usb.sent),1)
        for key,value in [('can_update_hz',1000),('baud_rate',500000),('can_frame_format','can_fd')]:
            bad=copy.deepcopy(p); bad[key]=value
            with self.assertRaises(ValueError): bus.configure(bad)
        self.assertEqual(len(usb.sent),1)
        bus.configure(p); self.assertEqual(len(usb.sent),7)
        state=bus.state(0); self.assertAlmostEqual(state.position,.1); self.assertEqual(state.registers,31)
        bus.command(0,.1,kp=5,kd=.2)
        self.assertEqual(usb.sent[-1][0],4)
        with self.assertRaises(ValueError): bus.command(0,float('nan'))
        with self.assertRaises(ValueError): bus.state(-1)
        bus.close(); self.assertEqual(usb.sent[-1][0],6)
    def test_older_firmware_rate_is_rejected(self):
        class OldFirmware(FakeUSB):
            def write(self,data):
                count=super().write(data)
                kind,seq,p=decode(self.rx)
                p=bytearray(p); struct.pack_into('<I',p,13,200)
                self.rx=bytearray(frame(kind,seq,p))
                return count
        usb=OldFirmware()
        with self.assertRaisesRegex(RuntimeError,'500 Hz'): ArmBus(transport=usb)
        self.assertTrue(usb.closed)
        self.assertEqual([kind for kind,_ in usb.sent],[1])

    def test_discovery_requires_capability_without_can_commands(self):
        usb=FakeUSB()
        with ArmBus(transport=usb) as bus:
            with self.assertRaisesRegex(RuntimeError, 'updated ELF'):
                bus.read_register(2, 1, 8)
        self.assertEqual([kind for kind,_ in usb.sent], [1])

    def test_crc_and_wire(self):
        packet=frame(4,123,b'test'); self.assertEqual(len(packet),64)
        self.assertEqual(decode(packet)[:2],(4,123))
        corrupt=bytearray(packet); corrupt[20]^=1
        with self.assertRaises(ValueError): decode(corrupt)
        with self.assertRaises(ValueError): frame(1,1,b'a'*53)


class NativeTransport(FakeUSB):
    def __init__(self,lib):
        super().__init__(); self.lib=lib; lib.fake_init()
    def write(self,data):
        import ctypes
        output=ctypes.create_string_buffer(64)
        if not self.lib.fake_exchange(data,output): raise RuntimeError('native firmware did not reply')
        self.rx.extend(output.raw); return len(data)


class NativeRoundTrip(unittest.TestCase):
    def test_posvel_mode_readback_mismatch_blocks_enable(self):
        with ArmBus(transport=NativeTransport(self.lib)) as bus:
            bus.set_bench_timeout(); bus.prepare_bench(-.15,.94)
            bus.probe(0)
            for _ in range(10): bus.state(0)
            bus.stop(); state=bus.state(0)
            self.lib.fake_ignore_mode_write(1)
            with self.assertRaisesRegex(RuntimeError,'readback mismatch'):
                bus.set_bench_mode(2)
            bus.prepare_bench(-.15,.94,posvel=True)
            bus.probe(0)
            for _ in range(10): state=bus.state(0)
            self.assertNotEqual(state.registers,31)
            with self.assertRaises(RuntimeError): bus.enable(0)
            self.assertEqual(self.lib.fake_posvel_commands(),0)

    def test_posvel_commands_and_watchdog(self):
        with ArmBus(transport=NativeTransport(self.lib)) as bus:
            bus.set_bench_timeout(); bus.prepare_bench(-.15,.94)
            bus.probe(0)
            for _ in range(10): bus.state(0)
            bus.stop(); bus.state(0); bus.set_bench_mode(2)
            bus.prepare_bench(-.15,.94,posvel=True)
            bus.probe(0)
            for _ in range(10): bus.state(0)
            bus.stop(); state=bus.state(0)
            with self.assertRaises(RuntimeError): bus.command(0,state.position,velocity=.11)
            with self.assertRaises(RuntimeError): bus.command(0,state.position,velocity=.1,kp=2)
            bus.command(0,state.position,velocity=.1); bus.enable(0)
            self.assertTrue(bus.state(0).enabled)
            with self.assertRaises(RuntimeError): bus.set_bench_mode(1)
            with self.assertRaises(RuntimeError): bus.read_register(2,1,10)
            self.lib.fake_advance(160)
            state=bus.state(0)
            self.assertTrue(state.fault)
            self.assertEqual(state.fault_reason,1)
            self.assertFalse(state.enabled)
            self.assertGreater(self.lib.fake_posvel_commands(),0)
            self.assertEqual(self.lib.fake_invalid_commands(),0)

    def test_posvel_runner_reaches_target_and_restores_mode(self):
        self._bench_runner(120, motion=True)

    def test_posvel_stall_is_not_success_and_restores_mode(self):
        self._bench_runner(120)

    def _bench_runner(self, enable_delay, motion=False):
        import contextlib
        import importlib.util
        import io
        from types import SimpleNamespace
        from unittest.mock import patch
        path=Path(__file__).resolve().parents[1]/'scripts/test_dm_4340_bench.py'
        spec=importlib.util.spec_from_file_location('bench_runner',path)
        runner=importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
        bus=ArmBus(transport=NativeTransport(self.lib))
        self.lib.fake_set_enable_delay(enable_delay)
        self.lib.fake_set_motion(int(motion))
        clock=SimpleNamespace(monotonic=lambda:self.lib.fake_now()/1000.,
                              sleep=lambda seconds:self.lib.fake_advance(int(seconds*1000)))
        output=io.StringIO()
        with patch.object(runner,'time',clock), patch.object(runner,'ArmBus',lambda _:bus), \
             patch('sys.argv',['bench','--execute','--degrees','45']), contextlib.redirect_stdout(output):
            self.assertEqual(runner.main(),0 if motion else 1)
        self.assertIn('Motor enabled confirmed',output.getvalue())
        self.assertIn('Motor disable acknowledged',output.getvalue())
        self.assertIn('Target reached' if motion else 'little/no motion',output.getvalue())
        self.assertEqual(self.lib.fake_mode(),1)
        self.assertEqual(self.lib.fake_mode_writes(),2)
        self.assertGreater(self.lib.fake_posvel_commands(),0)
        self.assertEqual(self.lib.fake_invalid_commands(),0)
        self.assertTrue(bus.closed)

    def test_fault_snapshot_survives_stop(self):
        with ArmBus(transport=NativeTransport(self.lib)) as bus:
            bus.set_bench_timeout(); bus.prepare_bench(-.15,.15); bus.probe(0)
            for _ in range(10): bus.state(0)
            bus.stop(); state=bus.state(0)
            bus.set_bench_mode(2); bus.prepare_bench(-.15,.94,posvel=True); bus.probe(0)
            for _ in range(10): bus.state(0)
            bus.stop(); state=bus.state(0)
            bus.command(0,state.position,velocity=.1); bus.enable(0)
            self.lib.fake_advance(160)
            state=bus.state(0)
            self.assertTrue(state.fault)
            self.assertEqual(state.fault_reason,1)
            self.assertEqual(state.fault_status,1)
            self.assertGreaterEqual(state.fault_command_age_ms,150)
            self.assertIn('lease expired',state.fault_detail())
            bus.stop()
            self.assertEqual(bus.state(0).fault_reason,1)

    def test_read_only_discovery(self):
        with ArmBus(transport=NativeTransport(self.lib)) as bus:
            self.assertTrue(bus.capabilities & 1)
            self.assertEqual(bus.read_register(2, 1, 8), (0, 1))
            self.assertEqual(bus.read_register(2, 1, 7), (0, 0))
            self.assertIsNone(bus.read_register(2, 2, 8))
            self.assertFalse(bus.configured)
            for slot in range(6):
                state = bus.state(slot)
                self.assertFalse(state.enabled or state.stopping)
            with self.assertRaises(ValueError): bus.read_register(2, 1, 0x55)

    @classmethod
    def setUpClass(cls):
        import ctypes
        import shutil
        import subprocess
        import tempfile
        if not shutil.which('cc'): raise unittest.SkipTest('C compiler unavailable')
        cls.temp=tempfile.TemporaryDirectory(); root=Path(__file__).resolve().parents[1]
        so=Path(cls.temp.name)/'firmware.so'
        subprocess.run(['cc','-shared','-fPIC','-std=c11','-I'+str(root/'src/damiao_firmware/include'),
                        str(root/'src/damiao_firmware/src/arm.c'),str(root/'src/damiao_firmware/src/wire.c'),
                        str(root/'tests/damiao_test_bridge.c'),'-lm','-o',str(so)],check=True)
        cls.lib=ctypes.CDLL(str(so))
        cls.lib.fake_exchange.argtypes=[ctypes.c_char_p,ctypes.c_void_p]
        cls.lib.fake_exchange.restype=ctypes.c_int
        cls.lib.fake_advance.argtypes=[ctypes.c_uint]
    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()
    def test_six_joint_host_firmware_roundtrip_and_timeout(self):
        usb=NativeTransport(self.lib); bus=ArmBus(transport=usb)
        p=ArmTests().profile(); p['hardware_confirmed']=True; p['joint_order_confirmed']=True
        for j in p['joints']: j['limits']=dict(lower_rad=-1,upper_rad=1,speed_rad_s=1,torque_nm=2,kp=5,kd=.2)
        bus.configure(p)
        for i in range(6): bus.probe(i)
        self.lib.fake_advance(40)
        for i in range(6): self.assertEqual(bus.state(i).registers,31)
        bus.stop(); self.lib.fake_advance(10)
        for i in range(6):
            bus.command(i,0,kp=5,kd=.2); bus.enable(i)
        self.assertTrue(bus.state(5).enabled)
        # Polling state does not keep any motion command alive.
        self.lib.fake_advance(160)
        state=bus.state(0); self.assertTrue(state.fault); self.assertFalse(state.enabled)
        with self.assertRaises(RuntimeError): bus.enable(0)
        self.lib.fake_advance(10); bus.reset_fault()
        with self.assertRaises(RuntimeError): bus.command(0,0)
        bus.close()

    def test_500hz_on_temporary_and_final_bus_maps(self):
        for filename, expected in [('arm_motors.json',[1,2,2,2,2,2]),
                                   ('arm_motors_final.json',[1,2,2,3,3,3])]:
            with self.subTest(profile=filename):
                root=Path(__file__).resolve().parents[1]
                p=json.loads((root/'src/robot_description/config'/filename).read_text())
                self.assertEqual([j['bus'] for j in p['joints']],expected)
                self.assertFalse(p['hardware_confirmed'])
                self.assertEqual(p['can_update_hz'],500)
                p['hardware_confirmed']=True; p['joint_order_confirmed']=True
                for j in p['joints']:
                    j['limits']=dict(lower_rad=-1,upper_rad=1,speed_rad_s=1,torque_nm=2,kp=5,kd=.2)
                bus=ArmBus(transport=NativeTransport(self.lib)); bus.configure(p)
                for i in range(6): bus.probe(i)
                self.lib.fake_advance(40); bus.stop(); self.lib.fake_advance(10)
                for i in range(6):
                    bus.command(i,0,kp=5,kd=.2); bus.enable(i)
                for i in range(6): bus.command(i,0,kp=5,kd=.2)
                before=[self.lib.fake_commands(i) for i in range(6)]
                self.lib.fake_advance(100)
                self.assertEqual([self.lib.fake_commands(i)-before[i] for i in range(6)],[50]*6)
                self.assertFalse(bus.state(0).fault)
                bus.close()


class ROSCommandValidation(unittest.TestCase):
    def test_complete_fresh_motor_commands_only(self):
        from types import SimpleNamespace as NS
        from motor_control.arm_driver import validate_command
        names=[f'arm_joint_{i+1}' for i in range(6)]
        msg=NS(header=NS(stamp=NS(sec=10,nanosec=0)),name=names,position=[0.]*6,velocity=[0.]*6,effort=[0.]*6)
        self.assertEqual(validate_command(msg,names,10_050_000_000),10_000_000_000)
        for stamp in (9_999_999_999,10_100_000_001):
            with self.assertRaises(ValueError): validate_command(msg,names,stamp)
        msg.name=['chassis_motor']*6
        with self.assertRaises(ValueError): validate_command(msg,names,10_000_000_000)
        msg.name=names; msg.effort=[]
        with self.assertRaises(ValueError): validate_command(msg,names,10_000_000_000)
        msg.effort=[0.]*6; msg.position[0]=float('nan')
        with self.assertRaises(ValueError): validate_command(msg,names,10_000_000_000)


if __name__ == "__main__":
    unittest.main()
