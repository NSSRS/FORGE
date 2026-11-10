"""Pocket mapping, freshness, primer and worker boundaries; no hardware."""
import source_paths
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import patch
from motors.arm_position import SimulatedPositionPort
from teleop.pocket_input import PocketMapping, PocketCommand, ArmIntent
from teleop.pocket_arm import ArmJogRunner, ArmWorker, IntentMailbox
from teleop.pocket_calibration import axis_from_samples, switch_from_samples
from teleop.radio_stream import RadioStream

ROOT = Path(__file__).resolve().parents[1]


def config():
    # Synthetic fixture mapping, NEVER a claim about the operator's radio.
    cfg = json.loads((ROOT/'src/bringup/config/pocket_controls.json').read_text())
    cfg['arm']['cw_raw_signs'] = [0]*6  # Explicitly exercise unknown-sign simulation.
    for name, channel, sign in (('chassis_forward',3,1),('chassis_turn',4,-1)):
        cfg['axes'][name] = dict(channel=channel, minimum=174, center=992, maximum=1811,
                                 sign=sign, deadzone=.05)
    for name, channel in (('sb',5),('sc',6),('se',7)):
        cfg['switches'][name].update(channel=channel,
            values=[174,1811] if name == 'se' else [174,992,1811])
    return cfg


def row(*, x=992, y=992, forward=992, turn=992, sb=174, sc=174, se=174, age=0):
    return ','.join(map(str, [age,x,y,forward,turn,sb,sc,se]+[992]*9))


class PocketPolicyTests(unittest.TestCase):
    def test_missing_a1_cannot_prime_or_move_and_requires_fresh_selection(self):
        self.policy=PocketCommand(PocketMapping(config()),available_joints=(2,3,4,5,6))
        self.prime()
        self.assertEqual(self.send(sb=992,se=1811,y=1811).arm.direction,1)
        out=self.send(sb=174,se=1811,y=1811,forward=1811)
        self.assertEqual(out.arm.joint,1);self.assertEqual(out.arm.direction,0)
        self.assertIn('A1 unavailable',out.status)
        self.assertEqual(out.chassis,(.08,0.))
        for _ in range(6):self.send(sb=174)
        self.send(sb=174,se=1811)
        self.assertFalse(self.policy.authorized)
        self.assertEqual(self.send(sb=992,se=1811,y=1811).arm.direction,0)
        for _ in range(6):self.send(sb=992)
        self.send(sb=992,se=1811)
        self.assertEqual(self.send(sb=992,se=1811,y=1811).arm.direction,1)

    def setUp(self):
        self.policy = PocketCommand(PocketMapping(config()))
        self.now = 1.

    def send(self, **kwargs):
        self.now += .1
        self.policy.receive(row(**kwargs), self.now)
        return self.policy.output(self.now)

    def prime(self, sb=992, sc=174):
        self.send()
        self.send(sb=sb,sc=sc)
        for _ in range(5):
            self.send(sb=sb,sc=sc)
        return self.send(sb=sb,sc=sc,se=1811)

    def test_left_chassis_without_se_and_right_never_drives_chassis(self):
        for _ in range(6): self.send()
        out = self.send(forward=1811,turn=174)
        self.assertEqual(out.chassis, (.08,.28))
        self.assertEqual(out.arm.direction, 0)
        out = self.send(x=174,y=1811)
        self.assertEqual(out.chassis, (0.,0.))
        self.assertEqual(out.arm.direction, 0)

    def test_all_six_selection_positions_last_changed_switch_wins(self):
        self.send()
        for sb,sc,expected in ((992,174,2),(1811,174,3),(174,174,1),
                               (174,992,5),(174,1811,6),(174,174,4)):
            out = self.send(sb=sb,sc=sc)
            self.assertEqual(out.arm.joint, expected)
        self.assertIsNone(self.send(sb=992,sc=992).arm.joint)

    def test_startup_switches_do_not_choose_two_motors(self):
        out = self.send(sb=992,sc=1811,se=1811,y=1811)
        self.assertIsNone(out.arm.joint)
        self.assertEqual(out.arm.direction,0)

    def test_cw_ccw_cardinal_axes(self):
        for x,y,direction in ((174,992,1),(992,1811,1),(1811,992,-1),(992,174,-1)):
            self.setUp(); self.prime()
            out = self.send(sb=992,se=1811,x=x,y=y,forward=1811)
            self.assertEqual(out.arm.direction,direction)
            self.assertEqual(out.chassis,(.08,0.))

    def test_se_release_cancels_without_affecting_left_stick(self):
        self.prime()
        self.assertEqual(self.send(sb=992,se=1811,y=1811).arm.direction,1)
        out = self.send(sb=992,se=174,y=1811,forward=1811)
        self.assertEqual(out.arm.direction,0)
        self.assertEqual(out.chassis,(.08,0.))
        self.assertEqual(self.send(sb=992,se=1811,y=1811).arm.direction,0)

    def test_selection_while_se_held_requires_release_and_neutral(self):
        self.prime(); self.send(sb=992,se=1811,y=1811)
        out = self.send(sb=1811,se=1811,y=1811)
        self.assertEqual((out.arm.joint,out.arm.direction),(3,0))
        for _ in range(5): self.send(sb=1811,se=1811)
        self.assertEqual(self.send(sb=1811,se=1811,y=1811).arm.direction,0)
        for _ in range(5): self.send(sb=1811)
        self.send(sb=1811,se=1811)
        self.assertEqual(self.send(sb=1811,se=1811,y=1811).arm.direction,1)

    def test_all_corners_stop_worker_and_resume_without_repriming(self):
        for x,y in ((174,174),(174,1811),(1811,174),(1811,1811)):
            with self.subTest(x=x,y=y):
                self.setUp(); self.prime()
                port = SimulatedPositionPort()
                box = IntentMailbox()
                runner = ArmJogRunner(port,box,self.policy.mapping,clock=lambda:self.now)
                moving = self.send(sb=992,se=1811,y=1811,forward=1811)
                box.put(moving.arm); runner.tick()
                self.assertTrue(port.moving)
                stopped = self.send(sb=992,se=1811,x=x,y=y,forward=1811)
                box.put(stopped.arm); runner.tick()
                self.assertEqual(stopped.arm.direction,0)
                self.assertEqual(stopped.chassis,(.08,0.))
                self.assertGreater(stopped.arm.generation,moving.arm.generation)
                self.assertIn('diagonal: stopped; SE still primed',stopped.status)
                self.assertTrue(self.policy.authorized)
                self.assertFalse(port.moving)
                self.assertFalse(port.armed)  # Physical torque stops; SE intent is retained.
                self.assertEqual(self.send(sb=992,se=1811,x=x,y=y).arm.direction,0)
                resumed = self.send(sb=992,se=1811,y=174)
                box.put(resumed.arm); runner.tick()
                self.assertEqual(resumed.arm.direction,-1)
                self.assertTrue(self.policy.authorized)
                self.assertTrue(port.moving)
                self.assertLess(port.target,0)

    def test_corners_do_not_grant_initial_prime(self):
        self.send()
        for _ in range(6): self.send(sb=992)
        self.assertTrue(self.policy.ready)
        self.send(sb=992,se=1811,x=174,y=1811)
        self.assertFalse(self.policy.authorized)
        self.assertEqual(self.send(sb=992,se=1811,y=1811).arm.direction,0)

    def test_corner_does_not_preserve_prime_after_release_selection_or_expiry(self):
        for event in ('release','selection','expiry','invalid','stop'):
            with self.subTest(event=event):
                self.setUp(); self.prime()
                self.send(sb=992,se=1811,x=174,y=1811)
                selected_sb = 992
                if event == 'release':
                    self.send(sb=992,se=174,x=174,y=1811)
                elif event == 'selection':
                    selected_sb = 1811
                    self.send(sb=selected_sb,se=1811,x=174,y=1811)
                elif event == 'expiry':
                    self.now += .16
                    self.policy.output(self.now)
                elif event == 'invalid':
                    self.policy.receive('NO_RECENT_CHANNEL_FRAMES',self.now)
                else:
                    self.policy.reset()
                self.assertFalse(self.policy.authorized)
                self.assertEqual(self.send(sb=selected_sb,se=1811,y=1811).arm.direction,0)

    def test_neutral_stops_then_held_se_allows_next_direction(self):
        self.prime()
        cw = self.send(sb=992,se=1811,y=1811)
        neutral = self.send(sb=992,se=1811)
        ccw = self.send(sb=992,se=1811,y=174)
        self.assertEqual((cw.arm.direction,neutral.arm.direction,ccw.arm.direction),(1,0,-1))
        self.assertGreater(ccw.arm.generation,cw.arm.generation)

    def test_source_age_deadline_not_renewed_by_output(self):
        self.prime()
        out = self.send(sb=992,se=1811,y=1811,forward=1811,age=100)
        self.assertEqual(out.arm.direction,1)
        self.assertEqual(self.policy.output(self.now+.04).arm.deadline,out.arm.deadline)
        out = self.policy.output(self.now+.051)
        self.assertEqual((out.chassis,out.arm.direction),((0.,0.),0))
        self.assertEqual(self.send(sb=992,se=1811,y=1811).arm.direction,0)

    def test_stall_corruption_and_out_of_band_switch(self):
        for bad in ('NO_RECENT_CHANNEL_FRAMES','bad',row(age=151)):
            self.setUp(); self.prime(); self.send(sb=992,se=1811,y=1811)
            self.policy.receive(bad,self.now+.01)
            self.assertEqual(self.policy.output(self.now+.01).arm.direction,0)
        self.setUp(); self.prime(); self.send(sb=992,se=1811,y=1811)
        out = self.send(sb=500,se=1811,y=1811,forward=1811)
        self.assertEqual(out.arm.direction,0)
        self.assertEqual(out.chassis,(.08,0.))
        self.now += .3
        self.assertEqual(self.send(sb=992,se=1811,y=1811).arm.direction,0)



class MeasuredPocketPolicyTests(unittest.TestCase):
    def test_operator_readings_select_six_motors_and_require_held_se(self):
        # Replay the operator's measured values through the deployed profile.
        policy = PocketCommand(PocketMapping.read(ROOT/'src/bringup/config/pocket_controls.json'))
        channels = [992,992,996,996,191,191,191,191,1004,1004,1004,1004,2047,191,1811,1811]
        now = 1.

        def send(updates=None):
            nonlocal now
            for channel, value in (updates or {}).items():
                channels[channel-1] = value
            now += .1
            policy.receive(','.join(map(str, [0]+channels)), now)
            return policy.output(now)

        for _ in range(6): send()
        out = send({3:1811,4:175})
        self.assertEqual((out.chassis,out.arm.direction), ((.08,.28),0))
        send({3:996,4:996})
        for channel, value, joint in ((6,1004,2),(6,1792,3),(6,191,1),
                                      (7,1004,5),(7,1792,6),(7,191,4)):
            with self.subTest(joint=joint):
                out = send({channel:value,8:191,1:992,2:992})
                self.assertEqual((out.arm.joint,out.arm.direction), (joint,0))
                self.assertEqual(send({1:174}).arm.direction, 0)
                for _ in range(5): send({1:992})
                send({8:1792})
                for x,y,direction in ((174,992,1),(992,1811,1),
                                      (1811,992,-1),(992,174,-1)):
                    out = send({1:x,2:y})
                    self.assertEqual((out.arm.joint,out.arm.direction), (joint,direction))
                    self.assertEqual(out.chassis, (0.,0.))
                self.assertEqual(send({8:191}).arm.direction, 0)
                self.assertEqual(send({8:1792}).arm.direction, 0)  # Deflected re-press.
                send({8:191,1:992,2:992})


class MappingTests(unittest.TestCase):
    def test_rig_profile_factory_and_support_guards(self):
        from teleop.pocket_teleop import arm_factory
        rig=ROOT/'src/robot_description/config/arm_rig_a2_a6.json'
        mapping=PocketMapping.read(ROOT/'src/bringup/config/pocket_controls_rig.json')
        factory=arm_factory(mapping,execute=False,unloaded=False,supported=True,plan=rig,port='')
        port=factory()
        self.assertEqual(port.available_joints,(2,3,4,5,6))
        self.assertEqual(port.selected,2)
        with self.assertRaisesRegex(ValueError,'window'):
            arm_factory(PocketMapping.read(ROOT/'src/bringup/config/pocket_controls.json'),
                        execute=True,unloaded=False,supported=True,plan=rig,port='/not/opened')
        with self.assertRaisesRegex(ValueError,'not both'):
            arm_factory(mapping,execute=True,unloaded=True,supported=True,plan=rig,port='/not/opened')

    def test_measured_profile_valid_and_cw_required_only_for_hardware(self):
        PocketMapping.read(ROOT/'src/bringup/config/pocket_controls.json')
        measured = PocketMapping.read(ROOT/'src/bringup/config/pocket_controls.json', hardware_arm=True)
        self.assertEqual(measured.cw_signs, (-1,)*6)
        PocketMapping(config())
        with self.assertRaisesRegex(ValueError,'cw_raw_signs'):
            PocketMapping(config(),hardware_arm=True)
        cfg=config();cfg['arm']['cw_raw_signs']=[-1,1,-1,1,1,1]
        PocketMapping(cfg,hardware_arm=True)

    def test_overlaps_and_invalid_deadlines_rejected(self):
        for mutate in (lambda c:c['switches']['sb'].update(channel=0),
                       lambda c:c['switches']['sb'].update(channel=1),
                       lambda c:c['switches']['se'].update(values=[992,1000]),
                       lambda c:c.update(timeout_s=.25),
                       lambda c:c['arm'].update(speed_deg_s=1)):
            cfg=config();mutate(cfg)
            with self.assertRaises(ValueError):PocketMapping(cfg)

    def test_calibration_detects_reversed_axis_and_reversed_switch_order(self):
        center=[992]*16; pos=center.copy();neg=center.copy()
        pos[3]=174;neg[3]=1811
        axis=axis_from_samples(center,pos,neg)
        self.assertEqual((axis['channel'],axis['sign']),(4,-1))
        switch=switch_from_samples((neg,center,pos))
        self.assertEqual(switch['values'],[1811,992,174])
        neg[2]=174
        with self.assertRaises(ValueError):axis_from_samples(center,pos,neg)


class WorkerTests(unittest.TestCase):
    def test_manual_rig_stick_keeps_running_and_expiry_stops(self):
        mapping=PocketMapping.read(ROOT/'src/bringup/config/pocket_controls_rig.json',hardware_arm=True)
        self.assertIsNone(mapping.window)
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedPositionPort(window_degrees=None,available_joints=(2,3,4,5,6))
            box=IntentMailbox();runner=ArmJogRunner(port,box,mapping,clock=lambda:clock.return_value)
            for i in range(101):
                now=1+i*.02;clock.return_value=now
                box.put(ArmIntent(1,2,1,now+.15));runner.tick()
            self.assertLess(port.values[1],-.15);self.assertTrue(port.moving)
            self.assertIsNone(runner.finished)
            box.put(ArmIntent(2,2,0,3.15));runner.tick();self.assertFalse(port.moving)
            box.put(ArmIntent(3,2,-1,3.15));runner.tick();self.assertTrue(port.moving)
            clock.return_value=3.16;runner.tick();self.assertFalse(port.moving);self.assertFalse(port.armed)
            port.close()

    def test_worker_saves_startup_failure_without_a_port(self):
        def factory():
            raise RuntimeError('retained controller fault')
        worker=ArmWorker(factory,PocketMapping(config()))
        worker.start();worker.thread.join(2)
        self.assertFalse(worker.thread.is_alive())
        with self.assertRaisesRegex(RuntimeError,'retained controller fault'):
            worker.close()
        self.assertEqual(worker.report['error'],'retained controller fault')
        self.assertIn('RuntimeError: retained controller fault',worker.report['error_traceback'])

    def test_worker_preserves_captured_fault_and_cleanup_error(self):
        port=SimulatedPositionPort()
        def capture(error):
            port.report['error_original']=str(error)
            port.report['fault_snapshot']={'fault':1,'reason':9,'slot':1}
            return 'A2: measured torque limit; fault latched'
        port.capture_failure=capture
        port.close=lambda: (_ for _ in ()).throw(RuntimeError('disable acknowledgement lost'))
        worker=ArmWorker(lambda:port,PocketMapping(config()))
        with patch('teleop.pocket_arm.ArmJogRunner.tick',side_effect=RuntimeError('MC02 rejected command 4')):
            worker.start();worker.thread.join(2)
        self.assertFalse(worker.thread.is_alive())
        with self.assertRaisesRegex(RuntimeError,'A2: measured torque limit'):
            worker.close()
        self.assertIn('disable acknowledgement lost',worker.report['error'])
        self.assertEqual(worker.report['cleanup_error'],'disable acknowledgement lost')
        self.assertEqual(worker.report['error_original'],'MC02 rejected command 4')
        self.assertEqual(worker.report['fault_snapshot']['reason'],9)

    def test_subset_worker_uses_physical_numbers_and_rejects_unavailable_intent(self):
        port=SimulatedPositionPort(window_degrees=1,available_joints=(2,3,4,5,6))
        runner=ArmJogRunner(port,self.box,self.mapping,clock=lambda:self.now)
        self.intent(joint=6);runner.tick()
        self.assertEqual(port.selected,6);self.assertTrue(port.moving)
        self.intent(generation=2,joint=1);runner.tick()
        self.assertFalse(port.moving);self.assertFalse(port.armed)
        self.assertEqual(port.values[0],0)
        self.intent(generation=3,joint=2);runner.tick()
        self.assertEqual(port.selected,2);self.assertTrue(port.moving)

    def setUp(self):
        self.now=1.
        self.mapping=PocketMapping(config())
        self.port=SimulatedPositionPort(window_degrees=1)
        self.box=IntentMailbox()
        self.runner=ArmJogRunner(self.port,self.box,self.mapping,clock=lambda:self.now)

    def intent(self,generation=1,joint=2,direction=1,deadline=2.):
        self.box.put(ArmIntent(generation,joint,direction,deadline))

    def test_expiry_and_released_se_stop_and_no_stale_replay(self):
        self.intent();self.runner.tick();self.assertTrue(self.port.moving)
        self.now=2.1;self.runner.tick();self.assertFalse(self.port.moving)
        self.intent(generation=2,deadline=3);self.runner.tick()
        self.intent(generation=3,direction=0,deadline=3);self.runner.tick()
        self.assertFalse(self.port.moving);self.assertFalse(self.port.armed)

    def test_cancellation_during_slow_prepare_cannot_enable_motion(self):
        self.intent()
        original=self.port.arm
        def slow_arm():
            original();self.intent(generation=2,direction=0)
        self.port.arm=slow_arm
        self.runner.tick()
        self.assertFalse(self.port.moving);self.assertFalse(self.port.armed)

    def test_cw_sign_and_boundary_does_not_restart(self):
        cfg=config();cfg['arm']['cw_raw_signs']=[1,-1,1,1,1,1]
        self.runner.mapping=PocketMapping(cfg)
        self.intent();self.runner.tick()
        self.assertLess(self.port.target,0)
        self.port.values[1]=self.port.target
        self.runner.tick();self.assertFalse(self.port.moving)
        self.runner.tick();self.assertFalse(self.port.moving)

    def test_measured_cw_directions_command_all_six_raw_targets(self):
        mapping = PocketMapping.read(ROOT/'src/bringup/config/pocket_controls.json', hardware_arm=True)
        for joint in range(1,7):
            for direction in (1,-1):
                with self.subTest(joint=joint, direction=direction):
                    port = SimulatedPositionPort(window_degrees=10)
                    box = IntentMailbox()
                    runner = ArmJogRunner(port, box, mapping, clock=lambda:1.)
                    box.put(ArmIntent(1,joint,direction,2.))
                    runner.tick()
                    self.assertEqual(port.selected, joint)
                    self.assertTrue(port.moving)
                    self.assertLess(port.target*direction, 0.)  # CW is raw negative.
                    box.put(ArmIntent(2,joint,0,2.))
                    runner.tick()
                    self.assertFalse(port.moving)
                    self.assertFalse(port.armed)

    def test_arm_initialization_does_not_block_radio_thread(self):
        release=threading.Event()
        def factory():
            release.wait(2);return SimulatedPositionPort()
        worker=ArmWorker(factory,self.mapping);worker.start()
        try:
            self.assertFalse(worker.ready.is_set())
            policy=PocketCommand(self.mapping)
            for now in (1,1.1,1.2,1.31):policy.receive(row(),now)
            policy.receive(row(forward=1811),1.4)
            self.assertEqual(policy.output(1.4).chassis,(.08,0.))
        finally:
            release.set();worker.close()


class StreamTests(unittest.TestCase):
    def test_partial_reads_stall_and_backlog_discard(self):
        class Serial:
            def __init__(self,*a,**kw):self.data=bytearray();self.closed=False
            @property
            def in_waiting(self):return len(self.data)
            def read(self,n):out=self.data[:n];del self.data[:n];return out
            def reset_input_buffer(self):self.data.clear()
            def close(self):self.closed=True
        with patch('teleop.radio_stream.time.monotonic',return_value=1):
            stream=RadioStream('fake',serial_factory=Serial)
        stream.stream.data.extend(b'partial')
        self.assertEqual(stream.poll(1.02),[])
        stream.stream.data.extend(b' row\n')
        self.assertEqual(stream.poll(1.04),['partial row'])
        stream.stream.data.extend(b'stale\n')
        self.assertEqual(stream.poll(1.3),[''])
        stream.stream.data.extend(b'x'*257)
        self.assertEqual(stream.poll(1.32),[''])
        self.assertEqual(stream.stream.in_waiting,0)
        stream.close();self.assertTrue(stream.stream.closed)


if __name__ == '__main__':unittest.main()
