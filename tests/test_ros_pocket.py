"""Real PTY -> Pocket node -> ROS output + offline arm; no motor devices."""
import source_paths
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from test_pocket_controls import config, row

ROOT=Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec('rclpy') and importlib.util.find_spec('serial'), 'requires ROS and pyserial')
class RosPocketTests(unittest.TestCase):
    def test_fault_exit_logs_one_error_and_saves_diagnostics(self):
        import pty
        master,slave=pty.openpty()
        process=None
        try:
            with tempfile.TemporaryDirectory() as directory:
                mapping=Path(directory)/'mapping.json';mapping.write_text(json.dumps(config()))
                report=Path(directory)/'report.json'
                program='''from unittest.mock import patch
from teleop.pocket_teleop import main
def fail():
    raise RuntimeError('A2: measured torque limit; measured 3.0017 Nm, limit 3.0000 Nm; fault latched')
with patch('teleop.pocket_teleop.arm_factory', return_value=fail):
    main()
'''
                env=dict(os.environ)
                env['PYTHONPATH']=os.pathsep.join([str(ROOT/'src'),str(ROOT/'src/teleop'),env.get('PYTHONPATH','')])
                process=subprocess.Popen([sys.executable,'-c',program,'--ros-args',
                    '-p',f'mapping_file:={mapping}','-p',f'serial_port:={os.ttyname(slave)}',
                    '-p','arm_execute:=false','-p',f'report:={report}'],
                    env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
                output,_=process.communicate(timeout=10)
                self.assertEqual(process.returncode,1,output)
                self.assertEqual(output.count('Pocket stopped:'),1,output)
                self.assertIn('A2: measured torque limit',output)
                self.assertNotIn('Traceback',output)
                saved=json.loads(report.read_text())
                self.assertIn('limit 3.0000 Nm',saved['error'])
                self.assertIn('RuntimeError: A2:',saved['error_traceback'])
        finally:
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.communicate(timeout=5)
            os.close(master);os.close(slave)

    def test_shared_stream_chassis_selection_primer_and_loss(self):
        self.run_pocket_case(list(range(1,7)))

    def test_subset_a2_to_a6_keeps_physical_numbers(self):
        self.run_pocket_case([2,3,4,5,6])

    def test_manual_travel_rig_passes_old_window_and_keeps_stop_policy(self):
        self.run_pocket_case([2,3,4,5,6],manual=True)

    def run_pocket_case(self,joints,manual=False):
        import pty
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String, Empty
        master,slave=pty.openpty()
        process=None;probe=None
        rclpy.init()
        try:
            probe=Node('pocket_integration_probe')
            commands=[];statuses=[];dashboards=[]
            probe.create_subscription(TwistStamped,'/cmd_vel',commands.append,10)
            probe.create_subscription(String,'/arm/teleop_status',lambda msg:statuses.append(msg.data),10)
            probe.create_subscription(String,'/pocket/telemetry',lambda msg:dashboards.append(json.loads(msg.data)),10)
            stop_publisher=probe.create_publisher(Empty,'/pocket/stop',1)
            with tempfile.TemporaryDirectory() as directory:
                mapping=Path(directory)/'mapping.json'
                mapping_data=config()
                if manual:mapping_data['arm'].update(window_deg=None,speed_deg_s=5)
                mapping.write_text(json.dumps(mapping_data))
                plan=Path(directory)/'plan.json'
                plan_data=json.loads((ROOT/'src/robot_description/config/arm_jog.json').read_text())
                plan_data['active_joints']=joints
                plan.write_text(json.dumps(plan_data))
                report=Path(directory)/'report.json'
                with (Path(directory)/'node.log').open('w+') as log:
                    env=dict(os.environ)
                    env['PYTHONPATH']=os.pathsep.join([str(ROOT/'src'),str(ROOT/'src/teleop'),env.get('PYTHONPATH','')])
                    process=subprocess.Popen([sys.executable,'-c','from teleop.pocket_teleop import main; main()',
                        '--ros-args','-p',f'mapping_file:={mapping}','-p',f'serial_port:={os.ttyname(slave)}',
                        '-p','arm_execute:=false','-p',f'arm_plan:={plan}','-p',f'report:={report}'],env=env,stdout=log,stderr=log)

                    def spin(seconds,values=None):
                        end=time.monotonic()+seconds;next_send=0
                        while time.monotonic()<end:
                            if values is not None and time.monotonic()>=next_send:
                                os.write(master,(row(**values)+'\n').encode());next_send=time.monotonic()+.08
                            rclpy.spin_once(probe,timeout_sec=.005)

                    spin(2,{})
                    if process.poll() is not None:
                        log.seek(0);self.fail(log.read())
                    self.assertTrue(commands)
                    self.assertTrue(dashboards)
                    self.assertTrue(dashboards[-1]['radio_fresh'])
                    self.assertEqual([r['joint'] for r in dashboards[-1]['arm']['rows']],joints)
                    self.assertEqual(dashboards[-1]['available_joints'],joints)
                    spin(.3,dict(forward=1811))
                    self.assertEqual(commands[-1].twist.linear.x,.08)
                    # Right-stick deflection with SE released never authorizes the arm.
                    spin(.3,dict(y=1811))
                    self.assertEqual(commands[-1].twist.linear.x,0.)
                    self.assertFalse(any(s.endswith(': CW') for s in statuses))
                    spin(.6,dict(sb=992))
                    spin(.15,dict(sb=992,se=1811))
                    spin(.3,dict(sb=992,se=1811,y=1811,forward=1811))
                    self.assertTrue(any('A2: CW' in s for s in statuses))
                    self.assertEqual(commands[-1].twist.linear.x,.08)
                    if manual:
                        spin(1.5,dict(sb=992,se=1811,y=1811))
                        self.assertEqual(dashboards[-1]['arm']['travel_policy'],'manual')
                        self.assertTrue(dashboards[-1]['arm']['moving'])
                        a2=next(row for row in dashboards[-1]['arm']['rows'] if row['joint']==2)
                        self.assertGreater(a2['position_rad'],.05236)  # Past the old 3-degree boundary.
                    # Diagonal zero reaches the worker while SE authorization survives.
                    for x,y in ((174,174),(174,1811),(1811,174),(1811,1811)):
                        spin(.3,dict(sb=992,se=1811,x=x,y=y,forward=1811))
                        self.assertTrue(dashboards[-1]['diagonal'])
                        self.assertTrue(dashboards[-1]['authorized'])
                        self.assertEqual(dashboards[-1]['direction'],0)
                        self.assertFalse(dashboards[-1]['arm']['moving'])
                        self.assertEqual(commands[-1].twist.linear.x,.08)
                        spin(.2,dict(sb=992,se=1811,y=1811))
                        self.assertEqual(dashboards[-1]['direction'],1)
                        self.assertTrue(dashboards[-1]['arm']['moving'])
                    spin(.2,dict(sb=992,y=1811))
                    # SC middle selects A5; it cannot move until SE is freshly pressed.
                    spin(.6,dict(sb=992,sc=992))
                    spin(.15,dict(sb=992,sc=992,se=1811))
                    spin(.3,dict(sb=992,sc=992,se=1811,x=174))
                    self.assertTrue(any('A5: CW' in s for s in statuses))
                    self.assertTrue(any(row['joint']==2 and row['position_rad']>0
                                        for d in dashboards for row in d['arm'].get('rows',[])))
                    stop_publisher.publish(Empty())
                    spin(.25,dict(sb=992,sc=992,se=1811,x=174))
                    self.assertIsNone(dashboards[-1]['selected'])
                    self.assertFalse(dashboards[-1]['arm']['moving'])
                    spin(.25)  # Serial silence cancels both paths.
                    self.assertEqual(commands[-1].twist.linear.x,0.)
                    count=sum('A5: CW' in s for s in statuses)
                    spin(.3,dict(sb=992,sc=992,se=1811,x=174))
                    self.assertEqual(sum('A5: CW' in s for s in statuses),count)
                    process.send_signal(signal.SIGINT);process.wait(timeout=8)
                    log.seek(0)
                    self.assertEqual(process.returncode,0,log.read())
                    values=json.loads(report.read_text())['final_positions_rad']
                    self.assertGreater(values[1],0)
                    self.assertGreater(values[4],0)
                    self.assertTrue(all(values[i]==0 for i in (0,2,3,5)))
        finally:
            if process and process.poll() is None:
                process.terminate();process.wait(timeout=8)
            os.close(master);os.close(slave)
            if probe:probe.destroy_node()
            if rclpy.ok():rclpy.shutdown()
