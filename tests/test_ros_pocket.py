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
    def test_shared_stream_chassis_selection_primer_and_loss(self):
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
                mapping=Path(directory)/'mapping.json';mapping.write_text(json.dumps(config()))
                report=Path(directory)/'report.json'
                with (Path(directory)/'node.log').open('w+') as log:
                    env=dict(os.environ)
                    env['PYTHONPATH']=os.pathsep.join([str(ROOT/'src'),str(ROOT/'src/teleop'),env.get('PYTHONPATH','')])
                    process=subprocess.Popen([sys.executable,'-c','from teleop.pocket_teleop import main; main()',
                        '--ros-args','-p',f'mapping_file:={mapping}','-p',f'serial_port:={os.ttyname(slave)}',
                        '-p','arm_execute:=false','-p',f'report:={report}'],env=env,stdout=log,stderr=log)

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
                    self.assertEqual(len(dashboards[-1]['arm']['rows']),6)
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
