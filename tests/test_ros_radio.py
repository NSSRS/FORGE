"""Real serial PTY -> radio node -> ROS chassis simulator; never opens motor hardware."""
import source_paths  # source-checkout package imports
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/motor_control'))


@unittest.skipUnless(importlib.util.find_spec('rclpy') and importlib.util.find_spec('serial'),
                     'requires ROS 2 and pyserial')
class RosRadioTests(unittest.TestCase):
    def test_serial_to_chassis_and_disconnect(self):
        import pty
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import TwistStamped
        from motor_control.motor_driver import ForgeMotorDriver

        master, slave = pty.openpty()
        process = None
        nodes = []
        rclpy.init(args=['--ros-args', '--params-file',
                        str(ROOT / 'src/bringup/config/chassis_bench.yaml'),
                        '-p', 'simulate:=true', '-p', 'execute:=false'])
        executor = SingleThreadedExecutor()
        try:
            driver = ForgeMotorDriver(); nodes.append(driver)
            probe = Node('radio_test_probe'); nodes.append(probe)
            commands, states = [], []
            probe.create_subscription(TwistStamped, '/cmd_vel', commands.append, 10)
            probe.create_subscription(JointState, '/joint_states', states.append, 10)
            for node in nodes:
                executor.add_node(node)
            env = dict(os.environ)
            env['PYTHONPATH'] = str(ROOT / 'src/motor_control') + os.pathsep + env.get('PYTHONPATH', '')
            args = [sys.executable, '-c', 'from teleop.keyboard_teleop import main; main()',
                    '--ros-args', '--params-file',
                    str(ROOT / 'src/bringup/config/chassis_bench.yaml'),
                    '-p', 'input_source:=radio', '-p', f'serial_port:={os.ttyname(slave)}']
            for prefix, channel in [('forward', 1), ('turn', 2)]:
                for field, value in [('channel', channel), ('min', 200), ('center', 1000), ('max', 1800)]:
                    args += ['-p', f'{prefix}_{field}:={value}']
            process = subprocess.Popen(args, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            def spin(seconds, values=None):
                end, next_send = time.monotonic() + seconds, 0
                while time.monotonic() < end:
                    if values is not None and time.monotonic() >= next_send:
                        os.write(master, (','.join(map(str, [0, *values] + [1000] * 14)) + '\n').encode())
                        next_send = time.monotonic() + .08
                    executor.spin_once(timeout_sec=.005)

            # Discovery and serial open; deflection on startup must not move.
            spin(2.0, (1800, 1800))
            self.assertIsNone(process.poll())
            self.assertTrue(commands)
            self.assertTrue(all(m.twist.linear.x == m.twist.angular.z == 0 for m in commands))
            spin(.6, (1000, 1000))
            spin(.4, (1800, 1000))
            self.assertAlmostEqual(commands[-1].twist.linear.x, .08)
            self.assertEqual(states[-1].name, ['motor_1_output', 'motor_2_output', 'motor_3_output'])
            self.assertTrue(all(v * s > 0 for v, s in zip(states[-1].velocity, [-1, -1, 1])))
            spin(.4, (1000, 1800))
            self.assertAlmostEqual(commands[-1].twist.angular.z, .28)
            self.assertTrue(all(v > 0 for v in states[-1].velocity))
            spin(.2)  # USB silence must clear the request.
            self.assertEqual((commands[-1].twist.linear.x, commands[-1].twist.angular.z), (0, 0))
            spin(.4)  # Allow the existing 60 RPM/s slew to reach zero from 20 RPM.
            self.assertTrue(all(abs(v) < .01 for v in states[-1].velocity))
            spin(.3, (1800, 1000))  # Deflection cannot rearm after silence.
            self.assertEqual(commands[-1].twist.linear.x, 0)
            spin(.6, (1000, 1000))
            spin(.3, (1800, 1000))
            self.assertAlmostEqual(commands[-1].twist.linear.x, .08)
            os.close(master); master = None
            spin(.6)
            self.assertTrue(all(abs(v) < .01 for v in states[-1].velocity))
            self.assertIsNotNone(process.poll())
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            if master is not None:
                os.close(master)
            os.close(slave)
            for node in reversed(nodes):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()
