"""Real ROS topic integration, automatically enabled on the Jazzy host."""
import importlib.util
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/forge_motor_control"))


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "requires ROS 2 Jazzy")
class RosChassisTests(unittest.TestCase):
    def test_chassis_topics_motor_feedback_and_publisher_loss(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from forge_motor_control.motor_driver import ForgeMotorDriver

        # All hardware execution is explicitly disabled, even if environment differs.
        rclpy.init(args=["--ros-args", "-p", "motor_ids:=[1,2,3,4]",
                        "-p", "simulate:=true", "-p", "execute:=false"])
        nodes = []
        executor = SingleThreadedExecutor()
        try:
            driver = ForgeMotorDriver(); nodes.append(driver)
            probe = Node("chassis_test_probe"); nodes.append(probe)
            states = []
            raw_pub = probe.create_publisher(Float64MultiArray, "/drive/motor_rpm_commands", 1)
            probe.create_subscription(JointState, "/joint_states", states.append, 10)
            pub = probe.create_publisher(TwistStamped, "/cmd_vel", 1)
            for node in nodes:
                executor.add_node(node)

            def spin_for(seconds, target=None):
                end = time.monotonic() + seconds
                next_send = 0
                while time.monotonic() < end:
                    if target is not None and time.monotonic() >= next_send:
                        msg = TwistStamped()
                        msg.header.stamp = probe.get_clock().now().to_msg()
                        msg.header.frame_id = "base_link"
                        msg.twist.linear.x, msg.twist.angular.z = target
                        pub.publish(msg)
                        next_send = time.monotonic() + 0.02
                    executor.spin_once(timeout_sec=0.005)

            spin_for(1.0)
            self.assertTrue(states)
            self.assertEqual(raw_pub.get_subscription_count(), 0)  # One command path only.
            self.assertEqual(states[-1].name, [f"motor_{i}_output" for i in range(1, 5)])
            for target, signs in [((0.02, 0.0), (-1, -1, 1, 1)),
                                  ((0.0, 0.07), (1, 1, 1, 1)),
                                  ((0.0, -0.07), (-1, -1, -1, -1))]:
                spin_for(0.4, target)
                self.assertTrue(all(v * s > 0 for v, s in zip(states[-1].velocity, signs)))
            # Upstream disappears while the unified driver keeps running.
            spin_for(0.6)
            self.assertTrue(all(abs(v) < 0.01 for v in states[-1].velocity))
        finally:
            for node in reversed(nodes):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()
