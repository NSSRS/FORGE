"""Exercise actual node callbacks with a minimal ROS facade and no hardware."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

PACKAGE = Path(__file__).resolve().parents[1] / "src/forge_motor_control"
sys.path.insert(0, str(PACKAGE))


class Message:
    def __init__(self):
        self.header = NS(stamp=NS(sec=10, nanosec=0), frame_id="base_link")
        self.twist = NS(linear=NS(x=0.0, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=0.0))
        self.data = []


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.subscriptions = []
        self.publications = []

    def declare_parameter(self, name, value):
        self.parameters[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def create_subscription(self, kind, topic, callback, depth):
        self.subscriptions.append(topic)
        return NS()

    def create_publisher(self, kind, topic, depth):
        self.publications.append(topic)
        return Mock()

    def create_timer(self, *args):
        return Mock()

    def get_logger(self):
        return Mock()

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=10_000_000_000, to_msg=lambda: NS(sec=10, nanosec=0)))


class UnifiedTests(unittest.TestCase):
    def setUp(self):
        FakeNode.overrides = {}
        modules = {"rclpy": NS(ok=lambda: True), "rclpy.node": NS(Node=FakeNode),
                   "sensor_msgs.msg": NS(JointState=Message),
                   "std_msgs.msg": NS(Float64MultiArray=Message),
                   "geometry_msgs.msg": NS(TwistStamped=Message)}
        self.facade = patch.dict(sys.modules, modules)
        self.facade.start()
        self.addCleanup(self.facade.stop)
        self.bus_class = patch("forge_motors.MotorBus").start()
        self.addCleanup(patch.stopall)
        self.bus = self.bus_class.return_value
        self.bus.state.return_value = NS(position_deg=0.0, velocity_rpm=0.0)

    def load(self, name):
        spec = importlib.util.spec_from_file_location(
            "forge_motor_control._test_" + name, PACKAGE / "forge_motor_control" / (name + ".py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_default_driver_handles_turning_and_expiry_in_one_node(self):
        module = self.load("motor_driver")
        driver = module.ForgeMotorDriver()
        self.assertEqual(driver.subscriptions, ["/cmd_vel"])
        driver._update()
        self.bus.velocity.assert_not_called()
        msg = Message()
        msg.twist.angular.z = 0.07
        with patch.object(module.time, "monotonic", return_value=1):
            driver._receive_chassis(msg)
            driver._update()
        self.assertEqual([c.args[0] for c in self.bus.velocity.call_args_list], [1, 2, 3, 4])
        self.assertTrue(all(c.args[1] > 0 for c in self.bus.velocity.call_args_list))
        self.bus.velocity.reset_mock()
        with patch.object(module.time, "monotonic", return_value=1.21):
            driver._update()
        self.assertEqual([c.args[1] for c in self.bus.velocity.call_args_list], [0] * 4)

    def test_direct_commissioning_uses_only_original_topic(self):
        FakeNode.overrides = {"command_mode": "direct_rpm", "motor_ids": [1, 2, 3]}
        driver = self.load("motor_driver").ForgeMotorDriver()
        self.assertEqual(driver.subscriptions, ["/drive/motor_rpm_commands"])
        msg = Message(); msg.data = [2.0, 2.0, 2.0]
        driver._receive_command(msg)
        driver._update()
        self.assertEqual([c.args for c in self.bus.velocity.call_args_list], [(1, 2), (2, 2), (3, 2)])

    def test_bad_chassis_mapping_and_unconfirmed_hardware_never_open_bus(self):
        for overrides in ({"motor_ids": [1, 2, 3]},
                          {"simulate": False, "execute": True}, {"command_mode": "typo"}):
            FakeNode.overrides = overrides
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.load("motor_driver").ForgeMotorDriver()
        self.bus_class.assert_not_called()

    def test_single_keyboard_selects_message_and_directions(self):
        module = self.load("keyboard_teleop")
        keyboard = module.KeyboardTeleop()
        self.assertEqual(keyboard.publications, ["/cmd_vel"])
        keyboard.publish((0.02, 0.07))
        msg = keyboard._publisher.publish.call_args.args[0]
        self.assertEqual((msg.twist.linear.x, msg.twist.angular.z), (0.02, 0.07))
        self.assertIn("a", keyboard._keys.commands)
        FakeNode.overrides = {"command_mode": "direct_rpm"}
        keyboard = module.KeyboardTeleop()
        self.assertEqual(keyboard.publications, ["/drive/motor_rpm_commands"])
        keyboard.publish((5.0, 0.0))
        self.assertEqual(keyboard._publisher.publish.call_args.args[0].data, [5.0] * 3)
        self.assertNotIn("a", keyboard._keys.commands)
