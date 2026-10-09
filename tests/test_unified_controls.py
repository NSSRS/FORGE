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

    def test_target_log_matches_bus_calls_after_mixing_and_expiry(self):
        FakeNode.overrides = {"log_motor_targets": True, "bench_omit_motor_id": 4,
                              "max_velocity_rpm": 20.0}
        module = self.load("motor_driver")
        driver = module.ForgeMotorDriver()
        logger = Mock()
        driver.get_logger = lambda: logger
        driver._update()
        logger.info.assert_not_called()  # Query-only has no target yet.
        msg = Message()
        msg.twist.linear.x, msg.twist.angular.z = .08, .28
        with patch.object(module.time, "monotonic", return_value=1):
            driver._receive_chassis(msg)
            driver._update()
        expected = "MotorBus targets (RPM): " + ", ".join(
            f"ID {call.args[0]}={call.args[1]:+.3f}"
            for call in self.bus.velocity.call_args_list)
        logger.info.assert_called_once_with(expected)
        self.assertEqual(len(self.bus.velocity.call_args_list), 3)
        with patch.object(module.time, "monotonic", return_value=1.21):
            driver._update()
        logger.info.assert_called_with(
            "MotorBus targets (RPM): ID 1=+0.000, ID 2=+0.000, ID 3=+0.000")

    def test_bench_omission_keeps_four_wheel_mix_and_filters_by_id(self):
        # Reorder IDs so omitting motor 4 cannot accidentally shift targets.
        FakeNode.overrides = {"motor_ids": [4, 3, 1, 2],
                              "motor_slots": [4, 3, 0, 1], "bench_omit_motor_id": 4}
        module = self.load("motor_driver")
        driver = module.ForgeMotorDriver()
        self.assertEqual(driver.subscriptions, ["/cmd_vel"])
        configs = self.bus_class.call_args.args[0]
        self.assertEqual([c.motor_id for c in configs], [3, 1, 2])
        self.assertEqual(self.bus_class.call_args.kwargs["motor_slots"], (3, 0, 1))
        for n, (v, w, signs) in enumerate(((.02, 0, (1, -1, -1)),
                                         (-.02, 0, (-1, 1, 1)),
                                         (0, .07, (1, 1, 1)),
                                         (0, -.07, (-1, -1, -1)))):
            msg = Message()
            msg.header.stamp.nanosec = n + 1
            msg.twist.linear.x, msg.twist.angular.z = v, w
            driver.get_clock = lambda: NS(now=lambda: NS(
                nanoseconds=10_000_000_100, to_msg=lambda: NS(sec=10, nanosec=100)))
            self.bus.velocity.reset_mock()
            with patch.object(module.time, "monotonic", return_value=1):
                driver._receive_chassis(msg)
                driver._update()
            calls = self.bus.velocity.call_args_list
            self.assertEqual([c.args[0] for c in calls], [3, 1, 2])
            self.assertTrue(all(c.args[1] * sign > 0 for c, sign in zip(calls, signs)))
        self.bus.velocity.reset_mock()
        with patch.object(module.time, "monotonic", return_value=1.21):
            driver._update()
        self.assertEqual([c.args for c in self.bus.velocity.call_args_list],
                         [(3, 0), (1, 0), (2, 0)])
        state = driver._state_publisher.publish.call_args.args[0]
        self.assertEqual(state.name, ["motor_3_output", "motor_1_output", "motor_2_output"])

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
                          {"simulate": False, "execute": True}, {"command_mode": "typo"},
                          {"bench_omit_motor_id": 99},
                          {"command_mode": "direct_rpm", "bench_omit_motor_id": 4}):
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
        with self.assertRaisesRegex(ValueError, "chassis/WASD only"):
            module.KeyboardTeleop()

    def test_radio_requires_calibration_and_monitor_has_no_publisher(self):
        FakeNode.overrides = {"input_source": "radio"}
        with self.assertRaises(ValueError):
            self.load("keyboard_teleop").KeyboardTeleop()
        FakeNode.overrides = {"input_source": "radio_monitor"}
        monitor = self.load("keyboard_teleop").KeyboardTeleop()
        self.assertEqual(monitor.publications, [])
