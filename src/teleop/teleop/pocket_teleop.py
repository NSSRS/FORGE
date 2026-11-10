"""Existing Pocket serial stream -> left chassis and SE-gated right arm."""
import json
import math
from pathlib import Path
import time

from .pocket_input import PocketMapping, PocketCommand
from .pocket_arm import ArmWorker
from .radio_stream import RadioStream
from .pocket_display import dashboard_snapshot


def read_arm_plan(plan):
    from motors.arm_bench import active_joints, validate_plan
    config = json.loads(Path(plan).read_text()) if plan else None
    if config is not None:
        validate_plan(config, all_motors=True)
    return config, active_joints(config) if config is not None else tuple(range(1,7))


def arm_factory(mapping, *, execute, unloaded, plan, port, supported=False):
    # Validate before opening either a serial port or a simulated adapter.
    from motors.arm_position import ArmPositionPort, SimulatedPositionPort
    config, joints = read_arm_plan(plan)
    if supported and ((mapping.window is not None and mapping.window>3) or mapping.speed>math.radians(5)):
        raise ValueError('Supported-arm profile requires manual travel or window <=3 degrees, and speed <=5 degrees/s')
    if supported and unloaded:
        raise ValueError('Choose supported arm or unloaded fixture, not both')
    if execute:
        if not (unloaded or supported) or not plan or not port:
            raise ValueError('Physical arm requires arm_unloaded or arm_supported, arm_plan and arm_port')
        if 0 in mapping.cw_signs:
            raise ValueError('Measure cw_raw_signs before physical arm jogging')
        config['fixture_confirmed'] = True  # Explicit fixture confirmation from this invocation.
        config['fixture_mode'] = 'supported' if supported else 'unloaded'
        config['supported_confirmed'] = bool(supported)
        from motors.arm_bench import validate_plan
        validate_plan(config, execute=True, all_motors=True, group_first=True)
        return lambda: ArmPositionPort(config, port=port, window_degrees=mapping.window)
    return lambda: SimulatedPositionPort(window_degrees=mapping.window,available_joints=joints)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String, Empty

    class PocketTeleop(Node):
        def __init__(self):
            super().__init__('pocket_teleop')
            for name, value in (('mapping_file',''), ('serial_port',''),
                                ('speed_m_s',.08), ('yaw_rate_rad_s',.28),
                                ('command_frame','base_link'), ('use_arm',True),
                                ('arm_execute',False), ('arm_unloaded',False), ('arm_supported',False),
                                ('arm_plan',''), ('arm_port',''), ('report','')):
                self.declare_parameter(name, value)
            self.value = lambda key: self.get_parameter(key).value
            if not self.value('serial_port') or not self.value('command_frame'):
                raise ValueError('Specify radio serial_port and command_frame')
            if self.value('arm_execute') and not self.value('use_arm'):
                raise ValueError('arm_execute requires use_arm')
            if self.value('arm_execute') and Path(self.value('serial_port')).resolve() == Path(self.value('arm_port')).resolve():
                raise ValueError('Radio and MC02 must use different serial ports')
            mapping = PocketMapping.read(self.value('mapping_file'), hardware_arm=self.value('arm_execute'))
            joints=read_arm_plan(self.value('arm_plan'))[1] if self.value('use_arm') else tuple(range(1,7))
            self.policy = PocketCommand(mapping, self.value('speed_m_s'), self.value('yaw_rate_rad_s'),
                                        available_joints=joints)
            self.worker = None
            if self.value('use_arm'):
                factory = arm_factory(mapping, execute=self.value('arm_execute'), unloaded=self.value('arm_unloaded'),
                                      plan=self.value('arm_plan'), port=self.value('arm_port'),
                                      supported=self.value('arm_supported'))
                self.worker = ArmWorker(factory, mapping)
            self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 1)
            self.status_publisher = self.create_publisher(String, '/arm/teleop_status', 1)
            self.last_status = None
            self.dashboard_publisher = self.create_publisher(String, '/pocket/telemetry', 1)
            self.create_subscription(Empty, '/pocket/stop', self.stop_controls, 1)
            self.next_dashboard = 0.

        def stop_controls(self, _message):
            self.policy.reset()
            if self.worker:
                self.worker.mailbox.put(self.policy.output(time.monotonic()).arm)
            self.publish((0.,0.))
            self.get_logger().info('Dashboard STOP: center sticks, release SE and reselect motor')

        def publish(self, target):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.value('command_frame')
            msg.twist.linear.x, msg.twist.angular.z = target
            self.publisher.publish(msg)

        def run(self):
            stream = None
            try:
                stream = RadioStream(self.value('serial_port'))
                if self.worker:
                    self.worker.start()
                mode = ('HARDWARE, mechanically supported' if self.value('arm_supported') else
                        'HARDWARE, unloaded only') if self.value('arm_execute') else 'SIMULATED ARM'
                self.get_logger().info(f'Pocket: LEFT chassis; RIGHT arm; SB=1/2/3 SC=4/5/6; hold SE. {mode}')
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.)
                    now = time.monotonic()
                    for line in stream.poll(now):
                        self.policy.receive(line, now)
                    if self.worker and self.worker.error:
                        # Stop chassis and join cleanup first. close() raises
                        # the completed diagnostic once, after report capture.
                        break
                    if not self.worker or not self.worker.ready.is_set():
                        self.policy.inhibit_arm()
                    output = self.policy.output(time.monotonic())
                    if self.worker:
                        self.worker.mailbox.put(output.arm)
                    self.publish(output.chassis)
                    status = f'{mode}: {output.status}'
                    if status != self.last_status:
                        self.get_logger().info(status)
                        msg = String(); msg.data = status
                        self.status_publisher.publish(msg)
                        self.last_status = status
                    now = time.monotonic()
                    if now >= self.next_dashboard:
                        state = self.worker.display_snapshot(now) if self.worker else {}
                        payload = dashboard_snapshot(self.policy, state, now,
                                                     hardware_arm=self.value('arm_execute'))
                        msg = String(); msg.data = json.dumps(payload, allow_nan=False)
                        self.dashboard_publisher.publish(msg)
                        self.next_dashboard = now + .1
                    time.sleep(.02)
            finally:
                self.policy.reset()
                if self.worker:
                    self.worker.mailbox.put(self.policy.output(time.monotonic()).arm)
                # Stop chassis before waiting for arm disable/mode restoration.
                if rclpy.ok():
                    for _ in range(5):
                        try:
                            self.publish((0.,0.))
                        except Exception:
                            # SIGINT can invalidate ROS between ok() and publish.
                            # Cleanup must still disable/restore the separate arm.
                            break
                try:
                    if stream:
                        stream.close()
                finally:
                    try:
                        if self.worker:
                            self.worker.close()
                    finally:
                        if self.value('report') and self.worker:
                            path = Path(self.value('report'))
                            path.parent.mkdir(parents=True, exist_ok=True)
                            path.write_text(json.dumps(self.worker.report, indent=2)+'\n')

    rclpy.init(args=args)
    node = None
    try:
        node = PocketTeleop()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        rclpy.logging.get_logger('pocket_teleop').error(f'Pocket stopped: {exc}')
        raise SystemExit(1) from None
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
