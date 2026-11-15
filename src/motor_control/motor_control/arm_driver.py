# Cross-layer TODOs: ROS-02 / CAL-01; see ../TODO.md and docs/ARM_LAYER_TODOS.md.
"""Separate ROS arm transport; never routes Damiao commands to the chassis bus."""
import json
import math
import time


def validate_command(msg, names, now_ns):
    """Require complete, newly stamped six-joint commands in raw shaft units."""
    stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    if not 0 <= now_ns - stamp <= 100_000_000:
        raise ValueError('arm command must be stamped within 100 ms')
    if list(msg.name) != names or any(len(getattr(msg,k)) != 6 for k in ('position','velocity','effort')):
        raise ValueError('arm command must contain the configured six names and all three vectors')
    if not all(math.isfinite(v) for k in ('position','velocity','effort') for v in getattr(msg,k)):
        raise ValueError('arm command contains nonfinite values')
    return stamp


def driver_class():
    """Load ROS lazily so command validation remains usable offline."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from motors.arm import ArmBus
    from .telemetry import DamiaoTelemetry, TelemetryPublisher

    class ArmDriver(Node):
        def __init__(self):
            super().__init__('arm_driver')
            self.bus = None
            self.declare_parameter('execute',False)
            self.declare_parameter('port','')
            self.declare_parameter('profile','')
            if not self.get_parameter('execute').value:
                raise RuntimeError('Arm hardware is opt-in: execute:=true plus a confirmed profile and explicit port')
            with open(self.get_parameter('profile').value) as stream:
                self.profile=json.load(stream)
            self.names=[j['name'] for j in self.profile['joints']]
            if len(self.names)!=6 or len(set(self.names))!=6:
                raise ValueError('six unique arm names are required')
            self.last_stamp=0
            self.last_command_at=0.0
            self.running=False
            self.bus=ArmBus(self.get_parameter('port').value)
            try:
                self.bus.configure(self.profile)
                for i in range(6): self.bus.probe(i)
            except Exception:
                self.bus.close()
                raise
            self.publisher=self.create_publisher(JointState,'/arm/joint_states',10)
            self.telemetry_publisher=TelemetryPublisher(self,self.publisher,'damiao:DM-MC02')
            try:
                self.telemetry=DamiaoTelemetry(self.bus,self.telemetry_publisher,self.names,
                    [joint['bus'] for joint in self.profile['joints']])
            except Exception:
                self.bus.close()
                raise
            self.subscription=self.create_subscription(JointState,'/arm/command',self.command,1)
            self.enable_service=self.create_service(Trigger,'~/enable',self.enable)
            self.disable_service=self.create_service(Trigger,'~/disable',self.disable)
            self.reset_service=self.create_service(Trigger,'~/reset',self.reset)
            self.timer=self.create_timer(.04,self.poll)

        def command(self,msg):
            try:
                stamp=validate_command(msg,self.names,self.get_clock().now().nanoseconds)
                if stamp <= self.last_stamp:
                    raise ValueError('replayed arm command')
                for i,j in enumerate(self.profile['joints']):
                    self.bus.command(i,msg.position[i],msg.velocity[i],j['limits']['kp'],j['limits']['kd'],msg.effort[i])
                self.last_stamp=stamp
                self.last_command_at=time.monotonic()
            except Exception as exc:
                self.fail(exc)

        def fail(self,exc):
            self.get_logger().error(str(exc))
            self.running=False
            self.last_command_at=0.0
            try: self.bus.stop()
            except Exception: pass

        def enable(self,request,response):
            try:
                if time.monotonic()-self.last_command_at > .1:
                    raise ValueError('send a fresh complete command at measured positions before enabling')
                for i in range(6): self.bus.enable(i)
                self.running=True
                response.success=True
                response.message='Arm enabled; continue publishing fresh commands within 150 ms'
            except Exception as exc:
                self.fail(exc); response.success=False; response.message=str(exc)
            return response

        def disable(self,request,response):
            try:
                self.bus.stop(); self.running=False; self.last_command_at=0.0
                response.success=True; response.message='Disable requested; check feedback for acknowledgement'
            except Exception as exc:
                response.success=False; response.message=str(exc)
            return response

        def reset(self,request,response):
            try:
                self.bus.reset_fault()
                for i in range(6): self.bus.probe(i)
                self.last_command_at=0.0; self.last_stamp=0
                response.success=True; response.message='Fault reset; re-probing motor parameters'
            except Exception as exc:
                response.success=False; response.message=str(exc)
            return response

        def poll(self):
            try:
                sample=self.telemetry.poll()
                if sample['fault']:
                    if self.running: self.fail(RuntimeError('MC02 latched an arm fault'))
            except Exception as exc:
                self.telemetry_publisher.error('damiao',exc)
                self.fail(exc)

        def destroy_node(self):
            try:
                if self.bus: self.bus.close()
            finally: super().destroy_node()

    return ArmDriver


def main(args=None):
    import rclpy
    ArmDriver=driver_class()
    rclpy.init(args=args)
    node=None
    try:
        node=ArmDriver(); rclpy.spin(node)
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()
