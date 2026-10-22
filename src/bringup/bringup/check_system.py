"""Bounded, read-only topic liveness check; never publishes motion commands."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState
from geometry_msgs.msg import TwistStamped


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--include-depth', action='store_true',
                        help='Require depth traffic when depth is explicitly enabled')
    options = parser.parse_args(args)
    if not math.isfinite(options.duration) or options.duration <= 0:
        parser.error('--duration must be positive and finite')
    rclpy.init()
    node = Node('system_test_check')
    counts, last = {}, {}
    topics = {'/imu/data_raw': Imu, '/stereo_cam/color/image_raw': Image,
              '/cmd_vel': TwistStamped,
              '/joint_states': JointState}
    if options.include_depth:
        topics['/stereo_cam/depth/image_raw'] = Image
    def receive(topic, message):
        counts[topic] = counts.get(topic, 0) + 1
        last[topic] = (time.monotonic(), message)
    subscriptions = [node.create_subscription(kind, topic,
        lambda msg, name=topic: receive(name, msg), qos_profile_sensor_data)
        for topic, kind in topics.items()]
    passed = True
    try:
        start = time.monotonic()
        deadline = start + options.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0, deadline-time.monotonic())))
        now = time.monotonic()
        for topic in topics:
            count = counts.get(topic, 0)
            age = now - last[topic][0] if topic in last else float('inf')
            live = count >= 2 and age < 2.0
            passed = passed and live
            print(f'{"LIVE" if live else "MISSING/STALE"}: {topic}: '
                  f'{count} messages, {count / (now-start):.1f}/s, last age {age:.2f}s')
        if '/joint_states' in last:
            print('Reported motors: ' + ', '.join(last['/joint_states'][1].name))
        print('This checks traffic only, not RF failsafe, calibration, sensor fusion or physical stopping.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if passed else 1
