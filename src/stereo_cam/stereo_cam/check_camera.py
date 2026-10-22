"""Bounded, read-only check of images and factory calibration on ROS topics."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


STREAMS = ('color', 'depth', 'left_ir', 'right_ir')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/stereo_cam')
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--streams', nargs='+', choices=STREAMS, default=list(STREAMS))
    options = parser.parse_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be positive')
    rclpy.init()
    node = Node('stereo_cam_check')
    images, infos, counts = {}, {}, {}
    subscriptions = []

    def image_received(stream, message):
        images[stream] = message
        counts[stream] = counts.get(stream, 0) + 1

    for stream in options.streams:
        base = f"/{options.namespace.strip('/')}/{stream}"
        subscriptions.append(node.create_subscription(
            Image, base + '/image_raw',
            lambda msg, name=stream: image_received(name, msg), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(
            CameraInfo, base + '/camera_info',
            lambda msg, name=stream: infos.__setitem__(name, msg), qos_profile_sensor_data))
    success = True
    try:
        deadline = time.monotonic() + options.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0, deadline - time.monotonic())))
        for stream in options.streams:
            image, info = images.get(stream), infos.get(stream)
            valid = (image is not None and info is not None and counts[stream] >= 2
                     and image.width > 0 and image.height > 0 and len(image.data) > 0
                     and info.k[0] > 0 and info.k[4] > 0)
            success = success and valid
            if image is not None:
                print(f'{stream}: {counts[stream]} frames, {image.width}x{image.height} '
                      f'{image.encoding}, frame={image.header.frame_id}, '
                      f'calibration={"OK" if valid else "missing/invalid or insufficient frames"}')
            else:
                print(f'{stream}: no images received')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1
