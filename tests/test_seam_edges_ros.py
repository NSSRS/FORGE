"""Synthetic ROS image delivery; run on an isolated domain with camera_env sourced."""
import os
import signal
import subprocess
import sys
import time
import unittest

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SeamRosTests(unittest.TestCase):
    def test_image_outputs_preserve_header_and_publish_without_gui(self):
        if os.environ.get('ROS_DOMAIN_ID') not in ('90', '91', '92'):
            self.skipTest('Use isolated test ROS_DOMAIN_ID 90, 91 or 92')
        rclpy.init()
        node = rclpy.create_node('seam_edges_test')
        messages = {}
        subscriptions = [node.create_subscription(Image, '/seam_detection/'+name,
            lambda msg, name=name: messages.__setitem__(name, msg), qos_profile_sensor_data)
            for name in ('annotated', 'edges')]
        pub = node.create_publisher(Image, '/test/seam_input', qos_profile_sensor_data)
        process = subprocess.Popen([sys.executable, '-c',
            'from seam_detection.node import main; main()', '--ros-args',
            '-p', 'show_video:=false', '-p', 'input_topic:=/test/seam_input'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            frame = np.full((400, 640, 3), 200, np.uint8)
            frame[:, 300:340] = 20
            message = CvBridge().cv2_to_imgmsg(frame, encoding='bgr8')
            message.header.frame_id = 'synthetic_optical_frame'
            message.header.stamp.sec = 123
            deadline = time.monotonic()+10
            while len(messages) < 2 and time.monotonic() < deadline and process.poll() is None:
                pub.publish(message)
                rclpy.spin_once(node, timeout_sec=.05)
            self.assertEqual(set(messages), {'annotated', 'edges'})
            for output in messages.values():
                self.assertEqual(output.header, message.header)
                self.assertEqual((output.width, output.height), (640, 400))
            self.assertEqual(messages['annotated'].encoding, 'bgr8')
            self.assertEqual(messages['edges'].encoding, 'mono8')
            self.assertTrue(any(messages['edges'].data))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                output, _ = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                output, _ = process.communicate()
            node.destroy_node()
            rclpy.shutdown()
        self.assertEqual(process.returncode, 0, output)
