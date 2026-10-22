"""ROS depth or RGB video window; Escape or closing the window exits."""
import argparse
from collections import deque
import time
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from stereo_cam.system_usage import SystemUsage


def colorize_depth(depth, encoding):
    """Fixed 0.25-6 m display range; invalid pixels stay black."""
    if encoding == '16UC1':
        meters = depth.astype(np.float32) * .001
    elif encoding == '32FC1':
        meters = depth.astype(np.float32)
    else:
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    valid = np.isfinite(meters) & (meters > 0)
    normalized = np.zeros(meters.shape, dtype=np.uint8)
    normalized[valid] = np.rint(np.clip((meters[valid]-.25)/5.75, 0, 1)*255).astype(np.uint8)
    frame = cv2.applyColorMap(255-normalized, cv2.COLORMAP_TURBO)
    frame[~valid] = 0
    return frame


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/stereo_cam/color/image_raw')
    argv = sys.argv if args is None else [parser.prog, *args]
    options = parser.parse_args(remove_ros_args(argv)[1:])
    rclpy.init(args=argv[1:])
    node = Node('stereo_cam_viewer')
    bridge = CvBridge()
    latest, received = None, 0.0
    arrivals = deque()
    title = 'FORGE - Stereo camera'
    usage = SystemUsage()

    def receive(message):
        nonlocal latest, received
        try:
            if message.encoding in ('16UC1', '32FC1'):
                latest = colorize_depth(bridge.imgmsg_to_cv2(message, desired_encoding='passthrough'),
                                        message.encoding)
                cv2.putText(latest, 'Depth 0.25-6 m | warm: near, cool: far | black: invalid',
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .48, (255,255,255), 1)
            else:
                latest = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            received = time.monotonic()
            arrivals.append(received)
        except Exception as exc:
            node.get_logger().error(f'Cannot display image: {exc}')

    subscription = node.create_subscription(
        Image, options.topic, receive,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, 960, 540)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if latest is None:
                frame = np.zeros((540, 960, 3), dtype=np.uint8)
                cv2.putText(frame, 'Waiting for camera footage...', (35, 245),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (220, 220, 220), 2)
                cv2.putText(frame, options.topic, (35, 295),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (160, 160, 160), 1)
            else:
                frame = latest.copy()
                age = time.monotonic() - received
                if age > 2:
                    cv2.putText(frame, f'No new frames for {age:.1f}s', (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # Separate footer keeps diagnostics legible without covering depth.
            footer = np.zeros((64, frame.shape[1], 3), dtype=np.uint8)
            status = usage.sample()
            text_width = cv2.getTextSize(status, cv2.FONT_HERSHEY_SIMPLEX, .55, 1)[0][0]
            scale = .55 * min(1., (frame.shape[1]-20)/max(1, text_width))
            cv2.putText(footer, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, (230,230,230), 1, cv2.LINE_AA)
            now = time.monotonic()
            while arrivals and arrivals[0] < now - 1.0:
                arrivals.popleft()
            # Count callbacks in the trailing second, not the requested stream rate.
            gap = f'{1000*(now-received):.0f} ms' if received else 'waiting'
            cv2.putText(footer, f'Received: {len(arrivals)} FPS | Last frame: {gap} ago',
                        (10, 51), cv2.FONT_HERSHEY_SIMPLEX, scale,
                        (230,230,230), 1, cv2.LINE_AA)
            cv2.imshow(title, np.vstack((frame, footer)))
            if cv2.waitKey(1) & 0xff in (27, ord('q')):
                break
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
