"""ROS image subscriber with latest-frame processing and optional edge window."""
from collections import deque
from dataclasses import replace
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from stereo_cam.frame_buffer import FrameBuffer
from stereo_cam.system_usage import SystemUsage

from seam_detection.edges import Settings, detect


def main(args=None):
    rclpy.init(args=args)
    node = Node('seam_edges')
    executor = None
    worker = None
    stop = threading.Event()
    try:
        def param(name, default):
            return node.declare_parameter(name, default).value

        topic = param('input_topic', '/stereo_cam/color/image_raw')
        show = param('show_video', True)
        fps = param('processing_fps', 30.)
        settings = Settings(
            processing_width=param('processing_width', 640),
            roi=tuple(param('roi', [.2, .2, .8, .95])),
            canny_low=param('canny_low', 50), canny_high=param('canny_high', 150),
            min_width=param('min_width_px', 8), max_width=param('max_width_px', 140),
            max_angle=param('max_angle_deg', 40.))
        settings.validate()
        if not math.isfinite(fps) or not 1 <= fps <= 120:
            raise ValueError('processing_fps must be finite and between 1 and 120')
        bridge, mailbox, usage = CvBridge(), FrameBuffer(), SystemUsage()
        annotated = node.create_publisher(Image, '/seam_detection/annotated', qos_profile_sensor_data)
        edge_pub = node.create_publisher(Image, '/seam_detection/edges', qos_profile_sensor_data)
        subscription = node.create_subscription(
            Image, topic, lambda msg: mailbox.put(msg, time.monotonic()),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        errors = []

        def receive():
            try:
                while not stop.is_set() and rclpy.ok():
                    executor.spin_once(timeout_sec=.05)
            except ExternalShutdownException:
                pass
            except Exception as exc:
                errors.append(exc)
                stop.set()

        worker = threading.Thread(target=receive, name='seam-receiver')
        worker.start()
        title = 'FORGE seam edges | R: select region | Q: quit'
        if show:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, 1280, 500)
            for name, value, maximum in [('Canny low', settings.canny_low, 254),
                                          ('Canny high', settings.canny_high, 255),
                                          ('Min width px', settings.min_width, settings.processing_width-1),
                                          ('Max width px', settings.max_width, settings.processing_width)]:
                cv2.createTrackbar(name, title, value, maximum, lambda value: None)
        result, source = None, None
        arrivals, processed = deque(), deque()
        received, processing_ms = 0., 0.
        node.get_logger().info(f'Observing {topic}; candidate edges only, no motion output')
        while rclpy.ok() and not stop.is_set():
            started = time.monotonic()
            if show:
                low = max(1, cv2.getTrackbarPos('Canny low', title))
                minimum = max(2, cv2.getTrackbarPos('Min width px', title))
                settings = replace(settings, canny_low=low,
                    canny_high=max(low+1, cv2.getTrackbarPos('Canny high', title)),
                    min_width=minimum,
                    max_width=max(minimum+1, cv2.getTrackbarPos('Max width px', title)))
            pending, samples = mailbox.take()
            arrivals.extend(samples)
            if pending is not None:
                message, received = pending
                try:
                    source = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                    before = time.monotonic()
                    result = detect(source, settings)
                    processing_ms = (time.monotonic()-before)*1000
                    processed.append(time.monotonic())
                    for pub, data, encoding in [(annotated, result.image, 'bgr8'),
                                                 (edge_pub, result.edges, 'mono8')]:
                        if pub.get_subscription_count():
                            output = bridge.cv2_to_imgmsg(data, encoding=encoding)
                            output.header = message.header
                            pub.publish(output)
                except (ValueError, cv2.error) as exc:
                    result = None
                    node.get_logger().error(f'Cannot process image: {exc}', throttle_duration_sec=2.)
            now = time.monotonic()
            for history in (arrivals, processed):
                while history and history[0] < now-1:
                    history.popleft()
            if show:
                if result is None or now-received > 1.:
                    canvas = np.zeros((400, settings.processing_width*2, 3), np.uint8)
                    text = 'Waiting for RGB images' if not received else 'STALE / INVALID - no current edge result'
                    cv2.putText(canvas, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 200, 255), 2)
                else:
                    canvas = np.hstack((result.image, cv2.cvtColor(result.edges, cv2.COLOR_GRAY2BGR)))
                footer = np.zeros((65, canvas.shape[1], 3), np.uint8)
                for i, text in enumerate((usage.sample(),
                    f'ROS callbacks {len(arrivals)}/s | Processed {len(processed)}/s | Detection {processing_ms:.1f} ms | Pixels only; not a confirmed seam')):
                    cv2.putText(footer, text, (8, 23+i*27), cv2.FONT_HERSHEY_SIMPLEX, .48, (230, 230, 230), 1)
                cv2.imshow(title, np.vstack((canvas, footer)))
                key = cv2.waitKey(1) & 0xff
                if key in (27, ord('q')) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key == ord('r') and source is not None:
                    # Select from an unannotated resized frame; reception remains bounded.
                    preview = cv2.resize(source, (settings.processing_width,
                        round(source.shape[0]*settings.processing_width/source.shape[1])))
                    selection_title = 'Select seam region, then ENTER; ESC cancels'
                    x, y, w, h = cv2.selectROI(selection_title, preview, showCrosshair=True)
                    cv2.destroyWindow(selection_title)
                    if w >= 16 and h >= 16:
                        settings = replace(settings, roi=(x/preview.shape[1], y/preview.shape[0],
                            (x+w)/preview.shape[1], (y+h)/preview.shape[0]))
                        node.get_logger().info(f'ROI for config: {list(settings.roi)}')
                    result = None
            stop.wait(max(0., 1/fps-(time.monotonic()-started)))
        if errors:
            raise RuntimeError('ROS receiver stopped') from errors[0]
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if worker is not None:
            worker.join()
        if executor is not None:
            executor.shutdown()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
