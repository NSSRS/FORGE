"""Read-only orientation display for vendor RPY; no position or ENU claim."""
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def rotation_matrix(rpy):
    """Display convention: Rz(yaw) Ry(pitch) Rx(roll), radians."""
    roll, pitch, yaw = rpy
    if not all(math.isfinite(value) for value in rpy):
        raise ValueError('Orientation angles must be finite')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


def display_rotation(rpy):
    """Apply the bench-observed roll and pitch directions only to the visual model."""
    roll, pitch, yaw = rpy
    return rotation_matrix((-roll, -pitch, yaw))


def body_corners():
    """Illustrative enclosure: short X side, long Y side, thin Z side."""
    return np.array([[x,y,z] for x in (-.55,.55) for y in (-1,1) for z in (-.2,.2)])


# Camera at +X,+Y,+Z looking toward the origin. Screen Y points down;
# depth points toward the observer. Keep projection separate from body rotation.
CAMERA_DEPTH = np.array([1., 1., 1.]) / math.sqrt(3)
CAMERA_RIGHT = np.array([-1., 1., 0.]) / math.sqrt(2)
CAMERA_UP = np.cross(CAMERA_DEPTH, CAMERA_RIGHT)
AXIS_COLORS = [(80, 80, 245), (80, 220, 100), (245, 170, 70)]


def project_world(points, center=(360, 330), scale=125):
    """Orthographic world-to-screen projection, with equal scale on both axes."""
    points = np.asarray(points)
    return points @ np.array([CAMERA_RIGHT, -CAMERA_UP]).T * scale + center


def draw_axes(image, rotation, center, scale, prefix='', sensor_axes=False):
    origin = tuple(np.rint(center).astype(int))
    for label, axis_color, axis in zip('XYZ', AXIS_COLORS, np.eye(3)):
        if sensor_axes and label in 'XY':
            # Physical IMU +X/+Y markings point along model-local -X/-Y.
            # This is a display annotation, not a ROS frame transform.
            axis = -axis
            label = '+' + label
        end = tuple(np.rint(project_world([rotation @ axis], center, scale)[0]).astype(int))
        cv2.arrowedLine(image, origin, end, axis_color, 2, cv2.LINE_AA, tipLength=.12)
        cv2.putText(image, prefix + label, end, cv2.FONT_HERSHEY_SIMPLEX,
                    .5, axis_color, 1, cv2.LINE_AA)


def render(rpy=None, age=None):
    image = np.full((620, 900, 3), (28, 24, 20), dtype=np.uint8)
    live = age is not None and age <= 0.5
    status = 'WAITING' if rpy is None else ('LIVE' if live else 'STALE - holding last sample')
    color = (100, 220, 110) if live else (80, 170, 245)
    cv2.putText(image, 'IMU ORIENTATION', (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (235,235,235), 2)
    cv2.putText(image, status, (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    cv2.putText(image, 'Display: roll/pitch reversed | IMU +X/+Y = model -X/-Y',
                (30, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (170,170,170), 1)
    if rpy is not None:
        rotation = display_rotation(rpy)
        corners = body_corners()
        world = corners @ rotation.T
        pixels = np.rint(project_world(world)).astype(int)
        # Painter's algorithm: opaque far faces first, near faces last.
        faces = [(0,1,3,2), (4,6,7,5), (0,4,5,1),
                 (2,3,7,6), (0,2,6,4), (1,5,7,3)]
        shades = [(65,65,140), (90,90,205), (65,115,65),
                  (85,165,85), (130,90,55), (195,135,75)]
        order = sorted(range(6), key=lambda i: np.mean(world[list(faces[i])], axis=0) @ CAMERA_DEPTH)
        for i in order:
            polygon = pixels[list(faces[i])]
            cv2.fillConvexPoly(image, polygon, shades[i], cv2.LINE_AA)
            cv2.polylines(image, [polygon], True, (210,210,210), 1, cv2.LINE_AA)
        draw_axes(image, rotation, (360,330), 185, sensor_axes=True)
        draw_axes(image, np.eye(3), (110,430), 65, prefix='0')
        cv2.putText(image, 'Fixed reference', (30,505), cv2.FONT_HERSHEY_SIMPLEX,
                    .48, (190,190,190), 1)
        for index, (name, value) in enumerate(zip(('Roll', 'Pitch', 'Yaw'), rpy)):
            cv2.putText(image, f'{name}: {math.degrees(value):+7.2f} deg', (625,250+45*index),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (230,230,230), 1)
        cv2.putText(image, ('Last sample: unknown age' if age is None else f'Last sample: {age:.2f}s ago'), (30,530),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1)
    else:
        cv2.putText(image, 'Waiting for valid IMU angles...', (230,330),
                    cv2.FONT_HERSHEY_SIMPLEX, .8, (200,200,200), 1)
    cv2.putText(image, 'Orientation only: mounting / navigation frame not calibrated.', (30,575),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (170,170,170), 1)
    cv2.putText(image, 'Esc or Q closes this window (and the combined test session).', (30,603),
                cv2.FONT_HERSHEY_SIMPLEX, .48, (170,170,170), 1)
    return image


class ImuViewer(Node):
    def __init__(self):
        super().__init__('imu_viewer')
        self.rpy = None
        self.received = None
        self.last_stamp = 0
        self.subscription = self.create_subscription(
            Vector3Stamped, '/imu/data_rpy', self.receive, qos_profile_sensor_data)

    def receive(self, message):
        values = (message.vector.x, message.vector.y, message.vector.z)
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if (not all(math.isfinite(value) for value in values) or stamp <= self.last_stamp
                or not 0 <= age <= 0.5):
            return
        self.rpy, self.received, self.last_stamp = values, time.monotonic()-age, stamp


def main(args=None):
    rclpy.init(args=args)
    node = ImuViewer()
    title = 'FORGE - IMU orientation'
    try:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, 900, 620)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            age = None if node.received is None else time.monotonic()-node.received
            cv2.imshow(title, render(node.rpy, age))
            if cv2.waitKey(1) & 0xff in (27, ord('q')):
                break
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
