"""Rigid body and rigid dual-arm suspension geometry."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, radians, sin


Vec2 = tuple[float, float]


def add(a: Vec2, b: Vec2) -> Vec2:
    return a[0] + b[0], a[1] + b[1]


def sub(a: Vec2, b: Vec2) -> Vec2:
    return a[0] - b[0], a[1] - b[1]


def scale(a: Vec2, factor: float) -> Vec2:
    return a[0] * factor, a[1] * factor


def rotate(vector: Vec2, angle: float) -> Vec2:
    c, s = cos(angle), sin(angle)
    return c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]


@dataclass(frozen=True)
class ModuleGeometry:
    name: str
    orientation: str
    rear_arm_length: float
    rear_arm_angle_deg: float
    forward_arm_length: float
    forward_arm_angle_deg: float
    arm_thickness: float
    rotation_locked: bool = False
    caster_offset_x: float = 0.0
    caster_offset_z: float = 0.0
    caster_initial_yaw_deg: float = 0.0
    caster_alignment_rate: float = 3.0

    @property
    def arm_vectors(self) -> tuple[Vec2, Vec2]:
        if self.orientation == "transverse":
            # Longitudinal side-view projection of arms that extend left/right.
            # Their lateral separation is retained in the plan-view renderer.
            rear_drop = -self.rear_arm_length * sin(radians(self.rear_arm_angle_deg))
            forward_drop = -self.forward_arm_length * sin(radians(self.forward_arm_angle_deg))
            return (0.0, rear_drop), (0.0, forward_drop)
        rear_angle = pi + radians(self.rear_arm_angle_deg)
        forward_angle = -radians(self.forward_arm_angle_deg)
        rear = scale((cos(rear_angle), sin(rear_angle)), self.rear_arm_length)
        forward = scale((cos(forward_angle), sin(forward_angle)), self.forward_arm_length)
        return rear, forward

    @property
    def reference_vectors(self) -> tuple[Vec2, Vec2]:
        """Pivot-to-wheel vectors at zero caster yaw."""
        rear, forward = self.arm_vectors
        caster = (self.caster_offset_x, self.caster_offset_z)
        return add(rear, caster), add(forward, caster)

    @property
    def transverse_offsets(self) -> tuple[float, float]:
        """Signed lateral wheel offsets used by a transverse module."""
        if self.orientation != "transverse":
            return 0.0, 0.0
        left = self.rear_arm_length * cos(radians(self.rear_arm_angle_deg))
        right = -self.forward_arm_length * cos(radians(self.forward_arm_angle_deg))
        return left, right


@dataclass(frozen=True)
class BodyGeometry:
    length: float
    rear_width: float
    equal_side_length: float
    thickness: float
    front_pivot_extension: float
    front_link_thickness: float
    center_of_mass: Vec2

    @property
    def front_pivot_x(self) -> float:
        return self.length + self.front_pivot_extension

    @property
    def outline(self) -> tuple[Vec2, Vec2, Vec2, Vec2]:
        return ((0.0, 0.0), (self.length, 0.0),
                (self.length, self.thickness), (0.0, self.thickness))


@dataclass(frozen=True)
class RobotGeometry:
    body: BodyGeometry
    rear_module: ModuleGeometry
    front_module: ModuleGeometry
    wheel_radius: float
    wheel_width: float


@dataclass(frozen=True)
class RobotPose:
    x: float
    z: float
    pitch: float
    rear_module_angle: float
    front_module_angle: float

    def transform_body_point(self, point: Vec2) -> Vec2:
        return add((self.x, self.z), rotate(point, self.pitch))


def module_wheel_centers(
    pose: RobotPose,
    pivot_body: Vec2,
    module: ModuleGeometry,
    module_angle: float,
) -> tuple[Vec2, Vec2]:
    pivot = pose.transform_body_point(pivot_body)
    absolute_angle = pose.pitch + module_angle
    rear, forward = module.reference_vectors
    return add(pivot, rotate(rear, absolute_angle)), add(pivot, rotate(forward, absolute_angle))


def module_arm_endpoints(
    pose: RobotPose,
    pivot_body: Vec2,
    module: ModuleGeometry,
    module_angle: float,
) -> tuple[Vec2, Vec2]:
    pivot = pose.transform_body_point(pivot_body)
    absolute_angle = pose.pitch + module_angle
    rear, forward = module.arm_vectors
    return add(pivot, rotate(rear, absolute_angle)), add(pivot, rotate(forward, absolute_angle))


def all_wheel_centers(robot: RobotGeometry, pose: RobotPose) -> dict[str, Vec2]:
    rear = module_wheel_centers(pose, (0.0, 0.0), robot.rear_module, pose.rear_module_angle)
    front = module_wheel_centers(
        pose, (robot.body.front_pivot_x, 0.0), robot.front_module, pose.front_module_angle
    )
    return {
        "rear_left_rear": rear[0],
        "rear_left_forward": rear[1],
        "rear_right_rear": rear[0],
        "rear_right_forward": rear[1],
        "front_rear": front[0],
        "front_forward": front[1],
    }


def transformed_body_outline(robot: RobotGeometry, pose: RobotPose) -> tuple[Vec2, ...]:
    return tuple(pose.transform_body_point(p) for p in robot.body.outline)


def front_link_endpoints(robot: RobotGeometry, pose: RobotPose) -> tuple[Vec2, Vec2]:
    """Rigid link from the triangular body's front vertex to the front module pivot."""
    body_front = pose.transform_body_point((robot.body.length, 0.0))
    front_pivot = pose.transform_body_point((robot.body.front_pivot_x, 0.0))
    return body_front, front_pivot
