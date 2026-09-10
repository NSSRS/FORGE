"""Flat-plate contact pose solution and geometric feasibility checks."""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, cos, degrees, hypot, pi, sin, sqrt

from robot_model import (
    ModuleGeometry, RobotGeometry, RobotPose, Vec2, add, all_wheel_centers,
    front_link_endpoints, module_arm_endpoints, module_wheel_centers, rotate, sub,
    transformed_body_outline,
)
from surface_model import FlatSurface


@dataclass
class CaseResult:
    pose: RobotPose
    wheel_centers: dict[str, Vec2]
    residuals: dict[str, float]
    valid_contacts: dict[str, bool]
    collisions: list[str]
    min_body_clearance: float
    max_contact_residual: float
    rms_contact_residual: float
    stability_margin: float
    exact_contact_pose: bool
    feasible: bool


def _equal_height_orientations(module: ModuleGeometry) -> list[float]:
    """Absolute assembly angles that put both wheel centers at equal z."""
    rear, forward = module.reference_vectors
    dx, dz = sub(rear, forward)
    if hypot(dx, dz) < 1e-12:
        # Identical longitudinal projections: the wheels differ laterally.
        return [0.0]
    base = atan2(-dz, dx)
    candidates = [base, base + pi]
    # Prefer solutions with the wheel pair below its pivot, but keep both for checks.
    return sorted(candidates, key=lambda a: (rotate(rear, a)[1] + rotate(forward, a)[1]) / 2.0)


def _required_pivot_height(module: ModuleGeometry, absolute_angle: float, radius: float) -> float:
    rear, forward = module.reference_vectors
    mean_z = (rotate(rear, absolute_angle)[1] + rotate(forward, absolute_angle)[1]) / 2.0
    return radius - mean_z


def solve_flat_pose(robot: RobotGeometry, surface: FlatSurface, tolerance: float) -> CaseResult:
    separation = robot.body.front_pivot_x
    candidates: list[tuple[float, RobotPose]] = []
    for rear_abs in _equal_height_orientations(robot.rear_module):
        rear_height = _required_pivot_height(robot.rear_module, rear_abs, robot.wheel_radius)
        if robot.front_module.rotation_locked:
            # The transverse front assembly is fixed to the body. Find the body
            # pitch that places its caster wheel centers on the flat plate.
            front_vector = robot.front_module.reference_vectors[0]

            def front_residual(pitch: float) -> float:
                front_pivot_z = rear_height + separation * sin(pitch)
                return front_pivot_z + rotate(front_vector, pitch)[1] - robot.wheel_radius

            samples = 1440
            previous_pitch = -pi / 2
            previous_value = front_residual(previous_pitch)
            roots: list[float] = []
            for index in range(1, samples + 1):
                pitch = -pi / 2 + index * pi / samples
                value = front_residual(pitch)
                if value == 0.0 or value * previous_value < 0.0:
                    low, high = previous_pitch, pitch
                    for _ in range(60):
                        middle = (low + high) / 2.0
                        if front_residual(low) * front_residual(middle) <= 0.0:
                            high = middle
                        else:
                            low = middle
                    roots.append((low + high) / 2.0)
                previous_pitch, previous_value = pitch, value
            for pitch in roots:
                pose = RobotPose(
                    x=0.0,
                    z=surface.height + rear_height,
                    pitch=pitch,
                    rear_module_angle=rear_abs - pitch,
                    front_module_angle=0.0,
                )
                candidates.append((abs(pitch), pose))
        else:
            for front_abs in _equal_height_orientations(robot.front_module):
                front_height = _required_pivot_height(robot.front_module, front_abs, robot.wheel_radius)
                ratio = (front_height - rear_height) / separation
                if abs(ratio) <= 1.0:
                    for pitch in (asin(ratio), pi - asin(ratio)):
                        if abs(pitch) <= pi / 2:
                            pose = RobotPose(
                                x=0.0,
                                z=surface.height + rear_height,
                                pitch=pitch,
                                rear_module_angle=rear_abs - pitch,
                                front_module_angle=front_abs - pitch,
                            )
                            candidates.append((abs(pitch), pose))
    if not candidates:
        raise RuntimeError("No exact rigid-pair flat-plate pose exists for the configured geometry.")

    evaluated = [
        _evaluate(robot, pose, surface, tolerance)
        for _, pose in sorted(candidates, key=lambda item: item[0])
    ]
    collision_free = [case for case in evaluated if not case.collisions]
    return (collision_free or evaluated)[0]


def _point_segment_distance(point: Vec2, a: Vec2, b: Vec2) -> float:
    ab = sub(b, a)
    denom = ab[0] ** 2 + ab[1] ** 2
    if denom == 0.0:
        return hypot(point[0] - a[0], point[1] - a[1])
    t = max(0.0, min(1.0, ((point[0] - a[0]) * ab[0] + (point[1] - a[1]) * ab[1]) / denom))
    nearest = (a[0] + t * ab[0], a[1] + t * ab[1])
    return hypot(point[0] - nearest[0], point[1] - nearest[1])


def _point_polygon_distance(point: Vec2, polygon: tuple[Vec2, ...]) -> float:
    return min(_point_segment_distance(point, polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon)))


def _point_in_convex_polygon(point: Vec2, polygon: tuple[Vec2, ...]) -> bool:
    signs = []
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        cross = (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0])
        if abs(cross) > 1e-12:
            signs.append(cross > 0)
    return not signs or all(signs) or not any(signs)


def _evaluate(robot: RobotGeometry, pose: RobotPose, surface: FlatSurface, tolerance: float) -> CaseResult:
    wheels = all_wheel_centers(robot, pose)
    residuals = {name: surface.signed_distance(p) - robot.wheel_radius for name, p in wheels.items()}
    valid = {name: abs(value) <= tolerance for name, value in residuals.items()}
    unique = {
        "rear_rear": wheels["rear_left_rear"],
        "rear_forward": wheels["rear_left_forward"],
        "front_rear": wheels["front_rear"],
        "front_forward": wheels["front_forward"],
    }
    outline = transformed_body_outline(robot, pose)
    collisions: list[str] = []

    min_clearance = min(p[1] - surface.height for p in outline)
    if min_clearance < robot.body_clearance_min:
        collisions.append("body-to-surface clearance")

    link_start, link_end = front_link_endpoints(robot, pose)
    link_clearance = min(link_start[1], link_end[1]) - robot.body.front_link_thickness / 2.0 - surface.height
    min_clearance = min(min_clearance, link_clearance)
    if link_clearance < robot.body_clearance_min:
        collisions.append("front-link-to-surface clearance")

    items = list(unique.items())
    for i, (name_a, a) in enumerate(items):
        for name_b, b in items[i + 1:]:
            if ({name_a, name_b} == {"front_rear", "front_forward"}
                    and robot.front_module.orientation == "transverse"):
                continue
            if hypot(a[0] - b[0], a[1] - b[1]) < 2 * robot.wheel_radius + robot.minimum_wheel_gap:
                collisions.append(f"wheel overlap: {name_a}/{name_b}")

    for name, center in unique.items():
        distance = _point_polygon_distance(center, outline)
        if _point_in_convex_polygon(center, outline) or distance < robot.wheel_radius + robot.minimum_wheel_body_gap:
            collisions.append(f"wheel-to-body: {name}")

    # Arms are capsules. Surface penetration uses the lowest segment endpoint.
    module_specs = [
        ("rear", (0.0, 0.0), robot.rear_module, pose.rear_module_angle),
        ("front", (robot.body.front_pivot_x, 0.0), robot.front_module, pose.front_module_angle),
    ]
    for label, pivot_body, module, angle in module_specs:
        pivot = pose.transform_body_point(pivot_body)
        centers = module_arm_endpoints(pose, pivot_body, module, angle)
        envelope = module.arm_thickness / 2.0
        for side, endpoint in zip(("rear", "forward"), centers):
            if min(pivot[1], endpoint[1]) - envelope < surface.height - tolerance:
                collisions.append(f"arm-to-surface: {label}_{side}")

    values = list(residuals.values())
    maximum = max(abs(v) for v in values)
    rms = sqrt(sum(v * v for v in values) / len(values))
    contact_x = sorted(p[0] for name, p in wheels.items() if valid[name])
    com = pose.transform_body_point(robot.body.center_of_mass)
    stability = min(com[0] - contact_x[0], contact_x[-1] - com[0]) if contact_x else float("-inf")
    exact = maximum <= tolerance
    feasible = exact and not collisions and min_clearance >= robot.body_clearance_min and stability >= 0.0
    return CaseResult(pose, wheels, residuals, valid, collisions, min_clearance, maximum, rms, stability, exact, feasible)


def attach_runtime_constraints(robot: RobotGeometry, minimum_wheel_gap: float, minimum_wheel_body_gap: float, minimum_body_clearance: float) -> RobotGeometry:
    # Keep the core dataclasses focused on geometry while exposing configured checks.
    object.__setattr__(robot, "minimum_wheel_gap", minimum_wheel_gap)
    object.__setattr__(robot, "minimum_wheel_body_gap", minimum_wheel_body_gap)
    object.__setattr__(robot, "body_clearance_min", minimum_body_clearance)
    return robot
