"""Dependency-free SVG engineering visualization."""

from __future__ import annotations

from html import escape
from math import degrees
from pathlib import Path

from contact import CaseResult
from robot_model import (
    RobotGeometry, front_link_endpoints, module_arm_endpoints, module_wheel_centers,
    transformed_body_outline,
)


def normalized_degrees(angle: float) -> float:
    return (degrees(angle) + 180.0) % 360.0 - 180.0


def write_case_svg(path: Path, robot: RobotGeometry, result: CaseResult) -> None:
    wheels = result.wheel_centers
    points = list(wheels.values()) + list(transformed_body_outline(robot, result.pose)) + [(-0.4, 0.0), (0.9, 0.0)]
    xmin, xmax = min(p[0] for p in points) - 0.10, max(p[0] for p in points) + 0.10
    zmin, zmax = -0.05, max(p[1] for p in points) + 0.12
    width, height, pad = 1200, 650, 55
    scale = min((width - 2 * pad) / (xmax - xmin), (height - 2 * pad) / (zmax - zmin))

    def xy(p):
        return pad + (p[0] - xmin) * scale, height - pad - (p[1] - zmin) * scale

    def line(a, b, **attrs):
        x1, y1 = xy(a); x2, y2 = xy(b)
        style = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in attrs.items())
        return f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" {style}/>'

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="#f8fafc"/>']
    svg.append(line((xmin, 0.0), (xmax, 0.0), stroke="#334155", stroke_width="5"))

    outline = transformed_body_outline(robot, result.pose)
    polygon = " ".join(f"{xy(p)[0]:.2f},{xy(p)[1]:.2f}" for p in outline)
    svg.append(f'<polygon points="{polygon}" fill="#cbd5e1" stroke="#0f172a" stroke-width="3"/>')
    link_start, link_end = front_link_endpoints(robot, result.pose)
    svg.append(line(link_start, link_end, stroke="#475569",
                    stroke_width=max(3, robot.body.front_link_thickness * scale),
                    stroke_linecap="round"))

    specs = [
        ("rear modules ×2", (0.0, 0.0), robot.rear_module, result.pose.rear_module_angle),
        ("front module", (robot.body.front_pivot_x, 0.0), robot.front_module, result.pose.front_module_angle),
    ]
    for label, pivot_body, module, angle in specs:
        pivot = result.pose.transform_body_point(pivot_body)
        arm_ends = module_arm_endpoints(result.pose, pivot_body, module, angle)
        wheel_centers = module_wheel_centers(result.pose, pivot_body, module, angle)
        for endpoint in arm_ends:
            svg.append(line(pivot, endpoint, stroke="#2563eb", stroke_width=max(2, module.arm_thickness * scale), stroke_linecap="round"))
        for endpoint, wheel_center in zip(arm_ends, wheel_centers):
            if endpoint != wheel_center:
                svg.append(line(endpoint, wheel_center, stroke="#7c3aed", stroke_width="5", stroke_linecap="round"))
        px, py = xy(pivot)
        svg.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="7" fill="#f59e0b" stroke="#78350f" stroke-width="2"/>')
        svg.append(f'<text x="{px + 10:.2f}" y="{py - 10:.2f}" font-family="sans-serif" font-size="16">{escape(label)}</text>')

    display_wheels = {
        "rear rear ×2": wheels["rear_left_rear"],
        "rear forward ×2": wheels["rear_left_forward"],
        "front rear": wheels["front_rear"],
        "front forward": wheels["front_forward"],
    }
    for name, center in display_wheels.items():
        cx, cy = xy(center)
        valid_key = "rear_left_" + name.split()[1] if name.startswith("rear ") else name.replace(" ", "_")
        color = "#16a34a" if result.valid_contacts.get(valid_key, False) else "#dc2626"
        svg.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{robot.wheel_radius * scale:.2f}" fill="{color}" fill-opacity="0.25" stroke="{color}" stroke-width="3"/>')
        svg.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="3" fill="#0f172a"/>')
    if robot.front_module.orientation == "transverse":
        front = wheels["front_rear"]
        fx, fy = xy(front)
        svg.append(f'<text x="{fx + 12:.2f}" y="{fy - robot.wheel_radius * scale - 8:.2f}" '
                   f'font-family="sans-serif" font-size="15">front wheels ×2 (lateral)</text>')

    status = "FEASIBLE" if result.feasible else "INFEASIBLE"
    summary = (f"{status} | pitch {degrees(result.pose.pitch):.2f}° | rear rotation "
               f"{normalized_degrees(result.pose.rear_module_angle):.2f}° | front rotation "
               f"{normalized_degrees(result.pose.front_module_angle):.2f}° | max residual "
               f"{result.max_contact_residual * 1000:.3f} mm")
    svg.append(f'<text x="{pad}" y="32" font-family="sans-serif" font-size="20" font-weight="bold">{escape(summary)}</text>')
    if result.collisions:
        text = "Collisions: " + "; ".join(result.collisions)
        svg.append(f'<text x="{pad}" y="58" font-family="sans-serif" font-size="15" fill="#b91c1c">{escape(text)}</text>')
    svg.append('</svg>')
    path.write_text("\n".join(svg), encoding="utf-8")


def write_case_png(path: Path, robot: RobotGeometry, result: CaseResult) -> bool:
    """Write a convenient raster preview when Pillow is installed."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return False

    wheels = result.wheel_centers
    outline = transformed_body_outline(robot, result.pose)
    points = list(wheels.values()) + list(outline) + [(-0.4, 0.0), (0.9, 0.0)]
    xmin, xmax = min(p[0] for p in points) - 0.10, max(p[0] for p in points) + 0.10
    zmin, zmax = -0.05, max(p[1] for p in points) + 0.12
    width, height, pad = 1200, 650, 55
    factor = min((width - 2 * pad) / (xmax - xmin), (height - 2 * pad) / (zmax - zmin))

    def xy(p):
        return pad + (p[0] - xmin) * factor, height - pad - (p[1] - zmin) * factor

    image = Image.new("RGB", (width, height), "#f8fafc")
    draw = ImageDraw.Draw(image)
    draw.line((xy((xmin, 0.0)), xy((xmax, 0.0))), fill="#334155", width=5)
    draw.polygon([xy(p) for p in outline], fill="#cbd5e1", outline="#0f172a", width=3)
    link_start, link_end = front_link_endpoints(robot, result.pose)
    draw.line((xy(link_start), xy(link_end)), fill="#475569",
              width=max(3, round(robot.body.front_link_thickness * factor)))
    specs = [
        ((0.0, 0.0), robot.rear_module, result.pose.rear_module_angle),
        ((robot.body.front_pivot_x, 0.0), robot.front_module, result.pose.front_module_angle),
    ]
    for pivot_body, module, angle in specs:
        pivot = result.pose.transform_body_point(pivot_body)
        arm_ends = module_arm_endpoints(result.pose, pivot_body, module, angle)
        wheel_centers = module_wheel_centers(result.pose, pivot_body, module, angle)
        for endpoint in arm_ends:
            draw.line((xy(pivot), xy(endpoint)), fill="#2563eb", width=max(2, round(module.arm_thickness * factor)))
        for endpoint, wheel_center in zip(arm_ends, wheel_centers):
            if endpoint != wheel_center:
                draw.line((xy(endpoint), xy(wheel_center)), fill="#7c3aed", width=5)
        px, py = xy(pivot)
        draw.ellipse((px - 7, py - 7, px + 7, py + 7), fill="#f59e0b", outline="#78350f", width=2)
    display = [wheels["rear_left_rear"], wheels["rear_left_forward"], wheels["front_rear"], wheels["front_forward"]]
    radius = robot.wheel_radius * factor
    for center in display:
        cx, cy = xy(center)
        draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), fill="#bbf7d0", outline="#16a34a", width=3)
        draw.ellipse((cx - 3, cy - 3, cx + 3, cy + 3), fill="#0f172a")
    if robot.front_module.orientation == "transverse":
        fx, fy = xy(wheels["front_rear"])
        draw.text((fx + 12, fy - radius - 18), "front wheels x2 (lateral)", fill="#334155")
    status = "FEASIBLE" if result.feasible else "INFEASIBLE"
    summary = (f"{status} | pitch {degrees(result.pose.pitch):.2f} deg | "
               f"rear {normalized_degrees(result.pose.rear_module_angle):.2f} deg | "
               f"front {normalized_degrees(result.pose.front_module_angle):.2f} deg | "
               f"max residual {result.max_contact_residual * 1000:.3f} mm")
    draw.text((pad, 18), summary, fill="#0f172a")
    if result.collisions:
        draw.text((pad, 39), "Collisions: " + "; ".join(result.collisions), fill="#b91c1c")
    image.save(path)
    return True
