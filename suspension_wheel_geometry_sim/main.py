"""Run the configured magnetic robot flat-plate geometry case."""

from __future__ import annotations

import argparse
import csv
import json
from math import degrees, sqrt
from pathlib import Path

from contact import attach_runtime_constraints, solve_flat_pose
from robot_model import BodyGeometry, ModuleGeometry, RobotGeometry
from surface_model import FlatSurface
from visualize import normalized_degrees, write_case_png, write_case_svg


PROJECT_DIR = Path(__file__).resolve().parent


def load_config(path: Path) -> dict:
    # JSON is a strict subset of YAML, keeping this file YAML-compatible without PyYAML.
    return json.loads(path.read_text(encoding="utf-8"))


def build_robot(config: dict) -> RobotGeometry:
    rc = config["robot"]
    rear_width = rc.get("body_rear_width", 0.600)
    equal_side = rc.get("body_equal_side_length", 0.500)
    if equal_side <= rear_width / 2.0:
        raise ValueError("body_equal_side_length must be greater than half body_rear_width")
    body_length = sqrt(equal_side ** 2 - (rear_width / 2.0) ** 2)
    body = BodyGeometry(
        length=body_length,
        rear_width=rear_width,
        equal_side_length=equal_side,
        thickness=rc["body_thickness"],
        front_pivot_extension=rc["front_pivot_extension"],
        front_link_thickness=rc.get("front_link_thickness", 0.030),
        center_of_mass=(body_length / 3.0, rc.get("center_of_mass_z", rc["body_thickness"] / 2.0)),
    )

    def module(name: str, values: dict) -> ModuleGeometry:
        return ModuleGeometry(name=name, **values)

    robot = RobotGeometry(
        body=body,
        rear_module=module("rear", config["rear_module"]),
        front_module=module("front", config["front_module"]),
        wheel_radius=rc["wheel_radius"],
        wheel_width=rc.get("wheel_width", 0.030),
    )
    checks = config["contact"]
    return attach_runtime_constraints(
        robot,
        checks["minimum_wheel_gap"],
        checks["minimum_wheel_body_gap"],
        rc["minimum_clearance"],
    )


def write_csv(path: Path, result) -> None:
    row = {
        "feasible": result.feasible,
        "exact_contact_pose": result.exact_contact_pose,
        "valid_contact_count": sum(result.valid_contacts.values()),
        "body_height_m": result.pose.z,
        "body_pitch_deg": degrees(result.pose.pitch),
        "rear_module_rotation_deg": normalized_degrees(result.pose.rear_module_angle),
        "front_module_rotation_deg": normalized_degrees(result.pose.front_module_angle),
        "max_contact_residual_m": result.max_contact_residual,
        "rms_contact_residual_m": result.rms_contact_residual,
        "minimum_body_clearance_m": result.min_body_clearance,
        "stability_margin_m": result.stability_margin,
        "collision_count": len(result.collisions),
        "collisions": "; ".join(result.collisions),
    }
    for name, center in result.wheel_centers.items():
        row[f"{name}_x_m"] = center[0]
        row[f"{name}_z_m"] = center[1]
        row[f"{name}_residual_m"] = result.residuals[name]
        row[f"{name}_contact"] = result.valid_contacts[name]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=row.keys())
        writer.writeheader()
        writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_DIR / "config.yaml",
        help="Configuration file (default: config.yaml beside main.py)",
    )
    parser.add_argument(
        "--batch",
        action="store_true",
        help="Generate results without opening the interactive simulation window",
    )
    args = parser.parse_args()
    config_path = args.config.resolve()
    config = load_config(config_path)
    robot = build_robot(config)
    result = solve_flat_pose(robot, FlatSurface(), config["contact"]["tolerance"])
    output = Path(config["output"]["directory"])
    if not output.is_absolute():
        output = config_path.parent / output
    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / "results.csv", result)
    write_case_svg(output / "robot_on_surface.svg", robot, result)
    wrote_png = write_case_png(output / "robot_on_surface.png", robot, result)

    print(f"Feasible: {result.feasible}")
    print(f"Valid contacts: {sum(result.valid_contacts.values())}/6")
    print(f"Maximum contact residual: {result.max_contact_residual * 1000:.6f} mm")
    print(f"Body pitch: {degrees(result.pose.pitch):.3f} deg")
    print(f"Minimum body clearance: {result.min_body_clearance * 1000:.3f} mm")
    print(f"Stability margin: {result.stability_margin * 1000:.3f} mm")
    print("Collisions: " + ("; ".join(result.collisions) if result.collisions else "none"))
    print(f"Wrote {output / 'results.csv'}")
    print(f"Wrote {output / 'robot_on_surface.svg'}")
    if wrote_png:
        print(f"Wrote {output / 'robot_on_surface.png'}")
    if not args.batch:
        try:
            from simulation_view import show_simulation
            show_simulation(robot, result, config)
        except Exception as error:
            print(f"Could not open simulation window: {error}")
            print("Batch results were still generated successfully.")
    return 0 if result.feasible else 2


if __name__ == "__main__":
    raise SystemExit(main())
