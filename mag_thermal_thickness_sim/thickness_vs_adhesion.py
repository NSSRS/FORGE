"""Plot the maximum magnetic adhesion allowed by steel thickness.

The upper limit is evaluated at ambient magnet temperature, so only the
empirical steel-thickness factor is applied:

    F_max(t) = F0 * t / (t + t_sat)
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from mag_thermal_thickness_sim.heat_visualizer import NOMINAL_FORCE, SATURATION_THICKNESS_MM


def calculate_adhesion(thicknesses_mm):
    """Return thickness retention and maximum force at ambient temperature."""
    thicknesses = np.asarray(thicknesses_mm, dtype=float)
    retention = thicknesses / (thicknesses + SATURATION_THICKNESS_MM)
    forces = NOMINAL_FORCE * retention
    return retention * 100.0, forces


def save_csv(thicknesses, retention_percent, forces, path):
    with path.open("w", newline="", encoding="utf-8-sig") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "steel_thickness_mm",
                "maximum_retention_percent",
                "maximum_adhesion_force_N",
            ]
        )
        writer.writerows(zip(thicknesses, retention_percent, forces))


def save_plot(thicknesses, retention_percent, forces, path, chinese=False):
    if chinese:
        plt.rcParams["font.sans-serif"] = [
            "Microsoft YaHei",
            "SimHei",
            "Noto Sans CJK SC",
            "DejaVu Sans",
        ]
        plt.rcParams["axes.unicode_minus"] = False
        title = "钢板厚度对磁铁吸附力上限的影响"
        xlabel = "钢板厚度（毫米）"
        left_label = "最大吸力保持率（%）"
        right_label = "最大磁吸力（N）"
        curve_label = "厚度限制下的吸力上限"
        force_label = "吸力上限"
        nominal_label = f"标称吸力 {NOMINAL_FORCE:.0f} N"
    else:
        title = "Steel Thickness vs. Maximum Magnetic Adhesion"
        xlabel = "Steel thickness (mm)"
        left_label = "Maximum retention (% of nominal)"
        right_label = "Maximum adhesion force (N)"
        curve_label = "Thickness-limited retention"
        force_label = "Adhesion-force limit"
        nominal_label = f"Nominal force: {NOMINAL_FORCE:.0f} N"

    fig, retention_ax = plt.subplots(figsize=(10, 6), dpi=160)
    force_ax = retention_ax.twinx()

    retention_line = retention_ax.plot(
        thicknesses,
        retention_percent,
        "o-",
        color="#175cd3",
        linewidth=2.4,
        markersize=5,
        label=curve_label,
    )[0]
    force_line = force_ax.plot(
        thicknesses,
        forces,
        "s--",
        color="#f97316",
        linewidth=2.0,
        markersize=4,
        label=force_label,
    )[0]
    force_ax.axhline(
        NOMINAL_FORCE,
        color="#667085",
        linestyle=":",
        linewidth=1.6,
        label=nominal_label,
    )

    retention_ax.set_title(title, fontsize=16, fontweight="bold")
    retention_ax.set_xlabel(xlabel, fontsize=12)
    retention_ax.set_ylabel(left_label, color="#175cd3", fontsize=12)
    force_ax.set_ylabel(right_label, color="#f97316", fontsize=12)
    retention_ax.tick_params(axis="y", colors="#175cd3")
    force_ax.tick_params(axis="y", colors="#f97316")
    retention_ax.set_ylim(0, 105)
    force_ax.set_ylim(0, NOMINAL_FORCE * 1.05)
    retention_ax.grid(True, alpha=0.25)

    lines = [retention_line, force_line, force_ax.lines[-1]]
    retention_ax.legend(lines, [line.get_label() for line in lines], loc="lower right")
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--minimum", type=int, default=1, help="minimum thickness in mm")
    parser.add_argument("--maximum", type=int, default=100, help="maximum thickness in mm")
    parser.add_argument("--step", type=int, default=1, help="thickness step in mm")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent)
    args = parser.parse_args()
    if args.minimum <= 0 or args.maximum < args.minimum or args.step <= 0:
        parser.error("require 0 < minimum <= maximum and step > 0")

    thicknesses = np.arange(args.minimum, args.maximum + 1, args.step, dtype=float)
    if thicknesses[-1] != args.maximum:
        thicknesses = np.append(thicknesses, float(args.maximum))
    retention_percent, forces = calculate_adhesion(thicknesses)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    save_csv(
        thicknesses,
        retention_percent,
        forces,
        args.output_dir / "thickness_vs_adhesion.csv",
    )
    save_plot(
        thicknesses,
        retention_percent,
        forces,
        args.output_dir / "thickness_vs_adhesion_en.png",
    )
    save_plot(
        thicknesses,
        retention_percent,
        forces,
        args.output_dir / "thickness_vs_adhesion_cn.png",
        chinese=True,
    )
    print("Thickness-versus-adhesion calculation complete.")


if __name__ == "__main__":
    main()
