"""Sweep steel thickness and plot peak magnet temperature.

Default model inputs:
    speed = 8 mm/s, power = 9 kW, wheel distance = 300 mm,
    cooling = 10 W/(m2 K), contact conductance = 500 W/(m2 K),
    simulation time = 60 s.
"""

import argparse
import csv
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SPEED_M_S = 0.008
POWER_KW = 9.0
WHEEL_DISTANCE_MM = 300.0
COOLING_W_M2K = 10.0
CONTACT_W_M2K = 500.0
SIMULATION_TIME_S = 60.0


def run_case(thickness_mm):
    """Run one independent thickness case and return its peak temperatures."""
    import mag_thermal_thickness_sim.heat_visualizer as model

    model.PLATE_HEIGHT = 2.0
    model.NY = 250
    diagnostics = {}
    surface, magnet_history, _ = model.simulate(
        SPEED_M_S,
        POWER_KW,
        WHEEL_DISTANCE_MM,
        float(thickness_mm),
        COOLING_W_M2K,
        CONTACT_W_M2K,
        SIMULATION_TIME_S,
        diagnostics=diagnostics,
    )
    contact = np.asarray(diagnostics["contact_steel_temperature"], dtype=float)
    return (
        float(thickness_mm),
        float(np.max(magnet_history)),
        float(np.nanmax(contact)),
        float(np.max(surface)),
    )


def save_csv(rows, path):
    with path.open("w", newline="", encoding="utf-8-sig") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "thickness_mm",
                "peak_magnet_temperature_C",
                "peak_steel_under_wheel_C",
                "peak_plate_surface_temperature_C",
            ]
        )
        writer.writerows(rows)


def save_plot(rows, path, chinese=False):
    thicknesses = [row[0] for row in rows]
    peak_magnet = [row[1] for row in rows]
    peak_contact = [row[2] for row in rows]

    if chinese:
        plt.rcParams["font.sans-serif"] = [
            "Microsoft YaHei",
            "SimHei",
            "Noto Sans CJK SC",
            "DejaVu Sans",
        ]
        plt.rcParams["axes.unicode_minus"] = False
        title = "钢板厚度对峰值温度的影响"
        xlabel = "钢板厚度（毫米）"
        ylabel = "峰值温度（摄氏度）"
        magnet_label = "磁体峰值温度"
        contact_label = "轮下钢板峰值温度"
    else:
        title = "Peak Temperature vs. Steel Thickness"
        xlabel = "Steel thickness (mm)"
        ylabel = "Peak temperature (°C)"
        magnet_label = "Peak magnet temperature"
        contact_label = "Peak steel temperature under wheel"

    fig, ax = plt.subplots(figsize=(10, 6), dpi=160)
    ax.plot(thicknesses, peak_magnet, "o-", linewidth=2.2, label=magnet_label)
    ax.plot(thicknesses, peak_contact, "s--", linewidth=1.8, label=contact_label)
    ax.set_title(title, fontsize=16, fontweight="bold")
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=int, default=2, help="thickness step in mm")
    parser.add_argument(
        "--workers", type=int, default=max(1, min(4, os.cpu_count() or 1))
    )
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent)
    args = parser.parse_args()
    if args.step <= 0:
        parser.error("--step must be positive")

    thicknesses = list(range(10, 41, args.step))
    if thicknesses[-1] != 40:
        thicknesses.append(40)
    print(f"Running {len(thicknesses)} thickness cases with {args.workers} workers...")
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        rows = sorted(executor.map(run_case, thicknesses))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    save_csv(rows, args.output_dir / "thickness_vs_heat.csv")
    save_plot(rows, args.output_dir / "thickness_vs_heat_en.png")
    save_plot(rows, args.output_dir / "thickness_vs_heat_cn.png", chinese=True)
    print("Thickness sweep complete.")


if __name__ == "__main__":
    main()
