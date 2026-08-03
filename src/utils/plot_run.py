#!/usr/bin/env python3
"""Plot a telemetry CSV produced by the C++ sim (results/telemetry/run.csv)."""

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

INK = "#0b0b0b"
MUTED = "#898781"
GRID = "#e1e0d9"
LINE = "#2a78d6"
SURFACE = "#fcfcfb"


def plot_run(csv_path: Path, out_path: Path) -> None:
    df = pd.read_csv(csv_path)
    speed_kmh = (df["vx"] ** 2 + df["vy"] ** 2).pow(0.5) * 3.6

    fig, axes = plt.subplots(2, 2, figsize=(11, 8), facecolor=SURFACE)
    fig.suptitle(f"Run telemetry — {csv_path.name}", color=INK, fontsize=13)

    def style(ax, title, ylabel):
        ax.set_facecolor(SURFACE)
        ax.set_title(title, color=INK, fontsize=10, loc="left")
        ax.set_ylabel(ylabel, color=MUTED, fontsize=9)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.grid(True, color=GRID, linewidth=0.8)
        for spine in ax.spines.values():
            spine.set_color(GRID)

    ax = axes[0, 0]
    ax.plot(df["x"], df["y"], color=LINE, linewidth=2)
    style(ax, "Trajectory", "y (m)")
    ax.set_xlabel("x (m)", color=MUTED, fontsize=9)
    ax.set_aspect("equal", adjustable="datalim")

    ax = axes[0, 1]
    ax.plot(df["time"], speed_kmh, color=LINE, linewidth=2)
    style(ax, "Speed", "km/h")
    ax.set_xlabel("time (s)", color=MUTED, fontsize=9)

    ax = axes[1, 0]
    ax.plot(df["time"], df["yaw_rate"], color=LINE, linewidth=2)
    style(ax, "Yaw rate", "rad/s")
    ax.set_xlabel("time (s)", color=MUTED, fontsize=9)

    ax = axes[1, 1]
    ax.plot(df["time"], df["lat_accel_g"], color=LINE, linewidth=2)
    style(ax, "Lateral acceleration", "g")
    ax.set_xlabel("time (s)", color=MUTED, fontsize=9)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    print(f"Wrote {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv", type=Path, default=Path("results/telemetry/run.csv"),
        help="Path to telemetry CSV (default: results/telemetry/run.csv)",
    )
    parser.add_argument(
        "--out", type=Path, default=Path("results/logs/run_plot.png"),
        help="Path to write the plot PNG (default: results/logs/run_plot.png)",
    )
    args = parser.parse_args()
    plot_run(args.csv, args.out)


if __name__ == "__main__":
    main()