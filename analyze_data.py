import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle

# ---------------------------------------------------------------
# THEME: swap these hex values to match your Aizawa / Marp palette
# ---------------------------------------------------------------
THEME = {
    "bg":        "#0B0D12",   # figure + axes background
    "panel":     "#11141B",   # subtle stat-card fill
    "grid":      "#232834",   # gridlines
    "text":      "#E6E8EE",   # primary text
    "muted":     "#8B93A7",   # secondary text / ticks
    "safe":      "#5EEAD4",   # permissive phase intact
    "hazard":    "#FB7185",   # signal overridden
    "threshold": "#FBBF24",   # AASHTO boundary line
}

FONT_STACK = ["Inter", "SF Pro Display", "Helvetica Neue", "Arial", "DejaVu Sans"]
THRESHOLD_S = 4.5


def pick_font():
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in FONT_STACK:
        if name in installed:
            return name
    return "DejaVu Sans"


def apply_theme():
    plt.rcParams.update({
        "font.family": pick_font(),
        "figure.facecolor": THEME["bg"],
        "axes.facecolor": THEME["bg"],
        "axes.edgecolor": THEME["grid"],
        "axes.labelcolor": THEME["muted"],
        "xtick.color": THEME["muted"],
        "ytick.color": THEME["muted"],
        "text.color": THEME["text"],
        "savefig.facecolor": THEME["bg"],
    })


def stat_card(fig, x, y, w, h, value, label, color):
    """Small rounded stat tile in figure coordinates."""
    ax = fig.add_axes([x, y, w, h])
    ax.set_axis_off()
    ax.add_patch(Rectangle((0, 0), 1, 1, transform=ax.transAxes,
                           facecolor=THEME["panel"], edgecolor=THEME["grid"],
                           linewidth=1))
    ax.text(0.08, 0.56, value, transform=ax.transAxes, fontsize=17,
            fontweight="bold", color=color, va="center")
    ax.text(0.08, 0.20, label.upper(), transform=ax.transAxes, fontsize=7.5,
            color=THEME["muted"], va="center")


def glow_scatter(ax, x, y, color, size, alpha_core, label=None, z=3):
    """Faint wide halo under a crisp core to get a soft glow."""
    ax.scatter(x, y, s=size * 5.5, color=color, alpha=0.06, linewidths=0, zorder=z)
    ax.scatter(x, y, s=size * 2.2, color=color, alpha=0.12, linewidths=0, zorder=z)
    ax.scatter(x, y, s=size, color=color, alpha=alpha_core, linewidths=0,
               label=label, zorder=z + 1)


def generate_safety_charts():
    csv_path = "data_outputs/simulation_telemetry.csv"

    if not os.path.exists(csv_path):
        print(f"Error: Cannot find {csv_path}. Run runner.py first to generate the data!")
        return

    df = pd.read_csv(csv_path)

    total_samples = len(df)
    overrides = int(df["Override_Triggered"].sum())
    max_speed = df["Speed_MPH"].max()
    avg_speed = df["Speed_MPH"].mean()
    override_rate = overrides / total_samples * 100 if total_samples else 0

    print("=== EXECUTING MATHEMATICAL ANALYSIS ON SIMULATION DATA ===")
    print(f"Total Vehicles Tracked over Crest: {total_samples}")
    print(f"High-Risk Structural Conflict Interventions: {overrides}")
    print(f"Average Approach Speed: {avg_speed:.1f} MPH")
    print(f"Maximum Speeding Hazard Captured: {max_speed:.1f} MPH")

    apply_theme()
    fig = plt.figure(figsize=(12, 7.2), dpi=100)
    ax = fig.add_axes([0.075, 0.105, 0.885, 0.63])

    normal = df[df["Override_Triggered"] == 0]
    hazard = df[df["Override_Triggered"] == 1]

    # Shaded hazard band below the threshold
    y_top = max(df["TTI"].max() * 1.06, THRESHOLD_S * 2)
    ax.axhspan(0, THRESHOLD_S, color=THEME["hazard"], alpha=0.07, zorder=1)

    glow_scatter(ax, normal["Speed_MPH"], normal["TTI"], THEME["safe"], 26, 0.75,
                 label="Permissive phase intact")
    glow_scatter(ax, hazard["Speed_MPH"], hazard["TTI"], THEME["hazard"], 34, 0.95,
                 label="Signal overridden to red")

    # Threshold line + inline label
    ax.axhline(THRESHOLD_S, color=THEME["threshold"], linestyle=(0, (6, 4)),
               linewidth=1.4, zorder=2)
    x_min, x_max = df["Speed_MPH"].min() - 2, df["Speed_MPH"].max() + 2
    ax.text(x_min + 0.4, THRESHOLD_S + 0.18, f"AASHTO safety boundary  {THRESHOLD_S}s",
            color=THEME["threshold"], fontsize=9, ha="left", va="bottom")

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(0, y_top)
    ax.set_xlabel("Detected oncoming approach speed (MPH)", fontsize=10.5, labelpad=10)
    ax.set_ylabel("Computed time-to-intersection (s)", fontsize=10.5, labelpad=10)
    ax.grid(True, color=THEME["grid"], linewidth=0.7, alpha=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.tick_params(length=0, labelsize=9.5)

    leg = ax.legend(loc="upper right", frameon=False, fontsize=9.5,
                    labelcolor=THEME["text"], markerscale=1.1, handletextpad=0.4)
    for h in leg.legend_handles:
        h.set_alpha(1)

    # Header
    fig.text(0.075, 0.94, "Dynamic Intersection Controller", fontsize=20,
             fontweight="bold", color=THEME["text"], va="center")
    fig.text(0.075, 0.9, "Speed vs. time-to-intersection (TTI) at the blind crest",
             fontsize=11.5, color=THEME["muted"], va="center")

    # Stat strip
    cw, ch, gap, y0 = 0.205, 0.075, 0.02, 0.84
    x0 = 0.075
    cards = [
        (f"{total_samples:,}", "Vehicles tracked", THEME["text"]),
        (f"{overrides:,}", "Red overrides", THEME["hazard"]),
        (f"{override_rate:.1f}%", "Override rate", THEME["hazard"]),
        (f"{avg_speed:.1f} mph", "Avg approach speed", THEME["safe"]),
    ]
    for i, (v, l, c) in enumerate(cards):
        stat_card(fig, x0 + i * (cw + gap), y0 - 0.075 + 0.0, cw, ch, v, l, c)

    output_image_path = "data_outputs/safety_chart.png"
    fig.savefig(output_image_path, dpi=300)
    plt.close(fig)
    print(f"\n>>> ACADEMIC GRAPH GENERATED SUCCESSFULLY: saved to {output_image_path}")


if __name__ == "__main__":
    generate_safety_charts()
