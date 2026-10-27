"""Render the hardware architecture used by FireResQ_Pitch.md.

Run with: python3 docs/assets/build_architecture.py
Requires matplotlib.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.path import Path as DrawingPath


OUTPUT = Path(__file__).resolve().parent
INK = "#172B45"
MUTED = "#53677D"
SIGNAL = "#48677F"
ACCENT = "#126B72"

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11})
fig, ax = plt.subplots(figsize=(12.8, 5.0), facecolor="white")
ax.set(xlim=(0, 12.8), ylim=(0, 5.0), aspect="equal")
ax.axis("off")
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)


def box(x, y, width, title, detail, *, fill="#F1F5F9", edge="#CBD5E1"):
    height = 1.3
    ax.add_patch(
        FancyBboxPatch(
            (x, y), width, height,
            boxstyle="round,pad=0.02,rounding_size=0.13",
            facecolor=fill, edgecolor=edge, linewidth=1.2, zorder=3,
        )
    )
    ax.text(
        x + width / 2, y + 0.91, title,
        ha="center", va="center", weight="bold", fontsize=12, color=INK,
        zorder=4,
    )
    ax.text(
        x + width / 2, y + 0.43, detail,
        ha="center", va="center", fontsize=10.5, color=MUTED,
        linespacing=1.45, zorder=4,
    )


def arrow(points, *, color=SIGNAL):
    path = DrawingPath(points, [DrawingPath.MOVETO] + [DrawingPath.LINETO] * (len(points) - 1))
    ax.add_patch(
        FancyArrowPatch(
            path=path, arrowstyle="-|>", mutation_scale=13,
            linewidth=1.6, color=color, zorder=2,
        )
    )


def label(x, y, text, *, color=MUTED, align="center"):
    ax.text(
        x, y, text, ha=align, va="center", fontsize=9.5, color=color,
        bbox={"facecolor": "white", "edgecolor": "none", "pad": 2},
        zorder=5,
    )


box(0.35, 3.25, 2.55, "Camera", "RGB / RGB-D\nTargets + surroundings")
box(4.15, 3.25, 3.65, "Laptop · ROS 2", "Vision + mapping\nRescue priority + navigation", fill="#EAF4F4", edge="#A7CDCE")
box(4.15, 0.55, 3.65, "ESP32", "Motor control\nElectromagnet switching", fill="#EAF4F4", edge="#A7CDCE")
box(0.35, 0.55, 2.55, "Wheel encoders", "Measure wheel movement")
box(9.1, 3.25, 3.3, "Motor driver + motors", "Two driven wheels\nPassive caster for balance")
box(9.1, 0.55, 3.3, "MOSFET + electromagnet", "Front-mounted pickup\nAttach + release")

arrow([(2.93, 3.9), (4.12, 3.9)])
label(3.52, 4.17, "Images")

arrow([(2.93, 1.2), (4.12, 1.2)])
label(3.52, 1.48, "Wheel ticks")

# Separate arrows preserve the commands and odometry feedback directions.
arrow([(6.7, 3.22), (6.7, 1.88)], color=ACCENT)
label(6.95, 2.57, "Drive +\npickup commands", align="left")
arrow([(5.25, 1.88), (5.25, 3.22)], color=ACCENT)
label(5.0, 2.57, "Motion\nfeedback", align="right")

arrow([(7.83, 1.48), (8.48, 1.48), (8.48, 3.9), (9.07, 3.9)])
label(8.48, 2.75, "Speed +\ndirection")
arrow([(7.83, 0.94), (9.07, 0.94)])
label(8.48, 0.67, "On / off")

fig.savefig(OUTPUT / "fireresq_architecture.png", dpi=180, facecolor="white")
plt.close(fig)
print(OUTPUT / "fireresq_architecture.png")
