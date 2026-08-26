#!/usr/bin/env python3
"""Elbow tracking error for the final campaign, 90 g and 180 g side by side.

One panel per payload; one trace per controller. The elbow is the joint the
gravity-compensation result rests on -- it is the only joint with an identified
K_servo -- so this is the figure that shows the closed-loop effect of the
affine correction directly rather than through an aggregate.

Usage:  python3 tools/plot_final_campaign.py [--data docs/data/final_campaign]
                                             [--out docs/figures]
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# kTarget[2]; signed error is target - q, matching RESULTS.md throughout.
ELBOW_TARGET = -0.8
ELBOW_COL = "q2"
SETTLED_T = 1.5

# Ordered worst-to-best so the legend reads as the progression it describes.
SERIES = [
    ("01_pd",                 "PD",                    "#94a3b8", 1.4, "-"),
    ("02_ct_empty",           "CT, empty model",       "#0369a1", 1.6, "-"),
    ("03_adapt_raw_static",   "static ID, raw",        "#f59e0b", 1.6, "--"),
    ("04_adapt_affine_static", "static ID, affine",    "#15803d", 2.2, "-"),
]


def load(path: Path) -> tuple[list[float], list[float]]:
    """Return (t, signed elbow error) skipping the leading '#' metadata line."""
    with path.open() as fh:
        rows = [ln for ln in fh if not ln.startswith("#")]
    t, e = [], []
    for r in csv.DictReader(rows):
        t.append(float(r["t"]))
        e.append(ELBOW_TARGET - float(r[ELBOW_COL]))
    return t, e


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", default="docs/data/final_campaign")
    ap.add_argument("--out", default="docs/figures")
    args = ap.parse_args()
    data, out = Path(args.data), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.0), sharey=True)
    for ax, mass in zip(axes, ("090", "180")):
        for suffix, label, color, lw, ls in SERIES:
            t, e = load(data / f"final_hw_{mass}g_{suffix}.csv")
            ax.plot(t, e, color=color, linewidth=lw, linestyle=ls, label=label)
        ax.axhline(0, color="#0f172a", linewidth=0.8)
        # Everything left of this line is transit, not the droop being measured.
        ax.axvline(SETTLED_T, color="#cbd5e1", linewidth=1, linestyle=":")
        ax.text(SETTLED_T + 0.05, 0.083, "settled window", fontsize=8,
                color="#64748b")
        ax.set_title(f"{int(mass)} g payload", fontsize=12, fontweight="bold")
        ax.set_xlabel("time [s]")
        ax.set_xlim(0, 4.0)
        ax.grid(True, alpha=0.25, linewidth=0.6)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    axes[0].set_ylabel("elbow error, target − q  [rad]")
    axes[0].set_ylim(-0.09, 0.09)
    axes[0].legend(loc="upper right", fontsize=9, framealpha=0.95)
    fig.suptitle("Elbow tracking error — the affine correction in closed loop",
                 fontsize=13, fontweight="bold", y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    for ext in ("svg", "png"):
        p = out / f"final_campaign_elbow.{ext}"
        fig.savefig(p, dpi=200, facecolor="white")
        print("wrote", p)


if __name__ == "__main__":
    main()
