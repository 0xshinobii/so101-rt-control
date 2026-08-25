#!/usr/bin/env python3
"""Write RESULTS.md figures (stdlib SVG; no matplotlib)."""
from __future__ import annotations

import csv
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "figures"
DATA = ROOT / "docs" / "data"
TARGET_ELBOW = -0.8
T_REF = 1.0
PERIOD_US = 5000.0


def load_tq2(path: Path) -> tuple[list[float], list[float]]:
    t, q = [], []
    with path.open() as handle:
        rows = csv.DictReader(
            (line for line in handle if not line.startswith("#"))
        )
        for row in rows:
            t.append(float(row["t"]))
            q.append(float(row["q2"]))
    return t, q


def window(t, y, t0, t1):
    out_t, out_y = [], []
    for ti, yi in zip(t, y):
        if t0 <= ti <= t1:
            out_t.append(ti)
            out_y.append(yi)
    return out_t, out_y


def svg_polyline(xs, ys, x0, y0, w, h, xmin, xmax, ymin, ymax, stroke, width=1.5):
    pts = []
    for x, y in zip(xs, ys):
        px = x0 + (x - xmin) / (xmax - xmin) * w
        py = y0 + (1.0 - (y - ymin) / (ymax - ymin)) * h
        pts.append(f"{px:.2f},{py:.2f}")
    return (
        f'<polyline fill="none" stroke="{stroke}" stroke-width="{width}" '
        f'points="{" ".join(pts)}"/>'
    )


def write_elbow():
    t_pd, q_pd = load_tq2(DATA / "hw_pd_151.csv")
    t_ct, q_ct = load_tq2(DATA / "hw_ct_151.csv")
    e_pd = [TARGET_ELBOW - q for q in q_pd]
    e_ct = [TARGET_ELBOW - q for q in q_ct]
    t_pd, e_pd = window(t_pd, e_pd, 1.2, 4.0)
    t_ct, e_ct = window(t_ct, e_ct, 1.2, 4.0)
    xmin, xmax = 1.2, 4.0
    ymin, ymax = -0.060, 0.008
    x0, y0, w, h = 52.0, 36.0, 500.0, 168.0
    W, H = 580, 230
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        'font-family="Helvetica,Arial,sans-serif" font-size="11">',
        f'<rect width="{W}" height="{H}" fill="#fff"/>',
        f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="none" '
        'stroke="#222"/>',
        f'<text x="{W / 2:.0f}" y="16" text-anchor="middle" font-size="13">'
        "151 g hang: elbow error (target − q)</text>",
        f'<text x="{x0 + w / 2:.0f}" y="{H - 6}" text-anchor="middle">t [s]</text>',
        '<text x="14" y="130" transform="rotate(-90 14 130)">error [rad]</text>',
    ]
    zero_y = y0 + (1.0 - (0.0 - ymin) / (ymax - ymin)) * h
    parts.append(
        f'<line x1="{x0}" y1="{zero_y:.1f}" x2="{x0 + w}" y2="{zero_y:.1f}" '
        'stroke="#aaa" stroke-dasharray="4 3"/>'
    )
    for y in (-0.04, -0.02, 0.0):
        py = y0 + (1.0 - (y - ymin) / (ymax - ymin)) * h
        parts.append(
            f'<text x="{x0 - 6}" y="{py + 4:.1f}" text-anchor="end">'
            f"{y:.2f}</text>"
        )
    for x in (1.2, 2.0, 3.0, 4.0):
        px = x0 + (x - xmin) / (xmax - xmin) * w
        label = "1.2" if x == 1.2 else f"{x:.0f}"
        parts.append(
            f'<text x="{px:.1f}" y="{y0 + h + 14:.1f}" text-anchor="middle">'
            f"{label}</text>"
        )
    parts.append(svg_polyline(t_pd, e_pd, x0, y0, w, h, xmin, xmax, ymin, ymax, "#c43"))
    parts.append(svg_polyline(t_ct, e_ct, x0, y0, w, h, xmin, xmax, ymin, ymax, "#26a"))
    parts.extend(
        [
            '<line x1="70" y1="28" x2="98" y2="28" stroke="#c43" stroke-width="2"/>',
            '<text x="102" y="32">PD (−0.042)</text>',
            '<line x1="200" y1="28" x2="228" y2="28" stroke="#26a" stroke-width="2"/>',
            '<text x="232" y="32">CT empty (−0.018)</text>',
            "</svg>",
        ]
    )
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "hw_151_elbow.svg").write_text("\n".join(parts) + "\n")


def write_jitter():
    series = [
        ("idle", [2.20, 2.43, 2.88, 3.34, 6.89], "#26a"),
        ("loaded", [2.22, 3.38, 6.18, 11.34, 438.0], "#c43"),
    ]
    labels = ["Min", "Avg", "p99", "p99.9", "Max"]
    x0, y0, w, h = 52.0, 36.0, 500.0, 168.0
    W, H = 580, 230
    ymin, ymax = math.log10(1.5), math.log10(8000.0)

    def py(v):
        return y0 + (1.0 - (math.log10(v) - ymin) / (ymax - ymin)) * h

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        'font-family="Helvetica,Arial,sans-serif" font-size="11">',
        f'<rect width="{W}" height="{H}" fill="#fff"/>',
        f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="none" '
        'stroke="#222"/>',
        f'<text x="{W / 2:.0f}" y="16" text-anchor="middle" font-size="13">'
        "PREEMPT_RT wakeup vs 5 ms control period</text>",
        '<text x="14" y="130" transform="rotate(-90 14 130)">late [µs]</text>',
    ]
    for tick in (2, 10, 50, 200, 1000, 5000):
        y = py(tick)
        parts.append(
            f'<line x1="{x0}" y1="{y:.1f}" x2="{x0 + w}" y2="{y:.1f}" '
            'stroke="#eee"/>'
        )
        parts.append(
            f'<text x="{x0 - 6}" y="{y + 4:.1f}" text-anchor="end">{tick}</text>'
        )
    y_period = py(PERIOD_US)
    parts.append(
        f'<line x1="{x0}" y1="{y_period:.1f}" x2="{x0 + w}" y2="{y_period:.1f}" '
        'stroke="#333" stroke-dasharray="6 3"/>'
    )
    parts.append(
        f'<text x="{x0 + w - 4}" y="{y_period - 4:.1f}" text-anchor="end" '
        'font-size="10">control period 5000 µs</text>'
    )
    group_w = w / len(labels)
    bar_w = 18.0
    for i, lab in enumerate(labels):
        gx = x0 + (i + 0.5) * group_w
        parts.append(
            f'<text x="{gx:.1f}" y="{y0 + h + 14:.1f}" text-anchor="middle">'
            f"{lab}</text>"
        )
        for k, (_name, vals, color) in enumerate(series):
            v = vals[i]
            bx = gx - bar_w - 2 + k * (bar_w + 4)
            yb = py(v)
            parts.append(
                f'<rect x="{bx:.1f}" y="{yb:.1f}" width="{bar_w}" '
                f'height="{y0 + h - yb:.1f}" fill="{color}"/>'
            )
    parts.extend(
        [
            '<rect x="60" y="24" width="10" height="10" fill="#26a"/>',
            '<text x="74" y="33">idle n=20k</text>',
            '<rect x="160" y="24" width="10" height="10" fill="#c43"/>',
            '<text x="174" y="33">loaded n=60k</text>',
            "</svg>",
        ]
    )
    (OUT / "rt_jitter.svg").write_text("\n".join(parts) + "\n")


def main():
    write_elbow()
    write_jitter()
    print("wrote", OUT / "hw_151_elbow.svg")
    print("wrote", OUT / "rt_jitter.svg")


if __name__ == "__main__":
    main()
