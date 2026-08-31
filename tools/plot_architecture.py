#!/usr/bin/env python3
"""Generate docs/figures/architecture.svg -- the stack diagram with the
hard-real-time boundary marked explicitly.

The diagram makes one distinction explicit: which code is allowed to touch
rclcpp and publishing infrastructure. Everything below the red dashed line runs
on a SCHED_FIFO thread and avoids rclcpp; the hardware backend necessarily
performs blocking serial I/O inside that thread. Everything above it may
allocate, log and take locks.

Numbers annotated on the figure are measured; see RESULTS.md for provenance.

Usage:  python3 tools/plot_architecture.py [--out docs/figures/architecture.svg]

No third-party dependencies are required for the SVG. PNG export additionally
needs `cairosvg` and is skipped with a warning if it is not installed.
"""

from __future__ import annotations

import argparse
import html
import os

# ---------------------------------------------------------------------------
# Canvas + palette
# ---------------------------------------------------------------------------

W, H = 1160, 870

INK = "#0f172a"        # primary text
MUTED = "#64748b"      # secondary text
FAINT = "#5b6b7f"      # tertiary text / leader lines
RT_RED = "#dc2626"     # the real-time boundary
PLANT_BLUE = "#0369a1" # the plant boundary and bus timing
GREEN = "#15803d"      # telemetry (best-effort) path

BOX_FILL = "#ffffff"
BOX_STROKE = "#cbd5e1"

SANS = "ui-sans-serif, -apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif"
MONO = "ui-monospace, SFMono-Regular, 'SF Mono', Menlo, Consolas, 'Liberation Mono', monospace"

out: list[str] = []


# ---------------------------------------------------------------------------
# Primitives
# ---------------------------------------------------------------------------

def esc(s: str) -> str:
    return html.escape(s, quote=False)


def rect(x, y, w, h, fill=BOX_FILL, stroke=BOX_STROKE, sw=1, rx=6, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{d}/>'
    )


def text(x, y, s, size=12, fill=INK, weight="400", family=SANS,
         anchor="start", spacing=None):
    ls = f' letter-spacing="{spacing}"' if spacing else ""
    out.append(
        f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" '
        f'fill="{fill}" font-weight="{weight}" text-anchor="{anchor}"{ls}>'
        f'{esc(s)}</text>'
    )


def line(x1, y1, x2, y2, stroke=FAINT, sw=1, dash=None, marker=False):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    m = ' marker-end="url(#arrow)"' if marker else ""
    out.append(
        f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{stroke}" '
        f'stroke-width="{sw}"{d}{m}/>'
    )


def arrow(x1, y1, x2, y2, stroke=MUTED, sw=1.4, marker="arrow", dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(
        f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{stroke}" '
        f'stroke-width="{sw}" marker-end="url(#{marker})"{d}/>'
    )


def box(x, y, w, h, title, lines=(), mono_lines=(), stroke=BOX_STROKE,
        fill=BOX_FILL, title_size=12.5, title_color=INK):
    """A labelled box: bold title, then optional sans and monospace lines."""
    rect(x, y, w, h, fill=fill, stroke=stroke)
    cy = y + 20
    text(x + 12, cy, title, size=title_size, weight="600", fill=title_color)
    for ln in lines:
        cy += 15
        text(x + 12, cy, ln, size=10.5, fill=MUTED)
    for ln in mono_lines:
        cy += 15
        text(x + 12, cy, ln, size=10, fill=MUTED, family=MONO)


def zone_label(x, y, s, color):
    text(x, y, s, size=10.5, weight="700", fill=color, spacing="1.4")


# ---------------------------------------------------------------------------
# Document head
# ---------------------------------------------------------------------------

out.append(
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
    f'viewBox="0 0 {W} {H}" role="img" '
    f'aria-label="SO-101 control stack with the hard real-time boundary marked">'
)
out.append("<defs>")
for name, color in (("arrow", MUTED), ("arrowGreen", GREEN),
                    ("arrowBlue", PLANT_BLUE), ("arrowFaint", FAINT)):
    out.append(
        f'<marker id="{name}" viewBox="0 0 10 10" refX="9" refY="5" '
        f'markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
        f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{color}"/></marker>'
    )
out.append("</defs>")

# White plate: keeps the figure readable on both light and dark GitHub themes.
rect(0, 0, W, H, fill="#ffffff", stroke="#ffffff", rx=0)

text(40, 42, "SO-101 payload-adaptive control — where the real-time boundary lives",
     size=19, weight="700")
text(40, 63, "ROS 2 Jazzy · C++17 · PREEMPT_RT · Pinocchio · MuJoCo / SO-ARM101   "
             "—   200 Hz, 5 ms period, one code path for sim and hardware",
     size=11.5, fill=MUTED)
# The telemetry half exists only in the ROS node; say so on the figure rather
# than letting the reader assume the hardware runner has it too.
text(40, 80, "This is the ROS 2 node. The hardware runner shares everything below "
             "the red line and logs to CSV instead of publishing.",
     size=10.5, fill=FAINT)

# Column geometry. Every box below is placed so that its RIGHT edge stays
# inside the panel that contains it -- see the assertions at the end.
LEFT, RIGHT = 40, 800          # left column (the stack)
COLW = RIGHT - LEFT
BX, BW = 824, 296              # right column (the timing budget)

# ---------------------------------------------------------------------------
# Non-real-time zone
# ---------------------------------------------------------------------------

rect(LEFT, 96, COLW, 152, fill="#f1f5f9", stroke="#cbd5e1", rx=10)
zone_label(LEFT + 16, 118, "NON-REAL-TIME — rclcpp EXECUTOR THREAD", MUTED)
text(LEFT + 16, 135, "may allocate, block, log, take locks — none of it can stall "
     "the servo loop", size=10.5, fill=FAINT)

box(56, 146, 168, 86, "params.yaml",
    lines=("gains, target, rate",),
    mono_lines=("rt_priority: 80", "rate_hz: 200.0"))
box(236, 146, 176, 86, "rclcpp::spin()",
    lines=("executor thread; never", "enters the control path"))
box(424, 146, 184, 86, "wall timer 20 ms",
    lines=("drains the ring,", "keeps latest sample"),
    mono_lines=("drain_and_publish()",))
box(620, 146, 164, 86, "publishers",
    mono_lines=("/joint_states", "/arm_metrics"),
    lines=("best-effort telemetry",))

arrow(224, 189, 234, 189)
arrow(412, 189, 422, 189)
arrow(608, 189, 618, 189)

# Parameters are read once, at construction. The arrow stops above the RT
# panel so it never crosses the zone labels inside it.
arrow(140, 232, 140, 316, dash="4 4", marker="arrowFaint")
text(148, 252, "read once at", size=10, fill=FAINT)
text(148, 264, "construction", size=10, fill=FAINT)

# ---------------------------------------------------------------------------
# THE boundary
# ---------------------------------------------------------------------------

BY = 290
line(LEFT, BY, W - 40, BY, stroke=RT_RED, sw=3, dash="11 7")
zone_label(LEFT, BY - 10, "HARD REAL-TIME BOUNDARY", RT_RED)

# The queue is the only thing that straddles the line. It is wide enough that
# both the producer arrow (x=722) and the consumer arrow (x=560) land inside it.
rect(500, BY - 30, 300, 60, fill="#ffffff", stroke=RT_RED, sw=2)
text(516, BY - 8, "SpscRing<Sample>", size=12.5, weight="600", family=MONO)
text(516, BY + 8, "lock-free, 1024 slots, wait-free push()", size=10, fill=MUTED)
text(516, BY + 22, "full → drop the sample, never block", size=10, fill=RT_RED)

# Consumer side: the 20 ms wall timer drains it.
arrow(560, BY - 32, 560, 236, stroke=GREEN, marker="arrowGreen")

# Sits clear of the dashed line rather than being struck through by it.
text(BX, BY - 20, "telemetry is best-effort.", size=10, fill=MUTED)
text(BX, BY - 6, "control is not.", size=10, weight="600", fill=RT_RED)

# ---------------------------------------------------------------------------
# Real-time zone
# ---------------------------------------------------------------------------

rect(LEFT, 322, COLW, 272, fill="#fff7ed", stroke="#fdba74", rx=10)
zone_label(LEFT + 16, 344, "REAL-TIME — DEDICATED std::thread", "#c2410c")
text(LEFT + 16, 361,
     "SCHED_FIFO 80  ·  mlockall(MCL_CURRENT|MCL_FUTURE)  ·  "
     "/dev/cpu_dma_latency = 0  ·  optional CPU pin",
     size=10.5, fill="#9a3412", family=MONO)

# Stops at x=700 so the producer arrow has a clear channel to its right.
rect(56, 372, 644, 44, fill="#ffffff", stroke="#fdba74")
text(68, 391, "clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)  —  absolute "
     "deadline, 5 ms period", size=11, family=MONO, weight="600")
text(68, 407, "A late wakeup does not shift the next one: the deadline advances "
     "by a fixed period, never from \u201cnow\u201d.", size=10, fill=MUTED)

box(56, 430, 168, 60, "min-jerk reference",
    mono_lines=("q_des, q̇_des, q̈_des",),
    lines=("1.0 s, computed in-loop",))
box(236, 430, 196, 60, "ControlLoop::step_once()",
    lines=("pre-sized state/reference,", "shared path · no rclcpp"))
box(444, 430, 204, 60, "Controller  (interface)",
    lines=("PD  |  computed torque  |", "adaptive CT + payload RLS"))
box(660, 430, 124, 60, "torque out",
    mono_lines=("τ ∈ ℝ⁶",),
    lines=("±2.94 N·m limit",))

arrow(224, 460, 234, 460)
arrow(432, 460, 442, 460)
arrow(648, 460, 658, 460)

box(236, 506, 196, 76, "PinocchioDynamics",
    mono_lines=("rnea() · crba()",),
    lines=("the controller's own model,", "never the plant's"))
box(444, 506, 204, 76, "PayloadMassRls",
    lines=("raw reading → affine correction",
           "→ [0, 0.5] kg projection"))

arrow(334, 490, 334, 504, marker="arrowFaint")
arrow(546, 490, 546, 504, marker="arrowFaint")

# Producer side: the completed iteration goes up to the ring. x=722 clears the
# clock box (ends at 700) so the arrow crosses no text.
arrow(722, 430, 722, 322, stroke=GREEN, marker="arrowGreen")
text(730, 352, "Sample", size=10, fill=GREEN, weight="600")

# ---------------------------------------------------------------------------
# Plant boundary
# ---------------------------------------------------------------------------

PY = 634
line(LEFT, PY, BX - 24, PY, stroke=PLANT_BLUE, sw=2, dash="8 6")
zone_label(LEFT, PY - 24, "PLANT BOUNDARY — ONE INTERFACE, TWO BACKENDS", PLANT_BLUE)
text(LEFT, PY - 10, "PlantInterface: torque in, state out. Controllers never "
     "learn which side they are on.", size=10, fill=MUTED)

arrow(722, 492, 722, PY - 4, stroke=PLANT_BLUE, marker="arrowBlue")
text(730, 552, "τ", size=13, fill=PLANT_BLUE, weight="700", family=MONO)

# ---------------------------------------------------------------------------
# Backends
# ---------------------------------------------------------------------------

rect(LEFT, 654, COLW, 172, fill="#eff6ff", stroke="#bfdbfe", rx=10)

box(56, 670, 340, 140, "MujocoBackend  —  simulation",
    lines=("τ written straight to d->ctrl; mj_step advances one",
           "period. Plant is so101_torque.xml (Menagerie SO-101),",
           "with vendor damping / frictionloss / armature intact.",
           "",
           "A blocking equivalence gate runs before any controller:",
           "gravity, nonlinear bias and the mass matrix must agree",
           "with Pinocchio to 1e-15 / 1e-13 N·m over five poses."),
    stroke="#bfdbfe")

box(412, 670, 372, 140, "HardwareBackend  —  SO-ARM101",
    lines=("The STS3215 has no closed-loop N·m mode, so torque is",
           "realized admittance-style (UR-like), not commanded:"),
    stroke="#bfdbfe")
rect(424, 730, 348, 26, fill="#ffffff", stroke="#93c5fd")
text(434, 747, "q_cmd = q_des + clamp(τ / K_servo, ±0.12 rad)",
     size=11, family=MONO, weight="600", fill=PLANT_BLUE)
text(424, 776, "FeetechBus · sync_read / sync_write · USB-serial 1 Mbaud",
     size=10.5, fill=MUTED)
text(424, 791, "6× STS3215, Mode 0, servo's own inner PD closes the loop",
     size=10.5, fill=MUTED)

# ---------------------------------------------------------------------------
# Right column: the 5 ms budget
# ---------------------------------------------------------------------------

rect(BX, 322, BW, 504, fill="#ffffff", stroke="#e2e8f0", rx=10)
text(BX + 18, 350, "Where the 5 ms actually goes", size=13.5, weight="700")
text(BX + 18, 368, "i7-7700 · Ubuntu 26.04 · kernel 7.0.0-30-realtime",
     size=10, fill=FAINT)

# One bar = one 5 ms control period, to scale.
bar_x, bar_y, bar_w, bar_h = BX + 18, 386, 260, 28
px_per_us = bar_w / 5000.0

rect(bar_x, bar_y, bar_w, bar_h, fill="#e2e8f0", stroke="#cbd5e1", rx=3)
rect(bar_x, bar_y, 2140 * px_per_us, bar_h, fill=PLANT_BLUE, stroke=PLANT_BLUE,
     rx=3)
# At this scale the OS wakeup is 0.6 px wide. That invisibility is the finding,
# so it is floored to 2 px rather than dropped.
rect(bar_x, bar_y, max(11.3 * px_per_us, 2.0), bar_h, fill=RT_RED,
     stroke=RT_RED, rx=0)

text(bar_x, bar_y + bar_h + 14, "0", size=10, fill=FAINT)
text(bar_x + bar_w, bar_y + bar_h + 14, "5 ms  (one control period)", size=10,
     fill=FAINT, anchor="end")

legend = [
    (RT_RED, RT_RED, "OS wakeup jitter", "11 µs", "0.23%"),
    (PLANT_BLUE, PLANT_BLUE, "serial bus I/O", "2.14 ms", "43%"),
    ("#e2e8f0", "#94a3b8", "headroom", "2.86 ms", "57%"),
]
ly = bar_y + bar_h + 34
for fill_c, stroke_c, label, value, pct in legend:
    rect(bar_x, ly - 8, 10, 10, fill=fill_c, stroke=stroke_c, rx=2)
    text(bar_x + 18, ly, label, size=10.5, fill=MUTED)
    text(bar_x + bar_w - 44, ly, value, size=10.5, fill=INK, weight="600",
         family=MONO, anchor="end")
    text(bar_x + bar_w, ly, pct, size=10.5, fill=MUTED, family=MONO,
         anchor="end")
    ly += 19

rows = [
    ("cyclictest -p 80, loaded max", "27 µs"),
    ("rt_jitter_bench idle max", "6.89 µs"),
    ("rt_jitter_bench loaded p99", "6.18 µs"),
    ("rt_jitter_bench loaded p99.9", "11.34 µs"),
    ("sync_read p99.9", "1.95 ms"),
    ("sync_write p99.9", "0.19 ms"),
    ("packet loss, 2000 loops", "0"),
]
ry = 516
text(BX + 18, ry, "MEASURED", size=10, weight="700", fill=FAINT, spacing="1.2")
ry += 8
for label, value in rows:
    ry += 20
    text(BX + 18, ry, label, size=10.5, fill=MUTED)
    text(BX + BW - 18, ry, value, size=10.5, fill=INK, weight="600",
         family=MONO, anchor="end")
    line(BX + 18, ry + 6, BX + BW - 18, ry + 6, stroke="#eef2f6", sw=1)

rect(BX + 18, 700, BW - 36, 98, fill="#fef2f2", stroke="#fecaca", rx=8)
text(BX + 32, 724, "The CPU was never the bottleneck.", size=12,
     weight="700", fill=RT_RED)
text(BX + 32, 748, "Loaded p99.9 wakeup is 11.34 µs on a", size=10.5,
     fill="#7f1d1d")
text(BX + 32, 763, "5000 µs period. The servo bus is ~189×", size=10.5,
     fill="#7f1d1d")
text(BX + 32, 778, "longer than the scheduler latency.", size=10.5,
     fill="#7f1d1d")


text(40, H - 18, "Source: RESULTS.md · regenerate with  "
     "python3 tools/plot_architecture.py", size=10, fill=FAINT, family=MONO)

# ---------------------------------------------------------------------------
# Geometry self-check. Boxes silently escaping their panel is the easiest way
# for this figure to regress, so assert containment instead of eyeballing it.
# ---------------------------------------------------------------------------

PANELS = {
    "non-RT":   (LEFT, 96, RIGHT, 248),
    "RT":       (LEFT, 322, RIGHT, 594),
    "backends": (LEFT, 654, RIGHT, 826),
    "budget":   (BX, 322, BX + BW, 826),
}
CHILDREN = [
    ("non-RT", 56, 146, 224, 232), ("non-RT", 236, 146, 412, 232),
    ("non-RT", 424, 146, 608, 232), ("non-RT", 620, 146, 784, 232),
    ("RT", 56, 372, 700, 416), ("RT", 56, 430, 224, 490),
    ("RT", 236, 430, 432, 490), ("RT", 444, 430, 648, 490),
    ("RT", 660, 430, 784, 490), ("RT", 236, 506, 432, 582),
    ("RT", 444, 506, 648, 582),
    ("backends", 56, 670, 396, 810), ("backends", 412, 670, 784, 810),
    ("budget", bar_x, bar_y, bar_x + bar_w, bar_y + bar_h),
    ("budget", BX + 18, 700, BX + BW - 18, 798),
]
for panel, x0, y0, x1, y1 in CHILDREN:
    px0, py0, px1, py1 = PANELS[panel]
    assert px0 <= x0 and x1 <= px1, f"{panel}: box x {x0}..{x1} escapes {px0}..{px1}"
    assert py0 <= y0 and y1 <= py1, f"{panel}: box y {y0}..{y1} escapes {py0}..{py1}"
assert PANELS["backends"][3] <= H - 30 and PANELS["budget"][3] <= H - 30

out.append("</svg>")


# ---------------------------------------------------------------------------
# Emit
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="docs/figures/architecture.svg")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    svg = "\n".join(out)
    with open(args.out, "w", encoding="utf-8") as fh:
        fh.write(svg + "\n")
    print(f"wrote {args.out}")

    png = os.path.splitext(args.out)[0] + ".png"
    try:
        import cairosvg  # optional; PNG if cairo is available
    except (ImportError, OSError):
        print("cairosvg/cairo not available -- skipping PNG export")
        return
    cairosvg.svg2png(bytestring=svg.encode("utf-8"), write_to=png, scale=2.0)
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
