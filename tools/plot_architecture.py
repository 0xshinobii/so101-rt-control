#!/usr/bin/env python3
"""Generate docs/figures/architecture.svg -- the stack diagram with the
hard-real-time boundary marked explicitly.

The diagram exists to answer one question a reviewer asks about any motion-
control repo: *which code is allowed to block, and which is not?* Everything
below the red dashed line runs on a SCHED_FIFO thread that must never
allocate, take a lock, or touch rclcpp. Everything above it may do all three.

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

W, H = 1160, 930

INK = "#0f172a"        # primary text
MUTED = "#64748b"      # secondary text
FAINT = "#94a3b8"      # tertiary text / leader lines
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

LEFT, RIGHT = 40, 800          # left column (the stack) spans LEFT..RIGHT
COLW = RIGHT - LEFT
BX, BW = 824, 296              # right column (the timing budget)

# ---------------------------------------------------------------------------
# Non-real-time zone
# ---------------------------------------------------------------------------

rect(LEFT, 86, COLW, 152, fill="#f1f5f9", stroke="#cbd5e1", rx=10)
zone_label(LEFT + 16, 108, "NON-REAL-TIME — rclcpp EXECUTOR THREAD", MUTED)
text(LEFT + 16, 125, "may allocate, block, log, take locks — none of it can stall the servo loop",
     size=10.5, fill=FAINT)

box(LEFT + 16, 136, 168, 86, "params.yaml",
    lines=("gains, target, rate",),
    mono_lines=("rt_priority: 80", "rate_hz: 200.0"))
box(LEFT + 200, 136, 176, 86, "rclcpp::spin()",
    lines=("executor thread; never", "enters the control path"))
box(LEFT + 392, 136, 184, 86, "wall timer 20 ms",
    lines=("drains the ring,", "keeps latest sample"),
    mono_lines=("drain_and_publish()",))
box(LEFT + 592, 136, 152, 86, "publishers",
    mono_lines=("/joint_states", "/arm_metrics"),
    lines=("best-effort telemetry",))

arrow(LEFT + 184, 179, LEFT + 198, 179)
arrow(LEFT + 376, 179, LEFT + 390, 179)
arrow(LEFT + 576, 179, LEFT + 590, 179)

# params are read once, at construction -- not on the hot path.
arrow(LEFT + 100, 222, LEFT + 100, 366, dash="4 4", marker="arrowFaint")
text(LEFT + 108, 252, "read once at", size=9.5, fill=FAINT)
text(LEFT + 108, 264, "construction", size=9.5, fill=FAINT)

# ---------------------------------------------------------------------------
# THE boundary
# ---------------------------------------------------------------------------

BY = 290  # boundary y
line(LEFT, BY, W - 40, BY, stroke=RT_RED, sw=3, dash="11 7")
zone_label(LEFT, BY - 10, "HARD REAL-TIME BOUNDARY", RT_RED)

# The queue is the only thing that straddles the line.
rect(LEFT + 330, BY - 30, 300, 60, fill="#ffffff", stroke=RT_RED, sw=2)
text(LEFT + 344, BY - 10, "SpscRing<Sample>", size=12.5, weight="600",
     family=MONO)
text(LEFT + 344, BY + 6, "lock-free, 1024 slots, wait-free push()", size=10,
     fill=MUTED)
text(LEFT + 344, BY + 20, "full → drop the sample, never block", size=10,
     fill=RT_RED)

arrow(LEFT + 420, BY - 32, LEFT + 420, 226, stroke=GREEN, marker="arrowGreen")
text(LEFT + 646, BY - 6, "telemetry is best-effort.", size=10, fill=MUTED)
text(LEFT + 646, BY + 8, "control is not.", size=10, weight="600", fill=RT_RED)

# ---------------------------------------------------------------------------
# Real-time zone
# ---------------------------------------------------------------------------

rect(LEFT, 322, COLW, 280, fill="#fff7ed", stroke="#fdba74", rx=10)
zone_label(LEFT + 16, 344, "REAL-TIME — DEDICATED std::thread", "#c2410c")
text(LEFT + 16, 361,
     "SCHED_FIFO 80  ·  mlockall(MCL_CURRENT|MCL_FUTURE)  ·  "
     "/dev/cpu_dma_latency = 0  ·  optional CPU pin",
     size=10.5, fill="#9a3412", family=MONO)

rect(LEFT + 16, 372, COLW - 32, 44, fill="#ffffff", stroke="#fdba74")
text(LEFT + 28, 391,
     "clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)  —  absolute deadline, "
     "5 ms period (200 Hz)", size=11, family=MONO, weight="600")
text(LEFT + 28, 407,
     "Relative sleeps pick up timer slack: an earlier bench using "
     "sleep_until measured 477 µs idle max and was discarded.",
     size=10, fill=MUTED)

box(LEFT + 16, 430, 176, 62, "min-jerk reference",
    mono_lines=("q_des, q̇_des, q̈_des",),
    lines=("1.0 s, computed in-loop",))
box(LEFT + 208, 430, 208, 62, "ControlLoop::step_once()",
    lines=("pre-allocated Eigen buffers", "no alloc · no I/O · no rclcpp"))
box(LEFT + 432, 430, 216, 62, "Controller  (interface)",
    lines=("PD  |  computed torque  |", "adaptive CT + payload RLS"))
box(LEFT + 664, 430, 120, 62, "torque out",
    mono_lines=("τ ∈ ℝ⁶",),
    lines=("±2.94 N·m limit",))

arrow(LEFT + 192, 461, LEFT + 206, 461)
arrow(LEFT + 416, 461, LEFT + 430, 461)
arrow(LEFT + 648, 461, LEFT + 662, 461)

box(LEFT + 208, 504, 208, 78, "PinocchioDynamics",
    mono_lines=("rnea() · crba()",),
    lines=("the controller's own model,", "never the plant's"))
box(LEFT + 432, 504, 216, 78, "PayloadMassRls",
    lines=("scalar mass, projected to [0, 0.5] kg",
           "before it reaches the control law —",
           "the clamp that caught −0.175 kg on hardware"))

arrow(LEFT + 312, 494, LEFT + 312, 502, marker="arrowFaint")
arrow(LEFT + 540, 494, LEFT + 540, 502, marker="arrowFaint")

# telemetry up into the ring
arrow(LEFT + 420, 430, LEFT + 420, BY + 32, stroke=GREEN, marker="arrowGreen")

# ---------------------------------------------------------------------------
# Plant boundary
# ---------------------------------------------------------------------------

PY = 634
line(LEFT, PY, BX - 24, PY, stroke=PLANT_BLUE, sw=2, dash="8 6")
zone_label(LEFT, PY - 24, "PLANT BOUNDARY — ONE INTERFACE, TWO BACKENDS", PLANT_BLUE)
text(LEFT, PY - 8, "PlantInterface: torque in, state out. "
     "Controllers never learn which side they are on.",
     size=10, fill=MUTED)

arrow(LEFT + 724, 492, LEFT + 724, PY - 4, stroke=PLANT_BLUE, marker="arrowBlue")
text(LEFT + 730, 546, "τ", size=13, fill=PLANT_BLUE, weight="700", family=MONO)

# ---------------------------------------------------------------------------
# Backends
# ---------------------------------------------------------------------------

rect(LEFT, 654, COLW, 216, fill="#eff6ff", stroke="#bfdbfe", rx=10)

box(LEFT + 16, 672, 356, 180, "MujocoBackend  —  simulation",
    lines=("τ written straight to d->ctrl; mj_step advances one period.",
           "Plant is so101_torque.xml (Menagerie SO-101), with",
           "vendor damping / frictionloss / armature left in place.",
           "",
           "Model mismatch is deliberate: Pinocchio carries no",
           "friction, so the estimator has something real to find.",
           "",
           "Equivalence against Pinocchio is a blocking gate before",
           "any controller runs: gravity, nonlinear bias and the mass",
           "matrix agree to ~1e-13 N·m over five poses."),
    stroke="#bfdbfe")

box(LEFT + 392, 672, 376, 180, "HardwareBackend  —  SO-ARM101",
    lines=("The STS3215 has no closed-loop N·m mode, so torque is",
           "realized admittance-style (UR-like), not commanded:"),
    stroke="#bfdbfe")
rect(LEFT + 404, 732, 352, 26, fill="#ffffff", stroke="#93c5fd")
text(LEFT + 414, 749, "q_cmd = q_des + clamp(τ / K_servo, ±0.12 rad)",
     size=11, family=MONO, weight="600", fill=PLANT_BLUE)
text(LEFT + 404, 778, "FeetechBus · sync_read / sync_write · USB-serial 1 Mbaud",
     size=10.5, fill=MUTED)
text(LEFT + 404, 794, "6× STS3215, Mode 0, servo's own inner PD closes the loop",
     size=10.5, fill=MUTED)
text(LEFT + 404, 816, "Only elbow K_servo is identified (≈ 11 N·m/rad).",
     size=10.5, fill="#b45309")
text(LEFT + 404, 831, "The other five are frozen placeholders — stated, not hidden.",
     size=10.5, fill="#b45309")

# ---------------------------------------------------------------------------
# Right column: the 5 ms budget
# ---------------------------------------------------------------------------

rect(BX, 322, BW, 548, fill="#ffffff", stroke="#e2e8f0", rx=10)
text(BX + 18, 350, "Where the 5 ms actually goes", size=13.5, weight="700")
text(BX + 18, 368, "i7-7700 · Ubuntu 26.04 · kernel 7.0.0-30-realtime",
     size=10, fill=FAINT)

# One bar = one 5 ms control period, to scale. 1 px = 19.2 us.
bar_x, bar_y, bar_w, bar_h = BX + 18, 384, 260, 28
px_per_us = bar_w / 5000.0

rect(bar_x, bar_y, bar_w, bar_h, fill="#e2e8f0", stroke="#cbd5e1", rx=3)
rect(bar_x, bar_y, 1900 * px_per_us, bar_h, fill=PLANT_BLUE, stroke=PLANT_BLUE,
     rx=3)
# At this scale the OS wakeup is 0.6 px wide. That invisibility is the finding,
# so it is floored to 1.5 px rather than dropped.
rect(bar_x, bar_y, max(11.3 * px_per_us, 1.5), bar_h, fill=RT_RED,
     stroke=RT_RED, rx=0)

text(bar_x, bar_y + bar_h + 14, "0", size=9.5, fill=FAINT)
text(bar_x + bar_w, bar_y + bar_h + 14, "5 ms  (one control period)", size=9.5,
     fill=FAINT, anchor="end")

# Legend below the bar: no leader lines, nothing to collide with.
legend = [
    (RT_RED, "OS wakeup jitter", "11 µs", "0.2%"),
    (PLANT_BLUE, "serial bus I/O", "1.90 ms", "38%"),
    ("#cbd5e1", "headroom", "3.09 ms", "62%"),
]
ly = bar_y + bar_h + 34
for color, label, value, pct in legend:
    rect(bar_x, ly - 8, 10, 10, fill=color, stroke=color, rx=2)
    text(bar_x + 18, ly, label, size=10.5, fill=MUTED)
    text(bar_x + bar_w - 42, ly, value, size=10.5, fill=INK, weight="600",
         family=MONO, anchor="end")
    text(bar_x + bar_w, ly, pct, size=10.5, fill=FAINT, family=MONO,
         anchor="end")
    ly += 19

rows = [
    ("cyclictest -p 80, loaded max", "27 µs"),
    ("rt_jitter_bench idle max", "6.89 µs"),
    ("rt_jitter_bench loaded p99", "6.18 µs"),
    ("rt_jitter_bench loaded p99.9", "11.34 µs"),
    ("sync_read p99.9", "1.71 ms"),
    ("sync_write p99.9", "0.19 ms"),
    ("packet loss, 2000 loops", "0"),
]
ry = 508
text(BX + 18, ry, "MEASURED", size=9.5, weight="700", fill=FAINT, spacing="1.2")
ry += 8
for label, value in rows:
    ry += 20
    text(BX + 18, ry, label, size=10.5, fill=MUTED)
    text(BX + BW - 18, ry, value, size=10.5, fill=INK, weight="600",
         family=MONO, anchor="end")
    line(BX + 18, ry + 6, BX + BW - 18, ry + 6, stroke="#f1f5f9", sw=1)

# The conclusion the bar is there to support.
rect(BX + 18, 678, BW - 36, 122, fill="#fef2f2", stroke="#fecaca", rx=8)
text(BX + 32, 702, "The CPU was never the bottleneck.", size=12,
     weight="700", fill=RT_RED)
text(BX + 32, 726, "PREEMPT_RT buys a 5–11 µs wakeup on a", size=10.5,
     fill="#7f1d1d")
text(BX + 32, 741, "5000 µs period. The servo bus spends", size=10.5,
     fill="#7f1d1d")
text(BX + 32, 756, "170× more of that budget than the", size=10.5,
     fill="#7f1d1d")
text(BX + 32, 771, "scheduler does.", size=10.5, fill="#7f1d1d")
text(BX + 32, 790, "Knowing that is the point of measuring.", size=10,
     fill="#991b1b", weight="600")

text(BX + 18, 822, "Loaded max hits 438 µs on 0.07% of ticks and", size=9.5,
     fill=FAINT)
text(BX + 18, 835, "cyclictest did not reproduce it — an application-", size=9.5,
     fill=FAINT)
text(BX + 18, 848, "side cause is not excluded. See RESULTS.md.", size=9.5,
     fill=FAINT)

text(40, H - 18, "Source: RESULTS.md · regenerate with  "
     "python3 tools/plot_architecture.py", size=9.5, fill=FAINT, family=MONO)

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
        import cairosvg  # optional; SVG is the committed source of truth
    except ImportError:
        print("cairosvg not installed -- skipping PNG export")
        return
    cairosvg.svg2png(bytestring=svg.encode("utf-8"), write_to=png, scale=2.0)
    print(f"wrote {png}")


if __name__ == "__main__":
    main()
