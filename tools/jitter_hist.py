"""Text histogram of rt_jitter_bench CSV (late_us column)."""
import argparse
import sys


def load_late_us(path):
    values = []
    with open(path) as handle:
        for line in handle:
            if line.startswith("#") or line.startswith("i,"):
                continue
            parts = line.strip().split(",")
            if len(parts) < 2:
                continue
            values.append(float(parts[1]))
    return values


def percentile(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    idx = (p / 100.0) * (len(sorted_vals) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = idx - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("--bins", type=int, default=40)
    args = parser.parse_args()

    vals = load_late_us(args.csv)
    if not vals:
        print("no samples", file=sys.stderr)
        return 1
    ordered = sorted(vals)
    print(
        "n=%d Min=%.3f Avg=%.3f p99=%.3f p99.9=%.3f Max=%.3f  (us)"
        % (
            len(vals),
            ordered[0],
            sum(vals) / len(vals),
            percentile(ordered, 99.0),
            percentile(ordered, 99.9),
            ordered[-1],
        )
    )

    lo, hi = ordered[0], ordered[-1]
    if hi <= lo:
        hi = lo + 1e-9
    width = (hi - lo) / args.bins
    counts = [0] * args.bins
    for v in vals:
        b = int((v - lo) / width)
        if b >= args.bins:
            b = args.bins - 1
        counts[b] += 1
    peak = max(counts) or 1
    for i, c in enumerate(counts):
        left = lo + i * width
        bar = "#" * int(40 * c / peak)
        print("%8.3f us | %6d %s" % (left, c, bar))
    return 0


if __name__ == "__main__":
    sys.exit(main())
