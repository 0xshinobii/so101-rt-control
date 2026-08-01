"""arm_bench -- validate the C++ port against the Phase 1.5 Python oracle.

Two checks, matching the Phase 2 acceptance criteria:

  Primary (trajectory diff): the C++ and oracle CSVs must agree sample-by-sample
  to a tight tolerance (~1e-6). MuJoCo is deterministic, so anything larger is a
  real bug (wrong keyframe/gain/sign/timestep/log-convention), not FP noise. Two
  scalar metrics can be reproduced by two offsetting bugs; a per-sample diff
  cannot -- this is the real proof of a faithful port.

  Secondary (scalars): steady-state arm-mean RMS (~0.0100 rad) and EE miss
  (~0.0140 m), reusing src.metrics.rms_error (the same "error" definition as
  every other phase).

Both CSVs share the column layout: t, q0..q5, qd0..qd5, tau0..tau5, ee_x/y/z.
The oracle CSV carries `# ee_target=x,y,z` in a header comment.

Usage:
    python tools/arm_bench.py --cpp cpp_baseline_so101.csv \
                              --oracle oracle_baseline_so101.csv
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.metrics import rms_error  # noqa: E402

# Mirror the Phase 1.5 oracle.
TARGET = np.array([0.6, 0.7, -0.8, 0.5, 0.4, 0.0])
N_ARM = 5
SETTLE_T = 1.0
REF_SS_RMS = 0.0100   # steady-state arm-mean RMS [rad]
REF_EE_MISS = 0.0140  # EE miss [m]


def load_csv(path):
    """Return (data[T,22], ee_target|None). Skips `#` comment lines."""
    ee_target = None
    data_rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if "ee_target=" in line:
                    vals = line.split("ee_target=")[1].split(",")
                    ee_target = np.array([float(v) for v in vals])
                continue
            if line[0].isalpha():  # header row (t,q0,...)
                continue
            data_rows.append([float(v) for v in line.split(",")])
    return np.array(data_rows), ee_target


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cpp", default="cpp_baseline_so101.csv")
    ap.add_argument("--oracle", default="oracle_baseline_so101.csv")
    ap.add_argument("--traj-tol", type=float, default=1e-6,
                    help="max per-sample abs diff allowed (primary check)")
    ap.add_argument("--rms-tol", type=float, default=1e-3,
                    help="allowed |steady-state RMS - reference| [rad]")
    ap.add_argument("--ee-tol", type=float, default=1e-4,
                    help="allowed |EE miss - reference| [m]")
    args = ap.parse_args()

    cpp, _ = load_csv(args.cpp)
    oracle, ee_target = load_csv(args.oracle)

    ok = True

    # --- Primary: sample-by-sample trajectory diff ---------------------------
    print("== Primary: trajectory diff (C++ vs oracle) ==")
    if cpp.shape != oracle.shape:
        print(f"  FAIL shape mismatch: cpp {cpp.shape} vs oracle {oracle.shape}")
        ok = False
    else:
        diff = np.abs(cpp - oracle)
        # group column ranges: t | q(6) | qd(6) | tau(6) | ee(3)
        groups = [("t", 0, 1), ("q", 1, 7), ("qdot", 7, 13),
                  ("tau", 13, 19), ("ee", 19, 22)]
        for name, a, b in groups:
            g = diff[:, a:b]
            print(f"  {name:5s} max|d|={g.max():.3e}  mean|d|={g.mean():.3e}")
        max_d = diff.max()
        verdict = "PASS" if max_d < args.traj_tol else "FAIL"
        print(f"  overall max|d|={max_d:.3e}  (tol {args.traj_tol:.0e}) -> {verdict}")
        if max_d >= args.traj_tol:
            ok = False

    # --- Secondary: scalar metrics from the C++ trajectory -------------------
    print("\n== Secondary: scalar metrics (C++) ==")
    t = cpp[:, 0]
    q = cpp[:, 1:7]
    settled = t > SETTLE_T
    ss_rms = rms_error(q[settled], TARGET)[:N_ARM]
    ss_mean = float(ss_rms.mean())
    print(f"  steady-state arm-mean RMS = {ss_mean:.4f} rad "
          f"(ref {REF_SS_RMS:.4f}, delta {abs(ss_mean - REF_SS_RMS):.2e})")
    if abs(ss_mean - REF_SS_RMS) > args.rms_tol:
        print(f"    FAIL: exceeds rms-tol {args.rms_tol:.0e}")
        ok = False

    if ee_target is not None:
        ee_final = cpp[-1, 19:22]
        miss = float(np.linalg.norm(ee_final - ee_target))
        print(f"  EE miss = {miss:.4f} m "
              f"(ref {REF_EE_MISS:.4f}, delta {abs(miss - REF_EE_MISS):.2e})")
        if abs(miss - REF_EE_MISS) > args.ee_tol:
            print(f"    FAIL: exceeds ee-tol {args.ee_tol:.0e}")
            ok = False
    else:
        print("  EE miss: skipped (no ee_target in oracle CSV header)")

    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
