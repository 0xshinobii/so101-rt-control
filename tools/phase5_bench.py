"""Phase 5 acceptance: online payload identification and tracking recovery."""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.metrics import rms_error  # noqa: E402
from tools.arm_bench import load_csv  # noqa: E402
from tools.phase4_bench import (  # noqa: E402
    N_ARM,
    SETTLE_T,
    TARGET,
    TORQUE_LIMIT,
    load_metadata,
    reference_position,
    settling_time,
)

# Frozen from the first deterministic characterization: the larger corrected
# error was 0.002685 kg. A 0.0035 kg gate retains ~30% regression margin.
MASS_ERROR_LIMIT_KG = 0.0035
MIN_RECOVERY = 0.50


def case_metrics(path, ee_target):
    data, _ = load_csv(path)
    metadata = load_metadata(path)
    if data.ndim != 2 or data.shape[1] not in (22, 23):
        raise ValueError(f"{path}: expected 22 or 23 columns, got {data.shape}")
    if not np.all(np.isfinite(data)):
        raise ValueError(f"{path}: trajectory contains NaN/Inf")

    time = data[:, 0]
    q = data[:, 1:7]
    qdot = data[:, 7:13]
    tau = data[:, 13:19]
    desired = reference_position(time)
    tracking_error = q - desired
    settled = time >= SETTLE_T
    ss_per_joint = rms_error(q[settled], TARGET)

    raw_peak = np.max(np.abs(tau), axis=0)
    if "raw_peak_tau" in metadata:
        raw_peak = np.array(
            [float(value) for value in metadata["raw_peak_tau"].split(",")]
        )
    saturated_samples = int(metadata.get("saturated_samples", 0))
    mass_estimate = float(metadata.get(
        "final_payload_mass_estimate",
        data[-1, 22] if data.shape[1] == 23 else "nan",
    ))
    mass_convergence_time = float("nan")
    if data.shape[1] == 23 and np.isfinite(mass_estimate):
        tolerance = max(0.005, 0.05 * abs(mass_estimate))
        within = np.abs(data[:, 22] - mass_estimate) <= tolerance
        for index in range(len(time)):
            if np.all(within[index:]):
                mass_convergence_time = float(time[index])
                break

    return {
        "path": path,
        "reference": metadata.get("reference", "unknown"),
        "ss_mean": float(ss_per_joint[:N_ARM].mean()),
        "ee_miss": float(np.linalg.norm(data[-1, 19:22] - ee_target)),
        "max_tracking_error": float(
            np.max(np.abs(tracking_error[:, :N_ARM]))
        ),
        "settling_time": settling_time(time, q),
        "final_speed": float(np.max(np.abs(qdot[-1, :N_ARM]))),
        "raw_peak": raw_peak,
        "applied_peak": np.minimum(raw_peak, TORQUE_LIMIT),
        "saturated_samples": saturated_samples,
        "mass_estimate": mass_estimate,
        "mass_convergence_time": mass_convergence_time,
        "estimator_updates": int(metadata.get("estimator_updates", 0)),
        "qdot": qdot,
    }


def recovery_ratio(mismatch, adaptive, known, metric):
    gap = mismatch[metric] - known[metric]
    if gap <= 0.0:
        raise ValueError(f"known bound does not beat mismatch for {metric}")
    return (mismatch[metric] - adaptive[metric]) / gap


def print_case(name, metrics):
    print(f"{name}:")
    print(f"  steady-state arm-mean RMS = {metrics['ss_mean']:.6f} rad")
    print(f"  EE miss                   = {metrics['ee_miss']:.6f} m")
    print(f"  max tracking error        = {metrics['max_tracking_error']:.6f} rad")
    print(f"  final speed               = {metrics['final_speed']:.6f} rad/s")
    print(f"  raw peak torque           = {metrics['raw_peak'].max():.6f} N.m")
    print(f"  saturated samples         = {metrics['saturated_samples']}")
    if np.isfinite(metrics["mass_estimate"]):
        print(f"  final mass estimate       = {metrics['mass_estimate']:.6f} kg")
        print(
            "  mass convergence time     = "
            f"{metrics['mass_convergence_time']:.3f} s"
        )
        print(f"  accepted RLS updates      = {metrics['estimator_updates']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--empty-adaptive", required=True)
    parser.add_argument("--mismatch-010", required=True)
    parser.add_argument("--adaptive-010", required=True)
    parser.add_argument("--mismatch-020", required=True)
    parser.add_argument("--adaptive-020", required=True)
    parser.add_argument("--known-020", required=True)
    parser.add_argument("--oracle", required=True)
    parser.add_argument(
        "--characterize",
        action="store_true",
        help="report metrics before freezing the measured mass-error bound",
    )
    args = parser.parse_args()

    _, ee_target = load_csv(args.oracle)
    if ee_target is None:
        raise ValueError("oracle CSV must provide # ee_target=x,y,z")

    cases = {
        "Adaptive empty": case_metrics(args.empty_adaptive, ee_target),
        "CT mismatch 0.10 kg": case_metrics(args.mismatch_010, ee_target),
        "Adaptive 0.10 kg": case_metrics(args.adaptive_010, ee_target),
        "CT mismatch 0.20 kg": case_metrics(args.mismatch_020, ee_target),
        "Adaptive 0.20 kg": case_metrics(args.adaptive_020, ee_target),
        "CT known 0.20 kg": case_metrics(args.known_020, ee_target),
    }
    for name, metrics in cases.items():
        print_case(name, metrics)

    empty = cases["Adaptive empty"]
    mismatch_010 = cases["CT mismatch 0.10 kg"]
    adaptive_010 = cases["Adaptive 0.10 kg"]
    mismatch_020 = cases["CT mismatch 0.20 kg"]
    adaptive_020 = cases["Adaptive 0.20 kg"]
    known_020 = cases["CT known 0.20 kg"]
    bias = empty["mass_estimate"]
    corrected_010 = adaptive_010["mass_estimate"] - bias
    corrected_020 = adaptive_020["mass_estimate"] - bias
    mass_error_010 = abs(corrected_010 - 0.10)
    mass_error_020 = abs(corrected_020 - 0.20)
    rms_recovery = recovery_ratio(
        mismatch_020, adaptive_020, known_020, "ss_mean"
    )
    ee_recovery = recovery_ratio(
        mismatch_020, adaptive_020, known_020, "ee_miss"
    )
    velocity_bias_caveat_010 = float(np.sqrt(np.mean(
        (adaptive_010["qdot"][:, :N_ARM] - empty["qdot"][:, :N_ARM]) ** 2
    )))
    velocity_bias_caveat_020 = float(np.sqrt(np.mean(
        (adaptive_020["qdot"][:, :N_ARM] - empty["qdot"][:, :N_ARM]) ** 2
    )))

    print("\nEstimator diagnostics:")
    print(f"  empty-arm bias floor      = {bias:.6f} kg")
    print(f"  corrected 0.10 kg         = {corrected_010:.6f} kg")
    print(f"  corrected 0.10 kg error   = {mass_error_010:.6f} kg")
    print(f"  corrected 0.20 kg         = {corrected_020:.6f} kg")
    print(f"  corrected 0.20 kg error   = {mass_error_020:.6f} kg")
    print(f"  0.20 kg RMS recovery      = {100.0 * rms_recovery:.2f}%")
    print(f"  0.20 kg EE recovery       = {100.0 * ee_recovery:.2f}%")
    print(
        "  qdot RMS delta vs empty   = "
        f"{velocity_bias_caveat_010:.6f} / "
        f"{velocity_bias_caveat_020:.6f} rad/s"
    )

    checks = [
        ("same bounded reference",
         all(case["reference"] == "smooth" for case in cases.values())),
        ("all estimators receive updates",
         empty["estimator_updates"] > 0
         and adaptive_010["estimator_updates"] > 0
         and adaptive_020["estimator_updates"] > 0),
        ("adaptive 0.10 kg improves steady-state RMS",
         adaptive_010["ss_mean"] < mismatch_010["ss_mean"]),
        ("adaptive 0.10 kg improves EE miss",
         adaptive_010["ee_miss"] < mismatch_010["ee_miss"]),
        ("adaptive 0.20 kg improves steady-state RMS",
         adaptive_020["ss_mean"] < mismatch_020["ss_mean"]),
        ("adaptive 0.20 kg improves EE miss",
         adaptive_020["ee_miss"] < mismatch_020["ee_miss"]),
        ("0.20 kg RMS recovery exceeds sanity floor",
         rms_recovery >= MIN_RECOVERY),
        ("0.20 kg EE recovery exceeds sanity floor",
         ee_recovery >= MIN_RECOVERY),
        ("adaptive runs do not saturate",
         empty["saturated_samples"] == 0
         and adaptive_010["saturated_samples"] == 0
         and adaptive_020["saturated_samples"] == 0),
        ("adaptive final speeds are bounded",
         empty["final_speed"] < 0.02
         and adaptive_010["final_speed"] < 0.02
         and adaptive_020["final_speed"] < 0.02),
    ]
    if not args.characterize:
        if MASS_ERROR_LIMIT_KG is None:
            raise RuntimeError(
                "freeze MASS_ERROR_LIMIT_KG after characterization"
            )
        checks.extend([
            ("bias-corrected 0.10 kg estimate is within measured bound",
             mass_error_010 <= MASS_ERROR_LIMIT_KG),
            ("bias-corrected 0.20 kg estimate is within measured bound",
             mass_error_020 <= MASS_ERROR_LIMIT_KG),
        ])

    print("\nAcceptance:")
    for name, passed in checks:
        print(f"  {'PASS' if passed else 'FAIL'} {name}")
    ok = all(passed for _, passed in checks)
    label = "CHARACTERIZATION" if args.characterize else "PHASE 5 BENCH"
    print(f"\n{label}:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
