"""Phase 4 acceptance: fair PD vs computed-torque benchmark matrix."""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.metrics import rms_error  # noqa: E402
from tools.arm_bench import load_csv  # noqa: E402

TARGET = np.array([0.6, 0.7, -0.8, 0.5, 0.4, 0.0])
N_ARM = 5
REFERENCE_DURATION = 1.0
SETTLE_T = 1.5
TORQUE_LIMIT = 2.94


def load_metadata(path):
    metadata = {}
    with open(path) as stream:
        for line in stream:
            if not line.startswith("#"):
                continue
            content = line[1:].strip()
            if "=" in content:
                key, value = content.split("=", 1)
                metadata[key.strip()] = value.strip()
    return metadata


def reference_position(times):
    s = np.clip(times / REFERENCE_DURATION, 0.0, 1.0)
    scale = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    return scale[:, None] * TARGET


def settling_time(times, q, threshold=0.01):
    error = np.abs(q[:, :N_ARM] - TARGET[:N_ARM])
    for index in np.flatnonzero(times >= REFERENCE_DURATION):
        if np.all(error[index:] <= threshold):
            return float(times[index])
    return float("inf")


def case_metrics(path, ee_target):
    data, _ = load_csv(path)
    metadata = load_metadata(path)
    if data.ndim != 2 or data.shape[1] != 22:
        raise ValueError(f"{path}: expected 22-column trajectory, got {data.shape}")
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
    applied_peak = np.minimum(raw_peak, TORQUE_LIMIT)
    if "applied_peak_tau" in metadata:
        applied_peak = np.array(
            [float(value) for value in metadata["applied_peak_tau"].split(",")]
        )

    saturated_samples = int(metadata.get(
        "saturated_samples",
        np.count_nonzero(np.any(np.abs(tau) > TORQUE_LIMIT, axis=1)),
    ))
    post_reference = time >= REFERENCE_DURATION
    signed_overshoot = (
        (q[post_reference, :N_ARM] - TARGET[:N_ARM])
        * np.sign(TARGET[:N_ARM])
    )

    return {
        "path": path,
        "reference": metadata.get("reference", "unknown"),
        "ss_mean": float(ss_per_joint[:N_ARM].mean()),
        "ss_per_joint": ss_per_joint[:N_ARM],
        "ee_miss": float(np.linalg.norm(data[-1, 19:22] - ee_target)),
        "max_tracking_error": float(np.max(np.abs(tracking_error[:, :N_ARM]))),
        "transient_rms": float(np.sqrt(np.mean(
            tracking_error[time <= SETTLE_T, :N_ARM] ** 2
        ))),
        "overshoot": float(max(0.0, np.max(signed_overshoot))),
        "settling_time": settling_time(time, q),
        "final_speed": float(np.max(np.abs(qdot[-1, :N_ARM]))),
        "raw_peak": raw_peak,
        "applied_peak": applied_peak,
        "saturated_samples": saturated_samples,
    }


def print_case(name, metrics):
    settling = metrics["settling_time"]
    settling_text = f"{settling:.3f} s" if np.isfinite(settling) else "not reached"
    print(f"{name}:")
    print(f"  steady-state arm-mean RMS = {metrics['ss_mean']:.6f} rad")
    print(f"  EE miss                   = {metrics['ee_miss']:.6f} m")
    print(f"  transient RMS             = {metrics['transient_rms']:.6f} rad")
    print(f"  max tracking error        = {metrics['max_tracking_error']:.6f} rad")
    print(f"  overshoot                 = {metrics['overshoot']:.6f} rad")
    print(f"  0.01-rad settling time    = {settling_text}")
    print(f"  raw peak torque           = {metrics['raw_peak'].max():.6f} N.m")
    print(f"  applied peak torque       = {metrics['applied_peak'].max():.6f} N.m")
    print(f"  saturated samples         = {metrics['saturated_samples']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pd-empty", required=True)
    parser.add_argument("--ct-empty", required=True)
    parser.add_argument("--pd-payload", required=True)
    parser.add_argument("--ct-payload", required=True)
    parser.add_argument("--oracle", required=True)
    args = parser.parse_args()

    _, ee_target = load_csv(args.oracle)
    if ee_target is None:
        raise ValueError("oracle CSV must provide # ee_target=x,y,z")

    cases = {
        "PD empty": case_metrics(args.pd_empty, ee_target),
        "CT empty": case_metrics(args.ct_empty, ee_target),
        "PD known payload": case_metrics(args.pd_payload, ee_target),
        "CT known payload": case_metrics(args.ct_payload, ee_target),
    }
    for name, metrics in cases.items():
        print_case(name, metrics)

    pd_empty = cases["PD empty"]
    ct_empty = cases["CT empty"]
    pd_payload = cases["PD known payload"]
    ct_payload = cases["CT known payload"]
    checks = [
        ("same bounded reference",
         all(case["reference"] == "smooth" for case in cases.values())),
        ("CT empty steady-state RMS beats PD",
         ct_empty["ss_mean"] < pd_empty["ss_mean"]),
        ("CT loaded steady-state RMS beats PD",
         ct_payload["ss_mean"] < pd_payload["ss_mean"]),
        ("CT empty EE miss beats PD",
         ct_empty["ee_miss"] < pd_empty["ee_miss"]),
        ("CT loaded EE miss beats PD",
         ct_payload["ee_miss"] < pd_payload["ee_miss"]),
        ("known payload materially degrades PD",
         pd_payload["ee_miss"] > 1.5 * pd_empty["ee_miss"]),
        ("CT loaded reaches <5 mm EE miss",
         ct_payload["ee_miss"] < 0.005),
        ("CT transients stay within 0.1 rad",
         ct_empty["max_tracking_error"] < 0.1
         and ct_payload["max_tracking_error"] < 0.1),
        ("CT runs do not saturate",
         ct_empty["saturated_samples"] == 0
         and ct_payload["saturated_samples"] == 0),
        ("CT final joint speeds are bounded",
         ct_empty["final_speed"] < 0.02
         and ct_payload["final_speed"] < 0.02),
    ]

    print("\nAcceptance:")
    for name, passed in checks:
        print(f"  {'PASS' if passed else 'FAIL'} {name}")
    ok = all(passed for _, passed in checks)
    print("\nPHASE 4 BENCH:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
