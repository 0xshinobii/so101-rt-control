"""Phase 6 acceptance: momentum-DOB disturbance rejection and bandwidth."""

import argparse
import csv
import math
import os
import sys

import numpy as np

TARGET = np.array([0.6, 0.7, -0.8, 0.5, 0.4, 0.0])
SECOND_TARGET = np.array([-0.6, -0.7, 0.8, -0.5, -0.4, 0.0])
N_ARM = 5
PRE_START = 1.5
FORCE_ONSET = 2.5
POST_START = 3.5
FREQUENCIES = (0.5, 2.0, 12.0)
THEORY_AMPLITUDE = (0.998, 0.970, 0.555)
THEORY_PHASE_DEG = (-3.576, -14.036, -56.310)


def load_case(path):
    metadata = {}
    lines = []
    with open(path) as stream:
        for line in stream:
            if line.startswith("#"):
                content = line[1:].strip()
                if "=" in content:
                    key, value = content.split("=", 1)
                    metadata[key.strip()] = value.strip()
            elif line.strip():
                lines.append(line)
    rows = list(csv.DictReader(lines))
    if not rows:
        raise ValueError(f"{path}: no trajectory rows")
    names = rows[0].keys()
    data = {
        name: np.array([float(row[name]) for row in rows])
        for name in names
    }
    return data, metadata


def matrix(data, prefix, count):
    return np.column_stack([data[f"{prefix}{i}"] for i in range(count)])


def tracking_rms(data, mask):
    q = matrix(data, "q", 6)
    per_joint = np.sqrt(np.mean(
        (q[mask, :N_ARM] - TARGET[:N_ARM]) ** 2, axis=0
    ))
    return float(np.mean(per_joint))


def window_metrics(case, ee_target, post=True):
    data, metadata = case
    time = data["t"]
    mask = time >= (POST_START if post else PRE_START)
    if not post:
        mask &= time < FORCE_ONSET
    q = matrix(data, "q", 6)
    qdot = matrix(data, "qd", 6)
    tau = matrix(data, "tau", 6)
    ee = np.column_stack([data["ee_x"], data["ee_y"], data["ee_z"]])
    error = q[mask, :N_ARM] - TARGET[:N_ARM]
    return {
        "rms": float(np.mean(np.sqrt(np.mean(error**2, axis=0)))),
        "ee": float(np.linalg.norm(np.mean(ee[mask], axis=0) - ee_target)),
        "peak_tau": float(np.max(np.abs(tau))),
        "q_ptp": float(np.max(np.ptp(q[mask, :N_ARM], axis=0))),
        "qdot_rms": float(np.sqrt(np.mean(qdot[mask, :N_ARM] ** 2))),
        "saturated": int(metadata.get("saturated_samples", 0)),
    }


def disturbance_norm(case, mask):
    data, _ = case
    estimate = matrix(data, "tau_ext_hat", 6)
    return float(np.sqrt(np.mean(np.sum(estimate[mask] ** 2, axis=1))))


def transfer_rms(case):
    data, _ = case
    mask = data["t"] >= POST_START
    q = matrix(data, "q", 6)
    error = q[mask, :N_ARM] - SECOND_TARGET[:N_ARM]
    return float(np.mean(np.sqrt(np.mean(error**2, axis=0))))


def complex_coefficient(values, times, frequency):
    phase = np.exp(-2j * np.pi * frequency * times)
    return np.sum(values * phase[:, None], axis=0)


def frequency_metrics(baseline_case, dob_case, frequency):
    baseline, _ = baseline_case
    dob, _ = dob_case
    periods = max(5, int(math.floor((dob["t"][-1] - POST_START) * frequency)))
    stop = POST_START + periods / frequency
    mask_b = (baseline["t"] >= POST_START) & (baseline["t"] < stop)
    mask_c = (dob["t"] >= POST_START) & (dob["t"] < stop)
    baseline_error = tracking_rms(baseline, mask_b)
    dob_error = tracking_rms(dob, mask_c)

    truth = matrix(dob, "tau_ext_true", 6)[mask_c]
    estimate = matrix(dob, "tau_ext_hat", 6)[mask_c]
    pre = (dob["t"] >= PRE_START) & (dob["t"] < FORCE_ONSET)
    estimate -= np.mean(matrix(dob, "tau_ext_hat", 6)[pre], axis=0)
    truth_coeff = complex_coefficient(truth, dob["t"][mask_c], frequency)
    estimate_coeff = complex_coefficient(
        estimate, dob["t"][mask_c], frequency
    )
    excited = np.abs(truth_coeff) > 0.01 * np.max(np.abs(truth_coeff))
    transfer = np.vdot(
        truth_coeff[excited], estimate_coeff[excited]
    ) / np.vdot(truth_coeff[excited], truth_coeff[excited])

    force = np.column_stack(
        [dob["force_x"], dob["force_y"], dob["force_z"]]
    )[mask_c]
    force_hat = np.column_stack(
        [dob["force_hat_x"], dob["force_hat_y"], dob["force_hat_z"]]
    )
    force_hat = force_hat[mask_c] - np.mean(force_hat[pre], axis=0)
    force_rmse = float(np.sqrt(np.mean((force_hat - force) ** 2)))
    return {
        "frequency": frequency,
        "baseline_rms": baseline_error,
        "dob_rms": dob_error,
        "rejection_ratio": dob_error / baseline_error,
        "amplitude": float(abs(transfer)),
        "phase_deg": float(np.degrees(np.angle(transfer))),
        "force_rmse": force_rmse,
        "condition": float(np.max(dob["jacobian_condition"][mask_c])),
    }


def write_frequency_artifact(path, metrics):
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0].keys()))
        writer.writeheader()
        writer.writerows(metrics)

    svg_path = os.path.splitext(path)[0] + ".svg"
    width, height, margin = 640, 360, 55
    x0, x1 = math.log10(0.4), math.log10(15.0)
    points = []
    theory = []
    for index, result in enumerate(metrics):
        x = margin + (math.log10(result["frequency"]) - x0) / (x1 - x0) * (
            width - 2 * margin
        )
        y = height - margin - result["amplitude"] / 1.25 * (
            height - 2 * margin
        )
        yt = height - margin - THEORY_AMPLITUDE[index] / 1.25 * (
            height - 2 * margin
        )
        points.append(f"{x:.1f},{y:.1f}")
        theory.append(f"{x:.1f},{yt:.1f}")
    with open(svg_path, "w") as stream:
        stream.write(
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
            '<rect width="100%" height="100%" fill="white"/>'
            f'<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" '
            f'y2="{height-margin}" stroke="black"/>'
            f'<line x1="{margin}" y1="{margin}" x2="{margin}" '
            f'y2="{height-margin}" stroke="black"/>'
            '<text x="320" y="22" text-anchor="middle" '
            'font-family="sans-serif" font-weight="bold">'
            'Phase 6 DOB frequency response</text>'
            f'<polyline points="{" ".join(theory)}" fill="none" '
            'stroke="#777" stroke-dasharray="6 4" stroke-width="2"/>'
            f'<polyline points="{" ".join(points)}" fill="none" '
            'stroke="#1565c0" stroke-width="3"/>'
            '<text x="320" y="345" text-anchor="middle" '
            'font-family="sans-serif">Frequency [Hz] (log scale)</text>'
            '<text x="16" y="180" text-anchor="middle" '
            'transform="rotate(-90 16 180)" font-family="sans-serif">'
            'Disturbance estimate amplitude ratio</text>'
            '<text x="450" y="48" font-family="sans-serif" fill="#1565c0">'
            'measured</text><text x="530" y="48" font-family="sans-serif" '
            'fill="#777">theory</text>'
            '<text x="88" y="322" text-anchor="middle" '
            'font-family="sans-serif">0.5</text>'
            '<text x="290" y="322" text-anchor="middle" '
            'font-family="sans-serif">2</text>'
            '<text x="552" y="322" text-anchor="middle" '
            'font-family="sans-serif">12</text>'
            '<text x="48" y="309" text-anchor="end" '
            'font-family="sans-serif">0</text>'
            '<text x="48" y="209" text-anchor="end" '
            'font-family="sans-serif">0.5</text>'
            '<text x="48" y="109" text-anchor="end" '
            'font-family="sans-serif">1.0</text>'
            "</svg>"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("a", "b", "c", "d", "dv", "e", "e_dob", "f", "g", "h"):
        parser.add_argument(f"--{name.replace('_', '-')}", required=True)
    parser.add_argument("--b-omega", nargs=3, required=True)
    parser.add_argument("--c-omega", nargs=3, required=True)
    parser.add_argument("--h-omega", required=True)
    parser.add_argument("--g-xfer", required=True)
    parser.add_argument("--h-xfer", required=True)
    parser.add_argument(
        "--frequency-output", default="phase6_frequency_response.csv"
    )
    parser.add_argument("--characterize", action="store_true")
    args = parser.parse_args()

    cases = {
        name: load_case(getattr(args, name))
        for name in ("a", "b", "c", "d", "dv", "e", "e_dob", "f", "g", "h")
    }
    h_omega = load_case(args.h_omega)
    g_xfer = load_case(args.g_xfer)
    h_xfer = load_case(args.h_xfer)
    b_omega = [load_case(path) for path in args.b_omega]
    c_omega = [load_case(path) for path in args.c_omega]

    a_data, _ = cases["a"]
    a_settled = a_data["t"] >= PRE_START
    ee_target = np.mean(np.column_stack(
        [a_data["ee_x"], a_data["ee_y"], a_data["ee_z"]]
    )[a_settled], axis=0)

    print("== Constant-force and stacked cases ==")
    summary = {}
    for name in ("a", "b", "c", "d", "dv", "e", "e_dob", "f", "g", "h"):
        post = name not in ("a", "e", "e_dob")
        summary[name] = window_metrics(cases[name], ee_target, post=post)
        m = summary[name]
        print(
            f"{name:5s} rms={m['rms']:.6f} ee={m['ee']:.6f} "
            f"peak_tau={m['peak_tau']:.3f} q_ptp={m['q_ptp']:.6f}"
        )

    g_mask = cases["g"][0]["t"] >= POST_START
    h_mask = cases["h"][0]["t"] >= POST_START
    g_residual = disturbance_norm(cases["g"], g_mask)
    h_residual = disturbance_norm(cases["h"], h_mask)
    print(f"G/H disturbance RMS norm = {g_residual:.6f}/{h_residual:.6f} N.m")
    e_mass = float(cases["e"][1]["final_payload_mass_estimate"])
    e_dob_mass = float(cases["e_dob"][1]["final_payload_mass_estimate"])
    print(f"E/E_dob mass = {e_mass:.6f}/{e_dob_mass:.6f} kg")

    print("\n== Direct frequency response ==")
    frequency_results = []
    for index, frequency in enumerate(FREQUENCIES):
        result = frequency_metrics(
            b_omega[index], c_omega[index], frequency
        )
        frequency_results.append(result)
        print(
            f"{frequency:4.1f} Hz: |H|={result['amplitude']:.3f} "
            f"phase={result['phase_deg']:.1f} deg "
            f"B/C rms={result['baseline_rms']:.6f}/"
            f"{result['dob_rms']:.6f} force_rmse="
            f"{result['force_rmse']:.3f} N"
        )
    write_frequency_artifact(args.frequency_output, frequency_results)

    h2 = frequency_metrics(b_omega[1], h_omega, 2.0)
    gx_rms = transfer_rms(g_xfer)
    hx_rms = transfer_rms(h_xfer)
    print(f"Hω 2 Hz rms={h2['dob_rms']:.6f}")
    print(f"G_xfer/H_xfer rms={gx_rms:.6f}/{hx_rms:.6f}")

    checks = [
        ("DOB improves constant-force RMS", summary["c"]["rms"] < summary["b"]["rms"]),
        ("DOB improves constant-force EE", summary["c"]["ee"] < summary["b"]["ee"]),
        ("stacked no-force remains bounded", summary["e_dob"]["rms"] < 0.01),
        ("stacked no-force preserves payload identification",
         abs(e_dob_mass - e_mass) < 0.02),
        ("stacked force rejection improves adaptive-only", summary["h"]["rms"] < summary["f"]["rms"]),
        ("stacked residual is smaller than lumped residual", h_residual < g_residual),
        ("payload model transfers better than frozen lumped residual", hx_rms < gx_rms),
        ("all primary DOB runs avoid saturation",
         all(summary[name]["saturated"] == 0 for name in ("c", "e_dob", "h"))),
        ("frequency amplitude decreases monotonically",
         all(frequency_results[i]["amplitude"] >
             frequency_results[i + 1]["amplitude"] for i in range(2))),
    ]
    if not args.characterize:
        for index, result in enumerate(frequency_results):
            checks.append((
                f"{FREQUENCIES[index]} Hz amplitude matches first-order theory",
                abs(result["amplitude"] - THEORY_AMPLITUDE[index]) < 0.12,
            ))
            checks.append((
                f"{FREQUENCIES[index]} Hz phase matches first-order theory",
                abs(result["phase_deg"] - THEORY_PHASE_DEG[index]) < 12.0,
            ))

    print("\nAcceptance:")
    for name, passed in checks:
        print(f"  {'PASS' if passed else 'FAIL'} {name}")
    ok = all(passed for _, passed in checks)
    print("\nPHASE 6 BENCH:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
