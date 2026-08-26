#!/usr/bin/env python3
"""Recompute the final 90/180 g hardware campaign table from CSV logs."""

import csv
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "docs" / "data" / "final_campaign"
TARGET = (0.6, 0.7, -0.8, 0.5, 0.4)
RUNS = (
    (90, "PD", "final_hw_090g_01_pd.csv", None),
    (90, "CT empty", "final_hw_090g_02_ct_empty.csv", 0.0),
    (90, "static raw", "final_hw_090g_03_adapt_raw_static.csv", "csv"),
    (90, "static affine", "final_hw_090g_04_adapt_affine_static.csv", "csv"),
    (90, "motion RLS", "final_hw_090g_05_adapt_motion_rls.csv", "csv"),
    (180, "PD", "final_hw_180g_01_pd.csv", None),
    (180, "CT empty", "final_hw_180g_02_ct_empty.csv", 0.0),
    (180, "static raw", "final_hw_180g_03_adapt_raw_static.csv", "csv"),
    (180, "static affine", "final_hw_180g_04_adapt_affine_static.csv", "csv"),
    (180, "motion RLS", "final_hw_180g_05_adapt_motion_rls.csv", "csv"),
)


def load(path):
    with path.open() as handle:
        return list(
            csv.DictReader(line for line in handle if not line.startswith("#"))
        )


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def main():
    print(
        "| mass | controller | mass used [kg] | lift offset [rad] | "
        "elbow offset [rad] | RMS no roll [rad] | EE z [m] |"
    )
    print("|---|---|---:|---:|---:|---:|---:|")
    for payload_g, controller, filename, mass_source in RUNS:
        rows = load(DATA / filename)
        settled = [row for row in rows if float(row["t"]) >= 1.5]
        joint_rms = [
            rms([TARGET[j] - float(row[f"q{j}"]) for row in settled])
            for j in range(4)
        ]
        arm_rms = rms(joint_rms)
        last = rows[-1]
        offsets = [TARGET[j] - float(last[f"q{j}"]) for j in range(5)]
        if mass_source == "csv":
            mass = f'{float(last["compensated_payload_mass"]):.4f}'
        elif mass_source is None:
            mass = "—"
        else:
            mass = f"{mass_source:.4f}"
        print(
            f"| {payload_g} g | {controller} | {mass} | "
            f"{offsets[1]:+.4f} | {offsets[2]:+.4f} | "
            f"{arm_rms:.4f} | {float(last['ee_z']):.3f} |"
        )


if __name__ == "__main__":
    main()
