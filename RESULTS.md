# Baseline reference metrics

Naive PD (`ki = 0`, no gravity/dynamics compensation) driving a single
step-to-setpoint move. These are the references every later controller is
measured against. Same metric definitions for both plants (see
[src/metrics.py](src/metrics.py)): whole-run joint RMS error and the final
Cartesian end-effector miss.

| Plant | Script | DOF | Joint RMS [rad] | EE miss [m] |
|-------|--------|-----|-----------------|-------------|
| 2-link arm (concept bootstrap) | `run_baseline.py` | 2 | 0.1119, 0.0858 | 0.0262 |
| SO-101 / SO-ARM100 (current plant) | `run_baseline_so101.py` | 5 arm + gripper | mean 0.0916 (per-joint below) | 0.0140 |

SO-101 per-arm-joint RMS [rad] (gripper held, excluded from the metric):

| shoulder_pan | shoulder_lift | elbow_flex | wrist_flex | wrist_roll |
|--------------|---------------|------------|------------|------------|
| 0.0932 | 0.1054 | 0.1304 | 0.0693 | 0.0598 |

Notes:
- The SO-101 is the plant for everything from Phase 1.5 onward; `arm2.xml` /
  `run_baseline.py` stay as the documented concept bootstrap.
- SO-101 motors use the real STS3215 torque envelope (+/-2.94 N.m); it was not
  widened to make PD work. Gravity torques on this light arm are small (<= ~0.8
  N.m), so the naive-PD steady-state droop is modest but present (clearest on
  `elbow_flex` / `shoulder_lift`) and shows up in the final EE miss.
- Plots (`baseline_tracking.png`, `baseline_so101_tracking.png`) are gitignored;
  regenerate with the scripts above.
