# Baseline reference metrics

Naive PD (`ki = 0`, no gravity/dynamics compensation) driving a single
step-to-setpoint move. These are the references every later controller is
measured against.

## Primary metrics (the droop that later phases remove)

The phenomenon we study is the **steady-state gravity droop** -- the residual
error once the arm has settled -- plus the Cartesian end-effector miss. A
step-to-setpoint RMS over the whole run is dominated by the ~0.5 s rise (how far
each joint travelled), not the droop, so it is kept only as a secondary transit
check below. Steady-state = RMS over the settled window (`t > 1 s`); final
offset = signed `target - final` (the sag).

| Plant | Script | DOF | Steady-state arm RMS [rad] | EE miss [m] |
|-------|--------|-----|----------------------------|-------------|
| 2-link arm (concept bootstrap) | `run_baseline.py` | 2 | -- (see full RMS below) | 0.0262 |
| SO-101 / SO-ARM100 (current plant) | `run_baseline_so101.py` | 5 arm + gripper | mean 0.0100 | 0.0140 |

SO-101 per-arm-joint steady-state droop (`t > 1 s`):

| | shoulder_pan | shoulder_lift | elbow_flex | wrist_flex | wrist_roll |
|-|--------------|---------------|------------|------------|------------|
| RMS [rad] | 0.0005 | 0.0198 | 0.0195 | 0.0062 | 0.0039 |
| final offset [rad] | +0.0001 | -0.0201 | -0.0189 | -0.0071 | +0.0021 |

The droop concentrates on the gravity-loaded joints (`shoulder_lift`,
`elbow_flex`, `wrist_flex`); `shoulder_pan` / `wrist_roll` have near-vertical
axes (~0 gravity torque) so they show essentially no droop. This is the number
Phase 3 (payload) will degrade and Phases 4-6 (computed-torque / estimator /
SMC) will recover.

## Secondary metric (transit / "did it reach the setpoint" check)

Full-window joint RMS [rad] -- transient-dominated, not the droop story:

| Plant | Arm-joint RMS [rad] | arm mean |
|-------|---------------------|----------|
| 2-link arm | 0.1119, 0.0858 | -- |
| SO-101 | 0.0932, 0.1054, 0.1304, 0.0693, 0.0598 | 0.0916 |

(SO-101 held gripper, full-window RMS: 0.0002 -- excluded from the arm metric.)

## Notes

- The SO-101 is the plant for everything from Phase 1.5 onward; `arm2.xml` /
  `run_baseline.py` stay as the documented concept bootstrap.
- SO-101 motors use the real STS3215 torque envelope (+/-2.94 N.m); it was not
  widened to make PD work. Gravity torques on this light arm are modest
  (`shoulder_lift` ~0.8 N.m at the target pose), so the empty-arm droop is small
  by design -- the compelling contrast comes in Phase 3 under payload.
- Plots (`baseline_tracking.png`, `baseline_so101_tracking.png`) are gitignored;
  regenerate with the scripts above.
