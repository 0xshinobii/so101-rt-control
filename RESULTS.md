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
Phase 4's known payload degrades this baseline and computed torque recovers it;
Phase 5 will repeat the loaded case without giving the payload to the model.

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
  by design.
- Plots (`baseline_tracking.png`, `baseline_so101_tracking.png`) are gitignored;
  regenerate with the scripts above.

## Phase 4 — Pinocchio computed torque

Phase 4 uses the same 1.0 s minimum-jerk reference for PD and computed torque.
The settled window begins at 1.5 s. Computed torque uses acceleration-domain
gains `Kp=400 s^-2`, `Kd=40 s^-1` (critically damped, natural frequency
20 rad/s), with the real +/-2.94 N.m actuator limit unchanged.

Rigid-body equivalence is a blocking test before controller execution:

- Empty and known-payload pairs both pass gravity, nonlinear-bias, and full
  mass-matrix comparisons at five poses and two nonzero velocity vectors.
- Worst empty-model differences: gravity `5.55e-16 N.m`, nonlinear bias
  `8.66e-13 N.m`, mass matrix `5.64e-12` absolute.
- Worst known-payload differences: gravity `1.33e-15 N.m`, nonlinear bias
  `8.66e-13 N.m`, mass matrix `5.64e-12` absolute.
- The 0.20 kg known payload reaches `1.546 N.m` worst holding torque in the
  gate pose set: 52.6% of the actuator limit, leaving transient headroom.

Empty-arm result:

- PD settled arm-mean RMS: `0.009959 rad`; computed torque: `0.002933 rad`
  (70.5% reduction).
- PD end-effector miss: `0.014012 m`; computed torque: `0.000327 m`
  (97.7% reduction).
- Computed torque settles inside 0.01 rad by 1.225 s, with zero saturated
  samples and `1.360 N.m` peak torque.

Known-payload result (the same payload is present in MuJoCo and Pinocchio):

- PD settled arm-mean RMS: `0.021612 rad`; computed torque: `0.002840 rad`
  (86.9% reduction).
- PD end-effector miss: `0.029999 m`; computed torque: `0.000184 m`
  (99.4% reduction).
- The payload more than doubles PD's Cartesian miss, while computed torque
  returns to the Coulomb-friction floor. It settles inside 0.01 rad by 1.235 s,
  with zero saturated samples and `1.923 N.m` peak torque.

The structural result is gravity cancellation, not a gain bake-off: plain PD
must retain position error to generate holding torque, while computed torque
provides the modeled gravity torque at zero error. Remaining steady-state error
is the MuJoCo `frictionloss` deadband; damping and armature affect only the
transient. Transient metrics are still reported by `tools/phase4_bench.py`, but
they are supporting evidence because the two controllers use different-domain
gains.

This is a perfect-knowledge result. The known-payload computed-torque case is
Phase 5's upper bound; an unknown payload omitted from Pinocchio should first
degrade tracking, then an online estimator should recover toward this result.
