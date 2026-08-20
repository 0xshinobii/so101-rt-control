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
Phase 5 repeats the loaded case without giving the payload mass to the model.

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
Phase 5's upper bound. The runs below omit the payload from Pinocchio, show the
resulting degradation, then recover toward this result with online estimation.

## Phase 5 — Online payload-mass estimation

Phase 5 keeps the payload's attachment pose and inertia shape as a known
template but treats its scalar mass as unknown. A real-time scalar RLS
estimator uses the exact per-kilogram inverse-dynamics regressor
`(ID_0.20kg - ID_empty) / 0.20`. Constant-mass runs use plain RLS
(`lambda = 1`).

The regression is causally aligned: torque `tau[k-1]` is paired with
`qdd_meas[k-1] = (qdot[k] - qdot[k-1]) / dt`, evaluated at the stored
`q[k-1], qdot[k-1]`. Estimation uses this raw measured acceleration; control
uses the separate known acceleration command
`qdd_des + Kp*error + Kd*error_rate`. A deterministic synthetic test detects
an intentional one-sample skew.

Tracking results on the same 1.0 s minimum-jerk reference:

| Payload / controller model | Settled arm RMS [rad] | EE miss [m] | Peak torque [N.m] |
|----------------------------|-----------------------|-------------|---------------------|
| 0.10 kg / empty, no estimator | 0.008030 | 0.010438 | 1.640 |
| 0.10 kg / empty + RLS | 0.002494 | 0.000877 | 1.637 |
| 0.20 kg / empty, no estimator | 0.015351 | 0.020944 | 1.917 |
| 0.20 kg / empty + RLS | 0.002545 | 0.000646 | 1.919 |
| 0.20 kg / known-payload upper bound | 0.002840 | 0.000184 | 1.923 |

At 0.20 kg, RLS recovers **102.36% of the steady-state RMS gap** and
**97.77% of the Cartesian miss gap** toward the Phase 4 perfect-model bound.
The RMS recovery slightly exceeds 100% because the remaining Coulomb-friction
deadband differs between trajectories; it is not evidence that estimated
dynamics are more accurate than perfect model knowledge. All adaptive runs
have zero saturated samples and bounded final speed.

Mass-identification diagnostics:

| Plant mass [kg] | Raw final estimate [kg] | Empty-bias corrected [kg] | Corrected error [kg] | Convergence [s] |
|-----------------|-------------------------|---------------------------|----------------------|-----------------|
| 0.00 | -0.009628 | -- | -- | 2.395 |
| 0.10 | 0.093058 | 0.102685 | 0.002685 | 2.735 |
| 0.20 | 0.192716 | 0.202344 | 0.002344 | 1.995 |

The negative empty-arm estimate is the measured friction/damping model-bias
floor. The controller projects the mass used for compensation into the
physical range, so it applies 0 kg compensation on the empty arm while
retaining the signed raw estimate for diagnostics. Subtracting the empty-arm
bias is approximate: friction is velocity-dependent, so common-mode
cancellation relies on the matched reference and similar velocity profiles.
The measured adaptive-to-empty velocity RMS differences are 0.003376 rad/s
(0.10 kg) and 0.005475 rad/s (0.20 kg).

The characterization's worst corrected mass error was 0.002685 kg. The
automated gate is frozen at 0.0035 kg, approximately 30% margin, rather than
being chosen before measurement. `tools/phase5_bench.py` also retains 50% gap
recovery only as a bug-detection floor; the measured recovery above is the
reported result.

## Phase 6 — Momentum disturbance observer

Phase 6 replaces the originally proposed sliding-mode controller with an
industry-aligned generalized-momentum disturbance observer (DOB). The observer
uses the Pinocchio mass matrix, gravity, and the required `C^T qdot` term:

`r = K [p - integral(tau_applied + C^T qdot - g + r) dt]`,
with `p = M qdot`. Under the convention
`M qddot + C qdot + g = tau_command + tau_external`, the residual converges to
external joint torque and the controller applies `tau_model - r`.

The blocking validator passes before any benchmark:

- Exact model/no disturbance: worst residual `8.03e-6 N.m`.
- Known constant joint disturbance: worst estimate error `8.03e-6 N.m`.
- `Mdot - C - C^T` identity: worst error `5.40e-12`.
- MuJoCo force-at-EE vs Pinocchio `J^T f`: worst error `3.33e-16 N.m`.

### Constant horizontal force

A 3 N world-X force begins at 2.5 s. Metrics use separate windows:
pre-onset `[1.5, 2.5)`, onset transient `[2.5, 3.5)`, and post-onset
`t >= 3.5`. The table reports the post-onset window; arm RMS is the mean of
the five per-joint RMS values, matching Phases 4–5.

| Case | Controller / plant | Arm RMS [rad] | EE miss [m] | Peak torque [N.m] |
|------|--------------------|---------------|-------------|---------------------|
| B | empty CT + force | 0.009301 | 0.011339 | 1.360 |
| C | empty CT+DOB + force | 0.000075 | 0.000533 | 1.445 |
| F | adaptive CT, 0.20 kg + force | 0.006961 | 0.007189 | 1.919 |
| G | empty-model CT+DOB, 0.20 kg + force | 0.000079 | 0.000534 | 2.004 |
| H | adaptive CT+DOB, 0.20 kg + force | 0.000072 | 0.000534 | 1.919 |

The DOB reduces constant-force joint RMS by **99.2%** (B→C) and Cartesian miss
by **95.3%**. The stacked controller reduces adaptive-only joint RMS by
**99.0%** (F→H) and Cartesian miss by **92.6%**.

G and H deliberately track almost identically: a live DOB can reject payload
and force as one lumped residual. The stacking advantage is observer authority
and interpretation, not tracking. G's disturbance RMS norm is `1.185729 N.m`;
H's is `0.704628 N.m`, **40.6% lower**, while H retains a physical payload
estimate. In the frozen cross-trajectory test, H also transfers better:
`0.008257 rad` versus G's `0.009323 rad` on the second pose. The large
one-second pose reversal transient saturates both transfer cases, so this is a
reported generalization diagnostic rather than a primary no-saturation gate.

The stacked no-force regression preserves Phase 5 behavior: adaptive CT and
adaptive CT+DOB both measure `0.003158 rad` over the Phase-4-compatible
`t >= 1.5` window, with raw final mass estimates `0.194724` and `0.186587 kg`.
The DOB is rebased while RLS is live, then enabled when the estimate is frozen,
so it cannot starve payload identification.

### Payload/force identifiability

Horizontal force is structurally distinct from payload gravity. On the empty
plant, RLS does not fit it: the raw estimate remains at the empty-model bias
(`-0.008362 kg`) and tracking equals plain CT (case D equals B). This measured
result is more benign than the pre-run prediction that RLS might hit its
projection bound.

The vertical-force aliasing case behaves differently. A 3 N downward force is
partly absorbed as a fictitious `0.150754 kg` payload estimate, demonstrating
why force direction matters and why payload RLS cannot by itself identify
general external contact.

For F/H, freezing `m_hat` at the known benchmark force onset intentionally uses
information a deployed system would not have. The freeze isolates the DOB
contribution by removing the identifiability confound. A deployable version
would need a disturbance detector or persistent-excitation test to decide when
RLS updates are trustworthy.

### Measured observer bandwidth and force reconstruction

The DOB corner is 8 Hz. A 3 N horizontal sinusoid is measured over whole-cycle
post-onset windows at 0.5, 2, and 12 Hz. `tau_external,true = J_v^T f` uses the
world-aligned gripper-frame Jacobian; amplitude and phase come from the direct
estimate-vs-truth complex gain.

| Frequency [Hz] | Theory amplitude / phase | Measured amplitude / phase | CT RMS [rad] | CT+DOB RMS [rad] |
|----------------|--------------------------|----------------------------|--------------|------------------|
| 0.5 | 0.998 / -3.6 deg | 1.102 / -7.8 deg | 0.005947 | 0.000642 |
| 2.0 | 0.970 / -14.0 deg | 0.992 / -24.6 deg | 0.003364 | 0.001345 |
| 12.0 | 0.555 / -56.3 deg | 0.469 / -63.7 deg | 0.000695 | 0.000311 |

The monotonic roll-off validates that the configured gain is a real observer
bandwidth. Absolute CT error is reported because the arm itself attenuates the
12 Hz force; a tracking ratio alone would be ambiguous. Damped-least-squares
Cartesian force reconstruction has RMSE `0.637`, `0.957`, and `1.143 N` at the
three frequencies (Jacobian condition approximately `7.1`). It is useful as a
sensorless contact estimate, but includes velocity-dependent friction and model
residuals, so it is not equivalent to a calibrated force/torque sensor.

Finally, the DOB beats the Phase-4 Coulomb-friction floor without inducing the
anticipated limit cycle: joint RMS falls from the 5 s CT no-force value
`0.003650 rad` to `0.000075 rad` under force rejection, while post-settle joint
peak-to-peak motion falls from `0.001617` to `0.000401 rad` and velocity RMS
from `0.001466` to `0.000356 rad/s`.

`tools/phase6_bench.py` freezes the measured acceptance tolerances and emits
`phase6_frequency_response.csv` plus an SVG plot. The complete Phase 2–6
regression runs through `docker/build_and_validate.sh`.
