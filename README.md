# so101-rt-control

**Payload-adaptive trajectory tracking on a 5-DOF arm, as a real-time C++ / ROS 2
stack — taken from MuJoCo to real hardware, with the sim-to-real gap measured.**

One control loop, written once, runs against two backends behind a single
`PlantInterface`: MuJoCo in simulation, and a ThinkRobotics SO-ARM101 over a
Feetech serial bus. The loop runs on a `SCHED_FIFO` thread under a `PREEMPT_RT`
kernel at 200 Hz. Controllers (PD, computed torque, adaptive computed torque with
online payload estimation) never learn which side of the plant boundary they are
on.

The sim-to-real gap is the substance of the project. The online payload estimator
that recovers **102% of the computed-torque error gap in simulation** recovers
**almost none of it on hardware** — 0.8 g of a 90 g payload, 9.4 g of a 180 g one,
after 799 updates — for four structural reasons that no amount of tuning removes. The hardware path is therefore a different estimator: identify the mass
statically from motor current at rest, calibrate the instrument, freeze it, then
track. Applying that correction inside
the controller cuts the elbow's steady-state offset by **78–83%** and the arm RMS
excluding wrist-roll by **49–55%** versus the same static ID without calibration.

---

## Contents

- [Headline results](#headline-results)
- [Architecture and the real-time boundary](#architecture-and-the-real-time-boundary)
- [Full metrics table](#full-metrics-table)
- [The sim-to-real finding, and what closed it](#the-sim-to-real-finding-and-what-closed-it)
- [What this does not claim](#what-this-does-not-claim)
- [Build and run](#build-and-run)
- [Repository layout](#repository-layout)
- [License](#license)

---

## Headline results

| | Result |
|---|---|
| **Computed torque vs naive PD** (sim, 0.20 kg known payload) | settled arm RMS **0.0216 → 0.0028 rad (−86.9%)**; end-effector miss **0.0300 → 0.00018 m (−99.4%)** |
| **Online payload RLS** (sim, 0.20 kg unknown) | recovers **102.4%** of the steady-state RMS gap and **97.8%** of the Cartesian gap toward the perfect-model bound; mass error **2.3 g** |
| **Computed torque vs PD** (hardware, 90 / 180 g) | arm RMS excluding wrist-roll **0.0268 → 0.0093** and **0.0356 → 0.0177 rad** (**−65% / −50%**) |
| **Payload compensation** (hardware, calibrated static ID vs raw static ID) | elbow offset **0.0237 → 0.0053 rad (−78%)** at 90 g and **0.0406 → 0.0069 rad (−83%)** at 180 g; arm RMS excluding wrist-roll **−49% / −55%** |
| **Payload identification** (hardware, static current ID + affine correction) | final campaign mass error **−7.2 g** at 90 g and **−4.0 g** at 180 g; observed repeat spread **7.8 g / 14.6 g** |
| **RT wakeup jitter** (PREEMPT_RT, i7-7700, under `stress-ng`) | p99 **6.18 µs**, p99.9 **11.34 µs** on a 5000 µs period — **0.2%** |
| **Serial bus round-trip** (200 Hz, 2000 loops, zero loss) | `sync_read` p99.9 **1.71 ms** + `sync_write` **0.19 ms** = **38%** of the period |

Every number above is measured on this repo's code. Full derivation, discarded
runs, and caveats are in **[RESULTS.md](RESULTS.md)**, which is the source of
truth — this README summarises it.

---

## Architecture and the real-time boundary

![Architecture: where the real-time boundary lives](docs/figures/architecture.svg)

The diagram shows **which code is allowed to block, and which is not.**

**Above the line** — the `rclcpp` executor thread. It may allocate, log, take
locks and publish. It runs a 20 ms wall timer that drains telemetry and publishes
`/joint_states` and `/arm_metrics`. Parameters are read once, at construction.

**The line itself** — a lock-free single-producer/single-consumer ring
(`SpscRing<Sample>`, 1024 slots). The RT thread's `push()` is wait-free and
*drops* the sample when the consumer falls behind. Telemetry is best-effort;
control is not. This queue is the only thing that crosses.

**Below the line** — a dedicated `std::thread` with `SCHED_FIFO` 80,
`mlockall(MCL_CURRENT|MCL_FUTURE)`, `/dev/cpu_dma_latency = 0`, and an optional
CPU pin. Its period comes from `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`
— an absolute deadline, so a late wakeup does not accumulate. `ControlLoop::step_once()`
works entirely from pre-allocated Eigen buffers: no heap traffic, no I/O, no
`rclcpp`.

**Below that** — `PlantInterface`, one abstraction with two backends. MuJoCo
writes torque straight to `d->ctrl`. On hardware the STS3215 has no closed-loop
N·m mode, so torque is realized admittance-style rather than commanded:

```
q_cmd = q_des + clamp(τ / K_servo, ±0.12 rad)
```

That is the same realization strategy Universal Robots and most position-controlled
industrial arms use, as opposed to the impedance-style direct torque command of a
Franka or KUKA iiwa.

### What the RT measurement shows

`PREEMPT_RT` buys a 5–11 µs wakeup on a 5000 µs period. The serial bus spends
**170× more of that budget than the scheduler does.** Measuring both locates the
bottleneck: not the CPU, and so a 1 kHz loop on this arm is a bus-protocol
problem (per-servo Return Delay Time) rather than a kernel one.

Regenerate the figure with `python3 tools/plot_architecture.py`.

---

## Full metrics table

### Simulation — MuJoCo SO-101, 200 Hz, 1.0 s minimum-jerk reference

Settled window `t ≥ 1.5 s`; arm RMS is the mean over the five arm joints
(gripper excluded). Actuator limits are the real STS3215 envelope (±2.94 N·m),
not widened to make a controller work.

| # | Payload | Controller | Model knows payload? | Settled arm RMS [rad] | EE miss [m] | Peak τ [N·m] |
|---|---|---|---|---|---|---|
| 1 | empty | naive PD | — | `0.009959` | `0.014012` | — |
| 2 | empty | computed torque | n/a | `0.002933` | `0.000327` | 1.360 |
| 3 | 0.10 kg | computed torque | **no** | `0.008030` | `0.010438` | 1.640 |
| 4 | 0.10 kg | CT + payload RLS | estimated | `0.002494` | `0.000877` | 1.637 |
| 5 | 0.20 kg | naive PD | — | `0.021612` | `0.029999` | — |
| 6 | 0.20 kg | computed torque | **no** | `0.015351` | `0.020944` | 1.917 |
| 7 | 0.20 kg | CT + payload RLS | estimated | `0.002545` | `0.000646` | 1.919 |
| 8 | 0.20 kg | computed torque | **yes** (upper bound) | `0.002840` | `0.000184` | 1.923 |

Deltas, each against a named reference row:

| Comparison | Reference | Arm RMS | EE miss |
|---|---|---|---|
| Computed torque removes gravity droop, empty arm | 2 vs 1 | **−70.5%** | **−97.7%** |
| Computed torque removes gravity droop, known 0.20 kg | 8 vs 5 | **−86.9%** | **−99.4%** |
| Cost of an unmodelled 0.20 kg payload | 6 vs 2 | **5.2× worse** | **64× worse** |
| RLS recovers it, 0.10 kg | 4 vs 3 | **−68.9%** | **−91.6%** |
| RLS recovers it, 0.20 kg | 7 vs 6 | **−83.4%** | **−96.9%** |
| **RLS gap recovery toward the perfect-model bound, 0.20 kg** | (6−7)/(6−8) | **102.4%** | **97.8%** |

The 102.4% slightly exceeds 100% because the residual Coulomb-friction deadband
differs between trajectories. It is not evidence that an estimated model beats
perfect knowledge.

Mass identification in simulation:

| Plant mass [kg] | Raw estimate [kg] | Empty-bias corrected [kg] | Error | Convergence [s] |
|---|---|---|---|---|
| 0.00 | `−0.009628` | — | — | 2.395 |
| 0.10 | `0.093058` | `0.102685` | **+2.7 g** | 2.735 |
| 0.20 | `0.192716` | `0.202344` | **+2.3 g** | 1.995 |

The negative empty-arm estimate is the measured friction/damping model-bias floor.
The automated gate is frozen at 3.5 g, chosen after measurement at ~30% margin
rather than before it.

**Blocking gate before any of the above runs:** rigid-body equivalence between the
MuJoCo plant and the Pinocchio controller model, over five poses and two nonzero
velocity vectors. Worst disagreement — gravity `1.33e-15 N·m`, nonlinear bias
`8.66e-13 N·m`, mass matrix `5.64e-12`. If that gate fails, the benchmark does not
execute.

### Real-time — PREEMPT_RT, i7-7700 / Ubuntu 26.04 / kernel `7.0.0-30-realtime`

`ubuntu-realtime` 1.1.3 from `resolute/main`; no Ubuntu Pro subscription required.

| Bench | Load | n | Min | Avg | p99 | p99.9 | Max |
|---|---|---|---|---|---|---|---|
| `cyclictest -p 80` | generic kernel, idle | 20k | 2 | 2 | — | — | 4 µs |
| `cyclictest -p 80` | realtime, idle | 20k | 2 | 2 | — | — | 6 µs |
| `cyclictest -p 80` | realtime, `stress-ng` | 60k | 2 | 2 | — | — | 27 µs |
| `rt_jitter_bench` (in-repo) | idle | 20k | 2.20 | 2.43 | 2.88 | 3.34 | **6.89 µs** |
| `rt_jitter_bench` (in-repo) | `stress-ng` | 60k | 2.22 | 3.38 | **6.18** | **11.34** | 438 µs |

The operational numbers are the loaded percentiles. Loaded Max 438 µs occurs on
0.07% of ticks and still meets the 5 ms period — **and `cyclictest` did not
reproduce it under the same load, so an application-side cause in `rt_jitter_bench`
is not excluded.** An earlier bench built on `std::this_thread::sleep_until`
measured 477 µs idle max and was discarded: libstdc++ may wait with a *relative*
`nanosleep`, which picks up timer slack. That was never a kernel number.

![PREEMPT_RT wakeup jitter](docs/figures/rt_jitter.png)

### Hardware — SO-ARM101, 6× STS3215 @ 200 Hz

Bus round-trip over 2000 loops, zero packet loss:

| | p99.9 | share of the 5 ms period |
|---|---|---|
| `sync_read` | 1.71 ms | 34% |
| `sync_write` | 0.19 ms | 4% |
| **together** | **1.90 ms** | **38%** |

Read is ~9× write — more than packet size alone explains. Per-servo Return Delay
Time (register 7) is the likely dominant term; six servos at the ~250 µs default is
close to the observed 1.71 ms. **Not yet verified**, and it is the single change
most likely to put 1 kHz in reach.

All hardware tracking numbers below come from a single campaign at **two
payloads, 90 g and 180 g**, with five controllers run at each. Every run uses the
same `kTarget`, the same 1.0 s minimum-jerk reference, a 4.0 s log at 200 Hz and
the same grasp. Offsets are signed `target − q` at `t = 4.0 s`; RMS is over the
settled window `t ≥ 1.5 s` and excludes wrist-roll.

Wrist-roll is excluded because it holds a constant ~0.027 rad (≈18 encoder LSB)
that no controller moves — `g_roll ≈ 0`, so no gravity overlay reaches it, and
including it dilutes every result by the same amount. A calibration offset there
is not ruled out.

#### Identifying the payload

| Method | Result | vs truth |
|---|---|---|
| Online `q̈` RLS, ported unchanged from sim | 0.0008 kg at 90 g; 0.0094 kg at 180 g | **fails; tracks like empty-model CT** |
| Static `Present_Current` ID, **raw** | 0.181 kg at 90 g; 0.400 kg at 180 g | **+91 g / +220 g** |
| Static `Present_Current` ID, **affine-corrected** | 0.0828 kg at 90 g; 0.1760 kg at 180 g | **−7.2 g / −4.0 g** |

Affine fit over four masses (0 / 90 / 180 / 273 g):
`m_raw = 2.31501·m_true − 0.01229 kg`, so `m_cal = (m_raw + 0.01229)/2.31501`.
Residual standard error **5.03 g** (`sqrt(SSE/(n−p))`, n=4 p=2), max residual
5.86 g. The correction lives in `PayloadMassRlsEstimator::calibrate_and_clamp()`
and is applied *before* the physical `[0, 0.5] kg` projection, which also removes
the artificial ~221 g compensation ceiling the raw path had.

**The spread bounds this, not the residual.** Near each fitted mass the run-to-run
variation is larger than any single error: 90 g readings span **7.8 g**,
and 180 g readings span **14.6 g**. The −7.2 g and −4.0 g above are
single draws from that distribution, not evidence of 2% accuracy.

#### What the correction buys in closed loop

| mass | controller | mass used [kg] | lift offset [rad] | elbow offset [rad] | arm RMS excl. roll [rad] | EE z [m] |
|---|---|---:|---:|---:|---:|---:|
| 90 g | PD | — | −0.0133 | −0.0514 | 0.0268 | 0.132 |
| 90 g | CT, empty model | 0 | −0.0087 | −0.0146 | 0.0093 | 0.145 |
| 90 g | static ID, raw | 0.1813 | −0.0010 | +0.0237 | 0.0127 | 0.159 |
| 90 g | **static ID, affine** | **0.0828** | −0.0041 | **+0.0053** | **0.0064** | 0.153 |
| 90 g | motion `q̈` RLS | 0.0008 | −0.0087 | −0.0161 | 0.0100 | 0.145 |
| 180 g | PD | — | −0.0240 | −0.0668 | 0.0356 | 0.124 |
| 180 g | CT, empty model | 0 | −0.0179 | −0.0299 | 0.0177 | 0.137 |
| 180 g | static ID, raw | 0.4002 | −0.0072 | +0.0406 | 0.0215 | 0.162 |
| 180 g | **static ID, affine** | **0.1760** | −0.0164 | **+0.0069** | **0.0097** | 0.148 |
| 180 g | motion `q̈` RLS | 0.0094 | −0.0179 | −0.0299 | 0.0178 | 0.137 |

![Elbow tracking error, 90 g and 180 g](docs/figures/final_campaign_elbow.svg)

Four things the table and figure show together:

- **Gravity compensation is most of the win.** PD → computed torque with the
  empty model cuts arm RMS **65%** at 90 g and **50%** at 180 g, before any
  payload estimation at all.
- **An uncalibrated payload estimate is worse than none.** Raw static ID reads
  0.181 kg for a 90 g payload, over-compensates, and ends *further* from target
  than empty-model CT (0.0127 vs 0.0093 rad). A biased mass in an otherwise
  correct model actively hurts.
- **The affine correction is what makes the estimate useful.** Against raw static
  ID it cuts elbow offset **78% / 83%** and arm RMS **49% / 55%**; against
  empty-model CT it wins by **31% / 45%**.
- **Motion RLS estimates ≈ 0 and reproduces empty-model CT** to three digits at
  both masses — the projection clamp absorbing an unusable estimate instead of
  injecting it.

Logs and per-run console output: `docs/data/final_campaign/`. Regenerate the
table with `python3 tools/final_campaign_metrics.py`, the figure with
`python3 tools/plot_final_campaign.py`.

**Lift gets worse under the correction, and that is the expected sign.** The corrected
mass is *smaller* than the raw one, so the gravity overlay shrinks — elbow stops
overshooting, and lift sags further. Lift's `K_servo = 90` is a placeholder, so
its error-versus-mass slope is arbitrary and the raw over-estimate had been
masking that. The improvement concentrates on the elbow, which is the one joint
with an identified stiffness — the sign and the location both follow from the
model.

---

## The sim-to-real finding, and what closed it

`τ_applied − ID_empty(q, q̇, q̈) = Φ(q, q̇, q̈)·m` is a valid identity on a torque
plant. On this servo bus it is false, for four independent reasons:

1. **`τ` is not plant torque.** It is a Goal_Position overlay; the STS3215's own
   inner PD produces the torque that actually holds the arm. Structural — it does
   not vanish at rest.
2. **`q̈` is a double difference of a 12-bit encoder.** One LSB at 200 Hz is
   `q̇ = 0.307 rad/s` and `q̈ = 61.4 rad/s²`, against a true min-jerk peak near
   4.6 rad/s². Noise is ~13× signal — and it sits in the *regressor*, so it biases
   systematically rather than averaging out (errors-in-variables).
3. **Fake inertia swamps gravity.** Pinocchio armature 0.028 × 61.4 = 1.72 N·m per
   LSB — already 2.1× the empty lift gravity torque, with the wrong sign, 799 times.
4. **Friction floor scaled ~18×** versus simulation, from the unmodelled 345:1
   gearbox.

The observed failure is not always the same shape — an earlier campaign drove the
estimate negative, this one leaves it pinned near zero — but the cause is: the
regressor carries no usable payload information at this encoder resolution.

Causes 2–3 vanish at rest. Cause 1 does not. Hence the pivot: identify the mass
**statically** from `Present_Current` (register 69) with a ±0.07 rad two-way
approach to cancel Coulomb friction, freeze it, then track. Whatever the online
estimator does produce is passed through a projection clamp `m ∈ [0, 0.5]`, so an
unusable estimate degrades to computed-torque-empty rather than being injected
into the gravity compensation — which is exactly what the motion-RLS rows in the
table above show happening.

That pivot is now closed end to end. The static estimate is a raw instrument
reading with a 2.31× scale, so it is affine-corrected inside
`PayloadMassRlsEstimator` before the physical projection, and the corrected mass
is what the controller compensates with. On hardware that cuts the elbow's
steady-state offset by 78–83% and the arm RMS (excluding wrist-roll) by 49–55%
against the identical run using the raw estimate.

---

## What this does not claim

- **Only the elbow's `K_servo` is identified** (≈ 11 N·m/rad, from `g(q)`/droop on
  a settled stream). The other five entries `(50, 90, 11, 50, 50, 50)` are frozen
  placeholders. The gravity-compensation results therefore rest on the elbow — the
  one joint with a measured stiffness. Wrist-flex carries real gravity torque and
  its overlay is scaled by an arbitrary `K = 50`.
- **Elbow `K ≈ 11` vs lift `K = 90` is physically odd** for nominally identical
  servos, with the elbow carrying *less* gravity torque. Whether all six units
  share one gear ratio was not verified.
- **The affine mass calibration belongs to one stacking geometry.** Jaw opening
  moved monotonically (−0.232 → +0.020 rad) as weights were stacked; its
  correlation with mass is **r = 0.994**. Mass and attachment geometry are nearly
  collinear, so the 2.31× slope conflates instrument scale with grasp geometry.
- **Every mass tested so far is a fitted mass.** 90 g and 180 g are fit points,
  so all of it is repeatability, not generalization. A true held-out test needs
  an unseen mass (~130 g, between fit points). Until then the affine map is not
  known to interpolate.
- **Do not read the 3.56 g in-sample RMSE, or any single run's error, as an
  accuracy figure.** Run-to-run spread (7.8 g at 90 g, 14.6 g at 180 g) is the
  practical bound; residual standard error is 5.03 g.
- **The closed-loop improvement is an elbow result.** Lift regresses in both
  calibrated runs; the aggregate improves because elbow and wrist-flex dominate.
  Only the elbow has an identified `K_servo`, so only the elbow's error-versus-mass
  slope is physically meaningful.
- **`n = 2` on the closed-loop claim.** One calibrated run at each of two masses,
  each against one uncalibrated run. The effect is large and the sign is predicted
  by the model, but this is not a repeated experiment.
- **The 6.5 mA current LSB and the ~2.5–3 A stall current are datasheet values, not
  measured here.** The claim that the fitted `k_t = −12.5223 N·m/A` is ~10× inflated
  by gearbox friction is therefore a *hypothesis*. One locked-rotor test settles it.
- **The ~221 g compensation ceiling is gone, for this geometry only.** The affine
  correction now runs before the `[0, 0.5] kg` projection, so raw estimates above
  0.5 kg no longer clip. That relies on the same 2.31× map, and therefore on the
  same stacking geometry.
- **No external benchmark is cited.** The measured single-digit-percent result is
  stated as measured. A comparison to commercial or published payload ID would
  require a cited, protocol-matched source.
- Sim friction is the MuJoCo Menagerie vendor default (`damping=0.60`,
  `frictionloss=0.052`, `armature=0.028`) — estimates for this class of servo, not
  measurements of this arm.

Remaining bench work is listed at the end of [RESULTS.md](RESULTS.md).

---

## Build and run

Everything runs in one Docker image (ROS 2 Jazzy + Eigen + Pinocchio + the MuJoCo
C library). On bare-metal Linux, Docker is namespacing over the host kernel, so
`SCHED_FIFO` and `mlockall` still work against `PREEMPT_RT`.

```bash
# Simulation: build the image, then build + gate + benchmark end to end.
# Requires the reference baseline CSV first (host, needs the Python deps):
python3 run_baseline_so101.py --csv oracle_baseline_so101.csv
./docker/build_and_validate.sh
```

`build_and_validate.sh` runs, in order: the rigid-body equivalence gates, the C++
vs Python PD regression, the computed-torque matrix and its acceptance checks, and
the deterministic RLS validator plus the unknown-payload matrix. Any gate failing
stops the run.

```bash
# ROS 2 node (simulation), publishing /joint_states and /arm_metrics.
# SCHED_FIFO needs CAP_SYS_NICE.
ros2 launch arm_bringup arm_control.launch.py

# Hardware, on the RT box with the arm on /dev/ttyACM0:
./docker/run_i7.sh
#   then, inside:
#   ros2 run arm_control calibrate_so101   # once; writes so101_follower_calib.json
#   ros2 run arm_control hardware_run --controller computed_torque --payload-g 180
#   ros2 run arm_control hardware_run --controller adaptive_computed_torque \
#       --payload-g 180 --kt -12.5223 \
#       --mass-cal-scale 2.31501 --mass-cal-offset -0.01229
#   (without --kt it falls back to k_t = 1.0; without the --mass-cal flags it
#    uses the raw, ~2.3x biased reading)
```

Standalone benches:

```bash
ros2 run arm_control rt_jitter_bench      # PREEMPT_RT wakeup jitter
ros2 run arm_control bus_timing           # sync_read / sync_write RTT
ros2 run arm_control validate_dynamics    # MuJoCo vs Pinocchio equivalence gate
ros2 run arm_control gravity_id           # static Present_Current mass ID
```

**Hardware note.** Streaming Goal_Position at 200 Hz requires `acc = 0, speed = 0`.
A non-zero acceleration restarts the servo's internal ramp on every rewrite and the
arm crawls (~0.06 rad/s at `speed = 40`).

---

## Repository layout

```
ros2_ws/src/
  arm_control/          the stack
    include/arm_control/
      plant_interface.hpp     torque in, state out -- sim and hardware both implement it
      controller.hpp          one signature for PD / CT / adaptive CT
      control_loop.hpp        RT-clean iteration: no alloc, no I/O
      rt_thread.hpp           SCHED_FIFO, mlockall, cpu_dma_latency
      spsc_ring.hpp           lock-free RT -> non-RT telemetry handoff
      pinocchio_dynamics.hpp  the controller's model (never the plant's)
      payload_mass_rls.hpp    scalar RLS with a projection clamp
      mujoco_backend.hpp      simulation plant
      hardware_backend.hpp    STS3215 bus + torque-to-position bridge
      feetech_bus.hpp         sync_read / sync_write protocol
    src/
      arm_control_node.cpp    ROS 2 wrapper; control thread never touches rclcpp
      hardware_run.cpp        hardware tracking, in-run mass ID, logging
      validate_dynamics.cpp   blocking rigid-body equivalence gate
      validate_payload_estimator.cpp   deterministic RLS validator
      rt_jitter_bench.cpp     wakeup-jitter measurement
      bus_timing.cpp          bus RTT measurement
      calibrate_so101.cpp     per-joint sign + home offset, checked against FK
  arm_bringup/          launch + params.yaml
  arm_msgs/             ArmMetrics

models/so101/           MuJoCo Menagerie SO-101 (Apache-2.0) + torque variants
docs/data/              hardware CSV logs cited by RESULTS.md
docs/figures/           figures, with the scripts that regenerate them
tools/                  benchmark and plotting scripts (Python)
docker/                 image + end-to-end validation script
RESULTS.md              canonical numbers -- the source of truth
```

---

## License

This repository is MIT licensed — see [LICENSE](LICENSE).

`models/so101/` is third-party: the SO-101 robot description from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), Apache-2.0,
with its original `LICENSE`, `README.md` and `CHANGELOG.md` preserved in place. See
[THIRD_PARTY.md](THIRD_PARTY.md).
