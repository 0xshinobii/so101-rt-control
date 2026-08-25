# so101-rt-control

**Payload-adaptive trajectory tracking on a 5-DOF arm, as a real-time C++ / ROS 2
stack — taken from MuJoCo to real hardware, with the sim-to-real gap measured
rather than hand-waved.**

One control loop, written once, runs against two backends behind a single
`PlantInterface`: MuJoCo in simulation, and a ThinkRobotics SO-ARM101 over a
Feetech serial bus. The loop runs on a `SCHED_FIFO` thread under a `PREEMPT_RT`
kernel at 200 Hz. Controllers (PD, computed torque, adaptive computed torque with
online payload estimation) never learn which side of the plant boundary they are
on.

The interesting result is not the tracking-error table. It is that the payload
estimator which recovered **102% of the computed-torque error gap in simulation**
returned **−0.175 kg on hardware** — and that there are four nameable structural
reasons why, none of which are "needs more tuning".

---

## Contents

- [Headline results](#headline-results)
- [Architecture and the real-time boundary](#architecture-and-the-real-time-boundary)
- [Full metrics table](#full-metrics-table)
- [The sim-to-real finding](#the-sim-to-real-finding)
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
| **Computed torque vs PD** (hardware, 151 g payload) | arm RMS excluding wrist-roll **0.024 → 0.012 rad (−50%)**; elbow miss **0.042 → 0.018 rad** |
| **Payload identification** (hardware, static current ID + affine correction) | **+8.0 g (4.4%)** repeat-run error at 181 g, with a **14.6 g (8.1%)** two-run spread |
| **RT wakeup jitter** (PREEMPT_RT, i7-7700, under `stress-ng`) | p99 **6.18 µs**, p99.9 **11.34 µs** on a 5000 µs period — **0.2%** |
| **Serial bus round-trip** (200 Hz, 2000 loops, zero loss) | `sync_read` p99.9 **1.71 ms** + `sync_write` **0.19 ms** = **38%** of the period |

Every number above is measured on this repo's code. Full derivation, discarded
runs, and caveats are in **[RESULTS.md](RESULTS.md)**, which is the source of
truth — this README summarises it.

---

## Architecture and the real-time boundary

![Architecture: where the real-time boundary lives](docs/figures/architecture.svg)

The diagram answers the question worth asking of any motion-control repo: **which
code is allowed to block, and which is not.**

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
Franka or KUKA iiwa. It is a named design point, not a workaround.

### The point of the RT measurement

`PREEMPT_RT` buys a 5–11 µs wakeup on a 5000 µs period. The serial bus spends
**170× more of that budget than the scheduler does.** Measuring both is what
tells you the CPU was never the bottleneck — and that a 1 kHz loop on this arm is
a bus-protocol problem (per-servo Return Delay Time), not a kernel problem.

Regenerate the figure with `python3 tools/plot_architecture.py`.

---

## Full metrics table

### Simulation — MuJoCo SO-101, 200 Hz, 1.0 s minimum-jerk reference

Settled window `t ≥ 1.5 s`; arm RMS is the mean over the five arm joints
(gripper excluded). Actuator limits are the real STS3215 envelope (±2.94 N·m),
never widened to make a controller work.

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

Deltas, each stated against a named reference rather than a floating "baseline":

| Comparison | Reference | Arm RMS | EE miss |
|---|---|---|---|
| Computed torque removes gravity droop, empty arm | 2 vs 1 | **−70.5%** | **−97.7%** |
| Computed torque removes gravity droop, known 0.20 kg | 8 vs 5 | **−86.9%** | **−99.4%** |
| Cost of an unmodelled 0.20 kg payload | 6 vs 2 | **5.2× worse** | **64× worse** |
| RLS recovers it, 0.10 kg | 4 vs 3 | **−68.9%** | **−91.6%** |
| RLS recovers it, 0.20 kg | 7 vs 6 | **−83.4%** | **−96.9%** |
| **RLS gap recovery toward the perfect-model bound, 0.20 kg** | (6−7)/(6−8) | **102.4%** | **97.8%** |

The 102.4% slightly exceeds 100% because the residual Coulomb-friction deadband
differs between trajectories. It is **not** evidence that an estimated model beats
perfect knowledge, and is not presented as such.

Mass identification in simulation:

| Plant mass [kg] | Raw estimate [kg] | Empty-bias corrected [kg] | Error | Convergence [s] |
|---|---|---|---|---|
| 0.00 | `−0.009628` | — | — | 2.395 |
| 0.10 | `0.093058` | `0.102685` | **+2.7 g** | 2.735 |
| 0.20 | `0.192716` | `0.202344` | **+2.3 g** | 1.995 |

The negative empty-arm estimate is the measured friction/damping model-bias floor.
The automated gate is frozen at 3.5 g — *after* measurement, at ~30% margin, not
guessed beforehand.

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

Tracking with a 151 g payload hung from the closed gripper. Same `kTarget`, 1.0 s
minimum-jerk, 4.0 s log, computed torque deliberately using the *empty* URDF:

| Controller | pan | lift | elbow | wrist flex | wrist roll | RMS (5 joints) | RMS (excl. roll) | EE z [m] |
|---|---|---|---|---|---|---|---|---|
| PD via the bridge | +0.005 | −0.021 | **−0.042** | 0.000 | +0.027 | 0.025 | **0.024** | 0.132 |
| Computed torque | +0.006 | −0.013 | **−0.018** | +0.005 | +0.029 | 0.017 | **0.012** | 0.143 |
| Adaptive (`q̈` RLS) | +0.005 | −0.013 | **−0.019** | +0.005 | +0.029 | 0.017 | **0.012** | 0.142 |

Signed final offset `target − q` in rad. Computed torque beats PD by **32%** on
the 5-joint RMS and **50%** with wrist-roll excluded — same data, and the
5-joint figure *understates* the result, because wrist-roll holds a constant
~0.027 rad (≈18 encoder LSB) that no controller moves: `g_roll ≈ 0`, so no gravity
overlay reaches it. That constant is reported, not quietly dropped. A calibration
offset there is not ruled out.

![151 g PD vs CT, elbow](docs/figures/hw_151_elbow.png)

The figure also shows slow PD drift *inside* the nominal settled window (elbow
−0.053 rad at 1.5 s → −0.042 rad at 4.0 s), so PD's RMS is somewhat
window-dependent. The PD–CT gap is larger than the drift, but the drift is stated.

**The hardware problem statement, in one line:** adding 70 g shifts the elbow
error by **0.020 rad** (+0.007 empty → −0.013 loaded) with no payload
compensation. That is what payload adaptation has to close.

Payload mass identification on hardware, by three independent methods:

| Method | Result | vs truth |
|---|---|---|
| Phase 5 `q̈` RLS, ported unchanged from sim | `raw = −0.175 kg`, clamped to 0 | **fails structurally** |
| Static `Present_Current` ID + affine calibration | 181 g repeat → +8.0 g | **+4.4%**, two-run spread 8.1% |
| Tracking-null (independent of current), 70 g | elbow 0.058 kg / lift ~0.155 kg | −17% / +121% |

Affine fit over four masses (0 / 90 / 181 / 273 g):
`m_raw = 2.31217·m_true − 0.01248 kg`, residual standard error **5.62 g**
(`sqrt(SSE/(n−p))`, n=4 p=2), max residual 6.57 g.

---

## The sim-to-real finding

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

Causes 2–3 vanish at rest. Cause 1 does not. Hence the pivot: identify the mass
**statically** from `Present_Current` (register 69) with a ±0.07 rad two-way
approach to cancel Coulomb friction, freeze it, then track. The projection clamp
`m ∈ [0, 0.5]` catching `−0.175 kg` and degrading gracefully to computed-torque-empty
is the safety layer doing exactly its job.

A tracking-error delta tells a reviewer that one controller beat another. A gap
*with a named mechanism*, plus a second independent estimator that brackets truth
within 17%, tells them how the engineer thinks. That is why this section exists.

---

## What this does not claim

Stated here rather than buried, because a portfolio repo that only shows wins is
not evidence of engineering judgement.

- **Only the elbow's `K_servo` is identified** (≈ 11 N·m/rad, from `g(q)`/droop on
  a settled stream). The other five entries `(50, 90, 11, 50, 50, 50)` are frozen
  placeholders. Gravity-compensation headlines therefore rest on the elbow — the
  one joint with a measured stiffness. Wrist-flex carries real gravity torque and
  its overlay is scaled by an arbitrary `K = 50`.
- **Elbow `K ≈ 11` vs lift `K = 90` is physically odd** for nominally identical
  servos, with the elbow carrying *less* gravity torque. Whether all six units
  share one gear ratio was not verified.
- **The affine mass calibration belongs to one stacking geometry.** Jaw opening
  moved monotonically (−0.232 → +0.020 rad) as weights were stacked; its
  correlation with mass is **r = 0.994**. Mass and attachment geometry are nearly
  collinear, so the 2.31× slope conflates instrument scale with grasp geometry.
- **+8.0 g at 181 g is repeatability, not generalization.** It is a repeat run at a
  *fitted* mass. A true held-out test needs a fifth, unseen mass (~130 g).
- **Do not read the 3.97 g in-sample RMSE as an accuracy figure.** When repeat
  spread (14.6 g) exceeds fit residual (5.6 g), the spread is the honest number.
- **The 6.5 mA current LSB and the ~2.5–3 A stall current are datasheet values, not
  measured here.** The claim that the fitted `k_t = −12.5223 N·m/A` is ~10× inflated
  by gearbox friction is therefore a *hypothesis*. One locked-rotor test settles it.
- **Operating envelope:** raw 0.5 kg (the compensation clamp) maps to 0.222 kg true.
  Above ~222 g the controller under-compensates even with a correct raw ID. The fix
  — apply the affine correction *before* the physical-mass clamp — is known and not
  yet applied.
- **No external benchmark is cited.** The measured single-digit-percent result is
  stated as measured. A comparison to commercial or published payload ID would
  require a cited, protocol-matched source, and inventing one would be worse than
  omitting it.
- Sim friction is the MuJoCo Menagerie vendor default (`damping=0.60`,
  `frictionloss=0.052`, `armature=0.028`) — estimates for this class of servo, not
  measurements of this arm.

Open bench work, ranked by value per minute, is listed at the end of
[RESULTS.md](RESULTS.md).

---

## Build and run

Everything runs in one Docker image (ROS 2 Jazzy + Eigen + Pinocchio + the MuJoCo
C library). On bare-metal Linux, Docker is namespacing over the host kernel, so
`SCHED_FIFO` and `mlockall` still work against `PREEMPT_RT`.

```bash
# Simulation: build the image, then build + gate + benchmark end to end.
# Requires the Phase 1.5 oracle CSV first (host, needs the Python deps):
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
#   ros2 run arm_control hardware_run --controller computed_torque --payload-g 151
#   ros2 run arm_control hardware_run --controller adaptive_computed_torque \
#       --payload-g 151 --kt -12.5223      # without --kt the estimator uses k_t = 1.0
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
      plant_interface.hpp     torque in, state out -- the orthogonality spine
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
