"""Phase 1.5 baseline: naive PD joint control of the real SO-101 arm in MuJoCo.

Same experiment as run_baseline.py (a single step-to-setpoint move under a
deliberately naive PD law), but on the actual SO-101 / SO-ARM100 plant: 5 arm
joints plus a gripper, full 3D dynamics, direct torque motors capped at the real
STS3215 envelope. This run records the reference metrics that every later
controller on the SO-101 is measured against.

The 2-link arm (run_baseline.py / arm2.xml) is kept as the documented concept
bootstrap; the SO-101 is the plant from here on.

Usage:
    python run_baseline_so101.py                     # headless, saves plot only
    python run_baseline_so101.py --viewer            # live MuJoCo viewer, real time
    python run_baseline_so101.py --viewer --speed 0.25  # live viewer, quarter speed
"""
import argparse
import os
import time

import numpy as np
import mujoco
import mujoco.viewer
import matplotlib
matplotlib.use("Agg")  # headless: write the plot to a file, no display needed
import matplotlib.pyplot as plt

from src.controller import PID
from src.metrics import rms_error

MODEL_PATH = os.path.join("models", "so101", "scene_torque.xml")
EE_SITE = "gripperframe"          # tip reference between the gripper jaws

# Joint order matches the MJCF: 5 arm joints then the gripper. The gripper is
# held at its home value -- its "tracking" is not part of the arm metric.
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]
N_ARM = 5                         # first N_ARM joints are the arm; last is gripper

# Desired joint angles [rad]. All within range and gravity-holdable at the real
# +/-2.94 N.m torque limit; the gripper is commanded to stay at home (0).
TARGET = np.array([0.6, 0.7, -0.8, 0.5, 0.4, 0.0])

# Naive PD (ki = 0): no gravity/dynamics compensation, on purpose -- it settles
# with a steady offset under gravity, which later model-based phases remove.
KP = np.array([40.0, 40.0, 25.0, 15.0, 8.0, 5.0])
KD = np.array([3.0, 3.0, 2.0, 1.0, 0.6, 0.4])

DURATION = 4.0                    # seconds of simulation
SETTLE_T = 1.0                    # arm is settled after this; steady-state window is t > SETTLE_T


def site_pos(model, data, name):
    """Current world position of a named site."""
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    return data.site_xpos[site_id].copy()


def end_effector_target(model, data, q_des, home_id):
    """End-effector position the arm *should* reach, via forward kinematics.

    Uses a throwaway forward pass at q_des, then restores the home keyframe so
    the caller starts the run from the intended pose.
    """
    data.qpos[:] = q_des
    mujoco.mj_forward(model, data)
    ee = site_pos(model, data, EE_SITE)
    mujoco.mj_resetDataKeyframe(model, data, home_id)
    return ee


def draw_target_marker(viewer, pos):
    """Show a fixed red sphere at the target end-effector position.

    Uses the viewer's user_scn overlay so the target is visible without
    adding anything to the physics model itself.
    """
    viewer.user_scn.ngeom = 1
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.01, 0, 0],
        pos=pos,
        mat=np.eye(3).flatten(),
        rgba=[1.0, 0.0, 0.0, 0.8],
    )


def main(use_viewer=False, speed=1.0):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")

    ee_target = end_effector_target(model, data, TARGET, home_id)

    controller = PID(kp=KP, ki=np.zeros(model.nu), kd=KD, n_joints=model.nu)

    mujoco.mj_resetDataKeyframe(model, data, home_id)  # start from the home pose
    dt = model.opt.timestep
    steps = int(DURATION / dt)

    t_log, q_log, ee_log = [], [], []

    if use_viewer:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            draw_target_marker(viewer, ee_target)
            for _ in range(steps):
                if not viewer.is_running():
                    break
                step_start = time.time()

                q, qdot = data.qpos.copy(), data.qvel.copy()
                data.ctrl[:] = controller(q, qdot, TARGET, dt)
                mujoco.mj_step(model, data)

                t_log.append(data.time)
                q_log.append(q)
                ee_log.append(site_pos(model, data, EE_SITE))

                viewer.sync()
                # pace to real time (scaled by `speed`) so the motion is watchable
                remaining = dt / speed - (time.time() - step_start)
                if remaining > 0:
                    time.sleep(remaining)

            # hold the final pose on screen until the user closes the window
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)
    else:
        for _ in range(steps):
            q, qdot = data.qpos.copy(), data.qvel.copy()
            data.ctrl[:] = controller(q, qdot, TARGET, dt)
            mujoco.mj_step(model, data)

            t_log.append(data.time)
            q_log.append(q)
            ee_log.append(site_pos(model, data, EE_SITE))

    q_log = np.array(q_log)
    ee_log = np.array(ee_log)

    # --- metrics ---
    # The primary number is the STEADY-STATE gravity droop -- that is what the
    # later model-based phases (computed-torque, estimator, SMC) actually remove.
    # A step-to-setpoint RMS over the whole window is dominated by the ~0.5 s rise
    # (i.e. how far each joint travelled), not by the droop, so it is demoted to a
    # secondary "did it reach the setpoint" check. The held gripper is reported
    # apart: its motion is coupling noise, not tracking.
    t_log = np.array(t_log)
    settled = t_log > SETTLE_T

    all_rms = rms_error(q_log, TARGET)                       # per-joint, full window
    full_rms = all_rms[:N_ARM]                               # transit check
    ss_rms = rms_error(q_log[settled], TARGET)[:N_ARM]       # steady-state droop
    final_offset = TARGET[:N_ARM] - q_log[-1, :N_ARM]        # signed final error (sag)
    miss = np.linalg.norm(ee_log[-1] - ee_target)           # final Cartesian miss (Delta)

    print(f"Steady-state droop  [primary, RMS over t > {SETTLE_T:g}s]:")
    for name, ss, fo in zip(JOINT_NAMES[:N_ARM], ss_rms, final_offset):
        print(f"  {name:13s} rms={ss:.4f} rad   final_offset={fo:+.4f} rad")
    print(f"  arm mean        rms={ss_rms.mean():.4f} rad")
    print(f"End-effector miss   [primary, Cartesian]: {miss:.4f} m")
    print()
    print("Full-window RMS     [secondary, transit / reached-setpoint check] [rad]:")
    for name, e in zip(JOINT_NAMES[:N_ARM], full_rms):
        print(f"  {name:13s} {e:.4f}")
    print(f"  arm mean       {full_rms.mean():.4f}")
    print(f"  (gripper, held) {all_rms[N_ARM]:.4f}")

    # --- plot commanded vs actual per joint ---
    fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
    axes = axes.ravel()
    for i, ax in enumerate(axes):
        ax.plot(t_log, q_log[:, i], label="actual")
        ax.axhline(TARGET[i], ls="--", color="k", label="commanded")
        # shade the steady-state window used for the droop metric
        ax.axvspan(SETTLE_T, DURATION, color="0.85", alpha=0.5, lw=0,
                   label=f"steady-state (t>{SETTLE_T:g}s)")
        label = JOINT_NAMES[i] + (" (held)" if i >= N_ARM else "")
        ax.set_ylabel(f"{label} [rad]")
        ax.legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("time [s]")
    axes[-2].set_xlabel("time [s]")
    fig.suptitle("Phase 1.5 baseline - naive PD joint tracking (SO-101, empty)")
    fig.tight_layout()
    fig.savefig("baseline_so101_tracking.png", dpi=120)
    print("Saved plot -> baseline_so101_tracking.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--viewer", action="store_true",
        help="open a live MuJoCo viewer and watch the arm move in real time",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="viewer playback speed multiplier, e.g. 0.25 for slow motion (default: 1.0)",
    )
    args = parser.parse_args()
    main(use_viewer=args.viewer, speed=args.speed)
