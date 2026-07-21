"""Phase 1 baseline: PID joint control of the 2-link arm in MuJoCo.

Runs one step-to-setpoint move on the empty arm, logs the trajectory, reports
the tracking error, and saves a commanded-vs-actual plot. This is the reference
that every later controller is measured against, so the control law is kept
deliberately naive.

Usage:
    python run_baseline.py                        # headless, saves plot only
    python run_baseline.py --viewer                # live MuJoCo viewer, real time
    python run_baseline.py --viewer --speed 0.25    # live viewer, quarter speed
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

MODEL_PATH = os.path.join("models", "arm2.xml")
TARGET = np.array([0.8, -0.6])   # desired joint angles [rad] for j1, j2
DURATION = 4.0                    # seconds of simulation


def site_pos(model, data, name):
    """Current world position of a named site."""
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    return data.site_xpos[site_id].copy()


def end_effector_target(model, data, q_des):
    """End-effector position the arm *should* reach, via forward kinematics.

    Uses a throwaway forward pass at q_des; the caller resets the state after.
    """
    data.qpos[:] = q_des
    mujoco.mj_forward(model, data)
    return site_pos(model, data, "ee")


def draw_target_marker(viewer, pos):
    """Show a fixed red sphere at the target end-effector position.

    Uses the viewer's user_scn overlay so the target is visible without
    adding anything to the physics model itself.
    """
    viewer.user_scn.ngeom = 1
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.03, 0, 0],
        pos=pos,
        mat=np.eye(3).flatten(),
        rgba=[1.0, 0.0, 0.0, 0.8],
    )


def main(use_viewer=False, speed=1.0):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    ee_target = end_effector_target(model, data, TARGET)

    # PD baseline (ki = 0): no gravity or dynamics compensation, on purpose.
    controller = PID(kp=[200.0, 80.0], ki=[0.0, 0.0], kd=[30.0, 12.0], n_joints=2)

    mujoco.mj_resetData(model, data)  # start from the hanging rest pose
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
                ee_log.append(site_pos(model, data, "ee"))

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
            ee_log.append(site_pos(model, data, "ee"))

    q_log = np.array(q_log)
    ee_log = np.array(ee_log)

    # --- metrics ---
    joint_rms = rms_error(q_log, TARGET)
    miss = np.linalg.norm(ee_log[-1] - ee_target)  # final Cartesian miss (Delta)
    print(f"Joint RMS error [rad]: j1={joint_rms[0]:.4f}, j2={joint_rms[1]:.4f}")
    print(f"End-effector miss [m]: {miss:.4f}")

    # --- plot commanded vs actual per joint ---
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t_log, q_log[:, i], label="actual")
        ax.axhline(TARGET[i], ls="--", color="k", label="commanded")
        ax.set_ylabel(f"joint {i + 1} [rad]")
        ax.legend(loc="best")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Phase 1 baseline - PID joint tracking (empty arm)")
    fig.tight_layout()
    fig.savefig("baseline_tracking.png", dpi=120)
    print("Saved plot -> baseline_tracking.png")


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
