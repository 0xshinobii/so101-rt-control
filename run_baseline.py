"""Phase 1 baseline: PID joint control of the 2-link arm in MuJoCo.

Runs one step-to-setpoint move on the empty arm, logs the trajectory, reports
the tracking error, and saves a commanded-vs-actual plot. This is the reference
that every later controller is measured against, so the control law is kept
deliberately naive.

Usage:
    python run_baseline.py
"""
import os
import numpy as np
import mujoco
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


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    ee_target = end_effector_target(model, data, TARGET)

    # PD baseline (ki = 0): no gravity or dynamics compensation, on purpose.
    controller = PID(kp=[200.0, 80.0], ki=[0.0, 0.0], kd=[30.0, 12.0], n_joints=2)

    mujoco.mj_resetData(model, data)  # start from the hanging rest pose
    dt = model.opt.timestep
    steps = int(DURATION / dt)

    t_log, q_log, ee_log = [], [], []
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
    main()
