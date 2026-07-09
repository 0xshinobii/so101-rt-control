"""Joint-space controllers for the arm.

Phase 1 uses a plain PID with the integral gain left at zero (i.e. a PD law) as
the naive baseline. It has no knowledge of the arm's dynamics or of gravity, so
it settles with a steady offset under load -- exactly the behaviour later,
model-based controllers are meant to remove. The integral term is kept in the
class so the same code serves future phases without a rewrite.
"""
import numpy as np


class PID:
    """Per-joint PID controller.

    Gains are stored as arrays (one entry per joint) so the whole control law is
    a single vectorised expression -- no per-joint branching to keep in sync.
    """

    def __init__(self, kp, ki, kd, n_joints):
        self.kp = np.asarray(kp, dtype=float)
        self.ki = np.asarray(ki, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self._integral = np.zeros(n_joints)

    def reset(self):
        """Clear the integral accumulator between runs."""
        self._integral[:] = 0.0

    def __call__(self, q, qdot, q_des, dt):
        """Return the joint torques that drive q towards q_des.

        q, qdot : current joint positions and velocities
        q_des   : desired joint positions (the setpoint)
        dt      : timestep, used to accumulate the integral term
        """
        error = q_des - q
        self._integral += error * dt
        # Derivative acts on the measured velocity, not on the error, so a step
        # change in the setpoint does not cause a derivative spike.
        return self.kp * error + self.ki * self._integral - self.kd * qdot
