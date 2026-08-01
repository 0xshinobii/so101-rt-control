// The plant abstraction: the arm, whether simulated or real hardware. Swap the
// backend and controllers stay unchanged -- the orthogonality spine. Torque in,
// state out; the backend is responsible for realizing the torque (MuJoCo applies
// it directly; a future hardware backend runs the torque->position bridge).
#pragma once
#include <Eigen/Dense>

namespace arm_control {

class PlantInterface {
public:
  virtual ~PlantInterface() = default;

  // Read current joint positions and velocities into caller-owned buffers
  // (pre-sized to dof(), so no allocation on the hot path).
  virtual void read_state(Eigen::VectorXd& q, Eigen::VectorXd& qdot) = 0;

  // Command joint torques; the backend realizes them (sim: write d->ctrl).
  virtual void apply_torque(const Eigen::VectorXd& tau) = 0;

  // Advance one control period (sim: mj_step; hardware: wait for the period).
  virtual void step() = 0;

  // End-effector world position (sim: gripperframe site; Pinocchio FK later).
  virtual Eigen::Vector3d ee_position() = 0;

  // Restore the initial/home pose.
  virtual void reset() = 0;

  virtual int dof() const = 0;
  virtual double timestep() const = 0;  // control period dt [s]
  virtual double time() const = 0;      // sim time [s] (post-step)
};

}  // namespace arm_control
