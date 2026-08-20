#pragma once

#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"

namespace arm_control {

// Generalized-momentum residual observer:
//   p_dot = tau_applied + tau_external + C(q,qdot)^T qdot - g(q)
// The residual converges to tau_external without differentiating qdot.
class MomentumDisturbanceObserver {
public:
  MomentumDisturbanceObserver(double timestep, double bandwidth_hz);

  // Update the estimate at the current measured state. The supplied model
  // terms must all come from the same inertial model.
  const Eigen::VectorXd& update(
      const Eigen::VectorXd& momentum,
      const Eigen::VectorXd& coriolis_transpose_qdot,
      const Eigen::VectorXd& gravity);

  // Store the torque actually applied during the next plant interval.
  void set_applied_torque(const Eigen::VectorXd& applied_torque);
  void set_frozen(bool frozen) { frozen_ = frozen; }
  bool frozen() const { return frozen_; }
  void reset();

  const Eigen::VectorXd& estimate() const { return residual_; }
  double gain() const { return gain_; }

private:
  double timestep_;
  double gain_;
  bool initialized_ = false;
  bool frozen_ = false;
  Eigen::VectorXd beta_;
  Eigen::VectorXd residual_;
  Eigen::VectorXd previous_applied_torque_;
  Eigen::VectorXd previous_coriolis_transpose_qdot_;
  Eigen::VectorXd previous_gravity_;
};

}  // namespace arm_control
