#include "arm_control/disturbance_observer.hpp"

#include <cmath>
#include <stdexcept>

namespace arm_control {

MomentumDisturbanceObserver::MomentumDisturbanceObserver(
    double timestep, double bandwidth_hz)
    : timestep_(timestep),
      gain_(2.0 * std::acos(-1.0) * bandwidth_hz),
      beta_(Eigen::VectorXd::Zero(kDof)),
      residual_(Eigen::VectorXd::Zero(kDof)),
      previous_applied_torque_(Eigen::VectorXd::Zero(kDof)),
      previous_coriolis_transpose_qdot_(Eigen::VectorXd::Zero(kDof)),
      previous_gravity_(Eigen::VectorXd::Zero(kDof)) {
  if (!(std::isfinite(timestep_) && timestep_ > 0.0)) {
    throw std::invalid_argument("DOB timestep must be finite and positive");
  }
  if (!(std::isfinite(bandwidth_hz) && bandwidth_hz > 0.0)) {
    throw std::invalid_argument("DOB bandwidth must be finite and positive");
  }
  if (gain_ * timestep_ >= 1.0) {
    throw std::invalid_argument(
        "DOB forward-Euler gain must satisfy 2*pi*bandwidth*dt < 1");
  }
}

const Eigen::VectorXd& MomentumDisturbanceObserver::update(
    const Eigen::VectorXd& momentum,
    const Eigen::VectorXd& coriolis_transpose_qdot,
    const Eigen::VectorXd& gravity) {
  if (momentum.size() != kDof ||
      coriolis_transpose_qdot.size() != kDof ||
      gravity.size() != kDof) {
    throw std::invalid_argument("DOB model terms must have six entries");
  }

  if (!initialized_) {
    beta_ = momentum;
    previous_coriolis_transpose_qdot_ = coriolis_transpose_qdot;
    previous_gravity_ = gravity;
    residual_.setZero();
    initialized_ = true;
    return residual_;
  }

  if (!frozen_) {
    beta_.noalias() +=
        timestep_ *
        (previous_applied_torque_ +
         previous_coriolis_transpose_qdot_ - previous_gravity_ + residual_);
    residual_.noalias() = gain_ * (momentum - beta_);
  }
  previous_coriolis_transpose_qdot_ = coriolis_transpose_qdot;
  previous_gravity_ = gravity;
  return residual_;
}

void MomentumDisturbanceObserver::set_applied_torque(
    const Eigen::VectorXd& applied_torque) {
  if (applied_torque.size() != kDof) {
    throw std::invalid_argument("DOB applied torque must have six entries");
  }
  previous_applied_torque_ = applied_torque;
}

void MomentumDisturbanceObserver::reset() {
  initialized_ = false;
  frozen_ = false;
  beta_.setZero();
  residual_.setZero();
  previous_applied_torque_.setZero();
  previous_coriolis_transpose_qdot_.setZero();
  previous_gravity_.setZero();
}

}  // namespace arm_control
