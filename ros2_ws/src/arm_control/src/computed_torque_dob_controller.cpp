#include "arm_control/computed_torque_dob_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_control {

ComputedTorqueDobController::ComputedTorqueDobController(
    const std::string& urdf_path,
    const Eigen::VectorXd& kp_acceleration,
    const Eigen::VectorXd& kd_acceleration, double timestep,
    double dob_bandwidth_hz, double torque_limit)
    : dynamics_(urdf_path),
      observer_(timestep, dob_bandwidth_hz),
      kp_(kp_acceleration),
      kd_(kd_acceleration),
      qdd_command_(Eigen::VectorXd::Zero(kDof)),
      model_torque_(Eigen::VectorXd::Zero(kDof)),
      gravity_(Eigen::VectorXd::Zero(kDof)),
      coriolis_transpose_qdot_(Eigen::VectorXd::Zero(kDof)),
      momentum_(Eigen::VectorXd::Zero(kDof)),
      raw_torque_(Eigen::VectorXd::Zero(kDof)),
      raw_peak_torque_(Eigen::VectorXd::Zero(kDof)),
      applied_peak_torque_(Eigen::VectorXd::Zero(kDof)),
      mass_matrix_(Eigen::MatrixXd::Zero(kDof, kDof)),
      coriolis_matrix_(Eigen::MatrixXd::Zero(kDof, kDof)),
      torque_limit_(torque_limit) {
  if (kp_.size() != kDof || kd_.size() != kDof) {
    throw std::invalid_argument(
        "computed-torque DOB gains must have six entries");
  }
  if (!(torque_limit_ > 0.0)) {
    throw std::invalid_argument("torque limit must be positive");
  }
}

void ComputedTorqueDobController::compute(
    const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& q_des, const Eigen::VectorXd& qdot_des,
    const Eigen::VectorXd& qddot_des, Eigen::VectorXd& tau_out) {
  dynamics_.mass_matrix(q, mass_matrix_);
  dynamics_.coriolis_matrix(q, qdot, coriolis_matrix_);
  dynamics_.gravity(q, gravity_);
  momentum_.noalias() = mass_matrix_ * qdot;
  coriolis_transpose_qdot_.noalias() = coriolis_matrix_.transpose() * qdot;
  const Eigen::VectorXd& disturbance = observer_.update(
      momentum_, coriolis_transpose_qdot_, gravity_);

  qdd_command_.array() =
      qddot_des.array() + kp_.array() * (q_des - q).array() +
      kd_.array() * (qdot_des - qdot).array();
  dynamics_.inverse_dynamics(q, qdot, qdd_command_, model_torque_);

  // r estimates external torque in Mqdd+Cqdot+g=tau_cmd+r, so reject it.
  raw_torque_.noalias() = model_torque_ - disturbance;
  bool saturated = false;
  for (int i = 0; i < kDof; ++i) {
    const double raw = raw_torque_[i];
    const double applied = std::clamp(raw, -torque_limit_, torque_limit_);
    tau_out[i] = applied;
    raw_peak_torque_[i] = std::max(raw_peak_torque_[i], std::abs(raw));
    applied_peak_torque_[i] =
        std::max(applied_peak_torque_[i], std::abs(applied));
    saturated |= std::abs(raw) > torque_limit_;
  }
  observer_.set_applied_torque(tau_out);
  ++sample_count_;
  if (saturated) ++saturated_samples_;
}

void ComputedTorqueDobController::reset() {
  observer_.reset();
  qdd_command_.setZero();
  model_torque_.setZero();
  gravity_.setZero();
  coriolis_transpose_qdot_.setZero();
  momentum_.setZero();
  raw_torque_.setZero();
  raw_peak_torque_.setZero();
  applied_peak_torque_.setZero();
  mass_matrix_.setZero();
  coriolis_matrix_.setZero();
  saturated_samples_ = 0;
  sample_count_ = 0;
}

}  // namespace arm_control
