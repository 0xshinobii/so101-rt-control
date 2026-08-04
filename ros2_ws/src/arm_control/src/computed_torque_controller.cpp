#include "arm_control/computed_torque_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_control {

ComputedTorqueController::ComputedTorqueController(
    const std::string& urdf_path, const Eigen::VectorXd& kp_acceleration,
    const Eigen::VectorXd& kd_acceleration, double torque_limit)
    : dynamics_(urdf_path),
      kp_(kp_acceleration),
      kd_(kd_acceleration),
      qdd_command_(Eigen::VectorXd::Zero(kDof)),
      raw_torque_(Eigen::VectorXd::Zero(kDof)),
      raw_peak_torque_(Eigen::VectorXd::Zero(kDof)),
      applied_peak_torque_(Eigen::VectorXd::Zero(kDof)),
      torque_limit_(torque_limit) {
  if (kp_.size() != kDof || kd_.size() != kDof) {
    throw std::invalid_argument("computed-torque gains must have six entries");
  }
  if (!(torque_limit_ > 0.0)) {
    throw std::invalid_argument("torque limit must be positive");
  }
}

void ComputedTorqueController::compute(const Eigen::VectorXd& q,
                                       const Eigen::VectorXd& qdot,
                                       const Eigen::VectorXd& q_des,
                                       const Eigen::VectorXd& qdot_des,
                                       const Eigen::VectorXd& qddot_des,
                                       Eigen::VectorXd& tau_out) {
  // The PD-shaped term is an acceleration command; RNEA maps it through
  // M(q), C(q,qdot), and G(q).
  qdd_command_.array() =
      qddot_des.array() + kp_.array() * (q_des - q).array() +
      kd_.array() * (qdot_des - qdot).array();
  dynamics_.inverse_dynamics(q, qdot, qdd_command_, raw_torque_);

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
  ++sample_count_;
  if (saturated) ++saturated_samples_;
}

void ComputedTorqueController::reset() {
  qdd_command_.setZero();
  raw_torque_.setZero();
  raw_peak_torque_.setZero();
  applied_peak_torque_.setZero();
  saturated_samples_ = 0;
  sample_count_ = 0;
}

}  // namespace arm_control
