#include "arm_control/pd_controller.hpp"

namespace arm_control {

PdController::PdController(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd)
    : kp_(kp), kd_(kd) {}

void PdController::compute(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                           const Eigen::VectorXd& q_des,
                           const Eigen::VectorXd& qdot_des,
                           const Eigen::VectorXd& /*qddot_des*/,
                           Eigen::VectorXd& tau_out) {
  // Coefficient-wise via .array(); assigning into the pre-sized tau_out reuses
  // its storage (no heap allocation on the hot path).
  tau_out.array() = kp_.array() * (q_des - q).array() +
                    kd_.array() * (qdot_des - qdot).array();
}

}  // namespace arm_control
