// Naive per-joint PD (the Phase 1.5 baseline). No gravity/dynamics
// compensation, so it settles with a steady offset under load -- the droop that
// later model-based controllers remove.
#pragma once
#include "arm_control/controller.hpp"

namespace arm_control {

class PdController : public Controller {
public:
  PdController(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd);

  // tau = kp .* (q_des - q) + kd .* (qdot_des - qdot)
  // For the Phase 1.5 step baseline qdot_des=0, exactly reproducing its
  // derivative-on-measurement law.
  void compute(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
               const Eigen::VectorXd& q_des,
               const Eigen::VectorXd& qdot_des,
               const Eigen::VectorXd& qddot_des,
               Eigen::VectorXd& tau_out) override;

  void reset() override {}  // stateless (no integral term in the naive PD)

private:
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
};

}  // namespace arm_control
