#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "arm_control/controller.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

namespace arm_control {

class ComputedTorqueController : public Controller {
public:
  ComputedTorqueController(const std::string& urdf_path,
                           const Eigen::VectorXd& kp_acceleration,
                           const Eigen::VectorXd& kd_acceleration,
                           double torque_limit = 2.94);

  void compute(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
               const Eigen::VectorXd& q_des,
               const Eigen::VectorXd& qdot_des,
               const Eigen::VectorXd& qddot_des,
               Eigen::VectorXd& tau_out) override;
  void reset() override;

  const Eigen::VectorXd& raw_peak_torque() const { return raw_peak_torque_; }
  const Eigen::VectorXd& applied_peak_torque() const {
    return applied_peak_torque_;
  }
  std::size_t saturated_samples() const { return saturated_samples_; }
  std::size_t sample_count() const { return sample_count_; }

private:
  PinocchioDynamics dynamics_;
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd qdd_command_;
  Eigen::VectorXd raw_torque_;
  Eigen::VectorXd raw_peak_torque_;
  Eigen::VectorXd applied_peak_torque_;
  double torque_limit_;
  std::size_t saturated_samples_ = 0;
  std::size_t sample_count_ = 0;
};

}  // namespace arm_control
