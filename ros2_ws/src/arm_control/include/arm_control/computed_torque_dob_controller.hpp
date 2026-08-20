#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Dense>

#include "arm_control/controller.hpp"
#include "arm_control/disturbance_observer.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

namespace arm_control {

class ComputedTorqueDobController : public Controller {
public:
  ComputedTorqueDobController(
      const std::string& urdf_path,
      const Eigen::VectorXd& kp_acceleration,
      const Eigen::VectorXd& kd_acceleration, double timestep,
      double dob_bandwidth_hz = 8.0, double torque_limit = 2.94);

  void compute(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
               const Eigen::VectorXd& q_des,
               const Eigen::VectorXd& qdot_des,
               const Eigen::VectorXd& qddot_des,
               Eigen::VectorXd& tau_out) override;
  void reset() override;

  const Eigen::VectorXd* estimated_disturbance_torque() const override {
    return &observer_.estimate();
  }
  void set_observer_frozen(bool frozen) { observer_.set_frozen(frozen); }
  const Eigen::VectorXd& raw_peak_torque() const { return raw_peak_torque_; }
  const Eigen::VectorXd& applied_peak_torque() const {
    return applied_peak_torque_;
  }
  std::size_t saturated_samples() const { return saturated_samples_; }
  std::size_t sample_count() const { return sample_count_; }

private:
  PinocchioDynamics dynamics_;
  MomentumDisturbanceObserver observer_;
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd qdd_command_;
  Eigen::VectorXd model_torque_;
  Eigen::VectorXd gravity_;
  Eigen::VectorXd coriolis_transpose_qdot_;
  Eigen::VectorXd momentum_;
  Eigen::VectorXd raw_torque_;
  Eigen::VectorXd raw_peak_torque_;
  Eigen::VectorXd applied_peak_torque_;
  Eigen::MatrixXd mass_matrix_;
  Eigen::MatrixXd coriolis_matrix_;
  double torque_limit_;
  std::size_t saturated_samples_ = 0;
  std::size_t sample_count_ = 0;
};

}  // namespace arm_control
