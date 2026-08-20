#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Dense>

#include "arm_control/controller.hpp"
#include "arm_control/disturbance_observer.hpp"
#include "arm_control/payload_mass_rls.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

namespace arm_control {

class AdaptiveComputedTorqueDobController : public Controller {
public:
  AdaptiveComputedTorqueDobController(
      const std::string& empty_urdf_path,
      const std::string& reference_payload_urdf_path,
      double reference_payload_mass,
      const Eigen::VectorXd& kp_acceleration,
      const Eigen::VectorXd& kd_acceleration, double timestep,
      PayloadMassRlsEstimator::Config estimator_config,
      double dob_bandwidth_hz = 8.0, double torque_limit = 2.94);

  void compute(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
               const Eigen::VectorXd& q_des,
               const Eigen::VectorXd& qdot_des,
               const Eigen::VectorXd& qddot_des,
               Eigen::VectorXd& tau_out) override;
  void reset() override;

  double estimated_payload_mass() const override {
    return estimator_.raw_mass();
  }
  const Eigen::VectorXd* estimated_disturbance_torque() const override {
    return &observer_.estimate();
  }
  double compensated_payload_mass() const { return estimator_.mass(); }
  double estimator_covariance() const { return estimator_.covariance(); }
  std::size_t estimator_updates() const {
    return estimator_.accepted_updates();
  }
  void set_estimator_frozen(bool frozen);
  void set_observer_frozen(bool frozen) { observer_.set_frozen(frozen); }
  const Eigen::VectorXd& raw_peak_torque() const { return raw_peak_torque_; }
  const Eigen::VectorXd& applied_peak_torque() const {
    return applied_peak_torque_;
  }
  std::size_t saturated_samples() const { return saturated_samples_; }
  std::size_t sample_count() const { return sample_count_; }

private:
  void validate_reference_models() const;
  void payload_aware_observer_terms(const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& qdot);

  PinocchioDynamics empty_dynamics_;
  PinocchioDynamics reference_payload_dynamics_;
  PayloadMassRlsEstimator estimator_;
  MomentumDisturbanceObserver observer_;
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd qdd_command_;
  Eigen::VectorXd qdd_measured_;
  Eigen::VectorXd empty_command_torque_;
  Eigen::VectorXd payload_command_torque_;
  Eigen::VectorXd command_regressor_;
  Eigen::VectorXd empty_measured_torque_;
  Eigen::VectorXd payload_measured_torque_;
  Eigen::VectorXd measured_regressor_;
  Eigen::VectorXd observed_extra_torque_;
  Eigen::VectorXd previous_base_torque_;
  Eigen::VectorXd gravity_empty_;
  Eigen::VectorXd gravity_payload_;
  Eigen::VectorXd gravity_model_;
  Eigen::VectorXd coriolis_transpose_qdot_;
  Eigen::VectorXd momentum_;
  Eigen::VectorXd raw_torque_;
  Eigen::VectorXd raw_peak_torque_;
  Eigen::VectorXd applied_peak_torque_;
  Eigen::VectorXd previous_q_;
  Eigen::VectorXd previous_qdot_;
  Eigen::MatrixXd mass_empty_;
  Eigen::MatrixXd mass_payload_;
  Eigen::MatrixXd mass_model_;
  Eigen::MatrixXd coriolis_empty_;
  Eigen::MatrixXd coriolis_payload_;
  Eigen::MatrixXd coriolis_model_;
  double reference_payload_mass_;
  double inverse_reference_payload_mass_;
  double timestep_;
  double torque_limit_;
  bool have_previous_sample_ = false;
  bool estimator_frozen_ = false;
  std::size_t saturated_samples_ = 0;
  std::size_t sample_count_ = 0;
};

}  // namespace arm_control
