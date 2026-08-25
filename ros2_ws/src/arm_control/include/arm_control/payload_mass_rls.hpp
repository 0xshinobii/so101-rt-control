#pragma once

#include <cstddef>

#include <Eigen/Dense>

namespace arm_control {

// Scalar recursive least-squares estimator for
//   observed_extra_torque = payload_regressor * payload_mass.
// The valid update path performs no allocation or I/O.
class PayloadMassRlsEstimator {
public:
  struct Config {
    double initial_mass = 0.0;
    double initial_covariance = 100.0;
    double forgetting_factor = 1.0;
    double min_mass = 0.0;
    double max_mass = 0.5;
    double excitation_threshold = 1e-8;
  };

  PayloadMassRlsEstimator();
  explicit PayloadMassRlsEstimator(Config config);

  bool update(const Eigen::VectorXd& payload_regressor,
              const Eigen::VectorXd& observed_extra_torque);
  void reset();
  void set_mass(double mass);

  // mass() is the physically projected value used by control. raw_mass()
  // retains the signed identification result so empty-arm model bias remains
  // observable instead of being hidden by the zero lower bound.
  double mass() const { return mass_; }
  double raw_mass() const { return raw_mass_; }
  double covariance() const { return covariance_; }
  std::size_t accepted_updates() const { return accepted_updates_; }
  std::size_t rejected_updates() const { return rejected_updates_; }

private:
  Config config_;
  double mass_ = 0.0;
  double raw_mass_ = 0.0;
  double covariance_ = 0.0;
  std::size_t accepted_updates_ = 0;
  std::size_t rejected_updates_ = 0;
};

}  // namespace arm_control
