#include "arm_control/payload_mass_rls.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_control {

PayloadMassRlsEstimator::PayloadMassRlsEstimator()
    : PayloadMassRlsEstimator(Config{}) {}

PayloadMassRlsEstimator::PayloadMassRlsEstimator(Config config)
    : config_(config) {
  if (!(config_.initial_covariance > 0.0)) {
    throw std::invalid_argument("RLS initial covariance must be positive");
  }
  if (!(config_.forgetting_factor > 0.0) ||
      config_.forgetting_factor > 1.0) {
    throw std::invalid_argument("RLS forgetting factor must be in (0, 1]");
  }
  if (config_.min_mass > config_.max_mass) {
    throw std::invalid_argument("RLS mass bounds are inverted");
  }
  if (config_.initial_mass < config_.min_mass ||
      config_.initial_mass > config_.max_mass) {
    throw std::invalid_argument("RLS initial mass is outside its bounds");
  }
  if (config_.excitation_threshold < 0.0) {
    throw std::invalid_argument("RLS excitation threshold must be non-negative");
  }
  reset();
}

bool PayloadMassRlsEstimator::update(
    const Eigen::VectorXd& payload_regressor,
    const Eigen::VectorXd& observed_extra_torque) {
  if (payload_regressor.size() != observed_extra_torque.size()) {
    throw std::invalid_argument("RLS regressor and observation sizes differ");
  }

  double regressor_norm_squared = 0.0;
  double projected_innovation = 0.0;
  for (Eigen::Index i = 0; i < payload_regressor.size(); ++i) {
    const double phi = payload_regressor[i];
    const double observation = observed_extra_torque[i];
    if (!std::isfinite(phi) || !std::isfinite(observation)) {
      ++rejected_updates_;
      return false;
    }
    regressor_norm_squared += phi * phi;
    projected_innovation += phi * (observation - phi * raw_mass_);
  }

  if (regressor_norm_squared < config_.excitation_threshold) {
    ++rejected_updates_;
    return false;
  }

  const double denominator =
      config_.forgetting_factor + covariance_ * regressor_norm_squared;
  raw_mass_ += (covariance_ / denominator) * projected_innovation;
  mass_ = std::clamp(raw_mass_, config_.min_mass, config_.max_mass);
  // Information (inverse covariance) adds as
  //   1 / P_new = lambda / P + phi^T phi.
  // Inverting gives P_new = P / (lambda + P * phi^T phi), whose
  // denominator is the same value used by the mass update above.
  covariance_ /= denominator;
  ++accepted_updates_;
  return true;
}

void PayloadMassRlsEstimator::reset() {
  raw_mass_ = config_.initial_mass;
  mass_ = raw_mass_;
  covariance_ = config_.initial_covariance;
  accepted_updates_ = 0;
  rejected_updates_ = 0;
}

void PayloadMassRlsEstimator::set_mass(double mass) {
  raw_mass_ = mass;
  mass_ = std::clamp(raw_mass_, config_.min_mass, config_.max_mass);
}

}  // namespace arm_control
