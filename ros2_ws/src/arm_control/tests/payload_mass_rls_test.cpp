#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/payload_mass_rls.hpp"

namespace {

using arm_control::PayloadMassRlsEstimator;
using arm_control::kDof;

Eigen::VectorXd synthetic_regressor(double q, double qddot) {
  Eigen::VectorXd phi(kDof);
  for (int joint = 0; joint < kDof; ++joint) {
    phi[joint] =
        (0.25 + 0.1 * joint) * (1.0 + 0.7 * q + 0.15 * qddot);
  }
  return phi;
}

}  // namespace

TEST(payload_mass_rls, noiseless_scalar_convergence) {
  PayloadMassRlsEstimator estimator;
  Eigen::VectorXd phi =
      (Eigen::VectorXd(kDof) << 0.2, -0.4, 0.8, 0.3, -0.1, 0.0).finished();
  const Eigen::VectorXd observation = 0.2 * phi;
  for (int i = 0; i < 500; ++i) estimator.update(phi, observation);
  EXPECT_NEAR(estimator.mass(), 0.2, 1e-5);
}

TEST(payload_mass_rls, reset_restores_prior_state) {
  PayloadMassRlsEstimator estimator;
  Eigen::VectorXd phi =
      (Eigen::VectorXd(kDof) << 0.2, -0.4, 0.8, 0.3, -0.1, 0.0).finished();
  estimator.update(phi, 0.2 * phi);
  estimator.reset();
  EXPECT_EQ(estimator.mass(), 0.0);
  EXPECT_EQ(estimator.accepted_updates(), 0u);
}

TEST(payload_mass_rls, low_excitation_is_rejected) {
  PayloadMassRlsEstimator estimator;
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(kDof);
  EXPECT_FALSE(estimator.update(zero, zero));
  EXPECT_EQ(estimator.rejected_updates(), 1u);
}

TEST(payload_mass_rls, mass_projection_preserves_raw_estimate) {
  PayloadMassRlsEstimator::Config bounded_config;
  bounded_config.max_mass = 0.3;
  PayloadMassRlsEstimator bounded(bounded_config);
  Eigen::VectorXd phi =
      (Eigen::VectorXd(kDof) << 0.2, -0.4, 0.8, 0.3, -0.1, 0.0).finished();
  bounded.update(phi, 10.0 * phi);
  EXPECT_EQ(bounded.mass(), 0.3);
  EXPECT_GT(bounded.raw_mass(), 0.3);
}

TEST(payload_mass_rls, affine_calibration_before_projection) {
  PayloadMassRlsEstimator::Config calibrated_config;
  calibrated_config.raw_mass_scale = 2.0;
  calibrated_config.raw_mass_offset = -0.01;
  PayloadMassRlsEstimator calibrated(calibrated_config);

  calibrated.set_raw_mass(0.19);
  EXPECT_NEAR(calibrated.raw_mass(), 0.19, 1e-12);
  EXPECT_NEAR(calibrated.mass(), 0.10, 1e-12);

  calibrated.set_raw_mass(1.19);
  EXPECT_EQ(calibrated.raw_mass(), 1.19);
  EXPECT_EQ(calibrated.mass(), calibrated_config.max_mass);

  calibrated.set_mass(0.20);
  EXPECT_EQ(calibrated.raw_mass(), 0.20);
  EXPECT_EQ(calibrated.mass(), 0.20);
}

TEST(payload_mass_rls, torque_acceleration_alignment) {
  // Synthetic causal sequence: tau[k] is generated from the state at k
  // and the acceleration inferred from qdot[k+1] - qdot[k]. A deliberately
  // shifted regressor must not recover the same mass.
  constexpr double dt = 0.005;
  constexpr double true_mass = 0.17;
  constexpr int samples = 400;
  std::vector<double> q(samples + 1);
  std::vector<double> qdot(samples + 1);
  std::vector<double> qddot(samples);
  std::vector<Eigen::VectorXd> applied_torque;
  applied_torque.reserve(samples);
  q[0] = 0.0;
  qdot[0] = 0.0;
  for (int k = 0; k < samples; ++k) {
    qddot[k] = 8.0 * std::sin(0.07 * k) + 2.0 * std::cos(0.031 * k);
    qdot[k + 1] = qdot[k] + dt * qddot[k];
    q[k + 1] = q[k] + dt * qdot[k];
    applied_torque.push_back(
        true_mass * synthetic_regressor(q[k], qddot[k]));
  }

  PayloadMassRlsEstimator aligned;
  PayloadMassRlsEstimator shifted;
  for (int k = 0; k < samples; ++k) {
    const double measured_qddot = (qdot[k + 1] - qdot[k]) / dt;
    const Eigen::VectorXd aligned_phi =
        synthetic_regressor(q[k], measured_qddot);
    const Eigen::VectorXd shifted_phi =
        synthetic_regressor(q[k + 1],
                            k + 1 < samples ? qddot[k + 1] : qddot[k]);
    aligned.update(aligned_phi, applied_torque[k]);
    shifted.update(shifted_phi, applied_torque[k]);
  }
  const double aligned_error = std::abs(aligned.mass() - true_mass);
  const double shifted_error = std::abs(shifted.mass() - true_mass);
  EXPECT_LT(aligned_error, 1e-6);
  EXPECT_GT(shifted_error, 10.0 * aligned_error);
  EXPECT_GT(shifted_error, 1e-4);
}
