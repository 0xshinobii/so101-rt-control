#include <cmath>
#include <cstdio>
#include <vector>

#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/payload_mass_rls.hpp"

namespace {

using arm_control::PayloadMassRlsEstimator;
using arm_control::kDof;

bool expect(bool condition, const char* name) {
  std::printf("  %s %s\n", condition ? "PASS" : "FAIL", name);
  return condition;
}

Eigen::VectorXd synthetic_regressor(double q, double qddot) {
  Eigen::VectorXd phi(kDof);
  for (int joint = 0; joint < kDof; ++joint) {
    phi[joint] =
        (0.25 + 0.1 * joint) * (1.0 + 0.7 * q + 0.15 * qddot);
  }
  return phi;
}

}  // namespace

int main() {
  bool ok = true;
  PayloadMassRlsEstimator estimator;
  Eigen::VectorXd phi =
      (Eigen::VectorXd(kDof) << 0.2, -0.4, 0.8, 0.3, -0.1, 0.0).finished();
  Eigen::VectorXd observation = 0.2 * phi;
  for (int i = 0; i < 500; ++i) estimator.update(phi, observation);
  ok &= expect(std::abs(estimator.mass() - 0.2) < 1e-5,
               "noiseless scalar convergence");

  estimator.reset();
  ok &= expect(estimator.mass() == 0.0 &&
                   estimator.accepted_updates() == 0,
               "reset restores prior state");
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(kDof);
  ok &= expect(!estimator.update(zero, zero) &&
                   estimator.rejected_updates() == 1,
               "low excitation is rejected");

  PayloadMassRlsEstimator::Config bounded_config;
  bounded_config.max_mass = 0.3;
  PayloadMassRlsEstimator bounded(bounded_config);
  bounded.update(phi, 10.0 * phi);
  ok &= expect(bounded.mass() == 0.3 && bounded.raw_mass() > 0.3,
               "mass projection preserves raw diagnostic estimate");

  // Synthetic causal sequence: tau[k-1] is generated from the state at k-1
  // and the acceleration inferred from qdot[k] - qdot[k-1]. A deliberately
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
  ok &= expect(aligned_error < 1e-6, "k-1 torque/acceleration alignment");
  ok &= expect(shifted_error > 10.0 * aligned_error &&
                   shifted_error > 1e-4,
               "intentional off-by-one sequence is detectably biased");

  std::printf("\nPAYLOAD ESTIMATOR VALIDATION: %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
