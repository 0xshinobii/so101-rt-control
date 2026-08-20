#include <cmath>
#include <cstdio>
#include <string>

#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/disturbance_observer.hpp"
#include "arm_control/mujoco_backend.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

namespace {

using arm_control::MomentumDisturbanceObserver;
using arm_control::MujocoBackend;
using arm_control::PinocchioDynamics;
using arm_control::kDof;

bool expect(bool condition, const char* name, double value = 0.0) {
  std::printf("  %s %-48s %.6e\n", condition ? "PASS" : "FAIL",
              name, value);
  return condition;
}

void trajectory(double t, Eigen::VectorXd& q, Eigen::VectorXd& qdot,
                Eigen::VectorXd& qddot) {
  for (int i = 0; i < kDof; ++i) {
    const double amplitude = 0.08 + 0.01 * i;
    const double omega = 0.7 + 0.08 * i;
    const double phase = 0.2 * i;
    q[i] = 0.1 * (i - 2) + amplitude * std::sin(omega * t + phase);
    qdot[i] = amplitude * omega * std::cos(omega * t + phase);
    qddot[i] =
        -amplitude * omega * omega * std::sin(omega * t + phase);
  }
}

double run_observer(PinocchioDynamics& dynamics,
                    const Eigen::VectorXd& external_torque,
                    Eigen::VectorXd& final_estimate) {
  constexpr double dt = 0.005;
  MomentumDisturbanceObserver observer(dt, 8.0);
  Eigen::VectorXd q(kDof), qdot(kDof), qddot(kDof);
  Eigen::VectorXd gravity(kDof), c_transpose_qdot(kDof);
  Eigen::VectorXd momentum(kDof), inverse_dynamics(kDof);
  Eigen::VectorXd applied(kDof);
  Eigen::MatrixXd mass(kDof, kDof), coriolis(kDof, kDof);

  for (int step = 0; step < 2400; ++step) {
    const double t = step * dt;
    trajectory(t, q, qdot, qddot);
    dynamics.mass_matrix(q, mass);
    dynamics.coriolis_matrix(q, qdot, coriolis);
    dynamics.gravity(q, gravity);
    dynamics.inverse_dynamics(q, qdot, qddot, inverse_dynamics);
    momentum.noalias() = mass * qdot;
    c_transpose_qdot.noalias() = coriolis.transpose() * qdot;
    observer.update(momentum, c_transpose_qdot, gravity);
    applied.noalias() = inverse_dynamics - external_torque;
    observer.set_applied_torque(applied);
  }
  final_estimate = observer.estimate();
  return (final_estimate - external_torque).lpNorm<Eigen::Infinity>();
}

}  // namespace

int main(int argc, char** argv) {
  const std::string urdf =
      argc > 1 ? argv[1] : "models/so101/so101_dynamics.urdf";
  PinocchioDynamics dynamics(urdf);
  bool ok = true;

  Eigen::VectorXd estimate(kDof);
  const double null_error =
      run_observer(dynamics, Eigen::VectorXd::Zero(kDof), estimate);
  ok &= expect(null_error < 1e-4, "exact-model residual converges to zero",
               null_error);

  const Eigen::VectorXd known =
      (Eigen::VectorXd(kDof) << 0.12, -0.08, 0.05, 0.03, -0.02, 0.0)
          .finished();
  const double known_error = run_observer(dynamics, known, estimate);
  ok &= expect(known_error < 1e-4,
               "known external joint torque is recovered", known_error);

  Eigen::VectorXd q(kDof), qdot(kDof), qddot(kDof);
  trajectory(1.7, q, qdot, qddot);
  Eigen::MatrixXd mass_plus(kDof, kDof), mass_minus(kDof, kDof);
  Eigen::MatrixXd coriolis(kDof, kDof);
  constexpr double epsilon = 1e-6;
  dynamics.mass_matrix(q + epsilon * qdot, mass_plus);
  dynamics.mass_matrix(q - epsilon * qdot, mass_minus);
  dynamics.coriolis_matrix(q, qdot, coriolis);
  const Eigen::MatrixXd identity_error =
      (mass_plus - mass_minus) / (2.0 * epsilon) -
      coriolis - coriolis.transpose();
  const double skew_error = identity_error.cwiseAbs().maxCoeff();
  ok &= expect(skew_error < 1e-9,
               "Mdot - C - C^T identity", skew_error);

  if (argc > 2) {
    MujocoBackend plant(argv[2]);
    Eigen::VectorXd q_home = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd qdot_home = Eigen::VectorXd::Zero(kDof);
    plant.read_state(q_home, qdot_home);
    const Eigen::Vector3d force(1.3, -0.7, 0.4);
    plant.set_ee_force_world(force);
    Eigen::VectorXd mujoco_generalized(kDof);
    plant.applied_generalized_force(mujoco_generalized);
    Eigen::Matrix<double, 3, kDof> jacobian;
    dynamics.frame_translation_jacobian(
        q_home, "gripper_frame_joint", jacobian);
    const Eigen::VectorXd pinocchio_generalized =
        jacobian.transpose() * force;
    const double force_map_error =
        (mujoco_generalized - pinocchio_generalized)
            .lpNorm<Eigen::Infinity>();
    ok &= expect(force_map_error < 1e-9,
                 "MuJoCo EE force equals Pinocchio J^T f",
                 force_map_error);
  }

  std::printf("\nPHASE 6 DOB VALIDATOR: %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
