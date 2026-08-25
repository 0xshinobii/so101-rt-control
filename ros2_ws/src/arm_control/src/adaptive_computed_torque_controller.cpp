#include "arm_control/adaptive_computed_torque_controller.hpp"

#include "arm_control/arm_types.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_control {

AdaptiveComputedTorqueController::AdaptiveComputedTorqueController(
    const std::string& empty_urdf_path,
    const std::string& reference_payload_urdf_path,
    double reference_payload_mass, const Eigen::VectorXd& kp_acceleration,
    const Eigen::VectorXd& kd_acceleration, double timestep,
    PayloadMassRlsEstimator::Config estimator_config, double torque_limit)
    : empty_dynamics_(empty_urdf_path),
      reference_payload_dynamics_(reference_payload_urdf_path),
      estimator_(estimator_config),
      kp_(kp_acceleration),
      kd_(kd_acceleration),
      qdd_command_(Eigen::VectorXd::Zero(kDof)),
      qdd_measured_(Eigen::VectorXd::Zero(kDof)),
      empty_command_torque_(Eigen::VectorXd::Zero(kDof)),
      payload_command_torque_(Eigen::VectorXd::Zero(kDof)),
      command_regressor_(Eigen::VectorXd::Zero(kDof)),
      empty_measured_torque_(Eigen::VectorXd::Zero(kDof)),
      payload_measured_torque_(Eigen::VectorXd::Zero(kDof)),
      measured_regressor_(Eigen::VectorXd::Zero(kDof)),
      observed_extra_torque_(Eigen::VectorXd::Zero(kDof)),
      raw_torque_(Eigen::VectorXd::Zero(kDof)),
      raw_peak_torque_(Eigen::VectorXd::Zero(kDof)),
      applied_peak_torque_(Eigen::VectorXd::Zero(kDof)),
      previous_q_(Eigen::VectorXd::Zero(kDof)),
      previous_qdot_(Eigen::VectorXd::Zero(kDof)),
      previous_applied_torque_(Eigen::VectorXd::Zero(kDof)),
      reference_payload_mass_(reference_payload_mass),
      inverse_reference_payload_mass_(
          reference_payload_mass > 0.0 ? 1.0 / reference_payload_mass : 0.0),
      timestep_(timestep),
      torque_limit_(torque_limit) {
  if (kp_.size() != kDof || kd_.size() != kDof) {
    throw std::invalid_argument(
        "adaptive computed-torque gains must have six entries");
  }
  if (!(reference_payload_mass_ > 0.0)) {
    throw std::invalid_argument("reference payload mass must be positive");
  }
  if (!(timestep_ > 0.0)) {
    throw std::invalid_argument("controller timestep must be positive");
  }
  if (!(torque_limit_ > 0.0)) {
    throw std::invalid_argument("torque limit must be positive");
  }
  validate_reference_models();
}

void AdaptiveComputedTorqueController::validate_reference_models() const {
  const pinocchio::Model& empty = empty_dynamics_.model();
  const pinocchio::Model& payload = reference_payload_dynamics_.model();
  if (empty.nq != payload.nq || empty.nv != payload.nv ||
      empty.njoints != payload.njoints || empty.names != payload.names ||
      empty.parents != payload.parents) {
    throw std::invalid_argument(
        "reference URDF must have the same articulated structure as empty URDF");
  }

  const pinocchio::JointIndex payload_parent = empty.getJointId("wrist_roll");
  for (pinocchio::JointIndex joint = 1; joint < empty.njoints; ++joint) {
    if (!empty.jointPlacements[joint].isApprox(
            payload.jointPlacements[joint], 1e-12)) {
      throw std::invalid_argument(
          "reference URDF changes a non-payload joint placement");
    }
    const bool inertia_matches =
        empty.inertias[joint].isApprox(payload.inertias[joint], 1e-12);
    if (joint == payload_parent) {
      if (inertia_matches) {
        throw std::invalid_argument(
            "reference URDF does not add payload inertia at wrist_roll");
      }
    } else if (!inertia_matches) {
      throw std::invalid_argument(
          "reference URDF changes inertia outside the payload parent");
    }
  }
}

void AdaptiveComputedTorqueController::compute(
    const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& q_des, const Eigen::VectorXd& qdot_des,
    const Eigen::VectorXd& qddot_des, Eigen::VectorXd& tau_out) {
  // tau[k-1] produced qdot[k] - qdot[k-1]. Evaluate the measured inverse
  // dynamics and regressor at the stored k-1 state to preserve that alignment.
  if (use_accel_rls_ && have_previous_sample_) {
    qdd_measured_.array() =
        (qdot - previous_qdot_).array() / timestep_;
    empty_dynamics_.inverse_dynamics(previous_q_, previous_qdot_,
                                     qdd_measured_,
                                     empty_measured_torque_);
    reference_payload_dynamics_.inverse_dynamics(
        previous_q_, previous_qdot_, qdd_measured_, payload_measured_torque_);
    measured_regressor_.array() =
        (payload_measured_torque_ - empty_measured_torque_).array() *
        inverse_reference_payload_mass_;
    observed_extra_torque_.array() =
        previous_applied_torque_.array() - empty_measured_torque_.array();
    estimator_.update(measured_regressor_, observed_extra_torque_);
  }

  // Compensation uses the known acceleration command, never measured
  // acceleration. This retains the Phase 4 computed-torque feedback shaping.
  qdd_command_.array() =
      qddot_des.array() + kp_.array() * (q_des - q).array() +
      kd_.array() * (qdot_des - qdot).array();
  empty_dynamics_.inverse_dynamics(q, qdot, qdd_command_,
                                   empty_command_torque_);
  reference_payload_dynamics_.inverse_dynamics(
      q, qdot, qdd_command_, payload_command_torque_);
  command_regressor_.array() =
      (payload_command_torque_ - empty_command_torque_).array() *
      inverse_reference_payload_mass_;
  raw_torque_.array() =
      empty_command_torque_.array() +
      estimator_.mass() * command_regressor_.array();

  bool saturated = false;
  for (int i = 0; i < kDof; ++i) {
    const double raw = raw_torque_[i];
    const double applied = std::clamp(raw, -torque_limit_, torque_limit_);
    tau_out[i] = applied;
    raw_peak_torque_[i] = std::max(raw_peak_torque_[i], std::abs(raw));
    applied_peak_torque_[i] =
        std::max(applied_peak_torque_[i], std::abs(applied));
    previous_applied_torque_[i] = applied;
    saturated |= std::abs(raw) > torque_limit_;
  }
  previous_q_ = q;
  previous_qdot_ = qdot;
  have_previous_sample_ = true;
  ++sample_count_;
  if (saturated) ++saturated_samples_;
}

void AdaptiveComputedTorqueController::update_from_measured_torque(
    const Eigen::VectorXd& q, const Eigen::VectorXd& tau_meas) {
  if (q.size() != kDof || tau_meas.size() != kDof) {
    throw std::invalid_argument("static-gravity update size");
  }
  empty_dynamics_.gravity(q, empty_measured_torque_);
  reference_payload_dynamics_.gravity(q, payload_measured_torque_);
  measured_regressor_.array() =
      (payload_measured_torque_ - empty_measured_torque_).array() *
      inverse_reference_payload_mass_;
  observed_extra_torque_.array() =
      tau_meas.array() - empty_measured_torque_.array();
  measured_regressor_[5] = 0.0;
  observed_extra_torque_[5] = 0.0;
  estimator_.update(measured_regressor_, observed_extra_torque_);
}

double AdaptiveComputedTorqueController::estimate_static_mass(
    const Eigen::VectorXd& q, const Eigen::VectorXd& tau_meas) {
  if (q.size() != kDof || tau_meas.size() != kDof) {
    throw std::invalid_argument("static-gravity estimate size");
  }
  empty_dynamics_.gravity(q, empty_measured_torque_);
  reference_payload_dynamics_.gravity(q, payload_measured_torque_);
  measured_regressor_.array() =
      (payload_measured_torque_ - empty_measured_torque_).array() *
      inverse_reference_payload_mass_;
  observed_extra_torque_.array() =
      tau_meas.array() - empty_measured_torque_.array();
  measured_regressor_[5] = 0.0;
  observed_extra_torque_[5] = 0.0;
  const double den = measured_regressor_.squaredNorm();
  if (den < 1e-8) {
    throw std::runtime_error("gravity regressor ~0 at this pose");
  }
  return measured_regressor_.dot(observed_extra_torque_) / den;
}

void AdaptiveComputedTorqueController::add_static_observation(
    const Eigen::VectorXd& q, const Eigen::VectorXd& tau_meas, double& num,
    double& den) {
  const double m = estimate_static_mass(q, tau_meas);
  const double d = measured_regressor_.squaredNorm();
  num += m * d;
  den += d;
}

void AdaptiveComputedTorqueController::reset() {
  estimator_.reset();
  qdd_command_.setZero();
  qdd_measured_.setZero();
  empty_command_torque_.setZero();
  payload_command_torque_.setZero();
  command_regressor_.setZero();
  empty_measured_torque_.setZero();
  payload_measured_torque_.setZero();
  measured_regressor_.setZero();
  observed_extra_torque_.setZero();
  raw_torque_.setZero();
  raw_peak_torque_.setZero();
  applied_peak_torque_.setZero();
  previous_q_.setZero();
  previous_qdot_.setZero();
  previous_applied_torque_.setZero();
  have_previous_sample_ = false;
  saturated_samples_ = 0;
  sample_count_ = 0;
}

}  // namespace arm_control
