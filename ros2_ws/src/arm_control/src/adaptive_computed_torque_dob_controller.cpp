#include "arm_control/adaptive_computed_torque_dob_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_control {

AdaptiveComputedTorqueDobController::AdaptiveComputedTorqueDobController(
    const std::string& empty_urdf_path,
    const std::string& reference_payload_urdf_path,
    double reference_payload_mass,
    const Eigen::VectorXd& kp_acceleration,
    const Eigen::VectorXd& kd_acceleration, double timestep,
    PayloadMassRlsEstimator::Config estimator_config,
    double dob_bandwidth_hz, double torque_limit)
    : empty_dynamics_(empty_urdf_path),
      reference_payload_dynamics_(reference_payload_urdf_path),
      estimator_(estimator_config),
      observer_(timestep, dob_bandwidth_hz),
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
      previous_base_torque_(Eigen::VectorXd::Zero(kDof)),
      gravity_empty_(Eigen::VectorXd::Zero(kDof)),
      gravity_payload_(Eigen::VectorXd::Zero(kDof)),
      gravity_model_(Eigen::VectorXd::Zero(kDof)),
      coriolis_transpose_qdot_(Eigen::VectorXd::Zero(kDof)),
      momentum_(Eigen::VectorXd::Zero(kDof)),
      raw_torque_(Eigen::VectorXd::Zero(kDof)),
      raw_peak_torque_(Eigen::VectorXd::Zero(kDof)),
      applied_peak_torque_(Eigen::VectorXd::Zero(kDof)),
      previous_q_(Eigen::VectorXd::Zero(kDof)),
      previous_qdot_(Eigen::VectorXd::Zero(kDof)),
      mass_empty_(Eigen::MatrixXd::Zero(kDof, kDof)),
      mass_payload_(Eigen::MatrixXd::Zero(kDof, kDof)),
      mass_model_(Eigen::MatrixXd::Zero(kDof, kDof)),
      coriolis_empty_(Eigen::MatrixXd::Zero(kDof, kDof)),
      coriolis_payload_(Eigen::MatrixXd::Zero(kDof, kDof)),
      coriolis_model_(Eigen::MatrixXd::Zero(kDof, kDof)),
      reference_payload_mass_(reference_payload_mass),
      inverse_reference_payload_mass_(
          reference_payload_mass > 0.0 ? 1.0 / reference_payload_mass : 0.0),
      timestep_(timestep),
      torque_limit_(torque_limit) {
  if (kp_.size() != kDof || kd_.size() != kDof) {
    throw std::invalid_argument(
        "adaptive computed-torque DOB gains must have six entries");
  }
  if (!(reference_payload_mass_ > 0.0) || !(timestep_ > 0.0) ||
      !(torque_limit_ > 0.0)) {
    throw std::invalid_argument(
        "reference mass, timestep, and torque limit must be positive");
  }
  validate_reference_models();
}

void AdaptiveComputedTorqueDobController::validate_reference_models() const {
  const pinocchio::Model& empty = empty_dynamics_.model();
  const pinocchio::Model& payload = reference_payload_dynamics_.model();
  if (empty.nq != payload.nq || empty.nv != payload.nv ||
      empty.njoints != payload.njoints || empty.names != payload.names ||
      empty.parents != payload.parents) {
    throw std::invalid_argument(
        "reference URDF must have the same articulated structure as empty URDF");
  }
}

void AdaptiveComputedTorqueDobController::payload_aware_observer_terms(
    const Eigen::VectorXd& q, const Eigen::VectorXd& qdot) {
  const double scale = estimator_.mass() * inverse_reference_payload_mass_;
  empty_dynamics_.mass_matrix(q, mass_empty_);
  reference_payload_dynamics_.mass_matrix(q, mass_payload_);
  mass_model_.noalias() =
      mass_empty_ + scale * (mass_payload_ - mass_empty_);

  empty_dynamics_.coriolis_matrix(q, qdot, coriolis_empty_);
  reference_payload_dynamics_.coriolis_matrix(
      q, qdot, coriolis_payload_);
  coriolis_model_.noalias() =
      coriolis_empty_ + scale * (coriolis_payload_ - coriolis_empty_);

  empty_dynamics_.gravity(q, gravity_empty_);
  reference_payload_dynamics_.gravity(q, gravity_payload_);
  gravity_model_.noalias() =
      gravity_empty_ + scale * (gravity_payload_ - gravity_empty_);

  momentum_.noalias() = mass_model_ * qdot;
  coriolis_transpose_qdot_.noalias() =
      coriolis_model_.transpose() * qdot;
}

void AdaptiveComputedTorqueDobController::compute(
    const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& q_des, const Eigen::VectorXd& qdot_des,
    const Eigen::VectorXd& qddot_des, Eigen::VectorXd& tau_out) {
  // Use the base (pre-DOB) torque in the Phase-5 regression. The measured
  // motion already contains DOB compensation; using the final command here
  // would let the two estimators directly compete for the same residual.
  if (have_previous_sample_ && !estimator_frozen_) {
    qdd_measured_.array() =
        (qdot - previous_qdot_).array() / timestep_;
    empty_dynamics_.inverse_dynamics(
        previous_q_, previous_qdot_, qdd_measured_,
        empty_measured_torque_);
    reference_payload_dynamics_.inverse_dynamics(
        previous_q_, previous_qdot_, qdd_measured_,
        payload_measured_torque_);
    measured_regressor_.array() =
        (payload_measured_torque_ - empty_measured_torque_).array() *
        inverse_reference_payload_mass_;
    observed_extra_torque_.array() =
        previous_base_torque_.array() - empty_measured_torque_.array();
    estimator_.update(measured_regressor_, observed_extra_torque_);
  }

  payload_aware_observer_terms(q, qdot);
  if (!estimator_frozen_) {
    // Identification owns the structured residual first. Rebase the DOB while
    // RLS is live so it cannot hide payload excitation from the mass estimate.
    observer_.reset();
  }
  const Eigen::VectorXd& disturbance = observer_.update(
      momentum_, coriolis_transpose_qdot_, gravity_model_);

  qdd_command_.array() =
      qddot_des.array() + kp_.array() * (q_des - q).array() +
      kd_.array() * (qdot_des - qdot).array();
  empty_dynamics_.inverse_dynamics(
      q, qdot, qdd_command_, empty_command_torque_);
  reference_payload_dynamics_.inverse_dynamics(
      q, qdot, qdd_command_, payload_command_torque_);
  command_regressor_.array() =
      (payload_command_torque_ - empty_command_torque_).array() *
      inverse_reference_payload_mass_;
  previous_base_torque_.array() =
      empty_command_torque_.array() +
      estimator_.mass() * command_regressor_.array();
  raw_torque_.noalias() = previous_base_torque_ - disturbance;

  bool saturated = false;
  for (int i = 0; i < kDof; ++i) {
    const double raw = raw_torque_[i];
    const double applied = std::clamp(raw, -torque_limit_, torque_limit_);
    tau_out[i] = applied;
    raw_peak_torque_[i] = std::max(raw_peak_torque_[i], std::abs(raw));
    applied_peak_torque_[i] =
        std::max(applied_peak_torque_[i], std::abs(applied));
    saturated |= std::abs(raw) > torque_limit_;
  }
  observer_.set_applied_torque(tau_out);
  previous_q_ = q;
  previous_qdot_ = qdot;
  have_previous_sample_ = true;
  ++sample_count_;
  if (saturated) ++saturated_samples_;
}

void AdaptiveComputedTorqueDobController::set_estimator_frozen(bool frozen) {
  if (frozen != estimator_frozen_) {
    estimator_frozen_ = frozen;
    observer_.reset();
  }
}

void AdaptiveComputedTorqueDobController::reset() {
  estimator_.reset();
  observer_.reset();
  qdd_command_.setZero();
  qdd_measured_.setZero();
  empty_command_torque_.setZero();
  payload_command_torque_.setZero();
  command_regressor_.setZero();
  empty_measured_torque_.setZero();
  payload_measured_torque_.setZero();
  measured_regressor_.setZero();
  observed_extra_torque_.setZero();
  previous_base_torque_.setZero();
  gravity_empty_.setZero();
  gravity_payload_.setZero();
  gravity_model_.setZero();
  coriolis_transpose_qdot_.setZero();
  momentum_.setZero();
  raw_torque_.setZero();
  raw_peak_torque_.setZero();
  applied_peak_torque_.setZero();
  previous_q_.setZero();
  previous_qdot_.setZero();
  mass_empty_.setZero();
  mass_payload_.setZero();
  mass_model_.setZero();
  coriolis_empty_.setZero();
  coriolis_payload_.setZero();
  coriolis_model_.setZero();
  have_previous_sample_ = false;
  estimator_frozen_ = false;
  saturated_samples_ = 0;
  sample_count_ = 0;
}

}  // namespace arm_control
