#include "arm_control/pinocchio_dynamics.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace arm_control {

const std::array<std::string, kDof> PinocchioDynamics::kJointNames = {
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper"};

PinocchioDynamics::PinocchioDynamics(const std::string& urdf_path,
                                     double joint_armature) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  if (model_.nq != kDof || model_.nv != kDof) {
    throw std::runtime_error("Pinocchio model must have exactly " +
                             std::to_string(kDof) + " position/velocity DOF; got nq=" +
                             std::to_string(model_.nq) + " nv=" +
                             std::to_string(model_.nv));
  }

  for (int i = 0; i < kDof; ++i) {
    if (!model_.existJointName(kJointNames[i])) {
      throw std::runtime_error("Pinocchio joint not found: " + kJointNames[i]);
    }
    const pinocchio::JointIndex joint_id = model_.getJointId(kJointNames[i]);
    const auto& joint = model_.joints[joint_id];
    if (joint.nq() != 1 || joint.nv() != 1) {
      throw std::runtime_error("Expected one-DOF joint: " + kJointNames[i]);
    }
    q_index_[i] = joint.idx_q();
    v_index_[i] = joint.idx_v();
    model_.armature[v_index_[i]] = joint_armature;
  }

  data_ = std::make_unique<pinocchio::Data>(model_);
  pin_q_ = Eigen::VectorXd::Zero(model_.nq);
  pin_v_ = Eigen::VectorXd::Zero(model_.nv);
  pin_a_ = Eigen::VectorXd::Zero(model_.nv);
}

void PinocchioDynamics::map_configuration(const Eigen::VectorXd& q) {
  if (q.size() != kDof) {
    throw std::invalid_argument("q size does not match SO-101 DOF");
  }
  for (int i = 0; i < kDof; ++i) pin_q_[q_index_[i]] = q[i];
}

void PinocchioDynamics::map_velocity(const Eigen::VectorXd& qdot,
                                     Eigen::VectorXd& mapped) {
  if (qdot.size() != kDof) {
    throw std::invalid_argument("velocity/acceleration size does not match SO-101 DOF");
  }
  for (int i = 0; i < kDof; ++i) mapped[v_index_[i]] = qdot[i];
}

void PinocchioDynamics::map_torque(const Eigen::VectorXd& pin_tau,
                                   Eigen::VectorXd& tau_out) {
  if (tau_out.size() != kDof) tau_out.resize(kDof);
  for (int i = 0; i < kDof; ++i) tau_out[i] = pin_tau[v_index_[i]];
}

void PinocchioDynamics::gravity(const Eigen::VectorXd& q,
                                Eigen::VectorXd& tau_out) {
  map_configuration(q);
  map_torque(pinocchio::computeGeneralizedGravity(model_, *data_, pin_q_),
             tau_out);
}

void PinocchioDynamics::nonlinear_effects(const Eigen::VectorXd& q,
                                          const Eigen::VectorXd& qdot,
                                          Eigen::VectorXd& tau_out) {
  map_configuration(q);
  map_velocity(qdot, pin_v_);
  map_torque(pinocchio::nonLinearEffects(model_, *data_, pin_q_, pin_v_),
             tau_out);
}

void PinocchioDynamics::mass_matrix(const Eigen::VectorXd& q,
                                    Eigen::MatrixXd& mass_out) {
  map_configuration(q);
  pinocchio::crba(model_, *data_, pin_q_);
  data_->M.triangularView<Eigen::StrictlyLower>() =
      data_->M.transpose().triangularView<Eigen::StrictlyLower>();

  mass_out.resize(kDof, kDof);
  for (int row = 0; row < kDof; ++row) {
    for (int col = 0; col < kDof; ++col) {
      mass_out(row, col) = data_->M(v_index_[row], v_index_[col]);
    }
  }
}

void PinocchioDynamics::inverse_dynamics(const Eigen::VectorXd& q,
                                         const Eigen::VectorXd& qdot,
                                         const Eigen::VectorXd& qddot,
                                         Eigen::VectorXd& tau_out) {
  map_configuration(q);
  map_velocity(qdot, pin_v_);
  map_velocity(qddot, pin_a_);
  map_torque(pinocchio::rnea(model_, *data_, pin_q_, pin_v_, pin_a_),
             tau_out);
}

}  // namespace arm_control
