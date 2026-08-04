#pragma once

#include <array>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "arm_control/arm_types.hpp"

namespace arm_control {

// Rigid-body dynamics model used by computed torque. Public vectors always use
// the MuJoCo/controller order in kJointNames; Pinocchio indices are resolved by
// name once at construction.
class PinocchioDynamics {
public:
  explicit PinocchioDynamics(const std::string& urdf_path,
                             double joint_armature = 0.028);

  void gravity(const Eigen::VectorXd& q, Eigen::VectorXd& tau_out);
  void nonlinear_effects(const Eigen::VectorXd& q,
                         const Eigen::VectorXd& qdot,
                         Eigen::VectorXd& tau_out);
  void mass_matrix(const Eigen::VectorXd& q, Eigen::MatrixXd& mass_out);
  void inverse_dynamics(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& qdot,
                        const Eigen::VectorXd& qddot,
                        Eigen::VectorXd& tau_out);

  int dof() const { return kDof; }
  const pinocchio::Model& model() const { return model_; }

  static const std::array<std::string, kDof> kJointNames;

private:
  void map_configuration(const Eigen::VectorXd& q);
  void map_velocity(const Eigen::VectorXd& qdot, Eigen::VectorXd& mapped);
  void map_torque(const Eigen::VectorXd& pin_tau, Eigen::VectorXd& tau_out);

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::array<int, kDof> q_index_{};
  std::array<int, kDof> v_index_{};
  Eigen::VectorXd pin_q_;
  Eigen::VectorXd pin_v_;
  Eigen::VectorXd pin_a_;
};

}  // namespace arm_control
